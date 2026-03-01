/*
  SmartFarmIoT_NodeB_ResearchAligned_ExtremeOOP_Fixed_NoAck.ino
  ------------------------------------------------------------
  Node B (Controller) - Extreme OOP refactor (FIXED channel, NO PING/ACK/ensurePeer)
  - ESP-NOW RX Soil(4) + Lux from Node A (NO reply)
  - Soil aggregation (trimmed/median) + EMA
  - Lux source: Remote first, Local BH1750 fallback with retry + smoothing
  - Growth phase by day since transplantEpoch (NVS)
  - DLI integrate + photoperiod + latch
  - Irrigation: anti-chatter, ignore window, emergency, dryback, pulse, autotune kWet models (NVS)
  - WiFiMulti + MQTT robust reconnect (DNS check + IP fallback, throttle, logs)
  - RTC + NTP fallback (timezone), hourly resync
  - TFT buffered UI (3 pages: Operate / Health / Control)
  - HTTP long-poll attributes updates (for dashboard/API control) + MQTT control kept for future
  ------------------------------------------------------------
*/

#include <HTTPClient.h>

#include "SmartFarmTypes.h"
#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <time.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <RTClib.h>
#include <math.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>
#include <WiFiClientSecure.h>
#include <Update.h>

// --------------------- Pin Mapping ---------------------
#define RELAY_LIGHT 4
#define RELAY_VALVE_MAIN 16
#define SQW_PIN 27
#define SDA 21
#define SCL 22
#define LED_R 14
#define LED_G 13
#define LED_B 12

// --------------------- ISR -----------------------------
DRAM_ATTR volatile bool g_rtcAlarmTriggered = false;

void IRAM_ATTR onRTCAlarm() {
  g_rtcAlarmTriggered = true;
}

// --------------------- Timezone ---------------------
static const char* TZ_INFO = "<+07>-7";  // Thailand UTC+7

// --------------------- ESP-NOW fixed channel (must match WiFi channel) ---------------------
const uint8_t ESPNOW_CHANNEL = 8;
#define SOIL_COUNT 4

// IMPORTANT: allow only Node A MAC
// Replace with Node A STA MAC (the MAC that appears in Node A serial at boot)
const uint8_t ALLOWED_SENDER_MAC[6] = {0xD4, 0xE9, 0xF4, 0xC5, 0x39, 0x60}; // <-- edit to your Node A

// --------------------- Safety / Limits ---------------------
const int SOIL_CRITICAL = 20;
const int LUX_SAFE_LIMIT = 30000;
const float LUX_MIN_VALID = 0.0f;
const float LUX_MAX_VALID = 120000.0f;

// --------------------- Light Conversion ---------------------
float SUN_FACTOR   = 0.0185f;
float LIGHT_FACTOR = 0.0135f;

// --------------------- Scheduler ---------------------
const unsigned long CONTROL_INTERVAL   = 1000UL;
const unsigned long NETWORK_INTERVAL   = 2000UL;
const unsigned long TELEMETRY_INTERVAL = 1000UL;
const unsigned long DEBUG_INTERVAL     = 3000UL;
const bool DEBUG_LOG = true;

// --------------------- BH1750 retry throttle ---------------------
const unsigned long BH_RETRY_INTERVAL = 3000UL;

// --------------------- Valve Anti-chatter ---------------------
const unsigned long VALVE_MIN_ON_MS  = 15000UL;
const unsigned long VALVE_MIN_OFF_MS = 30000UL;

// --------------------- Soil ignore after valve ON ---------------------
const unsigned long SOIL_IGNORE_AFTER_VALVE_ON_MS = 15000UL;

// --------------------- Dry-back controls ---------------------
const unsigned long DRYBACK_MIN_INTERVAL_MS = 45UL * 60UL * 1000UL;

// --------------------- ESP-NOW timeouts ---------------------
const unsigned long SOIL_RX_TIMEOUT_MS = 15000UL;
const unsigned long LUX_RX_TIMEOUT_MS  = 15000UL;

// --------------------- HTTP (Device API host for attributes) ---------------------
const char* TB_HOST  = "device.techmorrow.co";  // HTTP API host (NOT mqtt host)
const char* TB_TOKEN = "vBDE9tTsuz09nMWWJZkA";  // token

// --------------------- OTA (Fixed URL, triggered by dashboard) ---------------------
// NOTE: ใช้ URL คงที่ ไม่ต้องกรอก URL บน dashboard
// แนะนำให้เป็น HTTPS direct link ไปที่ไฟล์ .bin
const char* OTA_URL = "https://natpakanchamp.github.io/SmartFarmIoT/SmartFarmIoT.ino.bin";

// ถ้าอยากง่ายสุดให้ใช้ setInsecure() (ไม่ตรวจ cert) แต่ความปลอดภัยลดลง
const bool OTA_TLS_INSECURE = true;

// --------------------- Day Preset (dashboard "Mode" selector) ---------------------
// dayPreset = 0..(N-1) แล้ว map ไปเป็น "วันหลังย้ายปลูก"
static const int DAY_PRESETS[] = {0, 7, 21, 30, 46};
static const int DAY_PRESET_COUNT = sizeof(DAY_PRESETS) / sizeof(DAY_PRESETS[0]);

// --------------------- MQTT ---------------------
unsigned long lastMqttAttemptMs = 0;
const unsigned long MQTT_RETRY_MS = 3000UL;

// EMQX TCP (PubSubClient รองรับ)

const char* mqtt_broker = "caboose.proxy.rlwy.net";
const int   mqtt_port   = 25668;
const char* mqtt_user = "vBDE9tTsuz09nMWWJZkA";   // EMQX public ใช้ anonymous ได้
const char* mqtt_pass = "";

const char* topic_telemetry = "v1/devices/me/telemetry";
const char* mqtt_topic_cmd = "group8/command";

const char* topic_status    = "group8/status";
const char* topic_dli       = "group8/dli";
const char* topic_soil      = "group8/soil";
const char* topic_valve     = "group8/valve/main";
const char* topic_lux       = "group8/lux";
const char* topic_phase     = "group8/phase";
const char* topic_day       = "group8/day";
const char* topic_soil_link = "group8/link/soilA";
const char* topic_light     = "group8/light/main";

// --------------------- WiFi ---------------------
const char* ssid_3 = "CPE-9634";
const char* pass_3 = "9876543210";

// --------------------- NVS keys ---------------------
const char* KEY_DLI         = "dli";
const char* KEY_MDONE       = "mDone";
const char* KEY_EDONE       = "eDone";
const char* KEY_DAY         = "savedDay";
const char* KEY_TRANS_EPOCH = "trEpoch";

// autotune models
const char* KEY_KWET_P1 = "kWetP1";
const char* KEY_KWET_P2 = "kWetP2";
const char* KEY_KWET_P3 = "kWetP3";

// ============================================================
// ===================== Protocol (NO ACK/PING) ================
// ============================================================
enum MsgType : uint8_t {
  MSG_SOIL = 1,
  MSG_LUX  = 2
};

typedef struct __attribute__((packed)) {
  uint8_t  msgType;
  uint32_t seq;
  int16_t  soilPct[SOIL_COUNT];
  uint16_t soilRaw[SOIL_COUNT];
  uint8_t  sensorOkMask;          // bit0..bit3
  uint32_t uptimeMs;
} SoilPacket;

typedef struct __attribute__((packed)) {
  uint8_t  msgType;
  uint32_t seq;
  float    lux;
  uint32_t uptimeMs;
  uint8_t  sensorOk;              // 1=ok, 0=fault
} LuxPacket;

// ============================================================
// ===================== Growth phases =========================
// ============================================================
struct PhaseParams {
  float dliTarget;
  uint16_t photoStartMin;
  uint16_t photoEndMin;
  uint8_t soilLow;
  uint8_t soilHigh;
  bool dryBackMode;
};

PhaseParams PHASE_TABLE[3] = {
  {14.0f, 6 * 60, 22 * 60, 70, 80, false},
  {23.0f, 6 * 60, 22 * 60, 85, 95, false},
  {21.0f, 6 * 60, 21 * 60, 50, 60, true }
};

// ============================================================
// ===================== UI helpers ============================
// ============================================================
static uint16_t colorByBool(bool ok) { return ok ? TFT_GREEN : TFT_RED; }
static uint16_t colorByLevel(int value, int low, int high) {
  if (value < low) return TFT_ORANGE;
  if (value > high) return TFT_CYAN;
  return TFT_GREEN;
}

static const char* phaseLabel(GrowPhase_t p) {
  if (p == PHASE_1_ESTABLISH) return "P1";
  if (p == PHASE_2_VEGETATIVE) return "P2";
  return "P3";
}
static String macToString(const uint8_t* mac) {
  char s[18];
  snprintf(s, sizeof(s), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(s);
}

// ============================================================
// ===================== SystemState ===========================
// ============================================================
struct SystemState {
  // time
  bool timeSynced = false;

  // phase
  GrowPhase_t phase = PHASE_1_ESTABLISH;
  PhaseParams phaseCfg = PHASE_TABLE[0];
  time_t transplantEpoch = 0;

  // sensors (control values)
  int soilPercent = 100;
  float lux = NAN;

  // health
  bool remoteSoilHealthy = false;
  bool remoteLuxHealthy  = false;
  unsigned long soilRxAgeMs = 999999;
  unsigned long luxRxAgeMs  = 999999;
  uint8_t soilValidCnt = 0;
  uint8_t soilOkMask = 0;
  uint32_t soilSeq = 0;
  uint32_t luxSeq  = 0;

  // DLI
  float dliToday = 0.0f;

  // actuators
  bool valveOn = false;
  bool lightOn = false;
  bool valveManual = false;
  bool lightManual = false;
  bool emergency = false;

  // UI
  String reason = "Booting";

  // debug (latest pulse learn)
  float lastPulseSec = 0.0f;
  float kWetEst = -1.0f;
  int irrState = 0;
  int pulseCycle = 0;
};

// ===================== RGB STATUS SERVICE =====================
class RgbStatusService {
public:
  RgbStatusService(int r, int g, int b)
  : rPin_(r), gPin_(g), bPin_(b) {}

  void begin() {
    // ใช้ 10-bit PWM (0-1023)
    ledcAttach(rPin_, 5000, 10);
    ledcAttach(gPin_, 5000, 10);
    ledcAttach(bPin_, 5000, 10);
    off();
  }

  void setColor(uint16_t r, uint16_t g, uint16_t b) {
    ledcWrite(rPin_, r);
    ledcWrite(gPin_, g);
    ledcWrite(bPin_, b);
  }

  void off() {
    setColor(0, 0, 0);
  }

  void update(bool wifiOk, bool mqttOk) {

    // 🔴 WiFi หลุด = แดงค้าง
    if (!wifiOk) {
      setColor(1023, 0, 0);
      return;
    }

    // 🔴 MQTT ยังไม่ต่อ = กระพริบแดง
    if (!mqttOk) {
      static unsigned long lastBlink = 0;
      static bool state = false;

      if (millis() - lastBlink > 300) {
        lastBlink = millis();
        state = !state;
      }

      if (state)
        setColor(1023, 0, 0);
      else
        off();

      return;
    }

    // 🟢 ทุกอย่างโอเค = เขียวค้าง
    setColor(0, 1023, 0);
  }

private:
  int rPin_;
  int gPin_;
  int bPin_;
};
// ============================================================
// ===================== Relay + ValveGuard ===================
// ============================================================
class Relay {
public:
  Relay(int pin) : pin_(pin) {}
  void begin(bool initialOn=false) {
    pinMode(pin_, OUTPUT);
    write(initialOn);
  }
  void write(bool on) {
    // active LOW relay
    digitalWrite(pin_, on ? LOW : HIGH);
    state_ = on;
  }
  bool isOn() const { return state_; }
private:
  int pin_;
  bool state_ = false;
};

class ValveGuard {
public:
  ValveGuard(unsigned long minOnMs, unsigned long minOffMs)
  : minOnMs_(minOnMs), minOffMs_(minOffMs) {}

  bool canSwitch(bool curOn, bool wantOn, unsigned long lastSwitchMs, bool force) const {
    if (wantOn == curOn) return false;
    if (force) return true;
    unsigned long now = millis();
    if (!wantOn && curOn && (now - lastSwitchMs < minOnMs_)) return false;
    if (wantOn && !curOn && (now - lastSwitchMs < minOffMs_)) return false;
    return true;
  }
private:
  unsigned long minOnMs_;
  unsigned long minOffMs_;
};

// ============================================================
// ===================== Local BH1750 Lux ======================
// ============================================================
class LocalLuxSensor {
public:
  void begin() {
    bhReady_ = meter_.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
    if (!bhReady_) Serial.println("[BH1750] init fail (local)");
  }

  float readLuxSafe() {
    unsigned long now = millis();

    if (!bhReady_) {
      if (now - lastRetryMs_ >= BH_RETRY_INTERVAL) {
        lastRetryMs_ = now;
        bhReady_ = meter_.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
        if (bhReady_) {
          Serial.println("[BH1750] Recovered (local)");
          luxSmoothed_ = 0.0f;
          lastValidLux_ = 0.0f;
          lastValidLuxMs_ = 0;
        }
      }
      return (float)LUX_SAFE_LIMIT + 1000.0f;
    }

    float raw = meter_.readLightLevel();
    bool valid = !isnan(raw) && !isinf(raw) && (raw >= LUX_MIN_VALID) && (raw <= LUX_MAX_VALID);
    if (!valid) {
      bhReady_ = false;
      Serial.println("[Alarm] Lux sensor fault/out-of-range (local)");
      if (lastValidLuxMs_ != 0 && (now - lastValidLuxMs_) < 60000UL) return lastValidLux_;
      return (float)LUX_SAFE_LIMIT + 1000.0f;
    }

    if (luxSmoothed_ <= 0.0f) luxSmoothed_ = raw;
    luxSmoothed_ = 0.20f * raw + 0.80f * luxSmoothed_;

    lastValidLux_ = luxSmoothed_;
    lastValidLuxMs_ = now;
    return luxSmoothed_;
  }

private:
  BH1750 meter_;
  bool bhReady_ = false;
  unsigned long lastRetryMs_ = 0;

  float luxSmoothed_ = 0.0f;
  float lastValidLux_ = 0.0f;
  unsigned long lastValidLuxMs_ = 0;
};

// ============================================================
// ===================== ESP-NOW RX Service (NO ACK) ===========
// ============================================================
class EspNowRxService {
public:
  EspNowRxService(const uint8_t allowedMac[6]) { memcpy(allowedMac_, allowedMac, 6); }

  bool begin() {
    instance_ = this;
    if (esp_now_init() != ESP_OK) {
      Serial.println("[ESP-NOW] init failed");
      return false;
    }
    if (esp_now_register_recv_cb(&EspNowRxService::onRecvStatic) != ESP_OK) {
      Serial.println("[ESP-NOW] register recv cb failed");
      return false;
    }
    Serial.printf("[ESP-NOW] RX ready | SoilPacket=%u LuxPacket=%u\n",
                  (unsigned)sizeof(SoilPacket), (unsigned)sizeof(LuxPacket));
    return true;
  }

  bool soilHealthy() const {
    if (!hasSoil_) return false;
    if ((millis() - lastSoilRxMs_) > SOIL_RX_TIMEOUT_MS) return false;
    int cnt = 0;
    uint8_t m = soilOkMask_;
    for (int i=0;i<SOIL_COUNT;i++) if (m & (1<<i)) cnt++;
    return (cnt >= 2);
  }

  bool luxHealthy() const {
    if (!hasLux_) return false;
    if ((millis() - lastLuxRxMs_) > LUX_RX_TIMEOUT_MS) return false;
    if (luxOk_ != 1) return false;
    float lx = lux_;
    if (isnan(lx) || isinf(lx)) return false;
    if (lx < LUX_MIN_VALID || lx > LUX_MAX_VALID) return false;
    return true;
  }

  void getSoilSnapshot(int outPct[SOIL_COUNT], uint8_t &okMask, uint32_t &seq, unsigned long &ageMs) const {
    unsigned long rx;
    noInterrupts();
    for (int i=0;i<SOIL_COUNT;i++) outPct[i] = soilPct_[i];
    okMask = soilOkMask_;
    seq = soilSeq_;
    rx = lastSoilRxMs_;
    interrupts();
    ageMs = hasSoil_ ? (millis() - rx) : 999999UL;
  }

  void getLuxSnapshot(float &lux, uint8_t &ok, uint32_t &seq, unsigned long &ageMs) const {
    unsigned long rx;
    noInterrupts();
    lux = lux_;
    ok = luxOk_;
    seq = luxSeq_;
    rx = lastLuxRxMs_;
    interrupts();
    ageMs = hasLux_ ? (millis() - rx) : 999999UL;
  }

private:
  static EspNowRxService* instance_;
  uint8_t allowedMac_[6]{};

  volatile bool hasSoil_ = false;
  volatile int soilPct_[SOIL_COUNT] = {0,0,0,0};
  volatile uint8_t soilOkMask_ = 0;
  volatile uint32_t soilSeq_ = 0;
  volatile unsigned long lastSoilRxMs_ = 0;

  volatile bool hasLux_ = false;
  volatile float lux_ = 0.0f;
  volatile uint8_t luxOk_ = 0;
  volatile uint32_t luxSeq_ = 0;
  volatile unsigned long lastLuxRxMs_ = 0;

  static bool sameMac_(const uint8_t* a, const uint8_t* b) {
    for (int i=0;i<6;i++) if (a[i]!=b[i]) return false;
    return true;
  }

  static void onRecvStatic(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (instance_) instance_->onRecv(info, data, len);
  }

  void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || !data || len < 1) return;
    if (!sameMac_(info->src_addr, allowedMac_)) {
      Serial.printf("[ESP-NOW] drop unknown mac=%s\n", macToString(info->src_addr).c_str());
      return;
    }
    uint8_t type = data[0];

    if (type == MSG_SOIL) {
      if (len != (int)sizeof(SoilPacket)) {
        Serial.printf("[RX SOIL] bad len=%d expect=%u\n", len, (unsigned)sizeof(SoilPacket));
        return;
      }
      SoilPacket pkt{}; memcpy(&pkt, data, sizeof(pkt));
      for (int i=0;i<SOIL_COUNT;i++) {
        if (pkt.soilPct[i] < 0) pkt.soilPct[i] = 0;
        if (pkt.soilPct[i] > 100) pkt.soilPct[i] = 100;
      }
      noInterrupts();
      for (int i=0;i<SOIL_COUNT;i++) soilPct_[i] = pkt.soilPct[i];
      soilOkMask_ = pkt.sensorOkMask;
      soilSeq_ = pkt.seq;
      hasSoil_ = true;
      lastSoilRxMs_ = millis();
      interrupts();

      if (DEBUG_LOG) {
        Serial.printf("[RX SOIL] seq=%lu p=[%d,%d,%d,%d] mask=0x%02X\n",
          (unsigned long)pkt.seq, pkt.soilPct[0], pkt.soilPct[1], pkt.soilPct[2], pkt.soilPct[3], pkt.sensorOkMask);
      }
      return;
    }

    if (type == MSG_LUX) {
      if (len != (int)sizeof(LuxPacket)) {
        Serial.printf("[RX LUX ] bad len=%d expect=%u\n", len, (unsigned)sizeof(LuxPacket));
        return;
      }
      LuxPacket lp{}; memcpy(&lp, data, sizeof(lp));

      bool ok = (lp.sensorOk == 1) &&
                !isnan(lp.lux) && !isinf(lp.lux) &&
                lp.lux >= LUX_MIN_VALID && lp.lux <= LUX_MAX_VALID;

      noInterrupts();
      lux_ = lp.lux;
      luxOk_ = ok ? 1 : 0;
      luxSeq_ = lp.seq;
      hasLux_ = true;
      lastLuxRxMs_ = millis();
      interrupts();

      if (DEBUG_LOG) {
        Serial.printf("[RX LUX ] seq=%lu lux=%.2f ok=%u\n",
          (unsigned long)lp.seq, lp.lux, (unsigned)(ok?1:0));
      }
      return;
    }
  }
};
EspNowRxService* EspNowRxService::instance_ = nullptr;

// ============================================================
// ===================== Soil Aggregator =======================
// ============================================================
class SoilAggregator {
public:
  SoilAggregator(float alpha) : alpha_(alpha) {}

  int aggregateAndSmooth(const int v[SOIL_COUNT], uint8_t okMask, bool &enoughValid) {
    int a[SOIL_COUNT];
    int n = 0;
    for (int i=0;i<SOIL_COUNT;i++) {
      if (okMask & (1<<i)) a[n++] = constrain(v[i], 0, 100);
    }
    if (n < 2) {
      enoughValid = false;
      return 100;
    }
    enoughValid = true;

    for (int i=1;i<n;i++){
      int key=a[i], j=i-1;
      while (j>=0 && a[j]>key){ a[j+1]=a[j]; j--; }
      a[j+1]=key;
    }

    int agg;
    if (n >= 4) agg = (int)roundf((a[1] + a[2]) / 2.0f);
    else if (n == 3) agg = a[1];
    else agg = (int)roundf((a[0] + a[1]) / 2.0f);

    if (!emaInit_) { ema_ = (float)agg; emaInit_ = true; }
    else ema_ = alpha_ * agg + (1.0f - alpha_) * ema_;

    return constrain((int)roundf(ema_), 0, 100);
  }

private:
  float alpha_;
  float ema_ = 0.0f;
  bool emaInit_ = false;
};

// ============================================================
// ===================== Phase manager =========================
// ============================================================
class PhaseManager {
public:
  int daysAfterTransplant(const struct tm& ti, time_t transplantEpoch) const {
    if (transplantEpoch <= 0) return 0;
    struct tm copy = ti;
    time_t nowEpoch = mktime(&copy);
    if (nowEpoch < transplantEpoch) return 0;
    return (int)((nowEpoch - transplantEpoch) / 86400);
  }

  GrowPhase_t phaseFromDay(int d) const {
    if (d <= 20) return PHASE_1_ESTABLISH;
    if (d <= 45) return PHASE_2_VEGETATIVE;
    return PHASE_3_FINISHING;
  }

  void apply(SystemState &st, GrowPhase_t p) const {
    st.phase = p;
    st.phaseCfg = PHASE_TABLE[(int)p];
  }
};

// ============================================================
// ===================== DLI integrator ========================
// ============================================================
class DLIIntegrator {
public:
  DLIIntegrator(Preferences &prefs) : prefs_(prefs) {}

  void restore(SystemState &st) {
    st.dliToday = prefs_.getFloat(KEY_DLI, 0.0f);
    lastSaved_ = st.dliToday;
    lastDliMs_ = 0;
    lastSaveMs_ = 0;
  }

  void integrate(SystemState &st, float lux, bool luxValid) {
    unsigned long now = millis();
    if (lastDliMs_ == 0) lastDliMs_ = now;

    float dt = (now - lastDliMs_) / 1000.0f;
    lastDliMs_ = now;

    if (dt < 0) dt = 0;
    if (dt > 5.0f) dt = 5.0f;

    if (luxValid || st.lightOn) {
      float factor = st.lightOn ? LIGHT_FACTOR : SUN_FACTOR;
      float ppfd = lux * factor;
      st.dliToday += (ppfd * dt) / 1000000.0f;
    }

    bool timeToSave = (now - lastSaveMs_ > 900000UL);
    bool threshold  = (fabsf(st.dliToday - lastSaved_) >= 0.5f);
    if (timeToSave || threshold) {
      lastSaveMs_ = now;
      lastSaved_ = st.dliToday;
      prefs_.putFloat(KEY_DLI, st.dliToday);
      Serial.println("[System] DLI saved to NVS");
    }
  }

  void resetDaily(SystemState &st) {
    st.dliToday = 0.0f;
    prefs_.putFloat(KEY_DLI, 0.0f);
    lastSaved_ = 0.0f;
  }

private:
  Preferences &prefs_;
  unsigned long lastDliMs_ = 0;
  unsigned long lastSaveMs_ = 0;
  float lastSaved_ = -1.0f;
};

// ============================================================
// ===================== Light controller ======================
// ============================================================
class LightController {
public:
  LightController(Relay &relay) : relay_(relay) {}

  void update(SystemState &st, int nowMin, float lux) {
    if (st.lightManual) {
      st.lightOn = relay_.isOn();
      return;
    }

    bool inPhoto = (nowMin >= st.phaseCfg.photoStartMin && nowMin < st.phaseCfg.photoEndMin);
    float ppfdSun = lux * SUN_FACTOR;
    bool naturalEnough = (ppfdSun >= 250.0f);

    float onThreshold  = st.phaseCfg.dliTarget - 0.25f;
    float offThreshold = st.phaseCfg.dliTarget - 0.05f;

    bool want = false;
    if (inPhoto) {
      if (!naturalEnough) {
        if (!latch_ && st.dliToday < onThreshold) latch_ = true;
        if (latch_ && st.dliToday >= offThreshold) latch_ = false;
        want = latch_;
      } else {
        latch_ = false;
        want = false;
      }
    } else {
      latch_ = false;
      want = false;
    }

    if (want != st.lightOn) {
      relay_.write(want);
      st.lightOn = want;
    }
  }

private:
  Relay &relay_;
  bool latch_ = false;
};

// ============================================================
// ===================== Irrigation + AutoTune =================
// ============================================================
struct PulseModel {
  float kWetEst;
  float emaBeta;
  float minPulseSec;
  float maxPulseSec;
};

enum IrrMicroState : uint8_t { IRR_IDLE=0, IRR_IRRIGATING=1, IRR_SOAK_WAIT=2 };

class IrrigationController {
public:
  IrrigationController(Relay &valve, ValveGuard &guard, Preferences &prefs)
  : valve_(valve), guard_(guard), prefs_(prefs) {}

  void restoreModels() {
    float k1 = prefs_.getFloat(KEY_KWET_P1, modelP1_.kWetEst);
    float k2 = prefs_.getFloat(KEY_KWET_P2, modelP2_.kWetEst);
    float k3 = prefs_.getFloat(KEY_KWET_P3, modelP3_.kWetEst);

    if (k1 >= 0.05f && k1 <= 3.0f) modelP1_.kWetEst = k1;
    if (k2 >= 0.05f && k2 <= 3.0f) modelP2_.kWetEst = k2;
    if (k3 >= 0.05f && k3 <= 3.0f) modelP3_.kWetEst = k3;

    Serial.print("[AutoTune] kWet P1="); Serial.println(modelP1_.kWetEst, 4);
    Serial.print("[AutoTune] kWet P2="); Serial.println(modelP2_.kWetEst, 4);
    Serial.print("[AutoTune] kWet P3="); Serial.println(modelP3_.kWetEst, 4);
  }

  void update(SystemState &st, int soilPct, float lux) {
    if (!st.valveManual && !st.remoteSoilHealthy) {
      st.reason = "Failsafe: remote soil timeout";
      forceOff_(st);
      resetLearn_();
      return;
    }

    if (st.valveManual) {
      st.valveOn = valve_.isOn();
      pulseActive_ = false;
      irrState_ = IRR_IDLE;
      resetLearn_();
      return;
    }

    if (soilPct < SOIL_CRITICAL) {
      st.reason = "Emergency watering: soil critical";
      st.emergency = true;
      setValve_(st, true, true);
      pulseActive_ = false;
      irrState_ = IRR_IDLE;
      publishDebug_(st);
      return;
    }

    if (st.emergency && soilPct >= st.phaseCfg.soilLow) {
      st.emergency = false;
      forceOff_(st);
    }
    if (st.emergency) {
      publishDebug_(st);
      return;
    }

    bool ignore = (st.valveOn && (millis() - lastSwitchMs_ < SOIL_IGNORE_AFTER_VALVE_ON_MS));
    if (ignore) {
      updateLearnSM_(st, soilPct);
      publishDebug_(st);
      return;
    }

    int lowAdaptive = st.phaseCfg.soilLow;
    float ppfdSun = lux * SUN_FACTOR;
    if (st.phase == PHASE_2_VEGETATIVE && ppfdSun > 500.0f) {
      lowAdaptive = min(99, lowAdaptive + 3);
    }

    if (st.phaseCfg.dryBackMode) {
      if (soilPct <= lowAdaptive) {
        if (millis() - lastDryBackWaterMs_ >= DRYBACK_MIN_INTERVAL_MS) {
          if (irrState_ == IRR_IDLE) {
            startAdaptivePulse_(st, soilPct, targetMid_(st), false);
            if (irrState_ == IRR_IRRIGATING) {
              lastDryBackWaterMs_ = millis();
              pulseActive_ = true;
            }
          }
        }
      }

      if (soilPct >= st.phaseCfg.soilHigh) {
        forceOff_(st);
        pulseActive_ = false;
        irrState_ = IRR_IDLE;
      }

      updateLearnSM_(st, soilPct);
      publishDebug_(st);
      return;
    }

    if (soilPct <= lowAdaptive && !pulseActive_ && !st.valveOn) {
      if (pulseCycleCount_ < MAX_PULSE_CYCLES_PER_CALL && irrState_ == IRR_IDLE) {
        startAdaptivePulse_(st, soilPct, targetMid_(st), false);
        if (irrState_ == IRR_IRRIGATING) {
          pulseCycleCount_++;
          pulseActive_ = true;
        }
      }
    }

    if (soilPct >= st.phaseCfg.soilHigh) {
      forceOff_(st);
      pulseActive_ = false;
      pulseCycleCount_ = 0;
      irrState_ = IRR_IDLE;
    }

    updateLearnSM_(st, soilPct);
    publishDebug_(st);
  }

private:
  Relay &valve_;
  ValveGuard &guard_;
  Preferences &prefs_;

  unsigned long lastSwitchMs_ = 0;
  bool pulseActive_ = false;

  IrrMicroState irrState_ = IRR_IDLE;
  int soilBeforePulse_ = -1;
  int soilAfterPulse_  = -1;
  unsigned long pulseStartMs_ = 0;
  unsigned long pulseStopMs_  = 0;
  unsigned long soakStartMs_  = 0;
  const unsigned long SOAK_WAIT_MS = 60000UL;

  unsigned long lastDryBackWaterMs_ = 0;

  uint8_t pulseCycleCount_ = 0;
  const uint8_t MAX_PULSE_CYCLES_PER_CALL = 3;

  float lastPulseSec_ = 0.0f;

  PulseModel modelP1_ = {0.35f, 0.25f, 6.0f, 35.0f};
  PulseModel modelP2_ = {0.45f, 0.25f, 6.0f, 35.0f};
  PulseModel modelP3_ = {0.30f, 0.25f, 5.0f, 25.0f};

  PulseModel* modelByPhase_(GrowPhase_t p) {
    if (p == PHASE_1_ESTABLISH) return &modelP1_;
    if (p == PHASE_2_VEGETATIVE) return &modelP2_;
    return &modelP3_;
  }

  int targetMid_(const SystemState &st) const {
    return (int)((st.phaseCfg.soilLow + st.phaseCfg.soilHigh) / 2);
  }

  void resetLearn_() {
    soilBeforePulse_ = -1;
    soilAfterPulse_  = -1;
    pulseCycleCount_ = 0;
  }

  void setValve_(SystemState &st, bool wantOn, bool force) {
    if (!guard_.canSwitch(st.valveOn, wantOn, lastSwitchMs_, force)) return;
    valve_.write(wantOn);
    st.valveOn = wantOn;
    lastSwitchMs_ = millis();
  }

  void forceOff_(SystemState &st) {
    setValve_(st, false, true);
    pulseActive_ = false;
    irrState_ = IRR_IDLE;
  }

  float computePulseSec_(const SystemState &st, int soilNow, int soilTarget) {
    PulseModel* pm = modelByPhase_(st.phase);
    float need = (float)(soilTarget - soilNow);
    if (need < 0.0f) need = 0.0f;

    float k = pm->kWetEst;
    if (k < 0.08f) k = 0.08f;
    if (k > 3.00f) k = 3.00f;

    float pulseSec = need / k;
    if (pulseSec < pm->minPulseSec) pulseSec = pm->minPulseSec;
    if (pulseSec > pm->maxPulseSec) pulseSec = pm->maxPulseSec;
    return pulseSec;
  }

  void learnKwet_(const SystemState &st, int soilBefore, int soilAfter, float pulseSec) {
    PulseModel* pm = modelByPhase_(st.phase);
    if (soilBefore < 0 || soilAfter < 0 || pulseSec < 1.0f) return;

    float gain = (float)(soilAfter - soilBefore);
    if (gain < 0.3f || gain > 25.0f) return;

    float kNew = gain / pulseSec;
    if (kNew < 0.05f || kNew > 3.0f) return;

    pm->kWetEst = pm->emaBeta * kNew + (1.0f - pm->emaBeta) * pm->kWetEst;

    if (st.phase == PHASE_1_ESTABLISH) prefs_.putFloat(KEY_KWET_P1, modelP1_.kWetEst);
    else if (st.phase == PHASE_2_VEGETATIVE) prefs_.putFloat(KEY_KWET_P2, modelP2_.kWetEst);
    else prefs_.putFloat(KEY_KWET_P3, modelP3_.kWetEst);

    Serial.print("[AutoTune] Learned kWet="); Serial.print(pm->kWetEst, 4);
    Serial.print(" %/s, gain="); Serial.print(gain, 2);
    Serial.print("%, pulse="); Serial.print(pulseSec, 2);
    Serial.println("s");
  }

  void startAdaptivePulse_(SystemState &st, int soilNow, int soilTarget, bool force) {
    float pulseSec = computePulseSec_(st, soilNow, soilTarget);
    lastPulseSec_ = pulseSec;

    unsigned long durMs = (unsigned long)(pulseSec * 1000.0f);

    setValve_(st, true, force);
    if (!st.valveOn) {
      st.reason = "Irrigation blocked: min-off guard";
      Serial.println("[AutoTune] Pulse blocked by anti-chatter/min-off");
      return;
    }

    st.reason = "Irrigation pulse start " + String(pulseSec, 1) + "s";
    soilBeforePulse_ = soilNow;
    pulseStartMs_ = millis();
    pulseStopMs_  = pulseStartMs_ + durMs;
    irrState_ = IRR_IRRIGATING;

    Serial.print("[AutoTune] Start pulse "); Serial.print(pulseSec, 2);
    Serial.print("s | soilBefore="); Serial.println(soilBeforePulse_);
  }

  void updateLearnSM_(SystemState &st, int currentSoil) {
    unsigned long now = millis();

    if (irrState_ == IRR_IRRIGATING) {
      if (now >= pulseStopMs_) {
        setValve_(st, false, true);
        soakStartMs_ = now;
        irrState_ = IRR_SOAK_WAIT;
        Serial.println("[AutoTune] Pulse finished -> soak wait");
      }
      return;
    }

    if (irrState_ == IRR_SOAK_WAIT) {
      if (now - soakStartMs_ >= SOAK_WAIT_MS) {
        soilAfterPulse_ = currentSoil;
        float pulseSec = (float)(pulseStopMs_ - pulseStartMs_) / 1000.0f;
        learnKwet_(st, soilBeforePulse_, soilAfterPulse_, pulseSec);

        irrState_ = IRR_IDLE;
        pulseActive_ = false;
        soilBeforePulse_ = -1;
        soilAfterPulse_  = -1;

        Serial.print("[AutoTune] Soak done | soilAfter="); Serial.println(currentSoil);
        if (currentSoil >= st.phaseCfg.soilLow) pulseCycleCount_ = 0;
      }
    }
  }

  void publishDebug_(SystemState &st) {
    PulseModel* pm = modelByPhase_(st.phase);
    st.lastPulseSec = lastPulseSec_;
    st.kWetEst = pm ? pm->kWetEst : -1.0f;
    st.irrState = (int)irrState_;
    st.pulseCycle = pulseCycleCount_;
  }
};

// ============================================================
// ===================== Time Service (RTC+NTP) =================
// ============================================================
class TimeService {
public:
  TimeService(RTC_DS3231 &rtc) : rtc_(rtc) {}

  void begin(SystemState &st) {
    setenv("TZ", TZ_INFO, 1);
    tzset();

    rtcFound_ = rtc_.begin();
    if (rtcFound_) {
      Serial.println("RTC Found");
      if (rtc_.lostPower()) Serial.println("RTC lost power, sync needed");
    } else {
      Serial.println("Error: RTC not found!");
    }

    pinMode(SQW_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SQW_PIN), onRTCAlarm, FALLING);
    setupRTCAlarm_();

    st.timeSynced = false;
  }

  bool getLocalTimeSafe(struct tm* outTm) const {
    time_t now = time(nullptr);
    if (now < 1700000000) return false;
    localtime_r(&now, outTm);
    return true;
  }

  void timezoneSync(SystemState &st) {
    configTzTime(TZ_INFO, "pool.ntp.org", "time.google.com", "time.cloudflare.com");
    Serial.println("[Time] Waiting for sync...");

    bool ok = false;
    for (int i = 0; i < 3; i++) {
      struct tm t;
      if (getLocalTime(&t, 5000)) { ok = true; break; }
      Serial.printf("[Time] NTP try %d failed\n", i + 1);
      delay(700);
    }

    if (ok) {
      Serial.println("[Time] NTP Sync Success!");
      if (rtcFound_) {
        rtc_.adjust(DateTime(time(nullptr)));
        Serial.println("[Time] RTC updated from NTP.");
      }
      st.timeSynced = true;
    } else if (rtcFound_) {
      Serial.println("[Time] NTP failed. Using RTC time.");
      time_t rtcEpoch = rtc_.now().unixtime();
      struct timeval tv{rtcEpoch, 0};
      settimeofday(&tv, nullptr);
      st.timeSynced = true;
      Serial.println("[Time] System time set from RTC.");
    } else {
      Serial.println("[Time] Critical: no time source!");
      st.timeSynced = false;
    }

    printTimeDebugBoth_();
  }

  bool consumeMinuteTick() {
    if (!g_rtcAlarmTriggered) return false;
    g_rtcAlarmTriggered = false;
    if (rtcFound_) rtc_.clearAlarm(1);
    return true;
  }

private:
  RTC_DS3231 &rtc_;
  bool rtcFound_ = false;

  void setupRTCAlarm_() {
    if (!rtcFound_) return;

    rtc_.disable32K();
    rtc_.clearAlarm(1);
    rtc_.clearAlarm(2);
    rtc_.writeSqwPinMode(DS3231_OFF);

    if (!rtc_.setAlarm1(rtc_.now(), DS3231_A1_Second)) {
      Serial.println("Error setting alarm 1!");
    }

    Wire.beginTransmission(0x68);
    Wire.write(0x0E);
    Wire.write(0b00000101);
    Wire.endTransmission();

    Serial.println("[RTC] Alarm set: trigger every minute at :00");
  }

  void printTimeDebugBoth_() {
    time_t now = time(nullptr);
    struct tm tmUtc; gmtime_r(&now, &tmUtc);
    struct tm tmLoc; localtime_r(&now, &tmLoc);

    Serial.printf("[TimeDBG][UTC ] %04d-%02d-%02d %02d:%02d:%02d\n",
      tmUtc.tm_year + 1900, tmUtc.tm_mon + 1, tmUtc.tm_mday,
      tmUtc.tm_hour, tmUtc.tm_min, tmUtc.tm_sec);

    Serial.printf("[TimeDBG][LOC ] %04d-%02d-%02d %02d:%02d:%02d\n",
      tmLoc.tm_year + 1900, tmLoc.tm_mon + 1, tmLoc.tm_mday,
      tmLoc.tm_hour, tmLoc.tm_min, tmLoc.tm_sec);
  }
};

// ============================================================
// ===================== Net+MQTT Service (robust) ==============
// ============================================================
class NetMqttService {
public:
  NetMqttService(WiFiMulti &wm, PubSubClient &mqtt)
  : wifiMulti_(wm), mqtt_(mqtt) {}

  void begin() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    wifiMulti_.addAP(ssid_3, pass_3);

    mqtt_.setKeepAlive(30);
    mqtt_.setSocketTimeout(5);
    mqtt_.setBufferSize(1024);
  }

  void lockEspNowChannelToWifi() {
    uint8_t ch = WiFi.channel();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    Serial.printf("[ESP-NOW] locked to WiFi channel=%u\n", ch);
  }

  void update(SystemState &st, TimeService &timeSvc) {
    if (millis() - lastNetworkCheckMs_ < NETWORK_INTERVAL) return;
    lastNetworkCheckMs_ = millis();

    bool wifiNow = (wifiMulti_.run() == WL_CONNECTED);

    if (wifiNow && !wasWifiConnected_) {
      Serial.println("[Network] Reconnected! Syncing time now...");
      delay(1200);
      timeSvc.timezoneSync(st);
      lockEspNowChannelToWifi();

      Serial.print("WiFi.status="); Serial.println(WiFi.status());
      Serial.print("IP="); Serial.println(WiFi.localIP());
      Serial.print("RSSI="); Serial.println(WiFi.RSSI());
      Serial.print("[WiFi] Channel="); Serial.println(WiFi.channel());
      Serial.print("[MQTT] state="); Serial.println(mqtt_.state());
    }
    wasWifiConnected_ = wifiNow;

    if (!wifiNow) {
      Serial.println("[WiFi] Lost... reconnecting");
      return;
    }

    if (!mqtt_.connected()) {
      if (millis() - lastMqttAttemptMs < MQTT_RETRY_MS) return;
      lastMqttAttemptMs = millis();

      mqtt_.setServer(mqtt_broker, mqtt_port);

      String clientId = String("Group8_NodeB_") + WiFi.macAddress();
      clientId.replace(":", "");
      Serial.printf("[MQTT] Connecting as %s ...\n", clientId.c_str());

      bool okConn = mqtt_.connect(clientId.c_str(), mqtt_user, mqtt_pass);

      if (okConn) {
        Serial.println("[MQTT] Connected!");
        mqtt_.subscribe(mqtt_topic_cmd);

        // keep MQTT attributes path for future (dashboard control)
        mqtt_.subscribe("v1/devices/me/attributes");
        mqtt_.subscribe("v1/devices/me/attributes/request/1");
        mqtt_.subscribe("v1/devices/me/attributes/response/+");

        mqtt_.publish(topic_status, "SYSTEM RECOVERED", true);

        mqtt_.publish("v1/devices/me/attributes/request/1",
  "{\"sharedKeys\":\"lightMode,lightCmd,valveMode,valveCmd,otaGo,dayPreset,resetDay\"}");
      } else {
        Serial.print("[MQTT] failed, state="); Serial.println(mqtt_.state());
        Serial.print("[NET] IP="); Serial.println(WiFi.localIP());
        Serial.print("[NET] GW="); Serial.println(WiFi.gatewayIP());
        Serial.print("[NET] DNS="); Serial.println(WiFi.dnsIP());
        Serial.print("[NET] RSSI="); Serial.println(WiFi.RSSI());
      }
    }

    if (millis() - lastTimeResyncAttemptMs_ > 3600000UL) {
      lastTimeResyncAttemptMs_ = millis();
      timeSvc.timezoneSync(st);
    }
  }

  void loopMqtt() {
    if (mqtt_.connected()) mqtt_.loop();
  }

private:
  WiFiMulti &wifiMulti_;
  PubSubClient &mqtt_;
  unsigned long lastNetworkCheckMs_ = 0;
  unsigned long lastTimeResyncAttemptMs_ = 0;
  bool wasWifiConnected_ = false;
};

// ============================================================
// ===================== Telemetry Service =====================
// ============================================================
static void jsonEscapeTo(char* out, size_t outSize, const char* in) {
  if (!out || outSize == 0) return;
  size_t j = 0;
  for (size_t i = 0; in && in[i] && j + 2 < outSize; i++) {
    char c = in[i];
    if (c == '\"' || c == '\\') { out[j++]='\\'; out[j++]=c; }
    else if (c == '\n') { out[j++]='\\'; out[j++]='n'; }
    else if (c == '\r') { out[j++]='\\'; out[j++]='r'; }
    else if (c == '\t') { out[j++]='\\'; out[j++]='t'; }
    else if ((unsigned char)c < 0x20) { /* drop control */ }
    else { out[j++] = c; }
  }
  out[j] = '\0';
}

class TelemetryService {
public:
  TelemetryService(PubSubClient &mqtt) : mqtt_(mqtt) {}

  void publishIfDue(const SystemState &st) {
    if (millis() - lastTelemetryMs_ < TELEMETRY_INTERVAL) return;
    lastTelemetryMs_ = millis();
    if (!mqtt_.connected()) return;

    char reasonEsc[96];
    String r = st.reason;
    if (r.length() > 80) r = r.substring(0, 80);
    jsonEscapeTo(reasonEsc, sizeof(reasonEsc), r.c_str());

    float luxOut = (!isnan(st.lux) && !isinf(st.lux)) ? st.lux : -1.0f;

    char payload[1024];
    int n = snprintf(payload, sizeof(payload),
      "{"
        "\"soil\":%d,"
        "\"lux\":%.2f,"
        "\"dli\":%.2f,"
        "\"phase\":%d,"

        "\"valve\":%s,"
        "\"light\":%s,"
        "\"valveManual\":%s,"
        "\"lightManual\":%s,"
        "\"emergency\":%s,"

        "\"remoteSoilHealthy\":%s,"
        "\"remoteLuxHealthy\":%s,"
        "\"soilRxAgeMs\":%lu,"
        "\"luxRxAgeMs\":%lu,"
        "\"soilValidCnt\":%u,"
        "\"soilOkMask\":%u,"
        "\"soilSeq\":%lu,"
        "\"luxSeq\":%lu,"

        "\"reason\":\"%s\""
      "}",
      st.soilPercent,
      luxOut,
      st.dliToday,
      (int)st.phase,

      st.valveOn ? "true" : "false",
      st.lightOn ? "true" : "false",
      st.valveManual ? "true" : "false",
      st.lightManual ? "true" : "false",
      st.emergency ? "true" : "false",

      st.remoteSoilHealthy ? "true" : "false",
      st.remoteLuxHealthy ? "true" : "false",
      (unsigned long)st.soilRxAgeMs,
      (unsigned long)st.luxRxAgeMs,
      (unsigned)st.soilValidCnt,
      (unsigned)st.soilOkMask,
      (unsigned long)st.soilSeq,
      (unsigned long)st.luxSeq,

      reasonEsc
    );

    if (n <= 0 || (size_t)n >= sizeof(payload)) {
      Serial.println("[MQTT] payload too large, drop");
      return;
    }

    bool ok = mqtt_.publish(topic_telemetry, payload, false);
    Serial.printf("[MQTT] pub=%s | %s\n", ok ? "OK" : "FAIL", payload);
  }

private:
  PubSubClient &mqtt_;
  unsigned long lastTelemetryMs_ = 0;
};

// ============================================================
// =========== HTTP attributes updates (control via API) =======
// ============================================================

static bool jsonGet01_global(const char* json, const char* key, int &out01) {
  char pat[48];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char* p = strstr(json, pat);
  if (!p) return false;

  p = strchr(p, ':');
  if (!p) return false;
  p++;
  while (*p==' '||*p=='\t'||*p=='\r'||*p=='\n') p++;

  if (*p == '0') { out01 = 0; return true; }
  if (*p == '1') { out01 = 1; return true; }
  return false;
}

class HttpAttrCommandService {
public:
  void begin() {}

  void poll(SystemState &st, Relay &relayLight, Relay &relayValve) {
    if (WiFi.status() != WL_CONNECTED) return;

    unsigned long now = millis();
    if (now - lastPollMs_ < POLL_MS_) return;
    lastPollMs_ = now;

    HTTPClient http;
    String url = String("http://") + TB_HOST + "/api/v1/" + TB_TOKEN + "/attributes/updates?timeout=15000";
    http.begin(url);

    int code = http.GET();
    if (code != 200) {
      http.end();
      return;
    }

    String body = http.getString();
    http.end();

    handleJson_(body.c_str(), st, relayLight, relayValve);
  }

private:
  unsigned long lastPollMs_ = 0;
  const unsigned long POLL_MS_ = 300;

  void handleJson_(const char* msg, SystemState &st, Relay &relayLight, Relay &relayValve) {
    int v;

    if (jsonGet01_global(msg, "lightMode", v)) st.lightManual = (v == 1);
    if (jsonGet01_global(msg, "valveMode", v)) st.valveManual = (v == 1);

    if (jsonGet01_global(msg, "lightCmd", v)) {
      if (st.lightManual) { relayLight.write(v == 1); st.lightOn = (v == 1); }
    }
    if (jsonGet01_global(msg, "valveCmd", v)) {
      if (st.valveManual) { relayValve.write(v == 1); st.valveOn = (v == 1); }
    }
  }
};

// ============================================================
// ===================== UI Renderer (3 pages) =================
// ============================================================
class UiRenderer {
public:
  UiRenderer(TFT_eSPI &tft, TFT_eSprite &spr) : tft_(tft), spr_(spr) {}

  void begin() {
    tft_.init();
    tft_.fillScreen(TFT_BLACK);

    spr_.setColorDepth(8);
    void* ptr = spr_.createSprite(240, 240);
    if (!ptr) Serial.println("ERROR: Not enough RAM for Sprite!");
    else Serial.println("Sprite created successfully");
  }

  void update(const SystemState &st, int h, int m) {
    if (millis() - lastUiPageMs_ >= UI_PAGE_INTERVAL_MS_) {
      lastUiPageMs_ = millis();
      page_ = (page_ + 1) % 3;
    }

    spr_.fillSprite(TFT_BLACK);
    drawHeader_(st, h, m);

    if (page_ == 0) drawOperate_(st);
    else if (page_ == 1) drawHealth_(st);
    else drawControl_(st);

    spr_.setTextDatum(BC_DATUM);
    spr_.setTextSize(1);
    spr_.setTextColor(TFT_DARKGREY, TFT_BLACK);
    spr_.drawString(String(page_ + 1) + "/3", 120, 238);

    spr_.pushSprite(0, 0);
  }

private:
  TFT_eSPI &tft_;
  TFT_eSprite &spr_;
  uint8_t page_ = 0;
  unsigned long lastUiPageMs_ = 0;
  const unsigned long UI_PAGE_INTERVAL_MS_ = 6000UL;

  void drawHeader_(const SystemState &st, int h, int m) {
    spr_.fillRoundRect(4, 4, 232, 30, 6, TFT_DARKGREY);

    char ts[6];
    sprintf(ts, "%02d:%02d", h, m);
    spr_.setTextDatum(TR_DATUM);
    spr_.setTextColor(TFT_CYAN, TFT_DARKGREY);
    spr_.setTextSize(2);
    spr_.drawString(ts, 232, 12);

    spr_.setTextDatum(TL_DATUM);
    spr_.setTextColor(TFT_WHITE, TFT_DARKGREY);
    String mode = String(st.valveManual || st.lightManual ? "MANUAL" : "AUTO");
    String ph = String("PH:") + phaseLabel(st.phase);
    spr_.drawString(mode + "  " + ph, 10, 12);

    bool warn = (!st.remoteSoilHealthy) || (!st.remoteLuxHealthy) || st.emergency || (!st.timeSynced);
    uint16_t dot = warn ? TFT_RED : TFT_GREEN;
    spr_.fillCircle(210, 19, 5, dot);
  }

  void drawOperate_(const SystemState &st) {
    spr_.fillRoundRect(8, 40, 108, 74, 10, TFT_NAVY);
    spr_.setTextDatum(TL_DATUM);
    spr_.setTextColor(TFT_WHITE, TFT_NAVY);
    spr_.setTextSize(1);
    spr_.drawString("SOIL %", 16, 48);

    uint16_t soilCol = colorByLevel(st.soilPercent, st.phaseCfg.soilLow, st.phaseCfg.soilHigh);
    spr_.setTextColor(soilCol, TFT_NAVY);
    spr_.setTextSize(4);
    spr_.setTextDatum(MC_DATUM);
    spr_.drawNumber(st.soilPercent, 62, 84);

    spr_.fillRoundRect(124, 40, 108, 74, 10, TFT_DARKGREEN);
    spr_.setTextDatum(TL_DATUM);
    spr_.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    spr_.setTextSize(1);
    spr_.drawString("LUX", 132, 48);
    spr_.setTextSize(2);
    spr_.setTextDatum(MC_DATUM);
    spr_.setTextColor(TFT_YELLOW, TFT_DARKGREEN);
    String luxText = "N/A";
    if (!isnan(st.lux) && !isinf(st.lux)) luxText = String((int)st.lux);
    spr_.drawString(luxText, 178, 84);

    spr_.fillRoundRect(8, 120, 224, 46, 10, TFT_MAROON);
    spr_.setTextDatum(TL_DATUM);
    spr_.setTextSize(1);
    spr_.setTextColor(TFT_WHITE, TFT_MAROON);
    spr_.drawString("DLI TODAY / TARGET", 16, 128);

    spr_.setTextDatum(TR_DATUM);
    spr_.setTextSize(2);
    uint16_t dliCol = (st.dliToday >= st.phaseCfg.dliTarget - 0.1f) ? TFT_GREEN : TFT_ORANGE;
    spr_.setTextColor(dliCol, TFT_MAROON);
    spr_.drawString(String(st.dliToday, 1) + " / " + String(st.phaseCfg.dliTarget, 1), 224, 152);

    spr_.fillRoundRect(8, 172, 108, 32, 8, TFT_BLACK);
    spr_.drawRoundRect(8, 172, 108, 32, 8, TFT_WHITE);
    spr_.setTextDatum(MC_DATUM);
    spr_.setTextSize(1);
    spr_.setTextColor(TFT_WHITE, TFT_BLACK);
    spr_.drawString("VALVE", 38, 188);
    spr_.setTextColor(st.valveOn ? TFT_GREEN : TFT_RED, TFT_BLACK);
    spr_.drawString(st.valveOn ? "ON" : "OFF", 88, 188);

    spr_.fillRoundRect(124, 172, 108, 32, 8, TFT_BLACK);
    spr_.drawRoundRect(124, 172, 108, 32, 8, TFT_WHITE);
    spr_.setTextColor(TFT_WHITE, TFT_BLACK);
    spr_.drawString("LIGHT", 154, 188);
    spr_.setTextColor(st.lightOn ? TFT_GREEN : TFT_RED, TFT_BLACK);
    spr_.drawString(st.lightOn ? "ON" : "OFF", 204, 188);

    spr_.fillRoundRect(8, 208, 224, 28, 8, TFT_DARKGREY);
    spr_.setTextDatum(TL_DATUM);
    spr_.setTextSize(1);
    spr_.setTextColor(TFT_WHITE, TFT_DARKGREY);
    String s = st.reason;
    if (s.length() > 34) s = s.substring(0, 34);
    spr_.drawString(s, 14, 218);
  }

  void drawHealth_(const SystemState &st) {
    spr_.setTextSize(1);
    spr_.setTextDatum(TL_DATUM);

    spr_.fillRoundRect(8, 40, 224, 196, 10, TFT_BLACK);
    spr_.drawRoundRect(8, 40, 224, 196, 10, TFT_WHITE);

    int y = 50;
    auto row = [&](const char* k, String v, uint16_t c) {
      spr_.setTextColor(TFT_WHITE, TFT_BLACK);
      spr_.drawString(k, 16, y);
      spr_.setTextDatum(TR_DATUM);
      spr_.setTextColor(c, TFT_BLACK);
      spr_.drawString(v, 224, y);
      spr_.setTextDatum(TL_DATUM);
      y += 20;
    };

    row("Soil Link", st.remoteSoilHealthy ? "OK" : "TIMEOUT", colorByBool(st.remoteSoilHealthy));
    row("Soil Rx Age", String(st.soilRxAgeMs) + " ms", (st.soilRxAgeMs <= SOIL_RX_TIMEOUT_MS) ? TFT_GREEN : TFT_RED);
    row("Soil ValidCnt", String(st.soilValidCnt) + "/4", colorByBool(st.soilValidCnt >= 2));
    row("Soil OkMask", "0x" + String(st.soilOkMask, HEX), TFT_CYAN);

    row("Lux Link", st.remoteLuxHealthy ? "OK" : "TIMEOUT", colorByBool(st.remoteLuxHealthy));
    row("Lux Rx Age", String(st.luxRxAgeMs) + " ms", (st.luxRxAgeMs <= LUX_RX_TIMEOUT_MS) ? TFT_GREEN : TFT_RED);

    row("WiFi", (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "LOST", colorByBool(WiFi.status() == WL_CONNECTED));
    row("Time Sync", st.timeSynced ? "OK" : "INVALID", colorByBool(st.timeSynced));
  }

  void drawControl_(const SystemState &st) {
    spr_.fillRoundRect(8, 40, 224, 196, 10, TFT_BLACK);
    spr_.drawRoundRect(8, 40, 224, 196, 10, TFT_WHITE);

    spr_.setTextSize(1);
    spr_.setTextDatum(TL_DATUM);

    int y = 50;
    auto row = [&](const char* k, String v, uint16_t c = TFT_CYAN) {
      spr_.setTextColor(TFT_WHITE, TFT_BLACK);
      spr_.drawString(k, 16, y);
      spr_.setTextDatum(TR_DATUM);
      spr_.setTextColor(c, TFT_BLACK);
      spr_.drawString(v, 224, y);
      spr_.setTextDatum(TL_DATUM);
      y += 20;
    };

    row("Soil Band", String(st.phaseCfg.soilLow) + "-" + String(st.phaseCfg.soilHigh), TFT_YELLOW);
    row("Photo",
        String(st.phaseCfg.photoStartMin / 60) + ":" + (st.phaseCfg.photoStartMin % 60 < 10 ? "0" : "") + String(st.phaseCfg.photoStartMin % 60)
        + " - " +
        String(st.phaseCfg.photoEndMin / 60) + ":" + (st.phaseCfg.photoEndMin % 60 < 10 ? "0" : "") + String(st.phaseCfg.photoEndMin % 60),
        TFT_YELLOW);

    row("DLI Target", String(st.phaseCfg.dliTarget, 1), TFT_YELLOW);
    row("DryBack", st.phaseCfg.dryBackMode ? "ON" : "OFF", st.phaseCfg.dryBackMode ? TFT_GREEN : TFT_ORANGE);

    row("kWetEst", String(st.kWetEst, 3), TFT_CYAN);
    row("lastPulseSec", String(st.lastPulseSec, 1), TFT_CYAN);
    row("irrState", String(st.irrState), TFT_CYAN);
    row("pulseCycle", String(st.pulseCycle) + "/3", TFT_CYAN);
  }
};

// ============================================================
// ===================== App (Orchestrator) ====================
// ============================================================
class App {
public:
  App()
  : mqtt_(espClient_),
    relayLight_(RELAY_LIGHT),
    relayValve_(RELAY_VALVE_MAIN),
    valveGuard_(VALVE_MIN_ON_MS, VALVE_MIN_OFF_MS),
    espNowRx_(ALLOWED_SENDER_MAC),
    soilAgg_(0.25f),
    dli_(prefs_),
    lightCtl_(relayLight_),
    irrCtl_(relayValve_, valveGuard_, prefs_),
    timeSvc_(rtc_),
    netSvc_(wifiMulti_, mqtt_),
    tele_(mqtt_),
    ui_(tft_, sprite_),
    rgb_(LED_R, LED_G, LED_B) {}//เพิ่ม

  void begin() {
    Serial.begin(115200);
    delay(100);

    setenv("TZ", TZ_INFO, 1);
    tzset();

    Serial.println("\n--- Node B EXTREME OOP (Fixed, NO ACK) ---");

    Wire.begin(SDA, SCL);

    ui_.begin();
    rgb_.begin();//เพิ่ม

    prefs_.begin("smartfarm", false);
    st_.dliToday = prefs_.getFloat(KEY_DLI, 0.0f);
    st_.transplantEpoch = (time_t)prefs_.getULong64(KEY_TRANS_EPOCH, 0ULL);

    timeSvc_.begin(st_);

    relayLight_.begin(false);
    relayValve_.begin(false);
    st_.lightOn = relayLight_.isOn();
    st_.valveOn = relayValve_.isOn();

    localLux_.begin();

    netSvc_.begin();
    mqtt_.setCallback(App::mqttCallbackStatic);
    instance_ = this;

    Serial.print("Connecting WiFi");
    unsigned long startAttempt = millis();
    while (millis() - startAttempt < 15000UL) {
      if (wifiMulti_.run() == WL_CONNECTED) break;
      Serial.print(".");
      delay(500);
    }
    Serial.println();

    if (wifiMulti_.run() == WL_CONNECTED) {
      Serial.printf("[WiFi] channel=%d\n", WiFi.channel());
    } else {
      Serial.println("[WiFi] connect timeout, ESP-NOW may fail if channel mismatch");
    }

    // HTTP attributes poll (keep alive)
    // httpCmd_.begin();

    netSvc_.lockEspNowChannelToWifi();

    if (!espNowRx_.begin()) Serial.println("[ESP-NOW] RX init error");

    if (WiFi.status() == WL_CONNECTED) timeSvc_.timezoneSync(st_);

    dli_.restore(st_);

    struct tm ti;
    if (timeSvc_.getLocalTimeSafe(&ti)) {
      if (st_.transplantEpoch == 0) {
        struct tm t0 = ti;
        t0.tm_hour = 0; t0.tm_min = 0; t0.tm_sec = 0;
        st_.transplantEpoch = mktime(&t0);
        prefs_.putULong64(KEY_TRANS_EPOCH, (uint64_t)st_.transplantEpoch);
        Serial.println("[Init] transplantEpoch set to today 00:00");
      }
      int day = phaseMgr_.daysAfterTransplant(ti, st_.transplantEpoch);
      phaseMgr_.apply(st_, phaseMgr_.phaseFromDay(day));
      Serial.printf("[Phase Init] Day=%d -> Phase=%d\n", day, (int)st_.phase);
    } else {
      phaseMgr_.apply(st_, PHASE_1_ESTABLISH);
    }

    irrCtl_.restoreModels();

    st_.reason = "System Ready";
    ui_.update(st_, 12, 0);
  }

  void update() {
    netSvc_.update(st_, timeSvc_);
    netSvc_.loopMqtt();

    // ---------- OTA runner (run in main loop, not in MQTT callback) ----------
    if (otaRequested_ && !otaInProgress_) {
      // กันยิงซ้ำถี่ๆ (เผื่อ dashboard ส่งซ้ำ)
      if (millis() - otaRequestMs_ > 200) {
        otaRequested_ = false;
        lastOtaGo_ = 0;

        Serial.println("[OTA] start from main loop");
        bool okOta = otaFromHttps_(OTA_URL);
        if (!okOta) {
          st_.reason = "OTA failed";
          Serial.println("[OTA] FAILED (main loop)");
        }
      }
    }
// ------------------------------------------------------------------------

    // HTTP attributes updates (dashboard/API control)
    // httpCmd_.poll(st_, relayLight_, relayValve_);

    if (timeSvc_.consumeMinuteTick()) {
      Serial.println("[SQW] Minute Tick");
    }
    if (otaInProgress_) {
      return;
    }

    if (millis() - lastCalcUpdate_ >= CONTROL_INTERVAL) {
      lastCalcUpdate_ = millis();

      struct tm timeinfo;
      int h = 12, m = 0;
      bool validTime = timeSvc_.getLocalTimeSafe(&timeinfo);

      if (validTime) {
        h = timeinfo.tm_hour;
        m = timeinfo.tm_min;
        st_.timeSynced = true;

        handleDailyReset_(timeinfo);

        int day = phaseMgr_.daysAfterTransplant(timeinfo, st_.transplantEpoch);
        GrowPhase_t newP = phaseMgr_.phaseFromDay(day);
        if (newP != st_.phase) {
          phaseMgr_.apply(st_, newP);
          prefs_.putFloat(KEY_DLI, st_.dliToday);
          Serial.printf("[Phase Change] Day=%d -> Phase=%d\n", day, (int)st_.phase);
        }

        int soil4[SOIL_COUNT]; uint8_t okMask; uint32_t sSeq; unsigned long sAge;
        espNowRx_.getSoilSnapshot(soil4, okMask, sSeq, sAge);

        float rLux; uint8_t luxOk; uint32_t lSeq; unsigned long lAge;
        espNowRx_.getLuxSnapshot(rLux, luxOk, lSeq, lAge);

        st_.soilRxAgeMs = sAge;
        st_.luxRxAgeMs  = lAge;
        st_.soilOkMask  = okMask;
        st_.soilSeq     = sSeq;
        st_.luxSeq      = lSeq;

        int cnt = 0;
        for (int i=0;i<SOIL_COUNT;i++) if (okMask & (1<<i)) cnt++;
        st_.soilValidCnt = (uint8_t)cnt;

        st_.remoteSoilHealthy = espNowRx_.soilHealthy();
        st_.remoteLuxHealthy  = espNowRx_.luxHealthy();

        bool enough = false;
        st_.soilPercent = soilAgg_.aggregateAndSmooth(soil4, okMask, enough);
        if (!enough) st_.soilPercent = 100;

        if (st_.remoteLuxHealthy) st_.lux = rLux;
        else st_.lux = localLux_.readLuxSafe();

        bool luxValid = !isnan(st_.lux) && !isinf(st_.lux) && st_.lux >= LUX_MIN_VALID && st_.lux <= LUX_MAX_VALID;

        if (!luxValid) {
          if (!st_.lightManual) {
            relayLight_.write(false);
            st_.lightOn = false;
          }
          irrCtl_.update(st_, st_.soilPercent, 0.0f);
        } else {
          int nowMin = h*60 + m;
          lightCtl_.update(st_, nowMin, st_.lux);
          dli_.integrate(st_, st_.lux, luxValid);
          irrCtl_.update(st_, st_.soilPercent, st_.lux);
        }

        if (DEBUG_LOG && (millis() - lastDebug_ >= DEBUG_INTERVAL)) {
          lastDebug_ = millis();
          printDebug_(h, m);
        }
      } else {
        st_.timeSynced = false;
        Serial.println("Time Error: waiting for sync...");
        handleNoValidTimeSafeMode_();
      }

      ui_.update(st_, h, m);
    }

    bool wifiOk = (WiFi.status() == WL_CONNECTED);//เพิ่ม
    bool mqttOk = mqtt_.connected();//เพิ่ม

    rgb_.update(wifiOk, mqttOk);//เพิ่ม

    tele_.publishIfDue(st_);
  }

private:
  static App* instance_;
  static void mqttCallbackStatic(char* topic, byte* payload, unsigned int length) {
    if (instance_) instance_->mqttCallback_(topic, payload, length);
  }

  TFT_eSPI tft_;
  TFT_eSprite sprite_{&tft_};

  RgbStatusService rgb_;
  WiFiMulti wifiMulti_;
  WiFiClient espClient_;
  PubSubClient mqtt_;
  Preferences prefs_;
  RTC_DS3231 rtc_;

  Relay relayLight_;
  Relay relayValve_;
  ValveGuard valveGuard_;

  LocalLuxSensor localLux_;
  EspNowRxService espNowRx_;
  SoilAggregator soilAgg_;
  PhaseManager phaseMgr_;
  DLIIntegrator dli_;
  LightController lightCtl_;
  IrrigationController irrCtl_;
  TimeService timeSvc_;
  NetMqttService netSvc_;
  TelemetryService tele_;
  UiRenderer ui_;

  // HTTP control service (kept alongside MQTT control)
  // HttpAttrCommandService httpCmd_;

  SystemState st_;

  unsigned long lastCalcUpdate_ = 0;
  unsigned long lastDebug_ = 0;
  int lastResetDayKey_ = -1;
  volatile bool otaInProgress_ = false;

  // --- OTA request flag (do NOT run OTA inside MQTT callback) ---
  volatile bool otaRequested_ = false;
  unsigned long otaRequestMs_ = 0;
  int lastOtaGo_ = 0; 

  static bool jsonGet01(const char* json, const char* key, int &out01) {
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return false;

    p = strchr(p, ':');
    if (!p) return false;
    p++;
    while (*p==' '||*p=='\t'||*p=='\r'||*p=='\n') p++;

    if (*p == '0') { out01 = 0; return true; }
    if (*p == '1') { out01 = 1; return true; }
    return false;
  }
  static bool jsonGetInt(const char* json, const char* key, int &out) {
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    const char* p = strstr(json, pat);
    if (!p) return false;

    p = strchr(p, ':');
    if (!p) return false;
    p++;
    while (*p==' '||*p=='\t'||*p=='\r'||*p=='\n') p++;

    bool neg = false;
    if (*p=='-') { neg=true; p++; }
    if (*p<'0' || *p>'9') return false;

    long v = 0;
    while (*p>='0' && *p<='9') { v = v*10 + (*p - '0'); p++; }
    out = neg ? -(int)v : (int)v;
    return true;
  }
  bool otaFromHttps_(const char* url) {
    if (!url || !url[0]) return false;
    if (WiFi.status() != WL_CONNECTED) return false;
    if (otaInProgress_) return false;
    otaInProgress_ = true;

    // ลดโหลดระหว่าง OTA กัน brownout
    relayLight_.write(false);
    relayValve_.write(false);
    st_.lightOn = false;
    st_.valveOn = false;
    st_.reason = "OTA updating...";
    ui_.update(st_, 12, 0); // แค่รีเฟรชครั้งหนึ่ง (ไม่ยุ่งระบบอื่น)

    WiFiClientSecure client;
    if (OTA_TLS_INSECURE) client.setInsecure();

    HTTPClient https;
    if (!https.begin(client, url)) {
      Serial.println("[OTA] https.begin fail");
      otaInProgress_ = false;
      return false;
    }

    const char* hdrKeys[] = {"Location"};
    https.collectHeaders(hdrKeys, 1);
    int code = https.GET();

// follow redirect (GitHub / Pages / Releases)
    for (int i = 0; i < 5 && (code == 301 || code == 302 || code == 307 || code == 308); i++) {
      String loc = https.header("Location");
      https.end();

      if (loc.length() == 0) {
        Serial.println("[OTA] redirect without Location");
        otaInProgress_ = false;
        return false;
      }

      Serial.printf("[OTA] redirect -> %s\n", loc.c_str());

      if (!https.begin(client, loc)) {
        Serial.println("[OTA] https.begin redirect fail");
        otaInProgress_ = false;
        return false;
      }
      https.collectHeaders(hdrKeys, 1);
      code = https.GET();
    }

    if (code != HTTP_CODE_OK) {
      Serial.printf("[OTA] GET fail code=%d\n", code);
      https.end();
      otaInProgress_ = false;
      return false;
    }
    Serial.printf("[OTA] HTTP=%d size=%d\n", code, https.getSize());
    Serial.printf("[OTA] Content-Type=%s\n", https.header("Content-Type").c_str());
    Serial.printf("[OTA] Content-Encoding=%s\n", https.header("Content-Encoding").c_str());

    int len = https.getSize();
    WiFiClient* stream = https.getStreamPtr();

    if (!Update.begin(len > 0 ? (size_t)len : UPDATE_SIZE_UNKNOWN)) {
      Serial.println("[OTA] Update.begin fail");
      https.end();
      otaInProgress_ = false;
      return false;
    }

    size_t written = Update.writeStream(*stream);
    bool okEnd = Update.end();

    https.end();

    if (!okEnd) {
      Serial.printf("[OTA] Update.end fail err=%d\n", (int)Update.getError());
      otaInProgress_ = false;
      return false;
    }
    if (!Update.isFinished()) {
      Serial.println("[OTA] not finished");
      otaInProgress_ = false;
      return false;
    }
    if (written == 0) {
      Serial.println("[OTA] written=0");
      otaInProgress_ = false;
      return false;
    }

    Serial.println("[OTA] SUCCESS -> reboot");
    delay(300);
    ESP.restart();
    otaInProgress_ = false;
    return true;
  }

  void mqttCallback_(char* topic, byte* payload, unsigned int length) {
    static char msg[256];
    unsigned int n = (length < sizeof(msg) - 1) ? length : (sizeof(msg) - 1);
    memcpy(msg, payload, n);
    msg[n] = '\0';

    while (n > 0 && (msg[n - 1] == ' ' || msg[n - 1] == '\n' || msg[n - 1] == '\r' || msg[n - 1] == '\t')) {
      msg[n - 1] = '\0';
      n--;
    }

    Serial.printf("Message [%s] %s\n", topic, msg);

    if (strcmp(topic, "v1/devices/me/attributes") == 0 ||
        strncmp(topic, "v1/devices/me/attributes/response/", 31) == 0) {

      int v;

      if (jsonGet01(msg, "lightMode", v)) {
        st_.lightManual = (v == 1);
        Serial.printf("[ATTR] lightMode=%d -> %s\n", v, st_.lightManual ? "MANUAL" : "AUTO");
      }

      if (jsonGet01(msg, "lightCmd", v)) {
        Serial.printf("[ATTR] lightCmd=%d\n", v);
        if (st_.lightManual) {
          relayLight_.write(v == 1);
          st_.lightOn = (v == 1);
          Serial.printf(" -> LIGHT %s\n", v ? "ON" : "OFF");
        } else {
          Serial.println(" -> ignore lightCmd because LIGHT is AUTO");
        }
      }

      if (jsonGet01(msg, "valveMode", v)) {
        st_.valveManual = (v == 1);
        Serial.printf("[ATTR] valveMode=%d -> %s\n", v, st_.valveManual ? "MANUAL" : "AUTO");
      }

      if (jsonGet01(msg, "valveCmd", v)) {
        Serial.printf("[ATTR] valveCmd=%d\n", v);
        if (st_.valveManual) {
          relayValve_.write(v == 1);
          st_.valveOn = (v == 1);
          Serial.printf(" -> VALVE %s\n", v ? "ON" : "OFF");
        } else {
          Serial.println(" -> ignore valveCmd because VALVE is AUTO");
        }
      }
            // ===================== OTA trigger (Fixed URL) =====================
      // Dashboard ส่ง otaGo=1 เพื่อเริ่มอัปเดต
      if (jsonGet01(msg, "otaGo", v)) {
        if (v == 1 && lastOtaGo_ == 0) {
          Serial.println("[OTA] otaGo rising edge -> schedule OTA");
          otaRequested_ = true;
          otaRequestMs_ = millis();
          st_.reason = "OTA scheduled";
        }
        lastOtaGo_ = v;
      }

      // ===================== Day preset selector =====================
      // Dashboard ส่ง dayPreset เป็น index 0..N-1 (โหมดเลือกหลายค่า)
      int presetIdx;
      if (jsonGetInt(msg, "dayPreset", presetIdx)) {
        if (presetIdx < 0) presetIdx = 0;
        if (presetIdx >= DAY_PRESET_COUNT) presetIdx = DAY_PRESET_COUNT - 1;

        int daySet = DAY_PRESETS[presetIdx];
        Serial.printf("[DAY] dayPreset=%d -> daySet=%d\n", presetIdx, daySet);

        struct tm ti;
        if (timeSvc_.getLocalTimeSafe(&ti)) {
          struct tm t0 = ti;
          t0.tm_hour = 0; t0.tm_min = 0; t0.tm_sec = 0;
          time_t today00 = mktime(&t0);

          st_.transplantEpoch = today00 - (time_t)daySet * 86400;
          prefs_.putULong64(KEY_TRANS_EPOCH, (uint64_t)st_.transplantEpoch);

          GrowPhase_t newP = phaseMgr_.phaseFromDay(daySet);
          phaseMgr_.apply(st_, newP);

          st_.reason = String("Day preset -> ") + daySet;
        } else {
          Serial.println("[DAY] ignored: invalid time");
        }
      }

      // ===================== Manual reset day (DLI reset) =====================
      // Dashboard ส่ง resetDay=1 เพื่อ reset DLI + flags ทันที
      if (jsonGet01(msg, "resetDay", v) && v == 1) {
        dli_.resetDaily(st_);
        prefs_.putBool(KEY_MDONE, false);
        prefs_.putBool(KEY_EDONE, false);

        // กัน reset ซ้ำทันที: เซฟ KEY_DAY เป็นวันนี้
        struct tm ti;
        if (timeSvc_.getLocalTimeSafe(&ti)) {
          lastResetDayKey_ = ti.tm_yday;
          prefs_.putInt(KEY_DAY, lastResetDayKey_);
        }

        st_.reason = "Manual resetDay";
        Serial.println("[DAY] resetDay=1 -> DLI cleared");
      }
      return;
    }

    // ===================== MQTT TILE CONTROL (TYPE A: TEXT COMMANDS) =====================
    // Topic: group8/command
    // Payload: VALVE_MANUAL / VALVE_AUTO / VALVE_ON / VALVE_OFF / LIGHT_MANUAL / LIGHT_AUTO / LIGHT_ON / LIGHT_OFF
    if (strcmp(topic, mqtt_topic_cmd) == 0) {
      if (strcmp(msg, "VALVE_MANUAL") == 0) st_.valveManual = true;
      else if (strcmp(msg, "VALVE_AUTO") == 0) st_.valveManual = false;
      else if (strcmp(msg, "VALVE_ON") == 0) {
        if (st_.valveManual) { relayValve_.write(true); st_.valveOn = true; }
        else Serial.println("System is in AUTO Mode!!");
      }
      else if (strcmp(msg, "VALVE_OFF") == 0) {
        if (st_.valveManual) { relayValve_.write(false); st_.valveOn = false; }
        else Serial.println("System is in AUTO Mode!!");
      }
      else if (strcmp(msg, "LIGHT_MANUAL") == 0) st_.lightManual = true;
      else if (strcmp(msg, "LIGHT_AUTO") == 0) st_.lightManual = false;
      else if (strcmp(msg, "LIGHT_ON") == 0) {
        if (st_.lightManual) { relayLight_.write(true); st_.lightOn = true; }
        else Serial.println("System is in AUTO Mode!!");
      }
      else if (strcmp(msg, "LIGHT_OFF") == 0) {
        if (st_.lightManual) { relayLight_.write(false); st_.lightOn = false; }
        else Serial.println("System is in AUTO Mode!!");
      }
      else {
        Serial.println("Unknown command.");
      }
    }
  }

  void handleDailyReset_(const struct tm& timeinfo) {
    int dayKey = timeinfo.tm_yday;
    if (lastResetDayKey_ == -1) lastResetDayKey_ = prefs_.getInt(KEY_DAY, -1);

    if (dayKey != lastResetDayKey_) {
      lastResetDayKey_ = dayKey;
      prefs_.putInt(KEY_DAY, dayKey);

      dli_.resetDaily(st_);
      prefs_.putBool(KEY_MDONE, false);
      prefs_.putBool(KEY_EDONE, false);

      Serial.println("[Daily Reset] Cleared DLI + day flags");
    }
  }

  void handleNoValidTimeSafeMode_() {
    if (!st_.valveManual) { relayValve_.write(false); st_.valveOn = false; }
    if (!st_.lightManual) { relayLight_.write(false); st_.lightOn = false; }
    Serial.println("[Time] Invalid system time -> automation paused");
  }

  void printDebug_(int h, int m) {
    Serial.println("----- STATUS (Research Aligned | EXT OOP) -----");
    Serial.printf("Time: %02d:%02d\n", h, m);
    Serial.print("Phase: "); Serial.println((int)st_.phase);
    Serial.print("Target DLI: "); Serial.println(st_.phaseCfg.dliTarget);
    Serial.print("Photo(min): "); Serial.print(st_.phaseCfg.photoStartMin);
    Serial.print(" -> "); Serial.println(st_.phaseCfg.photoEndMin);
    Serial.print("Soil band: "); Serial.print(st_.phaseCfg.soilLow);
    Serial.print(" - "); Serial.println(st_.phaseCfg.soilHigh);

    Serial.print("Lux: "); Serial.println(st_.lux);
    Serial.print("DLI: "); Serial.println(st_.dliToday);
    Serial.print("Soil%: "); Serial.println(st_.soilPercent);

    Serial.print("Light: "); Serial.println(st_.lightOn ? "ON" : "OFF");
    Serial.print("Valve: "); Serial.println(st_.valveOn ? "ON" : "OFF");
    Serial.print("Emergency: "); Serial.println(st_.emergency ? "YES" : "NO");

    Serial.print("RemoteSoilHealthy: "); Serial.println(st_.remoteSoilHealthy ? "YES" : "NO");
    Serial.print("SoilRxAgeMs: "); Serial.println(st_.soilRxAgeMs);
    Serial.print("SoilValidCnt: "); Serial.print(st_.soilValidCnt); Serial.println("/4");
    Serial.printf("SoilOkMask: 0x%02X | SoilSeq=%lu\n", st_.soilOkMask, (unsigned long)st_.soilSeq);

    Serial.print("RemoteLuxHealthy: "); Serial.println(st_.remoteLuxHealthy ? "YES" : "NO");
    Serial.print("LuxRxAgeMs: "); Serial.println(st_.luxRxAgeMs);
    Serial.printf("LuxSeq=%lu\n", (unsigned long)st_.luxSeq);

    Serial.print("AutoTune: ON | irrState: "); Serial.println(st_.irrState);
    Serial.print("kWetEst: "); Serial.println(st_.kWetEst, 4);
    Serial.print("lastPulseSec: "); Serial.println(st_.lastPulseSec, 2);
    Serial.print("pulseCycle: "); Serial.println(st_.pulseCycle);
  }
};
App* App::instance_ = nullptr;

// ============================================================
// ===================== Arduino entrypoints ===================
// ============================================================
App app;

void setup() {
  app.begin();
}

void loop() {
  app.update();
}