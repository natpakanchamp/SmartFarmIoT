/*
  SmartFarmIoT_ResearchAligned.ino
  ------------------------------------------------------------
  Research-aligned control for kale-like leafy vegetables:
  - Phase-based growth strategy (Day 0-20 / 21-45 / 46+)
  - DLI + photoperiod light control
  - Phase-dependent irrigation with dry-back in finishing phase
  - Robust soil reading (median-of-5 + EMA + post-valve ignore window)
  - MQTT manual/auto controls
  - RTC + NTP fallback
  - TFT buffered display
  ------------------------------------------------------------
*/

#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <time.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Preferences.h>
#include <RTClib.h>
#include <math.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

// --------------------- Pin Mapping ---------------------
#define RELAY_LIGHT 4
#define RELAY_VALVE_MAIN 16
#define SQW_PIN 27

#define SDA 21
#define SCL 22

// --------------------- Safety / Limits ---------------------
const int SOIL_CRITICAL = 20;          // ต่ำมาก -> emergency watering
const int LUX_SAFE_LIMIT = 30000;      // fallback lux threshold

// --------------------- Light Conversion ---------------------
// ปรับตามโคมจริง/การติดตั้งจริงภายหลัง
float SUN_FACTOR = 0.0185f;    // lux -> ppfd (sunlight estimate)
float LIGHT_FACTOR = 0.0135f;  // lux -> ppfd (grow light estimate)

// --------------------- Time Schedulers ---------------------
const unsigned long CONTROL_INTERVAL = 1000UL;     // 1s
const unsigned long NETWORK_INTERVAL = 2000UL;     // 2s
const unsigned long TELEMETRY_INTERVAL = 10000UL;  // 10s
const unsigned long DEBUG_INTERVAL = 3000UL;       // 3s
const bool DEBUG_LOG = true;

// --------------------- BH1750 retry throttle ---------------------
unsigned long lastBhRetryMs = 0;
const unsigned long BH_RETRY_INTERVAL = 3000UL;

// --------------------- Valve Anti-chatter ---------------------
unsigned long valveLastSwitchMs = 0;
const unsigned long VALVE_MIN_ON_MS = 15000UL;
const unsigned long VALVE_MIN_OFF_MS = 30000UL;

// --------------------- UI Status line ---------------------
String uiReason = "System booting";

// --------------------- Lux filtering ---------------------
float luxSmoothed = 0.0f;
const float luxAlpha = 0.20f;
bool isLuxValid = false;
float lastValidLux = 0.0f;
unsigned long lastValidLuxMs = 0;
const float LUX_MIN_VALID = 0.0f;
const float LUX_MAX_VALID = 120000.0f;
bool bhReady = false;
float lastLuxForTelemetry = 0.0f;
bool hasLastLuxForTelemetry = false;

// --------------------- Soil robust filtering ---------------------
const unsigned long SOIL_IGNORE_AFTER_VALVE_ON_MS = 15000UL;  // ignore decision after valve ON

// --------------------- Dry-back controls ---------------------
unsigned long lastDryBackWaterMs = 0;
const unsigned long DRYBACK_MIN_INTERVAL_MS = 45UL * 60UL * 1000UL;  // 45 min

// --------------------- Pulse irrigation ---------------------
unsigned long valvePulseStartMs = 0;
unsigned long valvePulseDurationMs = 0;
bool pulseActive = false;

// --------------------- Core states ---------------------
float target_DLI = 12.0f;  // overridden by phase config
float current_DLI = 0.0f;
float lastSavedDLI = -1.0f;
bool isLightOn = false;
int soilPercent = 0;

struct RemoteSoilSnapshot {
  bool has;
  uint32_t seq;
  unsigned long rxMs;
};

bool isValveMainOn = false;
bool isValveManual = false;
bool isLightManual = false;
bool isEmergencyMode = false;

bool rtcFound = false;
volatile bool alarmTriggered = false;
bool isTimeSynced = false;
bool wasWifiConnected = false;
bool ledDecisionLatch = false;
int getSoilTargetMidForPhase();
void startAdaptivePulseByNeed(int soilNow, int soilTarget, bool force = false);
void updateAdaptivePulseStateMachine(int currentSoil);

static const char* TZ_INFO = "<+07>-7";   // UTC+7

// --------------------- ESP-NOW (Soil RX from Node A) ---------------------
enum MsgType : uint8_t {
  MSG_SOIL = 1,
  MSG_LUX  = 2,
};

// ================= ESP-NOW fixed channel config =================
const uint8_t ESPNOW_CHANNEL = 9;   // << ตั้งให้ตรง Node A และ WiFi AP
// ===============================================================

#define SOIL_COUNT 4

typedef struct __attribute__((packed)) {
  uint8_t  msgType;
  uint32_t seq;
  int16_t  soilPct[SOIL_COUNT];   // 4 จุด
  uint16_t soilRaw[SOIL_COUNT];   // 4 จุด (debug)
  uint8_t  sensorOkMask;          // bit0..bit3
  uint32_t uptimeMs;
} SoilPacket;

typedef struct __attribute__((packed)) {
  uint8_t  msgType;
  uint32_t seq;
  float    lux;
  uint32_t uptimeMs;
  uint8_t  sensorOk;    // 1=ok, 0=fault
} LuxPacket; // รับมาจาก Node A

typedef struct __attribute__((packed)) {
  uint8_t  msgType;   // MSG_PING
  uint32_t seq;
  uint32_t uptimeMs;
} PingPacket;

typedef struct __attribute__((packed)) {
  uint8_t  msgType;   // MSG_ACK
  uint32_t seq;       // echo seq จาก ping
  uint32_t uptimeMs;
} AckPacket;

volatile bool hasRemoteSoil = false;
volatile int remoteSoilPct[SOIL_COUNT] = {0,0,0,0};
volatile uint8_t remoteSoilOkMask = 0;
volatile uint32_t remoteSeq = 0;
volatile unsigned long lastSoilRxMs = 0;

// ค่า aggregate ที่ใช้ control (นิ่งขึ้น)
float soilAggEma = 0.0f;
bool soilAggEmaInit = false;
const float SOIL_AGG_ALPHA = 0.25f;

const unsigned long SOIL_RX_TIMEOUT_MS = 15000UL;   // ถ้าเกินนี้ถือว่าลิงก์หาย

volatile bool hasRemoteLux = false;
volatile float remoteLux = 0.0f;
volatile uint32_t remoteLuxSeq = 0;
volatile uint8_t remoteLuxOk = 0;
volatile unsigned long lastLuxRxMs = 0;

const unsigned long LUX_RX_TIMEOUT_MS = 15000UL;

// 1C:C3:AB:B4:77:CC
//const uint8_t ALLOWED_SENDER_MAC[6] = {0x1C, 0xC3, 0xAB, 0xB4, 0x77, 0xCC}; // แก้เป็น MAC ของ Node A จริง
//bool isSameMac(const uint8_t* a, const uint8_t* b);

// --------------------- Runtime timers ---------------------
unsigned long lastNetworkCheck = 0;
unsigned long lastCalcUpdate = 0;
unsigned long lastDliMillis = 0;
unsigned long lastDliSave = 0;
unsigned long lastTimeResyncAttempt = 0;
unsigned long lastTelemetry = 0;
unsigned long lastDebug = 0;
int lastResetDayKey = -1;

// --------------------- Legacy day flags (kept for compatibility) ---------------------
bool isMorningDone = false;
bool isEveningDone = false;

// --------------------- Objects ---------------------
BH1750 lightMeter;
WiFiMulti wifiMulti;
WiFiClient espClient;
PubSubClient client(espClient);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);
Preferences preferences;
RTC_DS3231 rtc;

// ------------------------ Auto-tuning Pulse by Soil Slope ------------

// เปิด/ปิดฟีเจอร์ auto-tuning
bool AUTO_TUNE_PULSE_ENABLED = true;

// สถานะไมโครสเตทของ pulse learning
enum IrrMicroState : uint8_t { IRR_IDLE = 0, IRR_IRRIGATING = 1, IRR_SOAK_WAIT = 2 };
IrrMicroState irrState = IRR_IDLE;

// โมเดลเรียนรู้ต่อ phase (kWet = % moisture gained per second of valve ON)
struct PulseModel {
  float kWetEst;       // %/sec
  float emaBeta;       // 0..1
  float minPulseSec;   // hard clamp
  float maxPulseSec;   // hard clamp
};

// 3 phase ตามที่มีอยู่แล้ว
PulseModel pulseModelP1 = {0.35f, 0.25f, 6.0f, 35.0f};
PulseModel pulseModelP2 = {0.45f, 0.25f, 6.0f, 35.0f};
PulseModel pulseModelP3 = {0.30f, 0.25f, 5.0f, 25.0f};

// runtime tracking สำหรับการเรียนรู้
int soilBeforePulse = -1;
int soilAfterPulse = -1;
unsigned long pulseStartMs = 0;
unsigned long pulseStopMs = 0;
unsigned long soakWaitStartMs = 0;
const unsigned long SOAK_WAIT_MS = 60000UL;  // 60s หลังปิดวาล์วก่อนประเมิน gain

// จำกัดรอบ pulse ต่อ "ครั้งที่เรียกเติมน้ำ"
uint8_t pulseCycleCount = 0;
const uint8_t MAX_PULSE_CYCLES_PER_CALL = 3;

// debug helper
float lastComputedPulseSec = 0.0f;
float lastLearnedKwet = -1.0f;

// NVS keys สำหรับโมเดลเรียนรู้
const char* KEY_KWET_P1 = "kWetP1";
const char* KEY_KWET_P2 = "kWetP2";
const char* KEY_KWET_P3 = "kWetP3";

// --------------------- WiFi ---------------------
// const char* ssid_3 = "JebHuaJai";
// const char* pass_3 = "ffffffff";

const char* ssid_3 = "CPE-9634";
const char* pass_3 = "9876543210";

// --------------------- MQTT ---------------------
const char* mqtt_broker = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_client_id = "Group8/lnwza555";
const char* mqtt_topic_cmd = "group8/command";

const char* topic_status = "group8/status";
const char* topic_dli = "group8/dli";
const char* topic_soil = "group8/soil";
const char* topic_valve = "group8/valve/main";
const char* topic_lux = "group8/lux";
const char* topic_phase = "group8/phase";
const char* topic_day = "group8/day";
const char* topic_soil_link = "group8/link/soilA";
const char* topic_light = "group8/light/main";

// --------------------- NVS keys ---------------------
const char* KEY_DLI = "dli";
const char* KEY_MDONE = "mDone";
const char* KEY_EDONE = "eDone";
const char* KEY_DAY = "savedDay";
const char* KEY_TRANS_EPOCH = "trEpoch";

// --------------------- Research-based Growth Phases ---------------------
enum GrowPhase : uint8_t {
  PHASE_1_ESTABLISH = 0,   // Day 0-20
  PHASE_2_VEGETATIVE = 1,  // Day 21-45
  PHASE_3_FINISHING = 2    // Day 46+
};

struct PhaseParams {
  float dliTarget;
  uint16_t photoStartMin;
  uint16_t photoEndMin;
  uint8_t soilLow;
  uint8_t soilHigh;
  bool dryBackMode;
};

// ค่าตั้งต้นอิงงานวิจัยที่คุยกัน
PhaseParams PHASE_TABLE[3] = {
  // dli, start, end, low, high, dryBack
  {14.0f, 6 * 60, 22 * 60, 70, 80, false}, // Day 0-20
  {23.0f, 6 * 60, 22 * 60, 85, 95, false}, // Day 21-45
  {21.0f, 6 * 60, 21 * 60, 50, 60, true }  // Day 46+
};

GrowPhase currentPhase = PHASE_1_ESTABLISH;
PhaseParams phaseCfg = PHASE_TABLE[0];

uint16_t colorByBool(bool ok) {
  return ok ? TFT_GREEN : TFT_RED;
}

uint16_t colorByLevel(int value, int low, int high) {
  if (value < low) return TFT_ORANGE;  // ต่ำกว่าเป้า
  if (value > high) return TFT_CYAN;   // สูงกว่าเป้า
  return TFT_GREEN;                    // อยู่ในช่วงเป้า
}

const char* phaseText(GrowPhase p) {
  if (p == PHASE_1_ESTABLISH) return "P1";
  if (p == PHASE_2_VEGETATIVE) return "P2";
  return "P3";
}

// วันเริ่มปลูก (local epoch)
time_t transplantEpoch = 0;

void lockEspNowChannelToFixed() {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.printf("[ESP-NOW] forced channel=%u\n", ESPNOW_CHANNEL);
}

// --------------------- Interrupt ---------------------
void IRAM_ATTR onRTCAlarm() {
  alarmTriggered = true;
}

// --------------------- Relay helpers ---------------------
void setRelayState(int pin, bool active) {
  // Active LOW relay
  digitalWrite(pin, active ? LOW : HIGH);
}

void setValveStateSafe(bool wantOn, bool force = false) {
  unsigned long now = millis();
  bool curOn = isValveMainOn;

  if (wantOn == curOn) return;

  if (!force) {
    if (!wantOn && curOn && (now - valveLastSwitchMs < VALVE_MIN_ON_MS)) return;
    if (wantOn && !curOn && (now - valveLastSwitchMs < VALVE_MIN_OFF_MS)) return;
  }

  setRelayState(RELAY_VALVE_MAIN, wantOn);
  isValveMainOn = wantOn;
  valveLastSwitchMs = now;
}

void forceValveOffAndResetPulse() {
  setValveStateSafe(false, true);   // บังคับปิด
  pulseActive = false;
  irrState = IRR_IDLE;
}

// --------------------- UI Paging ---------------------
uint8_t uiPage = 0; // 0=OPERATE,1=HEALTH,2=CONTROL
unsigned long lastUiPageMs = 0;
const unsigned long UI_PAGE_INTERVAL_MS = 6000UL;

// --------------------- Display ---------------------
void updateDisplay_Buffered(int h, int m) {
  // auto paging
  if (millis() - lastUiPageMs >= UI_PAGE_INTERVAL_MS) {
    lastUiPageMs = millis();
    uiPage = (uiPage + 1) % 3;
  }

  sprite.fillSprite(TFT_BLACK);

  drawHeader(h, m);

  if (uiPage == 0) {
    drawPageOperate();
  } else if (uiPage == 1) {
    drawPageHealth();
  } else {
    drawPageControl();
  }

  // footer page index
  sprite.setTextDatum(BC_DATUM);
  sprite.setTextSize(1);
  sprite.setTextColor(TFT_DARKGREY, TFT_BLACK);
  sprite.drawString(String(uiPage + 1) + "/3", 120, 238);

  sprite.pushSprite(0, 0);
}

// --------------------- Command helpers ---------------------
void setManualMode(bool &modeRef, const char* name, bool manual) {
  modeRef = manual;
  Serial.print("SET MODE: ");
  Serial.print(name);
  Serial.println(manual ? " MANUAL" : " AUTO");
}

void applyManualAction(bool manualMode, int relayPin, bool &stateRef, const char* name, bool turnOn) {
  if (manualMode) {
    setRelayState(relayPin, turnOn);
    stateRef = turnOn;
    Serial.print("MANUAL: ");
    Serial.print(name);
    Serial.println(turnOn ? " ON" : " OFF");
  } else {
    Serial.println("System is in AUTO Mode!!");
  }
}

// --------------------- MQTT callback ---------------------
void callback(char* topic, byte* payload, unsigned int length) {
  static char msg[64];
  unsigned int n = (length < sizeof(msg) - 1) ? length : (sizeof(msg) - 1);
  memcpy(msg, payload, n);
  msg[n] = '\0';

  // trim
  while (n > 0 && (msg[n - 1] == ' ' || msg[n - 1] == '\n' || msg[n - 1] == '\r' || msg[n - 1] == '\t')) {
    msg[n - 1] = '\0';
    n--;
  }

  Serial.print("Message [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(msg);

  if (strcmp(topic, mqtt_topic_cmd) == 0) {
    if (strcmp(msg, "VALVE_MANUAL") == 0) setManualMode(isValveManual, "VALVE", true);
    else if (strcmp(msg, "VALVE_AUTO") == 0) setManualMode(isValveManual, "VALVE", false);
    else if (strcmp(msg, "VALVE_ON") == 0) applyManualAction(isValveManual, RELAY_VALVE_MAIN, isValveMainOn, "VALVE", true);
    else if (strcmp(msg, "VALVE_OFF") == 0) applyManualAction(isValveManual, RELAY_VALVE_MAIN, isValveMainOn, "VALVE", false);

    else if (strcmp(msg, "LIGHT_MANUAL") == 0) setManualMode(isLightManual, "LIGHT", true);
    else if (strcmp(msg, "LIGHT_AUTO") == 0) setManualMode(isLightManual, "LIGHT", false);
    else if (strcmp(msg, "LIGHT_ON") == 0) applyManualAction(isLightManual, RELAY_LIGHT, isLightOn, "LIGHT", true);
    else if (strcmp(msg, "LIGHT_OFF") == 0) applyManualAction(isLightManual, RELAY_LIGHT, isLightOn, "LIGHT", false);

    else Serial.println("Unknown command.");
  }

  struct tm ti;
  int h = 12, m = 0;
  if (getLocalTimeSafe(&ti)) {
    h = ti.tm_hour;
    m = ti.tm_min;
  }
  updateDisplay_Buffered(h, m);
  Serial.println("[Instant Update Triggered!]");
}

// --------------------- RTC alarm ---------------------
void setupRTCAlarm() {
  if (!rtcFound) return;

  rtc.disable32K();
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.writeSqwPinMode(DS3231_OFF);

  if (!rtc.setAlarm1(rtc.now(), DS3231_A1_Second)) {
    Serial.println("Error setting alarm 1!");
  }

  // Enable INTCN + A1IE directly
  Wire.beginTransmission(0x68);
  Wire.write(0x0E);
  Wire.write(0b00000101);
  Wire.endTransmission();

  Serial.println("[RTC] Alarm set: trigger every minute at :00");
}

// --------------------- Time sync ---------------------
bool syncNtpOnce(uint32_t timeoutMs = 5000) {
  struct tm t;
  return getLocalTime(&t, timeoutMs);
}

bool getLocalTimeSafe(struct tm* outTm) {
  time_t now = time(nullptr);
  if (now < 1700000000) return false;
  localtime_r(&now, outTm);   // แปลงตาม TZ ที่ set ไว้
  return true;
}

void timezoneSync() {
  const char* ntp1 = "pool.ntp.org";
  const char* ntp2 = "time.google.com";
  const char* ntp3 = "time.cloudflare.com";

  configTzTime(TZ_INFO, ntp1, ntp2, ntp3);

  Serial.println("[Time] Waiting for sync...");

  bool ok = false;
  for (int i = 0; i < 3; i++) {
    if (syncNtpOnce(5000)) { ok = true; break; }
    Serial.printf("[Time] NTP try %d failed\n", i + 1);
    delay(700);
  }

  if (ok) {
    Serial.println("[Time] NTP Sync Success!");
    if (rtcFound) {
      time_t nowUtc = time(nullptr);
      rtc.adjust(DateTime(nowUtc));
      Serial.println("[Time] RTC updated from NTP.");
    }
    isTimeSynced = true;
  } else if (rtcFound) {
    Serial.println("[Time] NTP failed. Using RTC time.");
    time_t rtcEpoch = rtc.now().unixtime();
    struct timeval tv;
    tv.tv_sec = rtcEpoch;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);

    isTimeSynced = true;
    Serial.println("[Time] System time set from RTC.");
  } else {
    Serial.println("[Time] Critical: no time source!");
    isTimeSynced = false;
  }

  // Debug ยืนยันเวลาปัจจุบันหลัง sync/fallback
  struct tm dbg;
  if (getLocalTimeSafe(&dbg)) {
    Serial.printf("[TimeDBG] %04d-%02d-%02d %02d:%02d:%02d\n",
                  dbg.tm_year + 1900, dbg.tm_mon + 1, dbg.tm_mday,
                  dbg.tm_hour, dbg.tm_min, dbg.tm_sec);
  } else {
    Serial.println("[TimeDBG] time invalid");
  }
  printTimeDebugBoth();
}

String macToString(const uint8_t* mac) {
  char s[18];
  snprintf(s, sizeof(s), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(s);
}

// --------------------- Network handler ---------------------
void handleNetwork() {
  bool isWifiConnected = (wifiMulti.run() == WL_CONNECTED);

  if (isWifiConnected && !wasWifiConnected) {
    Serial.println("[Network] Reconnected! Syncing time now...");
    delay(1200);
    timezoneSync();
    struct tm ti;
    lockEspNowChannelToFixed();
    if (getLocalTimeSafe(&ti)) {
      updateDisplay_Buffered(ti.tm_hour, ti.tm_min);
    }
    Serial.print("WiFi.status="); Serial.println(WiFi.status());
    Serial.print("IP="); Serial.println(WiFi.localIP());
    Serial.print("RSSI="); Serial.println(WiFi.RSSI());
    Serial.print("[WiFi] Channel=");
    Serial.println(WiFi.channel()); 
  }
  wasWifiConnected = isWifiConnected;

  if (millis() - lastNetworkCheck > NETWORK_INTERVAL) {
    lastNetworkCheck = millis();

    if (!isWifiConnected) {
      Serial.println("[WiFi] Lost... reconnecting");
    } else if (!client.connected()) {
      Serial.print("[MQTT] Connecting...");
      String clientId = String(mqtt_client_id) + "-" + String(random(0xffff), HEX);

      if (client.connect(clientId.c_str())) {
        Serial.println("Connected!");
        client.subscribe(mqtt_topic_cmd);
        client.publish(topic_status, "SYSTEM RECOVERED");
      } else {
        Serial.print("failed, rc=");
        Serial.println(client.state());
      }
    }

    if (isWifiConnected) {
      if (millis() - lastTimeResyncAttempt > 3600000UL) {
        lastTimeResyncAttempt = millis();
        timezoneSync();
      }
    }
  }
}

// --------------------- Daily reset ---------------------
void handleDailyReset(const struct tm& timeinfo) {
  int dayKey = timeinfo.tm_yday;
  if (lastResetDayKey == -1) {
    lastResetDayKey = preferences.getInt(KEY_DAY, -1);
  }
  if (dayKey != lastResetDayKey) {
    lastResetDayKey = dayKey;
    preferences.putInt(KEY_DAY, dayKey);

    current_DLI = 0.0f;
    preferences.putFloat(KEY_DLI, 0.0f);

    isMorningDone = false;
    isEveningDone = false;
    preferences.putBool(KEY_MDONE, false);
    preferences.putBool(KEY_EDONE, false);

    Serial.println("[Daily Reset] Cleared DLI + day flags");
  }
}

// --------------------- Telemetry ---------------------
void reportTelemetry(float lux) {
  if (!client.connected()) return;

  char msg[50];
  sprintf(msg, "%.2f", current_DLI);
  client.publish(topic_dli, msg);

  sprintf(msg, "%d", soilPercent);
  client.publish(topic_soil, msg);

  sprintf(msg, "%.2f", lux);
  client.publish(topic_lux, msg);

  client.publish(topic_valve, isValveMainOn ? "ON" : "OFF", true);
  client.publish(topic_light, isLightOn ? "ON" : "OFF", true);
  client.publish(topic_soil_link, isRemoteSoilHealthy() ? "OK" : "TIMEOUT", true);

  sprintf(msg, "%d", (int)currentPhase);
  client.publish(topic_phase, msg);

  struct tm ti;
  if (getLocalTimeSafe(&ti)) {
    // day after transplant
    time_t nowEpoch = mktime((struct tm*)&ti);
    int day = 0;
    if (transplantEpoch > 0 && nowEpoch >= transplantEpoch) {
      day = (int)((nowEpoch - transplantEpoch) / 86400);
    }
    sprintf(msg, "%d", day);
    client.publish(topic_day, msg);
  }

  String statusMsg = "V:" + String(isValveManual ? "MAN" : "AUTO")
                   + " | L:" + String(isLightManual ? "MAN" : "AUTO");
  client.publish(topic_status, statusMsg.c_str());

  Serial.println("[MQTT] Telemetry sent");
}

// --------------------- Lux read safe ---------------------
float readLuxSafe() {
  unsigned long now = millis();

  if (!bhReady) {
    if (now - lastBhRetryMs >= BH_RETRY_INTERVAL) {
      lastBhRetryMs = now;
      bhReady = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
      if (bhReady) {
        Serial.println("[BH1750] Recovered");
        luxSmoothed = 0.0f; 
        isLuxValid = false;
        lastValidLux = 0.0f;
        lastValidLuxMs = 0;
      } else {
        Serial.println("[BH1750] Re-init failed");
      }
    }
    return (float)LUX_SAFE_LIMIT + 1000.0f;
  }

  float raw = lightMeter.readLightLevel();
  bool valid = !isnan(raw) && !isinf(raw) && (raw >= LUX_MIN_VALID) && (raw <= LUX_MAX_VALID);

  if (!valid) {
    isLuxValid = false;
    bhReady = false;
    Serial.println("[Alarm] Lux sensor fault/out-of-range");

    if (lastValidLuxMs != 0 && (now - lastValidLuxMs) < 60000UL) return lastValidLux;
    return (float)LUX_SAFE_LIMIT + 1000.0f;
  }

  if (luxSmoothed <= 0.0f) luxSmoothed = raw;
  luxSmoothed = (luxAlpha * raw) + ((1.0f - luxAlpha) * luxSmoothed);

  isLuxValid = true;
  lastValidLux = luxSmoothed;
  lastValidLuxMs = now;
  return luxSmoothed;
}

// --------------------- Phase helpers ---------------------
int getDaysAfterTransplant(const struct tm& ti) {
  if (transplantEpoch <= 0) return 0;
  struct tm copy = ti;
  time_t nowEpoch = mktime(&copy);
  if (nowEpoch < transplantEpoch) return 0;
  return (int)((nowEpoch - transplantEpoch) / 86400);
}

GrowPhase phaseFromDay(int d) {
  if (d <= 20) return PHASE_1_ESTABLISH;
  if (d <= 45) return PHASE_2_VEGETATIVE;
  return PHASE_3_FINISHING;
}

void applyPhaseConfig(GrowPhase p) {
  currentPhase = p;
  phaseCfg = PHASE_TABLE[(int)p];
  target_DLI = phaseCfg.dliTarget;
}

bool inPhotoperiodNow(int nowMin) {
  return (nowMin >= phaseCfg.photoStartMin && nowMin < phaseCfg.photoEndMin);
}

// --------------------- DLI integration ---------------------
void integrateDLI(float lux) {
  unsigned long nowMs = millis();
  if (lastDliMillis == 0) lastDliMillis = nowMs;

  float dtSeconds = (nowMs - lastDliMillis) / 1000.0f;
  lastDliMillis = nowMs;

  if (dtSeconds < 0) dtSeconds = 0;
  if (dtSeconds > 5.0f) dtSeconds = 5.0f;

  if (isLuxValid || isLightOn) {
    float factor_used = isLightOn ? LIGHT_FACTOR : SUN_FACTOR;
    float ppfd = lux * factor_used;
    current_DLI += (ppfd * dtSeconds) / 1000000.0f;
  }

  bool timeToSave = (nowMs - lastDliSave > 900000UL);
  bool thresholdReached = (fabsf(current_DLI - lastSavedDLI) >= 0.5f);
  if (timeToSave || thresholdReached) {
    lastDliSave = nowMs;
    lastSavedDLI = current_DLI;
    preferences.putFloat(KEY_DLI, current_DLI);
    Serial.println("[System] DLI saved to NVS");
  }
}

// --------------------- Light control ---------------------
void controlLightResearch(int nowMin, float lux) {
  if (isLightManual) {
    isLightOn = (digitalRead(RELAY_LIGHT) == LOW);
    return;
  }

  bool inPhoto = inPhotoperiodNow(nowMin);
  float ppfdSun = lux * SUN_FACTOR;
  bool naturalEnough = (ppfdSun >= 250.0f);  // feed-forward from sunlight

  float onThreshold = phaseCfg.dliTarget - 0.25f;
  float offThreshold = phaseCfg.dliTarget - 0.05f;

  bool wantLight = false;

  if (inPhoto) {
    if (!naturalEnough) {
      if (!ledDecisionLatch && current_DLI < onThreshold) ledDecisionLatch = true;
      if (ledDecisionLatch && current_DLI >= offThreshold) ledDecisionLatch = false;
      wantLight = ledDecisionLatch;
    } else {
      ledDecisionLatch = false;
      wantLight = false;
    }
  } else {
    ledDecisionLatch = false;
    wantLight = false;
  }

  if (wantLight != isLightOn) {
    setRelayState(RELAY_LIGHT, wantLight);
    isLightOn = wantLight;
  }
}

// --------------------- Pulse helpers ---------------------
unsigned long selectPulseDurationMs() {
  if (currentPhase == PHASE_1_ESTABLISH) return 10000UL;   // 10s
  if (currentPhase == PHASE_2_VEGETATIVE) return 18000UL;  // 18s
  return 12000UL;                                           // finishing
}

void startValvePulse(unsigned long durationMs, bool force = false) {
  setValveStateSafe(true, force);
  if (!isValveMainOn) {          // เปิดไม่สำเร็จ (เช่นโดน anti-chatter)
    pulseActive = false;
    return;
  }
  valvePulseStartMs = millis();
  valvePulseDurationMs = durationMs;
  pulseActive = true;
}

void updateValvePulse() {
  if (!pulseActive) return;
  if (millis() - valvePulseStartMs >= valvePulseDurationMs) {
    setValveStateSafe(false);
    pulseActive = false;
  }
}

RemoteSoilSnapshot getRemoteSoilSnapshot() {
  RemoteSoilSnapshot s;
  noInterrupts();
  s.has = hasRemoteSoil;
  s.seq = remoteSeq;
  s.rxMs = lastSoilRxMs;
  interrupts();
  return s;
}

// -------------------------- AggregateSoilFrom4 ---------------------------------
int aggregateSoilFrom4(const int v[SOIL_COUNT], uint8_t okMask, bool &enoughValid) {
  int a[SOIL_COUNT];
  int n = 0;

  for (int i = 0; i < SOIL_COUNT; i++) {
    if (okMask & (1 << i)) {
      int x = constrain(v[i], 0, 100);
      a[n++] = x;
    }
  }

  if (n < 2) {
    enoughValid = false;
    return 100; // fail-safe
  }

  enoughValid = true;

  // sort
  for (int i = 1; i < n; i++) {
    int key = a[i], j = i - 1;
    while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
    a[j + 1] = key;
  }

  if (n >= 4) return (int)roundf((a[1] + a[2]) / 2.0f); // trimmed mean
  if (n == 3) return a[1];                               // median
  return (int)roundf((a[0] + a[1]) / 2.0f);              // average of 2
}

// --------------------- Helper Soil for Control --------------
int getSoilPercentForControl() {
  if (!hasRemoteSoil) return 100;
  if ((millis() - lastSoilRxMs) > SOIL_RX_TIMEOUT_MS) return 100;

  int tmp[SOIL_COUNT];
  uint8_t okMask;
  noInterrupts();
  for (int i = 0; i < SOIL_COUNT; i++) tmp[i] = remoteSoilPct[i];
  okMask = remoteSoilOkMask;
  interrupts();

  bool enough = false;
  int agg = aggregateSoilFrom4(tmp, okMask, enough);
  if (!enough) return 100;

  // EMA รวมอีกชั้นให้ controller นิ่ง
  if (!soilAggEmaInit) {
    soilAggEma = (float)agg;
    soilAggEmaInit = true;
  } else {
    soilAggEma = SOIL_AGG_ALPHA * agg + (1.0f - SOIL_AGG_ALPHA) * soilAggEma;
  }

  return constrain((int)roundf(soilAggEma), 0, 100);
}

bool isRemoteSoilHealthy() {
  if (!hasRemoteSoil) return false;
  if ((millis() - lastSoilRxMs) > SOIL_RX_TIMEOUT_MS) return false;

  uint8_t m;
  noInterrupts();
  m = remoteSoilOkMask;
  interrupts();

  // ต้องมี sensor ใช้งานได้อย่างน้อย 2 ตัว
  int cnt = 0;
  for (int i = 0; i < SOIL_COUNT; i++) if (m & (1 << i)) cnt++;
  return (cnt >= 2);
}

bool isRemoteLuxHealthy() {
  if (!hasRemoteLux) return false;
  if ((millis() - lastLuxRxMs) > LUX_RX_TIMEOUT_MS) return false;
  if (remoteLuxOk != 1) return false;

  float lx;
  noInterrupts();
  lx = remoteLux;
  interrupts();

  if (isnan(lx) || isinf(lx)) return false;
  if (lx < LUX_MIN_VALID || lx > LUX_MAX_VALID) return false;
  return true;
}

float getLuxForControlAndDLI() {
  if (isRemoteLuxHealthy()) {
    float lx;
    noInterrupts();
    lx = remoteLux;
    interrupts();
    return lx;
  }
  // fallback ไป local BH1750 ของ Node B
  return readLuxSafe();
}

// --------------------- Water control ---------------------
void controlWaterResearch(float lux) {
  soilPercent = getSoilPercentForControl();

  // ถ้าลิงก์ remote soil หาย ให้ fail-safe: ปิดวาล์ว auto
  if (!isValveManual && !isRemoteSoilHealthy()) {
    uiReason = "Failsafe: remote soil timeout";
    forceValveOffAndResetPulse();

    // กัน state auto-tune ค้าง
    soilBeforePulse = -1;
    soilAfterPulse = -1;
    pulseCycleCount = 0;

    return; // ข้ามการตัดสินใจรดน้ำรอบนี้
  }

  if (isValveManual) {
    isValveMainOn = (digitalRead(RELAY_VALVE_MAIN) == LOW);

    // กัน state ค้างจาก auto-tune/pulse เมื่อผู้ใช้คุมมือ
    pulseActive = false;
    irrState = IRR_IDLE;
    soilBeforePulse = -1;
    soilAfterPulse = -1;
    pulseCycleCount = 0;

    return;
  }
  // Emergency
  if (soilPercent < SOIL_CRITICAL) {
    uiReason = "Emergency watering: soil critical";
    isEmergencyMode = true;
    setValveStateSafe(true, true);

    pulseActive = false;
    irrState = IRR_IDLE;
    return;
  }

  if (isEmergencyMode && soilPercent >= phaseCfg.soilLow) {
  isEmergencyMode = false;
  forceValveOffAndResetPulse();
  }
  if (isEmergencyMode) return;

  if (!AUTO_TUNE_PULSE_ENABLED) {
    updateValvePulse();
  }

  bool ignoreSoilControl = (isValveMainOn && (millis() - valveLastSwitchMs < SOIL_IGNORE_AFTER_VALVE_ON_MS));
  if (ignoreSoilControl) {
    // ให้ state machine เดินต่อแม้ช่วง ignore
    updateAdaptivePulseStateMachine(soilPercent);
    return;
  }

  float ppfdSun = lux * SUN_FACTOR;
  int lowAdaptive = phaseCfg.soilLow;

  // Feed-forward เฉพาะช่วงโตเร็ว: แสงแรง -> เริ่มเติมน้ำเร็วขึ้นเล็กน้อย
  if (currentPhase == PHASE_2_VEGETATIVE && ppfdSun > 500.0f) {
    lowAdaptive = min(99, lowAdaptive + 3);
  }

  if (phaseCfg.dryBackMode) {
  // finishing: deep & dry-back
  if (soilPercent <= lowAdaptive) {
    if (millis() - lastDryBackWaterMs >= DRYBACK_MIN_INTERVAL_MS) {

      if (AUTO_TUNE_PULSE_ENABLED) {
        // ยิงเฉพาะตอน state พร้อม
        if (irrState == IRR_IDLE) {
          int targetSoil = getSoilTargetMidForPhase();
          startAdaptivePulseByNeed(soilPercent, targetSoil, false);

          if (irrState == IRR_IRRIGATING) {
            lastDryBackWaterMs = millis();
            pulseActive = true;
          }
        }
      } else {
        // fallback: auto-tune ปิด
        startValvePulse(selectPulseDurationMs(), false);
        if (isValveMainOn) {
          lastDryBackWaterMs = millis();
          pulseActive = true;
        }
      }
    } // end interval check
  }

  if (soilPercent >= phaseCfg.soilHigh) {
    forceValveOffAndResetPulse();
    pulseActive = false;
    irrState = IRR_IDLE;
  }
    updateAdaptivePulseStateMachine(soilPercent);
    return;
  }

  // Phase 1/2: maintain moisture band
  if (soilPercent <= lowAdaptive && !pulseActive && !isValveMainOn) {
    if (AUTO_TUNE_PULSE_ENABLED) {
    // เป้าหมายกลางแบนด์เดิมของ phase
    int targetSoil = getSoilTargetMidForPhase();

    // จำกัดการยิง pulse ไม่เกินรอบที่กำหนดต่อ cycle
    if (pulseCycleCount < MAX_PULSE_CYCLES_PER_CALL && irrState == IRR_IDLE) {
      startAdaptivePulseByNeed(soilPercent, targetSoil, false);

    // นับรอบเฉพาะกรณีที่เริ่ม pulse สำเร็จจริง
      if (irrState == IRR_IRRIGATING) {
        pulseCycleCount++;
        pulseActive = true;   // กันเงื่อนไข !pulseActive รอบถัดไป
      }
    }
  } else {
    startValvePulse(selectPulseDurationMs(), false);
    if (isValveMainOn) pulseActive = true;
    }
  }

  if (soilPercent >= phaseCfg.soilHigh) {
    forceValveOffAndResetPulse();
    pulseActive = false;
    pulseCycleCount = 0; 
    irrState = IRR_IDLE;
  }
  updateAdaptivePulseStateMachine(soilPercent);
}

// --------------------- Safe mode no valid time ---------------------
void handleNoValidTimeSafeMode() {
  if (!isValveManual) setValveStateSafe(false);
  if (!isLightManual) {
    setRelayState(RELAY_LIGHT, false);
    isLightOn = false;
  }
  Serial.println("[Time] Invalid system time -> automation paused");
}

// ===================== [ADD-ON] Auto-tuning Helpers =====================

PulseModel* getPulseModelByPhase() {
  // ใช้ currentPhase เดิมจากโค้ดคุณ
  if (currentPhase == PHASE_1_ESTABLISH) return &pulseModelP1;
  if (currentPhase == PHASE_2_VEGETATIVE) return &pulseModelP2;
  return &pulseModelP3;
}

int getSoilTargetMidForPhase() {
  // ใช้ phaseCfg เดิมจากโค้ดคุณ
  return (int)((phaseCfg.soilLow + phaseCfg.soilHigh) / 2);
}

// float clampf_local(float x, float lo, float hi) {
//   if (x < lo) return lo;
//   if (x > hi) return hi;
//   return x;
// }

float computeAdaptivePulseSec(int soilNow, int soilTarget) {
  PulseModel* pm = getPulseModelByPhase();
  if (!pm) return 10.0f;

  float need = (float)(soilTarget - soilNow);
  if (need < 0.0f) need = 0.0f;

  float k = pm->kWetEst;
  if (k < 0.08f) k = 0.08f;     // กันหารเล็กเกินจริง
  if (k > 3.00f) k = 3.00f;     // กันเพี้ยนสูงเกินจริง

  float pulseSec = need / k;

  // dead-min: ถ้าเข้าเงื่อนไขต้องเปิดน้ำแล้ว ให้มีขั้นต่ำเพื่อให้มีผลจริง
  if (pulseSec < pm->minPulseSec) pulseSec = pm->minPulseSec;
  if (pulseSec > pm->maxPulseSec) pulseSec = pm->maxPulseSec;

  return pulseSec;
}

void learnKwetFromPulse(int soilBefore, int soilAfter, float pulseSec) {
  PulseModel* pm = getPulseModelByPhase();
  if (!pm) return;

  if (soilBefore < 0 || soilAfter < 0 || pulseSec < 1.0f) return;

  float gain = (float)(soilAfter - soilBefore);  // %
  // reject outliers
  if (gain < 0.3f || gain > 25.0f) return;

  float kNew = gain / pulseSec;  // %/sec
  if (kNew < 0.05f || kNew > 3.0f) return;

  // EMA update
  float b = pm->emaBeta;
  pm->kWetEst = b * kNew + (1.0f - b) * pm->kWetEst;

  lastLearnedKwet = pm->kWetEst;

  // บันทึกลง NVS ทันทีเล็กน้อย (ความถี่ต่ำจากรอบการรดจริง)
  if (currentPhase == PHASE_1_ESTABLISH) preferences.putFloat(KEY_KWET_P1, pulseModelP1.kWetEst);
  else if (currentPhase == PHASE_2_VEGETATIVE) preferences.putFloat(KEY_KWET_P2, pulseModelP2.kWetEst);
  else preferences.putFloat(KEY_KWET_P3, pulseModelP3.kWetEst);

  Serial.print("[AutoTune] Learned kWet=");
  Serial.print(pm->kWetEst, 4);
  Serial.print(" %/s, gain=");
  Serial.print(gain, 2);
  Serial.print("%, pulse=");
  Serial.print(pulseSec, 2);
  Serial.println("s");
}

void startAdaptivePulseByNeed(int soilNow, int soilTarget, bool force) {
  float pulseSec = computeAdaptivePulseSec(soilNow, soilTarget);
  lastComputedPulseSec = pulseSec;

  unsigned long durMs = (unsigned long)(pulseSec * 1000.0f);

  // ใช้ setValveStateSafe เดิม -> anti-chatter เดิมยังอยู่ครบ
  setValveStateSafe(true, force);
  bool after = isValveMainOn;

  // ถ้าวาล์วไม่เปิดจริง (โดน min-off block) อย่าเปลี่ยน state machine
  if (!after) {
    uiReason = "Irrigation blocked: min-off guard";
    Serial.println("[AutoTune] Pulse request blocked by anti-chatter/min-off");
    return;
  }else{
    uiReason = "Irrigation pulse start " + String(pulseSec, 1) + "s";
  }

  pulseStartMs = millis();
  pulseStopMs = pulseStartMs + durMs;
  soilBeforePulse = soilNow;
  irrState = IRR_IRRIGATING;

  Serial.print("[AutoTune] Start pulse ");
  Serial.print(pulseSec, 2);
  Serial.print("s | soilBefore=");
  Serial.println(soilBeforePulse);
}

void updateAdaptivePulseStateMachine(int currentSoil) {
  if (!AUTO_TUNE_PULSE_ENABLED) return;

  unsigned long now = millis();

  if (irrState == IRR_IRRIGATING) {
    if (now >= pulseStopMs) {
      // ปิดวาล์วด้วย logic เดิม
      setValveStateSafe(false, true);
      soakWaitStartMs = now;
      irrState = IRR_SOAK_WAIT;
      Serial.println("[AutoTune] Pulse finished -> soak wait");
    }
    return;
  }

  if (irrState == IRR_SOAK_WAIT) {
    if (now - soakWaitStartMs >= SOAK_WAIT_MS) {
      soilAfterPulse = currentSoil;
      float pulseSec = (float)(pulseStopMs - pulseStartMs) / 1000.0f;
      learnKwetFromPulse(soilBeforePulse, soilAfterPulse, pulseSec);

      // reset state
      irrState = IRR_IDLE;
      pulseActive = false;
      soilBeforePulse = -1;
      soilAfterPulse = -1;

      Serial.print("[AutoTune] Soak done | soilAfter=");
      Serial.println(currentSoil);

      if (currentSoil >= phaseCfg.soilLow) {
        pulseCycleCount = 0;
      }
    }
  }
}

// --------------------- Main calculate ---------------------
void calculate(int currentHour, int currentMin) {
  // Manual fail-safe when MQTT gone for long
  static unsigned long mqttLostSince = 0;
  if (!client.connected()) {
    if (mqttLostSince == 0) mqttLostSince = millis();
    if (millis() - mqttLostSince > 30000UL) {
      isValveManual = false;
      isLightManual = false;
    }
  } else {
    mqttLostSince = 0;
  }

  int nowMin = currentHour * 60 + currentMin;

  float lux = getLuxForControlAndDLI();
  bool luxOk = !isnan(lux) && !isinf(lux) && lux >= LUX_MIN_VALID && lux <= LUX_MAX_VALID;
  isLuxValid = luxOk;

  // ถ้า lux ไม่โอเค: เลือก policy เดียวให้ชัด
  if (!luxOk) {
    // ตัวอย่าง policy: ปิดไฟอัตโนมัติและข้าม DLI รอบนี้
    if (!isLightManual) {
      setRelayState(RELAY_LIGHT, false);
      isLightOn = false;
    }
    // soil control ยังเดินต่อได้ (เพราะใช้ remote soil)
    controlWaterResearch(0.0f);
    lastLuxForTelemetry = NAN;
    hasLastLuxForTelemetry = true;
    return;
  }

  lastLuxForTelemetry = lux;
  hasLastLuxForTelemetry = true;

  // 1) Light control
  controlLightResearch(nowMin, lux);

  // 2) DLI integration
  integrateDLI(lux);

  // 3) Water control
  controlWaterResearch(lux);

  // Debug
  if (DEBUG_LOG && (millis() - lastDebug >= DEBUG_INTERVAL)) {
    lastDebug = millis();
    PulseModel* pm = getPulseModelByPhase();
    Serial.println("----- STATUS (Research Aligned) -----");
    Serial.print("Phase: "); Serial.println((int)currentPhase);
    Serial.print("Target DLI: "); Serial.println(phaseCfg.dliTarget);
    Serial.print("Photo(min): "); Serial.print(phaseCfg.photoStartMin); Serial.print(" -> "); Serial.println(phaseCfg.photoEndMin);
    Serial.print("Soil band: "); Serial.print(phaseCfg.soilLow); Serial.print(" - "); Serial.println(phaseCfg.soilHigh);
    Serial.print("Lux: "); Serial.println(lux);
    Serial.print("DLI: "); Serial.println(current_DLI);
    Serial.print("Soil%: "); Serial.println(soilPercent);
    Serial.print("Light: "); Serial.println(isLightOn ? "ON" : "OFF");
    Serial.print("Valve: "); Serial.println(isValveMainOn ? "ON" : "OFF");
    Serial.print("Emergency: "); Serial.println(isEmergencyMode ? "YES" : "NO");
    Serial.print(" | AutoTune: "); Serial.println(AUTO_TUNE_PULSE_ENABLED ? "ON" : "OFF");
    Serial.print(" | irrState: "); Serial.println((int)irrState);
    Serial.print(" | kWetEst: "); Serial.println(pm ? pm->kWetEst : -1.0f, 4);
    Serial.print(" | lastPulseSec: "); Serial.println(lastComputedPulseSec, 2);
    RemoteSoilSnapshot s = getRemoteSoilSnapshot();
    Serial.print("RemoteSoilHealthy: "); Serial.println(isRemoteSoilHealthy() ? "YES" : "NO");
    int p0,p1,p2,p3; uint8_t mk;
    noInterrupts();
    p0=remoteSoilPct[0]; p1=remoteSoilPct[1]; p2=remoteSoilPct[2]; p3=remoteSoilPct[3];
    mk=remoteSoilOkMask;
    interrupts();
    Serial.printf("RemoteSoil4: [%d,%d,%d,%d] mask=0x%02X\n", p0,p1,p2,p3,mk);
    Serial.print("RemoteSeq: "); Serial.println((unsigned long)s.seq);
    Serial.print("SoilRxAgeMs: ");
    if (!s.has) Serial.println(-1);
    else Serial.println((unsigned long)(millis() - s.rxMs));
  }
}

// ------------------- Print Time Debug Helper ---------------------
void printTimeDebugBoth() {
  time_t now = time(nullptr);

  struct tm tmUtc;
  gmtime_r(&now, &tmUtc);

  struct tm tmLoc;
  localtime_r(&now, &tmLoc);

  Serial.printf("[TimeDBG][UTC ] %04d-%02d-%02d %02d:%02d:%02d\n",
    tmUtc.tm_year + 1900, tmUtc.tm_mon + 1, tmUtc.tm_mday,
    tmUtc.tm_hour, tmUtc.tm_min, tmUtc.tm_sec);

  Serial.printf("[TimeDBG][LOC ] %04d-%02d-%02d %02d:%02d:%02d\n",
    tmLoc.tm_year + 1900, tmLoc.tm_mon + 1, tmLoc.tm_mday,
    tmLoc.tm_hour, tmLoc.tm_min, tmLoc.tm_sec);
}

bool initEspNowReceiver() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    return false;
  }

  if (esp_now_register_recv_cb(onEspNowRecv) != ESP_OK) {
    Serial.println("[ESP-NOW] register recv cb failed");
    return false;
  }

  Serial.printf("[ESP-NOW] Receiver ready | SoilPacket=%u LuxPacket=%u\n",
                (unsigned)sizeof(SoilPacket), (unsigned)sizeof(LuxPacket));
  return true;
}

// ------------------ Compare MAC Address Node A ------------------------
bool isSameMac(const uint8_t* a, const uint8_t* b) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

// -------------- ESP-NOW receive callback (ESP32 core ใหม่) -----------------------
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data || len < 1) return;

  uint8_t type = data[0];

  // ---------- SOIL ----------
  if (type == MSG_SOIL) {
    if (len != (int)sizeof(SoilPacket)) {
      Serial.printf("[RX SOIL] bad len=%d expect=%u\n", len, (unsigned)sizeof(SoilPacket));
      return;
    }

    SoilPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    for (int i = 0; i < SOIL_COUNT; i++) {
      if (pkt.soilPct[i] < 0) pkt.soilPct[i] = 0;
      if (pkt.soilPct[i] > 100) pkt.soilPct[i] = 100;
    }

    noInterrupts();
    for (int i = 0; i < SOIL_COUNT; i++) remoteSoilPct[i] = pkt.soilPct[i];
    remoteSoilOkMask = pkt.sensorOkMask;
    remoteSeq = pkt.seq;
    hasRemoteSoil = true;
    lastSoilRxMs = millis();
    interrupts();

    Serial.printf("[RX SOIL] from=%s seq=%lu p=[%d,%d,%d,%d] mask=0x%02X\n",
      macToString(info->src_addr).c_str(),
      (unsigned long)pkt.seq,
      pkt.soilPct[0], pkt.soilPct[1], pkt.soilPct[2], pkt.soilPct[3],
      pkt.sensorOkMask);
    return;
  }

  // ---------- LUX (BH1750) ----------
  if (type == MSG_LUX) {
    if (len != (int)sizeof(LuxPacket)) {
      Serial.printf("[RX LUX ] bad len=%d expect=%u\n", len, (unsigned)sizeof(LuxPacket));
      return;
    }

    LuxPacket lp;
    memcpy(&lp, data, sizeof(lp));

    bool luxOk = (lp.sensorOk == 1) &&
                 !isnan(lp.lux) && !isinf(lp.lux) &&
                 (lp.lux >= LUX_MIN_VALID) && (lp.lux <= LUX_MAX_VALID);

    noInterrupts();
    remoteLux = lp.lux;
    remoteLuxSeq = lp.seq;
    remoteLuxOk = luxOk ? 1 : 0;
    hasRemoteLux = true;
    lastLuxRxMs = millis();
    interrupts();

    Serial.printf("[RX LUX ] from=%s seq=%lu lux=%.2f ok=%u\n",
      macToString(info->src_addr).c_str(),
      (unsigned long)lp.seq, lp.lux, (unsigned)remoteLuxOk);
    return;
  }

  // ignore อื่น ๆ
  Serial.printf("[ESP-NOW] unknown type=%u len=%d\n", (unsigned)type, len);
}

void drawHeader(int h, int m) {
  // แถบบน
  sprite.fillRoundRect(4, 4, 232, 30, 6, TFT_DARKGREY);

  // เวลา
  char ts[6];
  sprintf(ts, "%02d:%02d", h, m);
  sprite.setTextDatum(TR_DATUM);
  sprite.setTextColor(TFT_CYAN, TFT_DARKGREY);
  sprite.setTextSize(2);
  sprite.drawString(ts, 232, 12);

  // MODE + PHASE
  sprite.setTextDatum(TL_DATUM);
  sprite.setTextColor(TFT_WHITE, TFT_DARKGREY);
  String mode = String(isValveManual || isLightManual ? "MANUAL" : "AUTO");
  String ph = String("PH:") + phaseText(currentPhase);
  sprite.drawString(mode + "  " + ph, 10, 12);

  // Alarm dot
  bool warn = (!isRemoteSoilHealthy()) || (!isRemoteLuxHealthy()) || isEmergencyMode || (!isTimeSynced);
  uint16_t dot = warn ? TFT_RED : TFT_GREEN;
  sprite.fillCircle(210, 19, 5, dot);
}

void drawPageOperate() {
  // Soil big card
  sprite.fillRoundRect(8, 40, 108, 74, 10, TFT_NAVY);
  sprite.setTextDatum(TL_DATUM);
  sprite.setTextColor(TFT_WHITE, TFT_NAVY);
  sprite.setTextSize(1);
  sprite.drawString("SOIL %", 16, 48);

  uint16_t soilCol = colorByLevel(soilPercent, phaseCfg.soilLow, phaseCfg.soilHigh);
  sprite.setTextColor(soilCol, TFT_NAVY);
  sprite.setTextSize(4);
  sprite.setTextDatum(MC_DATUM);
  sprite.drawNumber(soilPercent, 62, 84);

  // Lux card
  sprite.fillRoundRect(124, 40, 108, 74, 10, TFT_DARKGREEN);
  sprite.setTextDatum(TL_DATUM);
  sprite.setTextColor(TFT_WHITE, TFT_DARKGREEN);
  sprite.setTextSize(1);
  sprite.drawString("LUX", 132, 48);
  sprite.setTextSize(2);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextColor(TFT_YELLOW, TFT_DARKGREEN);
  String luxText = "N/A";
  if (!isnan(lastLuxForTelemetry) && !isinf(lastLuxForTelemetry)) {
    luxText = String((int)lastLuxForTelemetry);
  }
  sprite.drawString(luxText, 178, 84);

  // DLI card (กว้างเต็ม)
  sprite.fillRoundRect(8, 120, 224, 46, 10, TFT_MAROON);
  sprite.setTextDatum(TL_DATUM);
  sprite.setTextSize(1);
  sprite.setTextColor(TFT_WHITE, TFT_MAROON);
  sprite.drawString("DLI TODAY / TARGET", 16, 128);

  sprite.setTextDatum(TR_DATUM);
  sprite.setTextSize(2);
  float dliLeft = current_DLI;
  float dliRight = phaseCfg.dliTarget;
  uint16_t dliCol = (dliLeft >= dliRight - 0.1f) ? TFT_GREEN : TFT_ORANGE;
  sprite.setTextColor(dliCol, TFT_MAROON);
  sprite.drawString(String(dliLeft, 1) + " / " + String(dliRight, 1), 224, 152);

  // Valve / Light state pills
  sprite.fillRoundRect(8, 172, 108, 32, 8, TFT_BLACK);
  sprite.drawRoundRect(8, 172, 108, 32, 8, TFT_WHITE);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextSize(1);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("VALVE", 38, 188);
  sprite.setTextColor(isValveMainOn ? TFT_GREEN : TFT_RED, TFT_BLACK);
  sprite.drawString(isValveMainOn ? "ON" : "OFF", 88, 188);

  sprite.fillRoundRect(124, 172, 108, 32, 8, TFT_BLACK);
  sprite.drawRoundRect(124, 172, 108, 32, 8, TFT_WHITE);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("LIGHT", 154, 188);
  sprite.setTextColor(isLightOn ? TFT_GREEN : TFT_RED, TFT_BLACK);
  sprite.drawString(isLightOn ? "ON" : "OFF", 204, 188);

  // Reason bar
  sprite.fillRoundRect(8, 208, 224, 28, 8, TFT_DARKGREY);
  sprite.setTextDatum(TL_DATUM);
  sprite.setTextSize(1);
  sprite.setTextColor(TFT_WHITE, TFT_DARKGREY);
  String s = uiReason;
  if (s.length() > 34) s = s.substring(0, 34);
  sprite.drawString(s, 14, 218);
}

void drawPageHealth() {
  sprite.setTextSize(1);
  sprite.setTextDatum(TL_DATUM);

  sprite.fillRoundRect(8, 40, 224, 196, 10, TFT_BLACK);
  sprite.drawRoundRect(8, 40, 224, 196, 10, TFT_WHITE);

  int y = 50;
  auto row = [&](const char* k, String v, uint16_t c) {
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.drawString(k, 16, y);
    sprite.setTextDatum(TR_DATUM);
    sprite.setTextColor(c, TFT_BLACK);
    sprite.drawString(v, 224, y);
    sprite.setTextDatum(TL_DATUM);
    y += 20;
  };

  unsigned long soilAge = hasRemoteSoil ? (millis() - lastSoilRxMs) : 999999;
  unsigned long luxAge  = hasRemoteLux  ? (millis() - lastLuxRxMs)  : 999999;

  row("Soil Link", isRemoteSoilHealthy() ? "OK" : "TIMEOUT", colorByBool(isRemoteSoilHealthy()));
  row("Soil Rx Age", String(soilAge) + " ms", (soilAge <= SOIL_RX_TIMEOUT_MS) ? TFT_GREEN : TFT_RED);
  uint8_t m;
  noInterrupts();
  m = remoteSoilOkMask;
  interrupts();
  int validCnt = 0;
  for (int i=0;i<SOIL_COUNT;i++) if (m & (1<<i)) validCnt++;
  row("Soil ValidCnt", String(validCnt) + "/4", colorByBool(validCnt >= 2));

  row("Lux Link", isRemoteLuxHealthy() ? "OK" : "TIMEOUT", colorByBool(isRemoteLuxHealthy()));
  row("Lux Rx Age", String(luxAge) + " ms", (luxAge <= LUX_RX_TIMEOUT_MS) ? TFT_GREEN : TFT_RED);
  row("Lux SensorOK", String(remoteLuxOk), colorByBool(remoteLuxOk == 1));

  row("WiFi", (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "LOST", colorByBool(WiFi.status() == WL_CONNECTED));
  row("MQTT", client.connected() ? "CONNECTED" : "LOST", colorByBool(client.connected()));
  row("Time Sync", isTimeSynced ? "OK" : "INVALID", colorByBool(isTimeSynced));
}

void drawPageControl() {
  sprite.fillRoundRect(8, 40, 224, 196, 10, TFT_BLACK);
  sprite.drawRoundRect(8, 40, 224, 196, 10, TFT_WHITE);

  sprite.setTextSize(1);
  sprite.setTextDatum(TL_DATUM);

  int y = 50;
  auto row = [&](const char* k, String v, uint16_t c = TFT_CYAN) {
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.drawString(k, 16, y);
    sprite.setTextDatum(TR_DATUM);
    sprite.setTextColor(c, TFT_BLACK);
    sprite.drawString(v, 224, y);
    sprite.setTextDatum(TL_DATUM);
    y += 20;
  };

  row("Soil Band", String(phaseCfg.soilLow) + "-" + String(phaseCfg.soilHigh), TFT_YELLOW);
  row("Photo", 
      String(phaseCfg.photoStartMin / 60) + ":" + (phaseCfg.photoStartMin % 60 < 10 ? "0" : "") + String(phaseCfg.photoStartMin % 60)
      + " - " +
      String(phaseCfg.photoEndMin / 60) + ":" + (phaseCfg.photoEndMin % 60 < 10 ? "0" : "") + String(phaseCfg.photoEndMin % 60),
      TFT_YELLOW);

  row("DLI Target", String(phaseCfg.dliTarget, 1), TFT_YELLOW);
  row("DryBack", phaseCfg.dryBackMode ? "ON" : "OFF", phaseCfg.dryBackMode ? TFT_GREEN : TFT_ORANGE);

  PulseModel* pm = getPulseModelByPhase();
  row("AutoTune", AUTO_TUNE_PULSE_ENABLED ? "ON" : "OFF", AUTO_TUNE_PULSE_ENABLED ? TFT_GREEN : TFT_ORANGE);
  row("kWetEst", pm ? String(pm->kWetEst, 3) : "-", TFT_CYAN);
  row("lastPulseSec", String(lastComputedPulseSec, 1), TFT_CYAN);
  row("irrState", String((int)irrState), TFT_CYAN);
  row("pulseCycle", String(pulseCycleCount) + "/" + String(MAX_PULSE_CYCLES_PER_CALL), TFT_CYAN);
}

// --------------------- Setup ---------------------
void setup() {
  Serial.begin(115200);
  setenv("TZ", TZ_INFO, 1); // Thailand UTC+7
  tzset();
  // ใช้ STA เพื่อทำงานร่วมกับ WiFi + MQTT ได้
  WiFi.mode(WIFI_STA);

  Serial.println("\n--- System Starting ---");
  Wire.begin(SDA, SCL);

  // RTC
  if (!rtc.begin()) {
    Serial.println("Error: RTC not found!");
    rtcFound = false;
  } else {
    rtcFound = true;
    Serial.println("RTC Found");
    if (rtc.lostPower()) Serial.println("RTC lost power, sync needed");
  }

  pinMode(SQW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SQW_PIN), onRTCAlarm, FALLING);
  setupRTCAlarm();

  // NVS
  preferences.begin("smartfarm", false);
  current_DLI = preferences.getFloat(KEY_DLI, 0.0f);
  lastSavedDLI = current_DLI;
  isMorningDone = preferences.getBool(KEY_MDONE, false);
  isEveningDone = preferences.getBool(KEY_EDONE, false);
  transplantEpoch = (time_t)preferences.getULong64(KEY_TRANS_EPOCH, 0ULL);
  // [ADD-ON] Restore learned kWet models
  float k1 = preferences.getFloat(KEY_KWET_P1, pulseModelP1.kWetEst);
  float k2 = preferences.getFloat(KEY_KWET_P2, pulseModelP2.kWetEst);
  float k3 = preferences.getFloat(KEY_KWET_P3, pulseModelP3.kWetEst);

  // validate กันค่าเสีย
  if (k1 >= 0.05f && k1 <= 3.0f) pulseModelP1.kWetEst = k1;
  if (k2 >= 0.05f && k2 <= 3.0f) pulseModelP2.kWetEst = k2;
  if (k3 >= 0.05f && k3 <= 3.0f) pulseModelP3.kWetEst = k3;

  Serial.print("[AutoTune] kWet P1="); Serial.println(pulseModelP1.kWetEst, 4);
  Serial.print("[AutoTune] kWet P2="); Serial.println(pulseModelP2.kWetEst, 4);
  Serial.print("[AutoTune] kWet P3="); Serial.println(pulseModelP3.kWetEst, 4);

  Serial.print("Restored DLI: "); Serial.println(current_DLI);

  // Display
  tft.init();
  tft.fillScreen(TFT_BLACK);

  sprite.setColorDepth(8);
  void* ptr = sprite.createSprite(240, 240);
  if (ptr == NULL) Serial.println("ERROR: Not enough RAM for Sprite!");
  else Serial.println("Sprite created successfully");

  // I/O
  pinMode(RELAY_LIGHT, OUTPUT);
  pinMode(RELAY_VALVE_MAIN, OUTPUT);
  // pinMode(SOIL_PIN, INPUT); ย้ายไป node A แล้ว

  setRelayState(RELAY_LIGHT, false);
  setRelayState(RELAY_VALVE_MAIN, false);

  // MQTT
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  // WiFi
  wifiMulti.addAP(ssid_3, pass_3);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("CONNECTING WIFI...", 10, 10, 4);
  tft.drawString("BOOTING SYSTEM...", 10, 40, 4);

  Serial.print("Connecting WiFi");
  unsigned long startAttempt = millis();
  while (millis() - startAttempt < 15000UL) {
    if (wifiMulti.run() == WL_CONNECTED) break;
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  if (wifiMulti.run() == WL_CONNECTED) {
    lockEspNowChannelToFixed();
    Serial.printf("[WiFi] channel=%d\n", WiFi.channel());
  } else {
    Serial.println("[WiFi] connect timeout, ESP-NOW may fail if channel mismatch");
  }

  // Init ESP-NOW receiver (Node B)
  if (!initEspNowReceiver()) {
    Serial.println("[ESP-NOW] Receiver init error");
  }

  tft.fillScreen(TFT_BLACK);
  tft.drawString("SYNCING TIME...", 10, 10, 4);
  timezoneSync();

  // initialize transplant epoch if empty
  struct tm ti;
  if (getLocalTimeSafe(&ti)) {
    if (transplantEpoch == 0) {
      struct tm t0 = ti;
      t0.tm_hour = 0;
      t0.tm_min = 0;
      t0.tm_sec = 0;
      transplantEpoch = mktime(&t0);
      preferences.putULong64(KEY_TRANS_EPOCH, (uint64_t)transplantEpoch);
      Serial.println("[Init] transplantEpoch set to today 00:00");
    }

    int day = getDaysAfterTransplant(ti);
    applyPhaseConfig(phaseFromDay(day));
    Serial.print("[Phase Init] Day=");
    Serial.print(day);
    Serial.print(" -> Phase=");
    Serial.println((int)currentPhase);
  } else {
    applyPhaseConfig(PHASE_1_ESTABLISH);
  }

  updateDisplay_Buffered(12, 0);
  Serial.println("System Ready");
}

// --------------------- Loop ---------------------
void loop() {
  handleNetwork();

  if (client.connected()) client.loop();

  // RTC minute tick
  if (alarmTriggered) {
    alarmTriggered = false;
    if (rtcFound) rtc.clearAlarm(1);
    Serial.println("[SQW] Minute Tick");
  }

  // Control loop
  if (millis() - lastCalcUpdate > CONTROL_INTERVAL) {
    lastCalcUpdate = millis();

    struct tm timeinfo;
    int h = 12, m = 0;
    bool validTime = getLocalTimeSafe(&timeinfo);

    if (validTime) {
      h = timeinfo.tm_hour;
      m = timeinfo.tm_min;
      isTimeSynced = true;

      handleDailyReset(timeinfo);

      // Update phase by day
      int day = getDaysAfterTransplant(timeinfo);
      GrowPhase p = phaseFromDay(day);
      if (p != currentPhase) {
        applyPhaseConfig(p);
        preferences.putFloat(KEY_DLI, current_DLI);
        Serial.print("[Phase Change] Day=");
        Serial.print(day);
        Serial.print(" -> Phase=");
        Serial.println((int)currentPhase);
      }

      calculate(h, m);
    } else {
      isTimeSynced = false;
      Serial.println("Time Error: waiting for sync...");
      handleNoValidTimeSafeMode();
    }

    updateDisplay_Buffered(h, m);
  }

  // Telemetry loop
  if (millis() - lastTelemetry > TELEMETRY_INTERVAL) {
    lastTelemetry = millis();
    float currentLux = hasLastLuxForTelemetry ? lastLuxForTelemetry : readLuxSafe();
    reportTelemetry(currentLux);
  }
}