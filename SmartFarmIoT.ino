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
#define SOIL_PIN 34
#define RELAY_VALVE_MAIN 16
#define SQW_PIN 27

#define SDA 21
#define SCL 22

// --------------------- Soil Calibration ---------------------
const int AIR_VALUE = 2730;    // ค่าแห้ง (ต้องคาลิเบรตตามของจริง)
const int WATER_VALUE = 970;   // ค่าเปียก (ต้องคาลิเบรตตามของจริง)

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
float soilSmoothedPct = 0.0f;
const float soilAlpha = 0.18f;
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
  int pct;
  uint32_t seq;
  uint8_t ok;
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
typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t  soilPct;     // 0..100
  uint16_t soilRaw;     // raw ADC from node A (optional for debug)
  uint32_t uptimeMs;
  uint8_t  sensorOk;    // 1=ok, 0=fault
} SoilPacket; // รับมาจาก Node A

typedef struct __attribute__((packed)) {
  uint32_t seq;
  float    lux;
  uint32_t uptimeMs;
  uint8_t  sensorOk;    // 1=ok, 0=fault
} LuxPacket; // รับมาจาก Node A

volatile bool hasRemoteSoil = false;
volatile int remoteSoilPct = 0;
volatile uint32_t remoteSeq = 0;
volatile uint8_t remoteSensorOk = 0;
volatile unsigned long lastSoilRxMs = 0;

const unsigned long SOIL_RX_TIMEOUT_MS = 15000UL;   // ถ้าเกินนี้ถือว่าลิงก์หาย

volatile bool hasRemoteLux = false;
volatile float remoteLux = 0.0f;
volatile uint32_t remoteLuxSeq = 0;
volatile uint8_t remoteLuxOk = 0;
volatile unsigned long lastLuxRxMs = 0;

const unsigned long LUX_RX_TIMEOUT_MS = 15000UL;

// 1C:C3:AB:B4:77:CC
const uint8_t ALLOWED_SENDER_MAC[6] = {0x1C, 0xC3, 0xAB, 0xB4, 0x77, 0xCC}; // แก้เป็น MAC ของ Node A จริง
bool isSameMac(const uint8_t* a, const uint8_t* b);

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
const char* ssid_3 = "JebHuaJai";
const char* pass_3 = "ffffffff";

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

// วันเริ่มปลูก (local epoch)
time_t transplantEpoch = 0;

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

// --------------------- Display ---------------------
void updateDisplay_Buffered(int h, int m) {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextDatum(TL_DATUM);

  // Static
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.setTextSize(3);
  sprite.drawString("Soil", 10, 10);
  sprite.drawString("%", 215, 10);
  sprite.drawString("DLI", 10, 40);
  sprite.drawLine(10, 71, 229, 71, TFT_WHITE);

  // Dynamic
  sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
  sprite.setTextDatum(TR_DATUM);
  sprite.drawNumber(soilPercent, 200, 10);
  sprite.drawFloat(current_DLI, 2, 200, 40);

  // Labels
  sprite.setTextSize(2);
  sprite.setTextDatum(TL_DATUM);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("VALVE", 49, 85);
  sprite.drawString("LIGHT", 164, 85);

  // Mode circles
  uint16_t valveModeColor = isValveManual ? TFT_ORANGE : TFT_CYAN;
  sprite.fillEllipse(62, 122, 23, 23, valveModeColor);
  sprite.setTextColor(TFT_BLACK, valveModeColor);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextSize(1);
  sprite.drawString(isValveManual ? "MAN" : "AUTO", 62, 122);

  uint16_t lightModeColor = isLightManual ? TFT_ORANGE : TFT_CYAN;
  sprite.fillEllipse(177, 122, 23, 23, lightModeColor);
  sprite.setTextColor(TFT_BLACK, lightModeColor);
  sprite.drawString(isLightManual ? "MAN" : "AUTO", 177, 122);

  // ON/OFF indicators
  sprite.fillEllipse(34, 184, 20, 20, isValveMainOn ? TFT_GREEN : TFT_BLACK);
  sprite.fillEllipse(91, 184, 20, 20, isValveMainOn ? TFT_BLACK : TFT_RED);

  sprite.fillEllipse(148, 184, 20, 20, isLightOn ? TFT_GREEN : TFT_BLACK);
  sprite.fillEllipse(205, 184, 20, 20, isLightOn ? TFT_BLACK : TFT_RED);

  // Indicator labels
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("ON", 34, 160);
  sprite.drawString("OFF", 91, 160);
  sprite.drawString("ON", 148, 160);
  sprite.drawString("OFF", 205, 160);

  // Time
  sprite.setTextSize(2);
  sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  sprite.setTextDatum(BR_DATUM);
  char timeStr[10];
  sprintf(timeStr, "%02d:%02d", h, m);
  sprite.drawString(timeStr, 235, 235);

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

// --------------------- Network handler ---------------------
void handleNetwork() {
  bool isWifiConnected = (wifiMulti.run() == WL_CONNECTED);

  if (isWifiConnected && !wasWifiConnected) {
    Serial.println("[Network] Reconnected! Syncing time now...");
    delay(1200);
    timezoneSync();
    struct tm ti;
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

// --------------------- Soil robust read ---------------------
int median5(int a, int b, int c, int d, int e) {
  int arr[5] = {a, b, c, d, e};
  for (int i = 1; i < 5; i++) {
    int key = arr[i], j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
  return arr[2];
}

bool isRawSoilFault(int raw) {
  return (raw > 3500 || raw < 100);
}

int rawToPercent(int raw) {
  int p = map(raw, AIR_VALUE, WATER_VALUE, 0, 100);
  return constrain(p, 0, 100);
}

int getSoilPercentRobust() {
  int r1 = analogRead(SOIL_PIN);
  int r2 = analogRead(SOIL_PIN);
  int r3 = analogRead(SOIL_PIN);
  int r4 = analogRead(SOIL_PIN);
  int r5 = analogRead(SOIL_PIN);
  int raw = median5(r1, r2, r3, r4, r5);

  if (isRawSoilFault(raw)) {
    Serial.println("[Alarm] Soil sensor fault detected");
    return 100;  // fail-safe
  }

  int p = rawToPercent(raw);

  if (soilSmoothedPct <= 0.01f) soilSmoothedPct = (float)p;
  soilSmoothedPct = soilAlpha * p + (1.0f - soilAlpha) * soilSmoothedPct;

  return constrain((int)roundf(soilSmoothedPct), 0, 100);
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
  s.pct = remoteSoilPct;
  s.seq = remoteSeq;
  s.ok = remoteSensorOk;
  s.rxMs = lastSoilRxMs;
  interrupts();
  return s;
}

// --------------------- Helper Soil for Control --------------
int getSoilPercentForControl() {
  RemoteSoilSnapshot s = getRemoteSoilSnapshot();

  if (s.has) {
    unsigned long age = millis() - s.rxMs;
    if (age <= SOIL_RX_TIMEOUT_MS && s.ok == 1) {
      return constrain(s.pct, 0, 100);
    }
  }
  return 100;
}

bool isRemoteSoilHealthy() {
  RemoteSoilSnapshot s = getRemoteSoilSnapshot();

  if (!s.has) return false;
  if ((millis() - s.rxMs) > SOIL_RX_TIMEOUT_MS) return false;
  if (s.ok != 1) return false;
  return true;
}

bool isRemoteLuxHealthy() {
  if (!hasRemoteLux) return false;
  if ((millis() - lastLuxRxMs) > LUX_RX_TIMEOUT_MS) return false;
  if (remoteLuxOk != 1) return false;
  if (isnan(remoteLux) || isinf(remoteLux)) return false;
  if (remoteLux < LUX_MIN_VALID || remoteLux > LUX_MAX_VALID) return false;
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
    Serial.println("[AutoTune] Pulse request blocked by anti-chatter/min-off");
    return;
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
  isLuxValid = isRemoteLuxHealthy() || (bhReady && !isnan(lux) && !isinf(lux) && lux >= LUX_MIN_VALID && lux <= LUX_MAX_VALID);
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
    Serial.print("RemoteSoilPct: "); Serial.println(s.pct);
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

// ------------------ Compare MAC Address Node A ------------------------
bool isSameMac(const uint8_t* a, const uint8_t* b) {
  for (int i = 0; i < 6; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

// -------------- ESP-NOW receive callback (ESP32 core ใหม่) -----------------------
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data) return;

  // รับเฉพาะจาก MAC ของ Node A ที่กำหนดไว้
  if (!isSameMac(info->src_addr, ALLOWED_SENDER_MAC)) return;

  // ---------- SoilPacket ----------
  if (len == (int)sizeof(SoilPacket)) {
    SoilPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    // validate แบบง่าย
    if (pkt.soilPct < 0 || pkt.soilPct > 100) return;

    remoteSoilPct = pkt.soilPct;
    remoteSeq = pkt.seq;
    remoteSensorOk = pkt.sensorOk;
    hasRemoteSoil = true;
    lastSoilRxMs = millis();
    return;
  }

  // ---- LuxPacket ----
  if (len == (int)sizeof(LuxPacket)) {
    LuxPacket lp;
    memcpy(&lp, data, sizeof(lp));

    bool luxOk = (lp.sensorOk == 1) &&
                 !isnan(lp.lux) && !isinf(lp.lux) &&
                 (lp.lux >= LUX_MIN_VALID) && (lp.lux <= LUX_MAX_VALID);

    remoteLux = lp.lux;
    remoteLuxSeq = lp.seq;
    remoteLuxOk = luxOk ? 1 : 0;
    hasRemoteLux = true;
    lastLuxRxMs = millis();
    return;
  }
}


// init ESP-NOW receiver on Control Unit (Node B)
bool initEspNowReceiver() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    return false;
  }

  if (esp_now_register_recv_cb(onEspNowRecv) != ESP_OK) {
    Serial.println("[ESP-NOW] register recv cb failed");
    return false;
  }

  Serial.println("[ESP-NOW] Receiver ready");
  return true;
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

  // Sensors
  bhReady = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  Serial.println(bhReady ? "BH1750 Ready!" : "Error initializing BH1750");

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

  // Init ESP-NOW receiver (Node B)
  if (!initEspNowReceiver()) {
    Serial.println("[ESP-NOW] Receiver init error");
  }

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