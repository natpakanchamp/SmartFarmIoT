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
#include <Preferences.h> // [เพิ่ม] สำหรับบันทึกค่าลง Flash Memory
#include <RTClib.h>
#include <AsyncMqttClient.h>
#include <math.h>

#define RELAY_LIGHT 4 // แก้ขา Pin ตามที่ใช้งานจริง
#define SOIL_PIN 34  // ขาอ่านค่าความชื้นดิน (ADC1 Only)
#define RELAY_VALVE_MAIN 16 // Soleniod valve ตัวที่ 1 สำหรับเปิดปิดพ่นน้ำ
#define SQW_PIN 27

// I2C
#define SDA 21
#define SCL 22

// Soil Moisture
const int AIR_VALUE = 2730;
const int WATER_VALUE = 970;

// Solenoind valve 1 สำหรับเปิดปิดพ่นน้ำ
const int SOIL_MIN = 40;
const int SOIL_MAX = 80;
const int SOIL_CRITICAL = 20;
const int LUX_SAFE_LIMIT = 30000; // ค่าแสงแดดช่วงเช้าอ่อนๆ

const int MORNING_START = 6;
const int MORNING_END = 8;
const int EVENING_START = 17;
const int EVENING_END = 19;


// Objects
BH1750 lightMeter;
WiFiMulti wifiMulti;
WiFiClient espClient;
PubSubClient client(espClient);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft); // Sprite แก้จอกระพริบ 
Preferences preferences; // ตัวจัดการหน่วยความจำถาวร
RTC_DS3231 rtc;

// Variables
float SUN_FACTOR = 0.0185; 
float LIGHT_FACTOR = 0.0135; 
float target_DLI = 12.0; 
float current_DLI = 0.0;
bool isLightOn = false;

// ---------- BH1750 retry throttle ----------
unsigned long lastBhRetryMs = 0;
const unsigned long BH_RETRY_INTERVAL = 3000UL; // ลอง init ใหม่ทุก 3 วิ เมื่อไม่พร้อม

// ---------- Valve anti-chatter ----------
unsigned long valveLastSwitchMs = 0;
const unsigned long VALVE_MIN_ON_MS  = 15000UL; // เปิดอย่างน้อย 15 วิ
const unsigned long VALVE_MIN_OFF_MS = 30000UL; // ปิดอย่างน้อย 30 วิ

// -------- Lux Filtering & Validation --------
float luxSmoothed = 0.0f;
const float luxAlpha = 0.20f;          // EMA ของแสง (0.1-0.3 กำลังดี)
bool isLuxValid = false;
float lastValidLux = 0.0f;
unsigned long lastValidLuxMs = 0;
const float LUX_MIN_VALID = 0.0f;
const float LUX_MAX_VALID = 120000.0f; // BH1750 ใช้งานจริงสูงราวนี้
bool bhReady = false;

// Soil Filtering Variable
float soilSmoothed = 0;   // ค่าความชื้นที่ผ่านการกรองแล้ว
float alpha = 0.1;        // ค่าน้ำหนัก EMA (0.1 นุ่มนวล, 1.0 ดิบ)
int soilPercent = 0;      // %

bool isValveMainOn = false;
bool isMorningDone = false;
bool isEveningDone = false;

// [Manual Modes] แยกกันอิสระ
bool isValveManual = false; 
bool isLightManual = false;
bool isEmergencyMode = false; // โหมดฉุกเฉิน

volatile bool alarmTriggered = false;
bool rtcFound = false;
unsigned long lastNetworkCheck = 0; // สำหรับเช็คเน็ตโดยไม่บล๊อกการทำงานหลัก
unsigned long lastCalcUpdate = 0; // สำหรับคำนวณ DLI และรดน้ำ
unsigned long lastDliMillis = 0;
unsigned long lastDliSave = 0; // จับเวลาบันทึก DLI
int lastResetDayKey = -1;
unsigned long lastTimeResyncAttempt = 0;
unsigned long lastTelemetry = 0;
unsigned long lastDebug = 0;        // รอบพิมพ์ debug

const unsigned long CONTROL_INTERVAL = 1000UL;   // 1 วินาที
const unsigned long NETWORK_INTERVAL = 2000UL;   // 2 วินาที
const unsigned long TELEMETRY_INTERVAL = 10000UL;// 10 วินาที
const unsigned long DEBUG_INTERVAL = 3000UL;     // 3 วินาที

const bool DEBUG_LOG = true;  // ถ้าร้อน/ช้า ให้เปลี่ยนเป็น false

bool wasWifiConnected = false;

bool isTimeSynced = false;
float lastSavedDLI = -1.0;

// --------------------- Interrupt ------------------------------------
void IRAM_ATTR onRTCAlarm(){
  alarmTriggered = true; // บอก ESP32 ว่าถึงเวลาทำงานแล้ว
}

// --------------------- WiFi Setting --------------------------------
// const char* ssid_1 = "@JumboPlusIoT";
//const char* pass_1 = "rebplhzu";

// const char* ssid_2 = "JumboPlus_DormIoT";
// const char* pass_2 = "rebplhzu";

const char* ssid_3 = "JebHuaJai";
const char* pass_3 = "ffffffff";

// --------------------- MQTT Config --------------------------------
// const char* mqtt_broker = "test.mosquitto.org";
// const char* mqtt_broker = "91.121.93.94";
const char* mqtt_broker = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_client_id = "Group8/lnwza555"; // คงค่าเดิม
const char* mqtt_topic_cmd = "group8/command";

const char* topic_status = "group8/status";
const char* topic_dli = "group8/dli";
const char* topic_soil = "group8/soil";
const char* topic_valve = "group8/valve/main";
const char* topic_lux = "group8/lux";

// ----------------------- Relay Control Function -----------------
void setRelayState(int pin, bool active){
  // Active LOW: true = ON, false = OFF
  digitalWrite(pin, active ? LOW : HIGH);
}

void setValveStateSafe(bool wantOn, bool force = false){
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

// --------------------- Sprite Display --------------------------------
void updateDisplay_Buffered(int h, int m){
  sprite.fillSprite(TFT_BLACK); // เคลียร์ Sprite เป็นสีดำก่อนวาดใหม่

  sprite.setTextDatum(TL_DATUM);

  // วาดข้อความ Static
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.setTextSize(3);
  sprite.drawString("Soil", 10, 10);
  sprite.drawString("%", 215, 10);
  sprite.drawString("DLI", 10, 40);
  sprite.drawLine(10, 71, 229, 71, TFT_WHITE);

  // วาดค่า Dynamic (ตัวเลข)
  sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
  sprite.setTextDatum(TR_DATUM);
  sprite.drawNumber(soilPercent, 200, 10);
  sprite.drawFloat(current_DLI, 2, 200, 40);

  // วาดสถานะ Valve/Light
  sprite.setTextSize(2);
  sprite.setTextDatum(TL_DATUM);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("VALVE", 49, 85);
  sprite.drawString("LIGHT", 164, 85);

  // วาดวงกลมสถานะ (ใช้สีตามเงื่อนไข)
  // Valve Mode
  uint16_t valveModeColor = isValveManual ? TFT_ORANGE : TFT_CYAN;
  sprite.fillEllipse(62, 122, 23, 23, valveModeColor);
  sprite.setTextColor(TFT_BLACK, valveModeColor);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextSize(1);
  sprite.drawString(isValveManual ? "MAN" : "AUTO", 62, 122);

  // Light Mode
  uint16_t lightModeColor = isLightManual ? TFT_ORANGE : TFT_CYAN;
  sprite.fillEllipse(177, 122, 23, 23, lightModeColor);
  sprite.setTextColor(TFT_BLACK, lightModeColor);
  sprite.drawString(isLightManual ? "MAN" : "AUTO", 177, 122);

  // Valve ON/OFF
  sprite.fillEllipse(34, 184, 20, 20, isValveMainOn ? TFT_GREEN : TFT_BLACK);
  sprite.fillEllipse(91, 184, 20, 20, isValveMainOn ? TFT_BLACK : TFT_RED);

  // Light ON/OFF
  sprite.fillEllipse(148, 184, 20, 20, isLightOn ? TFT_GREEN : TFT_BLACK);
  sprite.fillEllipse(205, 184, 20, 20, isLightOn ? TFT_BLACK : TFT_RED);

  // Labels
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

  // ดันข้อมูลจาก Sprite ขึ้นหน้าจอจริงทีเดียว (ไร้รอยต่อ)
  sprite.pushSprite(0, 0);
}

// --------------------- Command Helpers ------------------------------
// เปลี่ยนโหมด MAN/AUTO ของอุปกรณ์
void setManualMode(bool &modeRef, const char* name, bool manual){
  modeRef = manual;
  Serial.print("SET MODE: ");
  Serial.print(name);
  Serial.println(manual ? " MANUAL" : " AUTO");
}

// สั่ง ON/OFF เฉพาะเมื่ออยู่ Manual Mode
void applyManualAction(bool manualMode, int relayPin, bool &stateRef, const char* name, bool turnOn){
  if(manualMode){
    setRelayState(relayPin, turnOn);
    stateRef = turnOn;

    Serial.print("MANUAL: ");
    Serial.print(name);
    Serial.println(turnOn ? " ON" : " OFF");
  } else {
    Serial.println("System is in AUTO Mode!!");
  }
}

// --------------------- Callback Function ------------------------------
void callback(char* topic, byte* payload, unsigned int length){
  Serial.print("Message [");
  Serial.print(topic);
  Serial.print("]");

  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();
  Serial.print(msg);

  // สั่งงานผ่านมือถือ (Topic: group8/command)
  if(String(topic) == mqtt_topic_cmd){
  // ควบคุมน้ำเอง
    if(msg == "VALVE_MANUAL"){ setManualMode(isValveManual, "VALVE", true); }
    else if(msg == "VALVE_AUTO"){ setManualMode(isValveManual, "VALVE", false); }
    // Action Control ทำงานตอนอยู๋ในโหมด Manual เท่านั้น
    else if(msg == "VALVE_ON"){ 
      applyManualAction(isValveManual, RELAY_VALVE_MAIN, isValveMainOn, "VALVE", true);
    }
    else if(msg == "VALVE_OFF"){ 
      applyManualAction(isValveManual, RELAY_VALVE_MAIN, isValveMainOn, "VALVE", false);
    }
  // ควบคุมไฟเอง
    else if(msg == "LIGHT_MANUAL"){ setManualMode(isLightManual, "LIGHT", true);}
    else if(msg == "LIGHT_AUTO"){ setManualMode(isLightManual, "LIGHT", false); }
    // Action Control ทำงานได้เฉพาะอยู่ในโหมด manual เท่านั้น
    else if(msg == "LIGHT_ON"){ applyManualAction(isLightManual, RELAY_LIGHT, isLightOn, "LIGHT", true); }
    else if(msg == "LIGHT_OFF"){ applyManualAction(isLightManual, RELAY_LIGHT, isLightOn, "LIGHT", false); }
    else { Serial.println("Unknown command."); }
  }
  struct tm timeinfo;
  int h = 0, m = 0;
  if(getLocalTime(&timeinfo, 0)){ h = timeinfo.tm_hour; m = timeinfo.tm_min; }
  
  // ไม่เรียก calculate() เพื่อป้องกันการกระชาก แต่สั่งวาดจอได้เลย
  updateDisplay_Buffered(h, m);
  Serial.println(" [Instant Update Triggered!]");
}

// --------------------- Time & Alarm ---------------------------------------
// void syncTime(){
//   if(wifiMulti.run() == WL_CONNECTED){
//     configTime(0, 0, "pool.ntp.org");
//     struct tm timeinfo;

//     if (getLocalTime(&timeinfo, 2000)) { // รอ max 2 วิ
//       Serial.println("[Time] NTP Sync Success!");
//       // อัปเดตลง RTC ทันที เพื่อให้ RTC แม่นยำเสมอ

//       if(rtcFound) {
//         rtc.adjust(DateTime(
//           timeinfo.tm_year + 1900, 
//           timeinfo.tm_mon + 1, 
//           timeinfo.tm_mday, 
//           timeinfo.tm_hour, 
//           timeinfo.tm_min, 
//           timeinfo.tm_sec
//         ));
//         Serial.println("[Time] RTC Updated from NTP.");
//       }
//       return;
//     }
//   }

//   // 2. ถ้า NTP พลาด ให้ดึงจาก RTC มาใส่ ESP32
//   if (rtcFound) {
//     DateTime now = rtc.now();
//     time_t rtc_utc_epoch = now.unixtime() - 7 * 3600;
//     struct timeval tv = { .tv_sec = rtc_utc_epoch, .tv_usec = 0 };
//     settimeofday(&tv, NULL);
//     Serial.println("[Time] Recovered time from RTC (Offline Mode).");
//   } else {
//     Serial.println("[Time] Error: No Time Source!");
//   }
// }

void setupRTCAlarm() {
  if (!rtcFound) return;
  
  rtc.disable32K(); // ปิดขา 32K เพื่อประหยัดไฟ
  rtc.clearAlarm(1); // เคลียร์ Alarm เก่า
  rtc.clearAlarm(2);
  
  // ปิด Alarm 2
  rtc.writeSqwPinMode(DS3231_OFF); // ปิดโหมด Square Wave ปกติก่อน
  
  // **ตั้ง Alarm 1 ให้ทำงานเมื่อ "วินาทีเท่ากับ 00" (คือทำงานทุกๆ นาที)**
  // DS3231_A1_Second = Match seconds only (Every minute at XX:XX:00)
  if (!rtc.setAlarm1(rtc.now(), DS3231_A1_Second)) {
    Serial.println("Error setting alarm 1!");
  }
  
  // เปิดใช้งาน Interrupt ขา SQW
  // rtc.writeSqwPinMode(DS3231_USE_ALARM); // บาง Library ใช้อันนี้
  // สำหรับ RTClib เวอร์ชันใหม่ๆ มันจัดการให้ตอน setAlarm แต่เพื่อความชัวร์:
  Wire.beginTransmission(0x68);
  Wire.write(0x0E); // Control Register
  Wire.write(0b00000101); // INTCN=1, A1IE=1 (Enable Interrupt & Alarm 1)
  Wire.endTransmission();
  
  Serial.println("[RTC] Alarm set to trigger every minute at :00");
}

// --------------------- Time Zone Setting (UTC+7) ---------------------------
void timezoneSync(){
  const char* ntpServer = "pool.ntp.org";        
  struct tm timeinfo;

  // UTC core
  configTime(0, 0, ntpServer);
  Serial.println("Waiting for time syncing...");

  if(getLocalTime(&timeinfo, 2000)){
    Serial.println("[Time] NTP Sync Success!");
    if(rtcFound){
      rtc.adjust(DateTime(
        timeinfo.tm_year + 1900, 
        timeinfo.tm_mon + 1, 
        timeinfo.tm_mday, 
        timeinfo.tm_hour, 
        timeinfo.tm_min, 
        timeinfo.tm_sec
      ));
      isTimeSynced = true;
      Serial.println("[Time] Updated RTC with Internet time.");
    }
  }
  else if(rtcFound){
    Serial.println("[Time] NTP Failed. Using RTC time.");
    DateTime now = rtc.now();

    // RTC เก็บ local(+7) -> แปลงเป็น UTC ก่อน settimeofday
    time_t rtc_utc_epoch = now.unixtime() - 7 * 3600;
    struct timeval tv = { .tv_sec = rtc_utc_epoch, .tv_usec = 0 };
    settimeofday(&tv, NULL); // ตั้งเวลาเข้าระบบ ESP32

    isTimeSynced = true;
    Serial.println("[Time] System time set from RTC.");
  }
  else {
    Serial.println("[Time] Critical: No Time Source!");
    isTimeSynced = false;
  }
}

// --------------------- Handle Network -------------------------------------
void handleNetwork(){
  bool isWifiConnected = (wifiMulti.run() == WL_CONNECTED);

  // เช็คว่า "เพิ่งจะ" ต่อเน็ตติดหรือไม่? (Transition from Disconnect -> Connect)
  if(isWifiConnected && !wasWifiConnected){
    Serial.println("[Network] Reconnected! Syncing Time immediately...");
    timezoneSync(); // <--- ซิงค์ทันทีเมื่อเน็ตกลับมา!
  }
  // อัปเดตสถานะล่าสุดเก็บไว้เทียบรอบหน้า
  wasWifiConnected = isWifiConnected;

  // เช็คสถานะ MQTT
  if(millis() - lastNetworkCheck > NETWORK_INTERVAL){
    lastNetworkCheck = millis();

    if(!isWifiConnected){ Serial.println("WiFi lost... reconnecting"); }
    else if(!client.connected()){
      Serial.print("Attempting MQTT connection");
      String clientId = String(mqtt_client_id) + "-" + String(random(0xffff), HEX);

      if (client.connect(clientId.c_str())) {
        Serial.println("Connected!");
        client.subscribe(mqtt_topic_cmd);
        client.publish(topic_status, "SYSTEM RECOVERED");
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" (will try again in 5s)");
      }
    }
    // Auto Sync
    if(isWifiConnected){
      if(millis() - lastTimeResyncAttempt > 3600000UL){ // ทุก 1 ชม
        lastTimeResyncAttempt = millis();
        timezoneSync();
      }
    }
  }
}

void handleDailyReset(struct tm timeinfo){
  int dayKey = timeinfo.tm_yday;
  if (lastResetDayKey == -1) {
     lastResetDayKey = preferences.getInt("savedDay", -1);
  }
  if(dayKey != lastResetDayKey){
    lastResetDayKey = dayKey;
    preferences.putInt("savedDay", dayKey);

    current_DLI = 0.0;

    isMorningDone = false;
    isEveningDone = false;

    // *** ล้างค่าใน Flash Memory ด้วย ***
    preferences.putFloat("dli", 0.0);
    preferences.putBool("mDone", false);
    preferences.putBool("eDone", false);

    Serial.println("[Daily Reset] Cleared DLI");
  }
}

void reportTelemetry(float lux){
  if(!client.connected()) return;
  char msg[50];
  sprintf(msg, "%.2f", current_DLI);  client.publish(topic_dli, msg);
  sprintf(msg, "%d", soilPercent);    client.publish(topic_soil, msg);
  sprintf(msg, "%.2f", lux);          client.publish(topic_lux, msg);

  client.publish(topic_valve, isValveMainOn ? "ON" : "OFF", true); // retain=true

  String statusMsg = "V:" + String(isValveManual ? "MAN" : "AUTO") + " | L:" + String(isLightManual ? "MAN" : "AUTO");
  client.publish(topic_status, statusMsg.c_str());
  
  Serial.println("[MQTT] Telemetry Sent >>");
}
// ------------------------ อ่านค่า LUX ---------------------------------------------------
float readLuxSafe() {
  unsigned long now = millis();

  if (!bhReady) {
    if (now - lastBhRetryMs >= BH_RETRY_INTERVAL) {
      lastBhRetryMs = now;
      bhReady = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
      if (bhReady) {
        Serial.println("[BH1750] Recovered and configured.");
        luxSmoothed = 0.0f;
        isLuxValid = false;
        lastValidLux = 0.0f;
        lastValidLuxMs = 0;
      } else {
        Serial.println("[BH1750] not ready, re-init failed");
      }
    }
    return (float)LUX_SAFE_LIMIT + 1000.0f;
  }

  float raw = lightMeter.readLightLevel();
  bool valid = !isnan(raw) && !isinf(raw) && (raw >= LUX_MIN_VALID) && (raw <= LUX_MAX_VALID);

  if (!valid) {
    isLuxValid = false;
    bhReady = false; // ให้ไปรอบ retry throttle
    Serial.println("[Alarm] Lux Sensor Fault/Out-of-range!");

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

// --------------------- คำนวณระบบ DLI & SOIL -------------------------------
void calculate(int currentHour, int currentMin){
  // LIGHT SYSTEM
  float lux = readLuxSafe();
  bool wantLightOn = isLightOn;
  static unsigned long wifiLostTime = 0;
  // ---------- ป้องกันเมื่อเวลาเน็ตหลุดแล้วยังอยู่ในโหมด Manual -------------------------
  if(!client.connected()){
     if(wifiLostTime == 0) wifiLostTime = millis();
     
     if(millis() - wifiLostTime > 30000UL){ // ถ้าหลุดเกิน 30 วิ
        isValveManual = false;
        isLightManual = false;
        // Serial.println("[Fail-safe] Connection lost: Reverting to AUTO mode");
     }
  } else {
     wifiLostTime = 0; // รีเซ็ตตัวจับเวลาเมื่อเน็ตกลับมา
  }

  // --------------- Light Control -------------------------
  if(isLightManual){
    isLightOn = (digitalRead(RELAY_LIGHT) == LOW);
    wantLightOn = isLightOn;
  }
  else {
    wantLightOn = false;
    // เปิดไฟเฉพาะช่วง 18:00 - 23:59 และ DLI ยังไม่ถึงเป้า
    if(currentHour >= 18 && currentHour <= 23 && current_DLI < target_DLI){ wantLightOn = true; }
    }
    // สั่ง Relay (ตรวจสอบสถานะก่อนสั่งเพื่อลด Overhead)
    if(isLightOn != wantLightOn){
        setRelayState(RELAY_LIGHT, wantLightOn);
        isLightOn = wantLightOn;
    }

    // ---------- DLI time Integration --------------
    unsigned long nowMs = millis();
    if(lastDliMillis == 0) lastDliMillis = nowMs; 
    float dtSeconds = (nowMs - lastDliMillis) / 1000.0f;
    lastDliMillis = nowMs;
    // ป้องกันไม่ให้ dt มากผิดปกติ ไม่งั้น DLI ค่าจะกระโด
    if(dtSeconds < 0) dtSeconds = 0;
    if(dtSeconds > 5.0f) dtSeconds = 5.0f;

      // ถ้า lux invalid และไฟปลูกไม่ได้เปิด -> ไม่ควรสะสม DLI จากค่าเพี้ยน
    if (isLuxValid || isLightOn) {
    float factor_used = isLightOn ? LIGHT_FACTOR : SUN_FACTOR;
    float ppfd = lux * factor_used;
    current_DLI += (ppfd * dtSeconds) / 1000000.0f;
    }

    // บันทึก DLI ลง Flash ทุก 15 นาที กันค่าหายเมื่อไฟดับ 
    bool timeToSave = (nowMs - lastDliSave > 900000UL);
    bool thresholdReached = (fabsf(current_DLI - lastSavedDLI) >= 0.5f);

    if(timeToSave || thresholdReached) { // 900,000ms = 15 นาที
      lastDliSave = nowMs;
      lastSavedDLI = current_DLI; // อัปเดตตัวแปรอ้างอิง
      preferences.putFloat("dli", current_DLI);
      Serial.println("[System] DLI Saved to NVS");
    }

    Serial.print("Current DLI: ");
    Serial.println(current_DLI);

    // -------------- Soil Moisture --------------------------
    int rawSoil = analogRead(SOIL_PIN);

    // Fail-Safe Check: ตรวจสอบสายขาดหรือช็อต
    if(rawSoil > 3500 || rawSoil < 100) {
      // Sensor Fault! เข้าโหมดปลอดภัย (อาจจะหยุดรดน้ำ หรือแจ้งเตือน)
      Serial.println("[Alarm] Soil Sensor Fault Detected!");
      // ในที่นี้อาจจะใช้ค่าเดิมไปก่อน หรือตั้งเป็น 100% เพื่อไม่ให้รดน้ำมั่ว
      soilPercent = 100;
    } else {
      // EMA Filter: ลดสัญญาณรบกวน 
      if(soilSmoothed == 0) soilSmoothed = rawSoil; // ค่าเริ่มต้น
      soilSmoothed = (alpha * rawSoil) + ((1 - alpha) * soilSmoothed);

      soilPercent = map((int)soilSmoothed, AIR_VALUE, WATER_VALUE, 0, 100);
      soilPercent = constrain(soilPercent, 0, 100); // 0 - 100 %
    }
    // --------------- Time windows --------------------------
    int currentTimeMins = (currentHour * 60) + currentMin;
    // กำหนดช่วงเวลา (แปลงเป็นนาที)
    // ตัวอย่าง: ถ้า MORNING_START = 6 คือ 6*60 = 360 นาที (06:00)
    int morningStartMins = MORNING_START * 60; 
    int morningEndMins   = MORNING_END * 60;   
    int eveningStartMins = EVENING_START * 60;
    int eveningEndMins   = EVENING_END * 60;

    // ----------------- Valve Control -----------------
    if(isValveManual){
      isValveMainOn = (digitalRead(RELAY_VALVE_MAIN) == LOW);
    }else{
      // WATER SYSTEM
      bool isMorning = (currentTimeMins >= morningStartMins && currentTimeMins < morningEndMins);
      bool isEvening = (currentTimeMins >= eveningStartMins && currentTimeMins < eveningEndMins);

      if(soilPercent < SOIL_CRITICAL){
        isEmergencyMode = true;
        setValveStateSafe(true, true);
      }
      else if (isEmergencyMode && soilPercent > SOIL_MIN) {
        // ออกจากโหมดฉุกเฉินเมื่อดินเริ่มชื้น
        isEmergencyMode = false;
        setValveStateSafe(false);
      }
      else if (!isEmergencyMode) {
        if(!isMorning && !isEvening){
          setValveStateSafe(false);
        }
        else{
          bool *currentDoneFlag = isMorning ? &isMorningDone : &isEveningDone;

          if(!(*currentDoneFlag) && lux < LUX_SAFE_LIMIT){
            if (soilPercent >= SOIL_MAX) {
              setValveStateSafe(false);
              *currentDoneFlag = true;

              if(isMorning){
                isMorningDone = true;
                preferences.putBool("mDone", true);
                Serial.println("[Memory] Moorning Task Saved.");
              }
              if (isEvening && !isEveningDone) {
                isEveningDone = true;
                preferences.putBool("eDone", true);
                Serial.println("[Memory] Evening Task Saved.");
              }
              Serial.println("[Memory] Saved: Watering Done for this period.");
            }
            else if(soilPercent <= SOIL_MIN){
              setValveStateSafe(true);
            }
          }
          else{
            setValveStateSafe(false);
          }
        }
      }
    }

    // ------------------------ Debug Monitor ------------------------
  if (DEBUG_LOG && (millis() - lastDebug >= DEBUG_INTERVAL)) {
    lastDebug = millis();

    Serial.println("--- Status ---");
    Serial.print(" | Lux: "); Serial.println(lux);
    Serial.print(" | DLI: "); Serial.println(current_DLI);

    Serial.print("Current Hour: "); Serial.println(currentHour);
    Serial.print(" | Soil: "); Serial.println(soilPercent);
    Serial.print(" | Light: "); Serial.println(isLightOn);
    Serial.print(" | Valve: "); Serial.println(isValveMainOn ? "ON" : "OFF");
    Serial.print(" | M-Done: "); Serial.println(isMorningDone);
    Serial.print(" | E-Done: "); Serial.println(isEveningDone);
  }
}

void setup() {
  Serial.begin(115200);
  setenv("TZ", "ICT-7", 1); // Thailand UTC+7
  tzset();
  Serial.println("--- System Starting ---");
  Wire.begin(SDA, SCL);

  bhReady = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  Serial.println(bhReady ? "BH1750 Ready!" : "Error initialising BH1750!!");

  // Init RTC
  if (!rtc.begin()) {
    Serial.println("Error: RTC not found!");
  } else {
    Serial.println("RTC Found.");
    rtcFound = true;
    if (rtc.lostPower()) Serial.println("RTC lost power, please sync time.");
  }

  // ตั้งค่า Interrupt ขา SQW
  pinMode(SQW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SQW_PIN), onRTCAlarm, FALLING);
  setupRTCAlarm(); // เรียกฟังก์ชันตั้งค่า Alarm

  // Init NVS (Flash Memory)
  preferences.begin("smartfarm", false); 
  current_DLI = preferences.getFloat("dli", 0.0); // ดึงค่าเดิมกลับมา
  Serial.print("Restored DLI: "); Serial.println(current_DLI);

  // ดึงค่าสถานะต่างๆ กลับมา
  isMorningDone = preferences.getBool("mDone", false);
  isEveningDone = preferences.getBool("eDone", false);

  // ดึงโหมด Manual กลับมา (ถ้าอยากให้จำ)
  // isValveManual = preferences.getBool("vMan", false);
  // isLightManual = preferences.getBool("lMan", false);

  tft.init();
  // tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // Init Sprite
  sprite.setColorDepth(8); // (ลดสีเหลือ 256 สี เพื่อประหยัดแรม)
  
  void *ptr = sprite.createSprite(240, 240); // สร้าง Sprite
  if (ptr == NULL) {
    Serial.println("ERROR: Not enough RAM for Sprite!"); 
    // ถ้ายังไม่พออีก ให้แจ้งเตือนผ่าน Serial
  } else {
    Serial.println("Sprite created successfully!");
  }

  pinMode(RELAY_LIGHT, OUTPUT);
  pinMode(RELAY_VALVE_MAIN, OUTPUT);
  pinMode(SOIL_PIN, INPUT);

  setRelayState(RELAY_LIGHT, false);
  setRelayState(RELAY_VALVE_MAIN, false);

  // ต้องบอก MQTT Client ก่อนว่าจะไปที่ Server ไหน
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback); // และบอกว่าถ้ามีข้อความมา ให้ไปเรียกฟังก์ชัน callback
  

  // Connect Network
  // ตั้งค่า Time Server ไว้รอ (มันจะ sync เองใน background เมื่อเน็ตมา)
  configTime(0, 0, "pool.ntp.org");
//  wifiMulti.addAP(ssid_1, pass_1);
//  wifiMulti.addAP(ssid_2, pass_2);
  wifiMulti.addAP(ssid_3, pass_3);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("CONNECTING WIFI...", 10, 10, 4);
  tft.drawString("BOOTING SYSTEM...", 10, 40, 4);
  Serial.print("Connecting WiFi...");

  Serial.println("System Ready: All Relays OFF");
  delay(1000);

  unsigned long startAttempt = millis();

  while(millis() - startAttempt < 15000){
    if(wifiMulti.run() == WL_CONNECTED){
      break;
    }
    Serial.print(".");
    delay(500);
  }

  tft.fillScreen(TFT_BLACK);
  tft.drawString("SYNCING TIME...", 10, 10, 4);
  timezoneSync();

  updateDisplay_Buffered(12, 00);
  //Serial.println("Setup Done -> Entering Main Loop");
}

// -------------- Safe mode when time not valid --------------------------
void handleNoValidTimeSafeMode() {
  // ถ้าไม่ได้อยู่ manual ให้เข้าสภาวะปลอดภัย
  if (!isValveManual) {
    setValveStateSafe(false);
  }
  if (!isLightManual) {
    setRelayState(RELAY_LIGHT, false);
    isLightOn = false;
  }

  // ไม่แตะ current_DLI และไม่ทำ schedule/ช่วงเวลาใดๆ
  Serial.println("[Time] Invalid system time -> Automation paused (safe mode).");
}

void loop() {
  handleNetwork();

  if(client.connected()){ client.loop(); }

  // Check ว่า SQW ปลุกมาหรือไม่? (ทำงานทุก 1 นาที)
  if (alarmTriggered) {
    alarmTriggered = false; // เอาธงลง
    rtc.clearAlarm(1);      // เคลียร์ Alarm ในชิป RTC เพื่อให้มันปลุกรอบหน้าได้อีก
    Serial.println("[SQW] Minute Tick Triggered");
    // ตรงนี้สามารถใส่ Logic ที่อยากให้ทำทุกๆ "นาทีเป๊ะๆ" ได้
  }

  if(millis() - lastCalcUpdate > CONTROL_INTERVAL){
    lastCalcUpdate = millis();

    struct tm timeinfo;
    int h = 12;
    int m = 0;
  
    // ดึงเวลาจาก System Clock (ซึ่งถูกตั้งค่าโดย timezoneSync แล้ว ไม่ว่าจากแหล่งไหน)
    bool validTime = getLocalTime(&timeinfo, 0);

    if(validTime){
      h = timeinfo.tm_hour;
      m = timeinfo.tm_min;
      isTimeSynced = true;

      handleDailyReset(timeinfo);
      calculate(h, m);
    }else{
      Serial.println("Time Error: System waits for time sync...");
      isTimeSynced = false;
      handleNoValidTimeSafeMode();
    }
    updateDisplay_Buffered(h, m);
  }

  // Telemetry (ส่งข้อมูลเข้าเน็ต ทุกๆ 10 วินาที)
  // -----------------------------------------------------------
  if(millis() - lastTelemetry > TELEMETRY_INTERVAL){ // 10000ms = 10 วินาที
    lastTelemetry = millis();

    // ต้องอ่านค่าแสงตรงนี้เพื่อส่ง (เพราะเราแยกส่วนกันแล้ว)
    float currentLux = readLuxSafe();
    reportTelemetry(currentLux);
  }
}