#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <time.h>
#include <Ticker.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Preferences.h> // [เพิ่ม] สำหรับบันทึกค่าลง Flash Memory
#include <RTClib.h>
#include <AsyncMqttClient.h>

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

Ticker blinker;

// Variables
float SUN_FACTOR = 0.0185; 
float LIGHT_FACTOR = 0.0135; 
float target_DLI = 12.0; 
float current_DLI = 0.0;
bool isLightOn = false;

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

bool isSceneShown = false;

volatile bool alarmTriggered = false;
bool rtcFound = false;
unsigned long lastNetworkCheck = 0; // สำหรับเช็คเน็ตโดยไม่บล๊อกการทำงานหลัก
unsigned long lastCalcUpdate = 0; // สำหรับคำนวณ DLI และรดน้ำ
unsigned long lastDliMillis = 0;
unsigned long lastDliSave = 0; // จับเวลาบันทึก DLI
int lastResetDayKey = -1;
unsigned long lastTimeResyncAttempt = 0;
unsigned long lastTelemetry = 0;

unsigned long lastReconnectAttempt = 0;
bool wasWifiConnected = false;

bool isTimeSynced = false;

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

unsigned long lastScreenUpdate = 0;

// ----------------------- Relay Control Function -----------------
void setRelayState(int pin, bool active){
  pinMode(pin, OUTPUT);
  if(active){
    // Active LOW สั่งเปิด
    digitalWrite(pin, LOW);
  }else{
    // Active HIGH สั่งปิด
    digitalWrite(pin, HIGH);
  }
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
  sprite.drawString("Lux", 10, 40);
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

// --------------------- Callback Function ------------------------------
void callback(char* topic, byte* payload, unsigned int length){
  Serial.print("Message [");
  Serial.print(topic);
  Serial.print("]");

  String msg = "";
  for (int i = 0; i < length; i++){
    msg += (char)payload[i];
  }
  msg.trim();
  Serial.print(msg);

  // สั่งงานผ่านมือถือ (Topic: group8/command)
  if(String(topic) == mqtt_topic_cmd){
  // ควบคุมน้ำเอง
    if(msg == "VALVE_MANUAL"){ isValveManual = true; Serial.println("SET MODE: VALVE MANUAL");}
    else if(msg == "VALVE_AUTO"){ isValveManual = false; Serial.println("SET MODE: VALVE AUTO");}
    // Action Control ทำงานตอนอยู๋ในโหมด Manual เท่านั้น
    else if(msg == "VALVE_ON"){ 
      if(isValveManual){
        setRelayState(RELAY_VALVE_MAIN, true); 
        isValveMainOn = true; 
        Serial.println("MANUAL: VALVE ON"); 
      } else{
        Serial.println("System is in AUTO Mode!!");
      }
    }
    else if(msg == "VALVE_OFF"){ 
      if(isValveManual){
        setRelayState(RELAY_VALVE_MAIN, false); 
        isValveMainOn = false; 
        Serial.println("MANUAL: VALVE OFF"); 
      }
      else{
        Serial.println("System is in AUTO Mode!!");
      }
    }
  // ควบคุมไฟเอง
    else if(msg == "LIGHT_MANUAL"){ isLightManual = true; Serial.println("SET MODE: LIGHT MANUAL"); }
    else if(msg == "LIGHT_AUTO"){ isLightManual = false; Serial.println("SET MODE: LIGHT AUTO"); }
    // Action Control ทำงานได้เฉพาะอยู่ในโหมด manual เท่านั้น
    else if(msg == "LIGHT_ON"){ 
      if(isLightManual){
        setRelayState(RELAY_LIGHT, true); 
        isLightOn = true; 
        Serial.println("MANUAL: LIGHT ON");
      }
      else{
        Serial.println("System is in AUTO Mode!!");
      }
    }else if(msg == "LIGHT_OFF"){
      if(isLightManual){ 
        setRelayState(RELAY_LIGHT, false); 
        isLightOn = false; 
        Serial.println("MANUAL: LIGHT OFF"); 
      }
      else{
        Serial.println("System is in AUTO Mode!!");
      }
    }
  }
  struct tm timeinfo;
  int h = 0, m = 0;
  if(getLocalTime(&timeinfo, 0)){ h = timeinfo.tm_hour; m = timeinfo.tm_min; }
  
  // ไม่เรียก calculate() เพื่อป้องกันการกระชาก แต่สั่งวาดจอได้เลย
  updateDisplay_Buffered(h, m);
  Serial.println(" [Instant Update Triggered!]");
}

// --------------------- Time & Alarm ---------------------------------------
void syncTime(){
  if(wifiMulti.run() == WL_CONNECTED){
    configTime(25200, 0, "pool.ntp.org");
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 2000)) { // รอ max 2 วิ
      Serial.println("[Time] NTP Sync Success!");
      // อัปเดตลง RTC ทันที เพื่อให้ RTC แม่นยำเสมอ
      if(rtcFound) {
        rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
        Serial.println("[Time] RTC Updated from NTP.");
      }
      return;
    }
  }

  // 2. ถ้า NTP พลาด ให้ดึงจาก RTC มาใส่ ESP32
  if (rtcFound) {
    DateTime now = rtc.now();
    struct timeval tv = { .tv_sec = now.unixtime(), .tv_usec = 0 };
    settimeofday(&tv, NULL);
    Serial.println("[Time] Recovered time from RTC (Offline Mode).");
  } else {
    Serial.println("[Time] Error: No Time Source!");
  }
}

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
  const long gmtOffset_sec = 25200;          
  const int daylightOffset_sec = 0;          
  struct tm timeinfo;

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Waiting for time syncing...");

  if(getLocalTime(&timeinfo, 2000)){
    Serial.println("[Time] NTP Sync Success!");
    if(rtcFound){
      rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
      Serial.println("[Time] Updated RTC with Internet time.");
    }
  }
  else if(rtcFound){
    Serial.println("[Time] NTP Failed. Using RTC time.");
    DateTime now = rtc.now();

    time_t rtc_time = now.unixtime();
    if(rtc_time > 25200) {
        rtc_time -= 25200; 
    }

    struct timeval tv = { .tv_sec = rtc_time, .tv_usec = 0 };
    settimeofday(&tv, NULL); // ตั้งเวลาเข้าระบบ ESP32
    isTimeSynced = true;
    Serial.println("[Time] System time set from RTC.");

    // if(rtc.lostPower()){
    //   Serial.println("RTC lost power!");
    //   // ถ้า RTC แบตหมดจริงๆ ค่ามันจะเพี้ยน เราอาจจะต้องตั้งค่าเริ่มต้นมั่วๆ ไปก่อนกันแฮงค์
    //   rtc.adjust(DateTime(2026, 1, 1, 12, 0, 0));
    // }
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
  if(millis() - lastNetworkCheck > 5000){
    lastNetworkCheck = millis();

    if(wifiMulti.run() != WL_CONNECTED){ Serial.println("WiFi lost... reconnecting"); }
    else if(!client.connected()){
      Serial.print("Attemting MQTT connection");
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
    if(wifiMulti.run() == WL_CONNECTED){
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

    // รีเซ็ตค่าใน NVS ด้วย
    preferences.putFloat("dli", 0.0);

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

  String statusMsg = "V:" + String(isValveManual ? "MAN" : "AUTO") + " | L:" + String(isLightManual ? "MAN" : "AUTO");
  client.publish(topic_status, statusMsg.c_str());
  
  Serial.println("[MQTT] Telemetry Sent >>");
}

// --------------------- คำนวณระบบ DLI & SOIL -------------------------------
void calculate(int currentHour, int currentMin){
  // LIGHT SYSTEM
  float lux = lightMeter.readLightLevel();
  bool wantLightOn = isLightOn;

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

    float factor_used = isLightOn ? LIGHT_FACTOR : SUN_FACTOR;
    float ppfd = lux * factor_used;
    current_DLI += (ppfd * dtSeconds) / 1000000.0f;

    // บันทึก DLI ลง Flash ทุก 1 นาที กันค่าหายเมื่อไฟดับ 
    if(millis() - lastDliSave > 60000UL) { // 900,000ms = 15 นาที
      lastDliSave = millis();
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
      // ถ้าไม่ได้อยู่ในช่วงเช้าและเย็น
      if(soilPercent < SOIL_CRITICAL){
        isEmergencyMode = true;
        setRelayState(RELAY_VALVE_MAIN, true);
        isValveMainOn = true;
      }
      else if (isEmergencyMode && soilPercent > SOIL_MIN) {
        // ออกจากโหมดฉุกเฉินเมื่อดินเริ่มชื้น (ถึงจุด SOIL_MIN)
        isEmergencyMode = false;
        setRelayState(RELAY_VALVE_MAIN, false);
        isValveMainOn = false;
      }
    else if (!isEmergencyMode) {
      // Normal Auto Logic
      isMorning = (currentTimeMins >= morningStartMins && currentTimeMins < morningEndMins);
      isEvening = (currentTimeMins >= eveningStartMins && currentTimeMins < eveningEndMins);

      if(!isMorning && !isEvening){
        setRelayState(RELAY_VALVE_MAIN, false);
        isValveMainOn = false;
      }
      // ถ้าอยู่ในช่วงเช้าและเย็น
      else{
        bool *currentDoneFlag = isMorning ? &isMorningDone : &isEveningDone;
        // เช็คว่าจบการทำงานหรือยัง และแดดปลอดภัยมั้ย
        if(!(*currentDoneFlag) && lux < LUX_SAFE_LIMIT){
          // ถ้าดินชุ่มถึงเป้าหมายแล้ว (80%) -> จบงานทันที
          if (soilPercent >= SOIL_MAX) {
            setRelayState(RELAY_VALVE_MAIN, false); // ปิดวาล์ว
            isValveMainOn = false;
            *currentDoneFlag = true; // ระบบจะไม่รดน้ำอีกต่อไปจนกว่าจะเปลี่ยนรอบ
            if(isMorning) preferences.putBool("mDone", true); // จำว่าเช้าเสร็จแล้ว
            if(isEvening) preferences.putBool("eDone", true); // จำว่าเย็นเสร็จแล้ว
            Serial.println("[Memory] Saved: Watering Done for this period.");
          }
          // ถ้าดินแห้งต่ำว่า 40% -> เติมน้ำ
          else if(soilPercent <= SOIL_MIN){
            setRelayState(RELAY_VALVE_MAIN, true);
            isValveMainOn = true;
          }
          // ระบบนี้จะทำงานอยู่ในช่วง 41% - 79%
        }
        // รดน้ำเสร็จแล้ว หรือแดดแรงเกิน -> ปิดวาล์ว
        else{
          setRelayState(RELAY_VALVE_MAIN, false);
          isValveMainOn = false;
        }
      }
    }
  }

  // ------------------------ Debug Monitor ------------------------
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

void setup() {
  Serial.begin(115200);
  Serial.println("--- System Starting ---");
  Wire.begin(SDA, SCL);

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

  setRelayState(RELAY_LIGHT, false);
  setRelayState(RELAY_VALVE_MAIN, false);
  pinMode(SOIL_PIN, INPUT);

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)){ Serial.println(F("BH1750 Ready!")); } 
  else { Serial.print(F("Error initialising BH1750!!")); }

  // ต้องบอก MQTT Client ก่อนว่าจะไปที่ Server ไหน
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback); // และบอกว่าถ้ามีข้อความมา ให้ไปเรียกฟังก์ชัน callback
  

  // Connect Network
  // ตั้งค่า Time Server ไว้รอ (มันจะ sync เองใน background เมื่อเน็ตมา)
  configTime(25200, 0, "pool.ntp.org"); // UTC+7 = 25200 sec
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
  bool wifiConnected = false;

  while(millis() - startAttempt < 15000){
    if(wifiMulti.run() == WL_CONNECTED){
      wifiConnected = true; 
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

  if(millis() - lastCalcUpdate > 200){
    lastCalcUpdate = millis();

    struct tm timeinfo;
    int h = 0;
    int m = 0;
  
    // ดึงเวลาจาก System Clock (ซึ่งถูกตั้งค่าโดย timezoneSync แล้ว ไม่ว่าจากแหล่งไหน)
    bool validTime = getLocalTime(&timeinfo, 0);

    if(validTime){
      h = timeinfo.tm_hour;
      m = timeinfo.tm_min;
      handleDailyReset(timeinfo);
    }else{
      Serial.println("Time Error: System waits for time sync...");
      h = 12;
    }
    calculate(h, m);
    updateDisplay_Buffered(h, m);
  }

  // 2. Telemetry (ส่งข้อมูลเข้าเน็ต ทุกๆ 10 วินาที)
  // -----------------------------------------------------------
  if(millis() - lastTelemetry > 10000){ // 1000ms = 10 วินาที
    lastTelemetry = millis();

    // ต้องอ่านค่าแสงตรงนี้เพื่อส่ง (เพราะเราแยกส่วนกันแล้ว)
    float currentLux = lightMeter.readLightLevel(); 
    reportTelemetry(currentLux);
    Serial.println("[MQTT] Telemetry Sent");
  }
}