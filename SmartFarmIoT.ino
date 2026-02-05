#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <time.h>
#include <Ticker.h>
#include <SPI.h>
#include <TFT_eSPI.h>

#define RELAY_LIGHT 4 // แก้ขา Pin ตามที่ใช้งานจริง
#define SOIL_PIN 34  // ขาอ่านค่าความชื้นดิน (ADC1 Only)
#define RELAY_VALVE_MAIN 16 // Soleniod valve ตัวที่ 1 สำหรับเปิดปิดพ่นน้ำ

// I2C
#define SDA 21
#define SCL 22

// Soil Moisture
const int AIR_VALUE = 2730;
const int WATER_VALUE = 970;

// Solenoind valve 1 สำหรับเปิดปิดพ่นน้ำ
const int SOIL_MIN = 40;
const int SOIL_MAX = 80;
const int LUX_SAFE_LIMIT = 30000; // ค่าแสงแดดช่วงเช้าอ่อนๆ

const int MORNING_START = 6;
const int MORNING_END = 8;
const int EVENING_START = 17;
const int EVENING_END = 19;


// Objects
BH1750 lightMeter;
WiFiMulti wifiMulti;
WiFiClient espClient;
unsigned long lastReconnectAttempt = 0;

PubSubClient client(espClient);
Ticker blinker;
TFT_eSPI tft = TFT_eSPI();

// Variables
float SUN_FACTOR = 0.0185; 
float LIGHT_FACTOR = 0.0135; 
float target_DLI = 12.0; 
float current_DLI = 0.0;
bool isLightOn = false;
int soilPercent = 0; // %
bool isValveMainOn = false;

bool isMorningDone = false;
bool isEveningDone = false;

// [Manual Modes] แยกกันอิสระ
bool isValveManual = false; 
bool isLightManual = false;

bool isSceneShown = false;



// --------------------- WiFi Setting --------------------------------
const char* ssid_1 = "@JumboPlusIoT";
const char* pass_1 = "rebplhzu";

const char* ssid_2 = "JumboPlus_DormIoT";
const char* pass_2 = "rebplhzu";

const char* ssid_3 = "JebHuaJai";
const char* pass_3 = "ffffffff";

// --------------------- MQTT Config --------------------------------
const char* mqtt_broker = "test.mosquitto.org";
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
  if(active){
    // Active LOW สั่งเปิด
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }else{
    // Active HIGH สั่งปิด
    digitalWrite(pin, INPUT);
  }
}

// ------------------ UI --------------------------------------------
void drawScene_Main() {
  tft.fillScreen(TFT_BLACK); // ล้างจอ
  isSceneShown = true;

  tft.setTextColor(TFT_WHITE, TFT_BLACK); // สีขาว พื้นดำ
  tft.setTextSize(3);
  // tft.setFreeFont(); // (ถ้าเจ้านายไม่ได้โหลดฟอนต์แยก แนะนำให้ปิดบรรทัดนี้แล้วใช้ฟอนต์มาตรฐานจะง่ายกว่าครับ)

  // --- หัวข้อและหน่วย ---
  tft.drawString("Soil", 10, 10);
  tft.drawString("%", 215, 10);
  
  tft.drawString("Lux", 10, 40);
  // ตรง Lux อาจจะไม่มีหน่วย % หรือเปล่าครับ? อาจจะเป็นค่าดิบ หรือ DLI
  // tft.drawString("%", 215, 40); 

  tft.drawLine(10, 71, 229, 71, TFT_WHITE); // เส้นคั่นกลาง

  // --- โซนสถานะ (วงกลมเปล่าๆ) ---
  // Valve & Light Labels
  tft.setTextSize(2); // ลดขนาดฟอนต์หน่อย
  tft.drawString("VALVE", 49, 85);
  tft.drawString("LIGHT", 164, 85);

  // Auto/Manual Circles (วาดโครงไว้ก่อน)
  tft.drawEllipse(62, 122, 25, 25, TFT_WHITE);
  tft.drawEllipse(177, 122, 25, 25, TFT_WHITE);
  
  tft.setTextSize(1);
  tft.setTextDatum(MC_DATUM); // จัดกึ่งกลางข้อความ
  tft.drawString("MODE", 62, 122);  // เขียนกำกับไว้ตรงกลาง
  tft.drawString("MODE", 177, 122);

  // ON/OFF Circles (วาดโครงไว้ก่อน)
  tft.drawEllipse(34, 184, 25, 25, TFT_WHITE);   // Valve ON
  tft.drawEllipse(91, 184, 25, 25, TFT_WHITE);   // Valve OFF
  tft.drawEllipse(148, 184, 25, 25, TFT_WHITE);  // Light ON
  tft.drawEllipse(205, 184, 25, 25, TFT_WHITE);  // Light OFF

  // Label ON/OFF เล็กๆ
  tft.drawString("ON", 34, 160);
  tft.drawString("OFF", 91, 160);
  tft.drawString("ON", 148, 160);
  tft.drawString("OFF", 205, 160);
}

void updateDisplay_Dynamic(int h, int m){
  tft.setTextSize(3);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK); // สีเหลือง พื้นดำ (กันกระพริบ)
  tft.setTextDatum(TR_DATUM);

  tft.drawNumber(soilPercent, 200, 10); // แสดงค่า Soil ที่พิกัด x=200
  tft.drawFloat(current_DLI, 2, 200, 40); // แสดงค่า Lux/DLI

  // --- 2. อัปเดตสถานะ VALVE (วงกลมสี) ---
  // โหมด Auto/Manual
  if(isValveManual) {
     tft.fillEllipse(62, 122, 23, 23, TFT_ORANGE); // Manual = สีส้ม
     tft.setTextColor(TFT_BLACK, TFT_ORANGE);
     tft.setTextDatum(MC_DATUM);
     tft.drawString("MAN", 62, 122, 2); 
  } else {
     tft.fillEllipse(62, 122, 23, 23, TFT_CYAN); // Auto = สีฟ้า
     tft.setTextColor(TFT_BLACK, TFT_CYAN);
     tft.setTextDatum(MC_DATUM);
     tft.drawString("AUTO", 62, 122, 2);
  }

  // สถานะ ON/OFF (Valve)
  if(isValveMainOn) {
    tft.fillEllipse(34, 184, 20, 20, TFT_GREEN); // ปุ่ม ON ติดเขียว
    tft.fillEllipse(91, 184, 20, 20, TFT_BLACK); // ปุ่ม OFF ดับ
  } else {
    tft.fillEllipse(34, 184, 20, 20, TFT_BLACK); // ปุ่ม ON ดับ
    tft.fillEllipse(91, 184, 20, 20, TFT_RED);   // ปุ่ม OFF ติดแดง
  }

  // --- 3. อัปเดตสถานะ LIGHT (วงกลมสี) ---
  // โหมด Auto/Manual
  if(isLightManual) {
     tft.fillEllipse(177, 122, 23, 23, TFT_ORANGE);
     tft.setTextColor(TFT_BLACK, TFT_ORANGE);
     tft.drawString("MAN", 177, 122, 2);
  } else {
     tft.fillEllipse(177, 122, 23, 23, TFT_CYAN);
     tft.setTextColor(TFT_BLACK, TFT_CYAN);
     tft.drawString("AUTO", 177, 122, 2);
  }

  // สถานะ ON/OFF (Light)
  if(isLightOn) {
    tft.fillEllipse(148, 184, 20, 20, TFT_GREEN);
    tft.fillEllipse(205, 184, 20, 20, TFT_BLACK);
  } else {
    tft.fillEllipse(148, 184, 20, 20, TFT_BLACK);
    tft.fillEllipse(205, 184, 20, 20, TFT_RED);
  }

  // ใช้พื้นที่ว่างๆ เขียนเวลาปัจจุบัน
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextDatum(BR_DATUM); // ชิดขวาล่างสุด
  
  char timeStr[10];
  sprintf(timeStr, "%02d:%02d", h, m);
  tft.drawString(timeStr, 235, 235, 2); // มุมขวาล่าง
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
  Serial.print(msg);

  // สั่งงานผ่านมือถือ (Topic: group8/command)
  if(String(topic) == mqtt_topic_cmd){
  // ควบคุมน้ำเอง
    if(msg == "VALVE_MANUAL"){
      isValveManual = true;
      Serial.println("SET MODE: VALVE MANUAL");
    }else if(msg == "VALVE_AUTO"){
      isValveManual = false;
      Serial.println("SET MODE: VALVE AUTO");
    }
    // Action Control ทำงานตอนอยู๋ในโหมด Manual เท่านั้น
    else if(msg == "VALVE_ON"){
      if(isValveManual){
        setRelayState(RELAY_VALVE_MAIN, true);
        isValveMainOn = true;
        Serial.println("MANUAL: VALVE ON"); 
      }else{
        Serial.println("System is in AUTO Mode!!");
      }
    }else if(msg == "VALVE_OFF"){
      if(isValveManual){
        setRelayState(RELAY_VALVE_MAIN, false);
        isValveMainOn = false;
        Serial.println("MANUAL: VALVE OFF");
      }else{
        Serial.println("System is in AUTO Mode!!");
      }
    }
  // ควบคุมไฟเอง
    else if(msg == "LIGHT_MANUAL"){
      isLightManual = true;
      Serial.println("SET MODE: LIGHT MANUAL");
    }else if(msg == "LIGHT_AUTO"){
      isLightManual = false;
      Serial.println("SET MODE: LIGHT AUTO");
    }
    // Action Control ทำงานได้เฉพาะอยู่ในโหมด manual เท่านั้น
    else if(msg == "LIGHT_ON"){
      if(isLightManual){
        setRelayState(RELAY_LIGHT, true);
        isLightOn = true;
        Serial.println("MANUAL: LIGHT ON");
      }else{
        Serial.println("System is in AUTO Mode!!");
      }
      
    }else if(msg == "LIGHT_OFF"){
      if(isLightManual){
        setRelayState(RELAY_LIGHT, false);
        isLightOn = false;
        Serial.println("MANUAL: LIGHT OFF");
      }else{
        Serial.println("System is in AUTO Mode!!");
      }
    }
  }
}

// --------------------- Tick Status -----------------------------------------
void tick(){
  // สถานะไฟกระพริบ (ถ้ามี)
}

// --------------------- Time Zone Setting (UTC+7) ---------------------------
void timezoneSync(){
  const char* ntpServer = "pool.ntp.org";
  const long gmtOffset_sec = 25200;          
  const int daylightOffset_sec = 0;          

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Waiting for time syncing...");

  struct tm timeinfo;
  while(!getLocalTime(&timeinfo)){
    Serial.println(".");
    delay(500); 
  }
  Serial.println("\nTime Synced!");
}


// --------------------- Connection ---------------------------------
void connect(){
  if(wifiMulti.run() == WL_CONNECTED){
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);

    if(client.connected()){
      return;
    }

    if(client.connect(mqtt_client_id)){
      Serial.println("\nMQTT Reconnected (Silent)!");
      client.subscribe(mqtt_topic_cmd);
      client.publish(topic_status, "SYSTEM READY");

      if(!isSceneShown) drawScene_Main();
      return;
    }else{
      Serial.println("MQTT Failed... Retrying silently");
      delay(1000); 
      return;
    }
  }
  // ระหว่างรอ WiFi ให้ขึ้นจอหน่อย
  isSceneShown = false;
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Connecting...", 120, 120, 4);

  blinker.attach(0.5, tick);

  // WiFi
  Serial.print("Checking WiFi...");
  while(wifiMulti.run() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("Connected to: ");
  Serial.println(WiFi.SSID());
  
  // MQTT
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  Serial.print("Connecting MQTT...");

  while(!client.connected()){
    if(client.connect(mqtt_client_id)){
      Serial.println("\nMQTT Connected!");
      client.subscribe(mqtt_topic_cmd);
      client.publish(topic_status, "SYSTEM READY");
    } else {
      Serial.print(".");
      delay(1000);
    }
  }
  blinker.detach();

  // พอต่อติดแล้ว ให้วาด UI หลักรอไว้เลย
  drawScene_Main();
}

// --------------------- คำนวณระบบ DLI & SOIL -------------------------------
void calculate(int currentHour){
  // LIGHT SYSTEM
  float lux = lightMeter.readLightLevel();
  float factor_used;

  if(digitalRead(RELAY_LIGHT) == LOW){
    factor_used = LIGHT_FACTOR;
    isLightOn = true;
  }else{
    factor_used = SUN_FACTOR;
    isLightOn = false;
  }

  float ppfd = lux * factor_used;
  current_DLI += (ppfd * 1.0) / 1000000.0;
  Serial.print("Current DLI: ");
  Serial.println(current_DLI);

  // อ่านค่าความชื้นดิน
  int rawSoil = analogRead(SOIL_PIN);
  soilPercent = map(rawSoil, AIR_VALUE, WATER_VALUE, 0, 100);
  soilPercent = constrain(soilPercent, 0, 100); // 0 - 100 %

  if(isLightManual){
    isLightOn = (digitalRead(RELAY_LIGHT) == LOW);
  }
  else {
    if(currentHour >= 6 && currentHour < 18){
      setRelayState(RELAY_LIGHT, false);
    }else if(currentHour >= 18 && currentHour < 24){
      if(current_DLI < target_DLI){
        setRelayState(RELAY_LIGHT, true); // เปิดไฟ
      }else{
        setRelayState(RELAY_LIGHT, false);
      }
    }else{
      setRelayState(RELAY_LIGHT, false);
    }
    // ถ้าเป็นตอนเที่ยงคืนให้ Reset ค่า DLI
    if(currentHour == 0 && current_DLI > 1.0){ // >1.0 เพื่อกัน Reset รัวๆ
      current_DLI = 0; 
    }
  }

  if(isValveManual){
    isValveMainOn = (digitalRead(RELAY_VALVE_MAIN) == LOW);
  }else{
    // WATER SYSTEM
    bool isMorning = (currentHour >= MORNING_START && currentHour < MORNING_END);
    bool isEvening = (currentHour >= EVENING_START && currentHour < EVENING_END);
    // ถ้าไม่ได้อยู่ในช่วงเช้าและเย็น
    if(!isMorning && !isEvening){
      setRelayState(RELAY_VALVE_MAIN, false);
      isValveMainOn = false;
      isMorningDone = false;
      isEveningDone = false;
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

  // Debug Monitor
  Serial.println("--- Status ---");
  Serial.print(" | Lux: "); Serial.println(lux);
  Serial.print(" | DLI: "); Serial.println(current_DLI);

  Serial.print("Current Hour: "); Serial.println(currentHour);
  Serial.print(" | Soil: "); Serial.println(soilPercent);
  Serial.print(" | Light: "); Serial.println(isLightOn);
  Serial.print(" | Valve: "); Serial.println(isValveMainOn ? "ON" : "OFF");
  Serial.print(" | M-Done: "); Serial.println(isMorningDone);
  Serial.print(" | E-Done: "); Serial.println(isEveningDone);



  char msg[50];
  // ส่งค่า DLI (ปริมาณแสงตลอดทั้งวัน)
  sprintf(msg, "%.2f", current_DLI);
  client.publish(topic_dli, msg);
  // ส่งค่า SOIL (ค่าความชื้น)
  sprintf(msg, "%d", soilPercent);
  client.publish(topic_soil, msg);

  String statusMsg = "V:" + String(isValveManual ? "MANUAL" : "AUTO") + " | L:" + String(isLightManual ? "MANUAL" : "AUTO");
  client.publish(topic_status, statusMsg.c_str());

  sprintf(msg, ".2f", lux);
  client.publish(topic_lux, msg);
}

void setup() {
  Serial.begin(115200);
  Serial.println("--- System Starting ---");
  Wire.begin(SDA, SCL);

  tft.init();
  tft.setRotation(1);

  wifiMulti.addAP(ssid_1, pass_1);
  wifiMulti.addAP(ssid_2, pass_2);
  wifiMulti.addAP(ssid_3, pass_3);

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("BOOTING SYSTEM...", 10, 10, 4);

  setRelayState(RELAY_LIGHT, false);
  setRelayState(RELAY_VALVE_MAIN, false);
    
  pinMode(SOIL_PIN, INPUT);
  
  Serial.println("System Ready: All Relays OFF");
  delay(1000);

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)){
    Serial.print(F("BH1750 Ready!"));
  } else {
    Serial.print(F("Error initialising BH1750!!"));
  }

  // Connect Network
  connect();
  timezoneSync();
}

void loop() {
  if(!client.connected()){
    connect();
  }
  client.loop();

  if(millis() - lastScreenUpdate > 1000){
    lastScreenUpdate = millis();

    int currentHour = 0;
    int currentMin = 0;
    bool timeValid = false;

    struct tm timeinfo;
    if(getLocalTime(&timeinfo)){
      currentHour = timeinfo.tm_hour;
      currentMin = timeinfo.tm_min;
      timeValid = true;
      Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
      

      // เรียกฟังก์ชันคำนวณ (ไม่งั้น DLI ไม่ขยับ)
      calculate(currentHour);
      
    }else{
      Serial.println("Time Sync Failed: Offline");
      currentHour = 12;
      timeValid = false;
    }
    updateDisplay_Dynamic(currentHour, currentMin);
  }
}