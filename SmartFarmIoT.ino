#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <time.h>
#include <Ticker.h>

#define RELAY_LIGHT 4 // แก้ขา Pin ตามที่ใช้งานจริง

// Objects
BH1750 lightMeter;
WiFiMulti wifiMulti;
WiFiClient espClient;
PubSubClient client(espClient);
Ticker blinker;

// Variables
float SUN_FACTOR = 0.0185; 
float LIGHT_FACTOR = 0.0135; 

float target_DLI = 12.0; 
float current_DLI = 0.0;

bool isLightOn = false;

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
  wifiMulti.addAP(ssid_1, pass_1);
  wifiMulti.addAP(ssid_2, pass_2);
  wifiMulti.addAP(ssid_3, pass_3);

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
}

// --------------------- คำนวณระบบ DLI -------------------------------
void calculate(int currentHour){
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
  Serial.print(current_DLI);

  if(currentHour >= 6 && currentHour < 18){
    digitalWrite(RELAY_LIGHT,HIGH);
  }else if(currentHour >= 18 && currentHour < 24){
    if(current_DLI < target_DLI){
      digitalWrite(RELAY_LIGHT, LOW); // เปิดไฟ
    }else{
      digitalWrite(RELAY_LIGHT, HIGH); 
    }
  }
  if(currentHour == 0 && current_DLI > 1.0){ // >1.0 เพื่อกัน Reset รัวๆ
    current_DLI = 0; 
  }

  // Debug Monitor
  Serial.print(" | Lux: "); Serial.print(lux);
  Serial.print(" | DLI: "); Serial.println(current_DLI);

  char msg[50];
  sprintf(msg, "%.2f", current_DLI);
  client.publish(topic_dli, msg);
}

void setup() {
  Serial.begin(115200);
  Serial.println("--- System Starting ---");
  Wire.begin();

  digitalWrite(RELAY_LIGHT, HIGH); 
  pinMode(RELAY_LIGHT, OUTPUT);    
  
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

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to show time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  int currentHour = timeinfo.tm_hour;
  
  // เรียกฟังก์ชันคำนวณ (ไม่งั้น DLI ไม่ขยับ)
  calculate(currentHour);

  delay(1000);
}