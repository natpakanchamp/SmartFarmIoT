/*
  SmartFarmIoT_NodeA_SensorTx.ino
  ------------------------------------------------------------
  Node A (Sensor Transmitter):
  - Read Soil Moisture (median-of-5 + EMA)
  - Read BH1750 lux
  - Send to Node B via ESP-NOW
  ------------------------------------------------------------
*/

#include <Wire.h>
#include <BH1750.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <string.h>

// --------------------- Pin Mapping ---------------------
#define SOIL_PIN 34
#define SDA 21
#define SCL 22

// --------------------- Soil Calibration ---------------------
// ปรับตามการคาลิเบรตจริงของเซนเซอร์คุณ
const int AIR_VALUE   = 2730;   // แห้ง
const int WATER_VALUE = 970;    // เปียก

// --------------------- Soil Filter ---------------------
float soilSmoothedPct = 0.0f;
const float SOIL_ALPHA = 0.18f;

bool isRawSoilFault(int raw) {
  return (raw > 3500 || raw < 100);
}

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

int rawToPercent(int raw) {
  int p = map(raw, AIR_VALUE, WATER_VALUE, 0, 100);
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return p;
}

int getSoilPercentRobust(uint16_t &rawOut, bool &sensorOkOut) {
  int r1 = analogRead(SOIL_PIN);
  int r2 = analogRead(SOIL_PIN);
  int r3 = analogRead(SOIL_PIN);
  int r4 = analogRead(SOIL_PIN);
  int r5 = analogRead(SOIL_PIN);

  int raw = median5(r1, r2, r3, r4, r5);
  rawOut = (uint16_t)raw;

  if (isRawSoilFault(raw)) {
    sensorOkOut = false;
    return 100; // fail-safe value
  }

  sensorOkOut = true;
  int p = rawToPercent(raw);

  if (soilSmoothedPct <= 0.01f) soilSmoothedPct = (float)p;
  soilSmoothedPct = SOIL_ALPHA * p + (1.0f - SOIL_ALPHA) * soilSmoothedPct;

  int out = (int)roundf(soilSmoothedPct);
  if (out < 0) out = 0;
  if (out > 100) out = 100;
  return out;
}

// --------------------- BH1750 ---------------------
BH1750 lightMeter;
bool bhReady = false;
unsigned long lastBhRetryMs = 0;
const unsigned long BH_RETRY_INTERVAL = 3000UL;

float readLuxSafe(bool &ok) {
  unsigned long now = millis();

  if (!bhReady) {
    if (now - lastBhRetryMs >= BH_RETRY_INTERVAL) {
      lastBhRetryMs = now;
      bhReady = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
      if (bhReady) Serial.println("[BH1750] Recovered");
    }
    ok = false;
    return NAN;
  }

  float lux = lightMeter.readLightLevel();
  if (isnan(lux) || isinf(lux) || lux < 0.0f || lux > 120000.0f) {
    bhReady = false;
    ok = false;
    return NAN;
  }

  ok = true;
  return lux;
}

// --------------------- ESP-NOW Packet ---------------------
// ต้องตรงกับ Node B
typedef struct __attribute__((packed)) {
  uint32_t seq;
  int16_t  soilPct;     // 0..100
  uint16_t soilRaw;     // raw ADC
  uint32_t uptimeMs;
  uint8_t  sensorOk;    // 1=ok, 0=fault
} SoilPacket;

// (เสริม) packet แสง แยกอีกตัว หากอนาคต Node B อยากรับด้วย
typedef struct __attribute__((packed)) {
  uint32_t seq;
  float    lux;
  uint32_t uptimeMs;
  uint8_t  sensorOk;    // 1=ok, 0=fault
} LuxPacket;

// ใส่ MAC ของ Node B (ตัวรับ) ให้ถูกต้อง
// ตัวอย่าง: 24:6F:28:AA:BB:CC
uint8_t NODE_B_MAC[6] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

// จะใช้อยู่ channel เดียวกับ Node B (WiFi STA)
const uint8_t ESPNOW_CHANNEL = 1;

// --------------------- Send scheduling ---------------------
unsigned long lastSoilTxMs = 0;
unsigned long lastLuxTxMs  = 0;
const unsigned long SOIL_TX_INTERVAL_MS = 1000UL;   // 1s
const unsigned long LUX_TX_INTERVAL_MS  = 1000UL;   // 1s

uint32_t soilSeq = 0;
uint32_t luxSeq  = 0;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[ESP-NOW] Send fail");
  }
}

bool initEspNowTx() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    return false;
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, NODE_B_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_is_peer_exist(NODE_B_MAC)) {
    esp_now_del_peer(NODE_B_MAC);
  }

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW] add peer failed");
    return false;
  }

  Serial.println("[ESP-NOW] TX ready");
  return true;
}

void sendSoilPacket() {
  uint16_t raw = 0;
  bool soilOk = false;
  int soilPct = getSoilPercentRobust(raw, soilOk);

  SoilPacket pkt;
  pkt.seq = ++soilSeq;
  pkt.soilPct = (int16_t)soilPct;
  pkt.soilRaw = raw;
  pkt.uptimeMs = millis();
  pkt.sensorOk = soilOk ? 1 : 0;

  esp_err_t r = esp_now_send(NODE_B_MAC, (uint8_t*)&pkt, sizeof(pkt));
  if (r != ESP_OK) {
    Serial.printf("[ESP-NOW] Soil send err=%d\n", (int)r);
  }

  Serial.printf("[SOIL] seq=%lu raw=%u pct=%d ok=%d\n",
                (unsigned long)pkt.seq, pkt.soilRaw, pkt.soilPct, pkt.sensorOk);
}

void sendLuxPacket() {
  bool luxOk = false;
  float lux = readLuxSafe(luxOk);

  LuxPacket pkt;
  pkt.seq = ++luxSeq;
  pkt.lux = luxOk ? lux : NAN;
  pkt.uptimeMs = millis();
  pkt.sensorOk = luxOk ? 1 : 0;

  // ใช้ send ปลายทางเดียวกัน (Node B ตอนนี้ยังไม่ได้ parse packet นี้)
  esp_err_t r = esp_now_send(NODE_B_MAC, (uint8_t*)&pkt, sizeof(pkt));
  if (r != ESP_OK) {
    Serial.printf("[ESP-NOW] Lux send err=%d\n", (int)r);
  }

  if (luxOk) {
    Serial.printf("[LUX ] seq=%lu lux=%.2f ok=1\n", (unsigned long)pkt.seq, pkt.lux);
  } else {
    Serial.printf("[LUX ] seq=%lu lux=nan ok=0\n", (unsigned long)pkt.seq);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(SOIL_PIN, INPUT);

  Wire.begin(SDA, SCL);
  bhReady = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  Serial.println(bhReady ? "BH1750 Ready" : "BH1750 init fail");

  WiFi.mode(WIFI_STA);

  // ล็อก channel ให้ตรงกับ Node B (สำคัญ)
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("[WiFi] STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("[WiFi] Channel fixed to %u\n", ESPNOW_CHANNEL);

  if (!initEspNowTx()) {
    Serial.println("ESP-NOW TX init error");
  }

  Serial.println("Node A Ready");
}

void loop() {
  unsigned long now = millis();

  if (now - lastSoilTxMs >= SOIL_TX_INTERVAL_MS) {
    lastSoilTxMs = now;
    sendSoilPacket();
  }

  // ส่ง BH1750 แยก packet (เผื่ออนาคต)
  if (now - lastLuxTxMs >= LUX_TX_INTERVAL_MS) {
    lastLuxTxMs = now;
    sendLuxPacket();
  }
}