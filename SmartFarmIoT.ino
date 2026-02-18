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
#define SOIL_COUNT 4
#define SDA 21
#define SCL 22

const int SOIL_PINS[SOIL_COUNT] = {34, 35, 32, 33};

// --------------------- Soil Calibration ---------------------
// ปรับตามการคาลิเบรตจริงของเซนเซอร์คุณ
const int AIR_VALUE   = 2730;   // แห้ง
const int WATER_VALUE = 970;    // เปียก

const uint8_t CH_MIN = 1;
const uint8_t CH_MAX = 13;

//const uint8_t ESPNOW_CHANNEL = 9;   // ต้องตรง Node B
const uint8_t ESPNOW_CHANNEL = 6;

volatile bool ackReceived = false;
volatile uint32_t ackSeqRx = 0;
volatile unsigned long lastAckMs = 0;

bool channelLocked = false;
uint8_t currentChannel = 1;
uint32_t pingSeq = 0;

const unsigned long LINK_LOST_MS = 8000UL;   // ถ้าเกินนี้ถือว่าลิงก์หาย

float soilEma[SOIL_COUNT] = {0,0,0,0};
bool  soilEmaInit[SOIL_COUNT] = {false,false,false,false};
const float SOIL_ALPHA = 0.18f;

// --------------------- BH1750 ---------------------
BH1750 lightMeter;
bool bhReady = false;
unsigned long lastBhRetryMs = 0;
const unsigned long BH_RETRY_INTERVAL = 3000UL;

struct __attribute__((packed)) Soil4Packet {
  uint8_t  msgType;         // MSG_SOIL
  uint32_t seq;
  int16_t  soilPct[SOIL_COUNT]; // 0..100 ต่อจุด
  uint16_t soilRaw[SOIL_COUNT]; // raw ต่อจุด
  uint8_t  sensorOkMask;    // bit0..bit3 = สถานะแต่ละตัว
  uint32_t uptimeMs;
};

// --------------------- Soil Filter ---------------------

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

int getSoilPercentRobustAt(uint8_t idx, uint16_t &rawOut, bool &sensorOkOut) {
  const int pin = SOIL_PINS[idx];

  int r1 = analogRead(pin);
  int r2 = analogRead(pin);
  int r3 = analogRead(pin);
  int r4 = analogRead(pin);
  int r5 = analogRead(pin);

  int raw = median5(r1, r2, r3, r4, r5);
  rawOut = (uint16_t)raw;

  if (isRawSoilFault(raw)) {
    sensorOkOut = false;
    return 100; // fail-safe value
  }

  sensorOkOut = true;
  int p = rawToPercent(raw);

  if (!soilEmaInit[idx]) {
    soilEma[idx] = (float)p;
    soilEmaInit[idx] = true;
  } else {
    soilEma[idx] = SOIL_ALPHA * p + (1.0f - SOIL_ALPHA) * soilEma[idx];
  }

  int out = (int)roundf(soilEma[idx]);
  if (out < 0) out = 0;
  if (out > 100) out = 100;
  return out;
}

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
enum MsgType : uint8_t {
  MSG_SOIL = 1,
  MSG_LUX  = 2,
  MSG_PING = 3,
  MSG_ACK  = 4
};

// (เสริม) packet แสง แยกอีกตัว หากอนาคต Node B อยากรับด้วย
typedef struct __attribute__((packed)) {
  uint8_t  msgType;
  uint32_t seq;
  float    lux;
  uint32_t uptimeMs;
  uint8_t  sensorOk;    // 1=ok, 0=fault
} LuxPacket;

typedef struct __attribute__((packed)) {
  uint8_t  msgType;   // MSG_PING
  uint32_t seq;
  uint32_t uptimeMs;
} PingPacket;

typedef struct __attribute__((packed)) {
  uint8_t  msgType;   // MSG_ACK
  uint32_t seq;
  uint32_t uptimeMs;
} AckPacket;

// ใส่ MAC ของ Node B (ตัวรับ) ให้ถูกต้อง
// ตัวอย่าง: 24:6F:28:AA:BB:CC
uint8_t NODE_B_MAC[] = {0xD4, 0xE9, 0xF4, 0xC3, 0x30, 0xD4};

// --------------------- Send scheduling ---------------------
unsigned long lastSoilTxMs = 0;
unsigned long lastLuxTxMs  = 0;
const unsigned long SOIL_TX_INTERVAL_MS = 1000UL;   // 1s
const unsigned long LUX_TX_INTERVAL_MS  = 1000UL;   // 1s

uint32_t soilSeq = 0;
uint32_t luxSeq  = 0;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data || len < 1) return;

  uint8_t type = data[0];
  if (type == MSG_ACK && len == (int)sizeof(AckPacket)) {
    AckPacket a{};
    memcpy(&a, data, sizeof(a));
    ackSeqRx = a.seq;
    ackReceived = true;
    lastAckMs = millis();
    // Serial.printf("[ACK] seq=%lu\n", (unsigned long)a.seq);
  }
}

void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  (void)tx_info;
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
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, NODE_B_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;   // dynamic: ใช้ current channel ของ STA
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

void setWifiChannel(uint8_t ch) {
  if (ch < CH_MIN || ch > CH_MAX) return;
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  currentChannel = ch;
}

void sendSoilPacket() {
  Soil4Packet pkt{};
  pkt.msgType = MSG_SOIL;
  pkt.seq = ++soilSeq;
  pkt.uptimeMs = millis();
  pkt.sensorOkMask = 0;

  for (uint8_t i = 0; i < SOIL_COUNT; i++) {
    uint16_t raw = 0;
    bool ok = false;
    int pct = getSoilPercentRobustAt(i, raw, ok);

    pkt.soilPct[i] = (int16_t)pct;
    pkt.soilRaw[i] = raw;

    if (ok) pkt.sensorOkMask |= (1 << i);
  }

  esp_err_t r = esp_now_send(NODE_B_MAC, (uint8_t*)&pkt, sizeof(pkt));
  if (r != ESP_OK) {
    Serial.printf("[ESP-NOW] Soil4 send err=%d\n", (int)r);
  }

  Serial.printf("[SOIL] seq=%lu pct=[%d,%d,%d,%d] raw=[%u,%u,%u,%u] mask=0x%02X\n",
                (unsigned long)pkt.seq,
                pkt.soilPct[0], pkt.soilPct[1], pkt.soilPct[2], pkt.soilPct[3],
                pkt.soilRaw[0], pkt.soilRaw[1], pkt.soilRaw[2], pkt.soilRaw[3],
                pkt.sensorOkMask);
}

bool probeChannelOnce(uint8_t ch, uint16_t waitMs = 90) {
  setWifiChannel(ch);

  PingPacket p{};
  p.msgType = MSG_PING;
  p.seq = ++pingSeq;
  p.uptimeMs = millis();

  ackReceived = false;
  ackSeqRx = 0;

  esp_err_t r = esp_now_send(NODE_B_MAC, (uint8_t*)&p, sizeof(p));
  if (r != ESP_OK) return false;

  unsigned long t0 = millis();
  while (millis() - t0 < waitMs) {
    if (ackReceived && ackSeqRx == p.seq) {
      return true;
    }
    delay(2);
  }
  return false;
}

bool scanAndLockChannel() {
  Serial.println("[CH] scanning...");

  // รอบแรกไล่ 1..13
  for (uint8_t ch = CH_MIN; ch <= CH_MAX; ch++) {
    if (probeChannelOnce(ch)) {
      channelLocked = true;
      lastAckMs = millis();
      Serial.printf("[CH] locked at %u\n", ch);
      return true;
    }
    delay(1);
  }

  // รอบสองเผื่อจังหวะหลุด
  for (uint8_t ch = CH_MIN; ch <= CH_MAX; ch++) {
    if (probeChannelOnce(ch, 120)) {
      channelLocked = true;
      lastAckMs = millis();
      Serial.printf("[CH] locked(retry) at %u\n", ch);
      return true;
    }
    delay(1);
  }
  
  channelLocked = false;
  Serial.println("[CH] lock failed");
  return false;
}

void maintainChannelLock() {
  static unsigned long lastPingKeepAliveMs = 0;
  unsigned long now = millis();

  if (!channelLocked) {
    scanAndLockChannel();
    return;
  }

  if (now - lastPingKeepAliveMs >= 2000UL) {
    lastPingKeepAliveMs = now;
    (void)probeChannelOnce(currentChannel, 80);
  }

  unsigned long ackMsSnapshot;
  noInterrupts();
  ackMsSnapshot = lastAckMs;
  interrupts();

  if (now - ackMsSnapshot > LINK_LOST_MS) {
    Serial.println("[CH] link lost -> re-scan");
    channelLocked = false;
  }
}

void sendLuxPacket() {
  bool luxOk = false;
  float lux = readLuxSafe(luxOk);

  LuxPacket pkt;
  pkt.seq = ++luxSeq;
  pkt.lux = luxOk ? lux : NAN;
  pkt.uptimeMs = millis();
  pkt.sensorOk = luxOk ? 1 : 0;
  pkt.msgType = MSG_LUX;

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

  for (uint8_t i = 0; i < SOIL_COUNT; i++) {
    pinMode(SOIL_PINS[i], INPUT);
  }

  Wire.begin(SDA, SCL);
  bhReady = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  Serial.println(bhReady ? "BH1750 Ready" : "BH1750 init fail");

  WiFi.mode(WIFI_STA);

  setWifiChannel(ESPNOW_CHANNEL);
  Serial.printf("[CH] fixed at %u\n", ESPNOW_CHANNEL);
  channelLocked = true;
  lastAckMs = millis();

  // ล็อก channel ให้ตรงกับ Node B (สำคัญ)
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("[WiFi] STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (!initEspNowTx()) {
    Serial.println("ESP-NOW TX init error");
  }
  Serial.println("Node A Ready");
}

void loop() {

  // ยังไม่ lock channel อย่าส่ง soil/lux
  if (!channelLocked) {
    delay(20);
    return;
  }

  unsigned long now = millis();

  if (now - lastSoilTxMs >= SOIL_TX_INTERVAL_MS) {
    lastSoilTxMs = now;
    sendSoilPacket();
  }

  if (now - lastLuxTxMs >= LUX_TX_INTERVAL_MS) {
    lastLuxTxMs = now;
    sendLuxPacket();
  }
}