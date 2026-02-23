/*
  SmartFarmIoT_NodeA_SensorTx_OOP.ino
  ------------------------------------------------------------
  Node A (Sensor Transmitter) - OOP refactor
  - Soil 4ch: median-of-5 + EMA
  - BH1750 lux read with retry
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

// ===================== Protocol (shared) =====================
#define SOIL_COUNT 4

enum MsgType : uint8_t {
  MSG_SOIL = 1,
  MSG_LUX  = 2,
  MSG_PING = 3,
  MSG_ACK  = 4
};

struct __attribute__((packed)) Soil4Packet {
  uint8_t  msgType;               // MSG_SOIL
  uint32_t seq;
  int16_t  soilPct[SOIL_COUNT];
  uint16_t soilRaw[SOIL_COUNT];
  uint8_t  sensorOkMask;          // bit0..bit3
  uint32_t uptimeMs;
};

struct __attribute__((packed)) LuxPacket {
  uint8_t  msgType;               // MSG_LUX
  uint32_t seq;
  float    lux;
  uint32_t uptimeMs;
  uint8_t  sensorOk;              // 1=ok, 0=fault
};

struct __attribute__((packed)) PingPacket {
  uint8_t  msgType;               // MSG_PING
  uint32_t seq;
  uint32_t uptimeMs;
};

struct __attribute__((packed)) AckPacket {
  uint8_t  msgType;               // MSG_ACK
  uint32_t seq;
  uint32_t uptimeMs;
};

// ===================== Config =====================
#define SDA 21
#define SCL 22

const int SOIL_PINS[SOIL_COUNT] = {34, 35, 32, 33};

// Soil calibration
const int AIR_VALUE   = 2730;
const int WATER_VALUE = 970;

const uint8_t ESPNOW_CHANNEL = 6;

// ใส่ MAC ของ Node B (ตัวรับ) ให้ถูกต้อง
uint8_t NODE_B_MAC[] = {0xD4, 0xE9, 0xF4, 0xC3, 0x30, 0xD4};

// schedule
const unsigned long SOIL_TX_INTERVAL_MS = 2000UL;
const unsigned long LUX_TX_INTERVAL_MS  = 5000UL;

// ===================== Utilities =====================
static inline bool isRawSoilFault(int raw) {
  return (raw > 3500 || raw < 100);
}

static int median5(int a, int b, int c, int d, int e) {
  int arr[5] = {a, b, c, d, e};
  for (int i = 1; i < 5; i++) {
    int key = arr[i], j = i - 1;
    while (j >= 0 && arr[j] > key) { arr[j + 1] = arr[j]; j--; }
    arr[j + 1] = key;
  }
  return arr[2];
}

static int rawToPercent(int raw) {
  int p = map(raw, AIR_VALUE, WATER_VALUE, 0, 100);
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return p;
}

// ===================== SoilSensorArray =====================
class SoilSensorArray {
public:
  SoilSensorArray(const int* pins, uint8_t n, float alpha)
  : pins_(pins), n_(n), alpha_(alpha) {}

  void begin() {
    for (uint8_t i = 0; i < n_; i++) {
      pinMode(pins_[i], INPUT);
      ema_[i] = 0.0f;
      emaInit_[i] = false;
    }
  }

  // fills arrays + mask
  void readAll(int16_t outPct[SOIL_COUNT], uint16_t outRaw[SOIL_COUNT], uint8_t &okMask) {
    okMask = 0;
    for (uint8_t i = 0; i < n_; i++) {
      uint16_t raw = 0;
      bool ok = false;
      int pct = readOne(i, raw, ok);

      outPct[i] = (int16_t)pct;
      outRaw[i] = raw;
      if (ok) okMask |= (1 << i);
    }
  }

private:
  int readOne(uint8_t idx, uint16_t &rawOut, bool &okOut) {
    const int pin = pins_[idx];

    int r1 = analogRead(pin);
    int r2 = analogRead(pin);
    int r3 = analogRead(pin);
    int r4 = analogRead(pin);
    int r5 = analogRead(pin);

    int raw = median5(r1, r2, r3, r4, r5);
    rawOut = (uint16_t)raw;

    if (isRawSoilFault(raw)) {
      okOut = false;
      return 100; // fail-safe
    }

    okOut = true;
    int p = rawToPercent(raw);

    if (!emaInit_[idx]) {
      ema_[idx] = (float)p;
      emaInit_[idx] = true;
    } else {
      ema_[idx] = alpha_ * p + (1.0f - alpha_) * ema_[idx];
    }

    int out = (int)roundf(ema_[idx]);
    if (out < 0) out = 0;
    if (out > 100) out = 100;
    return out;
  }

  const int* pins_;
  uint8_t n_;
  float alpha_;
  float ema_[SOIL_COUNT];
  bool  emaInit_[SOIL_COUNT];
};

// ===================== LuxSensorBH1750 =====================
class LuxSensorBH1750 {
public:
  LuxSensorBH1750(uint8_t addr=0x23) : addr_(addr) {}

  void begin() {
    bhReady_ = meter_.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, addr_, &Wire);
    Serial.println(bhReady_ ? "BH1750 Ready" : "BH1750 init fail");
  }

  float readLuxSafe(bool &ok) {
    unsigned long now = millis();

    if (!bhReady_) {
      if (now - lastRetryMs_ >= retryIntervalMs_) {
        lastRetryMs_ = now;
        bhReady_ = meter_.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, addr_, &Wire);
        if (bhReady_) Serial.println("[BH1750] Recovered");
      }
      ok = false;
      return NAN;
    }

    float lux = meter_.readLightLevel();
    if (isnan(lux) || isinf(lux) || lux < 0.0f || lux > 120000.0f) {
      bhReady_ = false;
      ok = false;
      return NAN;
    }

    ok = true;
    return lux;
  }

private:
  BH1750 meter_;
  uint8_t addr_;
  bool bhReady_ = false;
  unsigned long lastRetryMs_ = 0;
  const unsigned long retryIntervalMs_ = 3000UL;
};

// ===================== EspNowLinkTx =====================
class EspNowLinkTx {
public:
  bool begin(const uint8_t peerMac[6], uint8_t channel) {
    memcpy(peerMac_, peerMac, 6);
    channel_ = channel;

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    // lock channel
    setWifiChannel(channel_);

    if (esp_now_init() != ESP_OK) {
      Serial.println("[ESP-NOW] init failed");
      return false;
    }

    esp_now_register_send_cb(&EspNowLinkTx::onDataSentStatic);
    esp_now_register_recv_cb(&EspNowLinkTx::onDataRecvStatic);
    instance_ = this;

    // add peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peerMac_, 6);
    peerInfo.channel = channel_;
    peerInfo.encrypt = false;

    if (esp_now_is_peer_exist(peerMac_)) {
      esp_now_del_peer(peerMac_);
    }
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("[ESP-NOW] add peer failed");
      return false;
    }

    Serial.println("[ESP-NOW] TX ready");
    return true;
  }

  esp_err_t send(const void* data, size_t len) {
    return esp_now_send(peerMac_, (const uint8_t*)data, len);
  }

  // optional: expose ack status ifคุณจะใช้ channel scan/keepalive ในอนาคต
  bool consumeAck(uint32_t &seqOut) {
    if (!ackReceived_) return false;
    ackReceived_ = false;
    seqOut = ackSeqRx_;
    return true;
  }

private:
  static EspNowLinkTx* instance_;

  static void onDataRecvStatic(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (instance_) instance_->onDataRecv(info, data, len);
  }
  static void onDataSentStatic(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    if (instance_) instance_->onDataSent(tx_info, status);
  }

  void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    (void)info;
    if (!data || len < 1) return;
    uint8_t type = data[0];
    if (type == MSG_ACK && len == (int)sizeof(AckPacket)) {
      AckPacket a{};
      memcpy(&a, data, sizeof(a));
      ackSeqRx_ = a.seq;
      ackReceived_ = true;
      lastAckMs_ = millis();
    }
  }

  void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    (void)tx_info;
    if (status != ESP_NOW_SEND_SUCCESS) {
      Serial.println("[ESP-NOW] Send fail");
    }
  }

  static void setWifiChannel(uint8_t ch) {
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
  }

  uint8_t peerMac_[6]{};
  uint8_t channel_ = 1;

  // ack state
  volatile bool ackReceived_ = false;
  volatile uint32_t ackSeqRx_ = 0;
  volatile unsigned long lastAckMs_ = 0;
};

EspNowLinkTx* EspNowLinkTx::instance_ = nullptr;

// ===================== NodeAApp =====================
class NodeAApp {
public:
  NodeAApp()
  : soil_(SOIL_PINS, SOIL_COUNT, 0.18f),
    lux_(),
    link_() {}

  void begin() {
    Serial.begin(115200);
    delay(200);

    Serial.print("[WiFi] MAC: ");
    Serial.println(WiFi.macAddress());

    Wire.begin(SDA, SCL);

    soil_.begin();
    lux_.begin();

    Serial.printf("[CH] fixed at %u\n", ESPNOW_CHANNEL);
    if (!link_.begin(NODE_B_MAC, ESPNOW_CHANNEL)) {
      Serial.println("ESP-NOW TX init error");
    }

    Serial.println("Node A Ready");
  }

  void update() {
    unsigned long now = millis();

    if (now - lastSoilTxMs_ >= SOIL_TX_INTERVAL_MS) {
      lastSoilTxMs_ = now;
      sendSoil();
    }

    if (now - lastLuxTxMs_ >= LUX_TX_INTERVAL_MS) {
      lastLuxTxMs_ = now;
      sendLux();
    }
  }

private:
  void sendSoil() {
    Soil4Packet pkt{};
    pkt.msgType = MSG_SOIL;
    pkt.seq = ++soilSeq_;
    pkt.uptimeMs = millis();

    soil_.readAll(pkt.soilPct, pkt.soilRaw, pkt.sensorOkMask);

    esp_err_t r = link_.send(&pkt, sizeof(pkt));
    if (r != ESP_OK) {
      Serial.printf("[ESP-NOW] Soil4 send err=%d\n", (int)r);
    }

    Serial.printf("[SOIL] seq=%lu pct=[%d,%d,%d,%d] raw=[%u,%u,%u,%u] mask=0x%02X\n",
                  (unsigned long)pkt.seq,
                  pkt.soilPct[0], pkt.soilPct[1], pkt.soilPct[2], pkt.soilPct[3],
                  pkt.soilRaw[0], pkt.soilRaw[1], pkt.soilRaw[2], pkt.soilRaw[3],
                  pkt.sensorOkMask);
  }

  void sendLux() {
    bool ok = false;
    float luxVal = lux_.readLuxSafe(ok);

    LuxPacket pkt{};
    pkt.msgType = MSG_LUX;
    pkt.seq = ++luxSeq_;
    pkt.uptimeMs = millis();
    pkt.sensorOk = ok ? 1 : 0;
    pkt.lux = ok ? luxVal : NAN;

    esp_err_t r = link_.send(&pkt, sizeof(pkt));
    if (r != ESP_OK) {
      Serial.printf("[ESP-NOW] Lux send err=%d\n", (int)r);
    }

    if (ok) Serial.printf("[LUX ] seq=%lu lux=%.2f ok=1\n", (unsigned long)pkt.seq, pkt.lux);
    else    Serial.printf("[LUX ] seq=%lu lux=nan ok=0\n", (unsigned long)pkt.seq);
  }

  SoilSensorArray soil_;
  LuxSensorBH1750 lux_;
  EspNowLinkTx link_;

  unsigned long lastSoilTxMs_ = 0;
  unsigned long lastLuxTxMs_  = 0;
  uint32_t soilSeq_ = 0;
  uint32_t luxSeq_  = 0;
};

NodeAApp app;

// ===================== Arduino entry =====================
void setup() {
  app.begin();
}

void loop() {
  app.update();
}