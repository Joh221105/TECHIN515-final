#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ── Wiring (XIAO ESP32S3) ────────────────────────────────────
// Teensy pin 1 (TX1) → XIAO D7 / GPIO44 (UART_RX)
// Teensy pin 0 (RX1) → XIAO D6 / GPIO43 (UART_TX)
#define UART_RX_PIN  44   // D7
#define UART_TX_PIN  43   // D6
#define UART_BAUD    115200

// Match ble_connect.py — do not change these UUIDs
#define SERVICE_UUID  "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHAR_UUID     "0000ffe1-0000-1000-8000-00805f9b34fb"

// Binary frame constants (must match Teensy firmware)
#define FRAME_TOTAL_THERMAL  396  // 0xFF 0xFE
#define FRAME_TOTAL_CONTROL   12  // 0xFF 0xFC
#define FRAME_TOTAL_MAX      FRAME_TOTAL_THERMAL

// Max bytes per BLE notification (default MTU 23 is too small; 512 needs MTU negotiation)
#define CHUNK_SIZE   512

BLECharacteristic* pChar     = nullptr;
bool               connected = false;

static uint8_t rx_buf[FRAME_TOTAL_MAX * 2];
static int     rx_len = 0;

class ServerCB : public BLEServerCallbacks {
    void onConnect(BLEServer*)    override { connected = true;  Serial.println("[BLE] connected"); }
    void onDisconnect(BLEServer*) override {
        connected = false;
        Serial.println("[BLE] disconnected — restarting advertising");
        BLEDevice::getAdvertising()->start();
    }
};

void setup() {
    Serial.begin(115200);
    delay(2000);   // give USB-CDC time to re-enumerate after reset
    Serial.println("=== PSSS ESP32-S3 BLE bridge ===");

    // UART from Teensy
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("UART1  RX=GPIO%d  TX=GPIO%d\n", UART_RX_PIN, UART_TX_PIN);

    // BLE setup
    BLEDevice::init("PSSS-Sensor");
    BLEDevice::setMTU(517);

    BLEServer*  srv = BLEDevice::createServer();
    srv->setCallbacks(new ServerCB());

    BLEService* svc = srv->createService(SERVICE_UUID);
    pChar = svc->createCharacteristic(CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pChar->addDescriptor(new BLE2902());
    svc->start();

    BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
    BLEDevice::getAdvertising()->start();
    Serial.println("BLE advertising as 'PSSS-Sensor'");
}

static void sendFrame(const uint8_t* data, int len) {
    for (int off = 0; off < len; off += CHUNK_SIZE) {
        int n = min(CHUNK_SIZE, len - off);
        pChar->setValue(const_cast<uint8_t*>(data + off), n);
        pChar->notify();
        delay(1);
    }
}

static unsigned long last_hb = 0;

void loop() {
    // Heartbeat — visible whenever monitor is opened
    if (millis() - last_hb >= 3000) {
        last_hb = millis();
        Serial.printf("[PSSS] uptime=%lus  ble=%s  rx_buf=%dB\n",
                      millis() / 1000, connected ? "connected" : "advertising", rx_len);
    }

    while (Serial1.available() && rx_len < (int)sizeof(rx_buf) - 1)
        rx_buf[rx_len++] = (uint8_t)Serial1.read();

    while (true) {
        // Find 0xFF + known type byte (0xFE thermal, 0xFC control)
        int hi = -1;
        for (int i = 0; i + 1 < rx_len; i++) {
            if (rx_buf[i] == 0xFF && (rx_buf[i + 1] == 0xFE || rx_buf[i + 1] == 0xFC)) {
                hi = i; break;
            }
        }
        if (hi < 0) {
            if (rx_len > 1) { rx_buf[0] = rx_buf[rx_len - 1]; rx_len = 1; }
            break;
        }
        if (hi > 0) { memmove(rx_buf, rx_buf + hi, rx_len - hi); rx_len -= hi; }
        if (rx_len < 2) break;
        int frameLen = (rx_buf[1] == 0xFE) ? FRAME_TOTAL_THERMAL : FRAME_TOTAL_CONTROL;
        if (rx_len < frameLen) break;

        if (connected) sendFrame(rx_buf, frameLen);

        memmove(rx_buf, rx_buf + frameLen, rx_len - frameLen);
        rx_len -= frameLen;
    }
}
