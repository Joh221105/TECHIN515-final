#include <Arduino.h>

// Teensy 4.1 HM-10 BLE Test
// Pin 0 (RX1) -> HM-10 TXD
// Pin 1 (TX1) -> HM-10 RXD

uint32_t last_send = 0;
bool connected = false;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(1000);
  Serial1.print("AT+NAMEPSSS_LEAK");
  delay(500);
  Serial.println("Waiting for BLE connection...");
}

void loop() {
  while (Serial1.available()) {
    String msg = Serial1.readStringUntil('\n');
    Serial.println(msg);  // print everything HM-10 sends for debugging

    if (!connected) {
      connected = true;
      Serial.println("[BLE] Phone connected!");
    }

    if (msg.indexOf("OK+LOST") >= 0) {
      connected = false;
      Serial.println("[BLE] Phone disconnected.");
    }
  }

  if (Serial.available()) Serial1.write(Serial.read());

  if (connected && millis() - last_send >= 250) {
    last_send = millis();
    Serial1.println("$PSSS,1,8.50,312,3.20*");
  }
}