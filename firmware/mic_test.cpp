// mic_test.cpp — SPH0641 PDM microphone sanity check for Teensy 4.1
//
// To use: copy this file's contents over src/main.cpp, flash, open serial monitor.
// Restore src/main.cpp when done.
//
// Wiring:
//   SPH0641: CLK→Pin21  DATA→Pin8  SEL→GND  VDD→3.3V
//   NeoPixel: DIN→Pin6

#include <Arduino.h>
#include <Audio.h>
#include <Adafruit_NeoPixel.h>

AudioInputPDM    pdm_in;
AudioAnalyzePeak peak;
AudioConnection  patch(pdm_in, 0, peak, 0);

Adafruit_NeoPixel pixel(1, 6, NEO_GRB + NEO_KHZ800);

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    AudioMemory(12);

    pixel.begin();
    pixel.setBrightness(48);
    pixel.setPixelColor(0, pixel.Color(0, 0, 40));
    pixel.show();

    Serial.println("=== SPH0641 Mic Test (Teensy 4.1) ===");
    Serial.println("Expected: level fluctuates with sound; bar grows when you speak/hiss near mic.");
    Serial.println("Problem : level stuck at 0.0000 → wiring issue or wrong pin.");
    Serial.println("Problem : level stuck at 1.0000 → clipping / PDM not settled yet.");
    Serial.println();
    Serial.println("  peak     dBFS  |-------- bar (40 cols = full scale) --------|");
}

void loop() {
    static unsigned long last_ms = 0;
    if (millis() - last_ms < 50) return;   // 20 Hz print rate
    last_ms = millis();

    if (!peak.available()) return;

    float level = peak.read();             // 0.0 – 1.0
    float db    = level > 0.0f ? 20.0f * log10f(level) : -99.0f;

    int bars = (int)(level * 40.0f);
    if (bars > 40) bars = 40;
    char bar[41];
    memset(bar, '=', bars);
    bar[bars] = '\0';

    Serial.printf("  %.4f  %6.1f  |%-40s|\n", level, db, bar);

    // NeoPixel VU: green → amber → red
    uint8_t r = (uint8_t)(level * 255.0f);
    uint8_t g = (uint8_t)((1.0f - level) * 180.0f);
    pixel.setPixelColor(0, pixel.Color(r, g, 0));
    pixel.show();
}
