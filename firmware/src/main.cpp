#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <Audio.h>

// ── Wiring ────────────────────────────────────────────────────
// SPH0645: BCLK→Pin21  LRCL→Pin20  DOUT→Pin8  SEL→GND  3V→3.3V
// MLX90614: SCL→Pin19  SDA→Pin18  VDD→3.3V  VSS→GND

#define AUDIO_SAMPLE_RATE_EXACT 44100.0f

// ── Noise detection thresholds ────────────────────────────────
#define NOISE_THRESHOLD     500     // RMS counts above this = loud noise
#define PRINT_INTERVAL_MS   500

// ── I2S audio objects (SPH0645) ───────────────────────────────
AudioInputI2S        i2s_in;
AudioAnalyzeRMS      rms;
AudioConnection      patch(i2s_in, 0, rms, 0);

// ── Thermal ───────────────────────────────────────────────────
Adafruit_MLX90614 mlx;

unsigned long last_print = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  AudioMemory(8);

  Wire.begin();
  if (!mlx.begin()) {
    Serial.println("ERROR: MLX90614 not found — check SDA→Pin18, SCL→Pin19");
    while (1);
  }

  Serial.println("Ready. Expose sensor to noise to trigger a reading.");
  Serial.println("RMS     | Obj Temp (C) | Ambient (C)");
  Serial.println("--------|-------------|------------");
}

void loop() {
  if (!rms.available()) return;

  float rms_val = rms.read() * 32768.0f;  // scale to counts

  if (rms_val < NOISE_THRESHOLD) return;  // quiet — skip

  unsigned long now = millis();
  if (now - last_print < PRINT_INTERVAL_MS) return;
  last_print = now;

  float obj_temp = mlx.readObjectTempC();
  float amb_temp = mlx.readAmbientTempC();

  Serial.printf("%7.0f | %11.2f | %11.2f\n", rms_val, obj_temp, amb_temp);
}
