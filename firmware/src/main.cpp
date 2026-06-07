#include <Arduino.h>
#include <Wire.h>
#include <Audio.h>
#include <Adafruit_MLX90640.h>
#include <Adafruit_NeoPixel.h>

// ── Wiring ────────────────────────────────────────────────────
// SPH0641 PDM: CLK→Pin21  DATA→Pin8  SEL→GND  VDD→3.3V
//   (0.1µF bypass cap VDD-GND close to mic)
// MLX90640:    SCL→Pin19  SDA→Pin18  VDD→3.3V  VSS→GND
// NeoPixel:    DIN→Pin6   VDD→5V     GND→GND
// ESP32-S3:    Teensy TX1(Pin0)→ESP32 RX | ESP32 TX→Teensy RX1(Pin1)

// ── Config ────────────────────────────────────────────────────
#define SAMPLE_RATE       160000
#define FFT_SIZE          1024
#define BAND_BIN_LOW      128        // ~20 kHz (160000/1024 * 128 = 20000 Hz)
#define BAND_BIN_HIGH     511        // ~80 kHz (Nyquist)
#define THRESH_LOW        0.15f
#define THRESH_HIGH       0.50f
#define SMOOTH_ALPHA      0.3f
#define NEO_PIN           6
#define NEO_COUNT         1
#define THERMAL_TX_W      16
#define THERMAL_TX_H      12
#define THERMAL_TX_PIXELS (THERMAL_TX_W * THERMAL_TX_H)
#define CONTROL_MS        100        // 10 Hz
#define THERMAL_MS        250        // 4 Hz
#define BASELINE_SAMPLES  (10000 / THERMAL_MS)

// ── Audio ─────────────────────────────────────────────────────
AudioInputPDM         pdm_in;
AudioAnalyzeFFT1024   fft;
AudioConnection       patch(pdm_in, 0, fft, 0);

// ── NeoPixel ──────────────────────────────────────────────────
Adafruit_NeoPixel strip(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

static const uint32_t ZONE_COLORS[3] = {
    0x00FF00,  // green  — far / no leak
    0xFFFF00,  // yellow — getting close
    0xFF0000,  // red    — leak nearby
};

// ── State ─────────────────────────────────────────────────────
static float    band_energy      = 0.0f;
static float    band_energy_norm = 0.0f;
static float    peak_max         = 1e-6f;
static float    peak_freq_hz     = 0.0f;
static uint8_t  proximity_zone   = 0;

Adafruit_MLX90640    mlx;
static float         thermal_frame[32 * 24];
static float         baseline_temp   = 0.0f;
static unsigned long last_control_ms = 0;
static unsigned long last_thermal_ms = 0;

// ── Acoustic pipeline ─────────────────────────────────────────
void runAcoustic() {
    if (!fft.available()) return;

    float raw      = 0.0f;
    float peak_val = 0.0f;
    int   peak_bin = BAND_BIN_LOW;

    for (int i = BAND_BIN_LOW; i <= BAND_BIN_HIGH; i++) {
        float m = fft.read(i);
        raw += m;
        if (m > peak_val) { peak_val = m; peak_bin = i; }
    }

    band_energy = SMOOTH_ALPHA * raw + (1.0f - SMOOTH_ALPHA) * band_energy;
    if (band_energy > peak_max) peak_max = band_energy;
    peak_max *= 0.9999f;

    // 3-second startup guard — hold at 0 until data is stable
    band_energy_norm = (peak_max > 1e-6f && millis() > 3000)
                       ? min(1.0f, band_energy / peak_max)
                       : 0.0f;

    peak_freq_hz = peak_bin * (SAMPLE_RATE / (float)FFT_SIZE);

    if      (band_energy_norm >= THRESH_HIGH) proximity_zone = 2;
    else if (band_energy_norm >= THRESH_LOW)  proximity_zone = 1;
    else                                      proximity_zone = 0;
}

// ── Frame emission ────────────────────────────────────────────
void emitControlFrame() {
    uint8_t buf[12];
    buf[0] = 0xFF; buf[1] = 0xFC;
    memcpy(&buf[2], &band_energy_norm, 4);
    memcpy(&buf[6], &peak_freq_hz,     4);
    buf[10] = proximity_zone;
    buf[11] = (proximity_zone >= 1) ? 1 : 0;
    Serial1.write(buf, 12);
}

void emitThermalFrame() {
    uint8_t buf[396];
    buf[0] = 0xFF; buf[1] = 0xFE;
    memcpy(&buf[2], &band_energy_norm, 4);
    uint32_t ts = millis();
    memcpy(&buf[6], &ts, 4);
    buf[10] = proximity_zone;
    buf[11] = (proximity_zone >= 1) ? 1 : 0;
    // Downsample 32×24 → 16×12 (2×2 average)
    for (int row = 0; row < THERMAL_TX_H; row++) {
        for (int col = 0; col < THERMAL_TX_W; col++) {
            float t = 0;
            for (int dr = 0; dr < 2; dr++)
                for (int dc = 0; dc < 2; dc++)
                    t += thermal_frame[(row * 2 + dr) * 32 + (col * 2 + dc)];
            t *= 0.25f;
            int16_t px = (int16_t)(t * 10.0f);
            memcpy(&buf[12 + (row * THERMAL_TX_W + col) * 2], &px, 2);
        }
    }
    Serial1.write(buf, 396);
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    while (!Serial && millis() < 3000);
    Serial.println("=== PSSS Proximity Leak Detector ===");

    strip.begin();
    strip.setBrightness(80);
    strip.setPixelColor(0, 0x000000);
    strip.show();

    AudioMemory(16);
    fft.windowFunction(AudioWindowHanning1024);

    Wire.begin();
    Wire.setClock(400000);
    if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
        Serial.println("ERROR: MLX90640 not found — check SDA→18, SCL→19");
        // Blink red until power-cycled
        while (1) {
            strip.setPixelColor(0, 0xFF0000); strip.show(); delay(200);
            strip.setPixelColor(0, 0x000000); strip.show(); delay(200);
        }
    }
    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_18BIT);
    mlx.setRefreshRate(MLX90640_4_HZ);
    Serial.println("MLX90640 ready. Collecting 10s baseline...");

    float sum = 0;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
        if (mlx.getFrame(thermal_frame) != 0) { i--; continue; }
        float min_t = thermal_frame[0];
        for (int p = 1; p < 768; p++) if (thermal_frame[p] < min_t) min_t = thermal_frame[p];
        sum += min_t;
        delay(THERMAL_MS);
        Serial.print(".");
    }
    baseline_temp = sum / BASELINE_SAMPLES;
    Serial.printf("\nBaseline: %.2f C. Monitoring...\n", baseline_temp);
    Serial.println("zone | energy | peak_hz");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
    runAcoustic();

    unsigned long now = millis();

    if (now - last_control_ms >= CONTROL_MS) {
        last_control_ms = now;
        strip.setPixelColor(0, ZONE_COLORS[proximity_zone]);
        strip.show();
        emitControlFrame();
        Serial.printf("zone=%d  energy=%.3f  peak=%.0f Hz\n",
                      proximity_zone, band_energy_norm, peak_freq_hz);
    }

    if (now - last_thermal_ms >= THERMAL_MS) {
        last_thermal_ms = now;
        if (mlx.getFrame(thermal_frame) == 0)
            emitThermalFrame();
    }
}
