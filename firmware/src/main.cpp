#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include <arm_math.h>
#include <Adafruit_MLX90640.h>
#include <Adafruit_NeoPixel.h>
#include <Audio.h>
#include <input_pdm.h>

// ── Wiring ────────────────────────────────────────────────────
// SPH0641 PDM: CLK→Pin21  DATA→Pin8  SEL→GND  VDD→3.3V  GND→GND
//   0.1µF bypass cap VDD-GND close to mic; short clock trace.
// MLX90640:    SCL→Pin19  SDA→Pin18  VDD→3.3V  VSS→GND
// NeoPixel:    DIN→Pin6   V+→5V/3.3V  GND→GND  330-470Ω on DIN
// ESP32-S3:    Teensy TX1(Pin1)→ESP32 D7/GPIO44(RX)
//              Teensy RX1(Pin0)←ESP32 D6/GPIO43(TX)
//
// NOTE: AudioInputPDM on Teensy 4.1 uses SAI1 in PDM mode.
// AUDIO_SAMPLE_RATE_EXACT controls the Audio PLL; PDM bit clock =
// sample_rate × 64 OSR.  Check SPH0641 datasheet max PDM CLK before
// raising AUDIO_SAMPLE_RATE_EXACT beyond 44.1 / 48 kHz.


// ESP32-S3:    Teensy TX1(Pin1)→ESP32 D7/GPIO44(RX)
//              Teensy RX1(Pin0)←ESP32 D6/GPIO43(TX)
//
// NOTE: AudioInputPDM on Teensy 4.1 uses SAI1 in PDM mode.
// AUDIO_SAMPLE_RATE_EXACT controls the Audio PLL; PDM bit clock =
// sample_rate × 64 OSR.  Check SPH0641 datasheet max PDM CLK before
// raising AUDIO_SAMPLE_RATE_EXACT beyond 44.1 / 48 kHz.

// ── DSP Config ───────────────────────────────────────────────
#define SAMPLE_RATE          44100   // Hz — Teensy Audio default; PDM CLK = 44100×64 = 2.8 MHz
#define FFT_SIZE              1024
#define SIGNAL_BAND_LOW       5000   // Hz — gas-leak audible hiss (Nyquist = 22050 Hz at 44.1 kHz)
#define SIGNAL_BAND_HIGH     20000   // Hz — near Nyquist, captures peak hiss energy
#define NOISE_BAND_LOW         300   // Hz — sub-signal reference band
#define NOISE_BAND_HIGH       2000
#define SNR_THRESHOLD_DB      4.0f
#define LEAK_CONFIRM_COUNT      10   // consecutive FFT frames above threshold
#define NOISE_FLOOR_ALPHA     0.05f  // noise floor tracker (outside signal band)
#define AMBIENT_ALPHA         0.01f  // slow ambient level tracker
#define AMBIENT_BOOT_FRAMES     20   // frames to warm up ambient estimate
#define SPECTRUM_BINS           24   // downsampled bars sent over BLE

// ── Thermal Config ────────────────────────────────────────────
#define DELTA_T_THRESHOLD_C   2.0f
#define THERMAL_SAMPLE_MS      250   // 4 Hz
#define CONTROL_SAMPLE_MS      100   // 10 Hz
#define BASELINE_SAMPLES      (10000 / THERMAL_SAMPLE_MS)
#define THERMAL_TX_W            16
#define THERMAL_TX_H            12

// ── NeoPixel Config ───────────────────────────────────────────
#define NEOPIXEL_PIN             6
#define NEOPIXEL_COUNT           1
#define NEOPIXEL_BRIGHTNESS     48

// ── Output ────────────────────────────────────────────────────
#define PRINT_INTERVAL_MS      500

// ── Derived DSP constants ─────────────────────────────────────
#define BIN_RES          ((float)SAMPLE_RATE / FFT_SIZE)
#define SIG_BIN_LOW      ((int)(SIGNAL_BAND_LOW  / BIN_RES))
#define SIG_BIN_HIGH     ((int)(SIGNAL_BAND_HIGH / BIN_RES))
#define NOISE_BIN_LOW    ((int)(NOISE_BAND_LOW   / BIN_RES))
#define NOISE_BIN_HIGH   ((int)(NOISE_BAND_HIGH  / BIN_RES))

// ── Audio objects ─────────────────────────────────────────────
AudioInputPDM     pdm_in;
AudioRecordQueue  queue_a;
AudioConnection   patch_a(pdm_in, 0, queue_a, 0);

// ── DSP buffers ───────────────────────────────────────────────
static float32_t fft_input[FFT_SIZE];
static float32_t fft_output[FFT_SIZE];
static float32_t magnitude[FFT_SIZE / 2];
static float32_t noise_floor[FFT_SIZE / 2];
static float32_t hann[FFT_SIZE];
static int16_t   sample_buf[FFT_SIZE];
static int       sample_count       = 0;
static arm_rfft_fast_instance_f32 fft_inst;
static bool      noise_floor_ready  = false;
static uint8_t   spectrum_u8[SPECTRUM_BINS];

// ── DSP state ─────────────────────────────────────────────────
static float snr_db           = 0.0f;
static float signal_level_db  = 0.0f;
static float ambient_level_db = 0.0f;
static float peak_freq        = 0.0f;
static int   confirm_count    = 0;
static int   ambient_frames   = 0;
static bool  acoustic_leak    = false;

// ── NeoPixel ──────────────────────────────────────────────────
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ── Thermal state ─────────────────────────────────────────────
Adafruit_MLX90640 mlx;
static bool  mlx_present    = false;
static float thermal_frame[32 * 24];
static float baseline_temp  = 0.0f;
static float object_temp    = 0.0f;
static float delta_t        = 0.0f;
static bool  thermal_anomaly = false;
static int   cold_pixel_col = -1;
static int   cold_pixel_row = -1;
static unsigned long last_thermal_ms = 0;
static unsigned long last_control_ms = 0;
static unsigned long last_print_ms   = 0;

// ── Helpers ───────────────────────────────────────────────────
static float bandPower(float32_t* mag, int lo, int hi) {
    int cap = min(hi, FFT_SIZE / 2 - 1);
    float32_t sum = 0;
    for (int i = lo; i <= cap; i++) sum += mag[i] * mag[i];
    int n = cap - lo + 1;
    return n > 0 ? sum / n : 0.0f;
}

static float todB(float p) { return p > 0 ? 10.0f * log10f(p) : -999.0f; }

// CMSIS-DSP real FFT packs DC at y[0], Nyquist at y[1], then (Re,Im) pairs for k=1…N/2-1.
static void rfftMagnitudes(const float32_t* y, float32_t* mag, int nReal) {
    mag[0] = fabsf(y[0]);
    for (int k = 1; k < nReal / 2; k++) {
        float re = y[2 * k], im = y[2 * k + 1];
        mag[k] = sqrtf(re * re + im * im);
    }
}

static void updateAmbientLevel(float levelDb) {
    if (ambient_frames < AMBIENT_BOOT_FRAMES) {
        ambient_level_db += levelDb;
        if (++ambient_frames == AMBIENT_BOOT_FRAMES)
            ambient_level_db /= AMBIENT_BOOT_FRAMES;
        return;
    }
    if (!acoustic_leak)
        ambient_level_db = (1.0f - AMBIENT_ALPHA) * ambient_level_db
                           + AMBIENT_ALPHA * levelDb;
}

static void updateLED() {
    uint32_t color;
    if (acoustic_leak)
        color = pixels.Color(255, 0, 0);             // red   — leak confirmed
    else if (snr_db >= SNR_THRESHOLD_DB * 0.6f)
        color = pixels.Color(255, 100, 0);            // amber — approaching threshold
    else
        color = pixels.Color(0, 200, 0);              // green — clear
    pixels.setPixelColor(0, color);
    pixels.show();
}

// ── Acoustic pipeline ─────────────────────────────────────────
static bool runAcoustic() {
    while (queue_a.available() > 0 && sample_count < FFT_SIZE) {
        int16_t* buf  = queue_a.readBuffer();
        int      copy = min((int)AUDIO_BLOCK_SAMPLES, FFT_SIZE - sample_count);
        memcpy(&sample_buf[sample_count], buf, copy * sizeof(int16_t));
        queue_a.freeBuffer();
        sample_count += copy;
    }
    if (sample_count < FFT_SIZE) return false;
    sample_count = 0;

    for (int i = 0; i < FFT_SIZE; i++)
        fft_input[i] = (float32_t)sample_buf[i] * hann[i];

    arm_rfft_fast_f32(&fft_inst, fft_input, fft_output, 0);
    rfftMagnitudes(fft_output, magnitude, FFT_SIZE);

    if (!noise_floor_ready) {
        memcpy(noise_floor, magnitude, sizeof(float32_t) * FFT_SIZE / 2);
        noise_floor_ready = true;
        return false;
    }

    // Adaptive noise floor tracked outside the signal band
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        if (i < SIG_BIN_LOW || i > SIG_BIN_HIGH)
            noise_floor[i] = (1.0f - NOISE_FLOOR_ALPHA) * noise_floor[i]
                             + NOISE_FLOOR_ALPHA * magnitude[i];
    }

    float32_t clean[FFT_SIZE / 2];
    for (int i = 0; i < FFT_SIZE / 2; i++)
        clean[i] = fmaxf(0.0f, magnitude[i] - noise_floor[i]);

    float raw_sig_pwr = bandPower(magnitude, SIG_BIN_LOW,   SIG_BIN_HIGH);
    float noise_pwr   = bandPower(magnitude, NOISE_BIN_LOW, NOISE_BIN_HIGH);
    float clean_pwr   = bandPower(clean,     SIG_BIN_LOW,   SIG_BIN_HIGH);
    signal_level_db   = todB(raw_sig_pwr + 1e-9f);

    float spectral_snr = (noise_pwr > 0 && clean_pwr > 0)
        ? todB(clean_pwr) - todB(noise_pwr) : 0.0f;

    updateAmbientLevel(signal_level_db);
    float level_rise = (ambient_frames >= AMBIENT_BOOT_FRAMES)
        ? signal_level_db - ambient_level_db : 0.0f;
    snr_db = fmaxf(0.0f, fmaxf(spectral_snr, level_rise));

    float32_t peak_val; uint32_t peak_bin;
    arm_max_f32(&clean[SIG_BIN_LOW], SIG_BIN_HIGH - SIG_BIN_LOW + 1, &peak_val, &peak_bin);
    peak_freq = (peak_bin + SIG_BIN_LOW) * BIN_RES;

    if (snr_db >= SNR_THRESHOLD_DB) {
        if (++confirm_count >= LEAK_CONFIRM_COUNT) acoustic_leak = true;
    } else {
        confirm_count = 0;
        acoustic_leak = false;
    }

    // Normalize to uint8 for BLE spectrum bars
    {
        int span = SIG_BIN_HIGH - SIG_BIN_LOW + 1;
        float bandMax[SPECTRUM_BINS];
        float gmax = 0.0f;
        for (int b = 0; b < SPECTRUM_BINS; b++) {
            int lo = SIG_BIN_LOW + (b * span) / SPECTRUM_BINS;
            int hi = SIG_BIN_LOW + ((b + 1) * span) / SPECTRUM_BINS - 1;
            if (hi > SIG_BIN_HIGH) hi = SIG_BIN_HIGH;
            float mx = 0.0f;
            for (int i = lo; i <= hi; i++) if (clean[i] > mx) mx = clean[i];
            bandMax[b] = mx;
            if (mx > gmax) gmax = mx;
        }
        for (int b = 0; b < SPECTRUM_BINS; b++)
            spectrum_u8[b] = gmax > 0.0f
                ? (uint8_t)fminf(255.0f, bandMax[b] / gmax * 255.0f) : 0;
    }
    return true;
}

// ── Frame emission ────────────────────────────────────────────
// Control frame (12 B): 0xFF 0xFC | snr_db f32 | peak_freq f32 | zone u8 | flags u8
// zone: 0=clear  1=approaching  2=leak
static void emitControlFrame() {
    uint8_t zone = acoustic_leak ? 2u
                 : (snr_db >= SNR_THRESHOLD_DB * 0.6f ? 1u : 0u);
    uint8_t flags = acoustic_leak ? 1u : 0u;
    uint8_t buf[12];
    buf[0] = 0xFF; buf[1] = 0xFC;
    memcpy(&buf[2], &snr_db,    4);
    memcpy(&buf[6], &peak_freq, 4);
    buf[10] = zone;
    buf[11] = flags;
    Serial1.write(buf, 12);
}

// Thermal frame (396 B): 0xFF 0xFE | snr_db f32 | ts_ms u32 | zone u8 | flags u8
//   | 16×12 int16 LE pixels (°C × 10)
static void emitThermalFrame() {
    uint8_t zone  = acoustic_leak ? 2u : 0u;
    uint8_t flags = (acoustic_leak ? 1u : 0u) | (thermal_anomaly ? 2u : 0u);
    uint32_t ts   = millis();
    uint8_t buf[396];
    buf[0] = 0xFF; buf[1] = 0xFE;
    memcpy(&buf[2], &snr_db, 4);
    memcpy(&buf[6], &ts,     4);
    buf[10] = zone;
    buf[11] = flags;
    for (int y = 0; y < THERMAL_TX_H; y++) {
        for (int x = 0; x < THERMAL_TX_W; x++) {
            float t = 0;
            for (int dr = 0; dr < 2; dr++)
                for (int dc = 0; dc < 2; dc++)
                    t += thermal_frame[(y*2+dr)*32 + (x*2+dc)];
            t *= 0.25f;
            int16_t px = (int16_t)lroundf(t * 10.0f);
            memcpy(&buf[12 + (y * THERMAL_TX_W + x) * 2], &px, 2);
        }
    }
    Serial1.write(buf, 396);
}

static void runControl() {
    unsigned long now = millis();
    if (now - last_control_ms < CONTROL_SAMPLE_MS) return;
    last_control_ms = now;
    emitControlFrame();
}

// ── Thermal pipeline ──────────────────────────────────────────
static void runThermal() {
    if (!mlx_present) return;
    unsigned long now = millis();
    if (now - last_thermal_ms < THERMAL_SAMPLE_MS) return;
    last_thermal_ms = now;

    if (mlx.getFrame(thermal_frame) != 0) return;

    float min_t = thermal_frame[0]; int min_p = 0;
    for (int p = 1; p < 768; p++) {
        if (thermal_frame[p] < min_t) { min_t = thermal_frame[p]; min_p = p; }
    }
    object_temp     = min_t;
    delta_t         = object_temp - baseline_temp;
    cold_pixel_col  = min_p % 32;
    cold_pixel_row  = min_p / 32;
    thermal_anomaly = (delta_t <= -DELTA_T_THRESHOLD_C);

    emitThermalFrame();
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("=== PSSS Leak Detector ===");
    Serial.printf("FFT bin: %.1f Hz  signal band: %d–%d Hz\n",
                  BIN_RES, SIGNAL_BAND_LOW, SIGNAL_BAND_HIGH);

    arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++)
        hann[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_SIZE - 1)));
    memset(noise_floor, 0, sizeof(noise_floor));
    memset(spectrum_u8, 0, sizeof(spectrum_u8));

    pixels.begin();
    pixels.setBrightness(NEOPIXEL_BRIGHTNESS);
    pixels.setPixelColor(0, pixels.Color(0, 0, 32));  // dim blue during boot
    pixels.show();

    AudioMemory(16);
    queue_a.begin();
    Serial.println("SPH0641 ready (SAI1, Pin 8).");

    Wire.begin();
    Wire.setClock(400000);
    mlx_present = mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire);
    if (!mlx_present) {
        Serial.println("WARNING: MLX90640 not found — acoustic-only mode.");
    } else {
        mlx.setMode(MLX90640_CHESS);
        mlx.setResolution(MLX90640_ADC_18BIT);
        mlx.setRefreshRate(MLX90640_4_HZ);
        Serial.println("MLX90640 ready. Collecting 10 s baseline — keep sensor still...");
        float sum = 0;
        for (int i = 0; i < BASELINE_SAMPLES; i++) {
            if (mlx.getFrame(thermal_frame) != 0) { i--; continue; }
            float min_t = thermal_frame[0];
            for (int p = 1; p < 768; p++) if (thermal_frame[p] < min_t) min_t = thermal_frame[p];
            sum += min_t;
            pixels.setPixelColor(0, pixels.Color(0, 0, 80));
            pixels.show();
            delay(THERMAL_SAMPLE_MS);
            Serial.print(".");
        }
        baseline_temp = sum / BASELINE_SAMPLES;
        pixels.setPixelColor(0, pixels.Color(0, 200, 0));
        pixels.show();
        Serial.printf("\nBaseline cold-pixel: %.2f C\n", baseline_temp);
    }

    Serial.printf("Monitoring. SNR >= %.1f dB triggers detection.\n", SNR_THRESHOLD_DB);
    Serial.println("SNR_dB | peak_Hz | acoustic | obj_C | dT   | cold(col,row) | STATUS");
    Serial.println("----------------------------------------------------------------------");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
    bool new_fft = runAcoustic();
    if (new_fft) updateLED();
    runControl();
    runThermal();

    unsigned long now = millis();
    if (new_fft && now - last_print_ms >= PRINT_INTERVAL_MS) {
        last_print_ms = now;
        char status[52];
        if (acoustic_leak && thermal_anomaly)
            snprintf(status, sizeof(status), "*** LEAK — cold@(%d,%d) ***",
                     cold_pixel_col, cold_pixel_row);
        else if (acoustic_leak)
            snprintf(status, sizeof(status), "*** LEAK (no cold spot) ***");
        else
            snprintf(status, sizeof(status), "monitoring...");

        Serial.printf("%6.2f | %7.1f | %8s | %5.1f | %4.1f | (%2d,%2d)       | %s\n",
                      snr_db, peak_freq,
                      acoustic_leak ? "LEAK" : "----",
                      object_temp, delta_t,
                      cold_pixel_col, cold_pixel_row,
                      status);
    }
}

