#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include <arm_math.h>
#include <Adafruit_MLX90640.h>
#include <Audio.h>

// ── Wiring ────────────────────────────────────────────────────
// SPH0645: BCLK→Pin21  LRCL→Pin20  DOUT→Pin8  SEL→GND  3V→3.3V
// MLX90640: SCL→Pin19  SDA→Pin18  VDD→3.3V  VSS→GND
// ESP32-S3 BLE: ESP32-S3 D6 (TX) → Teensy pin 1 (TX1/Serial1)
//               ESP32-S3 RX      → Teensy pin 0 (RX1/Serial1)
//               VCC→3.3V  GND→GND
//
// BLE-only build: binary frames on Serial1 forwarded to ESP32-S3 for BLE.

// ── DSP Config ───────────────────────────────────────────────
#define SAMPLE_RATE         100000
#define FFT_SIZE            1024
#define SIGNAL_BAND_LOW     10000   // Hz — ultrasonic leak band
#define SIGNAL_BAND_HIGH    49000   // Hz — capped below Nyquist (50 kHz)
#define NOISE_BAND_LOW      1000
#define NOISE_BAND_HIGH     8000
#define SNR_THRESHOLD_DB    6.0f
#define LEAK_CONFIRM_COUNT  3
#define NOISE_FLOOR_ALPHA   0.05f
#define SPECTRUM_BINS       24    // downsampled clean-spectrum bars (ultrasonic band)

// ── Thermal Config ────────────────────────────────────────────
#define DELTA_T_THRESHOLD_C  2.0f
#define THERMAL_SAMPLE_MS  250    // 4 Hz target for lower perceived latency
#define CONTROL_SAMPLE_MS  100    // 10 Hz control/status stream
#define BASELINE_SAMPLES     (10000 / THERMAL_SAMPLE_MS)  // always ~10 s of baseline
#define THERMAL_TX_W         16
#define THERMAL_TX_H         12
#define THERMAL_TX_PIXELS    (THERMAL_TX_W * THERMAL_TX_H)

// ── Output ────────────────────────────────────────────────────
#define PRINT_INTERVAL_MS    500

// ── Derived DSP constants ─────────────────────────────────────
#define BIN_RES         ((float)SAMPLE_RATE / FFT_SIZE)
#define SIG_BIN_LOW     ((int)(SIGNAL_BAND_LOW  / BIN_RES))
#define SIG_BIN_HIGH    ((int)(SIGNAL_BAND_HIGH / BIN_RES))
#define NOISE_BIN_LOW   ((int)(NOISE_BAND_LOW   / BIN_RES))
#define NOISE_BIN_HIGH  ((int)(NOISE_BAND_HIGH  / BIN_RES))

// ── Audio objects (SPH0645 via I2S) ──────────────────────────
AudioInputI2S        i2s_in;
AudioRecordQueue     queue;
AudioConnection      patch(i2s_in, 0, queue, 0);

// ── DSP buffers ───────────────────────────────────────────────
static float32_t fft_input[FFT_SIZE];
static float32_t fft_output[FFT_SIZE];
static float32_t magnitude[FFT_SIZE / 2];
static float32_t noise_floor[FFT_SIZE / 2];
static float32_t hann[FFT_SIZE];
static int16_t   sample_buf[FFT_SIZE];
static int       sample_count = 0;
static arm_rfft_fast_instance_f32 fft_inst;
static bool      noise_floor_ready = false;
static uint8_t   spectrum_u8[SPECTRUM_BINS];

// ── DSP state ─────────────────────────────────────────────────
static float snr_db          = 0.0f;
static float peak_freq       = 0.0f;
static int   confirm_count   = 0;
static bool  acoustic_leak   = false;   // PRIMARY leak indicator

// ── Thermal state ─────────────────────────────────────────────
Adafruit_MLX90640 mlx;
static float thermal_frame[32 * 24];   // 768 pixels from MLX90640
static float baseline_temp   = 0.0f;
static float object_temp     = 0.0f;   // coldest pixel in frame
static float delta_t         = 0.0f;
static bool  thermal_anomaly = false;  // cold spot present — used for localization confidence
static int   cold_pixel_col  = -1;     // 0-31, x position of coldest pixel
static int   cold_pixel_row  = -1;     // 0-23, y position of coldest pixel
static unsigned long last_thermal_ms = 0;

static unsigned long last_print_ms = 0;
static unsigned long last_control_ms = 0;

// ── Helpers ───────────────────────────────────────────────────
float bandPower(float32_t* mag, int lo, int hi) {
    int cap = min(hi, FFT_SIZE / 2 - 1);
    float32_t sum = 0;
    for (int i = lo; i <= cap; i++) sum += mag[i] * mag[i];
    int n = cap - lo + 1;
    return n > 0 ? sum / n : 0.0f;
}

float todB(float p) { return p > 0 ? 10.0f * log10f(p) : -999.0f; }

// CMSIS-DSP real FFT packs DC (real), Nyquist (real), then (Re, Im) pairs for k = 1 … N/2−1.
static void rfftMagnitudes(const float32_t* y, float32_t* mag, int nReal) {
    mag[0] = fabsf(y[0]);
    for (int k = 1; k < nReal / 2; k++) {
        float re = y[2 * k];
        float im = y[2 * k + 1];
        mag[k] = sqrtf(re * re + im * im);
    }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);  // ESP32-S3: pin0=RX1, pin1=TX1
    while (!Serial && millis() < 3000);

    Serial.println("=== PSSS Leak Detector ===");
    Serial.printf("FFT bin: %.1f Hz | Signal band: %d-%d Hz\n",
                  BIN_RES, SIGNAL_BAND_LOW, SIGNAL_BAND_HIGH);

    arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++)
        hann[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_SIZE - 1)));
    memset(noise_floor, 0, sizeof(noise_floor));
    memset(spectrum_u8, 0, sizeof(spectrum_u8));

    AudioMemory(12);
    queue.begin();
    Serial.println("SPH0645 ready.");

    Wire.begin();
    Wire.setClock(400000);

    if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
        Serial.println("ERROR: MLX90640 not found — check SDA→18, SCL→19");
        while (1);
    }
    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_18BIT);
    mlx.setRefreshRate(MLX90640_4_HZ);
    Serial.println("MLX90640 ready. Collecting 10s baseline — keep sensor still...");

    float sum = 0;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
        if (mlx.getFrame(thermal_frame) != 0) { i--; continue; }
        float min_t = thermal_frame[0];
        for (int p = 1; p < 768; p++) if (thermal_frame[p] < min_t) min_t = thermal_frame[p];
        sum += min_t;
        delay(THERMAL_SAMPLE_MS);
        Serial.print(".");
    }
    baseline_temp = sum / BASELINE_SAMPLES;
    Serial.printf("\nBaseline cold-pixel: %.2f C\n", baseline_temp);
    Serial.println("\nMonitoring. Acoustic SNR >= 6 dB triggers leak detection.");
    Serial.println("Thermal heatmap provides localization when a cold spot is present.");
    Serial.println("SNR(dB) | Peak(Hz) | Acoustic | ObjTemp | DeltaT | ColdSpot(col,row) | STATUS");
    Serial.println("-------------------------------------------------------------------------------");
}

// ── Acoustic pipeline ─────────────────────────────────────────
bool runAcoustic() {
    while (queue.available() > 0 && sample_count < FFT_SIZE) {
        int16_t* block = queue.readBuffer();
        int copy = min((int)AUDIO_BLOCK_SAMPLES, FFT_SIZE - sample_count);
        memcpy(&sample_buf[sample_count], block, copy * sizeof(int16_t));
        queue.freeBuffer();
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

    // Adaptive noise floor (spectral subtraction outside signal band)
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        if (i < SIG_BIN_LOW || i > SIG_BIN_HIGH)
            noise_floor[i] = (1 - NOISE_FLOOR_ALPHA) * noise_floor[i]
                             + NOISE_FLOOR_ALPHA * magnitude[i];
    }

    float32_t clean[FFT_SIZE / 2];
    for (int i = 0; i < FFT_SIZE / 2; i++)
        clean[i] = fmaxf(0.0f, magnitude[i] - noise_floor[i]);

    float sig_pwr   = bandPower(clean,     SIG_BIN_LOW,   SIG_BIN_HIGH);
    float noise_pwr = bandPower(magnitude, NOISE_BIN_LOW, NOISE_BIN_HIGH);
    snr_db = noise_pwr > 0 ? todB(sig_pwr) - todB(noise_pwr) : 0.0f;

    float32_t peak_val; uint32_t peak_bin;
    arm_max_f32(&clean[SIG_BIN_LOW], SIG_BIN_HIGH - SIG_BIN_LOW + 1, &peak_val, &peak_bin);
    peak_freq = (peak_bin + SIG_BIN_LOW) * BIN_RES;

    // ── Acoustic is the sole leak gate ───────────────────────
    if (snr_db >= SNR_THRESHOLD_DB) {
        if (++confirm_count >= LEAK_CONFIRM_COUNT) acoustic_leak = true;
    } else {
        confirm_count = 0;
        acoustic_leak = false;
    }

    // Ultrasonic band → fixed bins for web/BLE (normalized peak per sub-band)
    {
        int spanBins = SIG_BIN_HIGH - SIG_BIN_LOW + 1;
        float bandMax[SPECTRUM_BINS];
        float gmax = 0.0f;
        for (int b = 0; b < SPECTRUM_BINS; b++) {
            int lo = SIG_BIN_LOW + (b * spanBins) / SPECTRUM_BINS;
            int hi = SIG_BIN_LOW + ((b + 1) * spanBins) / SPECTRUM_BINS - 1;
            if (hi > SIG_BIN_HIGH) hi = SIG_BIN_HIGH;
            float mx = 0.0f;
            for (int i = lo; i <= hi; i++)
                if (clean[i] > mx) mx = clean[i];
            bandMax[b] = mx;
            if (mx > gmax) gmax = mx;
        }
        for (int b = 0; b < SPECTRUM_BINS; b++) {
            spectrum_u8[b] = (gmax > 0.0f)
                ? (uint8_t)fminf(255.0f, bandMax[b] / gmax * 255.0f)
                : 0;
        }
    }
    return true;
}

static void emitControlPacket() {
    // Control packet: 0xFF 0xFC + snr f32 + peak f32 + flags u8 + spectrum[24].
    const uint8_t header[2] = {0xFF, 0xFC};
    const uint8_t flags = (acoustic_leak ? 1u : 0u) | (thermal_anomaly ? 2u : 0u);
    Serial1.write(header, 2);
    Serial1.write((uint8_t*)&snr_db, sizeof(snr_db));
    Serial1.write((uint8_t*)&peak_freq, sizeof(peak_freq));
    Serial1.write(&flags, 1);
    Serial1.write(spectrum_u8, sizeof(spectrum_u8));
}

static void runControl() {
    unsigned long now = millis();
    if (now - last_control_ms < CONTROL_SAMPLE_MS) return;
    last_control_ms = now;
    emitControlPacket();
}

// ── Thermal pipeline ──────────────────────────────────────────
void runThermal() {
    unsigned long now = millis();
    if (now - last_thermal_ms < THERMAL_SAMPLE_MS) return;
    last_thermal_ms = now;

    int err = mlx.getFrame(thermal_frame);
    if (err != 0) {
        Serial.printf("[thermal] getFrame failed: %d\n", err);
        return;
    }

    // Find coldest pixel and its 2D position in the 32×24 frame
    float min_t = thermal_frame[0];
    int   min_p = 0;
    for (int p = 1; p < 768; p++) {
        if (thermal_frame[p] < min_t) {
            min_t = thermal_frame[p];
            min_p = p;
        }
    }

    object_temp     = min_t;
    delta_t         = object_temp - baseline_temp;
    cold_pixel_col  = min_p % 32;   // x: 0–31
    cold_pixel_row  = min_p / 32;   // y: 0–23

    // Thermal anomaly = cold spot is present; used as localization confidence flag,
    // NOT as a gate for leak detection.
    thermal_anomaly = (delta_t <= -DELTA_T_THRESHOLD_C);

    // Binary on UART1 for HM-10
    uint8_t header[2] = {0xFF, 0xFE};
    uint32_t thermal_ts_ms = millis();
    Serial1.write(header, 2);
    Serial1.write((uint8_t*)&snr_db, sizeof(snr_db));
    Serial1.write((uint8_t*)&peak_freq, sizeof(peak_freq));
    Serial1.write((uint8_t*)&thermal_ts_ms, sizeof(thermal_ts_ms));
    // Bit 0 = acoustic_leak (primary), Bit 1 = thermal_anomaly (localization confidence)
    uint8_t flags = (acoustic_leak ? 1u : 0u) | (thermal_anomaly ? 2u : 0u);
    Serial1.write(&flags, 1);
    Serial1.write(spectrum_u8, sizeof(spectrum_u8));
    // Downsample 32x24 -> 16x12 (2x2 average).
    for (int y = 0; y < THERMAL_TX_H; y++) {
        for (int x = 0; x < THERMAL_TX_W; x++) {
            int src_x = x * 2;
            int src_y = y * 2;
            float t0 = thermal_frame[src_y * 32 + src_x];
            float t1 = thermal_frame[src_y * 32 + (src_x + 1)];
            float t2 = thermal_frame[(src_y + 1) * 32 + src_x];
            float t3 = thermal_frame[(src_y + 1) * 32 + (src_x + 1)];
            float t_avg = 0.25f * (t0 + t1 + t2 + t3);
            int16_t px = (int16_t)lroundf(t_avg * 10.0f);
            Serial1.write((uint8_t*)&px, 2);
        }
    }
}

// ── Main loop ─────────────────────────────────────────────────
void loop() {
    bool new_fft = runAcoustic();
    runControl();
    runThermal();

    unsigned long now = millis();
    if (new_fft && now - last_print_ms >= PRINT_INTERVAL_MS) {
        last_print_ms = now;

        // Build a status string that reflects the new logic:
        //   acoustic alone  → LEAK DETECTED
        //   acoustic + cold → LEAK DETECTED + LOCALIZED
        //   no acoustic     → monitoring
        char status[48];
        if (acoustic_leak && thermal_anomaly) {
            snprintf(status, sizeof(status), "*** LEAK — localized @ col%d,row%d ***",
                     cold_pixel_col, cold_pixel_row);
        } else if (acoustic_leak) {
            snprintf(status, sizeof(status), "*** LEAK DETECTED (thermal: no cold spot) ***");
        } else {
            snprintf(status, sizeof(status), "monitoring...");
        }

        Serial.printf("%7.1f | %8.0f | %8s | %7.2f | %6.2f | (%2d, %2d)          | %s\n",
                      snr_db, peak_freq,
                      acoustic_leak   ? "LEAK" : "----",
                      object_temp, delta_t,
                      cold_pixel_col, cold_pixel_row,
                      status);
    }
}
