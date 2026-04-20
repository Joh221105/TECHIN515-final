#include <Arduino.h>
#include <Wire.h>
#include <arm_math.h>
#include <Adafruit_MLX90614.h>
#include <Audio.h>

// ── Wiring ────────────────────────────────────────────────────
// SPH0645: BCLK→Pin21  LRCL→Pin20  DOUT→Pin8  SEL→GND  3V→3.3V
// MLX90614: SCL→Pin19  SDA→Pin18  VDD→3.3V  VSS→GND

#define AUDIO_SAMPLE_RATE_EXACT 100000.0f

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

// ── Thermal Config ────────────────────────────────────────────
#define DELTA_T_THRESHOLD_C  2.0f   // drop >= 2 C = thermal anomaly
#define THERMAL_SAMPLE_MS    250
#define BASELINE_SAMPLES     20     // 5 seconds of baseline

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

// ── DSP state ─────────────────────────────────────────────────
static float snr_db          = 0.0f;
static float peak_freq       = 0.0f;
static int   confirm_count   = 0;
static bool  acoustic_leak   = false;

// ── Thermal state ─────────────────────────────────────────────
Adafruit_MLX90614 mlx;
static float baseline_temp  = 0.0f;
static float last_temp      = 0.0f;
static float object_temp    = 0.0f;
static float delta_t        = 0.0f;
static bool  thermal_anomaly = false;
static unsigned long last_thermal_ms = 0;

static unsigned long last_print_ms = 0;

// ── Helpers ───────────────────────────────────────────────────
float bandPower(float32_t* mag, int lo, int hi) {
    int cap = min(hi, FFT_SIZE / 2 - 1);
    float32_t sum = 0;
    for (int i = lo; i <= cap; i++) sum += mag[i] * mag[i];
    int n = cap - lo + 1;
    return n > 0 ? sum / n : 0.0f;
}

float todB(float p) { return p > 0 ? 10.0f * log10f(p) : -999.0f; }

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println("=== PSSS Leak Detector ===");
    Serial.printf("FFT bin: %.1f Hz | Signal band: %d-%d Hz\n",
                  BIN_RES, SIGNAL_BAND_LOW, SIGNAL_BAND_HIGH);

    arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++)
        hann[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_SIZE - 1)));
    memset(noise_floor, 0, sizeof(noise_floor));

    AudioMemory(12);
    queue.begin();
    Serial.println("SPH0645 ready.");

    Wire.begin();
    if (!mlx.begin()) {
        Serial.println("ERROR: MLX90614 not found — check SDA→18, SCL→19");
        while (1);
    }
    Serial.println("MLX90614 ready. Collecting 5s baseline — keep sensor still...");

    float sum = 0;
    for (int i = 0; i < BASELINE_SAMPLES; i++) {
        sum += mlx.readObjectTempC();
        delay(THERMAL_SAMPLE_MS);
        Serial.print(".");
    }
    baseline_temp = sum / BASELINE_SAMPLES;
    last_temp     = baseline_temp;
    Serial.printf("\nBaseline: %.2f C\n", baseline_temp);
    Serial.println("\nMonitoring. Expose to ultrasonic noise + spray IPA to confirm leak.");
    Serial.println("SNR(dB) | Peak(Hz) | Acoustic | ObjTemp | DeltaT | Thermal | FUSED");
    Serial.println("---------------------------------------------------------------------");
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
    arm_cmplx_mag_f32(fft_output, magnitude, FFT_SIZE / 2);

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

    if (snr_db >= SNR_THRESHOLD_DB) {
        if (++confirm_count >= LEAK_CONFIRM_COUNT) acoustic_leak = true;
    } else {
        confirm_count = 0;
        acoustic_leak = false;
    }
    return true;
}

// ── Thermal pipeline ──────────────────────────────────────────
void runThermal() {
    unsigned long now = millis();
    if (now - last_thermal_ms < THERMAL_SAMPLE_MS) return;
    last_thermal_ms = now;

    object_temp     = mlx.readObjectTempC();
    delta_t         = object_temp - baseline_temp;
    last_temp       = object_temp;
    thermal_anomaly = (delta_t <= -DELTA_T_THRESHOLD_C);
}

// ── Main loop ─────────────────────────────────────────────────
void loop() {
    bool new_fft = runAcoustic();
    runThermal();

    bool leak = acoustic_leak && thermal_anomaly;

    unsigned long now = millis();
    if (new_fft && now - last_print_ms >= PRINT_INTERVAL_MS) {
        last_print_ms = now;
        Serial.printf("%7.1f | %8.0f | %8s | %7.2f | %6.2f | %7s | %s\n",
                      snr_db, peak_freq,
                      acoustic_leak   ? "LEAK" : "----",
                      object_temp, delta_t,
                      thermal_anomaly ? "ANOMALY" : "-------",
                      leak            ? "*** LEAK CONFIRMED ***" : "monitoring...");
    }
}
