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
// sample_rate × 64 OSR. At 72 kHz, PDM CLK is about 4.61 MHz,
// inside the SPH0641 ultrasonic-mode range of 3.072-4.8 MHz.

// Experiment procedure:
// 1. Start with profile 1.
// 2. Press r to reset acoustic state.
// 3. Move the mic around the tire.
// 4. Watch the NeoPixel and STATUS lines.
// 5. If it false-triggers, try profile 2. If it misses leaks, try profile 3.
//
// Best profile criteria:
// - No leak: no sustained red / LEAK status.
// - Real puncture leak: fast red / LEAK status.
// - Handling noise: should not cause sustained red.

#ifndef EXPERIMENT_PROFILE_ID
#define EXPERIMENT_PROFILE_ID 8
#endif

// ── DSP Config ───────────────────────────────────────────────
#define SAMPLE_RATE          ((float)AUDIO_SAMPLE_RATE_EXACT)
#define FFT_SIZE              1024
#define AMBIENT_BOOT_FRAMES     20   // frames to warm up ambient estimate
#define MAX_WINDOW_SIZE         32   // max sliding-window depth across all profiles
#define SPECTRUM_BINS           24   // downsampled bars sent over BLE
#define PINPOINT_TOP_BINS         3   // strongest bins averaged for narrow pinhole leaks
#define PINPOINT_PROFILE_ID       6
#define PINPOINT_SCORE_HIT        3
#define PINPOINT_SCORE_NEAR       2
#define PINPOINT_SCORE_WEAK       1
#define PINPOINT_SCORE_DECAY      1
#define PINPOINT_SCORE_CONFIRM    4
#define PINPOINT_SCORE_RELEASE    1
#define PINPOINT_SCORE_MAX       10
#define PINPOINT_NEAR_MARGIN_DB 0.4f
#define PINPOINT_WEAK_SNR_DB    0.2f
#define AUDIBLE_FALLBACK_LOW_HZ     6000.0f
#define AUDIBLE_FALLBACK_HIGH_HZ   20000.0f
#define AUDIBLE_FALLBACK_NOISE_LOW_HZ 1000.0f
#define AUDIBLE_FALLBACK_NOISE_HIGH_HZ 5000.0f
#define AUDIBLE_FALLBACK_GATE_DB    6.0f

// ── Thermal Config ────────────────────────────────────────────
#define DELTA_T_THRESHOLD_C   2.0f
#define THERMAL_SAMPLE_MS      250   // 4 Hz
#define CONTROL_SAMPLE_MS      100   // 10 Hz
#define BASELINE_SAMPLES      (10000 / THERMAL_SAMPLE_MS)
#define THERMAL_TX_W            32
#define THERMAL_TX_H            24

// ── NeoPixel Config ───────────────────────────────────────────
#define NEOPIXEL_PIN             6
#define NEOPIXEL_COUNT           1
#define NEOPIXEL_BRIGHTNESS     48
#define LED_ATTACK_ALPHA       0.55f
#define LED_RELEASE_ALPHA      0.18f
#define STRONG_SIGNAL_BLINK_MS   60
#define STRONG_SIGNAL_HOLD_MS   500

// ── Output ────────────────────────────────────────────────────
#define PRINT_INTERVAL_MS      100

// ── Derived DSP constant ──────────────────────────────────────
#define BIN_RES          ((float)SAMPLE_RATE / FFT_SIZE)

// ── Experiment profiles ───────────────────────────────────────
struct DspExperimentProfile {
    const char* name;
    float signalBandLowHz;
    float signalBandHighHz;
    float noiseBandLowHz;
    float noiseBandHighHz;
    float snrThresholdDb;
    int leakConfirmCount;    // hits (or consecutive frames) needed to confirm
    int leakReleaseCount;    // window only: hold confirmed until hits drop to this; 0 = ignored
    int windowSize;          // 0 = N consecutive frames; >0 = sliding window
    float noiseFloorAlpha;
    float ambientAlpha;
    float ledMaxSnrDb;
    bool useProportionalLed;
    bool useLevelRise;
    bool useAudibleFallback;
};

static const DspExperimentProfile EXPERIMENT_PROFILES[] = {
    //                                                                         confirm rel  win
    {"ULTRASONIC_WIDE",          20000.0f, 36000.0f, 12000.0f, 19000.0f, 6.0f,   7,  0,  0, 0.025f, 0.004f,  14.0f, true,  true,  false},
    {"ULTRASONIC_BALANCED",      22000.0f, 34000.0f, 12000.0f, 19000.0f, 6.5f,   8,  0,  0, 0.020f, 0.003f,  14.0f, true,  true,  false},
    {"ULTRASONIC_STABLE",        24000.0f, 34000.0f, 12000.0f, 19000.0f, 8.0f,  12,  0,  0, 0.015f, 0.002f,  16.0f, true,  true,  false},
    {"ULTRASONIC_SENSITIVE",     20000.0f, 36000.0f, 10000.0f, 18000.0f, 4.5f,  10,  0,  0, 0.020f, 0.003f,  12.0f, true,  true,  false},
    {"ULTRASONIC_HIGH_CORE",     26000.0f, 35500.0f, 12000.0f, 19000.0f, 7.0f,   9,  0,  0, 0.018f, 0.003f,  15.0f, true,  true,  false},
    {"ULTRASONIC_STRICT_CLOSE",  26000.0f, 34000.0f, 12000.0f, 19000.0f, 10.0f, 10,  0,  0, 0.012f, 0.002f,  18.0f, true,  true,  false},
    {"PINPOINT_HIGH_BAND",       28000.0f, 36000.0f, 18000.0f, 26000.0f, 1.0f,   4,  0,  0, 0.018f, 0.002f,   8.0f, true,  true,  false},
    {"ULTRASONIC_MODERATE",      23000.0f, 34000.0f, 12000.0f, 19000.0f, 6.0f,   4,  0,  0, 0.018f, 0.0025f, 15.0f, true,  false, true},
    // Acoustic area-finder: 4/10 confirm, 2/10 release — tolerates bursty signal; thermal pinpoints cold spot
    {"SENSITIVE_WINDOW",         20000.0f, 36000.0f, 10000.0f, 18000.0f, 5.5f,   4,  2, 10, 0.020f, 0.003f,  12.0f, true,  true,  false},
};

static const int PROFILE_COUNT = sizeof(EXPERIMENT_PROFILES) / sizeof(EXPERIMENT_PROFILES[0]);
static int activeProfileId = 1;
static const DspExperimentProfile* activeProfile = &EXPERIMENT_PROFILES[1];
static int sig_bin_low = 0;
static int sig_bin_high = 0;
static int noise_bin_low = 0;
static int noise_bin_high = 0;

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
static float spectral_snr_db  = 0.0f;
static float peak_snr_db      = 0.0f;
static float pinpoint_metric_db = 0.0f;
static float audible_snr_db   = 0.0f;
static float audible_metric_db = 0.0f;
static float level_rise_db    = 0.0f;
static float signal_level_db  = 0.0f;
static float ambient_level_db = 0.0f;
static float peak_freq        = 0.0f;
static int   confirm_count    = 0;
static int   pinpoint_score   = 0;
static int   ambient_frames   = 0;
static bool  acoustic_leak    = false;
static float led_intensity_smoothed = 0.0f;
static unsigned long last_strong_signal_ms = 0;
static uint32_t fft_frame_count = 0;
static uint32_t above_threshold_count = 0;
static uint32_t confirmed_frame_count = 0;
static float max_snr_db = 0.0f;
static float sum_snr_db = 0.0f;

// ── Sliding-window confirmation ────────────────────────────────
static bool threshold_window[MAX_WINDOW_SIZE];
static int  window_head = 0;
static int  window_hits = 0;

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
static int normalizeProfileId(int id) {
    return (id >= 0 && id < PROFILE_COUNT) ? id : 1;
}

static float clamp01(float value) {
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

static int hzToBin(float hz) {
    int bin = (int)(hz / BIN_RES);
    if (bin < 0) bin = 0;
    if (bin > FFT_SIZE / 2 - 1) bin = FFT_SIZE / 2 - 1;
    return bin;
}

static void orderBins(int* lo, int* hi) {
    if (*lo <= *hi) return;
    int tmp = *lo;
    *lo = *hi;
    *hi = tmp;
}

static void computeProfileBins() {
    sig_bin_low = hzToBin(activeProfile->signalBandLowHz);
    sig_bin_high = hzToBin(activeProfile->signalBandHighHz);
    noise_bin_low = hzToBin(activeProfile->noiseBandLowHz);
    noise_bin_high = hzToBin(activeProfile->noiseBandHighHz);
    orderBins(&sig_bin_low, &sig_bin_high);
    orderBins(&noise_bin_low, &noise_bin_high);
}

static void resetExperimentCounters() {
    fft_frame_count = 0;
    above_threshold_count = 0;
    confirmed_frame_count = 0;
    max_snr_db = 0.0f;
    sum_snr_db = 0.0f;
}

static void resetAcousticState() {
    sample_count = 0;
    noise_floor_ready = false;
    memset(noise_floor, 0, sizeof(noise_floor));
    memset(spectrum_u8, 0, sizeof(spectrum_u8));
    snr_db = 0.0f;
    spectral_snr_db = 0.0f;
    peak_snr_db = 0.0f;
    pinpoint_metric_db = 0.0f;
    audible_snr_db = 0.0f;
    audible_metric_db = 0.0f;
    level_rise_db = 0.0f;
    signal_level_db = 0.0f;
    ambient_level_db = 0.0f;
    peak_freq = 0.0f;
    confirm_count = 0;
    pinpoint_score = 0;
    ambient_frames = 0;
    acoustic_leak = false;
    memset(threshold_window, 0, sizeof(threshold_window));
    window_head = 0;
    window_hits = 0;
    led_intensity_smoothed = 0.0f;
    last_strong_signal_ms = 0;
    resetExperimentCounters();
}

static void setActiveProfile(int id, bool resetState) {
    activeProfileId = normalizeProfileId(id);
    activeProfile = &EXPERIMENT_PROFILES[activeProfileId];
    computeProfileBins();
    if (resetState) resetAcousticState();
}

static float bandPower(const float32_t* mag, int lo, int hi) {
    int cap = min(hi, FFT_SIZE / 2 - 1);
    if (lo < 0) lo = 0;
    if (lo > cap) return 0.0f;
    float32_t sum = 0;
    for (int i = lo; i <= cap; i++) sum += mag[i] * mag[i];
    int n = cap - lo + 1;
    return n > 0 ? sum / n : 0.0f;
}

static float topBinAveragePower(const float32_t* mag, int lo, int hi) {
    int cap = min(hi, FFT_SIZE / 2 - 1);
    if (lo < 0) lo = 0;
    if (lo > cap) return 0.0f;

    float32_t top[PINPOINT_TOP_BINS];
    for (int i = 0; i < PINPOINT_TOP_BINS; i++) top[i] = 0.0f;

    for (int i = lo; i <= cap; i++) {
        float32_t p = mag[i] * mag[i];
        for (int j = 0; j < PINPOINT_TOP_BINS; j++) {
            if (p <= top[j]) continue;
            for (int k = PINPOINT_TOP_BINS - 1; k > j; k--) top[k] = top[k - 1];
            top[j] = p;
            break;
        }
    }

    int available = cap - lo + 1;
    int count = min(PINPOINT_TOP_BINS, available);
    float32_t sum = 0.0f;
    for (int i = 0; i < count; i++) sum += top[i];
    return count > 0 ? sum / count : 0.0f;
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

static void updateExperimentCounters() {
    fft_frame_count++;
    sum_snr_db += snr_db;
    if (snr_db > max_snr_db) max_snr_db = snr_db;
    if (snr_db >= activeProfile->snrThresholdDb) above_threshold_count++;
    if (acoustic_leak) confirmed_frame_count++;
}

static void updateAmbientLevel(float levelDb) {
    if (ambient_frames < AMBIENT_BOOT_FRAMES) {
        ambient_level_db += levelDb;
        if (++ambient_frames == AMBIENT_BOOT_FRAMES)
            ambient_level_db /= AMBIENT_BOOT_FRAMES;
        return;
    }
    if (!acoustic_leak)
        ambient_level_db = (1.0f - activeProfile->ambientAlpha) * ambient_level_db
                           + activeProfile->ambientAlpha * levelDb;
}

static bool isPinpointProfile() {
    return activeProfileId == PINPOINT_PROFILE_ID;
}

static void updatePinpointScore() {
    if (snr_db >= activeProfile->snrThresholdDb) {
        pinpoint_score += PINPOINT_SCORE_HIT;
    } else if (snr_db >= activeProfile->snrThresholdDb - PINPOINT_NEAR_MARGIN_DB) {
        pinpoint_score += PINPOINT_SCORE_NEAR;
    } else if (snr_db >= PINPOINT_WEAK_SNR_DB) {
        pinpoint_score += PINPOINT_SCORE_WEAK;
    } else {
        pinpoint_score -= PINPOINT_SCORE_DECAY;
    }

    if (pinpoint_score < 0) pinpoint_score = 0;
    if (pinpoint_score > PINPOINT_SCORE_MAX) pinpoint_score = PINPOINT_SCORE_MAX;

    confirm_count = pinpoint_score;
    if (pinpoint_score >= PINPOINT_SCORE_CONFIRM) acoustic_leak = true;
    else if (pinpoint_score <= PINPOINT_SCORE_RELEASE) acoustic_leak = false;
}

static void updateLED(bool newAcousticFrame) {
    unsigned long now = millis();
    if (newAcousticFrame) {
        float target_intensity = activeProfile->ledMaxSnrDb > 0.0f
            ? clamp01(snr_db / activeProfile->ledMaxSnrDb) : 0.0f;
        float alpha = target_intensity > led_intensity_smoothed
            ? LED_ATTACK_ALPHA : LED_RELEASE_ALPHA;
        led_intensity_smoothed = (1.0f - alpha) * led_intensity_smoothed
                                 + alpha * target_intensity;
        if (snr_db >= activeProfile->ledMaxSnrDb) last_strong_signal_ms = now;
    }
    bool strong_signal = now - last_strong_signal_ms <= STRONG_SIGNAL_HOLD_MS;

    if (!activeProfile->useProportionalLed) {
        uint32_t color;
        if (strong_signal)
            color = ((now / STRONG_SIGNAL_BLINK_MS) % 2 == 0)
                ? pixels.Color(255, 0, 0) : pixels.Color(0, 0, 0);
        else if (acoustic_leak)
            color = pixels.Color(255, 0, 0);             // red   - leak confirmed
        else if (snr_db >= activeProfile->snrThresholdDb * 0.6f)
            color = pixels.Color(255, 100, 0);            // amber - approaching threshold
        else
            color = pixels.Color(0, 200, 0);              // green - clear
        pixels.setPixelColor(0, color);
        pixels.show();
        return;
    }

    uint32_t color;
    if (strong_signal) {
        color = ((now / STRONG_SIGNAL_BLINK_MS) % 2 == 0)
            ? pixels.Color(255, 0, 0) : pixels.Color(0, 0, 0);
    } else {
        uint8_t red;
        uint8_t green;
        if (led_intensity_smoothed < 0.5f) {
            float t = led_intensity_smoothed * 2.0f;
            red = (uint8_t)(255.0f * t);
            green = 220;
        } else {
            float t = (led_intensity_smoothed - 0.5f) * 2.0f;
            red = 255;
            green = (uint8_t)(220.0f * (1.0f - t));
        }
        color = pixels.Color(red, green, 0);
    }
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
        if (i < sig_bin_low || i > sig_bin_high)
            noise_floor[i] = (1.0f - activeProfile->noiseFloorAlpha) * noise_floor[i]
                             + activeProfile->noiseFloorAlpha * magnitude[i];
    }

    float32_t clean[FFT_SIZE / 2];
    for (int i = 0; i < FFT_SIZE / 2; i++)
        clean[i] = fmaxf(0.0f, magnitude[i] - noise_floor[i]);

    float raw_sig_pwr = bandPower(magnitude, sig_bin_low,   sig_bin_high);
    float noise_pwr   = bandPower(magnitude, noise_bin_low, noise_bin_high);
    float clean_pwr   = bandPower(clean,     sig_bin_low,   sig_bin_high);
    float peak_pwr    = topBinAveragePower(clean, sig_bin_low, sig_bin_high);
    int audible_bin_low = hzToBin(AUDIBLE_FALLBACK_LOW_HZ);
    int audible_bin_high = hzToBin(AUDIBLE_FALLBACK_HIGH_HZ);
    int audible_noise_bin_low = hzToBin(AUDIBLE_FALLBACK_NOISE_LOW_HZ);
    int audible_noise_bin_high = hzToBin(AUDIBLE_FALLBACK_NOISE_HIGH_HZ);
    orderBins(&audible_bin_low, &audible_bin_high);
    orderBins(&audible_noise_bin_low, &audible_noise_bin_high);
    float audible_clean_pwr = bandPower(clean, audible_bin_low, audible_bin_high);
    float audible_noise_pwr = bandPower(magnitude, audible_noise_bin_low, audible_noise_bin_high);
    signal_level_db   = todB(raw_sig_pwr + 1e-9f);

    spectral_snr_db = (noise_pwr > 0 && clean_pwr > 0)
        ? todB(clean_pwr) - todB(noise_pwr) : 0.0f;
    peak_snr_db = (noise_pwr > 0 && peak_pwr > 0)
        ? todB(peak_pwr) - todB(noise_pwr) : 0.0f;
    audible_snr_db = (audible_noise_pwr > 0 && audible_clean_pwr > 0)
        ? todB(audible_clean_pwr) - todB(audible_noise_pwr) : 0.0f;
    audible_metric_db = fmaxf(0.0f, audible_snr_db - AUDIBLE_FALLBACK_GATE_DB);

    updateAmbientLevel(signal_level_db);
    level_rise_db = (ambient_frames >= AMBIENT_BOOT_FRAMES)
        ? signal_level_db - ambient_level_db : 0.0f;
    pinpoint_metric_db = fmaxf(0.0f, fmaxf(peak_snr_db, spectral_snr_db));
    pinpoint_metric_db = fmaxf(pinpoint_metric_db, audible_metric_db);
    float effective_level_rise  = activeProfile->useLevelRise      ? level_rise_db    : 0.0f;
    float effective_audible     = activeProfile->useAudibleFallback ? audible_metric_db : 0.0f;
    float combined_snr_db = fmaxf(0.0f, fmaxf(spectral_snr_db, effective_level_rise));
    combined_snr_db = fmaxf(combined_snr_db, peak_snr_db);
    combined_snr_db = fmaxf(combined_snr_db, effective_audible);
    snr_db = isPinpointProfile() ? pinpoint_metric_db : combined_snr_db;

    float32_t peak_val; uint32_t peak_bin;
    arm_max_f32(&clean[sig_bin_low], sig_bin_high - sig_bin_low + 1, &peak_val, &peak_bin);
    peak_freq = (peak_bin + sig_bin_low) * BIN_RES;

    if (isPinpointProfile()) {
        updatePinpointScore();
    } else if (activeProfile->windowSize > 0) {
        bool hit = snr_db >= activeProfile->snrThresholdDb;
        if (threshold_window[window_head]) window_hits--;
        threshold_window[window_head] = hit;
        if (hit) window_hits++;
        window_head = (window_head + 1) % activeProfile->windowSize;
        confirm_count = window_hits;
        if (window_hits >= activeProfile->leakConfirmCount)
            acoustic_leak = true;
        else if (activeProfile->leakReleaseCount == 0 || window_hits <= activeProfile->leakReleaseCount)
            acoustic_leak = false;
    } else if (snr_db >= activeProfile->snrThresholdDb) {
        if (++confirm_count >= activeProfile->leakConfirmCount) acoustic_leak = true;
    } else {
        confirm_count = 0;
        acoustic_leak = false;
    }

    updateExperimentCounters();

    // Normalize to uint8 for BLE spectrum bars
    {
        int span = sig_bin_high - sig_bin_low + 1;
        float bandMax[SPECTRUM_BINS];
        float gmax = 0.0f;
        for (int b = 0; b < SPECTRUM_BINS; b++) {
            int lo = sig_bin_low + (b * span) / SPECTRUM_BINS;
            int hi = sig_bin_low + ((b + 1) * span) / SPECTRUM_BINS - 1;
            if (hi < lo) hi = lo;
            if (hi > sig_bin_high) hi = sig_bin_high;
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
// Control frame (36 B): 0xFF 0xFC | snr_db f32 | peak_freq f32 | zone u8 | flags u8 | spectrum[24] u8
// zone: 0=clear  1=approaching  2=leak
static void emitControlFrame() {
    uint8_t zone = acoustic_leak ? 2u
                 : (snr_db >= activeProfile->snrThresholdDb * 0.6f ? 1u : 0u);
    uint8_t flags = acoustic_leak ? 1u : 0u;
    uint8_t buf[36];
    buf[0] = 0xFF; buf[1] = 0xFC;
    memcpy(&buf[2], &snr_db,    4);
    memcpy(&buf[6], &peak_freq, 4);
    buf[10] = zone;
    buf[11] = flags;
    memcpy(&buf[12], spectrum_u8, SPECTRUM_BINS);
    Serial1.write(buf, 36);
}

// Thermal frame (1548 B): 0xFF 0xFE | snr_db f32 | ts_ms u32 | zone u8 | flags u8
//   | 32×24 int16 LE pixels (°C × 10)
static void emitThermalFrame() {
    uint8_t zone  = acoustic_leak ? 2u : 0u;
    uint8_t flags = (acoustic_leak ? 1u : 0u) | (thermal_anomaly ? 2u : 0u);
    uint32_t ts   = millis();
    uint8_t buf[1548];
    buf[0] = 0xFF; buf[1] = 0xFE;
    memcpy(&buf[2], &snr_db, 4);
    memcpy(&buf[6], &ts,     4);
    buf[10] = zone;
    buf[11] = flags;
    for (int p = 0; p < THERMAL_TX_W * THERMAL_TX_H; p++) {
        int16_t px = (int16_t)lroundf(thermal_frame[p] * 10.0f);
        memcpy(&buf[12 + p * 2], &px, 2);
    }
    Serial1.write(buf, 1548);
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

// ── Experiment logging and commands ───────────────────────────
static int leakIntensity() {
    if (!acoustic_leak) return 0;

    float low = activeProfile->snrThresholdDb;
    float high = activeProfile->ledMaxSnrDb;
    if (high <= low) high = low + 1.0f;

    float normalized = clamp01((snr_db - low) / (high - low));
    if (normalized < 0.25f) return 1;
    if (normalized < 0.50f) return 2;
    if (normalized < 0.75f) return 3;
    return 4;
}

static void printStatusRow() {
    if (acoustic_leak) {
        Serial.printf("LEAK=YES INTENSITY=%d SNR=%.1f/%.1fdB SPEC=%.1f PEAKSNR=%.1f PINPOINT=%.1f AUDIBLE=%.1f RISE=%.1f SIGDB=%.1f AMBDB=%.1f CONF=%d PEAK=%.1fkHz\n",
                      leakIntensity(), snr_db, activeProfile->snrThresholdDb,
                      spectral_snr_db, peak_snr_db, pinpoint_metric_db, audible_metric_db, level_rise_db,
                      signal_level_db, ambient_level_db, confirm_count,
                      peak_freq / 1000.0f);
        return;
    }

    Serial.printf("LEAK=NO SNR=%.1f/%.1fdB SPEC=%.1f PEAKSNR=%.1f PINPOINT=%.1f AUDIBLE=%.1f RISE=%.1f SIGDB=%.1f AMBDB=%.1f CONF=%d PEAK=%.1fkHz\n",
                  snr_db, activeProfile->snrThresholdDb,
                  spectral_snr_db, peak_snr_db, pinpoint_metric_db, audible_metric_db, level_rise_db,
                  signal_level_db, ambient_level_db, confirm_count,
                  peak_freq / 1000.0f);
}

static void printActiveProfile() {
    Serial.printf("Experiment profile: %d %s\n", activeProfileId, activeProfile->name);
    Serial.printf("Signal band: %.0f-%.0f Hz\n",
                  activeProfile->signalBandLowHz, activeProfile->signalBandHighHz);
    Serial.printf("Noise band: %.0f-%.0f Hz\n",
                  activeProfile->noiseBandLowHz, activeProfile->noiseBandHighHz);
    Serial.printf("SNR threshold: %.1f dB\n", activeProfile->snrThresholdDb);
    if (activeProfile->windowSize > 0)
        Serial.printf("Confirm: %d/%d frames (sliding window), release at <=%d\n",
                      activeProfile->leakConfirmCount, activeProfile->windowSize,
                      activeProfile->leakReleaseCount);
    else
        Serial.printf("Confirm: %d consecutive frames\n", activeProfile->leakConfirmCount);
    Serial.printf("Noise alpha: %.3f\n", activeProfile->noiseFloorAlpha);
    Serial.printf("Ambient alpha: %.3f\n", activeProfile->ambientAlpha);
    Serial.printf("LED max SNR: %.1f dB\n", activeProfile->ledMaxSnrDb);
    Serial.printf("Proportional LED: %s\n", activeProfile->useProportionalLed ? "yes" : "no");
    Serial.printf("Sample rate: %.1f Hz  Nyquist: %.1f Hz  PDM clock: %.3f MHz\n",
                  SAMPLE_RATE, SAMPLE_RATE * 0.5f, SAMPLE_RATE * 64.0f / 1000000.0f);
    Serial.printf("FFT bin: %.1f Hz  signal bins: %d-%d  noise bins: %d-%d\n",
                  BIN_RES, sig_bin_low, sig_bin_high, noise_bin_low, noise_bin_high);
}

static void printHelp() {
    Serial.println("COMMANDS:");
    Serial.printf("0-%d choose profile\n", PROFILE_COUNT - 1);
    Serial.println("r reset before testing");
    Serial.println("p show profile");
    Serial.println("s show simple summary");
    Serial.println("h help");
    Serial.println("Output: LEAK=NO ... or LEAK=YES INTENSITY=1-4");
}

static void printSummary() {
    float avg = fft_frame_count > 0 ? sum_snr_db / (float)fft_frame_count : 0.0f;
    Serial.println("SUMMARY:");
    Serial.printf("profile_id=%d\n", activeProfileId);
    Serial.printf("profile_name=%s\n", activeProfile->name);
    Serial.printf("frames=%lu\n", (unsigned long)fft_frame_count);
    Serial.printf("above_threshold=%lu\n", (unsigned long)above_threshold_count);
    Serial.printf("confirmed_frames=%lu\n", (unsigned long)confirmed_frame_count);
    Serial.printf("max_snr_db=%.1f\n", max_snr_db);
    Serial.printf("avg_snr_db=%.1f\n", avg);
}

static void handleSerialCommands() {
    while (Serial.available() > 0) {
        char cmd = (char)Serial.read();
        if (cmd == '\r' || cmd == '\n' || cmd == ' ') continue;

        if (cmd >= '0' && cmd <= '0' + PROFILE_COUNT - 1) {
            int newProfileId = cmd - '0';
            setActiveProfile(newProfileId, true);
            Serial.printf("EVENT,profile_changed,%d,%s\n", activeProfileId, activeProfile->name);
            printActiveProfile();
            continue;
        }

        switch (cmd) {
            case 'h':
                printHelp();
                break;
            case 'p':
                printActiveProfile();
                break;
            case 'r':
                resetAcousticState();
                Serial.println("EVENT,acoustic_reset");
                break;
            case 's':
                printSummary();
                break;
            default:
                Serial.printf("EVENT,unknown_command,%c\n", cmd);
                printHelp();
                break;
        }
    }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);
    while (!Serial && millis() < 3000);

    setActiveProfile(EXPERIMENT_PROFILE_ID, false);

    Serial.println("=== PSSS Leak Detector ===");
    printActiveProfile();
    printHelp();

    arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);
    for (int i = 0; i < FFT_SIZE; i++)
        hann[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_SIZE - 1)));
    resetAcousticState();

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

    Serial.printf("Monitoring. SNR >= %.1f dB triggers detection.\n",
                  activeProfile->snrThresholdDb);
    Serial.println("Move the microphone around the tire.");
    Serial.println("Watch NeoPixel: green/yellow = searching, red = leak.");
    Serial.println("Serial output shows leak intensity from 1-4 only after detection.");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
    handleSerialCommands();
    bool new_fft = runAcoustic();
    updateLED(new_fft);
    runControl();
    runThermal();

    unsigned long now = millis();
    if (now - last_print_ms >= PRINT_INTERVAL_MS) {
        last_print_ms = now;
        printStatusRow();
    }
}
