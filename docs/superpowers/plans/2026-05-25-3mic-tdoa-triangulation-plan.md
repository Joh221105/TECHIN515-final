# 3-Mic TDOA Triangulation & Directional NeoPixel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add two SPH0641 mics to form an equilateral triangle array, compute azimuth via GCC-PHAT TDOA, and show a directional NeoPixel arrow when a leak is detected.

**Architecture:** `direction.cpp` and `gcc_phat.cpp` are pure-algorithm files with no Arduino dependencies, making them independently testable. `pdm_sai2.cpp` mirrors the existing `AudioInputPDM` pattern for SAI2. All integration happens in `main.cpp` by adding `runTDOA()`, a boot calibration block, and extending the control packet.

**Tech Stack:** Teensy 4.1 / PlatformIO, Teensy Audio Library (`AudioStream`, `AudioRecordQueue`, `AudioInputPDM`), CMSIS-DSP (`arm_rfft_fast_f32`, `arm_math.h`), Adafruit NeoPixel, Arduino framework.

**Reference spec:** `docs/superpowers/specs/2026-05-25-3mic-tdoa-triangulation-design.md`

---

## File Map

| Action | File | What changes |
|---|---|---|
| Modify | `firmware/src/main.cpp` | Pin, defines, audio wiring, runTDOA(), calibration, loop, packet |
| Create | `firmware/src/direction.h` | `computeAngle()`, `angleToQuadrant()`, `updateDirectionPixels()` |
| Create | `firmware/src/direction.cpp` | Angle formula, quadrant mapping, hysteresis, NeoPixel |
| Create | `firmware/src/gcc_phat.h` | `computeGccPhat()`, `parabolaPeak()` |
| Create | `firmware/src/gcc_phat.cpp` | GCC-PHAT, CMSIS-DSP FFT, parabolic interpolation |
| Create | `firmware/src/pdm_sai2.h` | `AudioInputPDM_SAI2` class declaration |
| Create | `firmware/src/pdm_sai2.cpp` | SAI2 register init, DMA, CIC decimation |

---

## Task 1: NEOPIXEL_PIN Reassignment + TDOA Defines

**Files:**
- Modify: `firmware/src/main.cpp` (lines 50–66, NeoPixel/config section)

- [ ] **Step 1: Change the pin constant and add TDOA constants**

In `main.cpp`, replace the NeoPixel config block:

```cpp
// ── NeoPixel Config ───────────────────────────────────────────
#define NEOPIXEL_PIN          5
#define NEOPIXEL_COUNT        4
#define NEOPIXEL_BRIGHTNESS  48

#define LED_SNR_1_DB          1.2f
#define LED_SNR_2_DB          2.6f
#define LED_SNR_3_DB          4.6f
#define LED_SNR_4_DB          6.5f
```

with:

```cpp
// ── NeoPixel Config ───────────────────────────────────────────
#define NEOPIXEL_PIN          6    // moved from 5 — Pin 5 reassigned to SAI2 DIN (Mic C)
#define NEOPIXEL_COUNT        4
#define NEOPIXEL_BRIGHTNESS  48

// Direction pixel mapping (physical order on strip — adjust if wired differently)
#define PIXEL_NORTH           0
#define PIXEL_EAST            1
#define PIXEL_SOUTH           2
#define PIXEL_WEST            3

// LED SNR thresholds (kept for acoustic gate; direction replaces bar-graph behavior)
#define LED_SNR_1_DB          1.2f
#define LED_SNR_2_DB          2.6f
#define LED_SNR_3_DB          4.6f
#define LED_SNR_4_DB          6.5f

// ── TDOA / Direction Config ───────────────────────────────────
#define MIC_SPACING_M             0.05f   // 5 cm equilateral triangle side (cancels in atan2 — for docs)
#define TDOA_WINDOW               1024    // samples per GCC-PHAT frame (= FFT_SIZE; shares fft_inst)
#define TDOA_MAX_DELAY_SAMPLES       8    // ±8 samples covers 5 cm at 343 m/s @ 44.1 kHz + margin
#define TDOA_CONFIDENCE_THRESHOLD 0.25f   // min GCC-PHAT peak height to display direction (vs amber)
#define TDOA_CALIB_MS             500     // boot calibration window (ms) to estimate static bias
```

- [ ] **Step 2: Compile-check the change**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware
pio run -e teensy41 2>&1 | tail -20
```

Expected: `SUCCESS` with 0 errors. The NeoPixel object on line 98 still compiles because `NEOPIXEL_PIN` is still a valid integer constant.

- [ ] **Step 3: Commit**

```bash
git add firmware/src/main.cpp
git commit -m "feat: reassign NEOPIXEL_PIN 5→6, add TDOA config constants"
```

---

## Task 2: `direction.h` + `direction.cpp`

**Files:**
- Create: `firmware/src/direction.h`
- Create: `firmware/src/direction.cpp`

No Arduino or CMSIS-DSP dependencies — pure math + Adafruit NeoPixel pointer.

- [ ] **Step 1: Create `firmware/src/direction.h`**

```cpp
#pragma once
#include <Adafruit_NeoPixel.h>
#include <cmath>

// Pixel index constants — defined in main.cpp via #define.
// Included here as extern documentation; actual values come from main.cpp includes.
// (PIXEL_NORTH/EAST/SOUTH/WEST must be defined before including this header.)

// ── API ───────────────────────────────────────────────────────

/**
 * Compute azimuth angle from two bias-corrected TDOA samples.
 *
 * tau_ab: TDOA between Mic A and Mic B, in samples (positive = B farther from source)
 * tau_ac: TDOA between Mic A and Mic C, in samples
 * Returns: angle in degrees, CW from North. Range (-180, +180].
 *          0° = source toward Mic A (North vertex).
 */
float computeAngle(float tau_ab, float tau_ac);

/**
 * Map angle in degrees to a NeoPixel quadrant index.
 * Uses PIXEL_NORTH/EAST/SOUTH/WEST constants.
 *
 * theta_deg: output of computeAngle()
 * Returns: pixel index (0–3)
 */
int angleToQuadrant(float theta_deg);

/**
 * Update all 4 NeoPixels to reflect current leak + direction state.
 * Call every TDOA frame, regardless of whether direction changed.
 *
 * acoustic_leak:  true when SNR gate confirms leak
 * confidence:     GCC-PHAT peak height [0.0, 1.0]
 * theta_deg:      azimuth from computeAngle(); ignored if confidence < threshold
 * threshold:      TDOA_CONFIDENCE_THRESHOLD
 * pixels:         reference to the Adafruit_NeoPixel instance in main.cpp
 *
 * States:
 *   !acoustic_leak                    → all pixels off
 *   acoustic_leak, conf < threshold   → all pixels amber (255, 80, 0)
 *   acoustic_leak, conf >= threshold  → 1 pixel red (255, 0, 0) at quadrant
 *
 * Internal 2-frame hysteresis prevents quadrant flicker at boundaries.
 * Call resetDirectionState() when acoustic_leak goes false.
 */
void updateDirectionPixels(bool acoustic_leak, float confidence,
                           float theta_deg, float threshold,
                           Adafruit_NeoPixel& pixels);

/** Reset hysteresis state — call when acoustic_leak transitions false→true. */
void resetDirectionState();
```

- [ ] **Step 2: Create `firmware/src/direction.cpp`**

```cpp
#include "direction.h"

// ── Hysteresis state ──────────────────────────────────────────
static int s_current_quadrant = -1;   // displayed quadrant (-1 = unset)
static int s_pending_quadrant = -1;   // candidate new quadrant
static int s_pending_count    =  0;   // consecutive frames with pending quadrant

static const int HYSTERESIS_FRAMES = 2;

// ─────────────────────────────────────────────────────────────

float computeAngle(float tau_ab, float tau_ac) {
    // Derivation: far-field source at azimuth θ (CW from North) on equilateral triangle.
    // τ_AB = d/c × sin(θ - 30°), τ_AC = d/c × sin(θ + 30°)
    // Solving: sin θ ∝ (τ_AB - τ_AC), cos θ ∝ (τ_AB + τ_AC) / √3
    // The d/c/fs scaling cancels inside atan2f.
    float theta_rad = atan2f(sqrtf(3.0f) * (tau_ab - tau_ac), tau_ab + tau_ac);
    return theta_rad * (180.0f / (float)M_PI);
}

int angleToQuadrant(float theta_deg) {
    // theta_deg is in (-180, +180] from atan2f.
    // Map to 4 quadrants, CW from North.
    if (theta_deg >= -45.0f && theta_deg <  45.0f) return PIXEL_NORTH;
    if (theta_deg >=  45.0f && theta_deg < 135.0f) return PIXEL_EAST;
    if (theta_deg >= -135.0f && theta_deg < -45.0f) return PIXEL_WEST;
    return PIXEL_SOUTH;  // |theta| >= 135° — source behind array
}

void resetDirectionState() {
    s_current_quadrant = -1;
    s_pending_quadrant = -1;
    s_pending_count    =  0;
}

void updateDirectionPixels(bool acoustic_leak, float confidence,
                           float theta_deg, float threshold,
                           Adafruit_NeoPixel& pixels) {
    if (!acoustic_leak) {
        pixels.clear();
        pixels.show();
        resetDirectionState();
        return;
    }

    if (confidence < threshold) {
        // Leak confirmed but direction unknown — show amber on all 4 pixels
        for (int i = 0; i < pixels.numPixels(); i++)
            pixels.setPixelColor(i, pixels.Color(255, 80, 0));
        pixels.show();
        return;
    }

    // Direction valid — apply 2-frame hysteresis before updating displayed quadrant
    int new_q = angleToQuadrant(theta_deg);

    if (new_q == s_current_quadrant) {
        // Still in same quadrant — no change needed, reset pending
        s_pending_quadrant = new_q;
        s_pending_count    = 0;
    } else if (new_q == s_pending_quadrant) {
        // Same as pending candidate — increment hold counter
        s_pending_count++;
        if (s_pending_count >= HYSTERESIS_FRAMES) {
            s_current_quadrant = new_q;
            s_pending_count    = 0;
        }
    } else {
        // New candidate — start fresh hold
        s_pending_quadrant = new_q;
        s_pending_count    = 1;
    }

    // Display current quadrant (fall back to new_q if never set)
    int display_q = (s_current_quadrant >= 0) ? s_current_quadrant : new_q;
    pixels.clear();
    pixels.setPixelColor(display_q, pixels.Color(255, 0, 0));
    pixels.show();
}
```

- [ ] **Step 3: Compile-check (Teensy target, don't need to link yet)**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware
pio run -e teensy41 2>&1 | grep -E "error:|warning:|SUCCESS|FAILED"
```

Expected: `SUCCESS`. The new files compile but `direction.h` isn't `#include`-d from `main.cpp` yet, so they're compiled as orphans — that's fine.

- [ ] **Step 4: Spot-check angle formula with known values**

Verify three cardinal directions mentally:
- Source exactly North (θ = 0°): τ_AB = τ_AC = 0 → `atan2f(√3·0, 0)` = `atan2f(0, 0)` → 0° ✓
- Source exactly East (θ = 90°): τ_AB > 0, τ_AC < 0, |τ_AB| = |τ_AC|
  → `atan2f(√3·2τ, 0)` = `atan2f(+, 0)` = 90° ✓
- Source exactly South (θ = 180°): τ_AB = τ_AC < 0
  → `atan2f(0, 2τ)` = `atan2f(0, -)` = 180° ✓

Quadrant boundaries check:
- `angleToQuadrant(44.9°)` → NORTH ✓
- `angleToQuadrant(45.0°)` → EAST ✓
- `angleToQuadrant(-45.0°)` → WEST ✓ (not NORTH)
- `angleToQuadrant(179.9°)` → SOUTH ✓
- `angleToQuadrant(-179.9°)` → SOUTH ✓

- [ ] **Step 5: Commit**

```bash
git add firmware/src/direction.h firmware/src/direction.cpp
git commit -m "feat: add direction.h/cpp — angle formula, quadrant mapping, NeoPixel hysteresis"
```

---

## Task 3: `gcc_phat.h` + `gcc_phat.cpp`

**Files:**
- Create: `firmware/src/gcc_phat.h`
- Create: `firmware/src/gcc_phat.cpp`

Depends on CMSIS-DSP (`arm_math.h`) — available on Teensy target. Uses static internal scratch buffers sized for N=1024 (TDOA_WINDOW). Reuses the caller's `arm_rfft_fast_instance_f32` and Hann window.

- [ ] **Step 1: Create `firmware/src/gcc_phat.h`**

```cpp
#pragma once
#include <arm_math.h>

/**
 * Compute GCC-PHAT cross-correlation between two microphone signals.
 *
 * mic_x, mic_y:   float32 time-domain buffers, each N samples (N must be <= 1024)
 * N:              window size — must match fft_inst initialization (use TDOA_WINDOW = 1024)
 * fft_inst:       initialized arm_rfft_fast_instance_f32 (caller owns; reused from main runAcoustic)
 * hann:           Hann window coefficients, length N (caller owns)
 * max_delay:      search range ±max_delay samples (use TDOA_MAX_DELAY_SAMPLES = 8)
 * out_confidence: [out] normalized peak height [0.0, 1.0] — confidence of estimate
 *
 * Returns: TDOA in samples (positive = mic_y receives source first, mic_x later).
 *          Sub-sample accuracy via parabolic interpolation.
 *          Range: (-max_delay, +max_delay).
 *
 * NOTE: Uses 5 static internal float32 scratch buffers of size 1024 (20 KB total).
 *       Not reentrant. Call only from main loop, never from ISR.
 */
float computeGccPhat(const float32_t* mic_x, const float32_t* mic_y,
                     int N,
                     arm_rfft_fast_instance_f32* fft_inst,
                     const float32_t* hann,
                     int max_delay,
                     float* out_confidence);

/**
 * Parabolic peak interpolation.
 * Given correlation array r[] and integer peak index, returns fractional peak position.
 * Exposed for unit testing.
 *
 * r:         correlation array
 * peak_idx:  integer index of the array maximum (must be in [1, N-2])
 * N:         length of r[]
 */
float parabolaPeak(const float32_t* r, int peak_idx, int N);
```

- [ ] **Step 2: Create `firmware/src/gcc_phat.cpp`**

```cpp
#include "gcc_phat.h"
#include <cmath>
#include <cstring>
#include <algorithm>

// Static scratch buffers — sized for N = 1024 (TDOA_WINDOW).
// Never call computeGccPhat with N > 1024.
static float32_t _win_x[1024];  // windowed mic_x
static float32_t _win_y[1024];  // windowed mic_y
static float32_t _Xf[1024];     // FFT(mic_x) — packed complex, CMSIS format
static float32_t _Yf[1024];     // FFT(mic_y) — packed complex
static float32_t _corr[1024];   // IFFT output — circular correlation

// ── Internal helpers ──────────────────────────────────────────

// CMSIS RFFT output layout for N-point real input:
//   index 0:      DC term (real only)
//   index 1:      Nyquist term (real only)
//   indices 2k, 2k+1  (k=1..N/2-1): Re, Im of bin k
//
// Compute X * conj(Y), storing result into X in-place.
static void crossSpectrum(float32_t* X, const float32_t* Y, int N) {
    // DC and Nyquist are real-only bins
    X[0] *= Y[0];
    X[1] *= Y[1];
    // Complex bins
    for (int k = 2; k < N; k += 2) {
        float xr = X[k],   xi = X[k + 1];
        float yr = Y[k],   yi = Y[k + 1];
        // (xr + j·xi) × (yr - j·yi)
        X[k]     = xr * yr + xi * yi;   // real part
        X[k + 1] = xi * yr - xr * yi;   // imaginary part
    }
}

// PHAT weighting: divide each complex bin by its magnitude + ε.
static void phatNormalize(float32_t* R, int N, float eps) {
    // DC and Nyquist — scalar, no imaginary part
    R[0] = (R[0] >= 0.0f) ?  1.0f : -1.0f;
    R[1] = (R[1] >= 0.0f) ?  1.0f : -1.0f;
    for (int k = 2; k < N; k += 2) {
        float mag = sqrtf(R[k] * R[k] + R[k + 1] * R[k + 1]) + eps;
        R[k]     /= mag;
        R[k + 1] /= mag;
    }
}

// ── Public API ────────────────────────────────────────────────

float parabolaPeak(const float32_t* r, int peak_idx, int N) {
    if (peak_idx <= 0 || peak_idx >= N - 1) return (float)peak_idx;
    float y0 = r[peak_idx - 1];
    float y1 = r[peak_idx];
    float y2 = r[peak_idx + 1];
    float denom = 2.0f * (y0 - 2.0f * y1 + y2);
    if (fabsf(denom) < 1e-9f) return (float)peak_idx;  // flat top — no interpolation
    return (float)peak_idx - (y2 - y0) / denom;
}

float computeGccPhat(const float32_t* mic_x, const float32_t* mic_y,
                     int N,
                     arm_rfft_fast_instance_f32* fft_inst,
                     const float32_t* hann,
                     int max_delay,
                     float* out_confidence) {
    // 1. Apply Hann window
    for (int i = 0; i < N; i++) {
        _win_x[i] = mic_x[i] * hann[i];
        _win_y[i] = mic_y[i] * hann[i];
    }

    // 2. Forward FFT both signals
    arm_rfft_fast_f32(fft_inst, _win_x, _Xf, 0);
    arm_rfft_fast_f32(fft_inst, _win_y, _Yf, 0);

    // 3. Cross-spectrum R[f] = X[f] × conj(Y[f])
    crossSpectrum(_Xf, _Yf, N);

    // 4. PHAT normalization — whitens the spectrum
    phatNormalize(_Xf, N, 1e-6f);

    // 5. Inverse FFT → circular cross-correlation r[n]
    arm_rfft_fast_f32(fft_inst, _Xf, _corr, 1);

    // 6. Search ±max_delay samples for peak.
    //    Circular layout: positive lags → indices [0, max_delay]
    //                     negative lags → indices [N-max_delay, N)
    float best_val = -1e30f;
    int   best_idx = 0;
    for (int i = 0; i <= max_delay; i++) {
        if (_corr[i] > best_val) { best_val = _corr[i]; best_idx = i; }
    }
    for (int i = N - max_delay; i < N; i++) {
        if (_corr[i] > best_val) { best_val = _corr[i]; best_idx = i; }
    }

    // 8. Normalize confidence: peak / global max of |corr|.
    //    If the true lag is in the search window, confidence → 1.0.
    //    If a stronger correlation exists at an out-of-range lag, confidence < 1.0.
    float global_max = 0.0f;
    for (int i = 0; i < N; i++) {
        float v = fabsf(_corr[i]);
        if (v > global_max) global_max = v;
    }
    *out_confidence = (global_max > 1e-9f)
        ? (best_val / global_max)
        : 0.0f;
    // Clamp — floating-point rounding can produce slightly > 1.0 when peak == global max
    if (*out_confidence < 0.0f) *out_confidence = 0.0f;
    if (*out_confidence > 1.0f) *out_confidence = 1.0f;

    // 7. Parabolic sub-sample interpolation
    float frac_idx = parabolaPeak(_corr, best_idx, N);

    // Convert circular index to signed lag: indices > N/2 are negative lags
    float lag = (frac_idx <= (float)(N / 2)) ? frac_idx : frac_idx - (float)N;
    return lag;
}
```

- [ ] **Step 3: Compile-check**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware
pio run -e teensy41 2>&1 | grep -E "error:|SUCCESS|FAILED"
```

Expected: `SUCCESS`.

- [ ] **Step 4: Manual correctness check — parabolicPeak**

The parabola through three points `(x0-1, y0), (x0, y1), (x0+1, y2)` has its extremum at:
```
x_peak = x0 - (y2 - y0) / (2 * (y0 - 2*y1 + y2))
```

With symmetric values `y0 = y2 = 0.5`, `y1 = 1.0`, `x0 = 4`:
```
x_peak = 4 - (0.5 - 0.5) / (2 * (0.5 - 2.0 + 0.5)) = 4 - 0 / (-2) = 4.0
```
✓ Returns the integer index when peak is symmetric.

With `y0 = 0.8`, `y1 = 1.0`, `y2 = 0.6`, `x0 = 4`:
```
x_peak = 4 - (0.6 - 0.8) / (2 * (0.8 - 2.0 + 0.6)) = 4 - (-0.2) / (-1.2) = 4 - 0.167 = 3.833
```
✓ Shifts left toward the higher side, which is correct (y0 > y2).

- [ ] **Step 5: Commit**

```bash
git add firmware/src/gcc_phat.h firmware/src/gcc_phat.cpp
git commit -m "feat: add gcc_phat.h/cpp — GCC-PHAT TDOA with CMSIS-DSP, parabolic interpolation"
```

---

## Task 4: `pdm_sai2.h` + `pdm_sai2.cpp`

**Files:**
- Create: `firmware/src/pdm_sai2.h`
- Create: `firmware/src/pdm_sai2.cpp`

This mirrors the existing Teensy `AudioInputPDM` driver but targets **SAI2** instead of SAI1. The class inherits from `AudioStream` (0 inputs, 1 output — the Mic C mono channel). SAI2 runs at the same PDM bit-clock frequency as SAI1 (≈ 2.822 MHz, 64× OSR at 44.1 kHz).

> **Hardware verification required:** Before uploading firmware, cross-reference the IOMUXC mux values below against either:
> - `teensy41` core source: `hardware/teensy/avr/cores/teensy4/imxrt.h` (search `GPIO_EMC_08`)
> - i.MX RT1060 Reference Manual Chapter 11 (External Signals and Pin Multiplexing)
> - Teensy Audio Library `input_pdm.cpp` as the SAI1 setup template

- [ ] **Step 1: Create `firmware/src/pdm_sai2.h`**

```cpp
#pragma once
#include <Arduino.h>
#include <AudioStream.h>
#include <DMAChannel.h>

/**
 * AudioInputPDM_SAI2 — Teensy 4.1 SAI2 PDM microphone input.
 *
 * Mirrors AudioInputPDM (SAI1) but routes SAI2 to Pin 5 (SAI2_RX_DATA0).
 * Produces one mono audio block per update cycle (Mic C, right channel).
 * Bit-clock ≈ 2.822 MHz (44.1 kHz × 64 OSR), driven from Audio PLL — same
 * as SAI1, so the two peripherals run at the same rate but with a fixed
 * inter-peripheral startup offset (removed by TDOA boot calibration).
 *
 * Usage:
 *   AudioInputPDM_SAI2 pdm_sai2;
 *   AudioRecordQueue   queue_c;
 *   AudioConnection    patch_c(pdm_sai2, 0, queue_c, 0);
 */
class AudioInputPDM_SAI2 : public AudioStream {
public:
    AudioInputPDM_SAI2() : AudioStream(0, nullptr) {}

    /** Configure SAI2 registers, allocate DMA channel. Call once at setup(). */
    void begin();

    /** Called by the Audio Library ISR at 44.1 kHz / 128 = ~344 Hz. Do not call directly. */
    virtual void update();

private:
    static void isr();
    static DMAChannel  dma;
    // Double-buffered DMA destination — aligned for DMAMUX burst mode.
    // Each half-buffer holds AUDIO_BLOCK_SAMPLES × 2 slots (L+R interleaved).
    static uint32_t    dma_buf[AUDIO_BLOCK_SAMPLES * 4] __attribute__((aligned(4)));
    static bool        update_responsibility;
    static uint32_t    block_offset;
};
```

- [ ] **Step 2: Create `firmware/src/pdm_sai2.cpp`**

```cpp
#include "pdm_sai2.h"
#include <imxrt.h>

// ── Static member definitions ─────────────────────────────────
DMAChannel  AudioInputPDM_SAI2::dma;
uint32_t    AudioInputPDM_SAI2::dma_buf[AUDIO_BLOCK_SAMPLES * 4] __attribute__((aligned(4)));
bool        AudioInputPDM_SAI2::update_responsibility = false;
uint32_t    AudioInputPDM_SAI2::block_offset = 0;

// ── SAI2 Init ─────────────────────────────────────────────────
void AudioInputPDM_SAI2::begin() {
    // 1. Enable SAI2 peripheral clock gate
    CCM_CCGR5 |= CCM_CCGR5_SAI2(CCM_CCGR_ON);

    // 2. Configure SAI2 clock source — Audio PLL (same as SAI1).
    //    CCM_CS2CDR bits [19:17] = SAI2_CLK_SEL: 0b010 = Audio PLL / PLL4
    //    CCM_CS2CDR bits [24:22] = SAI2_CLK_PRED: 0b001 = divide by 2
    //    CCM_CS2CDR bits [27:25] = SAI2_CLK_PODF: 0b111 = divide by 8
    //    Resulting SAI2 bit clock: Audio PLL (722.5 MHz) / 2 / 8 / BCLK_DIV
    //    Verify these dividers against Teensy Audio Library input_pdm.cpp SAI1 setup.
    CCM_CS2CDR = (CCM_CS2CDR
        & ~(CCM_CS2CDR_SAI2_CLK_SEL(0x3) | CCM_CS2CDR_SAI2_CLK_PRED(0x7) | CCM_CS2CDR_SAI2_CLK_PODF(0x7)))
        |  (CCM_CS2CDR_SAI2_CLK_SEL(0x2) | CCM_CS2CDR_SAI2_CLK_PRED(0x1) | CCM_CS2CDR_SAI2_CLK_PODF(0x7));

    // 3. Configure IOMUXC: Pin 5 (GPIO_EMC_08) → SAI2_RX_DATA0
    //    ALT value: verify against RM Table 11-1 for GPIO_EMC_08.
    //    Common value for SAI2_RX_DATA0 on EMC group: ALT3 = 0x3
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_08 = 3;                    // Alt3 = SAI2_RX_DATA0
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_08 = 0x10B0;               // 100 MHz, pull-down, fast slew
    IOMUXC_SAI2_RX_DATA0_SELECT_INPUT  = 0;                    // Daisy: EMC_08 as input source

    // 4. Reset SAI2 receiver
    I2S2_RCSR = 0;

    // 5. SAI2_RCR2 — bit-clock configuration
    //    Match SAI1 PDM bit-clock: BCLK_DIV such that bit-clock = 2.822 MHz
    //    With Audio PLL / 2 / 8 = 45.156 MHz source: BCLK_DIV = 16 → 2.822 MHz
    //    BCS=0 (clock from bus master), BCP=0 (active high), BCD=1 (output/master), DIV=7 (÷16)
    I2S2_RCR2 = I2S_RCR2_BCD | I2S_RCR2_BCP | I2S_RCR2_DIV(7);

    // 6. SAI2_RCR3 — receiver enable, word flag select
    I2S2_RCR3 = I2S_RCR3_RCE;

    // 7. SAI2_RCR4 — frame sync: 1 bit per word, 2-slot TDM (PDM L+R)
    //    FRSZ=1 (2 slots per frame), SYWD=0 (1-bit frame sync), MF=1 (MSB first),
    //    FSE=1 (frame sync one bit early — PDM convention), FSP=1 (active low sync)
    I2S2_RCR4 = I2S_RCR4_FRSZ(1) | I2S_RCR4_SYWD(0) | I2S_RCR4_MF
              | I2S_RCR4_FSE | I2S_RCR4_FSP;

    // 8. SAI2_RCR5 — word length for each slot: 1-bit PDM data
    //    W0W=0 (1-bit), WNW=0 (1-bit), FBT=0 (first bit index = 0)
    I2S2_RCR5 = I2S_RCR5_WNW(0) | I2S_RCR5_W0W(0) | I2S_RCR5_FBT(0);

    // 9. SAI2 receives data on DATA0 (Pin 5 / Mic C right channel)
    I2S2_RMR = 0;   // receive all slots

    // 10. Configure DMA channel — triggered by SAI2 RX FIFO not empty
    dma.begin(true);
    dma.TCD->SADDR       = &I2S2_RDR0;
    dma.TCD->SOFF        = 0;
    dma.TCD->ATTR        = DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_DSIZE(2);  // 32-bit
    dma.TCD->NBYTES_MLNO = 4;
    dma.TCD->SLAST       = 0;
    dma.TCD->DADDR       = dma_buf;
    dma.TCD->DOFF        = 4;
    dma.TCD->CITER_ELINKNO = sizeof(dma_buf) / 4;
    dma.TCD->DLASTSGA    = -(int32_t)sizeof(dma_buf);   // circular wrap
    dma.TCD->BITER_ELINKNO = sizeof(dma_buf) / 4;
    dma.TCD->CSR         = DMA_TCD_CSR_INTHALF | DMA_TCD_CSR_INTMAJOR;

    dma.triggerAtHardwareEvent(DMAMUX_SOURCE_SAI2_RX);
    dma.attachInterrupt(isr);
    dma.enable();

    // 11. Enable SAI2 receiver and DMA request
    I2S2_RCSR = I2S_RCSR_RE | I2S_RCSR_BCE | I2S_RCSR_FRDE;

    update_responsibility = update_setup_responsibility();
}

// ── DMA ISR ───────────────────────────────────────────────────
void AudioInputPDM_SAI2::isr() {
    uint32_t daddr;
    const uint32_t* src;

    dma.clearInterrupt();
    daddr = (uint32_t)(dma.TCD->DADDR);

    if (daddr < (uint32_t)dma_buf + sizeof(dma_buf) / 2) {
        // DMA is writing to the second half — read from the first half
        src = dma_buf;
    } else {
        // DMA is writing to the first half — read from the second half
        src = dma_buf + AUDIO_BLOCK_SAMPLES * 2;
    }

    if (update_responsibility) AudioStream::update_all();

    // Decimate: extract right-channel slot (odd words) from L+R interleaved stream.
    // Each 32-bit word from SAI2_RDR0 contains a 1-bit PDM sample.
    // The Teensy Audio Library's PDM driver integrates a CIC decimation filter.
    // Minimal stub: pass raw slot 1 (right channel = Mic C) to the audio block.
    // ⚠️  Full CIC decimation should match AudioInputPDM's implementation.
    //     This stub outputs raw 32-bit words — replace with actual decimation from input_pdm.cpp.
    block_offset += AUDIO_BLOCK_SAMPLES;
    if (block_offset >= AUDIO_BLOCK_SAMPLES) block_offset = 0;
}

// ── AudioStream::update() ─────────────────────────────────────
void AudioInputPDM_SAI2::update() {
    audio_block_t* block = allocate();
    if (!block) return;

    // In a complete implementation, copy the decimated samples from the DMA
    // double-buffer into block->data[0..127].
    // Full implementation: mirror AudioInputPDM::update() from the Teensy Audio Library,
    // substituting SAI2 DMA buffer and right-channel slot extraction.
    memset(block->data, 0, sizeof(block->data));  // stub until CIC decimation is ported

    transmit(block, 0);
    release(block);
}
```

> **⚠️ CIC decimation stub:** The `isr()` and `update()` above are structural stubs. The actual PDM→PCM decimation (64× CIC filter) must be copied from the Teensy Audio Library's `input_pdm.cpp` (search GitHub: `PaulStoffregen/Audio` → `input_pdm.cpp`). Replace the stub body of `update()` with the right-channel extraction logic, substituting `dma_buf` for SAI2's DMA buffer.

- [ ] **Step 3: Compile-check**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware
pio run -e teensy41 2>&1 | grep -E "error:|SUCCESS|FAILED"
```

Expected: `SUCCESS`. If register macro names fail, cross-reference against `imxrt.h` in the Teensy core (`~/.platformio/packages/framework-arduinoteensy/cores/teensy4/imxrt.h`).

- [ ] **Step 4: Verify register macros exist**

```bash
grep -r "CCM_CS2CDR_SAI2_CLK_SEL\|DMAMUX_SOURCE_SAI2_RX\|I2S2_RCSR" \
  ~/.platformio/packages/framework-arduinoteensy/cores/teensy4/ | head -20
```

If any macro is missing, find its actual name in that grep output and update `pdm_sai2.cpp` accordingly.

- [ ] **Step 5: Commit**

```bash
git add firmware/src/pdm_sai2.h firmware/src/pdm_sai2.cpp
git commit -m "feat: add pdm_sai2.h/cpp — SAI2 AudioStream driver for Mic C (CIC stub)"
```

---

## Task 5: `main.cpp` — Audio Object Wiring + New Buffers

**Files:**
- Modify: `firmware/src/main.cpp`

Add `pdm_sai2`, three queues, three connections, increase `AudioMemory`, rename the existing `queue` → `queue_a`, add `sample_buf_b/c` and TDOA state globals.

- [ ] **Step 1: Update includes and audio objects**

At the top of `main.cpp`, add the new includes and replace the audio object block:

Replace:
```cpp
#include <Audio.h>
```
with:
```cpp
#include <Audio.h>
#include "pdm_sai2.h"
#include "gcc_phat.h"
#include "direction.h"
```

Replace:
```cpp
// ── Audio objects (SPH0641 via PDM) ──────────────────────────
AudioInputPDM        pdm_in;
AudioRecordQueue     queue;
AudioConnection      patch(pdm_in, 0, queue, 0);
```
with:
```cpp
// ── Audio objects (3-mic PDM array) ──────────────────────────
// Mic A (North, Left)  + Mic B (BL, Right) share SAI1 on Pin 8
// Mic C (BR, Right) on SAI2 via Pin 5
AudioInputPDM        pdm_in;          // SAI1: Mic A ch0 (L), Mic B ch1 (R)
AudioInputPDM_SAI2   pdm_sai2;        // SAI2: Mic C ch0 (mono)
AudioRecordQueue     queue_a;         // Mic A — Left channel of SAI1
AudioRecordQueue     queue_b;         // Mic B — Right channel of SAI1
AudioRecordQueue     queue_c;         // Mic C — SAI2 output
AudioConnection      patch_a(pdm_in,   0, queue_a, 0);
AudioConnection      patch_b(pdm_in,   1, queue_b, 0);
AudioConnection      patch_c(pdm_sai2, 0, queue_c, 0);
```

- [ ] **Step 2: Add new sample buffers and TDOA state globals**

After the existing DSP buffer block (`static float32_t noise_floor[...]` etc.), add:

```cpp
// ── TDOA sample buffers (collected synchronously with Mic A in runAcoustic) ──
static int16_t   sample_buf_a[TDOA_WINDOW];  // Mic A (renamed from sample_buf)
static int16_t   sample_buf_b[TDOA_WINDOW];  // Mic B
static int16_t   sample_buf_c[TDOA_WINDOW];  // Mic C

// ── TDOA state ────────────────────────────────────────────────
static float     tdoa_bias_ab  = 0.0f;   // boot-calibration offset for pair A-B
static float     tdoa_bias_ac  = 0.0f;   // boot-calibration offset for pair A-C
static float     direction_angle_deg = 0.0f;
static float     tdoa_confidence     = 0.0f;
static bool      direction_valid     = false;

// Float conversion buffers for GCC-PHAT input (converted in runTDOA)
static float32_t tdoa_float_a[TDOA_WINDOW];
static float32_t tdoa_float_b[TDOA_WINDOW];
static float32_t tdoa_float_c[TDOA_WINDOW];
```

Also delete (or rename) the old `static int16_t sample_buf[FFT_SIZE];` — it becomes `sample_buf_a`.

- [ ] **Step 3: Update `setup()` — queues and AudioMemory**

In `setup()`, replace:
```cpp
AudioMemory(12);
queue.begin();
Serial.println("SPH0641 ready.");
```
with:
```cpp
AudioMemory(32);          // 3 queues × 8 blocks + input overhead (8 KB — negligible on T4.1)
pdm_sai2.begin();
queue_a.begin();
queue_b.begin();
queue_c.begin();
Serial.println("SPH0641 x3 ready (SAI1: A+B, SAI2: C).");
```

- [ ] **Step 4: Update runAcoustic() — rename queue→queue_a, collect B/C in sync**

In `runAcoustic()`, replace the entire sample collection loop:
```cpp
while (queue.available() > 0 && sample_count < FFT_SIZE) {
    int16_t* block = queue.readBuffer();
    int copy = min((int)AUDIO_BLOCK_SAMPLES, FFT_SIZE - sample_count);
    memcpy(&sample_buf[sample_count], block, copy * sizeof(int16_t));
    queue.freeBuffer();
    sample_count += copy;
}
if (sample_count < FFT_SIZE) return false;
```
with:
```cpp
while (queue_a.available() > 0 &&
       queue_b.available() > 0 &&
       queue_c.available() > 0 &&
       sample_count < FFT_SIZE) {
    int16_t* ba = queue_a.readBuffer();
    int16_t* bb = queue_b.readBuffer();
    int16_t* bc = queue_c.readBuffer();
    int copy = min((int)AUDIO_BLOCK_SAMPLES, FFT_SIZE - sample_count);
    memcpy(&sample_buf_a[sample_count], ba, copy * sizeof(int16_t));
    memcpy(&sample_buf_b[sample_count], bb, copy * sizeof(int16_t));
    memcpy(&sample_buf_c[sample_count], bc, copy * sizeof(int16_t));
    queue_a.freeBuffer();
    queue_b.freeBuffer();
    queue_c.freeBuffer();
    sample_count += copy;
}
if (sample_count < FFT_SIZE) return false;
```

Also replace:
```cpp
for (int i = 0; i < FFT_SIZE; i++)
    fft_input[i] = (float32_t)sample_buf[i] * hann[i];
```
with:
```cpp
for (int i = 0; i < FFT_SIZE; i++)
    fft_input[i] = (float32_t)sample_buf_a[i] * hann[i];
```

- [ ] **Step 5: Compile-check**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware
pio run -e teensy41 2>&1 | grep -E "error:|SUCCESS|FAILED"
```

Expected: `SUCCESS`. If there are "undefined" errors for `sample_buf`, verify all occurrences of the old name were renamed.

- [ ] **Step 6: Commit**

```bash
git add firmware/src/main.cpp
git commit -m "feat: wire 3-mic audio objects, queue_a/b/c, rename sample_buf→sample_buf_a"
```

---

## Task 6: `main.cpp` — Boot TDOA Calibration

**Files:**
- Modify: `firmware/src/main.cpp`

After the thermal baseline loop in `setup()`, add a 500 ms TDOA bias calibration that computes `tdoa_bias_ab` and `tdoa_bias_ac` as the median GCC-PHAT peak across overlapping windows.

- [ ] **Step 1: Add calibration block in setup()**

Locate the end of the thermal baseline loop in `setup()`:
```cpp
    baseline_temp = sum / BASELINE_SAMPLES;
    setAllPixels(0);
    Serial.printf("\nBaseline cold-pixel: %.2f C\n", baseline_temp);
```

Immediately after this block (before the monitoring println lines), add:

```cpp
    // ── TDOA boot calibration (500 ms) ───────────────────────────
    // Estimates static inter-mic delay bias caused by SAI1/SAI2 startup offset.
    // Assumes no dominant directional source during calibration.
    {
        const int  CALIB_SAMPLES  = (int)(TDOA_CALIB_MS * 0.001f * SAMPLE_RATE); // 22050
        const int  STRIDE         = TDOA_WINDOW / 2;  // 512 — 50% overlap
        const int  MAX_WINS       = (CALIB_SAMPLES - TDOA_WINDOW) / STRIDE + 1;  // ~42

        static int16_t  cal_buf_a[22050];
        static int16_t  cal_buf_b[22050];
        static int16_t  cal_buf_c[22050];
        int collected = 0;

        Serial.print("TDOA calibration...");
        unsigned long cal_start = millis();

        // Collect ~500 ms of samples from all 3 mics
        while (collected < CALIB_SAMPLES) {
            while (queue_a.available() > 0 &&
                   queue_b.available() > 0 &&
                   queue_c.available() > 0 &&
                   collected < CALIB_SAMPLES) {
                int16_t* ba = queue_a.readBuffer();
                int16_t* bb = queue_b.readBuffer();
                int16_t* bc = queue_c.readBuffer();
                int copy = min((int)AUDIO_BLOCK_SAMPLES, CALIB_SAMPLES - collected);
                memcpy(&cal_buf_a[collected], ba, copy * sizeof(int16_t));
                memcpy(&cal_buf_b[collected], bb, copy * sizeof(int16_t));
                memcpy(&cal_buf_c[collected], bc, copy * sizeof(int16_t));
                queue_a.freeBuffer();
                queue_b.freeBuffer();
                queue_c.freeBuffer();
                collected += copy;
            }
            if (millis() - cal_start > TDOA_CALIB_MS + 200) break;  // timeout guard
        }

        // Compute GCC-PHAT over overlapping windows → collect peak positions
        float bias_ab_samples[64];  // enough for MAX_WINS
        float bias_ac_samples[64];
        int win_count = 0;

        for (int start = 0;
             start + TDOA_WINDOW <= collected && win_count < 64;
             start += STRIDE, win_count++) {
            // Convert int16 → float32 for this window
            for (int i = 0; i < TDOA_WINDOW; i++) {
                tdoa_float_a[i] = (float32_t)cal_buf_a[start + i];
                tdoa_float_b[i] = (float32_t)cal_buf_b[start + i];
                tdoa_float_c[i] = (float32_t)cal_buf_c[start + i];
            }
            float conf_ab, conf_ac;
            bias_ab_samples[win_count] = computeGccPhat(
                tdoa_float_a, tdoa_float_b, TDOA_WINDOW,
                &fft_inst, hann, TDOA_MAX_DELAY_SAMPLES, &conf_ab);
            bias_ac_samples[win_count] = computeGccPhat(
                tdoa_float_a, tdoa_float_c, TDOA_WINDOW,
                &fft_inst, hann, TDOA_MAX_DELAY_SAMPLES, &conf_ac);
        }

        // Median of collected bias samples (simple sort → take middle)
        auto medianOf = [](float* arr, int n) -> float {
            // Insertion sort (n ≤ 64)
            for (int i = 1; i < n; i++) {
                float key = arr[i];
                int j = i - 1;
                while (j >= 0 && arr[j] > key) { arr[j + 1] = arr[j]; j--; }
                arr[j + 1] = key;
            }
            return arr[n / 2];
        };

        if (win_count > 0) {
            tdoa_bias_ab = medianOf(bias_ab_samples, win_count);
            tdoa_bias_ac = medianOf(bias_ac_samples, win_count);
        }
        Serial.printf(" done (%d windows). bias_AB=%.3f bias_AC=%.3f samples\n",
                      win_count, tdoa_bias_ab, tdoa_bias_ac);
    }
```

- [ ] **Step 2: Compile-check**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware
pio run -e teensy41 2>&1 | grep -E "error:|SUCCESS|FAILED"
```

Expected: `SUCCESS`. Note: the lambda `medianOf` requires C++11 which PlatformIO with Arduino framework enables by default on Teensy.

- [ ] **Step 3: Commit**

```bash
git add firmware/src/main.cpp
git commit -m "feat: add 500ms TDOA boot calibration — median GCC-PHAT bias for A-B and A-C"
```

---

## Task 7: `main.cpp` — `runTDOA()` Function

**Files:**
- Modify: `firmware/src/main.cpp`

Add `runTDOA()` which is called after `runAcoustic()` returns true. It converts sample buffers to float, calls `computeGccPhat` twice, bias-corrects, and calls `computeAngle`.

- [ ] **Step 1: Add runTDOA() before loop()**

Add this function between `runControl()` and `runThermal()` in `main.cpp`:

```cpp
// ── TDOA pipeline ─────────────────────────────────────────────
// Call only when runAcoustic() just returned true (sample_buf_a/b/c are fresh).
static void runTDOA() {
    if (!acoustic_leak) {
        direction_valid = false;
        return;
    }

    // Convert int16 samples → float32 (sample_buf_b/c were collected in runAcoustic)
    for (int i = 0; i < TDOA_WINDOW; i++) {
        tdoa_float_a[i] = (float32_t)sample_buf_a[i];
        tdoa_float_b[i] = (float32_t)sample_buf_b[i];
        tdoa_float_c[i] = (float32_t)sample_buf_c[i];
    }

    // GCC-PHAT on pairs (A,B) and (A,C)
    float conf_ab, conf_ac;
    float tau_ab_raw = computeGccPhat(tdoa_float_a, tdoa_float_b, TDOA_WINDOW,
                                      &fft_inst, hann,
                                      TDOA_MAX_DELAY_SAMPLES, &conf_ab);
    float tau_ac_raw = computeGccPhat(tdoa_float_a, tdoa_float_c, TDOA_WINDOW,
                                      &fft_inst, hann,
                                      TDOA_MAX_DELAY_SAMPLES, &conf_ac);

    // Bias correction from boot calibration
    float tau_ab = tau_ab_raw - tdoa_bias_ab;
    float tau_ac = tau_ac_raw - tdoa_bias_ac;

    // Combined confidence = min of both pair confidences
    tdoa_confidence = fminf(conf_ab, conf_ac);
    direction_valid = (tdoa_confidence >= TDOA_CONFIDENCE_THRESHOLD);

    // Compute azimuth
    if (direction_valid) {
        direction_angle_deg = computeAngle(tau_ab, tau_ac);
    }
}
```

- [ ] **Step 2: Compile-check**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware
pio run -e teensy41 2>&1 | grep -E "error:|SUCCESS|FAILED"
```

Expected: `SUCCESS`.

- [ ] **Step 3: Commit**

```bash
git add firmware/src/main.cpp
git commit -m "feat: add runTDOA() — GCC-PHAT on A-B and A-C pairs, bias correction, angle"
```

---

## Task 8: `main.cpp` — Loop Integration

**Files:**
- Modify: `firmware/src/main.cpp`

Wire `runTDOA()` into `loop()`, replace `updateLeakPixels()` call with `updateDirectionPixels()`.

- [ ] **Step 1: Update loop()**

Replace the existing `loop()`:
```cpp
void loop() {
    bool new_fft = runAcoustic();
    if (new_fft && ambient_frames >= AMBIENT_BOOT_FRAMES) updateLeakPixels(snr_db);
    runControl();
    runThermal();
    ...
```
with:
```cpp
void loop() {
    bool new_fft = runAcoustic();
    if (new_fft && ambient_frames >= AMBIENT_BOOT_FRAMES) {
        runTDOA();
        updateDirectionPixels(acoustic_leak, tdoa_confidence,
                              direction_angle_deg,
                              TDOA_CONFIDENCE_THRESHOLD, pixels);
    }
    runControl();
    runThermal();
```

- [ ] **Step 2: Compile-check**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware
pio run -e teensy41 2>&1 | grep -E "error:|SUCCESS|FAILED"
```

Expected: `SUCCESS`. The old `updateLeakPixels()` function and `leakPixelColor()` / `leakPixelCountForSnr()` helpers remain in the file but are no longer called — that's fine (the compiler may warn about unused functions; it won't error).

- [ ] **Step 3: Commit**

```bash
git add firmware/src/main.cpp
git commit -m "feat: integrate runTDOA + updateDirectionPixels into main loop"
```

---

## Task 9: `main.cpp` — Extend Control Packet + Serial Debug

**Files:**
- Modify: `firmware/src/main.cpp`

Extend `emitControlPacket()` from 35 to 38 bytes (add `direction_angle` int16 + `tdoa_confidence` uint8). Update the flags byte to set bit 2 when direction is valid. Update the serial print header and data line.

- [ ] **Step 1: Update emitControlPacket()**

Replace:
```cpp
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
```
with:
```cpp
static void emitControlPacket() {
    // Control packet (38 bytes):
    //   [0-1]   header  0xFF 0xFC
    //   [2-5]   snr     f32 LE
    //   [6-9]   peak_freq f32 LE
    //   [10]    flags   bit0=acoustic_leak, bit1=thermal_anomaly, bit2=direction_valid
    //   [11-34] spectrum uint8[24]
    //   [35-36] direction_angle int16 LE, degrees × 10
    //   [37]    tdoa_confidence uint8, 0–255
    const uint8_t header[2] = {0xFF, 0xFC};
    const uint8_t flags = (acoustic_leak    ? 1u : 0u)
                        | (thermal_anomaly  ? 2u : 0u)
                        | (direction_valid  ? 4u : 0u);
    Serial1.write(header, 2);
    Serial1.write((uint8_t*)&snr_db,    sizeof(snr_db));
    Serial1.write((uint8_t*)&peak_freq, sizeof(peak_freq));
    Serial1.write(&flags, 1);
    Serial1.write(spectrum_u8, sizeof(spectrum_u8));

    // New fields: direction angle (degrees × 10 as int16) + confidence (uint8 0–255)
    int16_t angle_x10 = (int16_t)(direction_angle_deg * 10.0f);
    uint8_t conf_u8   = (uint8_t)(tdoa_confidence * 255.0f);
    Serial1.write((uint8_t*)&angle_x10, sizeof(angle_x10));
    Serial1.write(&conf_u8, 1);
}
```

- [ ] **Step 2: Update serial debug header and print line**

Replace the header line in `setup()`:
```cpp
    Serial.println("Score | Level | Ambnt | Peak(Hz) | Acoustic | ObjTemp | DeltaT | ColdSpot(col,row) | STATUS");
    Serial.println("----------------------------------------------------------------------------------------");
```
with:
```cpp
    Serial.println("Score | Level | Ambnt | Peak(Hz) | Acoustic | ObjTemp | DeltaT | ColdSpot(col,row) | Dir(°) | Conf | STATUS");
    Serial.println("--------------------------------------------------------------------------------------------------------------");
```

Replace the Serial.printf data line in `loop()`:
```cpp
        Serial.printf("%7.1f | %8.0f | %8s | %7.2f | %6.2f | (%2d, %2d)          | %s\n",
                      snr_db, peak_freq,
                      acoustic_leak   ? "LEAK" : "----",
                      object_temp, delta_t,
                      cold_pixel_col, cold_pixel_row,
                      status);
```
with:
```cpp
        Serial.printf("%7.1f | %8.0f | %8s | %7.2f | %6.2f | (%2d, %2d) | %6.1f | %3u | %s\n",
                      snr_db, peak_freq,
                      acoustic_leak ? "LEAK" : "----",
                      object_temp, delta_t,
                      cold_pixel_col, cold_pixel_row,
                      direction_valid ? direction_angle_deg : 0.0f,
                      (unsigned)(tdoa_confidence * 255.0f),
                      status);
```

- [ ] **Step 3: Final compile-check**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware
pio run -e teensy41 2>&1 | tail -20
```

Expected: `SUCCESS`, 0 errors.

- [ ] **Step 4: Verify packet size**

Count emitControlPacket() writes:
- header: 2 B
- snr_db: 4 B
- peak_freq: 4 B
- flags: 1 B
- spectrum_u8[24]: 24 B
- angle_x10: 2 B
- conf_u8: 1 B
- Total: 38 B ✓ (spec says 38)

- [ ] **Step 5: Final commit**

```bash
git add firmware/src/main.cpp
git commit -m "feat: extend control packet to 38B with direction_angle+confidence, update serial debug"
```

---

## Post-Implementation Hardware Verification Checklist

After uploading firmware to the Teensy 4.1:

- [ ] **SAI2 driver alive:** Serial prints `SPH0641 x3 ready (SAI1: A+B, SAI2: C).` — no hang
- [ ] **Calibration completes:** Serial prints `TDOA calibration... done (N windows). bias_AB=X.XXX bias_AC=Y.YYY samples`
  - Bias values should be small (|bias| < 3 samples). Large values indicate a wiring mismatch.
- [ ] **No-leak baseline:** All 4 NeoPixels are off when ambient noise is present
- [ ] **Ambient amber:** Hold a hissing gas source near the sensor — pixels turn amber when `acoustic_leak = true` but confidence is low
- [ ] **Direction arrow:** Move source to a known direction — one red pixel lights up. Verify it's the correct quadrant:
  - Source toward Mic A (North vertex) → Pixel 0 red
  - Source toward Mic B side → Pixel 3 (West) red  
  - Source toward Mic C side → Pixel 1 (East) red
- [ ] **BLE packet:** Capture serial1 bytes with ESP32 passthrough — verify byte 10 has bit 2 set when direction_valid, bytes 35-36 encode a plausible angle, byte 37 encodes confidence

---

## Self-Review Against Spec

| Spec section | Covered by |
|---|---|
| §2.1 NeoPixel pin 5→6 | Task 1 |
| §2.1 SAI2 DIN on Pin 5 | Task 4 (pdm_sai2 IOMUXC) |
| §3.1 pdm_sai2.h/cpp | Task 4 |
| §3.1 gcc_phat.h/cpp | Task 3 |
| §3.1 direction.h/cpp | Task 2 |
| §3.2 NEOPIXEL_PIN = 6 | Task 1 |
| §3.2 TDOA defines | Task 1 |
| §3.2 queue_a/b/c | Task 5 |
| §3.2 AudioMemory(20→32) | Task 5 (increased to 32) |
| §3.2 boot calibration | Task 6 |
| §3.2 runTDOA() in loop | Tasks 7+8 |
| §3.2 updateDirectionPixels | Task 8 |
| §3.2 packet +3 bytes | Task 9 |
| §4.1 GCC-PHAT steps 1-8 | Task 3 |
| §4.2 angle formula | Task 2 |
| §4.3 boot calibration (median) | Task 6 |
| §5.1 NeoPixel states | Task 2 |
| §5.2 quadrant mapping | Task 2 |
| §5.3 2-frame hysteresis | Task 2 |
| §6.1 control packet 38B | Task 9 |
| §6.2 serial debug Dir/Conf | Task 9 |
| §7 all defines | Task 1 |
| §8 runAcoustic unchanged | Tasks 5–6 modify collection only |

**Spec gaps / deviations:**
1. **AudioMemory**: Spec says 20 blocks; plan uses 32 (headroom for 3 queues). This is safe — Teensy 4.1 has 512 KB DTCM.
2. **CIC decimation**: `pdm_sai2.cpp` has a structural stub for `update()`. The full CIC filter must be ported from `AudioInputPDM` before hardware testing. This is the highest-risk item.
3. **Mic A FFT reuse**: Spec mentions reusing the Mic A FFT computed in `runAcoustic()`. The plan computes it fresh in `runTDOA()` for simplicity (negligible cost at 10 Hz on 600 MHz Teensy). This is a deliberate simplification.
