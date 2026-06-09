# Design: 3-Mic TDOA Triangulation & Directional NeoPixel Indicator

**Date:** 2026-05-25  
**Project:** TECHIN515 Final — PSSS Leak Sensor  
**Target board:** Teensy 4.1  
**Status:** Approved, ready for implementation

---

## 1. Overview

Add two SPH0641 PDM microphones to the existing single-mic system, forming an equilateral triangle array (5 cm side). Use Time Difference of Arrival (TDOA) via GCC-PHAT cross-correlation to estimate the azimuth angle of a detected gas leak. Replace the existing SNR bar-graph NeoPixel behavior with a directional arrow: one of four pixels (N/E/S/W) lights red pointing toward the estimated leak source.

The existing acoustic SNR gate (`acoustic_leak`) and thermal localization pipeline are **unchanged** — TDOA only runs when a leak has already been confirmed.

---

## 2. Hardware / Wiring

### 2.1 Pin assignment changes

| Signal | Old pin | New pin | Reason |
|---|---|---|---|
| NeoPixel DIN | 5 | **6** | Pin 5 reassigned to SAI2 DIN |
| PDM CLK | 21 | 21 | Unchanged — shared by all 3 mics |
| PDM DATA1 | 8 | 8 | Unchanged — Mic A (L) + Mic B (R) |
| PDM DATA2 | — | **5** | New — SAI2 DIN for Mic C |

### 2.2 Microphone wiring

All three mics share **Pin 21** as their CLK input. Use a short star or daisy-chain connection; keep CLK traces/wires short and equal-length if possible. Each mic needs a **0.1 µF bypass cap** between VDD and GND, placed as close to the mic as possible.

**Mic A — North vertex, Left channel (existing mic)**
```
CLK  → Teensy Pin 21
DATA → Teensy Pin 8
SEL  → GND
VDD  → 3.3 V
GND  → GND
```

**Mic B — Bottom-left vertex (120° CCW from North), Right channel**
```
CLK  → Teensy Pin 21   (shared with Mic A)
DATA → Teensy Pin 8    (shared with Mic A — L/R pair: A=Left, B=Right)
SEL  → 3.3 V
VDD  → 3.3 V
GND  → GND
```

**Mic C — Bottom-right vertex (120° CW from North), Right channel**
```
CLK  → Teensy Pin 21   (shared)
DATA → Teensy Pin 5    (dedicated SAI2 DIN)
SEL  → 3.3 V
VDD  → 3.3 V
GND  → GND
```

**NeoPixels (4 chained, moved)**
```
DIN  → Teensy Pin 6   (was Pin 5)
V+   → 5 V (or 3.3 V)
GND  → GND
330–470 Ω series resistor on DIN recommended
```

### 2.3 Physical orientation

Mount the sensor board so **Mic A (North vertex) points toward the primary monitoring direction** (e.g., toward the pipe or wall). The NeoPixel strip is software-mapped: Pixel 0 = N, Pixel 1 = E, Pixel 2 = S, Pixel 3 = W. Physical pixel order on the strip can be adjusted via the `PIXEL_NORTH` / `PIXEL_EAST` / `PIXEL_SOUTH` / `PIXEL_WEST` defines.

### 2.4 Mic triangle geometry reference

```
          Mic A (North)
            △
           / \
     5 cm /   \ 5 cm
         /     \
   Mic B ——————— Mic C
  (BL, 120°CCW) (BR, 120°CW)
        5 cm
```

Center of triangle is the acoustic reference point. All distances measured center-to-center.

---

## 3. Software Architecture

### 3.1 New files

| File | Responsibility |
|---|---|
| `src/pdm_sai2.h` | Class declaration for `AudioInputPDM_SAI2` |
| `src/pdm_sai2.cpp` | SAI2 register config, DMA setup, CIC decimation — outputs 1 mono audio channel (Mic C) |
| `src/gcc_phat.h` | `computeGccPhat()`, `parabolaPeak()` declarations |
| `src/gcc_phat.cpp` | Self-contained GCC-PHAT implementation using CMSIS-DSP `arm_rfft_fast_f32` |
| `src/direction.h` | `computeAngle()`, `angleToQuadrant()`, `updateDirectionPixels()` declarations |
| `src/direction.cpp` | Angle formula, quadrant mapping, NeoPixel update logic |

### 3.2 Changes to `main.cpp`

- `NEOPIXEL_PIN` changed from `5` to `6`
- New defines: `MIC_SPACING_M`, `TDOA_CONFIDENCE_THRESHOLD`, `TDOA_WINDOW`, `TDOA_MAX_DELAY_SAMPLES`
- Add `AudioInputPDM_SAI2 pdm_sai2` and three `AudioRecordQueue` objects: `queue_a`, `queue_b`, `queue_c`
- Existing `AudioConnection patch(pdm_in, 0, queue, 0)` replaced with 3 connections (ch0→queue_a, ch1→queue_b, pdm_sai2→queue_c)
- `AudioMemory` increased from 12 to 20 blocks
- Boot sequence: add 500 ms TDOA bias calibration after thermal baseline
- `loop()`: add `runTDOA()` call; replace `updateLeakPixels()` with `updateDirectionPixels()`
- Control packet extended by 3 bytes (direction angle + confidence)

### 3.3 Audio object graph

```
AudioInputPDM    pdm_in        → queue_a (Mic A, Left channel)
                               → queue_b (Mic B, Right channel)
AudioInputPDM_SAI2  pdm_sai2  → queue_c (Mic C, Right channel from SAI2)
```

### 3.4 `AudioInputPDM_SAI2` design

Mirrors the existing Teensy `AudioInputPDM` but targets **SAI2** instead of SAI1:

- Configured with the same Audio PLL source and identical CCM dividers as SAI1 → identical PDM bit-clock frequency (≈ 2.822 MHz for 44.1 kHz × 64 OSR)
- DMA channel allocated from Teensy's DMA pool, triggered by SAI2 RX FIFO
- CIC decimation identical to SAI1 path (64× oversampling ratio, same filter coefficients)
- Outputs a single mono `AudioBlock` per update cycle (128 samples at 44.1 kHz)
- SAI2 and SAI1 are not phase-locked at startup; a fixed offset of ±1 sample is expected and removed by boot calibration

**Key registers configured:**
- `CCM_CCGR5` — enable SAI2 clock gate
- `CCM_CS1CDR` — point SAI2 to Audio PLL (same source as SAI1)
- `SAI2_RCR2` — set bit-clock divider matching SAI1
- `SAI2_RCR3/4/5` — frame sync, word length (1-bit PDM), 2-slot TDM (L+R)
- `IOMUXC` — configure Pin 5 as SAI2_RX_DATA0

---

## 4. TDOA Algorithm

### 4.1 GCC-PHAT (`src/gcc_phat.cpp`)

```
Input:  float32_t mic_x[TDOA_WINDOW], mic_y[TDOA_WINDOW]  (TDOA_WINDOW = 1024)
Output: float tdoa_samples  (sub-sample accuracy via parabolic interpolation)
        float peak_height   (normalized 0.0–1.0, used as confidence)
```

Steps:
1. Apply Hann window to both input buffers
2. `arm_rfft_fast_f32` on each → `X[f]`, `Y[f]`
3. Cross-spectrum `R[f] = X[f] × conj(Y[f])`
4. PHAT: `R_phat[f] = R[f] / (|R[f]| + ε)` — ε = 1e-6 prevents divide-by-zero in silence
5. `arm_rfft_fast_f32` inverse → correlation array `r[n]`
6. Search `r[n]` in range `[−TDOA_MAX_DELAY_SAMPLES, +TDOA_MAX_DELAY_SAMPLES]` for peak
7. Parabolic interpolation on peak ± 1 sample → fractional TDOA
8. Normalize peak height by `max(|r[n]|)` over search window

`TDOA_WINDOW = 1024` (shares FFT buffer with `runAcoustic()` for Mic A — Mic A FFT is computed once and reused).  
`TDOA_MAX_DELAY_SAMPLES = 8` (covers 5 cm at 343 m/s at 44.1 kHz with 1-sample margin).

### 4.2 Angle formula

Given TDOA values τ_AB and τ_AC (in samples, bias-corrected):

```
θ = atan2f( sqrtf(3.0f) * (τ_AB - τ_AC),  τ_AB + τ_AC )
```

- θ = 0° → North (toward Mic A)  
- θ increases clockwise  
- Valid range: −180° to +180°, wraps correctly via `atan2f`

**Derivation:** For a far-field source at azimuth θ (measured CW from North), the TDOA for each mic pair is a linear projection of the unit source direction onto the inter-mic axis. Solving the 2×2 system gives `sin θ ∝ (τ_AB − τ_AC)` and `cos θ ∝ (τ_AB + τ_AC)/√3`, yielding the formula above. The mic spacing `d`, speed of sound `c`, and sample rate `fs` cancel in the `atan2f` and are not needed.

### 4.3 Boot calibration

After the thermal baseline collection, run a 500 ms TDOA calibration:

1. Collect 22,050 samples from all 3 mics (500 ms at 44.1 kHz)
2. Compute GCC-PHAT on pairs (A,B) and (A,C) over overlapping 1024-sample windows (stride 512)
3. Take the **median** peak position across all windows as `τ_bias_AB` and `τ_bias_AC`
4. Store in globals; subtract from every live TDOA reading

Calibration assumes ambient noise with no dominant directional source. The calibration values are printed to Serial for debugging.

---

## 5. NeoPixel Direction Behavior

### 5.1 States

| Condition | All 4 pixels | Notes |
|---|---|---|
| No leak (`!acoustic_leak`) | Off | Same as current |
| Leak, low confidence (`peak < TDOA_CONFIDENCE_THRESHOLD`) | Amber `(255, 80, 0)` | Leak confirmed, direction unknown |
| Leak, direction valid | 1 pixel red `(255, 0, 0)` | Pixel index = nearest quadrant |

### 5.2 Quadrant mapping

```
θ in [−45°,  +45°) → Pixel PIXEL_NORTH  (default 0)
θ in [+45°, +135°) → Pixel PIXEL_EAST   (default 1)
θ in [+135°,+225°) → Pixel PIXEL_SOUTH  (default 2)
θ in [+225°,+315°) → Pixel PIXEL_WEST   (default 3)
  (equivalently: [−135°, −45°) → Pixel PIXEL_WEST)
```

`PIXEL_NORTH`, `PIXEL_EAST`, `PIXEL_SOUTH`, `PIXEL_WEST` are `#define` constants in `src/direction.h` so the physical strip order can be adjusted without touching logic.

### 5.3 Direction hysteresis

To prevent flickering when the angle estimate is near a quadrant boundary, a **2-frame hysteresis** is applied: the displayed quadrant only changes if the new quadrant differs from the current one for 2 consecutive TDOA frames.

---

## 6. BLE/Serial Protocol Changes

### 6.1 Control packet extension (10 Hz)

Extended from 35 bytes to **38 bytes**:

| Field | Size | Content |
|---|---|---|
| Header | 2 B | `0xFF 0xFC` (unchanged) |
| snr | 4 B | f32 LE (unchanged) |
| peak_freq | 4 B | f32 LE (unchanged) |
| flags | 1 B | bit 0 = acoustic_leak, bit 1 = thermal_anomaly, **bit 2 = direction_valid** |
| spectrum | 24 B | uint8 sub-band levels (unchanged) |
| **direction_angle** | **2 B** | int16 LE, degrees × 10 (e.g., 1234 = 123.4°) |
| **tdoa_confidence** | **1 B** | uint8, normalized 0–255 |

### 6.2 Serial debug output

Add direction angle and confidence to the existing print line:
```
Score | Level | Ambnt | Peak(Hz) | Acoustic | ObjTemp | DeltaT | ColdSpot | Dir(°) | Conf | STATUS
```

---

## 7. Configuration Constants

All tunable parameters will be `#define` constants at the top of `main.cpp`:

```c
#define NEOPIXEL_PIN              6      // moved from 5
#define MIC_SPACING_M         0.05f     // 5 cm equilateral triangle side
#define TDOA_WINDOW            1024     // samples per GCC-PHAT frame
#define TDOA_MAX_DELAY_SAMPLES    8     // ±8 sample search window
#define TDOA_CONFIDENCE_THRESHOLD 0.25f // min GCC-PHAT peak to show direction
#define TDOA_CALIB_MS           500     // boot calibration duration
#define PIXEL_NORTH               0
#define PIXEL_EAST                1
#define PIXEL_SOUTH               2
#define PIXEL_WEST                3
```

---

## 8. What Is Not Changed

- `runAcoustic()` — SNR/FFT pipeline on Mic A, unchanged
- `runThermal()` — MLX90640 pipeline, unchanged
- `runControl()` / `emitControlPacket()` — extended but not restructured
- Thermal packet format — unchanged
- ESP32-S3 BLE bridge firmware — no changes needed for new control packet bytes (ESP32 forwards raw bytes; web dashboard updated separately if desired)
- `SNR_THRESHOLD_DB`, `LEAK_CONFIRM_COUNT`, all existing acoustic/thermal defines — unchanged

---

## 9. Out of Scope

- Distance estimation (range from array to leak) — requires calibrated amplitude or wideband coherence, not in this spec
- Web dashboard update to display direction arrow — separate task
- Elevation (vertical) angle estimation — 2D azimuth only
