# NeoPixel Leak Proximity Indicator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a 4-LED NeoPixel indicator to the Teensy firmware so higher microphone SNR lights more LEDs as the sensor gets closer to the leak.

**Architecture:** The Teensy continues to own sensor processing and now also owns LED output. A pure helper maps `snr_db` to 0-4 lit LEDs; the existing acoustic loop calls the LED updater after each successful FFT. The ESP32-S3 BLE bridge and BLE packet format stay unchanged.

**Tech Stack:** PlatformIO, Teensy 4.1 Arduino framework, Teensy Audio library, CMSIS-DSP, Adafruit NeoPixel.

---

## File Map

| File | Change |
|---|---|
| `firmware/platformio.ini` | Add `adafruit/Adafruit NeoPixel` dependency |
| `firmware/src/main.cpp` | Add NeoPixel include, constants, setup, SNR mapping helper, and LED update call |
| `esp32s3/src/main.cpp` | No change |

## Task 1: Add NeoPixel Dependency

**Files:**
- Modify: `firmware/platformio.ini`

- [ ] **Step 1: Add dependency**

Add `adafruit/Adafruit NeoPixel@^1.12.3` under the existing `lib_deps` entries.

- [ ] **Step 2: Build**

Run:

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware
pio run
```

Expected: firmware compiles, or PlatformIO downloads the dependency and then compiles.

## Task 2: Add LED Mapping and Output

**Files:**
- Modify: `firmware/src/main.cpp`

- [ ] **Step 1: Add include and constants**

Add `#include <Adafruit_NeoPixel.h>`, `NEOPIXEL_PIN 5`, `NEOPIXEL_COUNT 4`, and fixed SNR thresholds.

- [ ] **Step 2: Add helper**

Add `leakPixelCountForSnr(float snr)` returning:

```cpp
0 for snr < 6
1 for 6 <= snr < 9
2 for 9 <= snr < 12
3 for 12 <= snr < 16
4 for snr >= 16
```

- [ ] **Step 3: Add updater**

Add `updateLeakPixels(float snr)`, set each pixel on/off, and call `pixels.show()`.

- [ ] **Step 4: Initialize pixels**

Call `pixels.begin()`, set conservative brightness, clear, and show in `setup()`.

- [ ] **Step 5: Update after acoustic FFT**

Call `updateLeakPixels(snr_db)` only when `runAcoustic()` returns true.

- [ ] **Step 6: Build**

Run:

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware
pio run
```

Expected: firmware compiles.
