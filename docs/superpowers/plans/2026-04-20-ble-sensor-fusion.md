# BLE Sensor Fusion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stream both thermal and acoustic sensor data from the Teensy 4.1 to the browser via HM-10 BLE, replacing the current ASCII Serial output with a binary frame protocol so the web app shows a live heatmap and fully populated fusion gate.

**Architecture:** Firmware emits a 1547-byte binary frame over Serial1 (→ HM-10 → BLE) every thermal tick (~4 Hz): 2-byte header + 9 bytes of acoustic metadata + 1536 bytes of thermal pixels. The browser's Web Bluetooth parser is updated to decode the acoustic fields and wire them into the existing sidebar elements.

**Tech Stack:** PlatformIO / Teensy Arduino (C++), Vanilla JS / Web Bluetooth API, Streamlit (unchanged).

---

## File Map

| File | Change |
|------|--------|
| `firmware/src/main.cpp` | Add Serial1 init; replace ASCII FRAME: output with binary write |
| `streamlit/app.html` | Update FRAME_TOTAL constant; update parser offsets; add acoustic sidebar wiring |

---

## Task 1: Firmware — Serial1 init + HM-10 wiring comment

**Files:**
- Modify: `firmware/src/main.cpp`

- [ ] **Step 1: Add HM-10 wiring comment**

In `firmware/src/main.cpp`, find the wiring comment block at the top (lines 7–9) and add the HM-10 line:

```cpp
// ── Wiring ────────────────────────────────────────────────────
// SPH0645: BCLK→Pin21  LRCL→Pin20  DOUT→Pin8  SEL→GND  3V→3.3V
// MLX90640: SCL→Pin19  SDA→Pin18  VDD→3.3V  VSS→GND
// HM-10 BLE: TX1(pin0)→HM-10 RX, RX1(pin1)→HM-10 TX, VCC→3.3V, GND→GND
//            Configure HM-10 to 115200 baud: AT+BAUD4
```

- [ ] **Step 2: Add Serial1 init in setup()**

In `setup()`, after `Serial.begin(115200)` (line 83), add:

```cpp
Serial1.begin(115200);
```

- [ ] **Step 3: Build to verify it compiles**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware && pio run
```

Expected: `SUCCESS` with no errors. Warnings about unused variables are fine.

- [ ] **Step 4: Commit**

```bash
git add firmware/src/main.cpp
git commit -m "feat: init Serial1 at 115200 for HM-10 BLE"
```

---

## Task 2: Firmware — Binary frame output over Serial1

**Files:**
- Modify: `firmware/src/main.cpp`

- [ ] **Step 1: Replace ASCII FRAME output with binary write**

In `runThermal()`, find and remove the ASCII block (currently starting with `Serial.print("FRAME:")`):

```cpp
// REMOVE this entire block:
Serial.print("FRAME:");
for (int i = 0; i < 768; i++) {
    Serial.print(thermal_frame[i], 1);
    if (i < 767) Serial.print(",");
}
Serial.println();
```

Replace it with:

```cpp
// Binary frame: [0xFF][0xFE] + snr_db(4B) + peak_freq(4B) + flags(1B) + thermal(1536B)
uint8_t header[2] = {0xFF, 0xFE};
Serial1.write(header, 2);
Serial1.write((uint8_t*)&snr_db,   sizeof(snr_db));
Serial1.write((uint8_t*)&peak_freq, sizeof(peak_freq));
uint8_t flags = (acoustic_leak ? 1u : 0u) | (thermal_anomaly ? 2u : 0u);
Serial1.write(&flags, 1);
for (int i = 0; i < 768; i++) {
    int16_t px = (int16_t)(thermal_frame[i] * 10.0f);
    Serial1.write((uint8_t*)&px, 2);
}
```

- [ ] **Step 2: Build to verify it compiles**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware && pio run
```

Expected: `SUCCESS`.

- [ ] **Step 3: Upload and verify USB Serial debug output still works**

```bash
cd /Users/john/Desktop/TECHIN515-final/firmware && pio run --target upload && pio device monitor --baud 115200
```

Expected output on USB Serial (unchanged):
```
=== PSSS Leak Detector ===
FFT bin: 97.7 Hz | Signal band: 10000-49000 Hz
SPH0645 ready.
MLX90640 ready. Collecting 10s baseline — keep sensor still...
....................
Baseline cold-pixel: XX.XX C

Monitoring. Expose to ultrasonic noise + spray IPA to confirm leak.
SNR(dB) | Peak(Hz) | Acoustic | ObjTemp | DeltaT | Thermal | FUSED
---------------------------------------------------------------------
    2.1 |    25000 |     ---- |   22.30 |   0.10 | ------- | monitoring...
```

The `FRAME:` lines should no longer appear. Binary data will now be going to HM-10 on Serial1, invisible on USB Serial.

- [ ] **Step 4: Commit**

```bash
git add firmware/src/main.cpp
git commit -m "feat: emit binary BLE frame over Serial1 to HM-10"
```

---

## Task 3: Web App — Update frame constants and parser

**Files:**
- Modify: `streamlit/app.html`

- [ ] **Step 1: Update frame size constants**

In `streamlit/app.html`, find the frame format constants block (around line 461):

```js
// ── Frame format ─────────────────────────────────────────────
// Header: 0xFF 0xFE
// Body:   768 × int16_t little-endian (temp × 10)
// Total:  2 + 1536 = 1538 bytes
const HEADER_A    = 0xFF;
const HEADER_B    = 0xFE;
const FRAME_BODY  = 768 * 2;         // 1536 bytes
const FRAME_TOTAL = 2 + FRAME_BODY;  // 1538 bytes
```

Replace with:

```js
// ── Frame format ─────────────────────────────────────────────
// Header:   0xFF 0xFE                          (2 bytes)
// Metadata: snr_db float32 + peak_freq float32 + flags uint8  (9 bytes)
// Body:     768 × int16_t little-endian (temp × 10)           (1536 bytes)
// Total:    2 + 9 + 1536 = 1547 bytes
const HEADER_A    = 0xFF;
const HEADER_B    = 0xFE;
const FRAME_META  = 9;
const FRAME_BODY  = 768 * 2;
const FRAME_TOTAL = 2 + FRAME_META + FRAME_BODY;  // 1547 bytes
```

- [ ] **Step 2: Update processIncoming() to decode acoustic fields**

Find the thermal decode loop inside `processIncoming()`:

```js
    const temps = new Float32Array(768);
    for (let i = 0; i < 768; i++) {
      const lo  = rxBuf[2 + i*2];
      const hi2 = rxBuf[2 + i*2 + 1];
      let val = (hi2 << 8) | lo;
      if (val > 0x7FFF) val -= 0x10000;
      temps[i] = val / 10.0;
    }
    renderFrame(temps);
    rxBuf = rxBuf.slice(FRAME_TOTAL);
```

Replace with:

```js
    // Decode acoustic metadata (bytes 2–10)
    const dv = new DataView(rxBuf.buffer, rxBuf.byteOffset + 2, FRAME_META);
    const snr        = dv.getFloat32(0, true);
    const peakFreq   = dv.getFloat32(4, true);
    const flags      = rxBuf[10];
    const acousticLeak = (flags & 1) !== 0;

    // Decode thermal pixels (bytes 11–1546)
    const temps = new Float32Array(768);
    for (let i = 0; i < 768; i++) {
      const lo  = rxBuf[11 + i*2];
      const hi2 = rxBuf[11 + i*2 + 1];
      let val = (hi2 << 8) | lo;
      if (val > 0x7FFF) val -= 0x10000;
      temps[i] = val / 10.0;
    }
    renderFrame(temps, snr, peakFreq, acousticLeak);
    rxBuf = rxBuf.slice(FRAME_TOTAL);
```

- [ ] **Step 3: Update renderFrame signature**

Find:

```js
function renderFrame(temps) {
```

Replace with:

```js
function renderFrame(temps, snr, peakFreq, acousticLeak) {
```

- [ ] **Step 4: Test the parser with a synthetic frame in the browser console**

Start the Streamlit app:
```bash
cd /Users/john/Desktop/TECHIN515-final/streamlit && streamlit run main.py
```

Open `http://localhost:8501` in Chrome. Open DevTools console (F12) and paste:

```js
// Build a synthetic 1547-byte frame
const frame = new Uint8Array(1547);
const dv = new DataView(frame.buffer);
frame[0] = 0xFF; frame[1] = 0xFE;
dv.setFloat32(2, 8.5,   true);   // snr_db = 8.5 dB
dv.setFloat32(6, 25000, true);   // peak_freq = 25 kHz
frame[10] = 0x01;                 // acoustic_leak=1, thermal_anomaly=0
for (let i = 0; i < 768; i++) dv.setInt16(11 + i*2, 200, true); // 20.0°C
processIncoming(frame);
```

Expected:
- Heatmap canvas renders a uniform blue/green field (~20°C)
- No JS errors in console
- `renderFrame` called (no crash from new signature)

- [ ] **Step 5: Commit**

```bash
git add streamlit/app.html
git commit -m "feat: update BLE frame parser for 1547-byte acoustic+thermal format"
```

---

## Task 4: Web App — Wire acoustic data into sidebar

**Files:**
- Modify: `streamlit/app.html`

- [ ] **Step 1: Update fusion gate logic and acoustic sidebar in renderFrame()**

Inside `renderFrame()`, find the fusion gate block:

```js
  const leakConfirmed = thermalPass; // extend with acoustic when available

  document.getElementById('s-cold').textContent  = minT.toFixed(1) + ' °C';
  ...

  const gate       = document.getElementById('fusion-gate');
  const fusionStat = document.getElementById('fusion-status');
  const dotThermal = document.getElementById('dot-thermal');
  const lblThermal = document.getElementById('lbl-thermal');

  dotThermal.className = 'fusion-dot ' + (thermalPass ? 'fail' : 'pass');
  lblThermal.textContent = 'Thermal: ' + (deltaT !== null ? deltaT.toFixed(1) + '°C' : 'calibrating');

  if (leakConfirmed) {
    gate.className        = 'fusion-gate leak';
    fusionStat.className  = 'fusion-status leak';
    fusionStat.textContent = '⚠ LEAK';
    leakLabel.style.display = 'block';
  } else {
    gate.className        = 'fusion-gate';
    fusionStat.className  = 'fusion-status';
    fusionStat.textContent = 'NO LEAK';
    leakLabel.style.display = 'none';
  }
```

Replace with:

```js
  const leakConfirmed = thermalPass && acousticLeak;

  document.getElementById('s-cold').textContent  = minT.toFixed(1) + ' °C';
  ...

  const gate       = document.getElementById('fusion-gate');
  const fusionStat = document.getElementById('fusion-status');
  const dotThermal = document.getElementById('dot-thermal');
  const lblThermal = document.getElementById('lbl-thermal');
  const dotAcoustic = document.getElementById('dot-acoustic');
  const lblAcoustic = document.getElementById('lbl-acoustic');

  dotThermal.className = 'fusion-dot ' + (thermalPass ? 'fail' : 'pass');
  lblThermal.textContent = 'Thermal: ' + (deltaT !== null ? deltaT.toFixed(1) + '°C' : 'calibrating');

  dotAcoustic.className = 'fusion-dot ' + (acousticLeak ? 'fail' : 'pass');
  lblAcoustic.textContent = `Acoustic: ${snr.toFixed(1)} dB @ ${(peakFreq / 1000).toFixed(1)} kHz`;

  if (leakConfirmed) {
    gate.className        = 'fusion-gate leak';
    fusionStat.className  = 'fusion-status leak';
    fusionStat.textContent = '⚠ LEAK';
    leakLabel.style.display = 'block';
  } else {
    gate.className        = 'fusion-gate';
    fusionStat.className  = 'fusion-status';
    fusionStat.textContent = 'NO LEAK';
    leakLabel.style.display = 'none';
  }
```

Note: the `...` lines between `s-cold` and the gate block are unchanged — only the `leakConfirmed` line and the acoustic dot/label lines are new.

- [ ] **Step 2: Test acoustic display with synthetic frames in browser console**

With the Streamlit app running, open DevTools console and paste:

```js
// Frame 1: acoustic leak detected, thermal OK (no leak confirmed)
const f1 = new Uint8Array(1547);
const d1 = new DataView(f1.buffer);
f1[0] = 0xFF; f1[1] = 0xFE;
d1.setFloat32(2, 9.2,   true);  // snr_db = 9.2 dB
d1.setFloat32(6, 32000, true);  // peak_freq = 32 kHz
f1[10] = 0x01;                   // acoustic_leak=1, thermal_anomaly=0
for (let i = 0; i < 768; i++) d1.setInt16(11 + i*2, 220, true); // 22.0°C (no thermal anomaly)
processIncoming(f1);
```

Expected: sidebar shows `Acoustic: 9.2 dB @ 32.0 kHz`, acoustic dot is red, fusion status is "NO LEAK" (thermal not anomalous).

```js
// Frame 2: both leak signals active → confirmed leak
const f2 = new Uint8Array(1547);
const d2 = new DataView(f2.buffer);
f2[0] = 0xFF; f2[1] = 0xFE;
d2.setFloat32(2, 9.2,   true);
d2.setFloat32(6, 32000, true);
f2[10] = 0x01;  // acoustic_leak=1

// Simulate a cold spot: first fill with 22°C baseline equivalent,
// then set first pixel very cold to trigger thermalPass locally.
// (baselineReady must be true — run the app fresh and wait 10 frames first,
//  or set baselineReady=true and baselineAvg=22.0 in console before this test)
baselineReady = true; baselineAvg = 22.0;
for (let i = 0; i < 768; i++) d2.setInt16(11 + i*2, 190, true); // 19.0°C (delta = -3°C → anomaly)
processIncoming(f2);
```

Expected: fusion status shows `⚠ LEAK`, gate border turns red, leak label appears on heatmap.

- [ ] **Step 3: Commit**

```bash
git add streamlit/app.html
git commit -m "feat: wire acoustic SNR/freq into sidebar and fusion gate"
```

---

## Task 5: End-to-End Verification Checklist

These steps require the physical HM-10 wired to the Teensy.

- [ ] **Step 1: Confirm HM-10 is configured at 115200 baud**

With HM-10 connected to a USB-serial adapter (not the Teensy yet), open a serial terminal at 9600 baud and send `AT`. Response: `OK`. Then send `AT+BAUD4`. Response: `OK+Set:4`. Reconnect at 115200 and send `AT` to confirm.

- [ ] **Step 2: Wire HM-10 to Teensy Serial1**

| HM-10 | Teensy 4.1 |
|-------|-----------|
| TX    | Pin 1 (RX1) |
| RX    | Pin 0 (TX1) |
| VCC   | 3.3 V |
| GND   | GND |

- [ ] **Step 3: Power Teensy and open Streamlit app**

```bash
cd /Users/john/Desktop/TECHIN515-final/streamlit && streamlit run main.py
```

Open `http://localhost:8501` in Chrome (must be Chrome — Web Bluetooth is not supported in Firefox/Safari).

- [ ] **Step 4: Connect via BLE**

Click "Connect HM-10". Chrome shows a device picker. Select the HM-10 device (usually named `HMSoft` or `MLT-BT05`).

Expected:
- Status pill shows `LIVE`
- Idle overlay disappears
- Heatmap begins rendering thermal frames
- Footer shows frame rate ~4 Hz and increasing bytes rx
- Sidebar shows live SNR dB and peak frequency (not "N/A")
- Fusion gate reflects both acoustic and thermal state
