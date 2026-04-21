# BLE Sensor Fusion — Design Spec
_Date: 2026-04-20_

## Goal

Stream both thermal (MLX90640) and acoustic (SPH0645 FFT) sensor data from the Teensy 4.1 to the browser-based web app via an HM-10 BLE module, replacing the current ASCII Serial output. The web app displays a live thermal heatmap and a fully populated fusion gate (acoustic + thermal) instead of the current "Acoustic: N/A" placeholder.

---

## Architecture

```
Teensy 4.1
├── SPH0645 (I2S) ──→ FFT pipeline ──→ snr_db, peak_freq, acoustic_leak
├── MLX90640 (I2C) ──→ thermal pipeline ──→ 768 floats, thermal_anomaly
│
└── Serial1 TX (pin 0) ──→ HM-10 BLE module
         binary frame every ~250 ms (~4 Hz)
                │
                │ BLE (Web Bluetooth API)
                ▼
         Browser (app.html)
         ├── processIncoming() — frame parser, decodes acoustic + thermal
         └── renderFrame()     — heatmap canvas + sidebar + fusion gate
```

- `Serial1` carries the binary BLE stream; USB `Serial` remains for debug output.
- Acoustic state variables (`snr_db`, `peak_freq`, `acoustic_leak`) are always current from the last FFT run. `runThermal()` reads them at frame-emit time — no synchronisation needed.

---

## HM-10 Wiring

| HM-10 pin | Teensy 4.1 pin |
|-----------|----------------|
| TX        | RX1 (pin 1)    |
| RX        | TX1 (pin 0)    |
| VCC       | 3.3 V          |
| GND       | GND            |

HM-10 must be pre-configured to **115200 baud** via AT command (`AT+BAUD4`). Default is 9600, which is too slow for ~6 KB/s throughput.

---

## Binary Frame Format

Total size: **1547 bytes**, emitted once per thermal frame (~4 Hz).

| Offset | Size   | Type      | Field                                          |
|--------|--------|-----------|------------------------------------------------|
| 0      | 1      | uint8     | Header A = `0xFF`                              |
| 1      | 1      | uint8     | Header B = `0xFE`                              |
| 2      | 4      | float32   | `snr_db` — acoustic SNR in dB (little-endian) |
| 6      | 4      | float32   | `peak_freq` — peak frequency in Hz (LE)        |
| 10     | 1      | uint8     | flags — bit 0: `acoustic_leak`, bit 1: `thermal_anomaly` |
| 11     | 1536   | int16×768 | thermal pixels, `temp × 10`, little-endian     |

---

## Firmware Changes (`firmware/src/main.cpp`)

1. **Wiring comment** — add HM-10 pin mapping near the top with existing wiring notes.
2. **`setup()`** — add `Serial1.begin(115200)`.
3. **`runThermal()`** — replace ASCII `FRAME:...` Serial print with binary write to `Serial1`:
   - Write `0xFF`, `0xFE`
   - Write `snr_db` as 4-byte float32 LE
   - Write `peak_freq` as 4-byte float32 LE
   - Write `flags` byte (`(acoustic_leak ? 1 : 0) | (thermal_anomaly ? 2 : 0)`)
   - Write 768 × `int16_t` (`thermal_frame[i] * 10`) LE

USB `Serial` debug prints remain unchanged.

---

## Web App Changes (`streamlit/app.html`)

### Frame parser (`processIncoming`)
- Update `FRAME_TOTAL` from `1538` → `1547`
- After locating the `0xFF 0xFE` header, decode before the thermal array:
  - bytes 2–5: `snr_db` via `DataView.getFloat32(offset, true)`
  - bytes 6–9: `peak_freq` via `DataView.getFloat32(offset, true)`
  - byte 10: `flags` (`acousticLeak = flags & 1`, `thermalAnomalyFirmware = (flags >> 1) & 1`)
  - bytes 11–1546: thermal int16 array (existing decode, offset shift by 9)
- Pass `snr`, `peakFreq`, `acousticLeak` into `renderFrame()`

### Sidebar updates (`renderFrame`)
- `#dot-acoustic` / `#lbl-acoustic` — show live SNR dB and peak frequency
- `#fusion-status` — driven by `acousticLeak && thermalPass` (matches firmware logic)
- `#dot-thermal` / `#lbl-thermal` — unchanged (already working)

No HTML structural changes required — all element IDs are already present.

---

## Success Criteria

- Browser connects to HM-10 and receives frames at ~4 Hz
- Thermal heatmap renders correctly
- Sidebar shows live SNR, peak frequency, and correct fusion gate state
- "Acoustic: N/A" placeholder is replaced with real data
- USB Serial debug output continues to work for development
