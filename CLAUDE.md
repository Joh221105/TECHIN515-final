# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

### Firmware (Teensy 4.1) — `firmware/`
```bash
# Build
pio run -e teensy41

# Flash
pio run -e teensy41 -t upload

# Serial monitor
pio device monitor -e teensy41
```

### ESP32-S3 BLE bridge — `esp32s3/`
```bash
# Build
pio run -e esp32s3

# Flash
pio run -e esp32s3 -t upload

# Monitor (USB-CDC — ARDUINO_USB_CDC_ON_BOOT=1 required)
pio device monitor -e esp32s3
```

### Host tooling
```bash
# Install Python deps
pip install bleak websockets pyserial opencv-python numpy

# BLE → WebSocket bridge (scan for device)
python tools/ble_connect.py

# BLE → WebSocket bridge (direct by address)
python tools/ble_connect.py --ble-address AA:BB:CC:DD:EE:FF

# Thermal heatmap viewer via serial
python tools/thermal_viewer.py --port /dev/tty.usbserial-* --protocol binary

# Serve web dashboard
cd web && python3 -m http.server 8080
# Open http://localhost:8080/index.html
```

## Architecture

The system is a gas-leak proximity detector called **PSSS** with a 4-layer pipeline:

```
Teensy 4.1 (firmware/)
    ↓ UART 115200 bps (binary frames)
ESP32-S3 XIAO (esp32s3/)
    ↓ BLE GATT notifications (chunked, 512-byte MTU)
ble_connect.py (tools/)
    ↓ WebSocket ws://localhost:8765 (JSON)
web/index.html
```

### Teensy firmware (`firmware/src/main.cpp`)

The Teensy is the sensor controller and LED driver. Two concurrent pipelines run in `loop()`:

- **Acoustic pipeline** (`runAcoustic()`): SPH0641 PDM mic → `AudioInputPDM` → `AudioAnalyzeFFT1024` at 160 kHz sample rate. Sums energy across bins 128–511 (≈20–80 kHz), exponential-smooths it, normalizes against a running peak, and maps to one of 3 proximity zones (0/1/2). A single WS2812 NeoPixel on pin 6 reflects the zone in green/yellow/red.

- **Thermal pipeline**: MLX90640 at 4 Hz (I2C pins 18/19). On boot, collects a 10-second ambient temperature baseline. During operation it downsamples the 32×24 frame to 16×12 via 2×2 averaging before transmission.

The firmware emits two binary frame types over `Serial1` (UART to ESP32):
- **Control frame** (10 Hz, 12 bytes): `0xFF 0xFC` + `band_energy_norm` f32 + `peak_freq_hz` f32 + `proximity_zone` u8 + `flags` u8
- **Thermal frame** (4 Hz, 396 bytes): `0xFF 0xFE` + `band_energy_norm` f32 + `timestamp_ms` u32 + `proximity_zone` u8 + `flags` u8 + 16×12 `int16` pixels (°C × 10)

### ESP32-S3 bridge (`esp32s3/src/main.cpp`)

Passive byte forwarder. Reads UART bytes from the Teensy, scans for `0xFF 0xFE` / `0xFF 0xFC` frame headers, buffers a complete frame, then sends it via BLE GATT notify on characteristic `ffe1` (service `ffe0`). Large frames are chunked at 512 bytes. Automatically restarts advertising on disconnect.

### Python bridge (`tools/ble_connect.py`)

Scans BLE for a device advertising service `ffe0` (name `PSSS-Sensor`), subscribes to `ffe1`, and reassembles the byte stream into complete frames using the same header-scan logic. Decodes each frame into a JSON dict (`kind: "control"` or `kind: "thermal"`) and broadcasts to all connected WebSocket clients.

### Web dashboard (`web/index.html`)

Single-file, no bundler. Connects to WebSocket at `ws://localhost:8765`. Handles two JSON message kinds:
- `"thermal"`: renders 16×12 grid onto a `<canvas>` using a built-in inferno colormap LUT; draws a colorbar with temperature range.
- `"control"`: updates the leak badge, band-energy SNR readout, and 24-bar spectrum display.

### Frame protocol invariant

**All three layers (Teensy, ESP32, Python) share the same frame constants.** When changing any frame field or size, update all three simultaneously:
- `firmware/src/main.cpp` — `emitControlFrame()` / `emitThermalFrame()`
- `esp32s3/src/main.cpp` — `FRAME_TOTAL_THERMAL` / `FRAME_TOTAL_CONTROL`
- `tools/ble_connect.py` — `FRAME_TOTAL_THERMAL` / `FRAME_TOTAL_CONTROL` + struct unpack formats

## Active branches

- `feat/single-mic-proximity-detector` — current branch; single SPH0641 mic, NeoPixel zone indicator, 16×12 thermal downsampling, 12-byte control frame.
- `feat/3mic-tdoa-triangulation` — planned; adds two more PDM mics for GCC-PHAT TDOA azimuth estimation and a directional NeoPixel arrow. Design spec: `docs/superpowers/specs/2026-05-25-3mic-tdoa-triangulation-design.md`.
