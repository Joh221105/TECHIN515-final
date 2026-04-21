# TECHIN515 Final — PSSS Leak Sensor Dashboard

Real-time stack for **Teensy 4.1** + **MLX90640** + **SPH0645**. The **default workflow** is **USB only** (no Bluetooth): firmware prints **`$PSSS`** lines on USB **`Serial`**, **`tools/ble_connect.py`** reads the Teensy port and forwards **JSON** over **WebSocket**, and **`web/index.html`** displays live data.

Optional **HM-10 / Web Bluetooth** is disabled in firmware by default; see [Optional: HM-10 BLE](#optional-hm-10-ble) to turn it back on.

## Hardware (minimum for default workflow)

- **Teensy 4.1** + USB cable to your computer
- **MLX90640** (I²C) and **SPH0645** (I²S) as wired in `firmware/src/main.cpp`

## Project structure

```
firmware/   PlatformIO — flash to Teensy
web/        Dashboard (open over http:// — see below)
tools/      ble_connect.py (USB serial → WebSocket); thermal_viewer.py (optional)
streamlit/  Optional Web Bluetooth UI (needs ENABLE_HM10_BLE=1 on Teensy)
```

## Recommended: USB + WebSocket (Option A)

### 1. Flash firmware

Upload from the `firmware/` project (PlatformIO). Default build has **`ENABLE_HM10_BLE=0`**: no `Serial1` traffic, HM-10 not required.

### 2. Python dependencies

```bash
pip install pyserial websockets
```

### 3. Serial port

```bash
python tools/ble_connect.py --list-ports
```

### 4. Start the bridge (115200 baud)

```bash
python tools/ble_connect.py --port /dev/tty.usbmodem14101 --baud 115200
```

Use `-v` / `--verbose` to print every text line and byte counts in the terminal for debugging.

### 5. Open the dashboard over HTTP

The page uses **`ws://localhost:8765`**. Serve `web/` so the origin is `http://localhost` (not `file://`), for example:

```bash
cd web && python3 -m http.server 8080
```

Then open **`http://localhost:8080/index.html`**.

Keep **`ble_connect.py` running** in another terminal while you use the dashboard.

## `$PSSS` line format (USB `Serial`)

Emitted from `runThermal()` at ~1 Hz:

```
$PSSS,<status>,<snr_db>,<peak_hz>,<cold_pixel_c>,<b0>,…,<b23>*
```

| Field | Type | Description |
|-------|------|-------------|
| status | int | `1` = OK (no fused leak), `0` = **ALERT** (acoustic ∧ thermal on device) |
| snr_db | float | Acoustic SNR (dB) |
| peak_hz | float | Peak frequency in the ultrasonic band (Hz) |
| cold_pixel_c | float | Coldest pixel (°C) |
| b0…b23 | int 0–255 | 24 normalized clean-magnitude bands (10–49 kHz) for the spectrum chart |

**Note:** `status` may be `0` with legitimate zeros in other fields — do not use loose truthiness on numeric fields.

## Optional: HM-10 BLE

1. Wire HM-10 to **UART1** (see comments in `firmware/src/main.cpp`), **115200** (`AT+BAUD4`).
2. In **`firmware/platformio.ini`**, set **`-DENABLE_HM10_BLE=1`** and re-flash. The firmware will stream **1571-byte** binary frames on **`Serial1`** (same layout as `streamlit/app.html` / `thermal_viewer.py`).
3. For a browser-only wireless UI you can use **`streamlit/run streamlit/main.py`** and connect via Web Bluetooth, or a UART dongle on the HM-10 path with **`thermal_viewer.py --protocol binary`**.

## OpenCV thermal viewer (`tools/thermal_viewer.py`)

```bash
pip install pyserial opencv-python numpy
python thermal_viewer.py --port <uart-port> --protocol binary
```

`binary` expects **1571-byte** frames (only when **`ENABLE_HM10_BLE=1`** on the Teensy, or a dongle on that UART). Legacy **`--protocol frame`** uses ASCII `FRAME:` lines if you still have old firmware.

## Binary frame layout (`Serial1`, when `ENABLE_HM10_BLE=1`)

| Section | Size | Content |
|---------|------|---------|
| Header | 2 B | `0xFF 0xFE` |
| Metadata | 9 B | `snr` f32 LE, `peak_freq` f32 LE, `flags` u8 |
| Spectrum | 24 B | uint8 sub-band levels |
| Payload | 1536 B | 768 × int16 LE, °C×10 |

**Total: 1571 bytes** per frame.
