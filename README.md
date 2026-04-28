# TECHIN515 Final — PSSS Leak Sensor Dashboard

Real-time BLE stack for **Teensy 4.1** + **MLX90640** + **SPH0645** + **ESP32-S3**.
The Teensy streams binary sensor frames over UART to ESP32-S3, `tools/ble_connect.py` receives BLE notifications and forwards JSON over WebSocket, and `web/index.html` renders the dashboard.

## Hardware

- **Teensy 4.1**
- **ESP32-S3** (BLE bridge)
- **MLX90640** (I2C) and **SPH0645** (I2S) as wired in `firmware/src/main.cpp`

## Project structure

```
firmware/   PlatformIO firmware for Teensy
esp32s3/    PlatformIO firmware for ESP32-S3 BLE bridge
tools/      ble_connect.py (BLE -> WebSocket bridge)
web/        Dashboard client (WebSocket)
```

## Run workflow (BLE-only)

1. Flash firmware in `firmware/` and `esp32s3/`.
2. Install bridge dependencies:

```bash
pip install bleak websockets
```

3. Start the BLE -> WebSocket bridge:

```bash
python tools/ble_connect.py
```

Optional direct connect by address:

```bash
python tools/ble_connect.py --ble-address AA:BB:CC:DD:EE:FF
```

4. Serve `web/` over HTTP and open the dashboard:

```bash
cd web && python3 -m http.server 8080
```

Open `http://localhost:8080/index.html`.

## Binary frame layout (Teensy -> ESP32-S3 UART -> BLE notification stream)

Control packet (10 Hz):

| Section | Size | Content |
|---------|------|---------|
| Header | 2 B | `0xFF 0xFC` |
| Metadata | 9 B | `snr` f32 LE, `peak_freq` f32 LE, `flags` u8 |
| Spectrum | 24 B | `uint8` sub-band levels |

Total: **35 bytes** per control frame.

Thermal packet (4 Hz):

| Section | Size | Content |
|---------|------|---------|
| Header | 2 B | `0xFF 0xFE` |
| Metadata | 13 B | `snr` f32 LE, `peak_freq` f32 LE, `thermal_ts_ms` u32 LE, `flags` u8 |
| Spectrum | 24 B | `uint8` sub-band levels |
| Payload | 384 B | 16 x 12 `int16` LE, degrees C x10 |

Total: **423 bytes** per frame.
