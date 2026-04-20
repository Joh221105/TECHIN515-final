# TECHIN515 Final — PSSS Leak Sensor Dashboard

Real-time sensor dashboard for a Teensy 4.1 + HM-10 BLE leak detector. The firmware sends `$PSSS` packets over BLE/serial; a Python bridge forwards them to a browser dashboard via WebSocket.

## Hardware

- **Teensy 4.1** — microcontroller
- **HM-10 BLE module** — wired to Teensy UART1 (TX1 → HM-10 RXD, RX1 → HM-10 TXD)
- USB cable from Teensy to your computer

## Project Structure

```
firmware/   PlatformIO project for Teensy 4.1
web/        Browser dashboard (index.html)
tools/      Python serial → WebSocket bridge
```

## Running the App

### 1. Flash the firmware

Open the `firmware/` folder in VS Code with PlatformIO installed, then **Upload** to the Teensy.

### 2. Install Python dependencies

```bash
pip install pyserial websockets
```

### 3. Find your serial port

```bash
python tools/ble_connect.py --list-ports
```

### 4. Start the bridge

```bash
python tools/ble_connect.py --port /dev/tty.usbmodem14101 --baud 9600
```

Replace `/dev/tty.usbmodem14101` with your actual port (`COM3` etc. on Windows).

### 5. Open the dashboard

Open `web/index.html` in a browser. It connects automatically to `ws://localhost:8765` and displays live sensor data.

## Data Format

The firmware transmits `$PSSS` sentences every 250 ms:

```
$PSSS,<status>,<value1>,<value2>,<value3>*
```

| Field   | Type  | Description              |
|---------|-------|--------------------------|
| status  | int   | 1 = OK, 0 = ALERT        |
| value1  | float | Sensor reading 1         |
| value2  | float | Sensor reading 2         |
| value3  | float | Sensor reading 3         |
