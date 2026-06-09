# NeoPixel Leak Proximity Indicator — Design Spec
_Date: 2026-05-12_

## Goal

Add a 4-LED NeoPixel proximity indicator to the Teensy 4.1 firmware. As the SPH0641 microphone gets closer to the leak source, the firmware lights more LEDs using fixed SNR thresholds.

## Hardware Connections

| Device | Signal | Connect To |
|---|---:|---|
| SPH0641 MEMS mic | CLK | Teensy 4.1 pin 21 |
| SPH0641 MEMS mic | DATA | Teensy 4.1 pin 8 |
| SPH0641 MEMS mic | SEL | GND |
| SPH0641 MEMS mic | VDD | 3.3V |
| SPH0641 MEMS mic | GND | GND |
| MLX90640 | SCL | Teensy 4.1 pin 19 |
| MLX90640 | SDA | Teensy 4.1 pin 18 |
| MLX90640 | VDD | 3.3V |
| MLX90640 | GND/VSS | GND |
| NeoPixel chain, 4 LEDs | DIN | Teensy 4.1 pin 5 |
| NeoPixel chain | V+ | 5V preferred, or 3.3V if reliable with the installed pixels |
| NeoPixel chain | GND | Common GND with Teensy and ESP32-S3 |
| ESP32-S3 XIAO | GPIO44 / D7 RX | Teensy pin 1 / TX1 |
| ESP32-S3 XIAO | GPIO43 / D6 TX | Teensy pin 0 / RX1 |
| ESP32-S3 | 3.3V | Shared regulated 3.3V |
| ESP32-S3 | GND | Common GND |

Recommended NeoPixel protection: place a 330-470 ohm resistor in series with the data line near the first NeoPixel. If powered from 5V, place a 470-1000 uF capacitor across NeoPixel V+ and GND.

## Firmware Behavior

The Teensy remains the sensor and LED controller. The ESP32-S3 remains a BLE conduit and does not need firmware changes.

The existing acoustic FFT pipeline computes `snr_db`. The new LED indicator maps `snr_db` to the number of lit NeoPixels:

| SNR | LEDs Lit | Meaning |
|---:|---:|---|
| `< 6 dB` | 0 | no clear leak signal |
| `6-9 dB` | 1 | weak/possible leak |
| `9-12 dB` | 2 | getting closer |
| `12-16 dB` | 3 | strong signal |
| `>= 16 dB` | 4 | very close |

Lit LEDs use a yellow-to-red gradient, with brightness increasing as the signal crosses higher thresholds. Unlit LEDs remain off.

## Scope

- Add `Adafruit_NeoPixel` to the Teensy firmware dependencies.
- Add NeoPixel constants and setup code in `firmware/src/main.cpp`.
- Add a small SNR-to-LED-count helper.
- Update LEDs after successful acoustic FFT updates.
- Leave BLE frame format unchanged.
- Leave `esp32s3/src/main.cpp` unchanged.

## Verification

- Build the Teensy firmware with PlatformIO.
- Confirm the ESP32-S3 firmware does not need changes.
- On hardware, move the SPH0641 closer to a leak source and verify LEDs increase from 0 through 4 as SNR rises.
