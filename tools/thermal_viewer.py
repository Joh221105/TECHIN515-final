"""
MLX90640 Real-Time Thermal Heatmap Viewer (OpenCV)
---------------------------------------------------
Reads binary HM-10 frames (0xFF 0xFE + metadata + 24 spectrum bytes + 768×int16 °C×10) from serial,
or legacy ASCII lines ``FRAME:`` + comma-separated floats (24×32).

Usage:
    pip install pyserial opencv-python numpy
    python thermal_viewer.py --port /dev/tty.usbserial-* --protocol binary   # HM-10 dongle / UART
    python thermal_viewer.py --port /dev/tty.usbmodem* --protocol binary   # if serial carries binary
    python thermal_viewer.py --port COM3 --protocol frame                  # legacy ASCII only

Binary mode matches ``firmware/src/main.cpp`` and ``streamlit/app.html`` (1571 bytes per frame).
Teensy USB CDC normally carries text + ``$PSSS`` lines only; use a USB–serial adapter on
UART1/HM-10 to view the heatmap in binary mode.
"""

import argparse
import queue
import struct
import sys
import threading

import cv2
import numpy as np
import serial

ROWS, COLS = 24, 32
DISPLAY_W, DISPLAY_H = 640, 480
BAUD = 115200
COLORMAP = cv2.COLORMAP_INFERNO  # swap to COLORMAP_JET, COLORMAP_HOT, etc.

HEADER = b"\xff\xfe"
FRAME_META = 9
SPECTRUM_BYTES = 24
FRAME_BODY = 768 * 2
FRAME_TOTAL = 2 + FRAME_META + SPECTRUM_BYTES + FRAME_BODY

frame_queue: queue.Queue = queue.Queue(maxsize=4)


def _push_frame(frame: np.ndarray) -> None:
    if frame_queue.full():
        try:
            frame_queue.get_nowait()
        except queue.Empty:
            pass
    frame_queue.put(frame)


def serial_reader_binary(port: str) -> None:
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
        print(f"[serial] Connected to {port} (binary {FRAME_TOTAL}-byte frames)")
    except serial.SerialException as e:
        print(f"[error] {e}")
        sys.exit(1)

    buf = bytearray()
    while True:
        buf.extend(ser.read(4096))
        while True:
            hi = buf.find(HEADER)
            if hi < 0:
                if len(buf) > 16000:
                    del buf[:-1]
                break
            if hi > 0:
                del buf[:hi]
            if len(buf) < FRAME_TOTAL:
                break
            if buf[0:2] != HEADER:
                del buf[:1]
                continue
            chunk = bytes(buf[:FRAME_TOTAL])
            del buf[:FRAME_TOTAL]
            try:
                floats = []
                off = 11 + SPECTRUM_BYTES
                for _ in range(768):
                    (px,) = struct.unpack_from("<h", chunk, off)
                    off += 2
                    floats.append(px / 10.0)
                frame = np.array(floats, dtype=np.float32).reshape(ROWS, COLS)
                _push_frame(frame)
            except struct.error:
                continue


def serial_reader_frame(port: str) -> None:
    try:
        ser = serial.Serial(port, BAUD, timeout=2)
        print(f"[serial] Connected to {port} (ASCII FRAME: lines)")
    except serial.SerialException as e:
        print(f"[error] {e}")
        sys.exit(1)

    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            continue

        if not line.startswith("FRAME:"):
            if line:
                print(f"[device] {line}")
            continue

        try:
            values = list(map(float, line[6:].split(",")))
        except ValueError:
            continue

        if len(values) != ROWS * COLS:
            print(f"[warn] expected {ROWS * COLS} values, got {len(values)}")
            continue

        frame = np.array(values, dtype=np.float32).reshape(ROWS, COLS)
        _push_frame(frame)


def run_viewer(port: str, protocol: str) -> None:
    if protocol == "binary":
        target = serial_reader_binary
    else:
        target = serial_reader_frame

    t = threading.Thread(target=target, args=(port,), daemon=True)
    t.start()

    cv2.namedWindow("MLX90640 Thermal", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("MLX90640 Thermal", DISPLAY_W, DISPLAY_H)

    last_frame = None

    while True:
        try:
            frame = frame_queue.get(timeout=0.05)
            last_frame = frame
        except queue.Empty:
            frame = last_frame

        if frame is not None:
            lo, hi = float(frame.min()), float(frame.max())
            spread = hi - lo if hi - lo > 0.5 else 0.5

            norm = ((frame - lo) / spread * 255).clip(0, 255).astype(np.uint8)
            colored = cv2.applyColorMap(norm, COLORMAP)
            display = cv2.resize(
                colored, (DISPLAY_W, DISPLAY_H), interpolation=cv2.INTER_LINEAR
            )

            cv2.putText(
                display,
                f"min {lo:.1f}C",
                (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                display,
                f"max {hi:.1f}C",
                (10, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            cv2.imshow("MLX90640 Thermal", display)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Serial port")
    parser.add_argument(
        "--protocol",
        choices=("binary", "frame"),
        default="binary",
        help="binary = HM-10 1571-byte frames; frame = legacy FRAME: CSV lines",
    )
    args = parser.parse_args()
    run_viewer(args.port, args.protocol)
