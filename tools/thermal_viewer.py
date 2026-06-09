"""
MLX90640 Handheld Thermal Wand - Alcohol Leak Detector Edition
--------------------------------------------------------------
Optimized for spotting rapid evaporative cooling caused by air pushing 
through an alcohol-wetted puncture.
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

# JET colormap goes from Blue (Cold) -> Green -> Yellow -> Red (Hot)
# Perfect for spotting the dark blue "ice spot" of evaporating alcohol.
COLORMAP = cv2.COLORMAP_JET

HEADER = b"\xff\xfe"
FRAME_META = 10        # snr_db(4) + ts_ms(4) + zone(1) + flags(1)
FRAME_BODY = ROWS * COLS * 2
FRAME_TOTAL = 2 + FRAME_META + FRAME_BODY  # 396

frame_queue = queue.Queue(maxsize=4)


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
                off = 2 + FRAME_META
                for _ in range(ROWS * COLS):
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

    cv2.namedWindow("Thermal Wand - Leak Detector", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Thermal Wand - Leak Detector", DISPLAY_W, DISPLAY_H)

    last_frame = None

    while True:
        try:
            frame = frame_queue.get(timeout=0.05)
            last_frame = frame
        except queue.Empty:
            frame = last_frame

        if frame is not None:
            # --- ALCOHOL ALGORITHM MODIFICATIONS ---
            # Use the median tire temperature as our baseline anchor
            median_temp = np.median(frame)
            coldest_spot = float(frame.min())
            
            # Alcohol evaporation drops the temperature quickly. 
            # We enforce a strict, narrow 4.0°C window around the tire's ambient baseline.
            # Anything colder than (median - 3.5°C) will completely saturate to deep blue.
            lo = median_temp - 3.5
            hi = median_temp + 0.5
            spread = hi - lo

            # Normalize the frame to this specific target window
            norm = ((frame - lo) / spread * 255).clip(0, 255).astype(np.uint8)
            colored = cv2.applyColorMap(norm, COLORMAP)
            
            # INTER_CUBIC smooths out the 32x24 blocks into a readable fluid plume
            display = cv2.resize(
                colored, (DISPLAY_W, DISPLAY_H), interpolation=cv2.INTER_CUBIC
            )

            # --- USER INTERFACE FOR THE WAND ---
            # Print standard metrics
            cv2.putText(
                display, f"Tire Baseline: {median_temp:.1f}C", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
            )
            cv2.putText(
                display, f"Coldest Point: {coldest_spot:.1f}C", (10, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2
            )

            # Look for a sharp negative delta (Coldest spot is significantly below tire baseline)
            delta = median_temp - coldest_spot
            if delta > 2.5:  # Adjust this threshold if it triggers on ambient variations
                cv2.putText(
                    display, "!!! LEAK DETECTED !!!", (10, DISPLAY_H - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3
                )

            cv2.imshow("Thermal Wand - Leak Detector", display)

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