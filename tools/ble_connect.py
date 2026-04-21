"""
ble_connect.py — Serial bridge for PSSS sensor data
Reads $PSSS sentences (USB serial from Teensy) and/or binary HM-10 frames from the
same wire, and broadcasts parsed packets over a local WebSocket so web/index.html
can display them.

Usage:
    pip install pyserial websockets
    python ble_connect.py --port /dev/tty.usbmodem* --baud 115200

Dependencies:
    pyserial >= 3.5
    websockets >= 11.0
"""

import argparse
import asyncio
import json
import struct
from datetime import datetime

import serial
import serial.tools.list_ports
import websockets

WEBSOCKET_HOST = "localhost"
WEBSOCKET_PORT = 8765

# Binary frame (must match firmware + streamlit/app.html)
HEADER = b"\xff\xfe"
FRAME_META = 9  # snr f32 + peak f32 + flags u8
SPECTRUM_BYTES = 24
FRAME_BODY = 768 * 2
FRAME_TOTAL = 2 + FRAME_META + SPECTRUM_BYTES + FRAME_BODY  # 1571

# All connected WebSocket clients
clients: set = set()


def parse_psss(line: str) -> dict | None:
    """Parse a $PSSS sentence into a dict.

    Short form: $PSSS,<status>,<snr>,<peak_hz>,<temp_c>*
    Full form:  same + 24 comma-separated spectrum bytes (0–255) after temp.
    """
    line = line.strip()
    if not line.startswith("$PSSS"):
        return None
    try:
        body = line.lstrip("$").rstrip("*")
        parts = body.split(",")
        out: dict = {
            "timestamp": datetime.now().isoformat(),
            "status": int(parts[1]),
            "value1": float(parts[2]),
            "value2": float(parts[3]),
            "value3": float(parts[4]),
            "raw": line,
        }
        if len(parts) >= 5 + SPECTRUM_BYTES:
            out["spectrum"] = [int(parts[i]) for i in range(5, 5 + SPECTRUM_BYTES)]
        return out
    except (IndexError, ValueError):
        return None


def parse_binary_frame(chunk: bytes) -> dict | None:
    """Parse one HM-10 frame into the same dict shape as parse_psss."""
    if len(chunk) != FRAME_TOTAL or chunk[0:2] != HEADER:
        return None
    snr, peak, flags = struct.unpack_from("<ffB", chunk, 2)
    spectrum = list(chunk[11 : 11 + SPECTRUM_BYTES])
    min_t = float("inf")
    off = 11 + SPECTRUM_BYTES
    for i in range(768):
        (px,) = struct.unpack_from("<h", chunk, off + i * 2)
        t = px / 10.0
        if t < min_t:
            min_t = t
    acoustic = bool(flags & 1)
    thermal = bool(flags & 2)
    fused = acoustic and thermal
    status = 0 if fused else 1
    raw = f"$PSSS,{status},{snr:.2f},{peak:.0f},{min_t:.2f}*"
    return {
        "timestamp": datetime.now().isoformat(),
        "status": status,
        "value1": snr,
        "value2": peak,
        "value3": min_t,
        "flags": flags,
        "acoustic_leak": acoustic,
        "thermal_anomaly": thermal,
        "spectrum": spectrum,
        "raw": raw,
    }


def _trim_rx_buffer(buf: bytearray) -> None:
    """Avoid unbounded growth when no sync byte is found."""
    if len(buf) > 16000:
        del buf[:-1]


async def serial_reader(port: str, baud: int, verbose: bool) -> None:
    """Read from serial (mixed text lines + optional binary) and broadcast JSON."""
    print(f"Opening serial port {port} at {baud} baud...")
    ser = serial.Serial(port, baud, timeout=0.05)
    print("Serial port open. Waiting for data...")
    loop = asyncio.get_event_loop()
    buf = bytearray()

    while True:
        chunk = await loop.run_in_executor(None, ser.read, 4096)
        if chunk:
            buf.extend(chunk)
            if verbose:
                print(f"  [rx] +{len(chunk)} bytes (buffer {len(buf)})")

        # --- Extract newline-terminated text lines ---
        while True:
            nl = buf.find(b"\n")
            if nl < 0:
                break
            line_bytes = bytes(buf[:nl])
            del buf[: nl + 1]
            text = line_bytes.decode("utf-8", errors="replace").strip()
            packet = parse_psss(text)
            if packet:
                message = json.dumps(packet)
                print(f"  -> {message}")
                if clients:
                    await asyncio.gather(*(c.send(message) for c in clients))
            elif text:
                if verbose:
                    print(f"  [serial] {text[:300]}{'…' if len(text) > 300 else ''}")
                raw_msg = json.dumps(
                    {"raw": text, "timestamp": datetime.now().isoformat()}
                )
                if clients:
                    await asyncio.gather(*(c.send(raw_msg) for c in clients))

        # --- Extract complete binary frames (0xFF 0xFE …) ---
        while True:
            hi = buf.find(HEADER)
            if hi < 0:
                _trim_rx_buffer(buf)
                break
            if hi > 0:
                del buf[:hi]
            if len(buf) < FRAME_TOTAL:
                break
            if buf[0:2] != HEADER:
                del buf[:1]
                continue
            frame = bytes(buf[:FRAME_TOTAL])
            packet = parse_binary_frame(frame)
            del buf[:FRAME_TOTAL]
            if packet:
                message = json.dumps(packet)
                print(f"  -> {message}")
                if clients:
                    await asyncio.gather(*(c.send(message) for c in clients))

        if not chunk:
            await asyncio.sleep(0.02)


async def websocket_handler(websocket):
    """Handle a WebSocket client connection."""
    clients.add(websocket)
    print(f"[WS] Client connected ({len(clients)} total)")
    try:
        await websocket.wait_closed()
    finally:
        clients.discard(websocket)
        print(f"[WS] Client disconnected ({len(clients)} remaining)")


async def main(port: str, baud: int, verbose: bool) -> None:
    print(f"Starting WebSocket server on ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    async with websockets.serve(websocket_handler, WEBSOCKET_HOST, WEBSOCKET_PORT):
        await serial_reader(port, baud, verbose)


def list_ports():
    print("Available serial ports:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}  —  {p.description}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PSSS BLE/serial → WebSocket bridge")
    parser.add_argument("--port", help="Serial port (e.g. /dev/tty.usbmodem14101 or COM3)")
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200, match Teensy + HM-10 AT+BAUD4)",
    )
    parser.add_argument(
        "--list-ports", action="store_true", help="List available serial ports and exit"
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Print raw serial lines and byte counts to the terminal (debug)",
    )
    args = parser.parse_args()

    if args.list_ports:
        list_ports()
    elif not args.port:
        print("Error: --port is required. Use --list-ports to see available ports.")
        list_ports()
    else:
        asyncio.run(main(args.port, args.baud, args.verbose))
