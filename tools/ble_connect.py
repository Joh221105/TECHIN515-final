"""
ble_connect.py — Serial bridge for PSSS sensor data
Reads $PSSS sentences from Teensy (USB serial or HM-10 BLE serial adapter)
and broadcasts each packet over a local WebSocket so web/index.html can display it.

Usage:
    pip install pyserial websockets
    python ble_connect.py --port /dev/tty.usbmodem* --baud 9600

Dependencies:
    pyserial >= 3.5
    websockets >= 11.0
"""

import asyncio
import argparse
import serial
import serial.tools.list_ports
import websockets
import json
from datetime import datetime

WEBSOCKET_HOST = "localhost"
WEBSOCKET_PORT = 8765

# All connected WebSocket clients
clients: set = set()


def parse_psss(line: str) -> dict | None:
    """Parse a $PSSS sentence into a dict.

    Expected format: $PSSS,<status>,<value1>,<value2>,<value3>*
    Example:         $PSSS,1,8.50,312,3.20*
    """
    line = line.strip()
    if not line.startswith("$PSSS"):
        return None
    try:
        body = line.lstrip("$").rstrip("*")
        parts = body.split(",")
        return {
            "timestamp": datetime.now().isoformat(),
            "status": int(parts[1]),
            "value1": float(parts[2]),
            "value2": float(parts[3]),
            "value3": float(parts[4]),
            "raw": line,
        }
    except (IndexError, ValueError):
        return None


async def serial_reader(port: str, baud: int):
    """Read lines from serial port and broadcast parsed packets to all WebSocket clients."""
    print(f"Opening serial port {port} at {baud} baud...")
    ser = serial.Serial(port, baud, timeout=1)
    print("Serial port open. Waiting for data...")
    loop = asyncio.get_event_loop()

    while True:
        line = await loop.run_in_executor(None, ser.readline)
        if not line:
            continue
        text = line.decode("utf-8", errors="replace")
        packet = parse_psss(text)
        if packet:
            message = json.dumps(packet)
            print(f"  -> {message}")
            # Broadcast to all connected websocket clients
            if clients:
                await asyncio.gather(*(c.send(message) for c in clients))
        else:
            # Forward raw non-PSSS lines for debug visibility
            raw_msg = json.dumps({"raw": text.strip(), "timestamp": datetime.now().isoformat()})
            if clients:
                await asyncio.gather(*(c.send(raw_msg) for c in clients))


async def websocket_handler(websocket):
    """Handle a WebSocket client connection."""
    clients.add(websocket)
    print(f"[WS] Client connected ({len(clients)} total)")
    try:
        await websocket.wait_closed()
    finally:
        clients.discard(websocket)
        print(f"[WS] Client disconnected ({len(clients)} remaining)")


async def main(port: str, baud: int):
    print(f"Starting WebSocket server on ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    async with websockets.serve(websocket_handler, WEBSOCKET_HOST, WEBSOCKET_PORT):
        await serial_reader(port, baud)


def list_ports():
    print("Available serial ports:")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device}  —  {p.description}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PSSS BLE/serial → WebSocket bridge")
    parser.add_argument("--port", help="Serial port (e.g. /dev/tty.usbmodem14101 or COM3)")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("--list-ports", action="store_true", help="List available serial ports and exit")
    args = parser.parse_args()

    if args.list_ports:
        list_ports()
    elif not args.port:
        print("Error: --port is required. Use --list-ports to see available ports.")
        list_ports()
    else:
        asyncio.run(main(args.port, args.baud))
