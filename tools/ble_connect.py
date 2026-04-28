"""
ble_connect.py — BLE-to-WebSocket bridge for PSSS sensor data.

Usage:
    pip install bleak websockets
    python ble_connect.py
    python ble_connect.py --ble-address AA:BB:CC:DD:EE:FF   # skip scan

Dependencies:
    websockets >= 11.0
    bleak >= 0.21
"""

import argparse
import asyncio
import json
import struct
from datetime import datetime

import websockets

WEBSOCKET_HOST = "localhost"
WEBSOCKET_PORT = 8765

# ESP32-S3 GATT identifiers (ffe0/ffe1 — same as legacy HM-10, no client changes needed)
BLE_SERVICE = "0000ffe0-0000-1000-8000-00805f9b34fb"
BLE_CHAR    = "0000ffe1-0000-1000-8000-00805f9b34fb"

# Binary frame layout (must match firmware)
HEADER_THERMAL = b"\xff\xfe"
HEADER_CONTROL = b"\xff\xfc"
FRAME_META_THERMAL = 13   # snr f32 + peak f32 + thermal_ts u32 + flags u8
FRAME_META_CONTROL = 9    # snr f32 + peak f32 + flags u8
SPECTRUM_BYTES = 24
THERMAL_W      = 16
THERMAL_H      = 12
THERMAL_PIXELS = THERMAL_W * THERMAL_H
FRAME_BODY_THERMAL = THERMAL_PIXELS * 2  # int16 thermal payload
FRAME_TOTAL_THERMAL = 2 + FRAME_META_THERMAL + SPECTRUM_BYTES + FRAME_BODY_THERMAL  # 423
FRAME_TOTAL_CONTROL = 2 + FRAME_META_CONTROL + SPECTRUM_BYTES  # 35

# All connected WebSocket clients
clients: set = set()


# ── Parsers ────────────────────────────────────────────────────────────────────

def parse_thermal_frame(chunk: bytes) -> dict | None:
    """Decode one thermal frame (0xFF 0xFE) into a JSON-ready dict."""
    if len(chunk) != FRAME_TOTAL_THERMAL or chunk[0:2] != HEADER_THERMAL:
        return None
    snr, peak, thermal_ts_ms, flags = struct.unpack_from("<ffIB", chunk, 2)
    spectrum = list(chunk[15: 15 + SPECTRUM_BYTES])
    off = 15 + SPECTRUM_BYTES
    temps: list[float] = []
    for i in range(THERMAL_PIXELS):
        (px,) = struct.unpack_from("<h", chunk, off + i * 2)
        temps.append(px / 10.0)
    min_t = min(temps)
    acoustic = bool(flags & 1)
    thermal  = bool(flags & 2)
    status   = 0 if acoustic else 1
    return {
        "kind":            "thermal",
        "timestamp":       datetime.now().isoformat(),
        "status":          status,
        "snr":             snr,
        "peak_freq":       peak,
        "thermal_ts_ms":   thermal_ts_ms,
        "min_temp":        min_t,
        # legacy aliases
        "value1":          snr,
        "value2":          peak,
        "value3":          min_t,
        "flags":           flags,
        "acoustic_leak":   acoustic,
        "thermal_anomaly": thermal,
        "spectrum":        spectrum,
        "thermal":         temps,
        "thermal_w":       THERMAL_W,
        "thermal_h":       THERMAL_H,
        "raw":             f"$PSSS,{status},{snr:.2f},{peak:.0f},{min_t:.2f}*",
    }

def parse_control_frame(chunk: bytes) -> dict | None:
    """Decode one control frame (0xFF 0xFC) into a JSON-ready dict."""
    if len(chunk) != FRAME_TOTAL_CONTROL or chunk[0:2] != HEADER_CONTROL:
        return None
    snr, peak, flags = struct.unpack_from("<ffB", chunk, 2)
    spectrum = list(chunk[11: 11 + SPECTRUM_BYTES])
    acoustic = bool(flags & 1)
    thermal = bool(flags & 2)
    status = 0 if acoustic else 1
    return {
        "kind":            "control",
        "timestamp":       datetime.now().isoformat(),
        "status":          status,
        "snr":             snr,
        "peak_freq":       peak,
        "value1":          snr,
        "value2":          peak,
        "flags":           flags,
        "acoustic_leak":   acoustic,
        "thermal_anomaly": thermal,
        "spectrum":        spectrum,
        "raw":             f"$PSSS,{status},{snr:.2f},{peak:.0f},*",
    }


# ── Frame reassembly helper ────────────────────────────────────────────────────

async def _dispatch_frames(buf: bytearray, verbose: bool) -> None:
    """Extract complete binary frames from buf, broadcast each as JSON."""
    while True:
        hi = -1
        for i in range(len(buf) - 1):
            if buf[i] == 0xFF and (buf[i + 1] == 0xFE or buf[i + 1] == 0xFC):
                hi = i
                break
        if hi < 0:
            if len(buf) > 16000:
                buf.clear()
            break
        if hi > 0:
            del buf[:hi]
        if len(buf) < 2:
            break
        frame_total = FRAME_TOTAL_THERMAL if buf[1] == 0xFE else FRAME_TOTAL_CONTROL
        if len(buf) < frame_total:
            break
        frame = bytes(buf[:frame_total])
        packet = parse_thermal_frame(frame) if buf[1] == 0xFE else parse_control_frame(frame)
        del buf[:frame_total]
        if packet and clients:
            msg = json.dumps(packet)
            if verbose:
                print(f"  [{packet['kind']}] snr={packet['snr']:.1f} status={packet['status']}")
            await asyncio.gather(*(c.send(msg) for c in clients))


# ── BLE mode ──────────────────────────────────────────────────────────────────

async def ble_reader(address: str | None, verbose: bool) -> None:
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        print("ERROR: bleak not installed. Run: pip install bleak")
        return

    buf = bytearray()

    async def on_notify(sender, data: bytearray) -> None:
        buf.extend(data)
        await _dispatch_frames(buf, verbose)

    # Resolve device
    if address:
        device = address
        print(f"Connecting directly to {address}...")
    else:
        print(f"Scanning for PSSS-Sensor (service {BLE_SERVICE})...")
        device = await BleakScanner.find_device_by_filter(
            lambda d, adv: any(
                BLE_SERVICE.lower() in str(s).lower()
                for s in (adv.service_uuids or [])
            ),
            timeout=10.0,
        )
        if device is None:
            print("ESP32-S3 not found by service UUID. Scanning all devices...\n")
            found = await BleakScanner.discover(timeout=5.0)
            for d in found:
                print(f"  {d.address}  {d.name or '(no name)'}")
            print(
                "\nRun again with --ble-address <ADDRESS> to connect by address,"
                "\nor pair the ESP32-S3 in System Settings > Bluetooth first."
            )
            return

    print(f"Found device: {getattr(device, 'name', device)} — connecting...")
    async with BleakClient(device) as client:
        print(f"Connected. Subscribing to {BLE_CHAR}...")
        await client.start_notify(BLE_CHAR, on_notify)
        print("Streaming BLE frames. Press Ctrl+C to stop.\n")
        while client.is_connected:
            await asyncio.sleep(1.0)
    print("BLE device disconnected.")


# ── WebSocket server ───────────────────────────────────────────────────────────

async def websocket_handler(websocket) -> None:
    clients.add(websocket)
    print(f"[WS] client connected ({len(clients)} total)")
    try:
        await websocket.wait_closed()
    finally:
        clients.discard(websocket)
        print(f"[WS] client disconnected ({len(clients)} remaining)")


async def main(args) -> None:
    print(f"WebSocket server → ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    async with websockets.serve(websocket_handler, WEBSOCKET_HOST, WEBSOCKET_PORT):
        await ble_reader(args.ble_address, args.verbose)


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PSSS BLE sensor → WebSocket bridge")

    parser.add_argument("--ble-address", metavar="ADDR",
                        help="Skip scan and connect directly by BLE address (e.g. AA:BB:CC:DD:EE:FF)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose debug output")
    args = parser.parse_args()
    asyncio.run(main(args))
