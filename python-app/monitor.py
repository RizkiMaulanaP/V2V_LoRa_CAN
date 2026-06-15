#!/usr/bin/env python3
"""
V2V Raw Data Monitor
Reads and prints binary sensor frames from the Heltec Wireless Tracker.

Usage:
    python monitor.py [--port /dev/ttyACM0] [--baud 921600]

Requires:
    pip install pyserial
"""

import sys
import struct
import argparse
from datetime import datetime
from functools import reduce
from typing import Optional

import serial

# ── Protocol constants ───────────────────────────────────────────────────────
FRAME_START = 0xAA
FRAME_END   = 0x55

FRAME_IMU     = 0x01
FRAME_GPS     = 0x02
FRAME_OBD     = 0x03
FRAME_LORA_RX = 0x04

FRAME_SIZES = {
    FRAME_IMU:     32,
    FRAME_GPS:     34,
    FRAME_OBD:     9,
    FRAME_LORA_RX: 22,
}

FRAME_STRUCTS = {
    FRAME_IMU:     struct.Struct("<BIffffffBB"),
    FRAME_GPS:     struct.Struct("<BIddffBBBB"),
    FRAME_OBD:     struct.Struct("<BIBBB"),
    FRAME_LORA_RX: struct.Struct("<BIBffBBhbBB"),
}

ALERT_LABELS = {0: "Normal", 1: "Traffic Jam", 2: "Road Damage"}

# ── Checksum ─────────────────────────────────────────────────────────────────
def xor_checksum(data: bytes) -> int:
    return reduce(lambda a, b: a ^ b, data, 0)

# ── Frame reader ─────────────────────────────────────────────────────────────
def read_frame(ser: serial.Serial) -> Optional[dict]:
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == FRAME_START:
            break

    t = ser.read(1)
    if not t or t[0] not in FRAME_SIZES:
        return None
    frame_type = t[0]

    rest = ser.read(FRAME_SIZES[frame_type] - 2)
    if len(rest) < FRAME_SIZES[frame_type] - 2:
        return None

    body = t + rest
    if body[-1] != FRAME_END:
        return None
    if xor_checksum(body[:-2]) != body[-2]:
        return None

    return _parse(frame_type, body)

def _parse(frame_type: int, body: bytes) -> Optional[dict]:
    fmt    = FRAME_STRUCTS[frame_type]
    fields = fmt.unpack(body[:fmt.size])

    if frame_type == FRAME_IMU:
        _, ts, ax, ay, az, gx, gy, gz, _, _ = fields
        return {"type": "IMU", "ts": ts,
                "ax": ax, "ay": ay, "az": az,
                "gx": gx, "gy": gy, "gz": gz}

    if frame_type == FRAME_GPS:
        _, ts, lat, lon, spd, hdop, sats, fix, _, _ = fields
        return {"type": "GPS", "ts": ts,
                "lat": lat, "lon": lon, "speed_kmh": spd,
                "hdop": hdop, "satellites": sats, "fix": bool(fix)}

    if frame_type == FRAME_OBD:
        _, ts, spd, _, _ = fields
        return {"type": "OBD", "ts": ts, "speed_kmh": spd}

    if frame_type == FRAME_LORA_RX:
        _, ts, nid, lat, lon, spd, alert, rssi, snr, _, _ = fields
        return {"type": "LORA_RX", "ts": ts,
                "node_id": nid, "lat": lat, "lon": lon, "speed_kmh": spd,
                "alert": ALERT_LABELS.get(alert, f"Unknown({alert})"),
                "rssi": rssi, "snr": snr}

    return None

# ── Display ──────────────────────────────────────────────────────────────────
def print_frame(frame: dict) -> None:
    now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    ts  = frame["ts"]
    t   = frame["type"]

    if t == "IMU":
        print(
            f"[{now}] IMU     | ts={ts:9d} ms | "
            f"a=[{frame['ax']:+7.3f}, {frame['ay']:+7.3f}, {frame['az']:+7.3f}] m/s²  "
            f"g=[{frame['gx']:+8.4f}, {frame['gy']:+8.4f}, {frame['gz']:+8.4f}] rad/s"
        )

    elif t == "GPS":
        fix_s = "FIX   " if frame["fix"] else "NO FIX"
        print(
            f"[{now}] GPS     | ts={ts:9d} ms | {fix_s} | "
            f"lat={frame['lat']:>13.6f}  lon={frame['lon']:>13.6f} | "
            f"spd={frame['speed_kmh']:5.1f} km/h | "
            f"hdop={frame['hdop']:.1f}  sats={frame['satellites']}"
        )

    elif t == "OBD":
        print(
            f"[{now}] OBD     | ts={ts:9d} ms | "
            f"speed={frame['speed_kmh']:3d} km/h"
        )

    elif t == "LORA_RX":
        print(
            f"[{now}] LORA RX | ts={ts:9d} ms | "
            f"node={frame['node_id']:2d} | "
            f"lat={frame['lat']:>13.6f}  lon={frame['lon']:>13.6f} | "
            f"spd={frame['speed_kmh']:3d} km/h | "
            f"alert={frame['alert']:<12} | "
            f"rssi={frame['rssi']:4d} dBm  snr={frame['snr']:3d} dB"
        )

# ── Main ─────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(description="V2V raw data monitor")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600)
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud} baud ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=2)
    except serial.SerialException as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    print("Listening for frames — Ctrl+C to stop\n")
    bad_streak = 0

    try:
        while True:
            frame = read_frame(ser)
            if frame is None:
                bad_streak += 1
                if bad_streak == 5:
                    print("[warn] No valid frames — check USB CDC On Boot setting")
                continue
            bad_streak = 0
            print_frame(frame)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
