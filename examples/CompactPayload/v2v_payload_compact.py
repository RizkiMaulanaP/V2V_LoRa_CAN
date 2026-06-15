#!/usr/bin/env python3
"""
DRAFT compact V2V payload codec (Python side) — mirrors v2v_payload_compact.h.

11-byte packed replacement for the 19-byte V2V_Payload (−27 % LoRa airtime at
SF9/BW125/CR4-8: 247 ms → 181 ms).  Both this file and the firmware header MUST
share the same constants and byte order (little-endian).

This is a DRAFT for review — it is NOT imported by main_ukf.py / simulate_v2v.py
yet.  Run it directly to see a round-trip precision check:

    python v2v_payload_compact.py
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

# ---- reference window (MUST match v2v_payload_compact.h) --------------------
REF_LAT = -7.0
REF_LON = 107.0
LAT_SPAN = 2.0      # ±1° around ref
LON_SPAN = 2.0
FIX24_MAX = float(0xFFFFFF)   # 2^24 - 1

SIZE = 11


@dataclass
class V2VCompact:
    node_id: int          # 0..63
    alert_type: int       # 0..3
    heading_deg: float    # 0..360
    speed: int            # km/h, 0..255
    latitude: float
    longitude: float
    tx_ts16: int          # millis & 0xFFFF


def _enc_axis(v: float, ref: float, span: float) -> int:
    lo = ref - span * 0.5
    u = (v - lo) / span * FIX24_MAX
    return max(0, min(0xFFFFFF, round(u)))


def _dec_axis(u: int, ref: float, span: float) -> float:
    lo = ref - span * 0.5
    return lo + (u / FIX24_MAX) * span


def pack(p: V2VCompact) -> bytes:
    b = bytearray(SIZE)
    b[0] = ((p.node_id & 0x3F) << 2) | (p.alert_type & 0x03)
    b[1] = round(p.heading_deg / 360.0 * 256.0) & 0xFF
    b[2] = p.speed & 0xFF
    la = _enc_axis(p.latitude, REF_LAT, LAT_SPAN)
    lo = _enc_axis(p.longitude, REF_LON, LON_SPAN)
    b[3], b[4], b[5] = la & 0xFF, (la >> 8) & 0xFF, (la >> 16) & 0xFF
    b[6], b[7], b[8] = lo & 0xFF, (lo >> 8) & 0xFF, (lo >> 16) & 0xFF
    struct.pack_into("<H", b, 9, p.tx_ts16 & 0xFFFF)
    return bytes(b)


def unpack(data: bytes) -> V2VCompact:
    if len(data) < SIZE:
        raise ValueError(f"need {SIZE} bytes, got {len(data)}")
    node_id = (data[0] >> 2) & 0x3F
    alert = data[0] & 0x03
    heading = data[1] * (360.0 / 256.0)
    speed = data[2]
    la = data[3] | (data[4] << 8) | (data[5] << 16)
    lo = data[6] | (data[7] << 8) | (data[8] << 16)
    tx_ts16 = struct.unpack_from("<H", data, 9)[0]
    return V2VCompact(node_id, alert, heading, speed,
                      _dec_axis(la, REF_LAT, LAT_SPAN),
                      _dec_axis(lo, REF_LON, LON_SPAN), tx_ts16)


if __name__ == "__main__":
    import math

    def haversine_m(a_lat, a_lon, b_lat, b_lon):
        R = 6371000.0
        p1, p2 = math.radians(a_lat), math.radians(b_lat)
        dphi = math.radians(b_lat - a_lat)
        dlmb = math.radians(b_lon - a_lon)
        h = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
        return 2 * R * math.asin(math.sqrt(h))

    samples = [
        V2VCompact(4, 2, 254.6, 37, -6.97588633, 107.63095150, 34148 & 0xFFFF),
        V2VCompact(1, 0, 0.0, 0, -6.97000000, 107.62151000, 9999),
        V2VCompact(63, 3, 359.9, 255, -6.99999000, 107.65332000, 65535),
    ]
    print(f"compact size = {SIZE} bytes  (vs 19 — airtime 247→181 ms @ SF9/CR4-8)\n")
    worst_pos = worst_hdg = 0.0
    for s in samples:
        raw = pack(s)
        d = unpack(raw)
        pos_err = haversine_m(s.latitude, s.longitude, d.latitude, d.longitude)
        hdg_err = abs(((d.heading_deg - s.heading_deg + 180) % 360) - 180)
        worst_pos = max(worst_pos, pos_err)
        worst_hdg = max(worst_hdg, hdg_err)
        print(f"node {d.node_id} alert {d.alert_type} spd {d.speed} "
              f"hdg {d.heading_deg:6.2f}° pos_err {pos_err*100:5.2f} cm "
              f"hdg_err {hdg_err:.2f}°  bytes={raw.hex()}")
    print(f"\nworst position error: {worst_pos*100:.2f} cm   "
          f"worst heading error: {worst_hdg:.2f}°")
