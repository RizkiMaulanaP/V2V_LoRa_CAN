#!/usr/bin/env python3
"""
Real-time V2V simulator — replays recorded CSV logs to the Flutter dashboard.

Reads a processed_*.csv (ego trajectory) and its sibling raw_*.csv (LORA_RX
neighbour receptions + OBD_EXT engine telemetry), then re-streams them over TCP
as the *same* newline-JSON contract main_ukf.py emits.  The Flutter app
(TcpDataSource → localhost:8765) shows the recorded drive exactly as if it were
live — no hardware required.

How it mirrors the live host:
  • Ego pose comes straight from the processed CSV (ENU metres + lat/lon).
  • Neighbours appear/update at their LORA_RX reception timestamps, are
    dead-reckoned (constant velocity in ENU) between the ~2 s updates, and
    expire after --neighbor-ttl seconds of silence (mimics the live registry).
  • Corrupted LoRa fixes (0,0 or implausibly far from ego) are dropped, same
    as plot_path.py.

Usage:
    python simulate_v2v.py                      # newest processed_*.csv, :8765
    python simulate_v2v.py processed_x.csv
    python simulate_v2v.py --speed 2 --fps 30   # 2× fast, 30 frames/s
    python simulate_v2v.py --loop               # repeat forever
    python simulate_v2v.py --raw raw_x.csv --port 8765

Then launch the Flutter app (already pointing at localhost:8765). Switching the
dashboard source to MockDataSource is no longer needed — this *is* the host.
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from functools import reduce
from pathlib import Path

import numpy as np
import pandas as pd

from v2v_json import JsonTcpServer, frame_to_line  # noqa: F401 (frame_to_line for --dry-run)
import v2v_warnings as vw

_EARTH_R = 6378137.0  # WGS-84 equatorial radius (m)


def log(*a):
    """Diagnostics go to stderr so --dry-run stdout stays pure JSON."""
    print(*a, file=sys.stderr)


# CmdFrame wire format (matches main_ukf.py / firmware) for the optional
# --serial buzzer link.  CMD_BUZZER carries the pattern id in the alert byte.
CMD_START = 0xBB
FRAME_END = 0x55
CMD_BUZZER = 0x04
_CMD_STRUCT = struct.Struct("<BBBfffBBB")


def send_buzzer(ser, pattern: int) -> None:
    body = struct.pack("<BBfffB", CMD_BUZZER, pattern & 0xFF, 0.0, 0.0, 0.0, 0)
    cksum = reduce(lambda a, b: a ^ b, body, 0)
    ser.write(_CMD_STRUCT.pack(CMD_START, CMD_BUZZER, pattern & 0xFF,
                               0.0, 0.0, 0.0, 0, cksum, FRAME_END))


def lora_alert_to_int(a) -> int:
    """Recorded lora_alert value → numeric alert_type for the warning engine."""
    s = str(a).strip().upper()
    if s in ("", "NAN", "NONE", "0", "0.0", "NORMAL"):
        return vw.ALERT_NORMAL
    if s in ("2", "2.0", "EMERGENCY", "BRAKE", "HARD BRAKE"):
        return vw.ALERT_BRAKE
    return vw.ALERT_TRAFFIC_JAM     # 1 / "Traffic Jam" / anything else


# ── CSV discovery ───────────────────────────────────────────────────────────
def find_latest_processed(folder: Path) -> Path | None:
    cands = sorted(folder.glob("processed_*.csv"),
                   key=lambda p: p.stat().st_mtime, reverse=True)
    for p in cands:
        try:
            with p.open() as f:
                if sum(1 for _ in f) >= 3:   # header + >=2 rows
                    return p
        except OSError:
            continue
    return cands[0] if cands else None


def find_raw_for(processed: Path) -> Path | None:
    if processed.name.startswith("processed_"):
        cand = processed.with_name("raw_" + processed.name[len("processed_"):])
        if cand.exists():
            return cand
    return None


def first_present(df: pd.DataFrame, *names: str) -> str | None:
    for n in names:
        if n in df.columns:
            return n
    return None


# ── coordinate helpers (shared ENU frame) ───────────────────────────────────
def enu_from_latlon(lat, lon, lat0, lon0):
    x = np.radians(lon - lon0) * _EARTH_R * np.cos(np.radians(lat0))
    y = np.radians(lat - lat0) * _EARTH_R
    return x, y


def latlon_from_enu(x, y, lat0, lon0):
    lat = lat0 + math.degrees(y / _EARTH_R)
    lon = lon0 + math.degrees(x / (_EARTH_R * math.cos(math.radians(lat0))))
    return lat, lon


def lora_alert_to_status(a) -> str:
    """Map a recorded lora_alert value → Flutter EmergencyStatus name."""
    s = str(a).strip().upper()
    if s in ("", "NAN", "NONE", "0", "0.0", "NORMAL"):
        return "NORMAL"
    if s in ("2", "2.0", "EMERGENCY"):
        return "EMERGENCY"
    return "WARNING"     # 1 / "Traffic Jam" / "Road Damage" / anything else


# ── loaders ─────────────────────────────────────────────────────────────────
def load_ego(path: Path):
    """Ego timeline in a single ENU frame, plus that frame's geographic origin."""
    df = pd.read_csv(path)
    if df.empty:
        raise SystemExit(f"{path.name}: no data rows.")

    t_col = first_present(df, "mcu_ts_ms", "wall_time")
    if t_col == "mcu_ts_ms":
        t_ms = pd.to_numeric(df["mcu_ts_ms"], errors="coerce").to_numpy(dtype=float)
    elif t_col == "wall_time":
        wt = pd.to_datetime(df["wall_time"], errors="coerce")
        t_ms = (wt - wt.iloc[0]).dt.total_seconds().to_numpy() * 1000.0
    else:
        t_ms = np.arange(len(df), dtype=float) * 100.0   # assume ~10 Hz

    has_ll = {"lat", "lon"}.issubset(df.columns)
    has_xy = {"pos_east_m", "pos_north_m"}.issubset(df.columns)
    if not has_ll and not has_xy:
        raise SystemExit(f"{path.name}: need lat/lon or pos_east_m/pos_north_m.")

    lat = pd.to_numeric(df["lat"], errors="coerce").to_numpy() if has_ll else None
    lon = pd.to_numeric(df["lon"], errors="coerce").to_numpy() if has_ll else None

    if has_xy:
        x = pd.to_numeric(df["pos_east_m"], errors="coerce").to_numpy()
        y = pd.to_numeric(df["pos_north_m"], errors="coerce").to_numpy()
        # Recover the ENU origin by inverting one ego row, so neighbours project
        # into the exact same frame the firmware/host used.
        if has_ll:
            good = (np.isfinite(x) & np.isfinite(y)
                    & np.isfinite(lat) & np.isfinite(lon))
            i = int(np.argmax(good)) if good.any() else 0
            lat0 = lat[i] - math.degrees(y[i] / _EARTH_R)
            lon0 = lon[i] - math.degrees(x[i] / (_EARTH_R * math.cos(math.radians(lat0))))
        else:
            lat0 = lon0 = 0.0
            lat = np.full(len(df), float("nan"))
            lon = np.full(len(df), float("nan"))
    else:
        good = np.isfinite(lat) & np.isfinite(lon)
        i = int(np.argmax(good)) if good.any() else 0
        lat0, lon0 = float(lat[i]), float(lon[i])
        x, y = enu_from_latlon(lat, lon, lat0, lon0)

    if not has_ll:   # synthesise lat/lon from ENU for the JSON contract
        lat = np.array([latlon_from_enu(xi, yi, lat0, lon0)[0] for xi, yi in zip(x, y)])
        lon = np.array([latlon_from_enu(xi, yi, lat0, lon0)[1] for xi, yi in zip(x, y)])

    s_col = first_present(df, "speed_kmh", "speed_ms")
    if s_col == "speed_ms":
        speed = pd.to_numeric(df["speed_ms"], errors="coerce").to_numpy() * 3.6
    elif s_col:
        speed = pd.to_numeric(df["speed_kmh"], errors="coerce").to_numpy()
    else:
        speed = np.zeros(len(df))

    h_col = first_present(df, "heading_deg", "yaw_deg")
    heading = (pd.to_numeric(df[h_col], errors="coerce").to_numpy()
               if h_col else np.zeros(len(df)))

    ok = np.isfinite(t_ms) & np.isfinite(x) & np.isfinite(y)
    if not ok.any():
        raise SystemExit(f"{path.name}: no usable ego rows.")
    order = np.argsort(t_ms[ok])
    sel = np.flatnonzero(ok)[order]
    return {
        "t_ms": t_ms[sel], "lat": lat[sel], "lon": lon[sel],
        "x": x[sel], "y": y[sel],
        "speed_kmh": np.nan_to_num(speed[sel]),
        "heading_deg": np.nan_to_num(heading[sel]),
        "origin": (lat0, lon0),
    }


def load_neighbor_events(raw_path: Path, ego, clip_m: float):
    """LORA_RX receptions as time-sorted events in the ego ENU frame."""
    if raw_path is None or not raw_path.exists():
        return []
    try:
        df = pd.read_csv(raw_path)
    except (OSError, pd.errors.ParserError):
        return []
    if "frame_type" not in df.columns:
        return []
    rx = df[df["frame_type"].astype(str) == "LORA_RX"]
    if rx.empty or "lora_lat" not in rx.columns:
        return []

    lat0, lon0 = ego["origin"]
    ts = pd.to_numeric(rx.get("mcu_ts_ms"), errors="coerce").to_numpy(dtype=float)
    lat = pd.to_numeric(rx["lora_lat"], errors="coerce").to_numpy()
    lon = pd.to_numeric(rx["lora_lon"], errors="coerce").to_numpy()
    node = (pd.to_numeric(rx["lora_node"], errors="coerce").to_numpy()
            if "lora_node" in rx.columns else np.zeros(len(rx)))
    spd = (pd.to_numeric(rx["lora_speed_kmh"], errors="coerce").to_numpy()
           if "lora_speed_kmh" in rx.columns else np.zeros(len(rx)))
    hd = (pd.to_numeric(rx["lora_heading_deg"], errors="coerce").to_numpy()
          if "lora_heading_deg" in rx.columns else np.zeros(len(rx)))
    alert = (rx["lora_alert"].astype(str).to_numpy()
             if "lora_alert" in rx.columns else np.array([""] * len(rx)))

    events, dropped = [], 0
    for k in range(len(rx)):
        if not (np.isfinite(ts[k]) and np.isfinite(lat[k]) and np.isfinite(lon[k])):
            continue
        if lat[k] == 0.0 and lon[k] == 0.0:
            dropped += 1
            continue
        ex, ny = enu_from_latlon(lat[k], lon[k], lat0, lon0)
        if clip_m > 0:
            j = int(np.searchsorted(ego["t_ms"], ts[k]))
            j = min(max(j, 0), len(ego["t_ms"]) - 1)
            if math.hypot(ex - ego["x"][j], ny - ego["y"][j]) > clip_m:
                dropped += 1
                continue
        nid = int(node[k]) if np.isfinite(node[k]) else 0
        events.append({
            "ts": float(ts[k]), "node": nid,
            "east": float(ex), "north": float(ny),
            "speed_kmh": float(spd[k]) if np.isfinite(spd[k]) else 0.0,
            "heading_deg": float(hd[k]) if np.isfinite(hd[k]) else 0.0,
            "status": lora_alert_to_status(alert[k]),
            "alert": lora_alert_to_int(alert[k]),
        })
    events.sort(key=lambda e: e["ts"])
    if dropped:
        log(f"[sim] dropped {dropped} corrupted/zero LoRa fixes")
    return events


def load_gps(raw_path: Path):
    """Ego GPS-health timeline (hdop / satellites / fix_valid) from GPS frames."""
    if raw_path is None or not raw_path.exists():
        return None
    try:
        df = pd.read_csv(raw_path)
    except (OSError, pd.errors.ParserError):
        return None
    if "frame_type" not in df.columns:
        return None
    g = df[df["frame_type"].astype(str) == "GPS"]
    if g.empty:
        return None
    ts = pd.to_numeric(g.get("mcu_ts_ms"), errors="coerce").to_numpy(dtype=float)
    hdop = (pd.to_numeric(g["hdop"], errors="coerce").to_numpy()
            if "hdop" in g.columns else np.zeros(len(g)))
    sats = (pd.to_numeric(g["satellites"], errors="coerce").to_numpy()
            if "satellites" in g.columns else np.full(len(g), np.nan))
    fix = (pd.to_numeric(g["fix_valid"], errors="coerce").to_numpy()
           if "fix_valid" in g.columns else np.ones(len(g)))
    ok = np.isfinite(ts)
    order = np.argsort(ts[ok])
    sel = np.flatnonzero(ok)[order]
    return {"t_ms": ts[sel], "hdop": hdop[sel], "sats": sats[sel], "fix": fix[sel]}


def gps_at(gps, data_t_ms):
    if gps is None:
        return {"fix_valid": 1, "hdop": 0.0, "satellites": None}
    i = int(np.searchsorted(gps["t_ms"], data_t_ms, side="right")) - 1
    if i < 0:
        return {"fix_valid": 0, "hdop": 99.0, "satellites": 0}
    sats = gps["sats"][i]
    return {
        "fix_valid": int(gps["fix"][i]) if np.isfinite(gps["fix"][i]) else 1,
        "hdop": float(gps["hdop"][i]) if np.isfinite(gps["hdop"][i]) else 0.0,
        "satellites": int(sats) if np.isfinite(sats) else None,
    }


def load_obd_ext(raw_path: Path):
    """Engine RPM / coolant timeline for the ego (latest-at-or-before lookup)."""
    if raw_path is None or not raw_path.exists():
        return None
    try:
        df = pd.read_csv(raw_path)
    except (OSError, pd.errors.ParserError):
        return None
    if "frame_type" not in df.columns:
        return None
    ext = df[df["frame_type"].astype(str) == "OBD_EXT"]
    if ext.empty:
        return None
    ts = pd.to_numeric(ext.get("mcu_ts_ms"), errors="coerce").to_numpy(dtype=float)
    rpm = (pd.to_numeric(ext["engine_rpm"], errors="coerce").to_numpy()
           if "engine_rpm" in ext.columns else np.zeros(len(ext)))
    cool = (pd.to_numeric(ext["coolant_c"], errors="coerce").to_numpy()
            if "coolant_c" in ext.columns else np.zeros(len(ext)))
    ok = np.isfinite(ts)
    order = np.argsort(ts[ok])
    sel = np.flatnonzero(ok)[order]
    return {"t_ms": ts[sel], "rpm": np.nan_to_num(rpm[sel]),
            "cool": np.nan_to_num(cool[sel])}


# ── frame assembly ──────────────────────────────────────────────────────────
def ego_at(ego, data_t_ms):
    i = int(np.searchsorted(ego["t_ms"], data_t_ms))
    i = min(max(i, 0), len(ego["t_ms"]) - 1)
    return i


def obd_at(obd, data_t_ms):
    if obd is None:
        return 0, 0.0
    i = int(np.searchsorted(obd["t_ms"], data_t_ms, side="right")) - 1
    if i < 0:
        return 0, 0.0
    return int(obd["rpm"][i]), float(obd["cool"][i])


def build_sim_frame(data_t_ms, ego, obd, gps, neighbors, origin, fuel_pct):
    i = ego_at(ego, data_t_ms)
    rpm, cool = obd_at(obd, data_t_ms)
    gps_state = gps_at(gps, data_t_ms)
    lat0, lon0 = origin

    ego_e = float(ego["x"][i])
    ego_n = float(ego["y"][i])
    ego_speed = float(ego["speed_kmh"][i])
    ego_head = float(ego["heading_deg"][i])

    # Contract is geographic (lat/lon); the app derives distance/bearing itself.
    # x/y are kept too (ignored by the current app, used by older tooling).
    ego_obj = {
        "lat": round(float(ego["lat"][i]), 8),
        "lon": round(float(ego["lon"][i]), 8),
        "x": round(ego_e, 3),
        "y": round(ego_n, 3),
        "speed_kmh": round(ego_speed, 2),
        "heading_deg": round(ego_head, 2),
        "engine_rpm": int(rpm),
        "engine_temp_c": float(cool),
        "fuel_level_pct": round(float(fuel_pct), 1),   # synthetic — not in CSV
        "fix_valid": int(gps_state["fix_valid"]),
        "hdop": round(float(gps_state["hdop"]), 2),
        "satellites": gps_state["satellites"],
    }

    neigh_list = []
    warn_neighbors = []
    for nid, n in neighbors.items():
        dt = max(0.0, (data_t_ms - n["base_ts"]) / 1000.0)
        hd = math.radians(n["heading_deg"])
        v = n["speed_kmh"] / 3.6
        east = n["east"] + v * math.sin(hd) * dt      # heading 0°=North, CW
        north = n["north"] + v * math.cos(hd) * dt
        n_lat, n_lon = latlon_from_enu(east, north, lat0, lon0)
        neigh_list.append({
            "id": f"N{nid}",
            "lat": round(n_lat, 8), "lon": round(n_lon, 8),
            "x": round(east, 3), "y": round(north, 3),
            "speed_kmh": round(n["speed_kmh"], 2),
            "heading_deg": round(n["heading_deg"], 2),
            "emergency_status": n["status"],
        })
        warn_neighbors.append({
            "id": f"N{nid}", "east": east, "north": north,
            "speed_kmh": n["speed_kmh"], "heading_deg": n["heading_deg"],
            "alert": n.get("alert", vw.ALERT_NORMAL),
        })

    warning, buzzer = vw.assess(
        {"east": ego_e, "north": ego_n,
         "speed_kmh": ego_speed, "heading_deg": ego_head},
        warn_neighbors, gps_state)

    frame = {"ts": int(data_t_ms), "ego": ego_obj, "neighbors": neigh_list,
             "warning": warning}
    return frame, buzzer


# ── replay loop ─────────────────────────────────────────────────────────────
def replay(ego, events, obd, gps, srv, ser, *, fps, speed, ttl_s, loop, dry_run,
           fuel_start, fuel_drain, inject_brake):
    origin = ego["origin"]
    t0 = float(ego["t_ms"][0])
    t_end = float(ego["t_ms"][-1])
    span = max(t_end - t0, 1.0)
    emit_dt = 1.0 / fps                       # recording seconds per frame
    # Demo brake injection window (recording ms) — exercises EEBL from old logs.
    brake_lo = (t0 + inject_brake * 1000.0) if inject_brake is not None else None
    brake_hi = (brake_lo + 2500.0) if brake_lo is not None else None

    while True:
        neighbors: dict[int, dict] = {}
        ev_idx = 0
        last_buzzer = vw.BUZZER_OFF
        last_danger_sent = 0.0
        warn_cooldown: dict = {}    # node/type → last warning-beep sim-time (ms)
        sim_start = time.monotonic()
        sim_elapsed = 0.0

        while True:
            data_t_ms = t0 + sim_elapsed * 1000.0
            if data_t_ms > t_end:
                break

            # apply all neighbour receptions up to now
            while ev_idx < len(events) and events[ev_idx]["ts"] <= data_t_ms:
                e = events[ev_idx]
                neighbors[e["node"]] = {
                    "base_ts": e["ts"], "east": e["east"], "north": e["north"],
                    "speed_kmh": e["speed_kmh"], "heading_deg": e["heading_deg"],
                    "status": e["status"], "alert": e["alert"],
                }
                ev_idx += 1
            # expire stale neighbours
            for nid in [k for k, n in neighbors.items()
                        if (data_t_ms - n["base_ts"]) / 1000.0 > ttl_s]:
                del neighbors[nid]

            # demo: force every active neighbour to broadcast a hard brake
            if brake_lo is not None and brake_lo <= data_t_ms <= brake_hi:
                for n in neighbors.values():
                    n["alert"] = vw.ALERT_BRAKE
                    n["status"] = "EMERGENCY"

            fuel = max(0.0, fuel_start - fuel_drain * ((data_t_ms - t0) / span))
            frame, buzzer = build_sim_frame(
                data_t_ms, ego, obd, gps, neighbors, origin, fuel)
            if dry_run:
                sys.stdout.write(frame_to_line(frame).decode())
            else:
                srv.broadcast(frame)

            # drive a bench MCU buzzer over serial, if connected
            #   danger : re-arm every 2.5 s; warning/gps : per-node 5 s cooldown
            if ser is not None:
                now_ms = data_t_ms
                w = frame.get("warning")
                send = False
                if buzzer == vw.BUZZER_DANGER:
                    if last_buzzer != vw.BUZZER_DANGER or now_ms - last_danger_sent > 2500:
                        send = True
                        last_danger_sent = now_ms
                elif buzzer != vw.BUZZER_OFF and w is not None:
                    key = w.get("neighbor_id") or w.get("type")
                    if now_ms - warn_cooldown.get(key, -1e12) >= 5000:
                        send = True
                        warn_cooldown[key] = now_ms
                if send:
                    try:
                        send_buzzer(ser, buzzer)
                    except OSError:
                        pass
                last_buzzer = buzzer

            # pace to real time (real seconds = recording seconds / speed)
            sim_elapsed += emit_dt
            target = sim_start + sim_elapsed / speed
            delay = target - time.monotonic()
            if delay > 0:
                time.sleep(delay)

        if not loop:
            break
        log("[sim] loop — restarting replay")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", help="processed CSV (default: newest)")
    ap.add_argument("--raw", help="raw CSV with LORA_RX (default: sibling raw_*.csv)")
    ap.add_argument("--port", type=int, default=8765, help="TCP port (default 8765)")
    ap.add_argument("--host", default="0.0.0.0", help="bind address (default 0.0.0.0)")
    ap.add_argument("--fps", type=float, default=20.0, help="frames/s (default 20)")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed (default 1×)")
    ap.add_argument("--neighbor-ttl", type=float, default=8.0,
                    help="drop a neighbour after this many seconds of silence")
    ap.add_argument("--lora-clip", type=float, default=2000.0,
                    help="drop LoRa fixes farther than this (m) from ego; 0=keep all")
    ap.add_argument("--loop", action="store_true", help="repeat forever")
    ap.add_argument("--fuel-start", type=float, default=80.0,
                    help="synthetic ego fuel %% at start (CSV has no fuel data)")
    ap.add_argument("--fuel-drain", type=float, default=15.0,
                    help="synthetic fuel %% drained over the whole run")
    ap.add_argument("--dry-run", action="store_true",
                    help="print frames to stdout instead of serving TCP")
    ap.add_argument("--serial", metavar="PORT",
                    help="also drive a bench MCU buzzer over this serial port "
                         "(sends CMD_BUZZER on warning/danger)")
    ap.add_argument("--serial-baud", type=int, default=921600)
    ap.add_argument("--inject-brake", type=float, metavar="SEC",
                    help="demo: force neighbours to broadcast a hard brake for "
                         "2.5s starting SEC into playback (exercises EEBL)")
    args = ap.parse_args()

    folder = Path(__file__).resolve().parent
    path = Path(args.csv) if args.csv else find_latest_processed(folder)
    if path is None or not path.exists():
        raise SystemExit("No processed CSV found. Pass one explicitly.")
    raw = Path(args.raw) if args.raw else find_raw_for(path)

    log(f"[sim] ego : {path.name}")
    log(f"[sim] raw : {raw.name if raw and raw.exists() else '(none — ego only)'}")

    ego = load_ego(path)
    events = load_neighbor_events(raw, ego, args.lora_clip)
    obd = load_obd_ext(raw)
    gps = load_gps(raw)

    dur = (ego["t_ms"][-1] - ego["t_ms"][0]) / 1000.0
    nodes = sorted({e["node"] for e in events})
    log(f"[sim] {len(ego['t_ms']):,} ego samples over {dur:.1f}s | "
          f"{len(events)} LoRa receptions from nodes {nodes or '—'}")
    log(f"[sim] playback {args.speed}× @ {args.fps:g} fps"
          + (" (looping)" if args.loop else ""))

    srv = None
    if not args.dry_run:
        srv = JsonTcpServer(host=args.host, port=args.port)
        srv.start()
        log("[sim] waiting for the Flutter app to connect "
              f"(TcpDataSource → :{args.port}) … Ctrl-C to stop")

    ser = None
    if args.serial:
        import serial  # lazy — only needed for the bench-buzzer link
        ser = serial.Serial(args.serial, args.serial_baud, timeout=1)
        log(f"[sim] buzzer link on {args.serial} @ {args.serial_baud}")

    try:
        replay(ego, events, obd, gps, srv, ser, fps=args.fps, speed=args.speed,
               ttl_s=args.neighbor_ttl, loop=args.loop, dry_run=args.dry_run,
               fuel_start=args.fuel_start, fuel_drain=args.fuel_drain,
               inject_brake=args.inject_brake)
    except KeyboardInterrupt:
        log("\n[sim] stopped")
    finally:
        if srv:
            srv.stop()
        if ser is not None:
            try:
                send_buzzer(ser, vw.BUZZER_OFF)
                ser.close()
            except OSError:
                pass
    log("[sim] replay complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())
