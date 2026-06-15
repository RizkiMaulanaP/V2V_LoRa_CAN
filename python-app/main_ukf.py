#!/usr/bin/env python3
"""
V2V host — UKF variant.

Differences vs main.py:
  • Ego fusion uses a 4-state UKF (e, n, v, ψ) — now shared via v2v_fusion.py.
  • Pushes the fused pose back to the MCU via CMD_UPDATE_STATE, so the LoRa
    broadcast carries filtered (lat, lon, heading, speed) instead of raw GPS.
  • Each LoRa-RX neighbour is tracked by a small CTRV Kalman filter, so the
    plotted position / range is dead-reckoned between the (~2 s) LoRa updates.
  • Parses FRAME_OBD_EXT (engine RPM, coolant °C) — pure telemetry, not fused.
  • Optional --json-tcp PORT streams the fused state to the Flutter app as
    newline-delimited JSON (same path BeamNG uses via main_beamng.py).

Requires:
    pip install pyserial numpy scipy matplotlib filterpy
"""

import csv
import sys
import time
import struct
import argparse
from datetime import datetime
from functools import reduce
from pathlib import Path
from typing import Optional

import numpy as np
import serial
from serial.tools import list_ports
import matplotlib.pyplot as plt

# Shared sensor-fusion core (numpy + filterpy only).
from v2v_fusion import (
    latlon_to_enu, enu_to_latlon, haversine, wrap_pi,
    math_rad_to_compass_deg, compass_deg_to_math_rad,
    EgoUKF, NeighborRegistry,
)
# JSON → Flutter bridge (stdlib only).
from v2v_json import JsonTcpServer, build_frame
# Shared collision-warning engine (also used by simulate_v2v.py).
import v2v_warnings as vw

# ═══════════════════════════════════════════════════════════════════════════════
# PROTOCOL (matches V2V_LoRa_TWAI.ino / V2V_LoRa_CAN.ino at FW_VERSION ≥ 2)
# ═══════════════════════════════════════════════════════════════════════════════
FRAME_START = 0xAA
FRAME_END   = 0x55

FRAME_IMU     = 0x01
FRAME_GPS     = 0x02
FRAME_OBD     = 0x03
FRAME_LORA_RX = 0x04
FRAME_HELLO   = 0x05
FRAME_OBD_EXT = 0x06

FRAME_SIZES = {
    FRAME_IMU:     32,
    FRAME_GPS:     34,
    FRAME_OBD:     9,
    FRAME_LORA_RX: 30,
    FRAME_HELLO:   10,
    FRAME_OBD_EXT: 12,
}

FRAME_STRUCTS = {
    FRAME_IMU:     struct.Struct("<BIffffffBB"),
    FRAME_GPS:     struct.Struct("<BIddffBBBB"),
    FRAME_OBD:     struct.Struct("<BIBBB"),
    FRAME_LORA_RX: struct.Struct("<BIIBfffBBhbBB"),
    FRAME_HELLO:   struct.Struct("<BIBBBB"),
    FRAME_OBD_EXT: struct.Struct("<BIHhBB"),
}

CMD_START        = 0xBB
CMD_BROADCAST    = 0x01
CMD_UPDATE_STATE = 0x02
CMD_HELLO        = 0x03
CMD_BUZZER       = 0x04   # pattern id carried in the alert_type byte

HEARTBEAT_S      = 1.0
HANDSHAKE_TRIES  = 10
STATE_PUSH_HZ    = 5.0   # CMD_UPDATE_STATE cadence
JSON_PUSH_HZ     = 30.0  # Flutter JSON broadcast cadence
WARN_COOLDOWN_S  = 5.0   # per-node re-beep cooldown for warning level
DANGER_REARM_S   = 2.5   # danger keeps re-beeping at this cadence while active

ALERT_LABELS = {0: "Normal", 1: "Traffic Jam", 2: "Hard Brake"}
# Reverse map (track stores the label string) → numeric alert for the engine.
_LABEL_TO_ALERT = {"Normal": vw.ALERT_NORMAL, "Traffic Jam": vw.ALERT_TRAFFIC_JAM,
                   "Road Damage": vw.ALERT_TRAFFIC_JAM, "Hard Brake": vw.ALERT_BRAKE}

# IMU axis convention — matches v2v_fusion / tested config:
#   forward acceleration = ax,  yaw rate = gx
IMU_FWD_ACCEL = "ax"
IMU_YAW_RATE  = "gx"

# ═══════════════════════════════════════════════════════════════════════════════
# DATA LOGGER
# ═══════════════════════════════════════════════════════════════════════════════
class DataLogger:
    _RAW_FIELDS = [
        "wall_time", "frame_type", "mcu_ts_ms",
        "ax_ms2", "ay_ms2", "az_ms2", "gx_rads", "gy_rads", "gz_rads",
        "gps_lat", "gps_lon", "gps_speed_kmh", "hdop", "satellites", "fix_valid",
        "obd_speed_kmh", "engine_rpm", "coolant_c",
        "lora_node", "lora_lat", "lora_lon", "lora_heading_deg",
        "lora_speed_kmh", "lora_alert", "lora_rssi_dbm", "lora_snr_db",
        "lora_remote_tx_ts_ms",
    ]
    _PROC_FIELDS = [
        "wall_time", "mcu_ts_ms",
        "lat", "lon", "pos_east_m", "pos_north_m",
        "speed_ms", "speed_kmh", "heading_deg",
    ]

    def __init__(self, outdir: Path):
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
        rpath = outdir / f"raw_{ts}.csv"
        ppath = outdir / f"processed_{ts}.csv"
        self._rf = open(rpath, "w", newline="")
        self._pf = open(ppath, "w", newline="")
        self._rw = csv.DictWriter(self._rf, fieldnames=self._RAW_FIELDS)
        self._pw = csv.DictWriter(self._pf, fieldnames=self._PROC_FIELDS)
        self._rw.writeheader()
        self._pw.writeheader()
        print(f"[LOG] raw       → {rpath}")
        print(f"[LOG] processed → {ppath}")

    def log_raw(self, frame: dict):
        row = {f: "" for f in self._RAW_FIELDS}
        row["wall_time"]  = datetime.now().isoformat(timespec="milliseconds")
        row["frame_type"] = frame["type"]
        row["mcu_ts_ms"]  = frame.get("ts", "")
        t = frame["type"]
        if t == "IMU":
            row.update({"ax_ms2": frame["ax"], "ay_ms2": frame["ay"], "az_ms2": frame["az"],
                        "gx_rads": frame["gx"], "gy_rads": frame["gy"], "gz_rads": frame["gz"]})
        elif t == "GPS":
            row.update({"gps_lat": frame["lat"], "gps_lon": frame["lon"],
                        "gps_speed_kmh": frame["speed_kmh"], "hdop": frame["hdop"],
                        "satellites": frame["satellites"], "fix_valid": int(frame["fix"])})
        elif t == "OBD":
            row["obd_speed_kmh"] = frame["speed_kmh"]
        elif t == "OBD_EXT":
            row["engine_rpm"] = frame["engine_rpm"]
            row["coolant_c"]  = frame["coolant_c"]
        elif t == "LORA_RX":
            row.update({"lora_node": frame["node_id"],
                        "lora_lat": frame["lat"], "lora_lon": frame["lon"],
                        "lora_heading_deg": frame["heading_deg"],
                        "lora_speed_kmh": frame["speed_kmh"],
                        "lora_alert": frame["alert"],
                        "lora_rssi_dbm": frame["rssi"],
                        "lora_snr_db": frame["snr"],
                        "lora_remote_tx_ts_ms": frame["remote_tx_ts"]})
        self._rw.writerow(row)

    def log_processed(self, ts_ms, ukf: EgoUKF):
        lat, lon = ukf.latlon
        if lat is None:
            return
        self._pw.writerow({
            "wall_time":   datetime.now().isoformat(timespec="milliseconds"),
            "mcu_ts_ms":   ts_ms,
            "lat":         f"{lat:.8f}",
            "lon":         f"{lon:.8f}",
            "pos_east_m":  f"{ukf.east:.3f}",
            "pos_north_m": f"{ukf.north:.3f}",
            "speed_ms":    f"{ukf.speed_ms:.4f}",
            "speed_kmh":   f"{ukf.speed_kmh:.2f}",
            "heading_deg": f"{ukf.heading_deg:.2f}",
        })

    def flush(self): self._rf.flush(); self._pf.flush()
    def close(self): self._rf.close(); self._pf.close()

# ═══════════════════════════════════════════════════════════════════════════════
# LIVE PLOTTER
# ═══════════════════════════════════════════════════════════════════════════════
class LivePlotter:
    _TRAIL  = 300
    _REDRAW = 20

    def __init__(self):
        plt.ion()
        self._fig, (self._ax_map, self._ax_spd) = plt.subplots(
            1, 2, figsize=(14, 6))
        self._fig.suptitle("V2V UKF Monitor", fontsize=11)
        self._trail_x: list[float] = []
        self._trail_y: list[float] = []
        self._spd_hist: list[float] = []
        self._count = 0

    def update(self, ukf: EgoUKF, neighbors: NeighborRegistry):
        if ukf.origin_lat is None:
            return
        self._count += 1
        self._trail_x.append(ukf.east)
        self._trail_y.append(ukf.north)
        self._spd_hist.append(ukf.speed_kmh)
        if len(self._trail_x) > self._TRAIL:
            self._trail_x  = self._trail_x[-self._TRAIL:]
            self._trail_y  = self._trail_y[-self._TRAIL:]
            self._spd_hist = self._spd_hist[-self._TRAIL:]
        if self._count % self._REDRAW == 0:
            self._redraw(ukf, neighbors)

    def _redraw(self, ukf: EgoUKF, neighbors: NeighborRegistry):
        ax = self._ax_map
        ax.cla()
        ax.set_title("Vehicle Positions (ENU)")
        ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3)

        if len(self._trail_x) > 1:
            ax.plot(self._trail_x, self._trail_y,
                    "b-", lw=0.8, alpha=0.4, label="My path")
        ax.plot(ukf.east, ukf.north, "bo", ms=10, zorder=5, label="Me")

        alen = max(3.0, ukf.speed_ms * 0.5)
        ax.annotate("", xy=(ukf.east  + alen*np.cos(ukf.yaw_rad),
                            ukf.north + alen*np.sin(ukf.yaw_rad)),
                    xytext=(ukf.east, ukf.north),
                    arrowprops=dict(arrowstyle="->", color="blue", lw=1.5))

        for nid, trk in neighbors.tracks.items():
            ax.plot(trk.east, trk.north, "r^", ms=9, zorder=5,
                    label=f"Node {nid}")
            ax.plot([ukf.east, trk.east], [ukf.north, trk.north],
                    "r--", lw=0.8, alpha=0.5)
            d = np.hypot(ukf.east - trk.east, ukf.north - trk.north)
            ax.text((ukf.east + trk.east)/2, (ukf.north + trk.north)/2,
                    f"{d:.1f} m", fontsize=7, ha="center", va="bottom",
                    color="darkred")
            psi    = float(trk.x[3])
            alen_n = max(2.0, trk.x[2] * 0.5)
            ax.annotate("", xy=(trk.east  + alen_n*np.cos(psi),
                                trk.north + alen_n*np.sin(psi)),
                        xytext=(trk.east, trk.north),
                        arrowprops=dict(arrowstyle="->", color="red", lw=1.0))
        ax.legend(fontsize=7, loc="upper left")

        ax2 = self._ax_spd
        ax2.cla()
        ax2.set_xlabel("Sample"); ax2.set_ylabel("Speed (km/h)")
        ax2.grid(True, alpha=0.3)
        if self._spd_hist:
            xs = list(range(len(self._spd_hist)))
            ax2.plot(xs, self._spd_hist, "k-", lw=0.8)
            ax2.set_title(
                f"Speed: {self._spd_hist[-1]:.1f} km/h  │  heading "
                f"{ukf.heading_deg:.0f}°", fontsize=10)

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()

# ═══════════════════════════════════════════════════════════════════════════════
# FRAME PARSER
# ═══════════════════════════════════════════════════════════════════════════════
def xor_checksum(data: bytes) -> int:
    return reduce(lambda a, b: a ^ b, data, 0)

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

    expected = FRAME_SIZES[frame_type] - 2
    rest = ser.read(expected)
    if len(rest) < expected:
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
        (_, ts, remote_tx, nid, lat, lon, heading,
         spd, alert, rssi, snr, _, _) = fields
        return {"type": "LORA_RX", "ts": ts,
                "remote_tx_ts": remote_tx,
                "node_id": nid, "lat": lat, "lon": lon,
                "heading_deg": heading, "speed_kmh": spd,
                "alert": ALERT_LABELS.get(alert, f"Unknown({alert})"),
                "rssi": rssi, "snr": snr}
    if frame_type == FRAME_HELLO:
        _, ts, node_id, fw_version, _, _ = fields
        return {"type": "HELLO", "ts": ts,
                "node_id": node_id, "fw_version": fw_version}
    if frame_type == FRAME_OBD_EXT:
        _, ts, rpm, coolant, _, _ = fields
        return {"type": "OBD_EXT", "ts": ts,
                "engine_rpm": rpm, "coolant_c": coolant}
    return None

# ═══════════════════════════════════════════════════════════════════════════════
# HOST → MCU COMMANDS
# CmdFrame: start cmd_type alert_type lat(4) lon(4) heading(4) speed cksum end
# ═══════════════════════════════════════════════════════════════════════════════
_CMD_STRUCT = struct.Struct("<BBBfffBBB")

def send_cmd(ser, cmd_type, alert_type=0, lat=0.0, lon=0.0,
             heading_deg=0.0, speed_kmh=0):
    body = struct.pack("<BBfffB", cmd_type, alert_type,
                       float(lat), float(lon), float(heading_deg),
                       int(speed_kmh) & 0xFF)
    cksum = reduce(lambda a, b: a ^ b, body, 0)
    pkt = _CMD_STRUCT.pack(CMD_START, cmd_type, alert_type,
                           float(lat), float(lon), float(heading_deg),
                           int(speed_kmh) & 0xFF, cksum, FRAME_END)
    ser.write(pkt)

def handshake(ser) -> Optional[dict]:
    print(f"[handshake] sending CMD_HELLO (timeout {HANDSHAKE_TRIES}×1s) ...")
    for attempt in range(1, HANDSHAKE_TRIES + 1):
        send_cmd(ser, CMD_HELLO)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            frame = read_frame(ser)
            if frame is None:
                continue
            if frame["type"] == "HELLO":
                return frame
            print("[handshake] firmware was already streaming; treating as connected")
            return frame
        print(f"[handshake] no reply (attempt {attempt}/{HANDSHAKE_TRIES})")
    return None

# ═══════════════════════════════════════════════════════════════════════════════
# SERIAL PORT DISCOVERY  (cross-platform: Windows COMx / Linux ttyACM / macOS)
# ═══════════════════════════════════════════════════════════════════════════════
def ports_help() -> str:
    ports = list(list_ports.comports())
    if not ports:
        return "  (no serial ports detected)"
    lines = []
    for p in ports:
        vid = f"{p.vid:#06x}" if p.vid is not None else "----"
        pid = f"{p.pid:#06x}" if p.pid is not None else "----"
        lines.append(f"  {p.device:14} {p.description}  [VID:PID={vid}:{pid}]")
    return "\n".join(lines)


def find_serial_port() -> Optional[str]:
    """Best-guess the MCU's USB-CDC port on any OS.

    The Heltec ESP32-S3 enumerates as Espressif native USB (VID 0x303A); some
    boards use a CP210x/CH34x/FTDI bridge instead. Returns COMx on Windows,
    /dev/ttyACM* or /dev/cu.* elsewhere.
    """
    ports = list(list_ports.comports())
    KNOWN_VIDS = {0x303A, 0x10C4, 0x1A86, 0x0403}  # Espressif, CP210x, CH34x, FTDI
    for p in ports:                       # 1) match by known USB vendor
        if p.vid in KNOWN_VIDS:
            return p.device
    for p in ports:                       # 2) match by device-name pattern
        d = p.device.lower()
        if "ttyacm" in d or "ttyusb" in d or "cu.usb" in d or d.startswith("com"):
            return p.device
    return ports[0].device if len(ports) == 1 else None  # 3) lone port


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    p = argparse.ArgumentParser(description="V2V MCU reader + UKF fusion")
    p.add_argument("--port",   default="auto",
                   help="serial port (e.g. COM5 on Windows, /dev/ttyACM0 on "
                        "Linux). Default 'auto' detects the MCU's USB port.")
    p.add_argument("--baud",   type=int, default=921600)
    p.add_argument("--outdir", default=".")
    p.add_argument("--no-push", action="store_true",
                   help="Don't push fused state back to MCU "
                        "(firmware will keep broadcasting raw GPS)")
    p.add_argument("--json-tcp", type=int, metavar="PORT", default=None,
                   help="Stream fused state to the Flutter app as "
                        "newline-JSON on this TCP port (e.g. 8765)")
    args = p.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    port = args.port
    if port == "auto":
        port = find_serial_port()
        if port is None:
            print("Error: could not auto-detect a serial port. "
                  "Pass one with --port (e.g. --port COM5).\n"
                  "Available ports:\n" + ports_help(), file=sys.stderr)
            sys.exit(1)
        print(f"[auto] selected serial port: {port}")

    print(f"Opening {port} @ {args.baud} baud ...")
    try:
        ser = serial.Serial(port, args.baud, timeout=2)
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}\nAvailable ports:\n" + ports_help(),
              file=sys.stderr)
        sys.exit(1)

    ukf       = EgoUKF()
    neighbors = NeighborRegistry()
    logger    = DataLogger(outdir)
    plotter   = LivePlotter()

    json_srv = None
    if args.json_tcp is not None:
        json_srv = JsonTcpServer(port=args.json_tcp)
        json_srv.start()

    hello = handshake(ser)
    if hello is None:
        print("[error] Firmware did not respond to CMD_HELLO — aborting.",
              file=sys.stderr)
        ser.close(); sys.exit(1)
    if hello.get("type") == "HELLO":
        print(f"[handshake] connected: node={hello['node_id']} "
              f"fw={hello['fw_version']} mcu_ts={hello['ts']}ms")

    last_heartbeat   = time.monotonic()
    last_state_push  = 0.0
    state_push_dt    = 1.0 / STATE_PUSH_HZ
    last_json_push   = 0.0
    json_push_dt     = 1.0 / JSON_PUSH_HZ
    proc_count       = 0
    last_alert_type  = 0
    last_rpm         = 0
    last_coolant     = 0
    # Ego GPS-health (for the GPS-lost warning) — no fix yet at startup.
    last_gps         = {"fix_valid": 0, "hdop": 99.0, "satellites": 0}
    last_buzzer      = vw.BUZZER_OFF
    last_danger_push = 0.0
    warn_cooldown    = {}    # node/type key → last warning-beep time (cooldown)

    print("Listening — Ctrl+C to stop\n")
    try:
        while True:
            now_m = time.monotonic()
            if now_m - last_heartbeat >= HEARTBEAT_S:
                send_cmd(ser, CMD_HELLO)
                last_heartbeat = now_m

            frame = read_frame(ser)
            if frame is None:
                continue
            if frame["type"] == "HELLO":
                continue

            logger.log_raw(frame)
            t = frame["type"]

            # ── IMU → predict ────────────────────────────────────────────────
            if t == "IMU":
                ax_fwd = frame[IMU_FWD_ACCEL]
                gz_yaw = frame[IMU_YAW_RATE]
                ukf.predict_imu(ax_fwd, gz_yaw, frame["ts"])
                if ukf.has_gps and ukf.has_obd:
                    proc_count += 1
                    logger.log_processed(frame["ts"], ukf)
                    plotter.update(ukf, neighbors)
                    if proc_count % 10 == 0:
                        lat, lon = ukf.latlon
                        lat_s = f"{lat:.6f}" if lat else "---"
                        lon_s = f"{lon:.6f}" if lon else "---"
                        print(f"[UKF] ({lat_s}, {lon_s})  "
                              f"spd={ukf.speed_kmh:5.1f} km/h  "
                              f"hdg={ukf.heading_deg:5.1f}°")

            # ── GPS → update ─────────────────────────────────────────────────
            elif t == "GPS":
                ukf.update_gps(frame["lat"], frame["lon"], frame["fix"])
                last_gps = {"fix_valid": 1 if frame["fix"] else 0,
                            "hdop": float(frame["hdop"]),
                            "satellites": int(frame["satellites"])}
                fix_s = "FIX   " if frame["fix"] else "NO FIX"
                print(f"[GPS]  {fix_s}  "
                      f"({frame['lat']:.6f}, {frame['lon']:.6f})  "
                      f"spd={frame['speed_kmh']:.1f} km/h  "
                      f"hdop={frame['hdop']:.1f}  sats={frame['satellites']}")

            # ── OBD speed → update ───────────────────────────────────────────
            elif t == "OBD":
                ukf.update_obd(frame["speed_kmh"])
                print(f"[OBD]  speed={frame['speed_kmh']} km/h")

            # ── OBD ext (RPM, coolant) → telemetry only ──────────────────────
            elif t == "OBD_EXT":
                last_rpm     = frame["engine_rpm"]
                last_coolant = frame["coolant_c"]
                print(f"[OBDx] rpm={frame['engine_rpm']:5d}  "
                      f"coolant={frame['coolant_c']:+4d}°C")

            # ── LoRa RX → neighbour filter ───────────────────────────────────
            elif t == "LORA_RX":
                if ukf.origin_lat is not None and frame["lat"] != 0.0:
                    neighbors.update(
                        frame["node_id"], frame["lat"], frame["lon"],
                        frame["speed_kmh"], frame["heading_deg"],
                        ukf.origin_lat, ukf.origin_lon, frame["ts"],
                        frame["rssi"], frame["snr"], frame["alert"],
                    )
                    trk    = neighbors.tracks[frame["node_id"]]
                    my_lat, my_lon = ukf.latlon
                    if my_lat is not None:
                        d = haversine(my_lat, my_lon, frame["lat"], frame["lon"])
                        dist_s = f"  range={d:.1f} m"
                    else:
                        dist_s = ""
                    print(f"[LORA] node={frame['node_id']}  "
                          f"spd={frame['speed_kmh']} km/h  "
                          f"hdg={frame['heading_deg']:5.1f}°  "
                          f"alert={frame['alert']}  "
                          f"rssi={frame['rssi']} dBm  snr={frame['snr']} dB"
                          f"{dist_s}")

            # ── Push fused state back to MCU ─────────────────────────────────
            if (not args.no_push and ukf.has_gps and ukf.has_obd
                    and now_m - last_state_push >= state_push_dt):
                lat, lon = ukf.latlon
                if lat is not None:
                    send_cmd(ser, CMD_UPDATE_STATE,
                             alert_type=last_alert_type,
                             lat=lat, lon=lon,
                             heading_deg=ukf.heading_deg,
                             speed_kmh=int(round(ukf.speed_kmh)))
                    last_state_push = now_m

            # ── Collision warning + Flutter stream + buzzer ──────────────────
            if (ukf.has_gps and ukf.has_obd
                    and now_m - last_json_push >= json_push_dt):
                ego_kin = {"east": ukf.east, "north": ukf.north,
                           "speed_kmh": ukf.speed_kmh,
                           "heading_deg": ukf.heading_deg}
                nbrs = [{"id": f"N{nid}", "east": trk.east, "north": trk.north,
                         "speed_kmh": trk.speed_kmh, "heading_deg": trk.heading_deg,
                         "alert": _LABEL_TO_ALERT.get(trk.last_alert, 0)}
                        for nid, trk in neighbors.tracks.items()]
                warning, buzzer = vw.assess(ego_kin, nbrs, last_gps)

                if json_srv is not None:
                    fr = build_frame(frame["ts"], ukf, neighbors,
                                     engine_rpm=last_rpm, engine_temp_c=last_coolant,
                                     gps=last_gps, warning=warning)
                    json_srv.broadcast(fr)

                # Drive the MCU buzzer on GP4.
                #   danger : beep on entry, then re-arm every DANGER_REARM_S
                #   warning/gps : per-node 5 s cooldown so it doesn't nag
                if not args.no_push:
                    if buzzer == vw.BUZZER_DANGER:
                        if (last_buzzer != vw.BUZZER_DANGER
                                or now_m - last_danger_push >= DANGER_REARM_S):
                            send_cmd(ser, CMD_BUZZER, alert_type=buzzer)
                            last_danger_push = now_m
                    elif buzzer != vw.BUZZER_OFF:   # warning level (warning / gps)
                        key = warning.get("neighbor_id") or warning.get("type")
                        if now_m - warn_cooldown.get(key, -1e9) >= WARN_COOLDOWN_S:
                            send_cmd(ser, CMD_BUZZER, alert_type=buzzer)
                            warn_cooldown[key] = now_m
                    last_buzzer = buzzer

                last_json_push = now_m

            # ── Predict-only update for neighbours (smooths display) ─────────
            if t == "IMU":
                neighbors.predict_all(frame["ts"])
                gone = neighbors.prune_stale(frame["ts"])
                for nid in gone:
                    print(f"[LORA] node={nid} dropped (stale)")

            if proc_count and proc_count % 100 == 0:
                logger.flush()

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        logger.close()
        ser.close()
        if json_srv is not None:
            json_srv.stop()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
