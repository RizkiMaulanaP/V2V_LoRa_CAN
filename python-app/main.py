#!/usr/bin/env python3

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
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

# ═══════════════════════════════════════════════════════════════════════════════
# PROTOCOL CONSTANTS  (must match V2V_LoRa_CAN.ino)
# ═══════════════════════════════════════════════════════════════════════════════
FRAME_START = 0xAA
FRAME_END   = 0x55

FRAME_IMU     = 0x01
FRAME_GPS     = 0x02
FRAME_OBD     = 0x03
FRAME_LORA_RX = 0x04
FRAME_HELLO   = 0x05

FRAME_SIZES = {
    FRAME_IMU:     32,
    FRAME_GPS:     34,
    FRAME_OBD:     9,
    FRAME_LORA_RX: 22,
    FRAME_HELLO:   10,
}

FRAME_STRUCTS = {
    FRAME_IMU:     struct.Struct("<BIffffffBB"),
    FRAME_GPS:     struct.Struct("<BIddffBBBB"),
    FRAME_OBD:     struct.Struct("<BIBBB"),
    FRAME_LORA_RX: struct.Struct("<BIBffBBhbBB"),
    FRAME_HELLO:   struct.Struct("<BIBBBB"),
}

# Commands sent host → MCU
CMD_START        = 0xBB
CMD_BROADCAST    = 0x01
CMD_UPDATE_STATE = 0x02
CMD_HELLO        = 0x03

HEARTBEAT_S      = 1.0    # send CMD_HELLO this often to keep firmware streaming
HANDSHAKE_TRIES  = 10     # CMD_HELLO retries during initial handshake

ALERT_LABELS = {0: "Normal", 1: "Traffic Jam", 2: "Road Damage"}

# ═══════════════════════════════════════════════════════════════════════════════
# MATH UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════
def skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ],
    ])

def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product [w,x,y,z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def omega_to_dq(w: np.ndarray, dt: float) -> np.ndarray:
    """Angular velocity [rad/s] → delta quaternion over dt."""
    angle = np.linalg.norm(w) * dt
    if angle < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = w / np.linalg.norm(w)
    return np.array([np.cos(angle / 2), *(np.sin(angle / 2) * axis)])

def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """[w,x,y,z] → 3×3 rotation matrix."""
    return Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()

def latlon_to_enu(lat: float, lon: float,
                  lat0: float, lon0: float) -> np.ndarray:
    """Lat/lon → local ENU metres relative to origin."""
    R = 6_371_000.0
    x = np.radians(lon - lon0) * R * np.cos(np.radians(lat0))
    y = np.radians(lat - lat0) * R
    return np.array([x, y, 0.0])

def enu_to_latlon(enu: np.ndarray,
                  lat0: float, lon0: float) -> tuple[float, float]:
    R = 6_371_000.0
    lat = lat0 + np.degrees(enu[1] / R)
    lon = lon0 + np.degrees(enu[0] / (R * np.cos(np.radians(lat0))))
    return float(lat), float(lon)

def haversine(lat1: float, lon1: float,
              lat2: float, lon2: float) -> float:
    """Great-circle distance in metres."""
    R = 6_371_000.0
    phi1, phi2 = np.radians(lat1), np.radians(lat2)
    dphi    = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)
    a = np.sin(dphi / 2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2)**2
    return 2 * R * np.arcsin(np.sqrt(a))

# ═══════════════════════════════════════════════════════════════════════════════
# ERROR-STATE KALMAN FILTER  (15-state)
#
# Nominal state : p(3)  v(3)  q(4)  ba(3)  bg(3)
# Error state   : δp(3) δv(3) δθ(3) δba(3) δbg(3)
#
# Predict  → IMU at ~100 Hz
# Update 1 → GPS position  (~1 Hz)
# Update 2 → OBD speed     (~2 Hz)
# ═══════════════════════════════════════════════════════════════════════════════
class ESKF:
    G_ENU = np.array([0.0, 0.0, -9.80665])   # gravity in ENU (z-up frame)

    def __init__(self):
        # ── Nominal state ────────────────────────────────────────────────────
        self.p  = np.zeros(3)
        self.v  = np.zeros(3)
        self.q  = np.array([1., 0., 0., 0.])  # [w,x,y,z]
        self.ba = np.zeros(3)                  # accel bias  (m/s²)
        self.bg = np.zeros(3)                  # gyro  bias  (rad/s)

        # ── Error covariance (15×15) ─────────────────────────────────────────
        self.P = np.diag([
            *([1.0 ] * 3),   # δp
            *([1.0 ] * 3),   # δv
            *([0.1 ] * 3),   # δθ
            *([0.01] * 3),   # δba
            *([0.001]*3),    # δbg
        ])

        # ── IMU process noise densities ──────────────────────────────────────
        self._sig_a  = 0.05    # accel noise  (m/s²)
        self._sig_g  = 0.005   # gyro  noise  (rad/s)
        self._sig_ba = 1e-4    # accel bias random walk
        self._sig_bg = 1e-5    # gyro  bias random walk

        # ── Measurement noise ────────────────────────────────────────────────
        self._R_gps = np.diag([5.0**2, 5.0**2, 10.0**2])  # GPS pos (m²)
        self._R_obd = np.array([[0.5**2]])                 # OBD |v| (m/s)²

        # ── State ────────────────────────────────────────────────────────────
        self.initialized  = False
        self.origin_lat   = None
        self.origin_lon   = None
        self._last_imu_ts = None
        self.origin_set   = False    # one-shot flag for main loop print

    # ── Initialisation ───────────────────────────────────────────────────────
    def init_orientation(self, a_avg: np.ndarray):
        """Roll/pitch from averaged gravity; yaw=0."""
        n = np.linalg.norm(a_avg)
        if n < 1.0:
            return
        a = a_avg / n
        roll  = np.arctan2( a[1],  a[2])
        pitch = np.arctan2(-a[0], np.sqrt(a[1]**2 + a[2]**2))
        q_sci = Rotation.from_euler("xyz", [roll, pitch, 0.0]).as_quat()
        self.q = np.array([q_sci[3], q_sci[0], q_sci[1], q_sci[2]])
        self.initialized = True

    def _set_origin(self, lat: float, lon: float):
        self.origin_lat = lat
        self.origin_lon = lon
        self.p[:] = 0.0
        self.origin_set = True

    # ── Predict (IMU) ────────────────────────────────────────────────────────
    def predict(self, a_meas: np.ndarray, w_meas: np.ndarray, ts_ms: int):
        if not self.initialized:
            return
        if self._last_imu_ts is None:
            self._last_imu_ts = ts_ms
            return
        dt = (ts_ms - self._last_imu_ts) * 1e-3
        self._last_imu_ts = ts_ms
        if dt <= 0 or dt > 0.5:
            return

        R = quat_to_rot(self.q)
        a_corr = a_meas - self.ba
        w_corr = w_meas - self.bg

        # World-frame acceleration (specific force rotated to ENU + gravity)
        a_world = R @ a_corr + self.G_ENU

        # Integrate position and velocity
        self.p += self.v * dt + 0.5 * a_world * dt**2
        self.v += a_world * dt

        # Integrate attitude
        self.q = quat_mult(self.q, omega_to_dq(w_corr, dt))
        self.q /= np.linalg.norm(self.q)

        # ── Error-state transition matrix F (15×15) ──────────────────────────
        F = np.eye(15)
        F[0:3,  3:6]  =  np.eye(3) * dt
        F[3:6,  6:9]  = -R @ skew(a_corr) * dt
        F[3:6,  9:12] = -R * dt
        F[6:9,  6:9]  =  np.eye(3) - skew(w_corr) * dt
        F[6:9, 12:15] = -np.eye(3) * dt

        # ── Discrete process noise Q (15×15) ─────────────────────────────────
        Q = np.zeros((15, 15))
        Q[3:6,   3:6]  = np.eye(3) * (self._sig_a  * dt)**2
        Q[6:9,   6:9]  = np.eye(3) * (self._sig_g  * dt)**2
        Q[9:12,  9:12] = np.eye(3) * (self._sig_ba * dt)**2
        Q[12:15,12:15] = np.eye(3) * (self._sig_bg * dt)**2

        self.P = F @ self.P @ F.T + Q

    # ── Update: GPS position ─────────────────────────────────────────────────
    def update_gps(self, lat: float, lon: float, fix_valid: bool):
        if not self.initialized or not fix_valid:
            return
        if self.origin_lat is None:
            self._set_origin(lat, lon)
            return                           # first fix sets origin; no innovation

        p_gps = latlon_to_enu(lat, lon, self.origin_lat, self.origin_lon)
        H = np.zeros((3, 15))
        H[0:3, 0:3] = np.eye(3)

        S = H @ self.P @ H.T + self._R_gps
        K = self.P @ H.T @ np.linalg.inv(S)
        self._inject(K @ (p_gps - self.p))
        self.P = (np.eye(15) - K @ H) @ self.P

    # ── Update: OBD speed ────────────────────────────────────────────────────
    def update_obd(self, speed_kmh: float):
        if not self.initialized:
            return
        speed_ms = speed_kmh / 3.6
        v_norm   = np.linalg.norm(self.v)

        H = np.zeros((1, 15))
        H[0, 3:6] = self.v / v_norm if v_norm > 0.1 else np.array([1., 0., 0.])

        S = (H @ self.P @ H.T + self._R_obd)[0, 0]
        K = (self.P @ H.T).flatten() / S
        self._inject(K * (speed_ms - v_norm))
        self.P = (np.eye(15) - np.outer(K, H)) @ self.P

    # ── Error injection ──────────────────────────────────────────────────────
    def _inject(self, dx: np.ndarray):
        self.p  += dx[0:3]
        self.v  += dx[3:6]
        dq = np.array([1.0, *(dx[6:9] / 2)])
        self.q   = quat_mult(self.q, dq)
        self.q  /= np.linalg.norm(self.q)
        self.ba += dx[9:12]
        self.bg += dx[12:15]

    # ── Derived quantities ───────────────────────────────────────────────────
    @property
    def euler_deg(self) -> np.ndarray:
        """Roll, pitch, yaw in degrees."""
        r = Rotation.from_quat([self.q[1], self.q[2], self.q[3], self.q[0]])
        return np.degrees(r.as_euler("xyz"))

    @property
    def speed_ms(self) -> float:
        return float(np.linalg.norm(self.v))

    @property
    def latlon(self) -> tuple[Optional[float], Optional[float]]:
        if self.origin_lat is None:
            return None, None
        return enu_to_latlon(self.p, self.origin_lat, self.origin_lon)

    def world_accel(self, a_meas: np.ndarray) -> np.ndarray:
        """Inertial acceleration in ENU (gravity removed)."""
        return quat_to_rot(self.q) @ (a_meas - self.ba) + self.G_ENU

    def body_accel(self, a_meas: np.ndarray) -> np.ndarray:
        """Inertial acceleration in body frame (gravity removed).
        [0]=forward  [1]=left  [2]=up"""
        return quat_to_rot(self.q).T @ self.world_accel(a_meas)

# ═══════════════════════════════════════════════════════════════════════════════
# CONDITION DETECTOR
# ═══════════════════════════════════════════════════════════════════════════════
CONDITION_COLORS = {
    "CRASH":       "red",
    "ROLLOVER":    "darkred",
    "HARD_BRAKE":  "orange",
    "HARD_ACCEL":  "yellow",
    "HARD_CORNER": "gold",
    "HIGH_SPEED":  "cyan",
    "STATIONARY":  "gray",
    "NORMAL":      "limegreen",
}

def detect_condition(a_body: np.ndarray,
                     roll_deg: float,
                     speed_ms: float) -> str:
    if np.linalg.norm(a_body) > 30.0:   return "CRASH"
    if abs(roll_deg) > 60.0:            return "ROLLOVER"
    if a_body[0]         < -5.0:        return "HARD_BRAKE"
    if a_body[0]         >  4.0:        return "HARD_ACCEL"
    if abs(a_body[1])    >  4.0:        return "HARD_CORNER"
    if speed_ms          > 22.2:        return "HIGH_SPEED"
    if speed_ms          <  0.5:        return "STATIONARY"
    return "NORMAL"

# ═══════════════════════════════════════════════════════════════════════════════
# DATA LOGGER
# ═══════════════════════════════════════════════════════════════════════════════
class DataLogger:
    _RAW_FIELDS = [
        "wall_time", "frame_type", "mcu_ts_ms",
        # IMU
        "ax_ms2", "ay_ms2", "az_ms2", "gx_rads", "gy_rads", "gz_rads",
        # GPS
        "gps_lat", "gps_lon", "gps_speed_kmh", "hdop", "satellites", "fix_valid",
        # OBD
        "obd_speed_kmh",
        # LoRa RX
        "lora_node", "lora_lat", "lora_lon", "lora_speed_kmh",
        "lora_alert", "lora_rssi_dbm", "lora_snr_db",
    ]
    _PROC_FIELDS = [
        "wall_time", "mcu_ts_ms",
        "lat", "lon", "pos_east_m", "pos_north_m",
        "vel_e_ms", "vel_n_ms", "vel_u_ms", "speed_ms", "speed_kmh",
        "roll_deg", "pitch_deg", "yaw_deg",
        "ax_world", "ay_world", "az_world",
        "ab_fwd_ms2", "ab_lat_ms2", "ab_up_ms2",
        "condition",
    ]

    def __init__(self, output_dir: Path):
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
        r_path = output_dir / f"raw_{ts}.csv"
        p_path = output_dir / f"processed_{ts}.csv"

        self._rf = open(r_path, "w", newline="")
        self._pf = open(p_path, "w", newline="")
        self._rw = csv.DictWriter(self._rf, fieldnames=self._RAW_FIELDS)
        self._pw = csv.DictWriter(self._pf, fieldnames=self._PROC_FIELDS)
        self._rw.writeheader()
        self._pw.writeheader()

        print(f"[LOG] raw       → {r_path}")
        print(f"[LOG] processed → {p_path}")

    def log_raw(self, frame: dict):
        row = {f: "" for f in self._RAW_FIELDS}
        row["wall_time"]  = datetime.now().isoformat(timespec="milliseconds")
        row["frame_type"] = frame["type"]
        row["mcu_ts_ms"]  = frame.get("ts", "")
        t = frame["type"]

        if t == "IMU":
            row.update({
                "ax_ms2": frame["ax"], "ay_ms2": frame["ay"], "az_ms2": frame["az"],
                "gx_rads": frame["gx"], "gy_rads": frame["gy"], "gz_rads": frame["gz"],
            })
        elif t == "GPS":
            row.update({
                "gps_lat": frame["lat"], "gps_lon": frame["lon"],
                "gps_speed_kmh": frame["speed_kmh"], "hdop": frame["hdop"],
                "satellites": frame["satellites"], "fix_valid": int(frame["fix"]),
            })
        elif t == "OBD":
            row["obd_speed_kmh"] = frame["speed_kmh"]
        elif t == "LORA_RX":
            row.update({
                "lora_node": frame["node_id"],
                "lora_lat": frame["lat"], "lora_lon": frame["lon"],
                "lora_speed_kmh": frame["speed_kmh"],
                "lora_alert": frame["alert"],
                "lora_rssi_dbm": frame["rssi"],
                "lora_snr_db": frame["snr"],
            })
        self._rw.writerow(row)

    def log_processed(self, ts_ms: int, eskf: "ESKF",
                      a_meas: np.ndarray, condition: str):
        lat, lon = eskf.latlon
        if lat is None:
            return
        rpy     = eskf.euler_deg
        a_world = eskf.world_accel(a_meas)
        a_body  = eskf.body_accel(a_meas)
        self._pw.writerow({
            "wall_time":   datetime.now().isoformat(timespec="milliseconds"),
            "mcu_ts_ms":   ts_ms,
            "lat":         f"{lat:.8f}",
            "lon":         f"{lon:.8f}",
            "pos_east_m":  f"{eskf.p[0]:.3f}",
            "pos_north_m": f"{eskf.p[1]:.3f}",
            "vel_e_ms":    f"{eskf.v[0]:.4f}",
            "vel_n_ms":    f"{eskf.v[1]:.4f}",
            "vel_u_ms":    f"{eskf.v[2]:.4f}",
            "speed_ms":    f"{eskf.speed_ms:.4f}",
            "speed_kmh":   f"{eskf.speed_ms * 3.6:.2f}",
            "roll_deg":    f"{rpy[0]:.3f}",
            "pitch_deg":   f"{rpy[1]:.3f}",
            "yaw_deg":     f"{rpy[2]:.3f}",
            "ax_world":    f"{a_world[0]:.4f}",
            "ay_world":    f"{a_world[1]:.4f}",
            "az_world":    f"{a_world[2]:.4f}",
            "ab_fwd_ms2":  f"{a_body[0]:.4f}",
            "ab_lat_ms2":  f"{a_body[1]:.4f}",
            "ab_up_ms2":   f"{a_body[2]:.4f}",
            "condition":   condition,
        })

    def flush(self):
        self._rf.flush()
        self._pf.flush()

    def close(self):
        self._rf.close()
        self._pf.close()

# ═══════════════════════════════════════════════════════════════════════════════
# LIVE PLOTTER
# ═══════════════════════════════════════════════════════════════════════════════
class LivePlotter:
    _TRAIL   = 300   # position trail samples
    _SPD_WIN = 150   # speed history samples
    _REDRAW  = 20    # redraw every N processed frames

    def __init__(self):
        plt.ion()
        self._fig, (self._ax_map, self._ax_spd) = plt.subplots(
            1, 2, figsize=(14, 6)
        )
        self._fig.suptitle("V2V Live Monitor", fontsize=11)
        self._trail_x:  list[float] = []
        self._trail_y:  list[float] = []
        self._spd_hist: list[float] = []
        self._cond_hist: list[str]  = []
        self.other_vehicles: dict[int, tuple[float, float]] = {}
        self._count = 0

    def update(self, eskf: "ESKF", condition: str):
        if not eskf.initialized or eskf.origin_lat is None:
            return
        self._count += 1
        self._trail_x.append(eskf.p[0])
        self._trail_y.append(eskf.p[1])
        self._spd_hist.append(eskf.speed_ms * 3.6)
        self._cond_hist.append(condition)

        if len(self._trail_x)  > self._TRAIL:
            self._trail_x  = self._trail_x[-self._TRAIL:]
            self._trail_y  = self._trail_y[-self._TRAIL:]
        if len(self._spd_hist) > self._SPD_WIN:
            self._spd_hist = self._spd_hist[-self._SPD_WIN:]
            self._cond_hist = self._cond_hist[-self._SPD_WIN:]

        if self._count % self._REDRAW == 0:
            self._redraw(eskf, condition)

    def add_vehicle(self, node_id: int, lat: float, lon: float,
                    lat0: float, lon0: float):
        enu = latlon_to_enu(lat, lon, lat0, lon0)
        self.other_vehicles[node_id] = (enu[0], enu[1])

    def _redraw(self, eskf: "ESKF", condition: str):
        # ── Map panel ────────────────────────────────────────────────────────
        ax = self._ax_map
        ax.cla()
        ax.set_title("Vehicle Positions (ENU)")
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

        if len(self._trail_x) > 1:
            ax.plot(self._trail_x, self._trail_y,
                    "b-", lw=0.8, alpha=0.4, label="My path")

        # My vehicle
        color = CONDITION_COLORS.get(condition, "blue")
        ax.plot(eskf.p[0], eskf.p[1], "o", color=color,
                ms=10, zorder=5, label=f"Me  [{condition}]")

        # Heading arrow
        yaw = np.radians(eskf.euler_deg[2])
        alen = max(3.0, eskf.speed_ms * 0.5)
        ax.annotate(
            "", xy=(eskf.p[0] + alen * np.cos(yaw),
                    eskf.p[1] + alen * np.sin(yaw)),
            xytext=(eskf.p[0], eskf.p[1]),
            arrowprops=dict(arrowstyle="->", color="blue", lw=1.5),
        )

        # Other vehicles and range lines
        for nid, (vx, vy) in self.other_vehicles.items():
            ax.plot(vx, vy, "r^", ms=9, zorder=5, label=f"Node {nid}")
            ax.plot([eskf.p[0], vx], [eskf.p[1], vy],
                    "r--", lw=0.8, alpha=0.5)
            dist = np.hypot(eskf.p[0] - vx, eskf.p[1] - vy)
            ax.text((eskf.p[0] + vx) / 2, (eskf.p[1] + vy) / 2,
                    f"{dist:.1f} m", fontsize=7,
                    ha="center", va="bottom", color="darkred")

        ax.legend(fontsize=7, loc="upper left")

        # ── Speed panel ───────────────────────────────────────────────────────
        ax2 = self._ax_spd
        ax2.cla()
        ax2.set_xlabel("Sample")
        ax2.set_ylabel("Speed (km/h)")
        ax2.grid(True, alpha=0.3)

        if self._spd_hist:
            xs     = list(range(len(self._spd_hist)))
            colors = [CONDITION_COLORS.get(c, "green") for c in self._cond_hist]
            ax2.plot(xs, self._spd_hist, "k-", lw=0.8, alpha=0.5)
            ax2.scatter(xs, self._spd_hist, c=colors, s=8, zorder=3)
            ax2.set_title(
                f"Speed: {self._spd_hist[-1]:.1f} km/h  │  {self._cond_hist[-1]}",
                fontsize=10,
            )

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()

# ═══════════════════════════════════════════════════════════════════════════════
# FRAME PARSER
# ═══════════════════════════════════════════════════════════════════════════════
def xor_checksum(data: bytes) -> int:
    return reduce(lambda a, b: a ^ b, data, 0)

class FrameStats:
    """Counters and samples for diagnosing 'no valid frame' issues."""
    def __init__(self):
        self.ok            = 0
        self.timeout_start = 0   # no data while hunting for 0xAA
        self.timeout_type  = 0   # got 0xAA, no type byte
        self.unknown_type  = 0   # type byte not in FRAME_SIZES
        self.short_body    = 0   # body truncated before full length
        self.bad_end       = 0   # last byte != 0x55
        self.bad_checksum  = 0   # XOR mismatch
        self.skipped_bytes = 0   # stray bytes before next 0xAA
        self.unknown_type_hist: dict[int, int] = {}
        self.stray_sample  = bytearray()   # rolling sample of stray bytes

    def note_stray(self, b: int):
        self.skipped_bytes += 1
        if len(self.stray_sample) < 64:
            self.stray_sample.append(b)

    def summary(self) -> str:
        parts = [
            f"ok={self.ok}",
            f"timeout(start/type)={self.timeout_start}/{self.timeout_type}",
            f"unknown_type={self.unknown_type}",
            f"short_body={self.short_body}",
            f"bad_end={self.bad_end}",
            f"bad_cksum={self.bad_checksum}",
            f"stray_bytes={self.skipped_bytes}",
        ]
        s = "  ".join(parts)
        if self.unknown_type_hist:
            top = sorted(self.unknown_type_hist.items(),
                         key=lambda kv: -kv[1])[:4]
            s += "  unknown=[" + ", ".join(f"0x{t:02X}×{n}" for t, n in top) + "]"
        if self.stray_sample:
            s += f"  stray_hex={self.stray_sample.hex(' ')[:80]}"
        return s


def read_frame(ser: serial.Serial,
               stats: Optional[FrameStats] = None) -> Optional[dict]:
    # 1. Hunt for FRAME_START
    while True:
        b = ser.read(1)
        if not b:
            if stats is not None:
                stats.timeout_start += 1
            return None
        if b[0] == FRAME_START:
            break
        if stats is not None:
            stats.note_stray(b[0])

    # 2. Type byte
    t = ser.read(1)
    if not t:
        if stats is not None:
            stats.timeout_type += 1
        return None
    if t[0] not in FRAME_SIZES:
        if stats is not None:
            stats.unknown_type += 1
            stats.unknown_type_hist[t[0]] = \
                stats.unknown_type_hist.get(t[0], 0) + 1
        return None
    frame_type = t[0]

    # 3. Body
    expected = FRAME_SIZES[frame_type] - 2
    rest = ser.read(expected)
    if len(rest) < expected:
        if stats is not None:
            stats.short_body += 1
        return None

    body = t + rest
    if body[-1] != FRAME_END:
        if stats is not None:
            stats.bad_end += 1
        return None
    if xor_checksum(body[:-2]) != body[-2]:
        if stats is not None:
            stats.bad_checksum += 1
        return None

    if stats is not None:
        stats.ok += 1
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

    if frame_type == FRAME_HELLO:
        _, ts, node_id, fw_version, _, _ = fields
        return {"type": "HELLO", "ts": ts,
                "node_id": node_id, "fw_version": fw_version}

    return None


# ═══════════════════════════════════════════════════════════════════════════════
# HOST → MCU COMMANDS
# ═══════════════════════════════════════════════════════════════════════════════
# CmdFrame layout (matches firmware):
#   start(1) cmd_type(1) alert_type(1) lat(4) lon(4) speed_kmh(1) cksum(1) end(1)
_CMD_STRUCT = struct.Struct("<BBBffBBB")

def send_cmd(ser: serial.Serial, cmd_type: int,
             alert_type: int = 0, lat: float = 0.0,
             lon: float = 0.0, speed_kmh: int = 0) -> None:
    body = struct.pack("<BBffB", cmd_type, alert_type, lat, lon, speed_kmh)
    cksum = reduce(lambda a, b: a ^ b, body, 0)
    pkt = _CMD_STRUCT.pack(CMD_START, cmd_type, alert_type,
                           lat, lon, speed_kmh, cksum, FRAME_END)
    ser.write(pkt)


def handshake(ser: serial.Serial, stats: FrameStats) -> Optional[dict]:
    """Send CMD_HELLO and wait for the MCU's HELLO reply. Returns the
    HELLO frame on success, or None if the firmware never answered."""
    print(f"[handshake] sending CMD_HELLO (timeout {HANDSHAKE_TRIES}×1s) ...")
    for attempt in range(1, HANDSHAKE_TRIES + 1):
        send_cmd(ser, CMD_HELLO)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            frame = read_frame(ser, stats)
            if frame is None:
                continue
            if frame["type"] == "HELLO":
                return frame
            # Any other frame implies the MCU is already streaming — accept it.
            print("[handshake] firmware was already streaming; treating as connected")
            return frame
        print(f"[handshake] no reply (attempt {attempt}/{HANDSHAKE_TRIES})")
    return None

# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="V2V MCU reader + ESKF fusion")
    parser.add_argument("--port",   default="/dev/ttyACM0")
    parser.add_argument("--baud",   type=int, default=921600)
    parser.add_argument("--outdir", default=".", help="CSV output directory")
    args = parser.parse_args()

    out_dir = Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Opening {args.port} @ {args.baud} baud ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=2)
    except serial.SerialException as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    eskf    = ESKF()
    logger  = DataLogger(out_dir)
    plotter = LivePlotter()

    # Accumulate IMU samples before initialising orientation
    _imu_init_buf: list[np.ndarray] = []
    _IMU_INIT_N   = 50       # 0.5 s at 100 Hz

    last_a_meas = np.array([0., 0., 9.80665])
    condition   = "NORMAL"
    proc_count  = 0
    bad_streak  = 0
    has_gps     = False   # True after first valid GPS fix
    has_obd     = False   # True after first OBD reading

    stats          = FrameStats()
    last_stats_log = 0     # ok-count snapshot of last stats print

    # ── HANDSHAKE: firmware stays silent until we send CMD_HELLO ─────────────
    hello = handshake(ser, stats)
    if hello is None:
        print("[error] Firmware did not respond to CMD_HELLO — aborting.",
              file=sys.stderr)
        ser.close()
        sys.exit(1)
    if hello.get("type") == "HELLO":
        print(f"[handshake] connected: node={hello['node_id']} "
              f"fw={hello['fw_version']} mcu_ts={hello['ts']}ms")

    last_heartbeat = time.monotonic()

    print("Listening — Ctrl+C to stop\n")

    try:
        while True:
            # Heartbeat keeps the firmware's HOST_TIMEOUT_MS from firing
            if time.monotonic() - last_heartbeat >= HEARTBEAT_S:
                send_cmd(ser, CMD_HELLO)
                last_heartbeat = time.monotonic()

            frame = read_frame(ser, stats)
            if frame is None:
                bad_streak += 1
                if bad_streak == 5 or (bad_streak > 5 and bad_streak % 50 == 0):
                    print(f"[warn] No valid frames (streak={bad_streak}) — "
                          "firmware may have dropped the connection "
                          "(host timeout = 3 s)")
                    print(f"[stats] {stats.summary()}")
                    stats.stray_sample.clear()
                continue
            if bad_streak >= 5:
                print(f"[info] Frame sync restored after {bad_streak} bad reads")
            bad_streak = 0

            # HELLO frames are handshake/heartbeat replies — don't log them
            if frame["type"] == "HELLO":
                continue

            # Periodic health line every 500 successful frames
            if stats.ok - last_stats_log >= 500:
                last_stats_log = stats.ok
                print(f"[stats] {stats.summary()}")
                stats.stray_sample.clear()

            logger.log_raw(frame)
            t = frame["type"]

            # ── IMU ──────────────────────────────────────────────────────────
            if t == "IMU":
                a_meas = np.array([frame["ax"], frame["ay"], frame["az"]])
                w_meas = np.array([frame["gx"], frame["gy"], frame["gz"]])
                last_a_meas = a_meas

                if not eskf.initialized:
                    _imu_init_buf.append(a_meas.copy())
                    if len(_imu_init_buf) >= _IMU_INIT_N:
                        eskf.init_orientation(np.mean(_imu_init_buf, axis=0))
                        print("[ESKF] Orientation initialised from gravity alignment")
                    continue

                if not (has_gps and has_obd):
                    continue   # wait for external velocity/position anchor

                eskf.predict(a_meas, w_meas, frame["ts"])

                a_body    = eskf.body_accel(a_meas)
                rpy       = eskf.euler_deg
                condition = detect_condition(a_body, rpy[0], eskf.speed_ms)

                proc_count += 1
                logger.log_processed(frame["ts"], eskf, a_meas, condition)
                plotter.update(eskf, condition)

                if eskf.origin_set:
                    eskf.origin_set = False
                    print(f"[ESKF] Origin: ({eskf.origin_lat:.6f}, {eskf.origin_lon:.6f})")

                # Console at ~10 Hz
                if proc_count % 10 == 0:
                    lat, lon = eskf.latlon
                    lat_s = f"{lat:.6f}" if lat else "---"
                    lon_s = f"{lon:.6f}" if lon else "---"
                    print(
                        f"[ESKF] ({lat_s}, {lon_s})  "
                        f"spd={eskf.speed_ms * 3.6:5.1f} km/h  "
                        f"rpy=({rpy[0]:+5.1f}, {rpy[1]:+5.1f}, {rpy[2]:+5.1f})°  "
                        f"[{condition}]"
                    )

            # ── GPS ──────────────────────────────────────────────────────────
            elif t == "GPS":
                if frame["fix"] and frame["lat"] != 0.0:
                    eskf.update_gps(frame["lat"], frame["lon"], frame["fix"])
                    has_gps = True
                fix_s = "FIX   " if frame["fix"] else "NO FIX"
                print(
                    f"[GPS]  {fix_s}  "
                    f"({frame['lat']:.6f}, {frame['lon']:.6f})  "
                    f"spd={frame['speed_kmh']:.1f} km/h  "
                    f"hdop={frame['hdop']:.1f}  sats={frame['satellites']}"
                )

            # ── OBD ──────────────────────────────────────────────────────────
            elif t == "OBD":
                eskf.update_obd(frame["speed_kmh"])
                has_obd = True
                print(f"[OBD]  speed={frame['speed_kmh']} km/h")

            # ── LoRa RX ──────────────────────────────────────────────────────
            elif t == "LORA_RX":
                if eskf.origin_lat is not None and frame["lat"] != 0.0:
                    plotter.add_vehicle(
                        frame["node_id"], frame["lat"], frame["lon"],
                        eskf.origin_lat, eskf.origin_lon,
                    )
                    my_lat, my_lon = eskf.latlon
                    dist_s = ""
                    if my_lat:
                        d = haversine(my_lat, my_lon, frame["lat"], frame["lon"])
                        dist_s = f"  range={d:.1f} m"
                print(
                    f"[LORA] node={frame['node_id']}  "
                    f"spd={frame['speed_kmh']} km/h  "
                    f"alert={frame['alert']}  "
                    f"rssi={frame['rssi']} dBm  snr={frame['snr']} dB"
                    f"{dist_s}"
                )

            if proc_count % 100 == 0 and proc_count > 0:
                logger.flush()

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        logger.close()
        ser.close()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
