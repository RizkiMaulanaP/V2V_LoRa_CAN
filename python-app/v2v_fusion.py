#!/usr/bin/env python3
"""
Shared V2V sensor-fusion core (no I/O — pure numpy + filterpy).

Extracted from main_ukf.py so multiple front-ends can reuse the same
fusion without pulling in serial / matplotlib:
  • main_ukf.py    — real MCU over USB CDC
  • main_beamng.py — BeamNG.tech simulator, in-process

Contains:
  • geo / heading helpers (ENU ↔ lat/lon, compass ↔ math-radians)
  • EgoUKF          — 4-state UKF (east, north, v, ψ)
  • NeighborTrack / NeighborRegistry — CTRV Kalman per remote node
"""

import numpy as np
from scipy.linalg import cholesky
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints


# ── Numerical guards ────────────────────────────────────────────────────────
# Sane physical bounds for IMU control inputs; anything outside (or non-finite)
# is a BeamNG glitch (collision / teleport / reset) that would wreck the filter.
_MAX_ACCEL = 50.0    # m/s²   (~5 g)
_MAX_YAW   = 10.0    # rad/s  (~570°/s)


def robust_cholesky(P):
    """Positive-definite-safe sqrt for the sigma points.

    Symmetrises P, then retries Cholesky with growing diagonal jitter, and
    finally clips eigenvalues — so a slightly non-PD covariance can't crash
    the UKF (numpy.linalg.LinAlgError) mid-run.
    """
    P = (P + P.T) * 0.5
    try:
        return cholesky(P)
    except np.linalg.LinAlgError:
        n = P.shape[0]
        eps = 1e-9
        for _ in range(10):
            try:
                return cholesky(P + np.eye(n) * eps)
            except np.linalg.LinAlgError:
                eps *= 10.0
        # Last resort: rebuild the nearest PD matrix from its eigen-decomp.
        w, V = np.linalg.eigh(P)
        w = np.clip(w, 1e-9, None)
        P_pd = (V * w) @ V.T
        return cholesky((P_pd + P_pd.T) * 0.5 + np.eye(P.shape[0]) * 1e-9)

# ═══════════════════════════════════════════════════════════════════════════════
# GEO / MATH UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════
_EARTH_R = 6_371_000.0


def latlon_to_enu(lat, lon, lat0, lon0):
    x = np.radians(lon - lon0) * _EARTH_R * np.cos(np.radians(lat0))
    y = np.radians(lat - lat0) * _EARTH_R
    return np.array([x, y])


def enu_to_latlon(e, n, lat0, lon0):
    lat = lat0 + np.degrees(n / _EARTH_R)
    lon = lon0 + np.degrees(e / (_EARTH_R * np.cos(np.radians(lat0))))
    return float(lat), float(lon)


def haversine(lat1, lon1, lat2, lon2):
    phi1, phi2 = np.radians(lat1), np.radians(lat2)
    dphi    = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)
    a = np.sin(dphi/2)**2 + np.cos(phi1)*np.cos(phi2)*np.sin(dlambda/2)**2
    return 2 * _EARTH_R * np.arcsin(np.sqrt(a))


def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi


# Wire convention: heading_deg is COMPASS (0° = North, CW).
# Internal state ψ is math/ENU (0 rad = East, CCW). Convert at the boundary.
def math_rad_to_compass_deg(psi_rad):
    return float((90.0 - np.degrees(psi_rad)) % 360.0)


def compass_deg_to_math_rad(heading_deg):
    return wrap_pi(np.radians(90.0 - heading_deg))


# ═══════════════════════════════════════════════════════════════════════════════
# EGO UKF — 4-state CTRV-ish
#   state  : [east, north, v, psi]
#   predict: IMU @ ~100 Hz with body forward accel + yaw rate as control
#   update : GPS (e, n)  /  OBD (v)
# ═══════════════════════════════════════════════════════════════════════════════
def _fx(x, dt, gz=0.0, ax=0.0):
    e, n, v, psi = x[0], x[1], x[2], x[3]
    return np.array([
        e + v*np.cos(psi)*dt + 0.5*ax*np.cos(psi)*dt**2,
        n + v*np.sin(psi)*dt + 0.5*ax*np.sin(psi)*dt**2,
        v + ax*dt,
        psi + gz*dt,
    ])


def _h_gps(x): return np.array([x[0], x[1]])
def _h_obd(x): return np.array([x[2]])


class EgoUKF:
    def __init__(self):
        pts = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2.0, kappa=1.0,
                                     sqrt_method=robust_cholesky)
        self.ukf = UnscentedKalmanFilter(
            dim_x=4, dim_z=2, dt=0.01, fx=_fx, hx=_h_gps, points=pts,
        )
        self.ukf.x = np.zeros(4)
        self.ukf.P = np.diag([1.0, 1.0, 1.0, 0.1])
        self.ukf.Q = np.diag([0.01, 0.01, 0.1, 0.01])

        self.R_gps = np.diag([1.5**2, 1.5**2])
        self.R_obd = np.array([[0.2**2]])

        self.origin_lat = None
        self.origin_lon = None
        self.has_gps = False
        self.has_obd = False
        self._last_imu_ts = None

    def _set_origin(self, lat, lon):
        self.origin_lat = lat
        self.origin_lon = lon
        self.ukf.x[:] = 0.0
        print(f"[UKF] Origin: ({lat:.6f}, {lon:.6f})")

    def _stabilize(self):
        """Keep P symmetric so tiny float asymmetry can't break the Cholesky."""
        self.ukf.P = (self.ukf.P + self.ukf.P.T) * 0.5

    def predict_imu(self, ax_fwd, gz_yaw, ts_ms):
        if self._last_imu_ts is None:
            self._last_imu_ts = ts_ms
            return
        dt = (ts_ms - self._last_imu_ts) * 1e-3
        self._last_imu_ts = ts_ms
        if dt <= 0 or dt > 0.5:
            return
        if not (self.has_gps and self.has_obd):
            return   # don't integrate IMU until anchored
        # Drop non-finite / out-of-range samples (BeamNG collision, reset, …).
        if not (np.isfinite(ax_fwd) and np.isfinite(gz_yaw)):
            return
        ax_fwd = float(np.clip(ax_fwd, -_MAX_ACCEL, _MAX_ACCEL))
        gz_yaw = float(np.clip(gz_yaw, -_MAX_YAW, _MAX_YAW))
        self.ukf.predict(dt=dt, gz=gz_yaw, ax=ax_fwd)
        self.ukf.x[3] = wrap_pi(self.ukf.x[3])
        self._stabilize()

    def update_gps(self, lat, lon, fix_valid):
        if not fix_valid or lat == 0.0:
            return
        if self.origin_lat is None:
            self._set_origin(lat, lon)
            self.has_gps = True
            return
        z = latlon_to_enu(lat, lon, self.origin_lat, self.origin_lon)
        self.ukf.update(z, R=self.R_gps, hx=_h_gps)
        self._stabilize()
        self.has_gps = True

    def update_obd(self, speed_kmh):
        self.ukf.update(np.array([speed_kmh / 3.6]), R=self.R_obd, hx=_h_obd)
        self._stabilize()
        self.has_obd = True

    # ── Read-out helpers ─────────────────────────────────────────────────────
    @property
    def east(self):  return float(self.ukf.x[0])
    @property
    def north(self): return float(self.ukf.x[1])
    @property
    def speed_ms(self):  return float(self.ukf.x[2])
    @property
    def speed_kmh(self): return float(self.ukf.x[2] * 3.6)
    @property
    def yaw_rad(self):   return float(self.ukf.x[3])
    @property
    def heading_deg(self): return math_rad_to_compass_deg(self.ukf.x[3])

    @property
    def latlon(self):
        if self.origin_lat is None:
            return None, None
        return enu_to_latlon(self.east, self.north,
                             self.origin_lat, self.origin_lon)


# ═══════════════════════════════════════════════════════════════════════════════
# NEIGHBOUR TRACKER — small CTRV KF per node_id
#   state   : [e, n, v, psi]   (ENU metres relative to ego origin, m/s, rad)
# ═══════════════════════════════════════════════════════════════════════════════
class NeighborTrack:
    def __init__(self, node_id, e, n, v, psi, ts_local_ms):
        self.node_id = node_id
        self.x  = np.array([e, n, v, psi], dtype=float)
        self.P  = np.diag([4.0, 4.0, 4.0, 0.25])
        self.last_ts_ms = ts_local_ms
        self.last_rssi  = 0
        self.last_snr   = 0
        self.last_alert = "Normal"

    def predict_to(self, ts_local_ms):
        dt = (ts_local_ms - self.last_ts_ms) * 1e-3
        if dt <= 0:
            return
        if dt > 10.0:
            self.P += np.diag([100.0, 100.0, 25.0, 1.0])
            self.last_ts_ms = ts_local_ms
            return

        e, n, v, psi = self.x
        c, s = np.cos(psi), np.sin(psi)
        self.x = np.array([e + v*c*dt, n + v*s*dt, v, wrap_pi(psi)])

        F = np.eye(4)
        F[0, 2] = c*dt;       F[0, 3] = -v*s*dt
        F[1, 2] = s*dt;       F[1, 3] =  v*c*dt
        Q = np.diag([0.5, 0.5, 1.0, 0.05]) * dt
        self.P = F @ self.P @ F.T + Q
        self.last_ts_ms = ts_local_ms

    def update(self, e, n, v, psi, ts_local_ms):
        self.predict_to(ts_local_ms)
        z = np.array([e, n, v, wrap_pi(psi)])
        y = z - self.x
        y[3] = wrap_pi(y[3])
        H = np.eye(4)
        R = np.diag([3.0**2, 3.0**2, 1.0**2, np.radians(15.0)**2])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[3] = wrap_pi(self.x[3])
        self.P = (np.eye(4) - K @ H) @ self.P

    @property
    def east(self):  return float(self.x[0])
    @property
    def north(self): return float(self.x[1])
    @property
    def speed_kmh(self): return float(self.x[2] * 3.6)
    @property
    def heading_deg(self): return math_rad_to_compass_deg(self.x[3])


class NeighborRegistry:
    def __init__(self, stale_after_s=10.0):
        self.tracks: dict[int, NeighborTrack] = {}
        self.stale_after_ms = int(stale_after_s * 1000)

    def update(self, node_id, lat, lon, speed_kmh, heading_deg,
               origin_lat, origin_lon, ts_local_ms, rssi, snr, alert):
        e, n = latlon_to_enu(lat, lon, origin_lat, origin_lon)
        v    = speed_kmh / 3.6
        psi  = compass_deg_to_math_rad(heading_deg)
        trk  = self.tracks.get(node_id)
        if trk is None:
            trk = NeighborTrack(node_id, e, n, v, psi, ts_local_ms)
            self.tracks[node_id] = trk
        else:
            trk.update(e, n, v, psi, ts_local_ms)
        trk.last_rssi  = rssi
        trk.last_snr   = snr
        trk.last_alert = alert

    def predict_all(self, ts_local_ms):
        for trk in self.tracks.values():
            trk.predict_to(ts_local_ms)

    def prune_stale(self, ts_local_ms):
        gone = [nid for nid, trk in self.tracks.items()
                if ts_local_ms - trk.last_ts_ms > self.stale_after_ms]
        for nid in gone:
            del self.tracks[nid]
        return gone
