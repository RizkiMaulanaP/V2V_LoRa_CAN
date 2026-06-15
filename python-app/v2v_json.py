#!/usr/bin/env python3
"""
JSON bridge for the V2V host → Flutter app.

Serialises the fused ego state (+ neighbour tracks) into the exact contract
the Flutter `TcpDataSource` / `SerialDataSource` parser expects:

    { "ts": <int ms>,
      "ego": { "lat","lon","x","y",
               "speed_kmh","heading_deg","engine_rpm","engine_temp_c" },
      "neighbors": [ { "id","lat","lon","x","y",
                       "speed_kmh","heading_deg","emergency_status" }, ... ] }

Each frame is emitted as one compact JSON object terminated by '\n', so the
client can split the byte stream on newlines.

`JsonTcpServer` is a tiny threaded broadcast server: every connected client
receives every `broadcast()`ed frame. It never blocks the producer — a slow
or dead client is simply dropped.
"""

import json
import socket
import threading
from typing import Optional

# Map the host's alert string → the Flutter EmergencyStatus enum names.
_ALERT_TO_STATUS = {
    "Normal": "NORMAL",
    "Traffic Jam": "WARNING",
    "Road Damage": "WARNING",
    "Hard Brake": "EMERGENCY",
}


def alert_to_status(alert: str) -> str:
    return _ALERT_TO_STATUS.get(alert, "NORMAL")


def build_frame(ts_ms, ukf, neighbors=None, engine_rpm=0, engine_temp_c=0,
                fuel_level_pct=0.0, gps=None, warning=None):
    """Assemble the Flutter-facing frame dict from a fused EgoUKF + neighbours.

    `ukf`        : an EgoUKF (must be anchored — origin set, lat/lon available).
    `neighbors`  : a NeighborRegistry, or None / empty.
    `gps`        : {fix_valid, hdop, satellites} ego GPS-health, or None.
    `warning`    : the collision-warning dict from v2v_warnings.assess(), or None.
    Returns None until the UKF has a geographic origin (nothing to show yet).
    """
    lat, lon = ukf.latlon
    if lat is None:
        return None

    gps = gps or {}
    ego = {
        "lat": round(lat, 8),
        "lon": round(lon, 8),
        "x": round(ukf.east, 3),
        "y": round(ukf.north, 3),
        "speed_kmh": round(ukf.speed_kmh, 2),
        "heading_deg": round(ukf.heading_deg, 2),
        "engine_rpm": int(engine_rpm),
        "engine_temp_c": float(engine_temp_c),
        "fuel_level_pct": round(float(fuel_level_pct), 1),
        "fix_valid": int(gps.get("fix_valid", 1)),
        "hdop": round(float(gps.get("hdop", 0.0)), 2),
        "satellites": gps.get("satellites"),
    }

    neigh_list = []
    if neighbors is not None:
        for nid, trk in neighbors.tracks.items():
            n_lat, n_lon = _track_latlon(trk, ukf)
            neigh_list.append({
                "id": f"N{nid}",
                "lat": round(n_lat, 8),
                "lon": round(n_lon, 8),
                "x": round(trk.east, 3),
                "y": round(trk.north, 3),
                "speed_kmh": round(trk.speed_kmh, 2),
                "heading_deg": round(trk.heading_deg, 2),
                "emergency_status": alert_to_status(getattr(trk, "last_alert", "Normal")),
            })

    return {"ts": int(ts_ms), "ego": ego, "neighbors": neigh_list,
            "warning": warning}


def _track_latlon(trk, ukf):
    """Neighbour ENU (east/north) → lat/lon using the ego UKF's origin."""
    # Local import keeps this module dependency-free unless actually used.
    from v2v_fusion import enu_to_latlon
    return enu_to_latlon(trk.east, trk.north, ukf.origin_lat, ukf.origin_lon)


def frame_to_line(frame: dict) -> bytes:
    return (json.dumps(frame, separators=(",", ":")) + "\n").encode("utf-8")


class JsonTcpServer:
    """Threaded newline-JSON broadcast server (one frame → all clients)."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8765):
        self.host = host
        self.port = port
        self._clients: list[socket.socket] = []
        self._lock = threading.Lock()
        self._srv: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

    def start(self):
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(8)
        self._running = True
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()
        print(f"[JSON] TCP server listening on {self.host}:{self.port}")

    def _accept_loop(self):
        while self._running:
            try:
                conn, addr = self._srv.accept()
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self._lock:
                self._clients.append(conn)
            print(f"[JSON] client connected: {addr[0]}:{addr[1]} "
                  f"({len(self._clients)} total)")

    def broadcast(self, frame: dict):
        if frame is None:
            return
        line = frame_to_line(frame)
        dead = []
        with self._lock:
            for c in self._clients:
                try:
                    c.sendall(line)
                except OSError:
                    dead.append(c)
            for c in dead:
                self._clients.remove(c)
                try:
                    c.close()
                except OSError:
                    pass
        if dead:
            print(f"[JSON] dropped {len(dead)} client(s) "
                  f"({len(self._clients)} remain)")

    @property
    def client_count(self) -> int:
        with self._lock:
            return len(self._clients)

    def stop(self):
        self._running = False
        with self._lock:
            for c in self._clients:
                try:
                    c.close()
                except OSError:
                    pass
            self._clients.clear()
        if self._srv:
            try:
                self._srv.close()
            except OSError:
                pass
