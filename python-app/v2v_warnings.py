#!/usr/bin/env python3
"""
Shared V2V collision-warning engine.

Pure kinematics in, a single JSON-ready warning out — so the same logic runs in
both the live host (main_ukf.py) and the CSV simulator (simulate_v2v.py), and
the Flutter app just *displays* the result.

Inputs use the local ENU frame (east/north metres about a shared origin) plus
compass heading (0°=North, clockwise) and km/h speed — exactly what both callers
already have.

Implements:
  • closing speed + time-to-collision (TTC) from relative velocity
  • speed-scaled distance thresholds (headway-time + floor) blended with TTC bands
  • approach / path filtering (only converging, in-corridor neighbours warn)
  • EEBL — emergency electronic brake light — for a same-heading lead vehicle
    that broadcasts a hard-brake flag (alert_type == ALERT_BRAKE)
  • GPS lost / low-accuracy ego warning

The output `warning` dict (or None) matches the JSON contract:
    {level, type, direction, distance_m, ttc_s, closing_kmh, neighbor_id}
plus a `buzzer` pattern code for the MCU annunciator.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# ── enums (string values go straight into JSON / Flutter) ───────────────────
LEVEL_SAFE = "safe"
LEVEL_WARNING = "warning"
LEVEL_DANGER = "danger"
_LEVEL_RANK = {LEVEL_SAFE: 0, LEVEL_WARNING: 1, LEVEL_DANGER: 2}

TYPE_FORWARD = "forward_collision"
TYPE_BRAKE = "emergency_brake"
TYPE_CROSS = "cross_traffic"
TYPE_REAR = "approaching_rear"
TYPE_BROADCAST = "emergency_broadcast"
TYPE_GPS = "gps_lost"

DIR_FRONT = "front"
DIR_REAR = "rear"
DIR_LEFT = "left"
DIR_RIGHT = "right"

# Buzzer pattern ids — must match firmware CMD_BUZZER handling.
BUZZER_OFF = 0
BUZZER_WARNING = 1     # single 1000 ms beep
BUZZER_DANGER = 2      # rapid beeps, ~2 s total
BUZZER_GPS = 3         # distinct short double-chirp

# alert_type byte carried in the V2V payload / lora_alert.
ALERT_NORMAL = 0
ALERT_TRAFFIC_JAM = 1
ALERT_BRAKE = 2


@dataclass
class WarnConfig:
    # TTC bands (s)
    ttc_danger: float = 3.0
    ttc_warning: float = 7.0
    # absolute distance floors (m) — trigger regardless of TTC when very close
    dist_danger: float = 8.0
    dist_warning: float = 20.0
    # speed-scaled headway times (s) → distance = speed * headway + floor
    headway_danger: float = 1.0
    headway_warning: float = 2.0
    # path/approach corridor: half-width = lane + slope * distance
    corridor_lane_m: float = 3.0
    corridor_slope: float = 0.10
    # geometry gates (deg)
    same_heading_deg: float = 30.0
    ahead_arc_deg: float = 60.0
    front_arc_deg: float = 45.0
    rear_arc_deg: float = 135.0
    # minimum closing speed to consider "approaching" (m/s)
    min_closing_ms: float = 0.3
    # GPS health
    max_hdop: float = 5.0
    min_sats: int = 4


DEFAULT = WarnConfig()


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _velocity_enu(speed_kmh: float, heading_deg: float) -> tuple[float, float]:
    """Compass heading (0°=N, CW) + km/h → (v_east, v_north) m/s."""
    v = speed_kmh / 3.6
    h = math.radians(heading_deg)
    return v * math.sin(h), v * math.cos(h)


def _direction(bearing_deg: float, cfg: WarnConfig) -> str:
    b = abs(bearing_deg)
    if b <= cfg.front_arc_deg:
        return DIR_FRONT
    if b >= cfg.rear_arc_deg:
        return DIR_REAR
    return DIR_RIGHT if bearing_deg > 0 else DIR_LEFT


def _assess_neighbor(ego, n, cfg: WarnConfig):
    """Return a candidate warning dict for one neighbour, or None if no threat."""
    dx = n["east"] - ego["east"]      # +east
    dy = n["north"] - ego["north"]    # +north
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        dist = 1e-3

    # relative velocity (neighbour − ego) and closing speed along line-of-sight
    ve_e, vn_e = _velocity_enu(ego["speed_kmh"], ego["heading_deg"])
    ve_n, vn_n = _velocity_enu(n["speed_kmh"], n["heading_deg"])
    rvx, rvy = ve_n - ve_e, vn_n - vn_e
    ux, uy = dx / dist, dy / dist
    closing = -(rvx * ux + rvy * uy)            # +ve ⇒ distance shrinking
    ttc = dist / closing if closing > cfg.min_closing_ms else math.inf

    # bearing in ego body frame (0=ahead, +right, −left, ±180=behind)
    he = math.radians(ego["heading_deg"])
    fwd = dx * math.sin(he) + dy * math.cos(he)
    rgt = dx * math.cos(he) - dy * math.sin(he)
    bearing = math.degrees(math.atan2(rgt, fwd))
    direction = _direction(bearing, cfg)

    dpsi = abs(_wrap180(n["heading_deg"] - ego["heading_deg"]))
    same_heading = dpsi <= cfg.same_heading_deg
    ahead = abs(bearing) <= cfg.ahead_arc_deg
    lateral = abs(rgt)
    corridor = cfg.corridor_lane_m + cfg.corridor_slope * dist
    in_path = ahead and lateral <= corridor

    alert = int(n.get("alert", ALERT_NORMAL) or 0)
    approaching = closing > cfg.min_closing_ms

    level = LEVEL_SAFE
    wtype = TYPE_FORWARD

    # 1) EEBL: same-heading lead vehicle broadcasting a hard brake.
    if alert == ALERT_BRAKE and ahead and same_heading:
        level, wtype = LEVEL_DANGER, TYPE_BRAKE
    # 2) Any other emergency broadcast from a relevant (ahead/closing) neighbour.
    elif alert >= ALERT_BRAKE and (ahead or approaching):
        level, wtype = LEVEL_WARNING, TYPE_BROADCAST
    # 3) Forward collision: converging, in the ego's corridor.
    elif in_path and approaching:
        wtype = TYPE_FORWARD
        if ttc < cfg.ttc_danger or dist < cfg.dist_danger:
            level = LEVEL_DANGER
        elif (ttc < cfg.ttc_warning
              or dist < cfg.dist_warning
              or dist < ego["speed_kmh"] / 3.6 * cfg.headway_warning + cfg.dist_danger):
            level = LEVEL_WARNING
    # 4) Cross traffic: converging from the side (different heading).
    elif approaching and not same_heading and direction in (DIR_LEFT, DIR_RIGHT):
        wtype = TYPE_CROSS
        if ttc < cfg.ttc_danger:
            level = LEVEL_DANGER
        elif ttc < cfg.ttc_warning:
            level = LEVEL_WARNING
    # 5) Rear approach: something closing fast from behind.
    elif approaching and direction == DIR_REAR:
        wtype = TYPE_REAR
        if ttc < cfg.ttc_warning:
            level = LEVEL_WARNING

    if level == LEVEL_SAFE:
        return None

    return {
        "level": level,
        "type": wtype,
        "direction": direction,
        "distance_m": round(dist, 1),
        "ttc_s": round(ttc, 1) if math.isfinite(ttc) else None,
        "closing_kmh": round(closing * 3.6, 1),
        "neighbor_id": n.get("id"),
    }


def gps_is_bad(gps, cfg: WarnConfig = DEFAULT) -> bool:
    if not gps:
        return False
    if not gps.get("fix_valid", 1):
        return True
    if gps.get("hdop", 0.0) and gps["hdop"] > cfg.max_hdop:
        return True
    if gps.get("satellites") is not None and gps["satellites"] < cfg.min_sats:
        return True
    return False


def buzzer_for(warning) -> int:
    if warning is None:
        return BUZZER_OFF
    if warning["type"] == TYPE_GPS:
        return BUZZER_GPS
    return BUZZER_DANGER if warning["level"] == LEVEL_DANGER else BUZZER_WARNING


def assess(ego, neighbors, gps=None, cfg: WarnConfig = DEFAULT):
    """Compute the single most-urgent warning.

    ego       : {east, north, speed_kmh, heading_deg}
    neighbors : [{id, east, north, speed_kmh, heading_deg, alert}, ...]
    gps       : {fix_valid, hdop, satellites} or None
    Returns   : (warning_dict_or_None, buzzer_code)
    """
    best = None
    best_key = None
    for n in neighbors or []:
        cand = _assess_neighbor(ego, n, cfg)
        if cand is None:
            continue
        # most severe first, then lowest TTC, then nearest
        ttc = cand["ttc_s"] if cand["ttc_s"] is not None else math.inf
        key = (_LEVEL_RANK[cand["level"]], -ttc, -cand["distance_m"])
        if best_key is None or key > best_key:
            best, best_key = cand, key

    # Collision threats take precedence; otherwise surface GPS health.
    if best is None and gps_is_bad(gps, cfg):
        best = {
            "level": LEVEL_WARNING, "type": TYPE_GPS, "direction": None,
            "distance_m": None, "ttc_s": None, "closing_kmh": None,
            "neighbor_id": None,
        }
    return best, buzzer_for(best)
