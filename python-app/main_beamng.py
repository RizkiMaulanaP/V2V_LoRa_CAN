#!/usr/bin/env python3
"""
V2V host — BeamNG.tech variant (in-process, no firmware / no serial).

Drives BeamNGpy directly, samples the simulated vehicle at the same per-sensor
rates the real node uses, feeds the SHARED UKF (v2v_fusion.EgoUKF), and streams
the fused state to the Flutter app as newline-JSON over TCP (v2v_json).

    BeamNG sensors ──► EgoUKF ──► JSON/TCP ──► Flutter TcpDataSource

Per-sensor cadence (matches firmware + BeamNG.tech/telemetry.py):
    IMU @ 100 Hz   → ukf.predict_imu(forward_accel, yaw_rate)
    OBD @ 2 Hz      → ukf.update_obd(speed_kmh)   + RPM / coolant telemetry
    GPS @ 0.5 Hz    → ukf.update_gps(lat, lon)

Run the Flutter app pointed at this host's IP:port (default 0.0.0.0:8765).

Requires:
    pip install beamngpy numpy scipy filterpy
"""

import os

# Make BeamNG's bundled libs discoverable before beamngpy import (Linux).
_HOME_PATH = "/home/rizkimapa/Games/BeamNG.tech.v0.38.5.0"
_binlinux = os.path.join(_HOME_PATH, "BinLinux")
os.environ["LD_LIBRARY_PATH"] = _binlinux + os.pathsep + os.environ.get("LD_LIBRARY_PATH", "")

import math
import time
import argparse

import numpy as np
from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Electrics, AdvancedIMU, GPS

from v2v_fusion import EgoUKF, NeighborRegistry, enu_to_latlon
from v2v_json import JsonTcpServer, build_frame

# ── BeamNG configuration (mirror BeamNG.tech/telemetry.py) ──────────────────
HOME_PATH = "/home/rizkimapa/Games/BeamNG.tech.v0.38.5.0/"
USER_PATH = "/home/rizkimapa/Documents/V2V_LoRa_CAN/BeamNG.tech/"
MAP       = "italy"
SPAWN_POS = (245.11, -906.94, 247.46)
SPAWN_ROT = (0.0010, 0.1242, 0.9884, -0.0872)
REF_LON, REF_LAT = 8.8017, 53.0793

# ── Per-sensor rates ────────────────────────────────────────────────────────
IMU_HZ       = 100
IMU_PERIOD_S = 1.0 / IMU_HZ
OBD_PERIOD_S = 0.5
GPS_PERIOD_S = 2.0
JSON_HZ      = 30.0
JSON_PERIOD_S = 1.0 / JSON_HZ

# ── IMU axis mapping (BeamNG AdvancedIMU local frame) ───────────────────────
# The UKF control inputs are *forward* linear accel and yaw (vertical) rate.
# BeamNG accSmooth/angVel are [x, y, z]; for the default mount, vehicle forward
# is +Y and yaw is about +Z. Flip the sign/index here if motion looks mirrored.
def imu_forward_accel(acc):  return acc[1]
def imu_yaw_rate(ang):       return ang[2]


def quat_to_yaw_deg(dir_vec):
    x, y = dir_vec[0], dir_vec[1]
    return (math.degrees(math.atan2(x, y))) % 360.0


# ════════════════════════════════════════════════════════════════════════════
# Optional scripted neighbour — gives the Flutter warning UI something to react
# to (BeamNG single-vehicle sims have no real LoRa peers). Mirrors the cadence
# of the app's MockDataSource: a peer approaches from LEFT then RIGHT.
# ════════════════════════════════════════════════════════════════════════════
_NEIGHBOR_ID     = 99
_SCENARIO_PERIOD = 25.0

def inject_demo_neighbor(neighbors: NeighborRegistry, ukf: EgoUKF,
                         t_sec: float, ts_ms: int):
    if ukf.origin_lat is None:
        return
    t = t_sec % _SCENARIO_PERIOD
    if t < 8:                       # LEFT, 50 m → 5 m
        dist = 50 - (t / 8.0) * 45
        dx, dy = -dist, 0.0
        alert = "Road Damage" if dist < 10 else "Normal"
    elif t < 12:                    # far / safe
        dx, dy, alert = -80.0, 30.0, "Normal"
    elif t < 20:                    # RIGHT, 50 m → 5 m
        dist = 50 - ((t - 12) / 8.0) * 45
        dx, dy = dist, 0.0
        alert = "Road Damage" if dist < 10 else "Normal"
    else:                           # far / safe
        dx, dy, alert = 80.0, 30.0, "Normal"

    # Place relative to ego in ENU, convert to lat/lon for the registry.
    e, n = ukf.east + dx, ukf.north + dy
    n_lat, n_lon = enu_to_latlon(e, n, ukf.origin_lat, ukf.origin_lon)
    neighbors.update(_NEIGHBOR_ID, n_lat, n_lon,
                     speed_kmh=40.0, heading_deg=0.0,
                     origin_lat=ukf.origin_lat, origin_lon=ukf.origin_lon,
                     ts_local_ms=ts_ms, rssi=-80, snr=8, alert=alert)


def main():
    p = argparse.ArgumentParser(description="V2V BeamNG → UKF → Flutter")
    p.add_argument("--host", default="localhost", help="BeamNG host")
    p.add_argument("--bng-port", type=int, default=25252)
    p.add_argument("--json-host", default="0.0.0.0",
                   help="Bind address for the Flutter JSON server")
    p.add_argument("--json-port", type=int, default=8765)
    p.add_argument("--duration", type=float, default=0.0,
                   help="Seconds to run (0 = until Ctrl+C)")
    p.add_argument("--demo-neighbor", action="store_true",
                   help="Inject a scripted LoRa peer so the warning UI reacts")
    args = p.parse_args()

    set_up_simple_logging()

    ukf       = EgoUKF()
    neighbors = NeighborRegistry()
    json_srv  = JsonTcpServer(host=args.json_host, port=args.json_port)
    json_srv.start()

    bng = BeamNGpy(args.host, args.bng_port, home=HOME_PATH, user=USER_PATH)
    bng.open(launch=True)

    t0 = time.time()
    def ts_ms():
        return int((time.time() - t0) * 1000)

    try:
        vehicle = Vehicle("ego", model="etk800", licence="V2V", color="White")
        scenario = Scenario(MAP, "v2v_ukf")
        scenario.add_vehicle(vehicle, pos=SPAWN_POS, rot_quat=SPAWN_ROT)
        scenario.make(bng)

        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.scenario.start()

        electrics = Electrics()
        vehicle.sensors.attach("electrics", electrics)
        imu = AdvancedIMU("imu", bng, vehicle, pos=(0, 0.5, 0.5),
                          gfx_update_time=IMU_PERIOD_S, is_using_gravity=False)
        gps = GPS("gps", bng, vehicle, pos=(0, 0, 1.5),
                  ref_lon=REF_LON, ref_lat=REF_LAT, gfx_update_time=GPS_PERIOD_S)

        vehicle.ai.set_mode("span")
        time.sleep(2.0)

        print(f"[BNG] streaming → JSON on {args.json_host}:{args.json_port}  "
              f"(IMU {IMU_HZ}Hz, OBD {1/OBD_PERIOD_S:.0f}Hz, GPS {1/GPS_PERIOD_S:.1f}Hz)")

        start = time.time()
        next_imu = next_obd = next_gps = next_json = start
        last_rpm = 0
        last_coolant = 0

        while True:
            now = time.time()
            if args.duration and (now - start) >= args.duration:
                break

            # ── IMU @ 100 Hz → predict ───────────────────────────────────────
            if now >= next_imu:
                next_imu += IMU_PERIOD_S
                imu_data = imu.poll()
                if imu_data:
                    latest = imu_data[max(imu_data.keys())]
                    acc = latest.get("accSmooth", latest.get("accRaw", [0, 0, 0]))
                    ang = latest.get("angVel", [0, 0, 0])
                    ukf.predict_imu(imu_forward_accel(acc),
                                    imu_yaw_rate(ang), ts_ms())

            # ── OBD @ 2 Hz → speed update + RPM/coolant telemetry ────────────
            if now >= next_obd:
                next_obd += OBD_PERIOD_S
                vehicle.sensors.poll()
                state = vehicle.state
                elec  = vehicle.sensors["electrics"]

                vel = state["vel"]
                speed_kmh = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2) * 3.6
                ukf.update_obd(speed_kmh)

                last_rpm     = int(elec.get("rpm", 0) or 0)
                last_coolant = int(elec.get("water_temperature",
                                            elec.get("watertemp", 0)) or 0)

            # ── GPS @ 0.5 Hz → position update ───────────────────────────────
            if now >= next_gps:
                next_gps += GPS_PERIOD_S
                gps_data = gps.poll()
                if gps_data:
                    g = gps_data[0]
                    if g.get("lon") is not None and g.get("lat") is not None:
                        ukf.update_gps(g["lat"], g["lon"], fix_valid=True)

            # ── JSON broadcast @ 30 Hz (once UKF is anchored) ────────────────
            if now >= next_json:
                next_json += JSON_PERIOD_S
                if ukf.has_gps and ukf.has_obd:
                    if args.demo_neighbor:
                        inject_demo_neighbor(neighbors, ukf, now - start, ts_ms())
                    neighbors.predict_all(ts_ms())
                    neighbors.prune_stale(ts_ms())
                    fr = build_frame(ts_ms(), ukf, neighbors,
                                     engine_rpm=last_rpm, engine_temp_c=last_coolant)
                    json_srv.broadcast(fr)

            sleep_for = min(next_imu, next_obd, next_gps, next_json) - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)

        gps.remove()
        imu.remove()

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        json_srv.stop()
        bng.disconnect()


if __name__ == "__main__":
    main()
