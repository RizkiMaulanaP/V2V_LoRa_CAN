import os

HOME_PATH = "/home/rizkimapa/Games/BeamNG.tech.v0.38.5.0"
binlinux = os.path.join(HOME_PATH, "BinLinux")
os.environ["LD_LIBRARY_PATH"] = binlinux + os.pathsep + os.environ.get("LD_LIBRARY_PATH", "")

"""
BeamNG.tech -> V2V/LoRa telemetry extractor.

Pulls per-vehicle telemetry (position, speed, heading, GPS lon/lat,
linear acceleration, yaw rate) and assembles sensor frames that mirror
the on-wire layout used by V2V_LoRa_CAN.ino on the ESP32 / Heltec node.

Each sensor runs on its own schedule (mirrors the millis() scheduling in
the firmware loop):
    IMU  @ 100 Hz   (every 10 ms)   -> FRAME_IMU
    OBD  @ 2 Hz      (every 0.5 s)   -> FRAME_OBD + FRAME_OBD_EXT
    GPS  @ 0.5 Hz    (every 2 s)     -> FRAME_GPS

Tested against the current BeamNGpy API (GPS + AdvancedIMU automation sensors).
Adjust HOME_PATH / USER_PATH, the map, and spawn position to your setup.
"""

import math
import struct
import time

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import Electrics, AdvancedIMU, GPS

# --- Configuration ---------------------------------------------------------
HOME_PATH = "/home/rizkimapa/Games/BeamNG.tech.v0.38.5.0/"   # install dir containing tech.key
USER_PATH = "/home/rizkimapa/Documents/V2V_LoRa_CAN/BeamNG.tech/"    # e.g. ~/.local/share/BeamNG.drive (or leave None)
MAP = "italy"
SPAWN_POS = (245.11, -906.94, 247.46)
SPAWN_ROT = (0.0010, 0.1242, 0.9884, -0.0872)  # quaternion

# Reference origin for GPS->lon/lat conversion (map origin on the globe).
REF_LON, REF_LAT = 8.8017, 53.0793

# --- Per-sensor rates (match the firmware) ---------------------------------
IMU_HZ        = 100               # FRAME_IMU  — 100 Hz   (IMU_SAMPLE_MS = 10)
IMU_PERIOD_S  = 1.0 / IMU_HZ      # 0.01 s
OBD_PERIOD_S  = 0.5               # FRAME_OBD  — every 0.5 s
GPS_PERIOD_S  = 2.0               # FRAME_GPS  — every 2 s

DURATION_S = 300                   # how long to run

# ---------------------------------------------------------------------------
# Serial frame layout (must match the structs in V2V_LoRa_CAN.ino)
#   Frame: [0xAA | type | ts(4) | payload | checksum | 0x55]
#   Checksum: XOR of all bytes between start and end (exclusive)
# ---------------------------------------------------------------------------
FRAME_START   = 0xAA
FRAME_END     = 0x55
FRAME_IMU     = 0x01
FRAME_GPS     = 0x02
FRAME_OBD     = 0x03
FRAME_OBD_EXT = 0x06

# Bodies are little-endian, starting at the `type` byte (start byte excluded
# from the struct so the XOR checksum spans [type .. last payload byte]).
_IMU_BODY     = struct.Struct("<BIffffff")   # type, ts, ax, ay, az, gx, gy, gz
_GPS_BODY     = struct.Struct("<BIddffBB")   # type, ts, lat, lon, spd_kmh, hdop, sats, fix
_OBD_BODY     = struct.Struct("<BIB")        # type, ts, speed_kmh
_OBD_EXT_BODY = struct.Struct("<BIHh")       # type, ts, rpm, coolant_c


def _xor(buf):
    cs = 0
    for b in buf:
        cs ^= b
    return cs


def build_frame(body):
    """Wrap a packed body (starting at the type byte) into a full serial frame."""
    cs = _xor(body)
    return bytes([FRAME_START]) + body + bytes([cs, FRAME_END])


def imu_frame(ts_ms, ax, ay, az, gx, gy, gz):
    return build_frame(_IMU_BODY.pack(FRAME_IMU, ts_ms, ax, ay, az, gx, gy, gz))


def gps_frame(ts_ms, lat, lon, speed_kmh, hdop, sats, fix_valid):
    return build_frame(_GPS_BODY.pack(FRAME_GPS, ts_ms, lat, lon,
                                      speed_kmh, hdop, sats, fix_valid))


def obd_frame(ts_ms, speed_kmh):
    return build_frame(_OBD_BODY.pack(FRAME_OBD, ts_ms, speed_kmh & 0xFF))


def obd_ext_frame(ts_ms, rpm, coolant_c):
    return build_frame(_OBD_EXT_BODY.pack(FRAME_OBD_EXT, ts_ms,
                                          rpm & 0xFFFF, int(coolant_c)))


def quat_to_yaw_deg(dir_vec):
    """Heading in degrees (0 = +Y / north-ish in BeamNG world space)."""
    x, y = dir_vec[0], dir_vec[1]
    return (math.degrees(math.atan2(x, y))) % 360.0


def main():
    set_up_simple_logging()

    bng = BeamNGpy("localhost", 25252, home=HOME_PATH, user=USER_PATH)
    bng.open(launch=True)

    t0 = time.time()
    def ts_ms():
        return int((time.time() - t0) * 1000) & 0xFFFFFFFF

    try:
        vehicle = Vehicle("ego", model="etk800", licence="V2V", color="White")

        scenario = Scenario(MAP, "v2v_telemetry")
        scenario.add_vehicle(vehicle, pos=SPAWN_POS, rot_quat=SPAWN_ROT)
        scenario.make(bng)

        # Deterministic stepping makes per-tick data reproducible.
        bng.settings.set_deterministic(60)
        bng.scenario.load(scenario)
        bng.scenario.start()

        # --- Attach sensors ---
        # Electrics: speed, rpm, throttle, gear, wheelspeed, etc. (classical sensor)
        # This is our "OBD-II" source (maps to FRAME_OBD / FRAME_OBD_EXT).
        electrics = Electrics()
        vehicle.sensors.attach("electrics", electrics)

        # Advanced IMU: tri-axial acceleration + angular velocity (tech sensor)
        # Sampled at 100 Hz to match IMU_SAMPLE_MS = 10 ms in the firmware.
        imu = AdvancedIMU(
            "imu",
            bng,
            vehicle,
            pos=(0, 0.5, 0.5),
            gfx_update_time=IMU_PERIOD_S,
            is_using_gravity=False,
        )

        # GPS: world position -> lon/lat against reference origin (tech sensor)
        gps = GPS(
            "gps",
            bng,
            vehicle,
            pos=(0, 0, 1.5),
            ref_lon=REF_LON,
            ref_lat=REF_LAT,
            gfx_update_time=GPS_PERIOD_S,
        )

        # Let the AI drive so there is motion to measure.
        vehicle.ai.set_mode("span")
        time.sleep(2.0)

        print("t_ms,frame,lon,lat,speed_kmh,heading_deg,ax,ay,az,yaw_rate,rpm,coolant")

        # --- Scheduler: independent per-sensor cadence (mirrors firmware) ---
        start = time.time()
        next_imu = start
        next_obd = start
        next_gps = start

        # Hold the most recent values so the CSV row always has context.
        last_speed_kmh = 0.0
        last_heading   = 0.0
        last_lon = last_lat = None

        while time.time() - start < DURATION_S:
            now = time.time()

            # --- IMU @ 100 Hz -> FRAME_IMU ---
            if now >= next_imu:
                next_imu += IMU_PERIOD_S
                imu_data = imu.poll()
                if imu_data:
                    latest = imu_data[max(imu_data.keys())]
                    acc = latest.get("accSmooth", latest.get("accRaw", [0, 0, 0]))
                    ang = latest.get("angVel", [0, 0, 0])
                    imu_frame(ts_ms(),
                              acc[0], acc[1], acc[2],
                              ang[0], ang[1], ang[2])   # ready to Serial.write()
                    print(f"{ts_ms()},IMU,,,,,"
                          f"{round(acc[0],3)},{round(acc[1],3)},{round(acc[2],3)},"
                          f"{round(ang[2],4)},,")

            # --- OBD @ 2 Hz (0.5 s) -> FRAME_OBD + FRAME_OBD_EXT ---
            if now >= next_obd:
                next_obd += OBD_PERIOD_S
                vehicle.sensors.poll()                 # refresh electrics + state
                state = vehicle.state
                elec = vehicle.sensors["electrics"]

                vel = state["vel"]
                speed_mps = math.sqrt(vel[0] ** 2 + vel[1] ** 2 + vel[2] ** 2)
                last_speed_kmh = speed_mps * 3.6
                last_heading = quat_to_yaw_deg(state["dir"])

                rpm = int(elec.get("rpm", 0) or 0)
                coolant = int(elec.get("water_temperature",
                                       elec.get("watertemp", 0)) or 0)
                speed_byte = max(0, min(255, int(round(last_speed_kmh))))

                obd_frame(ts_ms(), speed_byte)         # FRAME_OBD
                obd_ext_frame(ts_ms(), rpm, coolant)   # FRAME_OBD_EXT
                print(f"{ts_ms()},OBD,,,{speed_byte},{round(last_heading,2)},"
                      f",,,,{rpm},{coolant}")

            # --- GPS @ 0.5 Hz (2 s) -> FRAME_GPS ---
            if now >= next_gps:
                next_gps += GPS_PERIOD_S
                gps_data = gps.poll()
                g = gps_data[0] if gps_data else {"lon": None, "lat": None}
                last_lon, last_lat = g["lon"], g["lat"]
                if last_lon is not None and last_lat is not None:
                    gps_frame(ts_ms(), last_lat, last_lon,
                              last_speed_kmh, 0.9, 12, 1)   # hdop/sats/fix placeholders
                print(f"{ts_ms()},GPS,{last_lon},{last_lat},"
                      f"{round(last_speed_kmh,2)},{round(last_heading,2)},,,,,,")

            # Sleep until the next scheduled event (whichever sensor is due first).
            sleep_for = min(next_imu, next_obd, next_gps) - time.time()
            if sleep_for > 0:
                time.sleep(sleep_for)

        gps.remove()
        imu.remove()

    finally:
        bng.disconnect()


if __name__ == "__main__":
    main()
