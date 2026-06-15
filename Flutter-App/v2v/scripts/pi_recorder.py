#!/usr/bin/env python3
# ============================================================
# pi_recorder.py — Record V2V data ke file JSONL
# ------------------------------------------------------------
# Untuk tim HARDWARE. Jalankan di Pi saat skenario test.
#
# Workflow:
#   1. Setup sensor + Pi
#   2. python3 pi_recorder.py --scenario persimpangan --duration 60
#   3. Jalankan skenario (drive mobil sesuai plan)
#   4. Recorder otomatis stop setelah --duration detik
#   5. File output: recordings/<tanggal>_<scenario>_runN.jsonl
#   6. Kasih file ke tim Flutter
#
# TODO untuk tim hardware: ganti fungsi placeholder
#   - read_mcu_frame()
#   - run_ukf_on_ego()
#   - decode_lora_neighbors()
# ============================================================

import argparse
import json
import os
import sys
import time
from datetime import datetime


# ============================================================
# PLACEHOLDER FUNCTIONS — ganti dengan implementasi real
# ============================================================

def read_mcu_frame():
    """
    Baca 1 frame dari MCU via USB CDC.
    Return dict berisi raw sensor data.

    TODO: Implementasi sesuai protokol PCB tim hardware.
    Contoh return:
      {
        'gps_lat': -6.358883,
        'gps_lon': 107.292568,
        'imu_ax': 0.012, 'imu_ay': -0.003, 'imu_az': 9.78,
        'imu_gx': 0.01, 'imu_gy': 0.02, 'imu_gz': -0.001,
        'obd_speed_kmh': 23.5,
        'obd_rpm': 1850,
        'obd_temp_c': 92,
      }
    """
    raise NotImplementedError("Implement: baca dari /dev/ttyACM0 atau serupa")


def run_ukf_on_ego(raw_frame, prev_state):
    """
    Jalankan UKF pada raw sensor data ego.
    Return ego state: lat, lon, heading_deg, speed_kmh.

    TODO: Implementasi UKF sesuai paper/research tim.
    """
    raise NotImplementedError("Implement: UKF fusion GPS + IMU + OBD speed")


def decode_lora_neighbors():
    """
    Decode buffer LoRA yang berisi broadcast dari mobil lain.
    Return list of dict: lat, lon, speed_kmh, heading_deg,
    emergency_status, id.

    TODO: Implementasi sesuai protokol LoRA messaging.
    """
    raise NotImplementedError("Implement: parse LoRA scan buffer")


def neighbour_track(neighbor_raw, prev_track):
    """
    Propagate posisi neighbor pakai GPS + speed history.
    Return updated neighbor state.
    """
    return neighbor_raw  # simplification — propagate logic implement nanti


# ============================================================
# MAIN RECORDER
# ============================================================

def main():
    parser = argparse.ArgumentParser(description='V2V Pi Data Recorder')
    parser.add_argument('--scenario', required=True,
                        help='nama skenario (mis. persimpangan, overtaking)')
    parser.add_argument('--run', type=int, default=1,
                        help='nomor run (default: 1)')
    parser.add_argument('--duration', type=int, default=60,
                        help='durasi recording dalam detik (default: 60)')
    parser.add_argument('--rate', type=int, default=30,
                        help='Hz (frame per detik), default: 30')
    parser.add_argument('--output-dir', default='recordings',
                        help='folder output (default: recordings/)')
    args = parser.parse_args()

    # Setup output file
    os.makedirs(args.output_dir, exist_ok=True)
    date_str = datetime.now().strftime('%Y-%m-%d')
    fname = f'{date_str}_{args.scenario}_run{args.run}.jsonl'
    fpath = os.path.join(args.output_dir, fname)

    if os.path.exists(fpath):
        ans = input(f'File {fpath} sudah ada. Overwrite? (y/N): ')
        if ans.lower() != 'y':
            print('Aborted')
            sys.exit(1)

    print(f'== Recording skenario: {args.scenario} run #{args.run}')
    print(f'   Durasi: {args.duration}s @ {args.rate}Hz')
    print(f'   Output: {fpath}')
    print('   Press Ctrl+C to stop early\n')

    interval = 1.0 / args.rate
    end_time = time.time() + args.duration
    frame_count = 0
    ego_state = None  # UKF state

    try:
        with open(fpath, 'w') as f:
            while time.time() < end_time:
                t0 = time.time()

                # 1. Baca sensor
                raw = read_mcu_frame()

                # 2. UKF ego
                ego_state = run_ukf_on_ego(raw, ego_state)

                # 3. Neighbour track
                neighbors_raw = decode_lora_neighbors()
                neighbors = [neighbour_track(n, None) for n in neighbors_raw]

                # 4. Bentuk JSON frame
                frame = {
                    'ts': int(time.time() * 1000),
                    'ego': {
                        'lat': ego_state['lat'],
                        'lon': ego_state['lon'],
                        'speed_kmh': raw['obd_speed_kmh'],
                        'heading_deg': ego_state['heading_deg'],
                        'engine_rpm': raw['obd_rpm'],
                        'engine_temp_c': raw['obd_temp_c'],
                    },
                    'neighbors': [
                        {
                            'id': n['id'],
                            'lat': n['lat'],
                            'lon': n['lon'],
                            'speed_kmh': n['speed_kmh'],
                            'heading_deg': n['heading_deg'],
                            'emergency_status': n.get('emergency_status', 'NORMAL'),
                        }
                        for n in neighbors
                    ],
                }

                # 5. Tulis ke file
                f.write(json.dumps(frame) + '\n')
                f.flush()  # penting: pastikan data masuk disk
                frame_count += 1

                if frame_count % args.rate == 0:
                    remaining = int(end_time - time.time())
                    print(f'   ... {frame_count} frames, {remaining}s remaining')

                # Throttle
                elapsed = time.time() - t0
                sleep_time = interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        print('\n   ! Recording stopped by user')

    print(f'\n== Done! {frame_count} frames recorded to {fpath}')
    print(f'   File size: {os.path.getsize(fpath) / 1024:.1f} KB')


if __name__ == '__main__':
    main()
