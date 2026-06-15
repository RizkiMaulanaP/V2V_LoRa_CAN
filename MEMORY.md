# V2V LoRa CAN — project memory

Design reference for the V2V (vehicle-to-vehicle) collision-warning system.
Living doc — update when the architecture, contract, or radio settings change.

## Data flow
```
[BMI160 IMU + GPS + OBD/CAN] → ESP32 MCU (firmware)
        │  binary serial frames (USB CDC)
        ▼
   python-app host (UKF fusion + neighbour tracking + warning engine)
        │  newline-JSON over TCP :8765
        ▼
   Flutter dashboard (display only)

LoRa air link: MCU ⇄ MCU (other vehicles), compact 11-byte broadcast.
The host/sim never sees air bytes — only the decoded serial LoRaRxFrame / JSON.
```

## Components
- **Firmware**: `V2V_LoRa_CAN.ino` (MCP2515 CAN variant, the one in use).
  `V2V_LoRa_TWAI/` is the ESP32-TWAI variant — **unused, do not edit**.
- **Host**: `python-app/main_ukf.py` (live), `simulate_v2v.py` (CSV replay).
  Shared: `v2v_fusion.py` (UKF + CTRV neighbour tracks), `v2v_warnings.py`
  (collision engine), `v2v_json.py` (Flutter contract).
- **App**: `Flutter-App/v2v/` (TcpDataSource → localhost:8765).
- **Tools**: `plot_path.py` (trajectory plot incl. neighbours),
  `examples/CompactPayload/` (payload codec ref), `examples/BuzzerPWM/` (buzzer
  bench test — PWM volume; production uses full-volume digitalWrite).

## LoRa radio (both nodes must match)
- 921 MHz (AS923, Bandung), TX power 20 dBm, BW 125 kHz, **SF8**, CR 4/8.
- Airtime ≈ **107 ms** per 11-byte frame.
- **CAM-style TX trigger** (not fixed interval): broadcast when moved >4 m OR
  heading >4° OR speed ≥2 km/h, clamped to [500 ms, 2 s] heartbeat; checked
  every 100 ms. Fast/turning car beacons more; steady car backs off.
- CAD listen-before-talk was tried and **removed** (stalled TX). No LBT now.

## Over-air payload — compact 11 bytes (FW_VERSION 3)
Global fixed-point (no region config). Layout little-endian per field:
`[0] node(6b)|alert(2b)  [1] heading(×360/256)  [2] speed(km/h)
 [3-5] lat 24b(−90..90,~1.2m)  [6-8] lon 24b(−180..180,~2.4m)  [9-10] tx_ts16`
Encoded/decoded by `v2v_air_pack`/`v2v_air_unpack`. alert_type: 0=normal,
1=traffic-jam, 2=hard-brake (EEBL). Internal `V2V_Payload` struct (19 B) is the
full-precision in-MCU representation; only the wire format is compact.

## Warning engine (`v2v_warnings.py`, shared by host + sim)
`assess(ego, neighbors, gps) -> (warning, buzzer)`: closing-speed/TTC,
speed-scaled thresholds, path/corridor filtering, EEBL (dedicated brake flag),
GPS-lost. Flutter only displays the result. JSON `warning` =
`{level,type,direction,distance_m,ttc_s,closing_kmh,neighbor_id}`; ego carries
`fix_valid/hdop/satellites/fuel_level_pct`.

## Buzzer (GP4, full volume, no PWM)
Non-blocking pattern player (`startBuzzer`/`serviceBuzzer`), driven by the host
over serial via `CMD_BUZZER` (0x04, pattern id in alert byte):
- warning = 2 quick beeps; danger = that burst repeated ~2.5 s; gps = double chirp.
- Host cooldown: warning re-beeps at most every **5 s per node**; danger re-arms
  every 2.5 s while active.
Hard braking is detected on the sender (OBD speed drop) → broadcasts ALERT_BRAKE.

## Activity LEDs
GP17 = CAN RX, GP6 = LoRa TX/RX, GP5 = GPS sentence — single non-blocking blink.

## Run it
```
python-app$  python simulate_v2v.py --loop            # CSV → dashboard (no HW)
python-app$  python main_ukf.py --json-tcp 8765       # live hardware
Flutter-App/v2v$  flutter run -d linux
```
`simulate_v2v.py --inject-brake SEC` demos EEBL; `--serial PORT` drives a bench buzzer.

## Open / not done
- Firmware not compile-verified here (no arduino-cli) — needs a build/flash check.
- No LBT/CAD; shorter CAM intervals raise collision exposure a bit.
- Compact payload could go to ≤8 B (~90 ms) with sub-byte packing if needed.
