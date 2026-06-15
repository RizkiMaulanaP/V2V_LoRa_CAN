# V2V LoRa CAN — Vehicle-to-Vehicle Collision Warning System

A research prototype for **vehicle-to-vehicle (V2V) safety messaging over LoRa**.
Each vehicle fuses GPS + IMU + OBD-II data, broadcasts its state to nearby
vehicles over a long-range LoRa link, and warns the driver (visual dashboard +
audible buzzer) of collision threats computed from the relative motion of
neighbours.

```
 ┌───────────── Vehicle A ─────────────┐         ┌──── Vehicle B ────┐
 │  GPS ┐                                │         │                   │
 │  IMU ┼─► ESP32 MCU ──USB──► Pi host   │  LoRa   │   ESP32 MCU ...   │
 │  OBD ┘   (firmware)  CDC   (UKF +     │◄═══════►│                   │
 │            │                warnings) │ 921 MHz │                   │
 │         GP4 buzzer ◄──CMD_BUZZER──┘   │         └───────────────────┘
 │            │                  │ TCP JSON :8765
 │         3 status LEDs         ▼
 └────────────────────────  Flutter dashboard (phone / desktop)
```

- **Firmware** (ESP32 / SX1262) — sensor acquisition, LoRa TX/RX, buzzer + LEDs.
- **Host** (Python on a Raspberry Pi / laptop) — UKF sensor fusion, neighbour
  tracking, the collision-warning engine, and the JSON bridge to the app.
- **App** (Flutter) — the driver dashboard (gauges + differentiated warnings).
- **Simulator** — replays recorded drives into the app, no hardware needed.

See [`MEMORY.md`](MEMORY.md) for a condensed design reference.

---

## 1. Hardware (per vehicle)

| Block | Part | Interface |
|-------|------|-----------|
| MCU + radio | Heltec ESP32 + **SX1262** LoRa (Wireless-Tracker class) | — |
| GNSS | on-board GNSS (UC6580-class) | UART `Serial1` 115200, RX=GP33 TX=GP34, power enable GP3 |
| IMU | **BMI160** (accel ±8 g, gyro ±500 °/s) | SPI, CS=GP7 |
| CAN / OBD-II | **MCP2515** (8 MHz xtal) @ 500 kbit/s | SPI, CS=GP15, INT=GP16 |
| Display | ST7735 TFT (on-board) | — |
| Annunciator | active buzzer | GP4 |
| Status LEDs | CAN / LoRa / GPS activity | GP17 / GP6 / GP5 |
| Host link | USB CDC serial @ 921600 | — |

**Shared SPI bus** (MCP2515 + BMI160): SCK=GP9, MISO=GP11, MOSI=GP10.
PCB design files in [`pcb-design/`](pcb-design/).

---

## 2. LoRa radio

| Parameter | Value |
|-----------|-------|
| Frequency | 921 MHz (AS923) |
| TX power | 20 dBm |
| Bandwidth | 125 kHz |
| Spreading factor | **SF9** |
| Coding rate | 4/8 |
| Modem | LoRa, explicit header, CRC on |
| Airtime | ≈ 107 ms / frame (11-byte payload) |

### Transmit triggering — ETSI CAM-style
Instead of a fixed interval, a node broadcasts when its state has *changed
meaningfully*, so fast/turning vehicles update more often and parked ones go
quiet. Evaluated every 100 ms; transmit when **any** of:

- position moved **> 4 m**, or
- heading changed **> 4°**, or
- speed changed **≥ 2 km/h**,

bounded to **[500 ms, 2 s]** (the upper bound is a heartbeat). Hard-brake events
(EEBL) and host-driven alerts are sent immediately.

### Compact over-air payload — 11 bytes
To keep airtime low, the broadcast is a packed 11-byte message (vs a naïve
19-byte struct), little-endian per field:

| bytes | field | encoding | resolution |
|------|-------|----------|-----------|
| 0 | node_id (6b) \| alert (2b) | `(id<<2)\|alert` | 0..63 / 0..3 |
| 1 | heading | `v·360/256` | 1.41° |
| 2 | speed | km/h | 1 km/h |
| 3–5 | latitude | 24-bit, −90..+90 | ~1.2 m |
| 6–8 | longitude | 24-bit, −180..+180 | ~2.4 m |
| 9–10 | tx_ts | `millis() & 0xFFFF` | 1 ms (wraps 65 s) |

`alert`: `0` normal, `1` traffic-jam, `2` hard-brake. Codec + rationale in
[`examples/CompactPayload/`](examples/CompactPayload/).

---

## 3. Firmware (`V2V_LoRa_CAN.ino`)

`FW_VERSION 3`. Single-file Arduino sketch (Heltec ESP32 core).
> The `V2V_LoRa_TWAI/` folder is an alternate build using the ESP32 built-in
> TWAI controller instead of the MCP2515 — **not maintained / not in use**.

Responsibilities:
- **Sensor acquisition**: BMI160 @ 100 Hz, GNSS (TinyGPS++), OBD-II PID polling
  (speed / RPM / coolant) over CAN.
- **Host serial protocol** (USB CDC): length-typed frames
  `[0xAA | type | ts | payload | XOR-cksum | 0x55]`:

  | frame | id | bytes | contents |
  |-------|----|-------|----------|
  | IMU | 0x01 | 32 | ax/ay/az, gx/gy/gz |
  | GPS | 0x02 | 34 | lat/lon, speed, hdop, sats, fix |
  | OBD | 0x03 | 9 | speed |
  | OBD_EXT | 0x06 | 12 | RPM, coolant |
  | HELLO | 0x05 | 10 | node_id, fw_version |
  | LORA_RX | 0x04 | 30 | decoded neighbour + RSSI/SNR |

  Host→MCU commands `[0xBB | cmd | … | cksum | 0x55]`: `CMD_HELLO`,
  `CMD_UPDATE_STATE` (push fused pose back so broadcasts carry filtered data),
  `CMD_BROADCAST` (immediate alert), `CMD_BUZZER` (drive GP4; pattern in the
  alert byte).
- **LoRa**: pack/broadcast own state (compact), unpack neighbours in `OnRxDone`
  and forward to the host via `LORA_RX`.
- **EEBL**: `detectHardBrake()` flags own rapid deceleration (OBD speed drop) and
  broadcasts `ALERT_BRAKE` for ~2 s.
- **Buzzer** (GP4, full-volume, non-blocking pattern player): `1` warning = two
  quick beeps, `2` danger = that burst repeated ~2.5 s, `3` GPS = double-chirp.
- **Activity LEDs**: single non-blocking blink per CAN/LoRa/GPS event.

A standalone PWM buzzer-volume bench sketch lives in
[`examples/BuzzerPWM/`](examples/BuzzerPWM/) (production uses full volume).

---

## 4. Host (`python-app/`)

| File | Role |
|------|------|
| `main_ukf.py` | live host: reads MCU serial, runs fusion, streams JSON, drives buzzer |
| `v2v_fusion.py` | ego **UKF** (E,N,v,ψ) + per-neighbour **CTRV** Kalman tracks |
| `v2v_warnings.py` | shared collision-warning engine |
| `v2v_json.py` | JSON contract + threaded TCP broadcast server |
| `simulate_v2v.py` | CSV-replay host (no hardware) |
| `plot_path.py` | offline trajectory plot (ego + neighbours) |

### Warning engine
`assess(ego, neighbors, gps) → (warning, buzzer)`:

- **Closing speed & TTC** from relative velocity.
- **Speed-scaled thresholds** (TTC bands + headway distance, not fixed radii).
- **Path / approach filtering** (only converging, in-corridor neighbours warn;
  classifies forward / cross / rear).
- **EEBL** for a same-heading lead vehicle broadcasting a brake flag.
- **GPS-lost / low-accuracy** ego warning.
- Per-node buzzer cooldown (warning re-beeps ≤ once / 5 s; danger re-arms ~2.5 s).

### JSON contract (host → app, newline-delimited, TCP :8765)
```json
{
  "ts": 123456,
  "ego": {"lat","lon","x","y","speed_kmh","heading_deg",
          "engine_rpm","engine_temp_c","fuel_level_pct",
          "fix_valid","hdop","satellites"},
  "neighbors": [{"id","lat","lon","x","y","speed_kmh","heading_deg",
                 "emergency_status"}],
  "warning": {"level","type","direction","distance_m","ttc_s",
              "closing_kmh","neighbor_id"}   // or null
}
```
Full spec: [`Flutter-App/v2v/docs/DATA_CONTRACT.md`](Flutter-App/v2v/docs/DATA_CONTRACT.md).

---

## 5. App (`Flutter-App/v2v/`)

Driver dashboard: RPM + speed gauges, engine temp/fuel, and a central
**warning card**. Data sources are swappable in `home_screen.dart`:
`TcpDataSource` (live host / simulator), `MockDataSource` (offline CSV),
`SerialDataSource`, `JsonlFileDataSource`.

Warnings are **host-computed** and merely rendered, with differentiated types:
forward-collision, emergency-brake, cross-traffic, approaching-rear,
emergency-broadcast, GPS-lost — each with its own icon/colour, plus distance /
TTC / closing speed and a GPS-health chip.

---

## 6. Build & run

### Firmware
Arduino IDE / arduino-cli with the **Heltec ESP32** board package. Set a unique
`NODE_ID` per vehicle, flash both nodes (SF and air format must match → same
firmware version on every node), open serial @ 921600.

### Host
```bash
cd python-app
pip install pyserial numpy scipy matplotlib filterpy pandas
python main_ukf.py --json-tcp 8765           # auto-detect MCU; serve app on :8765
# pin the serial port if needed:
python main_ukf.py --port /dev/ttyACM0 --json-tcp 8765
```

### App
```bash
cd Flutter-App/v2v
flutter pub get
flutter run -d linux        # dashboard connects to localhost:8765
```
For a phone/tablet, set `TcpDataSource(host: '<PC-LAN-IP>')` in
`lib/presentation/screens/home_screen.dart` (server binds `0.0.0.0`).

### Simulator (no hardware)
```bash
cd python-app
python simulate_v2v.py --loop                # replay newest recording → :8765
python simulate_v2v.py --inject-brake 60     # demo an EEBL event at t=60 s
python simulate_v2v.py --serial /dev/ttyACM0 # also drive a bench buzzer
```

---

## 7. Repository layout
```
V2V_LoRa_CAN.ino        firmware (MCP2515 CAN variant — in use)
V2V_LoRa_TWAI/          firmware (ESP32 TWAI variant — unused)
python-app/             host, fusion, warning engine, simulator, tools
Flutter-App/v2v/        driver dashboard
examples/CompactPayload/ over-air payload codec + spec
examples/BuzzerPWM/     buzzer PWM volume bench sketch
pcb-design/             hardware design files
BMI160-Arduino/         IMU library
MEMORY.md               condensed design reference
```

---

## 8. Status & limitations
- Two-node tested; firmware is single-file and board-specific (Heltec).
- No listen-before-talk (CAD was evaluated and removed) — short CAM intervals
  raise collision exposure slightly.
- LoRa duty cycle exceeds the 1 % AS923 regulatory limit at short intervals;
  this is a **research prototype**, tune `CAM_T_MIN_MS` / SF for compliant use.
- `tx_ts` wraps every ~65 s (one-way latency only).
- Research/educational project — not a certified safety system.

## License
See [`LICENSE`](LICENSE).
