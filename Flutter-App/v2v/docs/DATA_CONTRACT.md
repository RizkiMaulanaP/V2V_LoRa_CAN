# V2V Data Contract — Pi → Flutter App

Dokumen ini menjelaskan **format data** yang harus dikirim dari Raspberry Pi ke Flutter app untuk dashboard V2V. Disepakati antara tim hardware (yang implementasi Pi processing) dan tim software (yang implementasi Flutter UI).

## Konteks arsitektur

```
[Sensors]                  [PCB MCU]                   [Raspberry Pi]                 [Flutter App]
   │                          │                              │                              │
 OBD ─┐                       │                              │                              │
 GPS ─┼──► pack frame ──USB── │ ──── UKF + Neighbour Track ──► ── JSON via TCP socket ────► UI
 IMU ─┘                       │                              │   (atau pipe/serial)         │
                                                             │                              │
                                                  Output:                                   │
                                                  • Ego lat/lon/heading/speed                │
                                                  • Engine RPM/temp                         │
                                                  • Neighbors lat/lon/heading/status        │
```

**Yang dibahas dokumen ini:** anak panah dari Pi ke Flutter app (panah terakhir).

## Transport layer

**Recommended:** TCP socket di localhost. Pi menjadi server, Flutter menjadi client.

| Parameter | Value |
|-----------|-------|
| Protocol | TCP/IP |
| Host | `127.0.0.1` |
| Port | `5555` (default; bisa di-config) |
| Frame delimiter | Newline `\n` |
| Encoding | UTF-8 |
| Format | JSON (line-delimited) |

**Alternatif:** USB CDC serial port langsung kalau Flutter app diluar Pi (misal di Android head unit). Format JSON tetap sama, hanya transport berbeda.

## Frame rate

| Min | Recommended | Max |
|-----|-------------|-----|
| 10 Hz | 30 Hz | 60 Hz |

Pi kirim 1 JSON line per frame. Flutter UI di-throttle ke max 60 Hz untuk render efficiency.

## Format JSON — satu frame

Setiap frame adalah **satu baris JSON** diakhiri `\n`:

```json
{
  "ts": 1748534400123,
  "ego": {
    "lat": -6.358883,
    "lon": 107.292568,
    "speed_kmh": 23.5,
    "heading_deg": 145.2,
    "engine_rpm": 1850,
    "engine_temp_c": 92,
    "fuel_level_pct": 67.5
  },
  "neighbors": [
    {
      "id": "B01",
      "lat": -6.358900,
      "lon": 107.292600,
      "speed_kmh": 45.0,
      "heading_deg": 280.0,
      "emergency_status": "NORMAL"
    },
    {
      "id": "B02",
      "lat": -6.358820,
      "lon": 107.292500,
      "speed_kmh": 38.0,
      "heading_deg": 145.0,
      "emergency_status": "EMERGENCY"
    }
  ]
}
```

## Field definitions

### Top level

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `ts` | int64 | Yes | Timestamp millis since epoch (UTC) |
| `ego` | object | Yes | State mobil ego (kita sendiri) |
| `neighbors` | array | Yes | Array mobil lain. Empty array `[]` kalau tidak ada |

### `ego` object

| Field | Type | Required | Unit / Range | Description |
|-------|------|----------|--------------|-------------|
| `lat` | float | Yes | WGS84 latitude, decimal degrees | Output UKF |
| `lon` | float | Yes | WGS84 longitude, decimal degrees | Output UKF |
| `speed_kmh` | float | Yes | km/h, 0-300 | Dari OBD PID 0x0D |
| `heading_deg` | float | Yes | derajat, 0-360 (0=Utara, 90=Timur, clockwise) | Dari UKF (IMU fusion) |
| `engine_rpm` | float | Yes | RPM, 0-9000 | Dari OBD PID 0x0C |
| `engine_temp_c` | float | Yes | Celsius, -40 to 150 | Dari OBD PID 0x05 |
| `fuel_level_pct` | float | Yes | persen, 0-100 (0 = kosong, 100 = penuh) | Dari OBD PID 0x2F |

### `neighbors[]` object (per mobil)

| Field | Type | Required | Unit / Range | Description |
|-------|------|----------|--------------|-------------|
| `id` | string | Yes | unique ID (MAC/VIN/UUID) | Identitas mobil — harus konsisten antar frame |
| `lat` | float | Yes | WGS84 latitude | Dari LoRA broadcast → Neighbour Track |
| `lon` | float | Yes | WGS84 longitude | Sama |
| `speed_kmh` | float | Yes | km/h | Dari LoRA broadcast |
| `heading_deg` | float | Yes | derajat, 0-360 | Dari LoRA broadcast |
| `emergency_status` | string | Yes | `"NORMAL"`, `"WARNING"`, `"EMERGENCY"` | Self-declared status dari mobil itu |

### `emergency_status` values

| Value | Arti | Efek di UI |
|-------|------|------------|
| `"NORMAL"` | Kondisi aman | Warning card cuma muncul kalau jarak < 25m |
| `"WARNING"` | Hati-hati (misal pengereman) | Minimal WARNING level di UI walau jarak masih jauh |
| `"EMERGENCY"` | Bahaya (kecelakaan, mogok) | Paksa DANGER level (merah pulsing) regardless of distance |

## Koordinat frame

**Format:** GEOGRAPHIC (WGS84 lat/lon decimal degrees). **Bukan** ENU/x-y meters.

Flutter app yang lakukan konversi lat/lon → distance & arah relatif ego:
1. Hitung delta lat/lon antar ego dan neighbor
2. Convert ke ENU meters pakai approximation lokal (valid untuk jarak <1km)
3. Rotate ke body frame ego pakai `ego.heading_deg`
4. Output: distance, direction (LEFT/RIGHT/FRONT/REAR)

**Tim hardware tidak perlu hitung body-frame.** Cukup pastikan `heading_deg` akurat dan konsisten (0=Utara, clockwise).

## Warning logic di Flutter (referensi)

Setelah parsing tiap frame, Flutter menjalankan untuk tiap neighbor:

```
1. Hitung distance dari ego (lat/lon → ENU meters → magnitude)
2. Hitung body-frame coords (ENU rotated by ego heading)
3. Tentukan base level:
     distance < 10m  → DANGER
     distance < 25m  → WARNING
     else            → SAFE (tidak tampil)
4. Override dari emergency_status:
     EMERGENCY → DANGER (regardless of distance)
     WARNING + base=SAFE → WARNING
5. Pilih ancaman TERDEKAT untuk ditampilkan
6. Tentukan arah relatif:
     dyBody > 5  → FRONT
     dyBody < -5 → REAR
     dxBody < 0  → LEFT
     dxBody > 0  → RIGHT
```

## Contoh implementasi Python (Pi side)

```python
import socket
import json
import time

# Setup server
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('127.0.0.1', 5555))
server.listen(1)

print("Waiting for Flutter app to connect...")
conn, addr = server.accept()
print(f"Connected: {addr}")

while True:
    # 1. Read sensor data (USB CDC)
    raw_frame = read_from_mcu()  # implement this

    # 2. Run UKF on ego data
    ego_state = run_ukf(raw_frame)  # output: lat, lon, speed, heading

    # 3. Decode LoRA broadcasts → neighbors
    neighbors = decode_lora_buffer()  # output: list of dicts

    # 4. Run Neighbour Track for each neighbor
    neighbors = [neighbour_track(n) for n in neighbors]

    # 5. Build JSON frame
    frame = {
        "ts": int(time.time() * 1000),
        "ego": {
            "lat": ego_state['lat'],
            "lon": ego_state['lon'],
            "speed_kmh": ego_state['speed'],
            "heading_deg": ego_state['heading'],
            "engine_rpm": raw_frame['rpm'],
            "engine_temp_c": raw_frame['temp'],
        },
        "neighbors": [
            {
                "id": n['id'],
                "lat": n['lat'],
                "lon": n['lon'],
                "speed_kmh": n['speed'],
                "heading_deg": n['heading'],
                "emergency_status": n['status'],
            }
            for n in neighbors
        ]
    }

    # 6. Send to Flutter
    conn.sendall((json.dumps(frame) + '\n').encode('utf-8'))

    # 7. Throttle to 30 Hz
    time.sleep(1/30)
```

## Error handling

| Scenario | Pi behavior | Flutter behavior |
|----------|-------------|------------------|
| Connection lost | Re-listen for client | Auto-retry connect, show DISCONNECTED badge |
| Malformed JSON | Skip line | Skip & log error, tidak crash |
| Missing required field | Tidak boleh terjadi (validate sebelum kirim) | Skip frame & log |
| Neighbor `id` berubah-ubah | Hindari (tracking jadi reset) | Anggap mobil baru |

## Recording mode (TAHAP 1 — sekarang ini)

**Untuk thesis defense yang efisien**, alur kerja yang disepakati:

1. **Tim hardware** run sensor test + Pi processing, **record output ke FILE** (bukan kirim live)
2. File di-share ke tim Flutter
3. Flutter playback file → verifikasi UI sesuai skenario

### Format file

**JSONL (JSON Lines)** — satu V2VFrame per baris, format persis sama dengan wire format di section sebelumnya.

```
{"ts":1748534400000,"ego":{...},"neighbors":[...]}
{"ts":1748534400100,"ego":{...},"neighbors":[...]}
{"ts":1748534400200,"ego":{...},"neighbors":[...]}
...
```

### Naming convention

```
recordings/<tanggal>_<skenario>_<run>.jsonl

Contoh:
  recordings/2026-06-08_persimpangan_run1.jsonl
  recordings/2026-06-08_remmemdadak_run2.jsonl
  recordings/2026-06-09_overtaking_run1.jsonl
```

### Workflow

```
1. Tim hardware: setup sensor + Pi
2. Tim hardware: jalanin skenario (mobil A jalan, mobil B nyalip kiri, dst)
3. Pi: jalanin recorder script — tulis tiap V2VFrame ke file .jsonl
4. Tim hardware: kasih file ke tim Flutter (USB/cloud)
5. Tim Flutter: taruh file di assets/recordings/ project
6. Tim Flutter: register path di pubspec.yaml > flutter > assets
7. Tim Flutter: ganti DataSource di home_screen.dart ke JsonlFileDataSource
8. Run Flutter — file playback realtime sesuai timestamp asli
```

### Contoh Python recorder (di Pi)

```python
import json
import time

# Buka file untuk record
recording_file = open(f'recordings/{scenario_name}.jsonl', 'w')

while recording:
    # 1. Read sensor data
    raw_frame = read_from_mcu()

    # 2. Run UKF & Neighbour Track (sama seperti live mode)
    ego_state = run_ukf(raw_frame)
    neighbors = decode_lora_and_track()

    # 3. Build JSON frame (skema sama dengan wire format)
    frame = {
        "ts": int(time.time() * 1000),
        "ego": {
            "lat": ego_state['lat'],
            "lon": ego_state['lon'],
            "speed_kmh": ego_state['speed'],
            "heading_deg": ego_state['heading'],
            "engine_rpm": raw_frame['rpm'],
            "engine_temp_c": raw_frame['temp'],
        },
        "neighbors": [...],
    }

    # 4. Tulis ke file
    recording_file.write(json.dumps(frame) + '\n')
    recording_file.flush()  # penting: pastikan masuk disk

    time.sleep(1/30)  # 30 Hz

recording_file.close()
```

### Best practices recording

- **Flush sering** — biar kalau Pi crash, data sebelumnya gak hilang
- **Skenario pendek** — 30-60 detik per file (mudah debug)
- **Beri label jelas** — naming convention di atas
- **Record beberapa kondisi** — siang/malam, mobil pelan/cepat, neighbor banyak/sedikit
- **Sertakan catatan skenario** — bikin file `recordings/test_log.md` yang catat apa yang terjadi tiap run

## Live mode (TAHAP 2 — nanti)

Setelah recording mode jalan dan UI terverifikasi, langkah berikutnya adalah live streaming. Format wire datanya sudah disepakati di section sebelumnya (TCP socket `127.0.0.1:5555`, JSON line-delimited). Pi cuma perlu tambahin TCP server pararel sama recorder — tidak ubah format data.

## Change log

| Date | Author | Change |
|------|--------|--------|
| 2026-06-XX | Ghefira | Initial draft sesuai diagram arsitektur sistem |

## Open questions

- Apakah Pi kirim raw IMU data juga (untuk display orientation) atau cukup heading saja? **Cukup heading.**
- Format binary lebih cepat dari JSON?  **Untuk thesis, JSON cukup. Optimasi nanti.**
- Bagaimana handle multiple Flutter app connect ke Pi yang sama? **Single client untuk sekarang.**
