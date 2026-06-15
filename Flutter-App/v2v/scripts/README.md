# Deployment V2V App ke Raspberry Pi

Panduan deploy Flutter app V2V ke Raspberry Pi pakai **flutter-pi** (embedder ringan, no desktop environment).

## Arsitektur deployment

```
┌─────────────────────┐         ┌──────────────────────┐
│   Laptop Dev        │         │   Raspberry Pi       │
│                     │         │                      │
│  flutter build      │         │  flutter-pi          │
│  bundle             │ ───►    │  (autostart via      │
│                     │ rsync   │   systemd)           │
│  scripts/deploy.sh  │ ssh     │                      │
└─────────────────────┘         │  Layar HDMI / DSI    │
                                └──────────────────────┘
```

Bundle (asset Flutter terkompilasi) di-build sekali di laptop dev, lalu di-rsync ke Pi. Pi run binary `flutter-pi` yang load bundle tersebut dan render fullscreen langsung di GPU tanpa desktop environment.

## Setup pertama kali (one-time)

### A. Di Raspberry Pi

1. **Login ke Pi** (via SSH atau langsung):
   ```bash
   ssh pi@raspberrypi.local
   ```

2. **Cek OS** — pastikan Raspberry Pi OS Bookworm, idealnya 64-bit:
   ```bash
   cat /etc/os-release
   uname -m
   ```

3. **Copy script `setup_pi.sh` ke Pi** (sementara, sebelum app di-deploy):
   ```bash
   # Di laptop dev:
   scp scripts/setup_pi.sh pi@raspberrypi.local:~/
   ```

4. **Run setup di Pi**:
   ```bash
   ssh pi@raspberrypi.local
   chmod +x setup_pi.sh
   ./setup_pi.sh
   ```
   Akan install dependencies, compile flutter-pi (~5-10 menit di Pi 4). Jawab `y` saat ditanya upgrade.

5. **Reboot Pi**:
   ```bash
   sudo reboot
   ```

### B. Di laptop dev

1. **Pastikan Flutter SDK terinstall**:
   ```bash
   flutter --version
   ```

2. **Setup SSH key ke Pi** (supaya tidak perlu password tiap deploy):
   ```bash
   ssh-keygen        # kalau belum ada
   ssh-copy-id pi@raspberrypi.local
   ```

3. **Make script executable**:
   ```bash
   chmod +x scripts/deploy.sh
   ```

## Deploy harian

Setiap kali Anda update kode dan mau push ke Pi:

```bash
./scripts/deploy.sh
```

Script akan:
1. Build Flutter bundle (debug mode)
2. Rsync hasil ke Pi
3. Restart service kalau sudah di-install

**Override config via env var:**

```bash
# Pi dengan IP statis
PI_HOST=192.168.1.42 ./scripts/deploy.sh

# Cuma update bundle, tidak rebuild
SKIP_BUILD=1 ./scripts/deploy.sh

# Username Pi bukan "pi"
PI_USER=ghefira ./scripts/deploy.sh
```

## Install autostart (one-time, setelah deploy pertama)

Supaya app auto-jalan saat Pi boot:

```bash
# Copy install script ke Pi
scp scripts/install_service.sh scripts/v2v-app.service pi@raspberrypi.local:~/

# Run di Pi
ssh pi@raspberrypi.local
chmod +x install_service.sh
./install_service.sh
```

Setelah ini, **setiap Pi reboot, app V2V langsung muncul fullscreen di layar**.

## Operasi sehari-hari

```bash
# Cek status service
ssh pi@raspberrypi.local 'sudo systemctl status v2v-app'

# Lihat log realtime
ssh pi@raspberrypi.local 'journalctl -u v2v-app -f'

# Restart manual
ssh pi@raspberrypi.local 'sudo systemctl restart v2v-app'

# Stop sementara
ssh pi@raspberrypi.local 'sudo systemctl stop v2v-app'

# Disable autostart (kalau mau debug manual)
ssh pi@raspberrypi.local 'sudo systemctl disable v2v-app'
```

## Run manual (untuk debug)

Kalau mau lihat error/output secara detail:

```bash
ssh pi@raspberrypi.local
sudo systemctl stop v2v-app    # stop service dulu
flutter-pi ~/v2v_app           # run langsung, lihat output
```

Quit dengan `Ctrl+C` atau `q`.

## Troubleshooting

**❌ "Cannot SSH to Pi"**
- Cek Pi nyala & terhubung WiFi/Ethernet
- Coba `ping raspberrypi.local`
- Cek IP via router atau `arp -a`
- Pakai IP langsung: `PI_HOST=192.168.x.x`

**❌ "flutter-pi: command not found"**
- Setup belum jalan. Run `./setup_pi.sh` di Pi.

**❌ "Permission denied: /dev/dri/card0"**
- User belum di group video/render. Run di Pi:
  ```bash
  sudo usermod -a -G video,render,input $USER
  sudo reboot
  ```

**❌ "Failed to create EGL display"**
- GPU driver belum aktif. Edit `/boot/firmware/config.txt`:
  ```ini
  dtoverlay=vc4-kms-v3d
  gpu_mem=256
  ```
  Lalu reboot.

**❌ "Layar blank / hitam"**
- Pastikan HDMI terhubung sebelum Pi boot
- Force resolution di `/boot/firmware/config.txt`:
  ```ini
  hdmi_force_hotplug=1
  hdmi_group=2
  hdmi_mode=82      # 1920x1080
  ```

**❌ "App lag / FPS rendah"**
- Cek CPU/RAM: `htop` di Pi
- Cek temperature: `vcgencmd measure_temp` (jangan >80°C)
- Pasang heatsink, atau pakai Pi 5
- Disable animasi gauge yang berat (edit `sensor_view.dart`)

**❌ "Service crash terus restart"**
- Cek log: `journalctl -u v2v-app --since '5 min ago'`
- Bundle mungkin rusak — re-deploy: `./scripts/deploy.sh`

## Rotate display (kalau orientasi salah)

7-inch DSI/HDMI screen kadang portrait sementara dashboard butuh landscape. Edit `/boot/firmware/config.txt`:

```ini
# Rotasi 90/180/270 derajat
display_rotate=1    # 90° clockwise
# display_rotate=2  # 180°
# display_rotate=3  # 270°
```

Reboot setelah edit.

## Update workflow harian

Workflow umum setelah semua setup selesai:

1. Edit kode di laptop
2. `./scripts/deploy.sh`
3. Tunggu ~30 detik (build + rsync + restart)
4. Lihat hasil di layar Pi
5. Cek log kalau ada error: `ssh pi@raspberrypi.local 'journalctl -u v2v-app -f'`

## Catatan release vs debug build

Default `deploy.sh` pakai **debug build** — gampang setup, support hot reload, tapi lebih lambat (JIT). Cocok untuk development & test performance baseline.

**Release build** lebih cepat (~2-3x), tapi butuh AOT compile yang lebih kompleks: butuh `gen_snapshot` cross-compiled untuk ARM. Lihat https://github.com/ardera/flutter-pi#building-the-app untuk panduan release build. Saya bisa bantu setup ini saat siap untuk production deployment.
