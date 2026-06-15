#!/bin/bash
# ============================================================
# deploy.sh — Build Flutter bundle & push ke Raspberry Pi
# ------------------------------------------------------------
# Jalankan di LAPTOP DEV (bukan di Pi):
#   chmod +x scripts/deploy.sh
#   ./scripts/deploy.sh
#
# Konfigurasi via environment variables (atau edit default di bawah):
#   PI_USER  - username di Pi          (default: pi)
#   PI_HOST  - hostname/IP Pi          (default: raspberrypi.local)
#   PI_PATH  - path tujuan di Pi       (default: ~/v2v_app)
#   SKIP_BUILD - kalau "1", skip build, langsung rsync
#   SKIP_RESTART - kalau "1", tidak restart service
#
# Contoh:
#   PI_HOST=192.168.1.42 ./scripts/deploy.sh
#   SKIP_BUILD=1 ./scripts/deploy.sh        # cuma update bundle
#
# PRASYARAT:
#   - Flutter SDK terinstall di laptop dev
#   - SSH key sudah copy ke Pi:
#       ssh-copy-id $PI_USER@$PI_HOST
#   - flutter-pi sudah terinstall di Pi (jalankan setup_pi.sh)
# ============================================================

set -euo pipefail

# ---- Config ----
PI_USER="${PI_USER:-pi}"
PI_HOST="${PI_HOST:-raspberrypi.local}"
PI_PATH="${PI_PATH:-/home/$PI_USER/v2v_app}"
SKIP_BUILD="${SKIP_BUILD:-0}"
SKIP_RESTART="${SKIP_RESTART:-0}"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[deploy]${NC} $*"; }
warn() { echo -e "${YELLOW}[deploy]${NC} $*"; }
err()  { echo -e "${RED}[deploy]${NC} $*"; }

# ---- Sanity check ----
cd "$(dirname "$0")/.."  # ke project root

if [ ! -f "pubspec.yaml" ]; then
    err "pubspec.yaml tidak ditemukan. Jalankan dari project root."
    exit 1
fi

if ! command -v flutter &> /dev/null; then
    err "Flutter SDK tidak terinstall di PATH."
    exit 1
fi

# ---- Cek koneksi ke Pi ----
log "Cek koneksi ke $PI_USER@$PI_HOST..."
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$PI_USER@$PI_HOST" "echo connected" 2>/dev/null; then
    err "Tidak bisa SSH ke $PI_USER@$PI_HOST."
    err "  - Pastikan Pi nyala & terhubung jaringan"
    err "  - Cek hostname: ping $PI_HOST"
    err "  - Setup SSH key: ssh-copy-id $PI_USER@$PI_HOST"
    exit 1
fi
log "  ✅ SSH OK"

# ---- Build ----
if [ "$SKIP_BUILD" != "1" ]; then
    log "flutter pub get..."
    flutter pub get

    log "flutter build bundle (debug mode untuk dev/test)..."
    flutter build bundle
    log "  ✅ Bundle ada di build/flutter_assets/"
else
    warn "SKIP_BUILD=1 → skip build, langsung rsync"
fi

if [ ! -d "build/flutter_assets" ]; then
    err "build/flutter_assets/ tidak ada. Jalankan dengan SKIP_BUILD=0."
    exit 1
fi

# ---- Pastikan target dir ada ----
log "Pastikan $PI_PATH ada di Pi..."
ssh "$PI_USER@$PI_HOST" "mkdir -p $PI_PATH"

# ---- Rsync ----
log "Rsync ke $PI_USER@$PI_HOST:$PI_PATH..."
rsync -avh --delete --progress \
    build/flutter_assets/ \
    "$PI_USER@$PI_HOST:$PI_PATH/"
log "  ✅ Bundle deployed"

# ---- Restart service ----
if [ "$SKIP_RESTART" != "1" ]; then
    log "Restart v2v-app service di Pi..."
    if ssh "$PI_USER@$PI_HOST" "systemctl is-enabled v2v-app.service" 2>/dev/null; then
        ssh "$PI_USER@$PI_HOST" "sudo systemctl restart v2v-app.service"
        log "  ✅ Service restarted"
    else
        warn "v2v-app.service belum di-install."
        warn "  Jalankan di Pi: ./scripts/install_service.sh"
        warn "  Atau manual: flutter-pi $PI_PATH"
    fi
else
    warn "SKIP_RESTART=1 → tidak restart service"
fi

echo ""
log "✅ Deploy selesai!"
echo ""
echo "  Cek log di Pi:"
echo "    ssh $PI_USER@$PI_HOST 'journalctl -u v2v-app.service -f'"
echo ""
echo "  Atau run manual untuk debug:"
echo "    ssh $PI_USER@$PI_HOST 'flutter-pi $PI_PATH'"
echo ""
