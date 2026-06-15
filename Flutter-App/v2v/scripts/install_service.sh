#!/bin/bash
# ============================================================
# install_service.sh — Install systemd autostart di Pi
# ------------------------------------------------------------
# Jalankan SEKALI di Pi setelah bundle pertama kali ter-deploy:
#   chmod +x install_service.sh
#   ./install_service.sh
#
# Apa yang dilakukan:
#   1. Copy v2v-app.service ke /etc/systemd/system/
#   2. Daemon-reload
#   3. Enable autostart saat boot
#   4. Start service sekarang
#
# Catatan: file service akan auto-edit "pi" jadi $USER kalau berbeda.
# ============================================================

set -euo pipefail

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[install-service]${NC} $*"; }
warn() { echo -e "${YELLOW}[install-service]${NC} $*"; }
err()  { echo -e "${RED}[install-service]${NC} $*"; }

# ---- Sanity check ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_FILE="$SCRIPT_DIR/v2v-app.service"

if [ ! -f "$SERVICE_FILE" ]; then
    err "v2v-app.service tidak ditemukan di $SCRIPT_DIR"
    exit 1
fi

if [ "$EUID" -eq 0 ]; then
    err "Jangan jalankan sebagai root. Pakai user biasa, sudo dipanggil saat perlu."
    exit 1
fi

# ---- Pastikan flutter-pi terinstall ----
if ! command -v flutter-pi &> /dev/null; then
    err "flutter-pi tidak terinstall. Jalankan dulu: ./setup_pi.sh"
    exit 1
fi

# ---- Pastikan bundle ada ----
BUNDLE_PATH="/home/$USER/v2v_app"
if [ ! -f "$BUNDLE_PATH/kernel_blob.bin" ] && [ ! -f "$BUNDLE_PATH/isolate_snapshot_data" ]; then
    warn "Bundle Flutter belum ada di $BUNDLE_PATH"
    warn "  Jalankan dulu di laptop dev: ./scripts/deploy.sh"
    read -p "  Lanjutkan install service tetap? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# ---- Patch service file kalau username bukan "pi" ----
TMP_SERVICE=$(mktemp)
if [ "$USER" != "pi" ]; then
    log "Username Anda '$USER' (bukan 'pi'), patch service file..."
    sed -e "s|User=pi|User=$USER|g" \
        -e "s|Group=pi|Group=$USER|g" \
        -e "s|/home/pi/v2v_app|/home/$USER/v2v_app|g" \
        "$SERVICE_FILE" > "$TMP_SERVICE"
else
    cp "$SERVICE_FILE" "$TMP_SERVICE"
fi

# ---- Install ----
log "Copy service file ke /etc/systemd/system/v2v-app.service..."
sudo cp "$TMP_SERVICE" /etc/systemd/system/v2v-app.service
rm "$TMP_SERVICE"

log "Daemon-reload..."
sudo systemctl daemon-reload

log "Enable autostart saat boot..."
sudo systemctl enable v2v-app.service

log "Start service sekarang..."
sudo systemctl start v2v-app.service

sleep 2
echo ""
log "Status service:"
sudo systemctl status v2v-app.service --no-pager || true

echo ""
log "✅ Service installed!"
echo ""
echo "  Useful commands:"
echo "    sudo systemctl status v2v-app          # cek status"
echo "    sudo systemctl restart v2v-app         # restart manual"
echo "    sudo systemctl stop v2v-app            # stop"
echo "    sudo systemctl disable v2v-app         # disable autostart"
echo "    journalctl -u v2v-app -f               # tail logs realtime"
echo "    journalctl -u v2v-app --since '5 min ago'  # logs 5 menit terakhir"
echo ""
echo "  Setelah ini, Pi akan auto-start V2V app setiap boot."
echo ""
