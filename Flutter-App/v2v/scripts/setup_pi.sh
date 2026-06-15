#!/bin/bash
# ============================================================
# setup_pi.sh — Install flutter-pi di Raspberry Pi
# ------------------------------------------------------------
# Jalankan SEKALI di Pi (langsung di Pi atau via SSH):
#   chmod +x setup_pi.sh
#   ./setup_pi.sh
#
# Apa yang dilakukan:
#   1. Update OS
#   2. Install build dependencies untuk flutter-pi
#   3. Clone & compile flutter-pi
#   4. Install binary ke /usr/local/bin/flutter-pi
#   5. Add user ke group video, render, input, dialout
# ============================================================

set -euo pipefail

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[setup-pi]${NC} $*"; }
warn() { echo -e "${YELLOW}[setup-pi]${NC} $*"; }
err()  { echo -e "${RED}[setup-pi]${NC} $*"; }

# ============================================================
# Sanity check
# ============================================================
if [[ "$EUID" -eq 0 ]]; then
    err "Jangan jalankan sebagai root. Pakai user biasa, akan minta sudo saat perlu."
    exit 1
fi

log "Cek info Pi..."
uname -a
cat /etc/os-release | grep -E '^(NAME|VERSION)='
echo ""

# ============================================================
# 1. Update OS
# ============================================================
log "Update package list..."
sudo apt update

log "Upgrade packages (boleh skip kalau sudah baru)..."
read -p "Upgrade semua package? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo apt full-upgrade -y
fi

# ============================================================
# 2. Install build dependencies
# ============================================================
log "Install build dependencies untuk flutter-pi..."
sudo apt install -y \
    git cmake pkg-config build-essential \
    libgl1-mesa-dev libgles2-mesa-dev libegl1-mesa-dev \
    libdrm-dev libgbm-dev \
    fontconfig libsystemd-dev libinput-dev libudev-dev libxkbcommon-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav

# ============================================================
# 3. Clone & build flutter-pi
# ============================================================
FPI_DIR="$HOME/flutter-pi"

if [ -d "$FPI_DIR" ]; then
    warn "Folder $FPI_DIR sudah ada, skip clone (pull update saja)."
    cd "$FPI_DIR"
    git pull --recurse-submodules
else
    log "Clone flutter-pi..."
    git clone --recursive https://github.com/ardera/flutter-pi.git "$FPI_DIR"
    cd "$FPI_DIR"
fi

log "Build flutter-pi (akan agak lama, ~5-10 menit di Pi 4)..."
mkdir -p build && cd build
cmake ..
make -j"$(nproc)"

log "Install flutter-pi binary ke /usr/local/bin..."
sudo make install

# ============================================================
# 4. Setup user permissions
# ============================================================
log "Add user '$USER' ke group video, render, input, dialout..."
sudo usermod -a -G video,render,input,dialout "$USER"

# ============================================================
# 5. GPU memory split (Pi 4/5)
# ============================================================
log "Cek GPU memory split..."
GPU_MEM=$(vcgencmd get_mem gpu 2>/dev/null | grep -oE '[0-9]+' || echo "unknown")
log "  Current GPU memory: ${GPU_MEM}M"
if [ "$GPU_MEM" != "unknown" ] && [ "$GPU_MEM" -lt 256 ]; then
    warn "GPU memory <256M. Saran: naikkan via 'sudo raspi-config' → Performance → GPU Memory"
fi

# ============================================================
# Done
# ============================================================
echo ""
log "✅ Setup selesai!"
echo ""
echo "  Verify flutter-pi terinstall:"
echo "    which flutter-pi"
echo "    flutter-pi --help"
echo ""
echo "  Langkah berikutnya:"
echo "    1. REBOOT Pi: sudo reboot"
echo "       (penting agar group permissions aktif)"
echo "    2. Di laptop dev: jalankan ./scripts/deploy.sh"
echo ""
