#!/usr/bin/env bash
set -euo pipefail

echo "[1/7] Update system"
sudo apt update
sudo apt -y upgrade

echo "[2/7] Core packages"
sudo apt install -y git python3 python3-venv python3-pip \
  alsa-utils libportaudio2 portaudio19-dev libsndfile1 \
  ola ola-python

echo "[3/7] Enable SPI/I2C (non-interactive)"
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_i2c 0

echo "[4/7] HiFiBerry overlay (ALSA) in /boot/firmware/config.txt"
CFG=/boot/firmware/config.txt
sudo sed -i 's/^dtparam=audio=on/# dtparam=audio=on/' "$CFG" || true
grep -q '^dtparam=audio=off' "$CFG" || echo 'dtparam=audio=off' | sudo tee -a "$CFG"
grep -q '^dtoverlay=hifiberry-dacplusadc' "$CFG" || echo 'dtoverlay=hifiberry-dacplusadc' | sudo tee -a "$CFG"
# Enable i2s for HiFiBerry if commented
grep -q '^dtparam=i2s=on' "$CFG" || sed -i 's/^#dtparam=i2s=on/dtparam=i2s=on/' "$CFG" || echo 'dtparam=i2s=on' | sudo tee -a "$CFG"

echo "[5/7] Python venv (with system packages so OLA Python is visible)"
cd ~/pi-dmx-controller
rm -rf .venv
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo "[6/7] OLA daemon + patch Universe 0 to DMXKing port 0"
sudo systemctl enable olad
sudo systemctl restart olad
sleep 2
DEV_ID=$(ola_dev_info | awk '/DMXking/{print id}{id=$2}')
sudo ola_patch -d "${DEV_ID:-10}" -p 0 -u 0 || true

echo "[7/7] Reboot recommended for overlays"
echo "Bootstrap complete. Run: sudo reboot"
