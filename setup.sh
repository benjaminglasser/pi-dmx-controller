#!/usr/bin/env bash
set -euo pipefail

# ====== Config ======
PROJ_DIR="/home/pi/dmx_project"
SERVICE_NAME="dmx_audio_react.service"
PYTHON="/usr/bin/python3"

echo ">>> Updating apt and installing packages…"
sudo apt-get update
sudo apt-get install -y \
  git curl ca-certificates \
  ola ola-python \
  python3-pip python3-venv python3-rpi.gpio python3-spidev \
  portaudio19-dev alsa-utils \
  wiringpi build-essential

echo ">>> Ensuring SPI enabled…"
# (SPI overlay normally enabled by dtparam=spi=on below)

FIRMWARE_CFG="/boot/firmware/config.txt"
SNIPPET_FILE="${PROJ_DIR}/config/firmware-config.snippet.txt"
if ! grep -q "### DMX AUDIO REACT START" "$FIRMWARE_CFG"; then
  echo ">>> Appending firmware config snippet to ${FIRMWARE_CFG}"
  sudo tee -a "$FIRMWARE_CFG" >/dev/null <<'EOF'

### DMX AUDIO REACT START
# Disable onboard analog audio and enable HiFiBerry DAC+ADC overlay
dtparam=audio=off
dtoverlay=hifiberry-dacplusadc
# Enable SPI (MCP3008)
dtparam=spi=on
### DMX AUDIO REACT END
EOF
else
  echo ">>> Firmware snippet already present."
fi

echo ">>> Creating project venv + installing Python deps…"
mkdir -p "$PROJ_DIR"
cd "$PROJ_DIR"

# Create venv if missing
if [ ! -d "venv" ]; then
  $PYTHON -m venv venv
fi

# Upgrade pip + install
source venv/bin/activate
pip install --upgrade pip
# requirements.txt is minimal; we install sounddevice & numpy here to ensure PortAudio wheels match
pip install -r requirements.txt || true
pip install numpy sounddevice

echo ">>> Writing Systemd service…"
sudo mkdir -p /etc/systemd/system
sudo cp "${PROJ_DIR}/systemd/${SERVICE_NAME}" "/etc/systemd/system/${SERVICE_NAME}"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"

# Optional: Stop Pulse/pipewire if you want ALSA direct capture by default.
# Uncomment if needed on a fresh image where pipewire grabs the device:
# systemctl --user stop pipewire pipewire-pulse wireplumber || true
# systemctl --user disable pipewire pipewire-pulse wireplumber || true

echo ">>> Verifying OLA runs…"
sudo systemctl enable olad
sudo systemctl start olad

echo ">>> Setup complete."
echo "Reboot recommended now: sudo reboot"