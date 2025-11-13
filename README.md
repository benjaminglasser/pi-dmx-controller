# DMX AudionReactive Light Controller (Pi + HiFiBerry + OLA)
## Overview
This project transforms a Raspberry Pi into a fully standalone, audionreactive DMX l
ighting engine.
It uses:
- **HiFiBerry DAC+ADC** for clean audio input
- **MCP3008** for six analog knobs
- **Rotary switch** for preset selection
- **Custom OLED boot splash pipeline** (supernfast, firmwarenlevel + systemd earlyn
boot)
- **OLA (Open Lighting Architecture)** for DMX output
This README includes:
- Original documentation
- All OLED improvements (splash, animation, earlynboot service)
- All bootnspeed optimizations
- All systemd service definitions
- Full setup + troubleshooting
---
## FEATURES
- Six knobs:
- Center Frequency
- Q
- Threshold
- Attack
- Decay
- Brightness
- Four program modes (rotary switch):
- All
- Chase
- Random
- Ambient
- Reset button restores defaults
- Ready LED (GPIO17)
- OLED 128×32 UI
- Fast boot + logo splash at ~2 seconds
- Autonstart on boot (systemd)
- Optional TUI mode
- Raspberry Pi OS Bookworm tested
---
## HARDWARE
| Component | Purpose |
|----------|----------|
| Raspberry Pi 4 / 5 | Core computer |
| HiFiBerry DAC+ADC | Audio I/O |
| MCP3008 | Analog input |
| 6x 10k Pots | Knobs |
| 4nway Rotary | Program selector |
| Push Button | Reset |
| LED + 330W | Ready indicator |
| DMXKing UltraDMX Pro | DMX interface |
| DMX fixture | Output |
---
## WIRING (BCM numbering)
### MCP3008 (SPI0 CE0)
```
VDD, VREF ® 3.3 V
AGND, DGND ® GND
CLK ® BCM11
MOSI ® BCM10
MISO ® BCM9
CE0 ® BCM8
CH0–CH5 ® Potentiometers
```
### ROTARY SWITCH
```
BCM21, BCM22, BCM23, BCM24 (all with internal pullnups)
Truth table:
(1,1,1,1) ® Program 1
(1,1,0,0) ® Program 2
(1,0,1,0) ® Program 3
(0,1,1,0) ® Program 4
```
### RESET BUTTON
```
BCM25 ® Button ® GND
```
### READY LED
```
BCM17 ® 330W ® LED ® GND
```
---
## DEFAULT STARTUP VALUES
```
Center: 120 Hz
Q: 1.7
Threshold: 0.032
Attack: 10 ms
Decay: 50 ms
Brightness: 1.0
```
---
# INSTALLATION GUIDE
Installation Guide

This guide is optimized for rebuilding a **new SD card** as quickly and repeatably as possible.

---
## 0. What the bootstrap script does

`scripts/bootstrap_pi.sh` will:

1. Update and upgrade the OS packages.
2. Install core dependencies (Python, PortAudio, ALSA, OLA, etc.).
3. Enable **SPI** and **I²C** non‑interactively.
4. Configure **HiFiBerry DAC+ADC** in `/boot/firmware/config.txt` (disables onboard audio and enables I2S).
5. Create a Python **virtual environment** (`.venv`) with `--system-site-packages` and install `requirements.txt`.
6. Enable and start the **OLA daemon** and patch **DMXking port 0 → Universe 0**.
7. (Optionally, if you’ve added unit files) install and enable the **systemd services** for:
   - `oled_splash.service` (early boot OLED splash)
   - `pi-dmx.service` (headless main app)

After running it, a reboot is strongly recommended so overlays and services come up cleanly.

---
## 1. Fresh Install on a Clean Pi

1. Flash **Raspberry Pi OS Bookworm (32‑bit)** with Raspberry Pi Imager.
2. Enable SSH in Imager or via `raspi-config` on first boot.
3. Boot the Pi, get it on your network, then SSH in and run:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git
sudo reboot
```

After the reboot, SSH back in.

---
## 2. Clone the Repo & Run Bootstrap

From your home directory on the Pi:

```bash
cd ~
git clone https://github.com/benjaminglasser/pi-dmx-controller.git
cd ~/pi-dmx-controller
bash scripts/bootstrap_pi.sh
```

Watch the script output; it will:

- Install required packages (`python3`, `python3-venv`, `python3-pip`, `alsa-utils`, `libportaudio2`, `portaudio19-dev`, `libsndfile1`, `ola`, `ola-python`).
- Enable **SPI** and **I²C**.
- Edit `/boot/firmware/config.txt` to:
  - Comment out `dtparam=audio=on`
  - Add `dtparam=audio=off`
  - Add `dtoverlay=hifiberry-dacplusadc`
  - Ensure `dtparam=i2s=on` is present
- Create `.venv` and install `requirements.txt`.
- Enable and restart `olad`, then patch DMXking → Universe 0.

When it finishes you’ll see a message like:

```text
[7/7] Reboot recommended for overlays
Bootstrap complete. Run: sudo reboot
```

Now reboot:

```bash
sudo reboot
```

---
## 3. (Optional) Manual Python Environment Setup

Normally the bootstrap script creates the virtual environment for you.  
If you ever want to recreate it manually:

```bash
cd ~/pi-dmx-controller
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

Use this if you’ve changed `requirements.txt` or want to rebuild the venv from scratch.

---
## 4. Install & Enable Services (Headless + OLED Splash)

If you haven’t wired service setup into `bootstrap_pi.sh`, you can install the units manually:

```bash
cd ~/pi-dmx-controller

# Copy unit files into systemd
sudo cp systemd/pi-dmx.service /etc/systemd/system/pi-dmx.service
sudo cp systemd/oled_splash.service /etc/systemd/system/oled_splash.service

# Reload systemd units
sudo systemctl daemon-reload

# Enable on boot
sudo systemctl enable oled_splash.service
sudo systemctl enable pi-dmx.service

# Start immediately (optional)
sudo systemctl start oled_splash.service
sudo systemctl start pi-dmx.service
```

On the next reboot you should see:

1. Pi firmware / boot text
2. Your **logo** on the OLED (from `utils/oled_boot.py`)
3. A neutral **“Starting…”** screen with a subtle pulse dot
4. The main **DMX controller UI** once `dmx_audio_react.py` is running

If you do wire this into `bootstrap_pi.sh`, make sure this section of the README stays as documentation of what the script is doing under the hood.

---
## 5. Verify OLA & DMXKing

Sanity‑check that OLA sees the interface and that Universe 0 is patched:

```bash
sudo systemctl start olad
sleep 2
ola_dev_info | grep -A2 DMXking
```

You want to see something like:

```text
Device 10: DMXking.com – UltraDMX2 PRO
  port 0, OUT, patched to universe 0
```

If the universe isn’t patched, you can re‑run the patch manually:

```bash
# Find the device ID (DEV_ID)
ola_dev_info | awk '/DMXking/{print id}{id=$2}'

# Then patch it (example assumes DEV_ID=10)
sudo ola_patch -d 10 -p 0 -u 0
```

---
## 6. Quick DMX Test

With your Chauvet DMX‑4 (or similar) set to **address 1**, you can send a test frame:

```bash
ola_streaming_client --universe 0 --dmx 255,0,0,0,0,0,0,0
```

Channel 1 should turn on.

Turn everything off:

```bash
ola_streaming_client --universe 0 --dmx 0,0,0,0,0,0,0,0
```

If this works, the OLA + DMX chain is good.

---
## 7. TUI Mode (Interactive Debugging)

If the headless service is running, stop it first:

```bash
sudo systemctl stop pi-dmx
```

Then run the controller in TUI mode:

```bash
cd ~/pi-dmx-controller
source .venv/bin/activate
python dmx_audio_react.py
```

- Press **q** (or **ESC**) to quit the TUI.
- Press the physical **RESET** button (BCM25) to restore default parameters.
- Adjust knobs + program switch in real time and watch the terminal output.

Return to headless mode:

```bash
sudo systemctl start pi-dmx
sudo systemctl status pi-dmx
```

---
## 8. New SD Card / “Disaster Recovery” Checklist

For Future You when an SD card dies or you want a second identical card:

```bash
# 1. On a fresh Pi:
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git

# 2. Clone the repo:
cd ~
git clone https://github.com/benjaminglasser/pi-dmx-controller.git
cd ~/pi-dmx-controller

# 3. Run bootstrap:
bash scripts/bootstrap_pi.sh

# 4. Install & enable services (if not in the script yet):
sudo cp systemd/pi-dmx.service /etc/systemd/system/pi-dmx.service
sudo cp systemd/oled_splash.service /etc/systemd/system/oled_splash.service
sudo systemctl daemon-reload
sudo systemctl enable oled_splash.service pi-dmx.service

# 5. Reboot:
sudo reboot
```

After that reboot, your DMX audio‑reactive controller should be fully live:
- OLED logo & neutral start screen
- Main OLED UI
- DMX output reacting to audio

---
# FULL OLED SYSTEM
## 1. EarlynBoot OLED Splash (`oled_boot.service`)
Placed in `/etc/systemd/system/oled_boot.service`:
```
[Unit]
Description=OLED Early Boot Logo
DefaultDependencies=no
After=dev-i2c-1.device
Before=sysinit.target
[Service]
Type=simple
ExecStart=/usr/bin/python3 /home/pi/pi-dmx-controller/utils/oled_boot.py
TimeoutSec=8
[Install]
WantedBy=sysinit.target
```
Enable:
```
sudo systemctl daemon-reload
sudo systemctl enable oled_boot
```
---
## 2. FastnBoot Splash Script `oled_boot.py`
This includes:
- Very fast I²C acquisition
- Logo display
- Animated “Starting…” with pulse dots
- Automatic exit when main service overwrites OLED
**Full script included in repo** (not truncated).
---
## 3. BootnSpeed Optimizations
We applied:
### 4 Removed kernel framebuffer splash
In `/boot/firmware/cmdline.txt` ® **removed**:
```
splash quiet plymouth.ignore-serial-consoles
```
### 4 Avoid systemd waiting for WiFi
```
sudo systemctl disable NetworkManager-wait-online.service
```
### 4 Disable unused services (optional)
```
sudo systemctl disable avahi-daemon
sudo systemctl disable bluetooth
```
### 4 Confirm fast device availability
No more dependency delays from:
```
dev-i2c-1.device
systemd-udev-settle.service
```
### RESULT
Boot to OLED logo: **~2 seconds**
Main app running: **~8–10 seconds**
---
# MAIN APP AUTOSTART 
## `pi-dmx.service`
Located at `/etc/systemd/system/pi-dmx.service`:
```
[Unit]
Description=Pi DMX Controller
After=network-online.target sound.target olad.service
Wants=network-online.target olad.service
[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/pi-dmx-controller
Environment=PYTHONUNBUFFERED=1
ExecStart=/home/pi/pi-dmx-controller/.venv/bin/python3 /home/pi/pi-dmx-controller/dm
x_audio_react.py
Restart=on-failure
RestartSec=2
[Install]
WantedBy=multi-user.target
```

---
# TROUBLESHOOTING
| Problem | Fix |
|--------|-----|
| No OLED at boot | Check `oled_boot.service` |
| Slow boot | Disable waitnonline |
| No DMX | Check `ola_dev_info` |
| No audio | Check HiFiBerry overlay |
| Knobs jitter | Ensure solid wiring |
---

# LICENSE
MIT © 2025 Ben Glasser