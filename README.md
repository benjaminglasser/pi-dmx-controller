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
# FULL OLED SYSTEM (UPDATED)
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
## 2. Updated FastnBoot Splash Script `oled_boot.py`
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
# MAIN APP AUTOSTART (UPDATED)
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
# INSTALLATION GUIDE
## 1. Fresh Install
```
sudo apt update && sudo apt full-upgrade -y
sudo apt install python3 python3-pip python3-venv ola git
sudo reboot
```
Clone + bootstrap:
```
git clone https://github.com/benjaminglasser/pi-dmx-controller.git
cd ~/pi-dmx-controller
bash scripts/bootstrap_pi.sh
sudo reboot
```
---
# PYTHON ENVIRONMENT
```
cd ~/pi-dmx-controller
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -r requirements.txt
```
---
# VERIFY OLA + DMXKING
```
sudo systemctl start olad
sleep 2
ola_dev_info | grep DMXking
```
---
# QUICK DMX TEST
```
ola_streaming_client --universe 0 --dmx 255,0,0,0
```
---
# TUI MODE
```
sudo systemctl stop pi-dmx
cd ~/pi-dmx-controller
source .venv/bin/activate
python dmx_audio_react.py
```
Quit with **q**.
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