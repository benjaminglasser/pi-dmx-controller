# DMX Audio-Reactive Light Controller (Pi + HiFiBerry + OLA)
This project turns a Raspberry Pi with a HiFiBerry DAC+ADC and DMXKing UltraDMX Pro into a real-time, ---
## Features
- Six analog knobs: Center Frequency, Q, Threshold, Attack, Decay, Brightness
- Four programs via rotary switch: All, Chase, Random, Ambient
- Reset button restores defaults instantly
- Blue LED indicates system ready
- Optional auto-start on boot (systemd) or interactive TUI mode
- Tested on Raspberry Pi OS Bookworm (32-bit) + OLA 0.10.9 + Python 3.11
---
## Hardware
| Component | Purpose |
|------------|----------|
| Raspberry Pi 4 B / 5 | Host computer |
| HiFiBerry DAC+ADC HAT | Stereo input + output for audio reactivity |
| MCP3008 ADC | Reads potentiometers (SPI0 CE0) |
| 6× 10 kW Potentiometers | Parameter control |
| 4-way Rotary Switch | Program select |
| Push Button (momentary) | Reset to defaults |
| Blue LED + 330 W resistor | System active indicator |
| DMXKing UltraDMX Pro | USB ® DMX interface |
| Chauvet DJ DMX-4 or similar | Fixture output device |
---
## Wiring (BCM numbering)
### MCP3008 (SPI0 CE0)
```
VDD, VREF ® 3.3 V
AGND, DGND ® GND
CLK ® BCM11
MOSI ® BCM10
MISO ® BCM9
CE0 ® BCM8
CH0–CH5 ® Pots (Freq, Q, Thresh, Attack, Decay, Bright)
CH6–CH7 ® Unused
```
### Rotary Switch
```
BCM21, BCM22, BCM23, BCM24 (with internal pull-ups)
Truth Table:
(1,1,1,1) ® Program 1: All
(1,1,0,0) ® Program 2: Chase
(1,0,1,0) ® Program 3: Random
(0,1,1,0) ® Program 4: Ambient
```
### Reset Button
```
BCM25 ® Button ® GND (internal pull-up)
```
### Blue LED
```
BCM5 ® 330 W Resistor ® LED Anode
LED Cathode ® GND
```
---
## Default Startup Values
```
Center: 120 Hz
Q: 1.7
Threshold: 0.032
Attack: 10 ms
Decay: 50 ms
Brightness: 1.0
```
---
## Fresh Install (from clean Pi)
1. Flash **Raspberry Pi OS Bookworm (32-bit)** using Raspberry Pi Imager and enable SSH.
2. Boot and update:
```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install python3 python3-pip python3-venv ola git
sudo reboot
```
3. Clone and bootstrap:
```bash
git clone https://github.com/benjaminglasser/pi-dmx-controller.git
cd ~/pi-dmx-controller
bash scripts/bootstrap_pi.sh
sudo reboot
```
This script:
- Installs OLA, PortAudio, and Python environment dependencies
- Enables SPI/I²C
- Adds HiFiBerry overlay (`dtoverlay=hifiberry-dacplusadc`) in `/boot/firmware/config.txt`
- Builds a Python virtual environment using `--system-site-packages`
- Patches Universe 0 ® DMXKing port 0 in OLA
---
## Python Environment (Manual Setup)
```bash
cd ~/pi-dmx-controller
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```
### requirements.txt
```text
# Core DSP / math
numpy==1.26.4 # stable on Pi (avoids libopenblas issues)

# Audio
sounddevice==0.5.3

# GPIO + SPI
RPi.GPIO==0.7.1
spidev==3.8
gpiozero==2.0.1
colorzero==2.0

# DMX / OLA
protobuf==3.20.3

# Utility
cffi==2.0.0
pycparser==2.23
pyserial==3.5
```
---
## Verify OLA & Universe
```bash
sudo systemctl start olad
sleep 2
ola_dev_info | grep DMXking
bash scripts/verify_universe.sh
```
Expected output:
```
Device 10: DMXking.com – UltraDMX2 PRO
port 0, OUT, patched to universe 0
```
---
## Quick DMX Test
Set your Chauvet DMX-4 address to 1, then:
```bash
ola_streaming_client --universe 0 --dmx 255,0,0,0,0,0,0,0
```
Channel 1 should light.
Turn off:
```bash
ola_streaming_client --universe 0 --dmx 0,0,0,0,0,0,0,0
```
---
## TUI Mode (Interactive Debugging)
### Temporarily Run in TUI Mode
1. Stop auto-start service:
```bash
sudo systemctl stop pi-dmx
```
2. Run manually:
```bash
cd ~/pi-dmx-controller
source .venv/bin/activate
python dmx_audio_react.py
```
3. Press `q` to quit.
Press the reset button (GPIO 25) to restore defaults.
Adjust knobs and programs in real time.
### Return to Headless Mode
```bash
sudo systemctl start pi-dmx
sudo systemctl status pi-dmx
```
```The blue LED will light when OLA and the service are active.```

---
## Systemd Service (Headless Mode)
This section explains how the project runs automatically in the background as a systemd service.
Systemd ensures your controller script starts on boot, restarts if it fails, and can be controlled manually with systemctl commands.
The service file (/etc/systemd/system/pi-dmx.service) defines how and when the Pi launches your dmx_audio_react.py script and ensures OLA and audio systems are ready first.
```ini
[Unit]
Description=Pi DMX Controller
After=network-online.target sound.target olad.service
Wants=network-online.target olad.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/pi-dmx-controller
Environment=PYTHONUNBUFFERED=1
ExecStart=/home/pi/pi-dmx-controller/.venv/bin/python3 /home/pi/pi-dmx-controller/dmx_audio_react.py
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```
Service control:
```bash
sudo systemctl start dmx_audio_react
sudo systemctl stop dmx_audio_react
sudo systemctl enable dmx_audio_react
sudo systemctl disable dmx_audio_react
sudo journalctl -u dmx_audio_react -f
```
---
## Troubleshooting
| Symptom | Likely Cause | Fix |
|----------|--------------|-----|
| Blue LED off | OLA not running | `sudo systemctl restart olad` |
| TUI error | Service still active | `sudo systemctl stop pi-dmx` |
| No audio input | PipeWire conflict | `systemctl --user stop pipewire*` |
| Knobs unresponsive | SPI disabled | Add `dtparam=spi=on` to `/boot/firmware/config.txt` |
| TX LED not blinking on DMXKing | USB or power fault | Check cable and `ola_dev_info` |
| No module named 'ola' | Python cannot see OLA bindings | `sudo apt install ola ola-python` then rebuild | libopenblas.so.0 not found | Wrong NumPy build | Downgrade to `numpy==1.26.4` |
---
## Audio Testing
```bash
arecord -l
aplay -l
alsamixer
```
---
## License
MIT © 2025 Ben Glasser