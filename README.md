# DMX Audio-Reactive Light Controller (Pi + HiFiBerry + OLA)
This project turns a Raspberry Pi with a **HiFiBerry DAC+ADC** into a real-time audio-reactive DMX controller. ---
## Features
- 6 knobs: Center Frequency, Q, Threshold, Attack, Decay, Brightness
- 4 programs (via rotary switch): All, Chase, Random, Ambient
- Reset button restores defaults instantly
- Blue LED indicates system ready
- Auto-start on boot (systemd) or manual TUI mode
---
## Hardware
- Raspberry Pi 4/5
- HiFiBerry DAC+ADC HAT
- MCP3008 ADC
- 6× 10kW potentiometers
- 4-way rotary switch
- Push button (momentary)
- Blue LED + 330W resistor
- DMX interface (Enttec or compatible)
---
## Wiring (BCM numbering)

Below is a high-level overview of the wiring setup.  
For a complete pinout, connection diagrams, and detailed instructions, see the full [Wiring Guide](docs/wiring.md).

### MCP3008 (SPI0 CE0)
```
VDD, VREF ® 3.3V
AGND, DGND ® GND
CLK ® BCM11
MOSI ® BCM10
MISO ® BCM9
CE0 ® BCM8
CH0–CH5 ® Potentiometers (Freq, Q, Thresh, Attack, Decay, Bright)
CH6–CH7 ® Unused
```
### Rotary Switch
```
BCM21, BCM22, BCM23, BCM24 (internal pull-ups)
Truth Table:
(1,1,1,1) ® Program 1: All
(1,1,1,0) ® Program 2: Chase
(1,0,1,0) ® Program 3: Random
(0,1,1,0) ® Program 4: Ambient
```
### Reset Button
```
BCM25 ® button ® GND (internal pull-up)
```
### Blue LED
```
BCM5 ® 330W resistor ® LED anode, LED cathode ® GND
```
---
## Defaults (on startup)
```
Center: 120 Hz
Q: 1.7
Threshold: 0.032
Attack: 10 ms
Decay: 50 ms
Brightness: 1.0
```
---
## Fresh Install
```
sudo apt update && sudo apt install -y git
git clone https://github.com/<YOUR_USERNAME>/pi-dmx-controller.git ~/dmx_project
cd ~/dmx_project
chmod +x setup.sh
./setup.sh
sudo reboot
```
---
## Python Environment
```
cd ~/dmx_project
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```
`requirements.txt`
```
cffi==2.0.0
colorzero==2.0
gpiozero==2.0.1
numpy==2.3.4
protobuf==3.20.3
pycparser==2.23
pyserial==3.5
sounddevice==0.5.3
spidev==3.8
```
---
## Service Control (Headless Mode)
```
sudo systemctl status dmx_audio_react
sudo systemctl stop dmx_audio_react
sudo systemctl start dmx_audio_react
sudo systemctl disable dmx_audio_react
sudo systemctl enable dmx_audio_react
sudo journalctl -u dmx_audio_react -f
```
---
## TUI Mode (Interactive Debugging)
The Text User Interface (TUI) lets you visualize parameters, see real-time envelope and frequency response, ### n Temporarily Run in TUI Mode
1. **Stop the auto-start service** (so it doesn’t conflict):
```bash
sudo systemctl stop dmx_audio_react
```
2. **Activate your environment and launch manually:**
```bash
cd ~/dmx_project
source venv/bin/activate
python3 dmx_audio_react.py
```
3. The TUI will appear in your terminal window.
- Press **`q`** to quit.
- Press the **Reset button (GPIO25)** to restore defaults.
- Adjust knobs and switch programs interactively.
### n Return to Automatic (Headless) Mode
Once finished, restart the background service:
```bash
sudo systemctl start dmx_audio_react
```
To confirm it’s running:
```bash
sudo systemctl status dmx_audio_react
```
The **blue LED** will light when the service (and OLA) are active.
---
## nn Editing the Script Manually
You can safely make manual edits to your main script (`dmx_audio_react.py`) without breaking the service.
### Step-by-step:
1. **Stop the service:**
```bash
sudo systemctl stop dmx_audio_react
```
2. **Open and edit the script:**
```bash
cd ~/dmx_project
nano dmx_audio_react.py
```
3. **(Optional)** Test interactively with TUI before re-enabling autostart:
```bash
source venv/bin/activate
python3 dmx_audio_react.py
```
4. **Once verified**, restart the service for headless operation:
```bash
sudo systemctl start dmx_audio_react
```
5. **Enable or disable autostart at boot:**
```bash
sudo systemctl enable dmx_audio_react # auto-run on boot
sudo systemctl disable dmx_audio_react # disable auto-run
```
n *Tip:* Check recent logs anytime with:
```bash
sudo journalctl -u dmx_audio_react -n 30 --no-pager
```
---
## Backup to Your Mac
```
rsync -avz pi@raspberrypi.local:/home/pi/dmx_project ~/Desktop/Pi_DMX_Backup
```
---
## Troubleshooting
| Symptom | Likely Cause | Fix |
|----------|--------------|-----|
| Blue LED off | OLA not running | `sudo systemctl restart olad` |
| TUI error | Run manually (not via systemd) | `sudo systemctl stop dmx_audio_react` |
| No audio input | PipeWire conflict | Stop PipeWire with `systemctl --user stop pipewire*` |
| Knobs unresponsive | SPI disabled | Add `dtparam=spi=on` in `/boot/firmware/config.txt` |
---
## Quick Test Scripts
### Read Rotary Switch
```bash
python3 - <<'PY'
import time, RPi.GPIO as GPIO
PINS=[21,22,23,24]
MAP={(1,1,1,1):1,(1,1,1,0):2,(1,0,1,0):3,(0,1,1,0):4}
GPIO.setmode(GPIO.BCM)
for p in PINS: GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
last=None
try:
while True:
s=tuple(GPIO.input(p) for p in PINS)
if s!=last:
last=s
print("state=",s,"® Program",MAP.get(s,"?"))
time.sleep(0.05)
finally:
GPIO.cleanup()
PY
```
### Check Reset Button
```bash
python3 - <<'PY'
import time, RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_UP)
print("Press button to test (Ctrl+C to exit)")
try:
while True:
print("LOW=pressed" if GPIO.input(25)==0 else "HIGH=idle")
time.sleep(0.2)
finally:
GPIO.cleanup()
PY
```
---
## Systemd Service (reference)
`/etc/systemd/system/dmx_audio_react.service`
```
[Unit]
Description=DMX Audio Reactive Light Controller
After=network-online.target sound.target olad.service
Wants=network-online.target olad.service
[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/dmx_project
Environment=PYTHONUNBUFFERED=1
ExecStart=/home/pi/dmx_project/venv/bin/python3 /home/pi/dmx_project/dmx_audio_react.py
Restart=on-failure
RestartSec=2
[Install]
WantedBy=multi-user.target
```
---
## Audio Testing
```
arecord -l
aplay -l
alsamixer
```
---
## License
MIT © 2025 Ben Glasser