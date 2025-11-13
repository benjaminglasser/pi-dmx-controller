# DMX Audio‑Reactive Light Controller (Pi + HiFiBerry + OLA)

## Overview
This project transforms a Raspberry Pi into a fully standalone, audio‑reactive DMX lighting engine.

It uses:
- **HiFiBerry DAC+ADC** for clean audio input  
- **MCP3008** for six analog knobs  
- **Rotary switch** for preset selection  
- **Custom OLED boot splash pipeline** (super‑fast, firmware‑level + systemd early boot)  
- **OLA (Open Lighting Architecture)** for DMX output  

This README includes:
- Original documentation  
- All OLED improvements (splash, animation, early‑boot service)  
- All boot‑speed optimizations  
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
- Four program modes (rotary switch): All, Chase, Random, Ambient  
- Reset button restores defaults  
- Ready LED (GPIO17)  
- OLED 128×32 UI  
- Fast boot + logo splash (~2 seconds)  
- Auto‑start on boot (systemd)  
- Optional TUI mode  
- Raspberry Pi OS Bookworm tested  

---

## HARDWARE
| Component | Purpose |
|----------|----------|
| Raspberry Pi 4 / 5 | Core computer |
| HiFiBerry DAC+ADC | Audio I/O |
| MCP3008 | Analog input |
| 6× 10k Pots | Knobs |
| 4‑way Rotary | Program selector |
| Push Button | Reset |
| LED + 330Ω | Ready indicator |
| DMXKing UltraDMX Pro | DMX interface |
| DMX fixture | Output |

---

## WIRING (BCM numbering)

### MCP3008 (SPI0 CE0)
```
VDD, VREF → 3.3 V  
AGND, DGND → GND  
CLK  → BCM11  
MOSI → BCM10  
MISO → BCM9  
CE0  → BCM8  
CH0–CH5 → Potentiometers  
```

### ROTARY SWITCH
```
BCM21, BCM22, BCM23, BCM24 (all with pull‑ups)

Truth table:
(1,1,1,1) → Program 1  
(1,1,0,0) → Program 2  
(1,0,1,0) → Program 3  
(0,1,1,0) → Program 4  
```

### RESET BUTTON
```
BCM25 → Button → GND
```

### READY LED
```
BCM17 → 330Ω → LED → GND
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
This guide is optimized for rebuilding a **new SD card** as quickly and reliably as possible.

---

## 0. What the bootstrap script does

`scripts/bootstrap_pi.sh` will:

1. Update OS packages  
2. Install Python, PortAudio, ALSA, OLA  
3. Enable **SPI** and **I²C**  
4. Configure **HiFiBerry DAC+ADC** overlay  
5. Create `.venv` with system‑site packages  
6. Enable + patch OLA → Universe 0  
7. (Optional) Install + enable:  
   - `oled_splash.service`  
   - `pi-dmx.service`  

Reboot required afterwards.

---

## 1. Fresh Install on a Clean Pi

Flash Raspberry Pi OS Bookworm (32‑bit) → enable SSH → boot → then run:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git
sudo reboot
```

---

## 2. Clone the Repo & Run Bootstrap
```bash
cd ~
git clone https://github.com/benjaminglasser/pi-dmx-controller.git
cd ~/pi-dmx-controller
bash scripts/bootstrap_pi.sh
sudo reboot
```

---

## 3. Manual Python Environment (Optional)
```bash
cd ~/pi-dmx-controller
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

---

## 4. Install & Enable Services

```bash
cd ~/pi-dmx-controller

sudo cp systemd/pi-dmx.service /etc/systemd/system/
sudo cp systemd/oled_splash.service /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable oled_splash.service
sudo systemctl enable pi-dmx.service

sudo systemctl start oled_splash.service
sudo systemctl start pi-dmx.service
```

Reboot → logo appears at 2s → “Starting…” animation → app UI.

---

## 5. Verify OLA & DMXKing
```bash
sudo systemctl start olad
sleep 2
ola_dev_info | grep -A2 DMXking
```

Expected:
```
Device 10: DMXking.com – UltraDMX2 PRO
  port 0, OUT, patched to universe 0
```

---

## 6. Quick DMX Test
```bash
ola_streaming_client --universe 0 --dmx 255,0,0,0
```

Turn off:
```bash
ola_streaming_client --universe 0 --dmx 0,0,0,0
```

---

## 7. TUI Mode
```bash
sudo systemctl stop pi-dmx
cd ~/pi-dmx-controller
source .venv/bin/activate
python dmx_audio_react.py
```

Quit: **q**  
Return to service:
```bash
sudo systemctl start pi-dmx
```

---

## 8. New SD Card / Recovery Checklist
```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git
git clone https://github.com/benjaminglasser/pi-dmx-controller.git
cd ~/pi-dmx-controller
bash scripts/bootstrap_pi.sh
sudo systemctl daemon-reload
sudo systemctl enable oled_splash.service pi-dmx.service
sudo reboot
```

---

# OLED SYSTEM

## 1. `oled_boot.service`
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

---

## 2. `oled_boot.py`
Full script lives in `/utils/oled_boot.py`  
Contains:
- ultra‑fast I²C init  
- logo display  
- “Starting…” animation with pulse dot  
- exits after HOLD period or once overwritten  

---

## 3. Boot Speed Optimizations
You applied:

- Removed:
  ```
  splash quiet plymouth.ignore-serial-consoles
  ```
- Disabled:
  ```
  NetworkManager-wait-online.service
  avahi-daemon
  bluetooth
  ```
- Validated fast device availability  
- Total boot:
  - OLED logo: ~2s  
  - Controller live: ~8–10s  

---

# MAIN APP AUTOSTART

## `pi-dmx.service`
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
ExecStart=/home/pi/pi-dmx-controller/.venv/bin/python3 /home/pi/pi-dmx-controller/dmx_audio_react.py
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

---

# TROUBLESHOOTING
| Problem | Fix |
|--------|-----|
| No OLED at boot | Check `oled_splash.service` |
| Slow boot | Disable `wait-online` |
| No DMX | Check `ola_dev_info` |
| No audio | Verify HiFiBerry overlay |
| Knobs jitter | Check wiring |

---

# LICENSE
MIT © 2025 Ben Glasser
