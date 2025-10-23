# 🧩 Wiring Guide — DMX Audio-Reactive Controller (Raspberry Pi)

This guide covers all wiring for the **Raspberry Pi + HiFiBerry DAC+ADC + MCP3008** setup with six knobs, a rotary program switch, a reset button, and a status LED.

---

## 🧠 Overview

| Component | Purpose | GPIO / Interface |
|------------|----------|------------------|
| **HiFiBerry DAC+ADC** | Audio input & output | I2S bus |
| **MCP3008 ADC** | Reads six potentiometers | SPI bus (CE0) |
| **Rotary Switch** | Selects 1 of 4 programs | GPIO 21 – 24 |
| **Reset Button** | Restores default parameters | GPIO 25 |
| **Blue LED** | Indicates system ready | GPIO 5 |

---

## 🔌 MCP3008 Connections (SPI0 CE0)

| MCP3008 Pin | Connects To | Notes |
|--------------|-------------|-------|
| VDD / VREF | 3.3 V ( Pi pin 1 ) | Power |
| AGND / DGND | Ground ( Pi pin 6 ) | Common ground |
| CLK | BCM 11 ( Pi pin 23 ) | SPI Clock |
| MOSI | BCM 10 ( Pi pin 19 ) | Master Out Slave In |
| MISO | BCM 9 ( Pi pin 21 ) | Master In Slave Out |
| CE0 | BCM 8 ( Pi pin 24 ) | Chip Enable |
| CH0 – CH5 | Knobs 1–6 (10 kΩ pots) | Each to 3.3 V / GND / signal wire |
| CH6 – CH7 | Unused | Optional future inputs |

Each potentiometer (10 kΩ) →  
• Left leg → 3.3 V  
• Right leg → GND  
• Middle leg → MCP3008 channel (CH0–CH5)

| Channel | Parameter |
|----------|------------|
| CH0 | Center Frequency |
| CH1 | Q Factor |
| CH2 | Threshold |
| CH3 | Attack Time |
| CH4 | Decay Time |
| CH5 | Brightness |

---

## 🔄 Rotary Program Switch (4-Way)

Connect four switch outputs to these GPIOs (internal pull-ups enabled):

| Function | GPIO Pin | Program Code |
|-----------|-----------|--------------|
| Switch Bit 1 | BCM 21 | (1, 1, 1, 1) → Program 1 (All) |
| Switch Bit 2 | BCM 22 | (1, 1, 1, 0) → Program 2 (Chase) |
| Switch Bit 3 | BCM 23 | (1, 0, 1, 0) → Program 3 (Random) |
| Switch Bit 4 | BCM 24 | (0, 1, 1, 0) → Program 4 (Ambient) |

All switch commons → GND.  
Each position connects a unique combo of these GPIOs to ground, as shown above.

---

## 🔘 Reset Button (BCM 25)

| Pin | Connection |
|------|-------------|
| 1 | BCM 25 |
| 2 | GND |

The script uses an internal pull-up resistor, so pressing the button pulls the pin LOW.  
Pressing restores default parameters instantly.

---

## 🔵 System LED (BCM 5)

| Pin | Connection |
|------|-------------|
| Anode (+) | BCM 5 → through 330 Ω resistor |
| Cathode (–) | GND |

The LED lights solid when the program and OLA are running, signaling the system is ready.

---

## 🧷 DMX Output

Use a USB-DMX interface (e.g., Enttec Open DMX USB) connected to the Pi.  
OLA detects it automatically as **Universe 1**.

---

## 🧾 Pin Reference (BCM Layout Summary)

| Pin | Function |
|------|-----------|
| BCM 5 | Blue LED |
| BCM 8 – 11 | SPI (MCP3008) |
| BCM 21 – 24 | Rotary Switch (Program) |
| BCM 25 | Reset Button |

---

## 🧩 Tips
- Keep all grounds common between Pi, MCP3008, and DMX hardware.  
- Use short shielded wires for analog signals (CH0–CH5) to reduce noise.  
- If the pots behave inversely, swap the outer legs (3.3 V ↔ GND).  
- Always power down the Pi before rewiring any GPIO pins.

---

© 2025 Ben Glasser