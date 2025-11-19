#!/usr/bin/env python3
# dmx_audio_react.py
# Audio-reactive DMX with 6 knobs (MCP3008), 4-position program switch,
# RESET button (GPIO25), and READY LED (GPIO17).
#
# Knob mapping:
#   CH0 -> Center frequency (log, 20–18k)
#   CH1 -> Q
#   CH2 -> Threshold
#   CH3 -> Cycle(N)  [0, 4, 8, 16, 32, 64, 128, 256 triggers between presets]
#   CH4 -> Decay (150–9999ms)
#   CH5 -> Brightness
#
# Presets:
#   P1: ALL channels 1–4
#   P2: Single-channel chase 1→2→3→4
#   P3: Group chase: (1+4) ↔ (2+3) per trigger
#   P4: Group chase: (1+2) ↔ (3+4) per trigger
#
# Cycle behavior:
#   The rotary switch defines BASE_PROGRAM (1–4).
#   The Cycle(N) knob defines how many triggers before flipping to the neighbor:
#     base 1 -> cycles 1 ↔ 2
#     base 2 -> cycles 2 ↔ 3
#     base 3 -> cycles 3 ↔ 4
#     base 4 -> cycles 4 ↔ 1
#
#   If C=0: no cycling, always stay on BASE_PROGRAM.
#   If C>0: after N triggers, toggle between base and neighbor.
#
# OLED:
#   Line1:  "P<active>  <center>Hz  Q<q>"
#   Line2:  VU meter with threshold tick
#   Line3:  "Th<0–1>  C<N>  D<ms>  B<0–1>"

import os, sys, time, math, random, threading, curses, re
from dataclasses import dataclass
from array import array

import numpy as np
import sounddevice as sd
import spidev
import RPi.GPIO as GPIO

# --- OLA Python module path (if not pip-installed) ---
if 'ola' not in sys.modules:
    possible = "/home/pi/ola/python"
    if os.path.isdir(possible) and possible not in sys.path:
        sys.path.insert(0, possible)
from ola.ClientWrapper import ClientWrapper

# --- OLED (optional) ---
try:
    import board, busio
    from PIL import Image, ImageDraw, ImageFont
    import adafruit_ssd1305   # Waveshare 128x32 = SSD1305
    _OLED_AVAILABLE = True
except Exception:
    _OLED_AVAILABLE = False

# ===================== Config =====================

UNIVERSE   = 0
DMX_CHANS  = 4

# Startup defaults
DEFAULT_CENTER_HZ = 120.0
DEFAULT_Q         = 1.7
DEFAULT_THRESH    = 0.032
DEFAULT_ATTACK_MS = 10.0      # fixed attack internally; knob is repurposed
DEFAULT_DECAY_MS  = 900.0
DEFAULT_BRIGHT    = 0.5
THRESH_MIN = 0.001
THRESH_MAX = 0.200

# Center frequency sweep range for the band-pass filter
MIN_CENTER_HZ = 20.0
MAX_CENTER_HZ = 18000.0

APP_STATE = "boot"   # "boot" | "loading" | "ready" | "error"
APP_ERROR = ""       # non-empty when APP_STATE == "error"

# Audio
SR  = 44100
HOP = 1024

# Detection / logic
ENV_EMA       = 0.55
AGC_ON        = True
AGC_TARGET    = 0.020
REFRACTORY_MS = 110.0
WEIGHTING_ON  = False
INPUT_GAIN    = 1.0
BRIGHTNESS    = DEFAULT_BRIGHT

# Program state
PROGRAM      = 1   # currently ACTIVE program (for UI)
BASE_PROGRAM = 1   # selected by rotary switch (1..4)

RUNNING      = True
STOP_THREADS = False

# When > time.time(), knob readings are ignored (used after RESET / startup)
IGNORE_KNOBS_UNTIL = 0.0

PREFERRED_INPUTS = [r"hifiberry", r"dac\+adc", r"scarlett", r"usb audio", r"codec", r"line"]

# Rotary switch pins (BCM) and mapping
SW_PINS = [21, 22, 23, 24]  # pulled-up
SW_MAP = {
    (1,1,1,1): 1,
    (1,1,0,0): 2,
    (1,0,1,0): 3,
    (0,1,1,0): 4,
}
SW_DEBOUNCE_SAMPLES = 3
SW_SAMPLE_PERIOD_S  = 0.01

# Reset button and Ready LED
RESET_PIN     = 25              # momentary to GND (active LOW)
READY_LED_PIN = 17              # LED anode -> 330Ω -> BCM17, cathode -> GND

# MCP3008 on SPI0 CE0
SPI_BUS, SPI_DEV = 0, 0

# DMX throttling
DMX_RATE_HZ       = 25.0
_DMX_MIN_INTERVAL = 1.0 / DMX_RATE_HZ

# --- TUI flash message (rendered inside curses, not printed) ---
_ui_flash_msg   = ""
_ui_flash_until = 0.0

def ui_flash(msg: str, seconds: float = 1.5):
    global _ui_flash_msg, _ui_flash_until
    _ui_flash_msg   = msg
    _ui_flash_until = time.time() + seconds

def _set_stop(val: bool):
    global STOP_THREADS
    STOP_THREADS = val

def _set_run(val: bool):
    global RUNNING
    RUNNING = val

# ===================== Cycle logic globals =====================

# Cycle steps options for Cycle(N) knob
CYCLE_STEPS_OPTIONS = [0, 4, 8, 16, 32, 64, 128, 256]

CYCLE_STEPS         = 0    # active N (0 = off) – default at boot
CYCLE_TRIGGER_COUNT = 0   # how many triggers since last flip
CYCLE_PHASE         = 0   # 0 = base program, 1 = neighbor

def program_pair_for_base(base: int):
    """Return (base, neighbor) pair for cycling."""
    if base == 1:
        return (1, 2)
    elif base == 2:
        return (2, 3)
    elif base == 3:
        return (3, 4)
    else:
        return (4, 1)

def set_cycle_steps(steps: int):
    """Set a new cycle N and reset counters/phase."""
    global CYCLE_STEPS, CYCLE_TRIGGER_COUNT, CYCLE_PHASE
    CYCLE_STEPS         = int(steps)
    CYCLE_TRIGGER_COUNT = 0
    CYCLE_PHASE         = 0

# ===================== OLA / DMX =====================

wrapper = ClientWrapper()
client  = wrapper.Client()

dmx_frame      = array('B', [0]*DMX_CHANS)
_last_dmx_send = 0.0
_dmx_dirty     = False
_pending_vals  = [0]*DMX_CHANS

class _CompatBuf:
    """Compatibility wrapper so OLA works with both .tostring() and .tobytes()."""
    __slots__ = ("_b",)
    def __init__(self, seq):
        self._b = bytes(seq)
    def tostring(self):   # old API
        return self._b
    def tobytes(self):    # new API
        return self._b
    def __bytes__(self):
        return self._b

def send_dmx(vals):
    """Queue up to 4 channel DMX; actual SendDmx happens in wrapper thread."""
    global _dmx_dirty, _pending_vals
    for i in range(DMX_CHANS):
        v = max(0, min(255, int(vals[i])))
        _pending_vals[i] = v
    _dmx_dirty = True

def _dmx_pump():
    """Runs in the OLA wrapper thread: rate-limit & flush DMX if dirty, reschedule itself."""
    global _dmx_dirty, _last_dmx_send
    now = time.time()

    # graceful exit: if we're stopping, end the wrapper loop
    if STOP_THREADS:
        try:
            wrapper.Stop()
        except Exception:
            pass
        return  # don't reschedule

    if _dmx_dirty and (now - _last_dmx_send) >= _DMX_MIN_INTERVAL:
        for i in range(DMX_CHANS):
            dmx_frame[i] = _pending_vals[i]
        client.SendDmx(UNIVERSE, _CompatBuf(dmx_frame), lambda s: None)
        _last_dmx_send = now
        _dmx_dirty = False

    wrapper.AddEvent(20, _dmx_pump)  # milliseconds

# ===================== Light envelopes =====================

@dataclass
class LightState:
    env: float = 0.0
    post: float = 0.0
    t_ms: float = 0.0
    active: bool = False

states  = [LightState() for _ in range(DMX_CHANS)]
POST_EMA = 0.6

class BandParams:
    def __init__(self):
        self.center    = DEFAULT_CENTER_HZ
        self.q         = DEFAULT_Q
        self.thresh    = DEFAULT_THRESH
        self.attack_ms = DEFAULT_ATTACK_MS
        self.decay_ms  = DEFAULT_DECAY_MS

band = BandParams()
_runtime = {'attack_ms': band.attack_ms, 'decay_ms': band.decay_ms}

def trigger_idxs(idxs, attack_ms, decay_ms):
    for i in idxs:
        s = states[i]
        s.env = s.post = 0.0
        s.t_ms = 0.0
        s.active = True
    _runtime['attack_ms'] = attack_ms
    _runtime['decay_ms']  = decay_ms

def update_lights(dt_ms):
    a = max(1e-3, _runtime['attack_ms'])
    d = max(1e-3, _runtime['decay_ms'])
    vals = []
    for s in states:
        if s.active:
            if s.t_ms < a:
                s.env = min(1.0, s.env + dt_ms/a)
            else:
                s.env = max(0.0, s.env - dt_ms/d)
            s.t_ms += dt_ms
            if s.env <= 0.0 and s.t_ms > a:
                s.active = False
        else:
            s.env = max(0.0, s.env - dt_ms/d)
        s.post = POST_EMA*s.env + (1.0-POST_EMA)*s.post
        vals.append(int(255 * s.post * BRIGHTNESS))
    return vals

# Ambient stub (unused in this version, but kept for future)
_ambient_next_time = 0.0
def ambient_vals(dt_ms):
    """Unused in this version; preserved for future ambient experiments."""
    global _ambient_next_time
    now = time.time()
    base_min, base_max = 0.4, 1.2
    if now >= _ambient_next_time:
        _ambient_next_time = now + random.uniform(base_min, base_max)
        k = random.choices([1, 2, 3], weights=[0.65, 0.30, 0.05])[0]
        idxs = random.sample(range(DMX_CHANS), k)
        trigger_idxs(idxs, band.attack_ms, band.decay_ms)
    return update_lights(dt_ms)

# ===================== DSP =====================

class BiquadBandpass:
    def __init__(self, sr, center_hz, q):
        self.sr = sr
        self.center = center_hz
        self.q = q
        self.reset()
        self._design()
    def reset(self):
        self.x1=self.x2=self.y1=self.y2=0.0
    def set_params(self, center_hz, q):
        self.center = max(MIN_CENTER_HZ, min(MAX_CENTER_HZ, float(center_hz)))
        self.q      = max(0.3,           min(12.0,          float(q)))
        self._design()
    def _design(self):
        w0 = 2.0*math.pi*self.center/self.sr
        alpha = math.sin(w0)/(2.0*self.q)
        b0,b1,b2 =  math.sin(w0)/2.0, 0.0, -math.sin(w0)/2.0
        a0,a1,a2 =  1.0 + alpha, -2.0*math.cos(w0), 1.0 - alpha
        self.b0,self.b1,self.b2 = b0/a0, b1/a0, b2/a0
        self.a1,self.a2 = a1/a0, a2/a0
    def process(self, x):
        y = np.empty_like(x, dtype=np.float32)
        b0,b1,b2,a1,a2 = self.b0,self.b1,self.b2,self.a1,self.a2
        x1,x2,y1,y2 = self.x1,self.x2,self.y1,self.y2
        for i in range(len(x)):
            xi = float(x[i])
            yo = b0*xi + b1*x1 + b2*x2 - a1*y1 - a2*y2
            y[i] = yo
            x2, x1 = x1, xi
            y2, y1 = y1, yo
        self.x1,self.x2,self.y1,self.y2 = x1,x2,y1,y2
        return y

class EnvDetector:
    def __init__(self, sr, attack_ms=8.0, release_ms=80.0):
        self.sr = sr
        self.set_times(attack_ms, release_ms)
        self.y = 0.0
    def set_times(self, attack_ms, release_ms):
        self.alpha_a = math.exp(-1.0/(max(1e-3, attack_ms)*1e-3*self.sr))
        self.alpha_r = math.exp(-1.0/(max(1e-3, release_ms)*1e-3*self.sr))
    def process(self, x):
        out = np.empty_like(x, dtype=np.float32)
        y = self.y
        aa, ar = self.alpha_a, self.alpha_r
        for i in range(len(x)):
            s = abs(float(x[i]))
            if s > y: y = aa*y + (1.0-aa)*s
            else:     y = ar*y + (1.0-ar)*s
            out[i] = y
        self.y = y
        return out

class Agc:
    def __init__(self, target=0.02, tau=0.95):
        self.target=target
        self.gain  = 1.0
        self.tau   = tau
    def update(self, env_mean):
        eps=1e-6
        desired=self.target/max(eps, env_mean)
        desired=max(0.1, min(20.0, desired))
        self.gain=self.tau*self.gain+(1.0-self.tau)*desired
        return self.gain

# ===================== Input device pick =====================

def pick_input_device():
    devs = sd.query_devices()
    for pat in PREFERRED_INPUTS:
        rx = re.compile(pat, re.I)
        for i, d in enumerate(devs):
            if d.get("max_input_channels",0) >= 2 and rx.search(d.get("name","")):
                return i, d['name']
    for i, d in enumerate(devs):
        if d.get("max_input_channels",0) >= 2:
            return i, d['name']
    raise RuntimeError("No suitable input device (>=2ch) found")

DEVICE_INDEX, DEVICE_NAME = pick_input_device()

# ===================== MCP3008 (knobs) =====================

spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEV)
spi.max_speed_hz = 1350000
spi.mode = 0
spi_lock = threading.Lock()

def read_mcp3008(ch: int) -> int:
    """Read single-ended channel 0..7 (10-bit). Thread-safe & resilient."""
    cmd = [1, (8+ch) << 4, 0]
    with spi_lock:
        try:
            resp = spi.xfer2(cmd)
        except OSError:
            time.sleep(0.02)
            try:
                spi.close()
            except Exception:
                pass
            spi.open(SPI_BUS, SPI_DEV)
            spi.max_speed_hz = 1350000
            spi.mode = 0
            resp = spi.xfer2(cmd)
    return ((resp[1] & 3) << 8) | resp[2]

def lerp(a,b,t): return a + (b-a)*t
def clamp(x,a,b): return max(a, min(b, x))

# --- smooth + movement detection (soft-takeover) ---
_knob_ema         = [None]*8
_knob_last_soft   = [None]*8
_knob_jumped_soft = [False]*8

# Smaller threshold = knobs take over sooner (less "sticky").
# 0.005 = 0.5% of full travel.
MOVE_EPS          = 0.005

def read_knob_norm(ch: int, alpha=0.25) -> float:
    raw = read_mcp3008(ch) / 1023.0
    prev = _knob_ema[ch]
    v = raw if prev is None else (alpha*raw + (1-alpha)*prev)
    _knob_ema[ch] = v
    return v

def _jump_takeover(ch_idx, map_fn, alpha=0.25):
    """
    Read one knob (normalized 0..1 with EMA smoothing). If that knob hasn't
    taken control yet, require a small movement from its last seeded position
    before 'jumping' the parameter to the knob's mapped value.
    Returns: None if not yet jumped, else the mapped value.
    """
    n = read_knob_norm(ch_idx, alpha=alpha)  # 0..1 smoothed
    last = _knob_last_soft[ch_idx]

    # First time we see it, seed and do nothing
    if last is None:
        _knob_last_soft[ch_idx] = n
        return None

    # If not yet jumped, wait for movement past threshold
    if not _knob_jumped_soft[ch_idx]:
        if abs(n - last) >= MOVE_EPS:
            _knob_jumped_soft[ch_idx] = True
            _knob_last_soft[ch_idx] = n
            return map_fn(n)
        else:
            _knob_last_soft[ch_idx] = n
            return None
    else:
        # Already jumped: always follow knob
        _knob_last_soft[ch_idx] = n
        return map_fn(n)

# --- parameter mapping functions ---

def map_center(x):
    """
    Map knob (0..1) → center frequency [MIN_CENTER_HZ .. MAX_CENTER_HZ] on a
    log scale, with extra resolution in the low range.
    """
    x = clamp(x, 0.0, 1.0)
    gamma = 1.4          # >1 => more detail in lows, less in highs
    t = x**gamma         # skew the knob toward the low end

    log_min = math.log10(MIN_CENTER_HZ)
    log_max = math.log10(MAX_CENTER_HZ)
    log_f   = log_min + (log_max - log_min) * t
    return 10**log_f

def map_q(x):
    return lerp(0.4, 6.0, clamp(x, 0, 1))

def map_thresh(x):
    """Map knob 0..1 → internal threshold THRESH_MIN..THRESH_MAX."""
    return lerp(THRESH_MIN, THRESH_MAX, clamp(x, 0, 1))

def thresh_to_ui(t: float) -> float:
    """Map internal threshold THRESH_MIN..THRESH_MAX → 0.0..1.0 for UI."""
    t = clamp(t, THRESH_MIN, THRESH_MAX)
    return (t - THRESH_MIN) / (THRESH_MAX - THRESH_MIN)

def map_decay(x):
    return lerp(150.0, 9999.0, clamp(x, 0, 1))

def map_bright(x):
    return lerp(0.0, 1.0, clamp(x, 0, 1))

def map_cycle_steps(x):
    """Map knob 0..1 to one of CYCLE_STEPS_OPTIONS."""
    x = clamp(x, 0.0, 1.0)
    idx = int(round(x * (len(CYCLE_STEPS_OPTIONS) - 1)))
    return CYCLE_STEPS_OPTIONS[idx]

def update_from_knobs():
    """Soft-takeover for CH0,1,2,3,4; CH5 (brightness) is also soft but never reset."""
    global BRIGHTNESS, IGNORE_KNOBS_UNTIL, CYCLE_STEPS

    # During a short window after RESET/boot, ignore knobs so defaults can "stick"
    if time.time() < IGNORE_KNOBS_UNTIL:
        return

    # CH0: Center frequency
    v = _jump_takeover(0, map_center, alpha=0.35)
    if v is not None:
        band.center = v

    # CH1: Q
    v = _jump_takeover(1, map_q, alpha=0.35)
    if v is not None:
        band.q = v

    # CH2: Threshold
    v = _jump_takeover(2, map_thresh, alpha=0.30)
    if v is not None:
        band.thresh = v

    # CH3: Cycle (N) — soft-takeover, but ONLY reset counters when N actually changes
    v = _jump_takeover(3, map_cycle_steps, alpha=0.30)
    if v is not None:
        # Only re-apply if the quantized N changed (0, 4, 8, 16, 32, 64, 128, 256)
        if int(v) != CYCLE_STEPS:
            set_cycle_steps(v)

    # CH4: Decay
    v = _jump_takeover(4, map_decay, alpha=0.35)
    if v is not None:
        band.decay_ms = v

    # CH5: Brightness
    v = _jump_takeover(5, map_bright, alpha=0.35)
    if v is not None:
        BRIGHTNESS = v

# ===================== Reset Button & Ready LED =====================

def reset_to_defaults(channel=None):
    """
    Reset all parameters EXCEPT brightness.
    Brightness stays where it is and its soft-takeover state is untouched.
    Also resets Cycle(N) back to 0.
    """
    global band, IGNORE_KNOBS_UNTIL

    # Core band params reset
    band.center    = DEFAULT_CENTER_HZ
    band.q         = DEFAULT_Q
    band.thresh    = DEFAULT_THRESH
    band.attack_ms = DEFAULT_ATTACK_MS
    band.decay_ms  = DEFAULT_DECAY_MS

    # Cycle: turn OFF and reset its counters/state
    set_cycle_steps(0)   # C0 on OLED, clears count and phase

    # Clear soft-takeover state for knobs 0–4 so they re-seed cleanly
    # (Brightness knob stays "in control" where it is.)
    for i in range(5):   # CH0..CH4
        _knob_ema[i]         = None
        _knob_last_soft[i]   = None
        _knob_jumped_soft[i] = False

    # Short window where we ignore knob updates so defaults can stick
    IGNORE_KNOBS_UNTIL = time.time() + 0.3  # ~300 ms

    ui_flash("[RESET] Defaults (C0; brightness kept).", 1.5)

def setup_gpio_inputs():
    """Configure GPIO once (no edge detection; we poll)."""
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    # Rotary switch pins (pull-ups)
    for p in SW_PINS:
        GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # Reset button on BCM25 (pull-up). Press = GND -> 0
    GPIO.setup(RESET_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # READY LED
    GPIO.setup(READY_LED_PIN, GPIO.OUT, initial=GPIO.LOW)

def switch_reader():
    """
    Background thread:
      - Poll rotary switch (debounce + map to BASE_PROGRAM)
      - Poll reset button (BCM25) and call reset_to_defaults() on clean press
    """
    global BASE_PROGRAM, STOP_THREADS, CYCLE_TRIGGER_COUNT, CYCLE_PHASE

    last_switch = None
    switch_stable = 0
    last_reset_state = 1  # pulled-up: 1 = released, 0 = pressed

    try:
        while not STOP_THREADS:
            try:
                s = tuple(GPIO.input(p) for p in SW_PINS)
                r = GPIO.input(RESET_PIN)
            except RuntimeError:
                break

            # Rotary switch debounce
            if s == last_switch:
                switch_stable += 1
            else:
                last_switch, switch_stable = s, 1

            if switch_stable >= SW_DEBOUNCE_SAMPLES:
                prog = SW_MAP.get(s)
                if prog and prog != BASE_PROGRAM:
                    BASE_PROGRAM = prog
                    # When base program changes, restart cycle from base side
                    CYCLE_TRIGGER_COUNT = 0
                    CYCLE_PHASE = 0

            # Reset button: detect clean 1 -> 0 edge with tiny debounce
            if last_reset_state == 1 and r == 0:
                time.sleep(0.02)
                try:
                    if GPIO.input(RESET_PIN) == 0:
                        reset_to_defaults()
                except RuntimeError:
                    break
            last_reset_state = r

            time.sleep(SW_SAMPLE_PERIOD_S)
    finally:
        pass

# ===================== Audio loop =====================

live_band_env   = 0.0
live_threshold  = band.thresh
input_rms       = 0.0
last_trigger_ts = 0.0
chase_idx       = 0
group34_phase   = 0   # toggles between 0/1 for grouped programs (3 & 4)

bp   = None
envd = None
agc  = Agc(target=AGC_TARGET, tau=0.95)

def audio_loop():
    global bp, envd, live_band_env, live_threshold, input_rms
    global last_trigger_ts, chase_idx, group34_phase
    global PROGRAM, BASE_PROGRAM, CYCLE_STEPS, CYCLE_TRIGGER_COUNT, CYCLE_PHASE
    global APP_STATE, APP_ERROR

    bp   = BiquadBandpass(SR, band.center, band.q)
    envd = EnvDetector(SR, attack_ms=8.0, release_ms=80.0)

    band.attack_ms = DEFAULT_ATTACK_MS

    frame_dt_ms = (HOP / SR) * 1000.0
    was_above = False

    def cb(indata, frames, time_info, status):
        nonlocal was_above
        global live_band_env, live_threshold, input_rms
        global last_trigger_ts, chase_idx, group34_phase
        global PROGRAM, BASE_PROGRAM, CYCLE_STEPS, CYCLE_TRIGGER_COUNT, CYCLE_PHASE

        if not RUNNING:
            return

        # mono
        x = indata.mean(axis=1).astype(np.float32)
        input_rms = float(np.sqrt(np.mean(x*x)) + 1e-12)

        # knobs
        update_from_knobs()

        # filter + envelope
        bp.set_params(band.center, band.q)
        y = bp.process(x)
        e = envd.process(y)

        # AGC + optional weighting + input gain
        g = agc.update(float(np.mean(e))) if AGC_ON else 1.0
        w = math.sqrt(max(1.0, band.center/100.0)) if WEIGHTING_ON else 1.0
        e_scaled = e * (g * INPUT_GAIN * w)

        # outer EMA for live readout
        v, a = live_band_env, ENV_EMA
        for s in e_scaled:
            v = a*v + (1.0-a)*float(s)
        live_band_env  = v
        live_threshold = band.thresh

        # trigger logic (with refractory)
        now = time.time()
        above    = (live_band_env >= band.thresh)
        can_fire = ((now - last_trigger_ts)*1000.0 >= REFRACTORY_MS)

        # Decide which program to use for THIS trigger
        if CYCLE_STEPS > 0:
            p_base, p_neighbor = program_pair_for_base(BASE_PROGRAM)
            active_prog = p_base if CYCLE_PHASE == 0 else p_neighbor
        else:
            active_prog = BASE_PROGRAM

        PROGRAM = active_prog

        if above and not was_above and can_fire and active_prog in (1, 2, 3, 4):
            last_trigger_ts = now

            # Per-program light behavior
            if active_prog == 1:  # ALL
                trigger_idxs([0, 1, 2, 3], band.attack_ms, band.decay_ms)

            elif active_prog == 2:  # CHASE (single channel)
                trigger_idxs([chase_idx], band.attack_ms, band.decay_ms)
                chase_idx = (chase_idx + 1) % 4

            elif active_prog == 3:  # GROUP CHASE: (1+4) ↔ (2+3)
                if group34_phase == 0:
                    trigger_idxs([0, 3], band.attack_ms, band.decay_ms)
                    group34_phase = 1
                else:
                    trigger_idxs([1, 2], band.attack_ms, band.decay_ms)
                    group34_phase = 0

            elif active_prog == 4:  # GROUP CHASE: (1+2) ↔ (3+4)
                if group34_phase == 0:
                    trigger_idxs([0, 1], band.attack_ms, band.decay_ms)
                    group34_phase = 1
                else:
                    trigger_idxs([2, 3], band.attack_ms, band.decay_ms)
                    group34_phase = 0

            # Cycle trigger counting
            if CYCLE_STEPS > 0:
                CYCLE_TRIGGER_COUNT += 1
                if CYCLE_TRIGGER_COUNT >= CYCLE_STEPS:
                    CYCLE_TRIGGER_COUNT = 0
                    CYCLE_PHASE = 1 - CYCLE_PHASE

        # DMX output
        send_dmx(update_lights(frame_dt_ms))

        was_above = above

    try:
        with sd.InputStream(device=DEVICE_INDEX,
                            channels=2,
                            samplerate=SR,
                            blocksize=HOP,
                            callback=cb):
            APP_STATE = "ready"
            while not STOP_THREADS:
                time.sleep(0.05)
    except Exception as e:
        APP_STATE = "error"
        APP_ERROR = f"Audio init failed: {e}"
    finally:
        pass

# ===================== OLED UI (128x32) =====================

class OledUI:
    """
    128x32 SSD1305 via I2C. Runs in its own thread and pulls from globals:
      PROGRAM, BASE_PROGRAM, live_band_env, band.thresh, band.center, band.q,
      band.decay_ms, BRIGHTNESS, CYCLE_STEPS
    """
    def __init__(self, addr=0x3D, fps=15):
        self.enabled = False
        self.addr = addr
        self.period = 1.0 / max(1, fps)
        self._font = None
        if not _OLED_AVAILABLE:
            return
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.oled = adafruit_ssd1305.SSD1305_I2C(128, 32, self.i2c, addr=self.addr)
            self.oled.fill(0); self.oled.show()
            from PIL import ImageFont
            try:
                self._font = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 8
                )
            except Exception:
                self._font = ImageFont.load_default()
            self.enabled = True
        except Exception:
            self.enabled = False

    def clear(self):
        if not self.enabled: return
        try:
            self.oled.fill(0); self.oled.show()
        except Exception:
            pass

    def _fmt_center(self, f):
        return f"{int(f):d}"

    def _draw_meter(self, draw, x, y, w, h, env_val, thr_val):
        SCALE = 0.20
        env_t = max(0.0, min(1.0, float(env_val) / SCALE))
        thr_t = max(0.0, min(1.0, float(thr_val) / SCALE))
        env_px = int(env_t * (w - 1))
        thr_px = int(thr_t * (w - 1))
        draw.rectangle((x, y, x + w - 1, y + h - 1), outline=1, fill=0)
        if env_px > 0:
            draw.rectangle((x + 1, y + 1, x + 1 + env_px, y + h - 2),
                           outline=1, fill=1)
        tick_x = x + thr_px
        draw.line((tick_x, y, tick_x, y + h - 1), fill=1)

    def _render_error(self, draw, err_msg):
        f = self._font
        draw.text((0, 0),  "ERROR", font=f, fill=1)
        msg = (err_msg or "See logs")[:20]
        draw.text((0, 12), msg, font=f, fill=1)
        draw.text((0, 24), "Reboot or fix & retry", font=f, fill=1)

    def snapshot_params(self):
        return {
            "base_program": BASE_PROGRAM,
            "active_program": PROGRAM,
            "env": float(live_band_env),
            "thr": float(band.thresh),
            "center": float(band.center),
            "q": float(band.q),
            "d": float(band.decay_ms),
            "b": float(BRIGHTNESS),
            "c": int(CYCLE_STEPS),
        }

    def render_once(self):
        if not self.enabled:
            return

        W, H = self.oled.width, self.oled.height  # 128x32
        image = Image.new("1", (W, H))
        draw = ImageDraw.Draw(image)
        f = self._font

        if APP_STATE == "loading":
            return
        elif APP_STATE == "error":
            self._render_error(draw, APP_ERROR)
        else:
            s = self.snapshot_params()
            line1 = f"P{s['active_program']}  {self._fmt_center(s['center'])}Hz  Q{s['q']:.1f}"
            draw.text((0, 0), line1, font=f, fill=1)

            meter_x, meter_y, meter_w, meter_h = 0, 10, W, 8
            self._draw_meter(draw, meter_x, meter_y, meter_w, meter_h,
                             s["env"], s["thr"])

            # Threshold normalized for display (0.0–1.0)
            thr_norm = thresh_to_ui(s["thr"])

            line3 = f"Th{thr_norm:0.1f}  C{s['c']}  D{int(s['d']):d}  B{s['b']:.2f}"
            draw.text((0, 22), line3, font=f, fill=1)

        try:
            self.oled.image(image)
            self.oled.show()
        except Exception:
            self.enabled = False

    def loop(self):
        next_t = time.time()
        while not STOP_THREADS:
            t0 = time.time()
            self.render_once()
            next_t += self.period
            time.sleep(max(0.0, next_t - t0))
        self.clear()

    def show_splash(self, image_path=None, seconds=2.0):
        if not self.enabled:
            return
        try:
            W, H = self.oled.width, self.oled.height
            image = Image.new("1", (W, H))
            draw = ImageDraw.Draw(image)
            if image_path:
                try:
                    src = Image.open(image_path).convert("1")
                    src = src.resize((W, H))
                    image.paste(src, (0, 0))
                except Exception:
                    draw.rectangle((0, 0, W-1, H-1), outline=1, fill=0)
                    draw.text((2, 10), "Starting...", font=self._font, fill=1)
            else:
                draw.rectangle((0, 0, W-1, H-1), outline=1, fill=0)
                draw.text((2, 10), "Starting...", font=self._font, fill=1)
            self.oled.image(image); self.oled.show()
            t_end = time.time() + max(0.0, float(seconds))
            while time.time() < t_end and not STOP_THREADS:
                time.sleep(0.05)
        except Exception:
            pass

# ===================== TUI =====================

def safe_addstr(stdscr, y, x, s):
    h, w = stdscr.getmaxyx()
    if y < 0 or y >= h or x >= w: return
    if x < 0:
        s = s[-x:]; x = 0
    maxlen = w - x
    if maxlen > 0:
        stdscr.addnstr(y, x, s, maxlen)

def draw_band_bar(stdscr, y, x, width, center, q):
    left_hz, right_hz = MIN_CENTER_HZ, MAX_CENTER_HZ
    def hz_to_col(f):
        lf = math.log10(max(left_hz, min(right_hz, f)))
        lmin, lmax = math.log10(left_hz), math.log10(right_hz)
        return int((lf - lmin)/(lmax-lmin) * (width-1))
    bw   = center/max(1e-6, q)
    f_lo = max(left_hz,  center - 0.5*bw)
    f_hi = min(right_hz, center + 0.5*bw)
    c0   = x + hz_to_col(f_lo)
    c1   = x + hz_to_col(f_hi)
    c0, c1 = min(c0, x+width-1), min(c1, x+width-1)
    safe_addstr(stdscr, y, x, "─"*width)
    for col in range(c0, c1+1):
        safe_addstr(stdscr, y, col, "━")
    safe_addstr(stdscr, y+1, x, f"{int(f_lo)} Hz ← band → {int(f_hi)} Hz".ljust(width))

def draw_threshold_meter(stdscr, y, x, width, env_val, thr):
    m = 0.20
    e = max(0.0, min(1.0, env_val/m))
    t = max(0.0, min(1.0, thr/m))
    bar = ["-"]*width
    thr_col = min(width-1, int(t*(width-1)))
    env_col = min(width-1, int(e*(width-1)))
    for i in range(env_col+1):
        bar[i] = "#"
    bar[thr_col] = "|"
    safe_addstr(stdscr, y, x, "".join(bar))

def tui(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.timeout(33)   # ~30 FPS

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        bar_width = max(20, min(65, w - 2))

        safe_addstr(
            stdscr, 0, 0,
            f"Base P{BASE_PROGRAM}  Active P{PROGRAM}   RUN={'ON' if RUNNING else 'PAUSE'}   "
            f"Device={DEVICE_NAME}  SR={SR}  HOP={HOP}"
        )
        safe_addstr(
            stdscr, 1, 0,
            f"ENV_EMA={ENV_EMA:.2f}  AGC={'ON' if AGC_ON else 'OFF'} target={AGC_TARGET:.3f}  "
            f"Refractory={REFRACTORY_MS:.0f}ms  Weighting={'ON' if WEIGHTING_ON else 'OFF'}"
        )
        safe_addstr(
            stdscr, 2, 0,
            f"Cycle: C={CYCLE_STEPS}  phase={CYCLE_PHASE}  count={CYCLE_TRIGGER_COUNT}"
        )

        row = 4
        labels_vals = [
            ("center (Hz)",  band.center),
            ("Q",            band.q),
            ("threshold",    thresh_to_ui(band.thresh)),  # 0.0–1.0 for UI
            ("decay (ms)",   band.decay_ms),
            ("brightness",   BRIGHTNESS),
        ]
        safe_addstr(stdscr, row-1, 0, "Params (knobs with soft-takeover):")
        for label, val in labels_vals:
            safe_addstr(stdscr, row, 2, f"{label:<12}: {val:>8.3f}")
            row += 1

        safe_addstr(stdscr, row+1, 0, "Band Env vs Threshold (| is threshold):")
        draw_threshold_meter(stdscr, row+2, 0, bar_width, live_band_env, band.thresh)
        safe_addstr(
            stdscr, row+3, 0,
            f"env={live_band_env:.4f}  thresh={thresh_to_ui(band.thresh):.1f} (0–1 UI)"
        )

        safe_addstr(stdscr, row+5, 0, "Targeted Frequency Band:")
        draw_band_bar(stdscr, row+6, 0, bar_width, band.center, band.q)

        safe_addstr(stdscr, row+8, 0, "Channels:")
        for i, s in enumerate(states, start=1):
            safe_addstr(
                stdscr, row+8+i, 1,
                f"ch{i}: env={s.env:.3f} post={s.post:.3f} stage={'on' if s.active else 'idle'}"
            )

        # transient UI flash at bottom line
        if time.time() < _ui_flash_until and _ui_flash_msg:
            h, w = stdscr.getmaxyx()
            msg = _ui_flash_msg
            x = max(0, (w - len(msg)) // 2)
            stdscr.addnstr(h - 1, x, msg, max(0, w - x))

        stdscr.refresh()

        ch = stdscr.getch()
        if ch in (ord('q'), ord('Q'), 27):  # ESC
            _set_stop(True)
            _set_run(False)
            break

# ===================== Main =====================

def main():

    print(f"[OK] Using input: {DEVICE_INDEX} - {DEVICE_NAME}")
    print("[OK] Knobs CH0..CH5 with soft-takeover.")
    print(f"[OK] DMX -> Universe {UNIVERSE}, Channels 1..4")

    setup_gpio_inputs()

    # Initial ignore window so startup noise doesn't cause jumps
    global APP_STATE, IGNORE_KNOBS_UNTIL
    APP_STATE = "loading"
    IGNORE_KNOBS_UNTIL = time.time() + 0.3

    # OLED UI (optional)
    oled_ui = OledUI(addr=0x3D, fps=15)
    th_oled = None
    if getattr(oled_ui, "enabled", False):
        th_oled = threading.Thread(target=oled_ui.loop, daemon=True)
        th_oled.start()
        print("[OK] OLED UI: 128x32 @ 0x3D")
    else:
        print("[INFO] OLED UI not available (skipping).")

    # Threads
    th_sw = threading.Thread(target=switch_reader)
    th_sw.start()

    th_audio = threading.Thread(target=audio_loop)
    th_audio.start()

    use_tui = sys.stdout.isatty() and os.environ.get("ENABLE_TUI", "1") == "1"
    th_tui = None
    if use_tui:
        th_tui = threading.Thread(target=lambda: curses.wrapper(tui))
        th_tui.start()
    else:
        print("[INFO] No TTY detected (or ENABLE_TUI=0). Running headless.")

    wrapper.AddEvent(0, _dmx_pump)

    try:
        wrapper.Run()
    except KeyboardInterrupt:
        pass
    finally:
        _set_stop(True)
        _set_run(False)

        try: th_audio.join(timeout=2.0)
        except Exception: pass
        try: th_sw.join(timeout=1.0)
        except Exception: pass
        if th_tui is not None:
            try: th_tui.join(timeout=1.0)
            except Exception: pass
        if th_oled is not None:
            try: th_oled.join(timeout=1.0)
            except Exception: pass

        try: wrapper.Stop()
        except Exception: pass

        # All channels off
        try:
            client.SendDmx(UNIVERSE, _CompatBuf(bytes([0]*DMX_CHANS)), lambda s: None)
            time.sleep(0.05)
        except Exception:
            pass

        print("\nAll channels off. Bye.")

        try: spi.close()
        except Exception: pass
        try: GPIO.cleanup()
        except Exception: pass

if __name__ == "__main__":
    main()