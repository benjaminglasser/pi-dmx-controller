#!/usr/bin/env python3
# dmx_audio_react.py
# Audio-reactive DMX with 6 knobs (MCP3008), 4-position program switch,
# RESET button (GPIO25), and READY LED (GPIO17).

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

# ===================== Config =====================

UNIVERSE   = 0
DMX_CHANS  = 4

# Startup defaults (your preferred values)
DEFAULT_CENTER_HZ = 120.0
DEFAULT_Q         = 1.7
DEFAULT_THRESH    = 0.032
DEFAULT_ATTACK_MS = 10.0
DEFAULT_DECAY_MS  = 50.0
DEFAULT_BRIGHT    = 1.0

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

PROGRAM       = 1   # 1..4
RUNNING       = True
STOP_THREADS = False

PREFERRED_INPUTS = [r"hifiberry", r"dac\+adc", r"scarlett", r"usb audio", r"codec", r"line"]

# Rotary switch pins (BCM) and mapping (your working truth table)
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

# --- Jump-on-first-move state for 6 knobs (CH0..CH5)
_knob_last   = [None]*6   # last normalized reading (0..1) per knob
_knob_jumped = [False]*6  # has this knob taken control yet?
MOVE_THRESH  = 0.005      # how far the knob must move (0..1) before it "jumps"

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

# ===================== OLA / DMX =====================

wrapper = ClientWrapper()
client  = wrapper.Client()

dmx_frame      = array('B', [0]*DMX_CHANS)
_last_dmx_send = 0.0

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
        # copy pending into the frame and send from THIS (owner) thread
        for i in range(DMX_CHANS):
            dmx_frame[i] = _pending_vals[i]
        client.SendDmx(UNIVERSE, _CompatBuf(dmx_frame), lambda s: None)
        _last_dmx_send = now
        _dmx_dirty = False

    # reschedule ~every 10–20ms
    wrapper.AddEvent(20, _dmx_pump)  # milliseconds

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

# queued DMX state
_dmx_dirty = False
_pending_vals = [0]*DMX_CHANS

def send_dmx(vals):
    """Queue up to 4 channel DMX; actual SendDmx happens in wrapper thread."""
    global _dmx_dirty, _pending_vals
    for i in range(DMX_CHANS):
        v = max(0, min(255, int(vals[i])))
        _pending_vals[i] = v
    _dmx_dirty = True

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

# Ambient mode
_ambient_next_time = 0.0
def ambient_vals(dt_ms):
    global _ambient_next_time
    now = time.time()

    # --- Dynamic speed control only in ambient preset ---
    if PROGRAM == 4:
        # Normalize center frequency knob (20–11 000 Hz)
        norm = (math.log10(band.center) - math.log10(20)) / (math.log10(11000) - math.log10(20))
        # Wider range: slower lows, gentler highs
        speed_factor = lerp(0.2, 2.0, 1 - norm)  # flipped: right = faster, left = slower
    else:
        speed_factor = 1.0

    # Random trigger interval scaled by speed factor
    base_min, base_max = 0.4 * speed_factor, 1.2 * speed_factor

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
        self.reset(); self._design()
    def reset(self):
        self.x1=self.x2=self.y1=self.y2=0.0
    def set_params(self, center_hz, q):
        self.center = max(20.0,  min(18000.0, float(center_hz)))
        self.q      = max(0.3,   min(12.0,    float(q)))
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
        self.target=target; self.gain=1.0; self.tau=tau
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
            # Recover if SPI got closed during shutdown race: reopen once.
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

# --- smooth + movement detection ---
_knob_ema  = [None]*8
_knob_last = [None]*8
_knob_jumped = [False]*8   # becomes True after first motion per knob
MOVE_EPS   = 0.02          # 2% change counts as a “move”

def read_knob_norm(ch: int, alpha=0.25) -> float:
    raw = read_mcp3008(ch) / 1023.0
    prev = _knob_ema[ch]
    v = raw if prev is None else (alpha*raw + (1-alpha)*prev)
    _knob_ema[ch] = v
    return v

def knob_moved(ch: int, v: float) -> bool:
    """Detect first meaningful knob movement."""
    last = _knob_last[ch]
    moved = (last is not None and abs(v - last) > MOVE_EPS)
    _knob_last[ch] = v
    return moved

# --- parameter mapping functions ---
def map_center(x):
    x = clamp(x, 0.0, 1.0)
    xb = x**0.6
    return 20.0 * ((11000.0/20.0)**xb)
def map_q(x):       return lerp(0.4,  6.0,   clamp(x,0,1))
def map_thresh(x):  return lerp(0.001,0.200, clamp(x,0,1))
def map_attack(x):  return lerp(1.0,  800.0, clamp(x,0,1))
def map_decay(x):   return lerp(5.0,  2000.0,clamp(x,0,1))
def map_bright(x):  return lerp(0.0,  1.0,   clamp(x,0,1))

def _jump_takeover(ch_idx, map_fn, alpha=0.25):
    """
    Read one knob (normalized 0..1 with EMA smoothing). If that knob hasn't
    taken control yet, require a small movement from its last seeded position
    before 'jumping' the parameter to the knob's mapped value.
    Returns: None if not yet jumped, else the mapped value.
    """
    n = read_knob_norm(ch_idx, alpha=alpha)  # 0..1 smoothed
    last = _knob_last[ch_idx]

    # First time we see it, seed and do nothing
    if last is None:
        _knob_last[ch_idx] = n
        return None

    # If not yet jumped, wait for movement past threshold
    if not _knob_jumped[ch_idx]:
        if abs(n - last) >= MOVE_THRESH:
            _knob_jumped[ch_idx] = True
            _knob_last[ch_idx] = n
            return map_fn(n)
        else:
            _knob_last[ch_idx] = n
            return None
    else:
        # Already jumped: always follow knob
        _knob_last[ch_idx] = n
        return map_fn(n)

def update_from_knobs():
    """Jump-on-first-move for all six knobs (CH0..CH5)."""
    global BRIGHTNESS

    v = _jump_takeover(0, map_center)   # center Hz
    if v is not None: band.center = v

    v = _jump_takeover(1, map_q)        # Q
    if v is not None: band.q = v

    v = _jump_takeover(2, map_thresh, alpha=0.20)  # threshold (a bit more smoothing)
    if v is not None: band.thresh = v

    v = _jump_takeover(3, map_attack)   # attack ms
    if v is not None: band.attack_ms = v

    v = _jump_takeover(4, map_decay)    # decay ms
    if v is not None: band.decay_ms = v

    v = _jump_takeover(5, map_bright)   # brightness
    if v is not None: BRIGHTNESS = v

# ===================== Reset Button & Ready LED =====================

_last_reset_msg_time = 0.0
def reset_to_defaults(channel=None):
    global band, BRIGHTNESS, _knob_last, _knob_jumped
    band.center    = DEFAULT_CENTER_HZ
    band.q         = DEFAULT_Q
    band.thresh    = DEFAULT_THRESH
    band.attack_ms = DEFAULT_ATTACK_MS
    band.decay_ms  = DEFAULT_DECAY_MS
    BRIGHTNESS     = DEFAULT_BRIGHT

    # Re-arm “jump-on-first-move”
    for i in range(6):
        _knob_jumped[i] = False

    # Seed last positions so tiny ADC noise doesn’t cause instant jumps
    try:
        current = [read_knob_norm(i) for i in range(6)]
        for i in range(6):
            _knob_last[i] = current[i]
    except Exception:
        pass

    # Show message inside the TUI (no stdout printing)
    ui_flash("[RESET] Restored default parameters.", 1.5)
    
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
      - Poll rotary switch (debounce + map to PROGRAM)
      - Poll reset button (BCM25) and call reset_to_defaults() on 1->0 press
    """
    global PROGRAM, STOP_THREADS

    last_switch = None
    switch_stable = 0

    prev_reset = 1          # pulled-up, so idle=1
    reset_stable = 0
    RESET_DEBOUNCE = 4      # ~4 * 10ms = ~40ms
    SWITCH_DEBOUNCE = SW_DEBOUNCE_SAMPLES

    try:
        while not STOP_THREADS:
            # --- Safe GPIO reads (handle cleanup race on exit) ---
            try:
                s = tuple(GPIO.input(p) for p in SW_PINS)
                r = GPIO.input(RESET_PIN)
            except RuntimeError:
                # mode may have been cleaned up during shutdown
                break

            # --- Rotary switch ---
            if s == last_switch:
                switch_stable += 1
            else:
                last_switch, switch_stable = s, 1
            if switch_stable >= SWITCH_DEBOUNCE:
                prog = SW_MAP.get(s)
                if prog and prog != PROGRAM:
                    PROGRAM = prog

            # --- Reset button (active LOW) ---
            if r == prev_reset:
                reset_stable += 1
            else:
                prev_reset, reset_stable = r, 1

            # Detect a stable press (1 -> 0)
            if prev_reset == 0 and reset_stable == RESET_DEBOUNCE:
                reset_to_defaults()
                time.sleep(0.25)  # hold-off to avoid repeats while held

            time.sleep(SW_SAMPLE_PERIOD_S)  # ~10ms
    finally:
        # Don't cleanup here; other parts of the script still use GPIO
        pass

# ===================== Audio loop =====================

live_band_env   = 0.0
live_threshold  = band.thresh
input_rms       = 0.0
last_trigger_ts = 0.0
chase_idx       = 0

bp   = None
envd = None
agc  = Agc(target=AGC_TARGET, tau=0.95)

def audio_loop():
    global bp, envd, live_band_env, live_threshold, input_rms
    global last_trigger_ts, chase_idx, PROGRAM

    bp   = BiquadBandpass(SR, band.center, band.q)
    envd = EnvDetector(SR, attack_ms=8.0, release_ms=80.0)

    frame_dt_ms = (HOP / SR) * 1000.0
    was_above = False

    # LED ON once stream is running
    GPIO.output(READY_LED_PIN, GPIO.HIGH)

    def cb(indata, frames, time_info, status):
        nonlocal was_above
        global live_band_env, live_threshold, input_rms, last_trigger_ts, chase_idx, PROGRAM

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

        if above and not was_above and can_fire and PROGRAM in (1,2,3):
            last_trigger_ts = now

            if PROGRAM == 1:  # ALL
                trigger_idxs([0,1,2,3], band.attack_ms, band.decay_ms)

            elif PROGRAM == 2:  # CHASE
                trigger_idxs([chase_idx], band.attack_ms, band.decay_ms)
                chase_idx = (chase_idx + 1) % 4

            elif PROGRAM == 3:  # RANDOM JUMP
                rand_idx = random.randint(0, 3)
                trigger_idxs([rand_idx], band.attack_ms, band.decay_ms)

        # DMX output
        send_dmx(ambient_vals(frame_dt_ms) if PROGRAM == 4 else update_lights(frame_dt_ms))

    try:
        with sd.InputStream(device=DEVICE_INDEX, channels=2, samplerate=SR, blocksize=HOP, callback=cb):
            while not STOP_THREADS:
                time.sleep(0.05)
    finally:
        # LED OFF if audio thread exits
        GPIO.output(READY_LED_PIN, GPIO.LOW)

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
    left_hz, right_hz = 20.0, 11000.0
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
    # Non-blocking getch with ~33 ms timeout (≈30 FPS), no manual sleep
    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.timeout(33)   # getch waits up to 33 ms; avoids KeyboardInterrupt during sleep

    last = time.time()

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        bar_width = max(20, min(65, w - 2))

        safe_addstr(stdscr, 0, 0,
            f"Program {PROGRAM}   RUN={'ON' if RUNNING else 'PAUSE'}   Device={DEVICE_NAME}  SR={SR}  HOP={HOP}")
        safe_addstr(stdscr, 1, 0,
            f"ENV_EMA={ENV_EMA:.2f}  AGC={'ON' if AGC_ON else 'OFF'} target={AGC_TARGET:.3f}  "
            f"Refractory={REFRACTORY_MS:.0f}ms  Weighting={'ON' if WEIGHTING_ON else 'OFF'}")
        safe_addstr(stdscr, 2, 0, "Press 'q' or ESC to quit • Press RESET (BCM25) to restore defaults")

        # params
        row = 4
        labels_vals = [
            ("center (Hz)",  band.center),
            ("Q",            band.q),
            ("threshold",    band.thresh),
            ("attack (ms)",  band.attack_ms),
            ("decay (ms)",   band.decay_ms),
            ("brightness",   BRIGHTNESS),
        ]
        safe_addstr(stdscr, row-1, 0, "Params (knobs with soft-takeover):")
        for label, val in labels_vals:
            safe_addstr(stdscr, row, 2, f"{label:<12}: {val:>8.3f}")
            row += 1

        safe_addstr(stdscr, row+1, 0, "Band Env vs Threshold (| is threshold):")
        draw_threshold_meter(stdscr, row+2, 0, bar_width, live_band_env, band.thresh)
        safe_addstr(stdscr, row+3, 0, f"env={live_band_env:.4f}  thresh={band.thresh:.4f} (scale ~0.20)")

        safe_addstr(stdscr, row+5, 0, "Targeted Frequency Band:")
        draw_band_bar(stdscr, row+6, 0, bar_width, band.center, band.q)

        safe_addstr(stdscr, row+8, 0, "Channels:")
        for i, s in enumerate(states, start=1):
            safe_addstr(stdscr, row+8+i, 1,
                        f"ch{i}: env={s.env:.3f} post={s.post:.3f} stage={'on' if s.active else 'idle'}")

        # transient UI flash at bottom line
        if time.time() < _ui_flash_until and _ui_flash_msg:
            h, w = stdscr.getmaxyx()
            msg = _ui_flash_msg
            x = max(0, (w - len(msg)) // 2)
            stdscr.addnstr(h - 1, x, msg, max(0, w - x))

        stdscr.refresh()

        # Key handling
        ch = stdscr.getch()
        if ch in (ord('q'), ord('Q'), 27):  # 27 = ESC
            # make the pump stop the wrapper loop
            _set_stop(True)
            _set_run(False)
            break

        last = time.time()

# ===================== Main =====================

def main():

    print(f"[OK] Using input: {DEVICE_INDEX} - {DEVICE_NAME}")
    print("[OK] Knobs CH0..CH5 with jump-on-move.")
    print(f"[OK] DMX -> Universe {UNIVERSE}, Channels 1..4")

    setup_gpio_inputs()

    # Start app threads (switch + audio)
    th_sw = threading.Thread(target=switch_reader)
    th_sw.start()

    th_audio = threading.Thread(target=audio_loop)
    th_audio.start()

    # If TUI is enabled, run it in its own thread so main thread can own OLA
    use_tui = sys.stdout.isatty() and os.environ.get("ENABLE_TUI", "1") == "1"
    th_tui = None
    if use_tui:
        th_tui = threading.Thread(target=lambda: curses.wrapper(tui))
        th_tui.start()
    else:
        print("[INFO] No TTY detected (or ENABLE_TUI=0). Running headless.")

    # Kick off the DMX pump and run the OLA loop IN THE MAIN THREAD
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

        try: wrapper.Stop()
        except Exception: pass

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
