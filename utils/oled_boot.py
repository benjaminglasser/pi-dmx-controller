#!/usr/bin/env python3
import time, subprocess
try:
    import board, busio
    from PIL import Image, ImageDraw, ImageFont
    import adafruit_ssd1305
except Exception:
    raise SystemExit(0)

LOGO = "/home/pi/pi-dmx-controller/assets/logo.jpg"  # your JPEG
W, H = 128, 32
HOLD_SECONDS = 2.0   # show logo up to this long before switching

def safe_font(size=8):
    try:
        return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", size)
    except Exception:
        return ImageFont.load_default()

def draw_logo(oled):
    from PIL import Image
    try:
        img = Image.open(LOGO).convert("1")
        if (img.width, img.height) != (W, H):
            img = img.resize((W, H))
        oled.image(img); oled.show()
        return True
    except Exception:
        return False

def draw_neutral(oled, tick=0):
    image = Image.new("1", (W, H))
    d = ImageDraw.Draw(image)
    font = safe_font(8)
    # Minimal, abstract “Starting” with subtle animated dot
    d.rectangle((0,0,W-1,H-1), outline=1, fill=0)
    d.text((6, 6), "Starting…", font=font, fill=1)
    dot_x = 90 + (tick % 3) * 8
    d.ellipse((dot_x, 7, dot_x+3, 10), outline=1, fill=1)
    oled.image(image); oled.show()

def is_main_active():
    """Return True once the main service is running."""
    try:
        r = subprocess.run(
            ["systemctl", "is-active", "dmx_audio_react.service"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, timeout=0.6
        )
        return r.stdout.strip() == "active"
    except Exception:
        return False

def main():
    t0 = time.time()
    oled = None
    while time.time() - t0 < 3.0:
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            oled = adafruit_ssd1305.SSD1305_I2C(W, H, i2c, addr=0x3D)
            break
        except Exception:
            time.sleep(0.15)
    if oled is None:
        raise SystemExit(0)

    draw_logo(oled)

    # --- Phase 1: show logo briefly, exit early if main app starts
    end = time.time() + HOLD_SECONDS
    while time.time() < end:
        if is_main_active():
            return
        time.sleep(0.1)

    # --- Phase 2: subtle “Starting…” pulse until main app starts (max ~6s)
    start = time.time()
    while (time.time() - start) < 6.0:
        if is_main_active():
            return
        draw_neutral(oled, int((time.time() - start) * 10))
        time.sleep(0.1)

if __name__ == "__main__":
    main()
