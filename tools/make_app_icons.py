#!/usr/bin/env python3
"""Draw the app icons from code so they can be tweaked and regenerated.

    python tools/make_app_icons.py      (needs Pillow; only for regenerating)

RPBot Studio: top-down robot with LiDAR scan arcs on the studio's ink and blue.
DAPLink Flasher: MCU chip with a flash bolt on the flasher's accent blue.
Each app gets a multi-size .ico (Windows window, taskbar, EXE), a small .png for the header
and a 256 px .png for the Linux window and launcher.
"""
from pathlib import Path

from PIL import Image, ImageDraw

REPO = Path(__file__).resolve().parent.parent
CANVAS = 1024
ICO_SIZES = (16, 20, 24, 32, 40, 48, 64, 96, 128, 256)

INK = (16, 16, 20, 255)          # studio_theme.INK
INK_EDGE = (58, 58, 66, 255)
BLUE = (45, 129, 255, 255)       # studio_theme.BLUE
ACCENT = (22, 89, 196, 255)      # dap_flasher light accent
WHITE = (255, 255, 255, 255)


def tile(fill, edge=None):
    image = Image.new('RGBA', (CANVAS, CANVAS), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)
    draw.rounded_rectangle((16, 16, CANVAS - 17, CANVAS - 17), radius=210, fill=fill,
                           outline=edge, width=20 if edge else 0)
    return image, draw


def disc(draw, x, y, r, fill):
    draw.ellipse((x - r, y - r, x + r, y + r), fill=fill)


def arc(draw, x, y, r, start, end, width, fill):
    """Arc with round caps; angles in degrees, clockwise from 3 o'clock like Pillow."""
    import math
    draw.arc((x - r, y - r, x + r, y + r), start, end, fill=fill, width=width)
    mid = r - width / 2
    for angle in (start, end):
        a = math.radians(angle)
        disc(draw, x + mid * math.cos(a), y + mid * math.sin(a), width / 2, fill)


def rpbot_studio():
    image, draw = tile(INK, INK_EDGE)
    cx, cy = 512, 640
    for radius in (330, 455):
        arc(draw, cx, cy, radius, 222, 318, 58, BLUE)
    for side in (-1, 1):
        x = cx + side * 250
        draw.rounded_rectangle((x - 34, cy - 92, x + 34, cy + 92), radius=26, fill=WHITE)
    disc(draw, cx, cy, 246, INK)
    disc(draw, cx, cy, 218, WHITE)
    disc(draw, cx, cy, 104, INK)
    disc(draw, cx, cy, 70, BLUE)
    return image


def dap_flasher(small=False):
    """small: fewer, wider pins and a bigger bolt so 16-24 px stays readable."""
    shape, _ = tile(WHITE)
    fade = Image.linear_gradient('L').resize((CANVAS, CANVAS))
    image = Image.composite(Image.new('RGBA', (CANVAS, CANVAS), ACCENT), Image.new('RGBA', (CANVAS, CANVAS), BLUE), fade)
    image.putalpha(shape.getchannel('A'))
    draw = ImageDraw.Draw(image)
    lo, hi, pin = (222, 802, 100) if small else (262, 762, 96)
    half = 52 if small else 26
    for offset in ((-150, 150) if small else (-180, -60, 60, 180)):
        c = 512 + offset
        draw.rounded_rectangle((c - half, lo - pin, c + half, hi + pin), radius=22, fill=WHITE)
        draw.rounded_rectangle((lo - pin, c - half, hi + pin, c + half), radius=22, fill=WHITE)
    draw.rounded_rectangle((lo - 26, lo - 26, hi + 26, hi + 26), radius=60, fill=ACCENT)
    draw.rounded_rectangle((lo, lo, hi, hi), radius=46, fill=WHITE)
    box = (lo + 90, lo + 40, hi - 90, hi - 40) if small else (lo + 120, lo + 70, hi - 120, hi - 70)
    w, h = box[2] - box[0], box[3] - box[1]
    bolt = [(0.62, 0), (0.08, 0.57), (0.44, 0.57), (0.34, 1), (0.92, 0.40), (0.56, 0.40), (0.78, 0)]
    draw.polygon([(box[0] + u * w, box[1] + v * h) for u, v in bolt], fill=ACCENT)
    if not small:
        disc(draw, lo + 62, lo + 62, 22, ACCENT)
    return image


def save(image, folder, name, mark_size, small=None):
    folder.mkdir(exist_ok=True)
    frames = [(small if small and size <= 24 else image).resize((size, size), Image.LANCZOS) for size in ICO_SIZES]
    frames[-1].save(folder / f'{name}.ico', sizes=[(s, s) for s in ICO_SIZES], append_images=frames[:-1])
    image.resize((mark_size, mark_size), Image.LANCZOS).save(folder / f'{name}.png', optimize=True)
    frames[-1].save(folder / f'{name}-256.png', optimize=True)  # Linux window and launcher icon
    print('wrote', folder / f'{name}.ico', folder / f'{name}.png', folder / f'{name}-256.png')


if __name__ == '__main__':
    save(rpbot_studio(), REPO / 'workstation' / 'assets', 'rpbot-studio', 40)
    save(dap_flasher(), REPO / 'tools' / 'assets', 'dap-flasher', 44, dap_flasher(small=True))
