#!/usr/bin/env python3
"""Generate ProjectTeleopMac app icon (.icns) from scratch.

Design: MG400 robot arm silhouette with PT monogram on dark teal background.
Outputs ProjectTeleopMac.icns next to this script.
"""
from __future__ import annotations

import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter, ImageFont

HERE = Path(__file__).resolve().parent
ICONSET = HERE / "ProjectTeleopMac.iconset"
ICNS = HERE / "ProjectTeleopMac.icns"

SIZES = [
    (16, 1), (16, 2),
    (32, 1), (32, 2),
    (128, 1), (128, 2),
    (256, 1), (256, 2),
    (512, 1), (512, 2),
]

BG_TOP = (10, 38, 52)
BG_BOTTOM = (4, 18, 26)
ACCENT = (54, 196, 220)
ARM = (224, 234, 240)
ARM_DARK = (170, 188, 200)
JOINT = (40, 60, 70)


def vgradient(size: int) -> Image.Image:
    img = Image.new("RGB", (size, size), BG_TOP)
    draw = ImageDraw.Draw(img)
    for y in range(size):
        t = y / max(size - 1, 1)
        r = int(BG_TOP[0] * (1 - t) + BG_BOTTOM[0] * t)
        g = int(BG_TOP[1] * (1 - t) + BG_BOTTOM[1] * t)
        b = int(BG_TOP[2] * (1 - t) + BG_BOTTOM[2] * t)
        draw.line([(0, y), (size, y)], fill=(r, g, b))
    return img


def rounded_mask(size: int, radius: int) -> Image.Image:
    mask = Image.new("L", (size, size), 0)
    draw = ImageDraw.Draw(mask)
    draw.rounded_rectangle([(0, 0), (size, size)], radius=radius, fill=255)
    return mask


def draw_arm(img: Image.Image) -> None:
    """Draw stylized MG400 SCARA arm silhouette."""
    size = img.size[0]
    draw = ImageDraw.Draw(img, "RGBA")

    s = size / 1024.0

    def px(v: float) -> int:
        return int(round(v * s))

    # Base column
    base_cx = px(512)
    base_top_y = px(640)
    base_bot_y = px(820)
    base_w = px(180)
    draw.rounded_rectangle(
        [(base_cx - base_w // 2, base_top_y),
         (base_cx + base_w // 2, base_bot_y)],
        radius=px(28),
        fill=ARM,
    )
    # Base shadow strip
    draw.rounded_rectangle(
        [(base_cx - base_w // 2, base_bot_y - px(40)),
         (base_cx + base_w // 2, base_bot_y)],
        radius=px(20),
        fill=ARM_DARK,
    )

    # J1 shoulder joint
    j1 = (base_cx, base_top_y)
    j1_r = px(70)
    draw.ellipse([(j1[0] - j1_r, j1[1] - j1_r), (j1[0] + j1_r, j1[1] + j1_r)], fill=ARM)
    draw.ellipse([(j1[0] - px(22), j1[1] - px(22)), (j1[0] + px(22), j1[1] + px(22))], fill=ACCENT)

    # Upper link (J1 -> J2): up + slight right
    j2 = (px(640), px(420))
    link_w = px(70)
    draw_thick_line(draw, j1, j2, link_w, ARM)
    # J2 elbow
    j2_r = px(60)
    draw.ellipse([(j2[0] - j2_r, j2[1] - j2_r), (j2[0] + j2_r, j2[1] + j2_r)], fill=ARM)
    draw.ellipse([(j2[0] - px(18), j2[1] - px(18)), (j2[0] + px(18), j2[1] + px(18))], fill=ACCENT)

    # Forearm (J2 -> J3): more horizontal, to upper-left
    j3 = (px(360), px(290))
    draw_thick_line(draw, j2, j3, link_w, ARM)
    j3_r = px(54)
    draw.ellipse([(j3[0] - j3_r, j3[1] - j3_r), (j3[0] + j3_r, j3[1] + j3_r)], fill=ARM)
    draw.ellipse([(j3[0] - px(16), j3[1] - px(16)), (j3[0] + px(16), j3[1] + px(16))], fill=ACCENT)

    # End effector vertical shaft + flange
    flange_top = (j3[0], j3[1] + px(8))
    flange_bot = (j3[0], j3[1] + px(150))
    draw.rounded_rectangle(
        [(flange_top[0] - px(20), flange_top[1]),
         (flange_top[0] + px(20), flange_bot[1])],
        radius=px(8),
        fill=ARM_DARK,
    )
    # Tool tip
    tip_r = px(28)
    draw.ellipse(
        [(flange_bot[0] - tip_r, flange_bot[1] - tip_r),
         (flange_bot[0] + tip_r, flange_bot[1] + tip_r)],
        fill=ACCENT,
    )


def draw_thick_line(draw: ImageDraw.ImageDraw, a, b, width: int, fill) -> None:
    draw.line([a, b], fill=fill, width=width)
    # Cap ends with circles so joints look smooth at any angle
    r = width // 2
    for p in (a, b):
        draw.ellipse([(p[0] - r, p[1] - r), (p[0] + r, p[1] + r)], fill=fill)


def draw_monogram(img: Image.Image) -> None:
    size = img.size[0]
    draw = ImageDraw.Draw(img, "RGBA")

    s = size / 1024.0

    def px(v: float) -> int:
        return int(round(v * s))

    text = "PT"
    font = load_font(int(280 * s))
    bbox = draw.textbbox((0, 0), text, font=font)
    tw = bbox[2] - bbox[0]
    th = bbox[3] - bbox[1]
    cx = size // 2
    cy = px(880)
    x = cx - tw // 2 - bbox[0]
    y = cy - th // 2 - bbox[1]

    # Subtle accent underline behind text
    underline_y = cy + px(110)
    draw.rounded_rectangle(
        [(cx - px(150), underline_y),
         (cx + px(150), underline_y + px(14))],
        radius=px(7),
        fill=ACCENT,
    )

    draw.text((x, y), text, fill=ARM, font=font)


def load_font(size: int) -> ImageFont.FreeTypeFont:
    candidates = [
        "/System/Library/Fonts/SFNSDisplay-Bold.otf",
        "/System/Library/Fonts/Supplemental/Futura.ttc",
        "/System/Library/Fonts/HelveticaNeue.ttc",
        "/Library/Fonts/Arial Bold.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
    ]
    for path in candidates:
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
    return ImageFont.load_default()


def make_master(size: int) -> Image.Image:
    rgb = vgradient(size)
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    img.paste(rgb, (0, 0))
    draw_arm(img)
    draw_monogram(img)

    # Round corners for non-iconset standalone preview; iconutil overrides this
    radius = int(size * 0.22)
    mask = rounded_mask(size, radius)
    rounded = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    rounded.paste(img, (0, 0), mask)
    return rounded


def main() -> int:
    if ICONSET.exists():
        shutil.rmtree(ICONSET)
    ICONSET.mkdir(parents=True)

    master = make_master(1024)

    for nominal, scale in SIZES:
        side = nominal * scale
        resized = master.resize((side, side), Image.LANCZOS)
        suffix = "" if scale == 1 else "@2x"
        name = f"icon_{nominal}x{nominal}{suffix}.png"
        resized.save(ICONSET / name, "PNG")

    # macOS iconutil
    if ICNS.exists():
        ICNS.unlink()
    res = subprocess.run(
        ["iconutil", "-c", "icns", str(ICONSET), "-o", str(ICNS)],
        capture_output=True,
        text=True,
    )
    if res.returncode != 0:
        sys.stderr.write(res.stderr)
        return res.returncode

    print(f"wrote {ICNS}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
