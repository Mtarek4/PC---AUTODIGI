"""Draws the XD ribbon icons (32x32 large + 16x16 small) with supersampling. Run: python3 make_icons.py <outdir>"""
import math, sys
from PIL import Image, ImageDraw

NAVY = (31, 58, 95, 255); TEAL = (0, 150, 150, 255); ORANGE = (242, 140, 40, 255); RED = (214, 69, 69, 255)
LIGHT = (232, 238, 244, 255); WHITE = (255, 255, 255, 255); DARK = (34, 48, 60, 255); GRAY = (120, 135, 150, 255)

def render(draw_fn, size):
    S = 16                                    # supersampling factor
    px = size * S
    img = Image.new("RGBA", (px, px), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    u = px / 100.0                            # 100 x 100 design grid
    small = size <= 16
    draw_fn(d, u, small)
    return img.resize((size, size), Image.LANCZOS)

def rr(d, u, x0, y0, x1, y1, fill=None, outline=None, w=0, r=4):
    d.rounded_rectangle([x0 * u, y0 * u, x1 * u, y1 * u], radius=r * u, fill=fill, outline=outline, width=int(w * u))

def line(d, u, pts, fill, w):
    d.line([(x * u, y * u) for x, y in pts], fill=fill, width=int(w * u), joint="curve")

def dot(d, u, x, y, r, fill, ring=WHITE, ringw=0):
    if ringw: d.ellipse([(x - r - ringw) * u, (y - r - ringw) * u, (x + r + ringw) * u, (y + r + ringw) * u], fill=ring)
    d.ellipse([(x - r) * u, (y - r) * u, (x + r) * u, (y + r) * u], fill=fill)

def dashed(d, u, x0, y0, x1, y1, fill, w, dash, gap):
    L = math.hypot(x1 - x0, y1 - y0); n = int(L // (dash + gap)) + 1
    for i in range(n):
        a = i * (dash + gap) / L; b = min(1.0, (i * (dash + gap) + dash) / L)
        if a >= 1: break
        line(d, u, [(x0 + (x1 - x0) * a, y0 + (y1 - y0) * a), (x0 + (x1 - x0) * b, y0 + (y1 - y0) * b)], fill, w)

def wall_band(d, u, small, y0=34, y1=66):
    rr(d, u, 6, y0, 94, y1, fill=LIGHT, outline=NAVY, w=(9 if small else 6), r=3)

# ---------------------------------------------------------------- the icons
def both_sides(d, u, s):
    wall_band(d, u, s)
    dot(d, u, 30, 34, 10 if s else 8, ORANGE, ringw=3); dot(d, u, 70, 66, 10 if s else 8, ORANGE, ringw=3)   # one click on EACH face
    line(d, u, [(30, 34), (30, 14)], ORANGE, 6 if s else 4); line(d, u, [(70, 66), (70, 86)], ORANGE, 6 if s else 4)

def centerline(d, u, s):
    rr(d, u, 6, 28, 94, 72, fill=LIGHT, outline=NAVY, w=(9 if s else 6), r=3)
    dashed(d, u, 24, 50, 76, 50, TEAL, 8 if s else 6, 11, 8)
    dot(d, u, 20, 50, 9 if s else 8, TEAL, ringw=3); dot(d, u, 80, 50, 9 if s else 8, TEAL, ringw=3)

def diagonal(d, u, s):
    wall_band(d, u, s)
    line(d, u, [(16, 66), (84, 34)], ORANGE, 9 if s else 6)
    dot(d, u, 16, 66, 11 if s else 9, ORANGE, ringw=3); dot(d, u, 84, 34, 11 if s else 9, ORANGE, ringw=3)

def wall_auto(d, u, s):
    # two walls meeting at a corner (a plan, not one wall) and a spark instead of click dots: found without clicking
    w = 9 if s else 6
    rr(d, u, 6, 56, 94, 84, fill=LIGHT, outline=NAVY, w=w, r=3)
    rr(d, u, 6, 14, 34, 84, fill=LIGHT, outline=NAVY, w=w, r=3)
    d.rectangle([(6 + w) * u, (56 + w) * u, 40 * u, (84 - w) * u], fill=LIGHT)      # open the corner: one L-shaped body
    cx, cy, R, r = 70, 28, (27 if s else 25), (9 if s else 8)
    pts = [(cx + (R if i % 2 == 0 else r) * math.cos(math.pi * i / 4 - math.pi / 2), cy + (R if i % 2 == 0 else r) * math.sin(math.pi * i / 4 - math.pi / 2)) for i in range(8)]
    d.polygon([(x * u, y * u) for x, y in pts], fill=ORANGE)

def publish(d, u, s):
    # left: a filled T-junction (Revit)   ->   right: the same T as thin lines on a dark canvas (CAD)
    def T(x, y, k, fill, outline, w):
        bar, stem = 14 * k, 16 * k
        pts = [(x, y), (x + 34 * k, y), (x + 34 * k, y + bar), (x + 25 * k, y + bar), (x + 25 * k, y + 40 * k),
               (x + 9 * k, y + 40 * k), (x + 9 * k, y + bar), (x, y + bar)]
        if fill is not None: d.polygon([(a * u, b * u) for a, b in pts], fill=fill)
        d.line([(a * u, b * u) for a, b in pts + [pts[0]]], fill=outline, width=int(w * u), joint="curve")
    T(4, 28, 1.0, LIGHT, NAVY, 6 if s else 4.5)
    rr(d, u, 58, 14, 97, 86, fill=DARK, outline=GRAY, w=3, r=5)
    T(60.5, 30, 1.0, None, WHITE, 4.5 if s else 3)
    line(d, u, [(38, 64), (52, 64)], ORANGE, 10 if s else 8)
    d.polygon([(50 * u, 50 * u), (50 * u, 78 * u), (67 * u, 64 * u)], fill=ORANGE)

def settings(d, u, s):
    cx, cy = 50, 50
    pts = []
    teeth = 8
    for i in range(teeth * 2):
        a0 = math.pi * 2 * i / (teeth * 2) - math.pi / (teeth * 2) * 0.0
        r = 44 if i % 2 == 0 else 33
        for da in (-0.5, 0.5):
            a = math.pi * 2 * (i + da) / (teeth * 2) * 1.0
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    d.polygon([(x * u, y * u) for x, y in pts], fill=NAVY)
    dot(d, u, cx, cy, 15, LIGHT); dot(d, u, cx, cy, 8, NAVY) if not s else None

def diagnostics(d, u, s):
    # magnifier with a pulse line
    d.ellipse([12 * u, 10 * u, 70 * u, 68 * u], fill=LIGHT, outline=NAVY, width=int((10 if s else 7) * u))
    line(d, u, [(60, 60), (88, 88)], NAVY, 13 if s else 10)
    line(d, u, [(20, 40), (30, 40), (36, 26), (44, 54), (50, 40), (60, 40)], TEAL, 8 if s else 5.5)

def col_setup(d, u, s):
    dashed(d, u, 6, 16, 94, 16, TEAL, 6 if s else 4, 10, 6)
    dashed(d, u, 6, 84, 94, 84, TEAL, 6 if s else 4, 10, 6)
    d.rectangle([32 * u, 16 * u, 60 * u, 84 * u], fill=LIGHT, outline=NAVY, width=int((9 if s else 6) * u))
    x = 80
    line(d, u, [(x, 26), (x, 74)], ORANGE, 8 if s else 6)
    d.polygon([((x - 9) * u, 34 * u), ((x + 9) * u, 34 * u), (x * u, 16 * u)], fill=ORANGE)
    d.polygon([((x - 9) * u, 66 * u), ((x + 9) * u, 66 * u), (x * u, 84 * u)], fill=ORANGE)

def wall_setup(d, u, s):
    # a wall between two level lines, with a height arrow
    dashed(d, u, 6, 16, 94, 16, TEAL, 6 if s else 4, 10, 6)
    dashed(d, u, 6, 84, 94, 84, TEAL, 6 if s else 4, 10, 6)
    rr(d, u, 20, 16, 52, 84, fill=LIGHT, outline=NAVY, w=(9 if s else 6), r=2)
    x = 76
    line(d, u, [(x, 26), (x, 74)], ORANGE, 8 if s else 6)
    d.polygon([((x - 9) * u, 34 * u), ((x + 9) * u, 34 * u), (x * u, 16 * u)], fill=ORANGE)
    d.polygon([((x - 9) * u, 66 * u), ((x + 9) * u, 66 * u), (x * u, 84 * u)], fill=ORANGE)

def col_center(d, u, s):
    d.rectangle([12 * u, 12 * u, 88 * u, 88 * u], outline=NAVY, width=int((9 if s else 6) * u))
    dot(d, u, 50, 50, 9 if s else 8, ORANGE, ringw=3)

def col_diagonal(d, u, s):
    d.rectangle([12 * u, 12 * u, 88 * u, 88 * u], outline=NAVY, width=int((9 if s else 6) * u))
    dot(d, u, 12, 12, 9 if s else 8, ORANGE, ringw=3); dot(d, u, 88, 88, 9 if s else 8, ORANGE, ringw=3)
    line(d, u, [(12, 12), (88, 88)], ORANGE, 6 if s else 4)

def col_sides(d, u, s):
    d.rectangle([12 * u, 12 * u, 88 * u, 88 * u], outline=NAVY, width=int((9 if s else 6) * u))
    dot(d, u, 12, 50, 9 if s else 8, ORANGE, ringw=3); dot(d, u, 88, 50, 9 if s else 8, ORANGE, ringw=3)

def door_window(d, u, s):
    # a wall line with a gap (the opening) and a door-swing arc - reads as "find the opening in the wall"
    w = 9 if s else 6
    line(d, u, [(8, 50), (36, 50)], NAVY, w)
    line(d, u, [(64, 50), (92, 50)], NAVY, w)
    d.arc([16 * u, 18 * u, 80 * u, 82 * u], start=270, end=340, fill=ORANGE, width=int((7 if s else 5) * u))
    line(d, u, [(36, 50), (36, 16)], ORANGE, 6 if s else 4)
    dot(d, u, 36, 50, 6 if s else 5, ORANGE)

def door_window_setup(d, u, s):
    dashed(d, u, 6, 16, 94, 16, TEAL, 6 if s else 4, 10, 6)
    dashed(d, u, 6, 84, 94, 84, TEAL, 6 if s else 4, 10, 6)
    line(d, u, [(15, 50), (38, 50)], NAVY, 8 if s else 6)
    line(d, u, [(62, 50), (85, 50)], NAVY, 8 if s else 6)
    d.arc([12 * u, 22 * u, 64 * u, 78 * u], start=270, end=340, fill=ORANGE, width=int((7 if s else 5) * u))

ICONS = {"wall_both": both_sides, "wall_center": centerline, "wall_diag": diagonal, "wall_auto": wall_auto,
         "wall_setup": wall_setup, "col_setup": col_setup, "col_center": col_center, "col_diagonal": col_diagonal, "col_sides": col_sides,
         "door_window": door_window, "door_window_setup": door_window_setup,
         "publish_cad": publish, "settings": settings, "diagnostics": diagnostics}

if __name__ == "__main__":
    out = sys.argv[1]
    for name, fn in ICONS.items():
        for size in (32, 16):
            render(fn, size).save(f"{out}/{name}_{size}.png")
    # preview sheet (scaled up so the icons can be checked by eye)
    sheet = Image.new("RGBA", (len(ICONS) * 140, 300), (245, 245, 245, 255))
    for i, (name, fn) in enumerate(ICONS.items()):
        sheet.paste(render(fn, 128), (i * 140 + 6, 6), render(fn, 128))
        sheet.paste(render(fn, 32).resize((96, 96), Image.NEAREST), (i * 140 + 6, 140), render(fn, 32).resize((96, 96), Image.NEAREST))
        sheet.paste(render(fn, 16).resize((48, 48), Image.NEAREST), (i * 140 + 90, 236), render(fn, 16).resize((48, 48), Image.NEAREST))
    sheet.save(f"{out}/_preview.png")
