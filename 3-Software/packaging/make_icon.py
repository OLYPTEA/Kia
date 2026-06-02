"""Generate the Kia Studio app icon — a clean minimal robot-arm mark (azure on dark).

Writes a multi-resolution .ico to packaging/kia.ico and kia_studio/resources/kia.ico,
plus a PNG preview. Run: python packaging/make_icon.py
"""
import os

from PIL import Image, ImageDraw

S = 512
ACCENT = (45, 155, 219, 255)      # #2d9bdb
DARK = (20, 23, 29, 255)          # surface
MARGIN = 22


def _round(d, p, r, **kw):
    d.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], **kw)


def render() -> Image.Image:
    img = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([8, 8, S - 8, S - 8], radius=108, fill=DARK)

    base, elbow, wrist = (170, 392), (212, 232), (348, 196)
    lw = 40
    # links (round joins via end caps)
    d.line([base, elbow], fill=ACCENT, width=lw)
    d.line([elbow, wrist], fill=ACCENT, width=lw)
    for p in (base, elbow, wrist):
        _round(d, p, lw // 2, fill=ACCENT)

    # base plate
    d.rounded_rectangle([112, 392, 228, 426], radius=14, fill=ACCENT)
    # joint highlights (hollow dots)
    _round(d, elbow, 17, fill=DARK)
    _round(d, base, 17, fill=DARK)
    # gripper fork at the wrist tip
    tx, ty = wrist
    d.line([(tx, ty), (tx + 70, ty - 40)], fill=ACCENT, width=30)
    fx, fy = tx + 78, ty - 46
    d.line([(fx - 16, fy - 30), (fx + 22, fy - 8)], fill=ACCENT, width=20)
    d.line([(fx - 2, fy + 26), (fx + 36, fy + 4)], fill=ACCENT, width=20)
    return img


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    res = os.path.join(os.path.dirname(here), "kia_studio", "resources")
    os.makedirs(res, exist_ok=True)
    img = render()
    img.save(os.path.join(here, "kia_preview.png"))
    sizes = [(s, s) for s in (16, 24, 32, 48, 64, 128, 256)]
    for path in (os.path.join(here, "kia.ico"), os.path.join(res, "kia.ico")):
        img.save(path, sizes=sizes)
        print("wrote", path)


if __name__ == "__main__":
    main()
