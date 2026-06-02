"""Offscreen-ish render harness for visual validation of the 3D viewport.

Boots a real (briefly shown) GLViewWidget, renders the arm at several poses, and saves
PNGs. Used to validate mesh calibration against the FK skeleton without a live device.

Usage:  python tools/render_arm.py [out_dir]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PySide6.QtWidgets import QApplication

from kia_studio.ui.widgets.viewport3d import Viewport3D

POSES = {
    "home":   (0.0, 90.0, 0.0, 0.0),
    "reach":  (30.0, 60.0, -30.0, 10.0),
    "folded": (-45.0, 120.0, -60.0, 30.0),
    "up":     (0.0, 30.0, 20.0, 0.0),
}


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
    os.makedirs(out, exist_ok=True)
    app = QApplication.instance() or QApplication([])
    view = Viewport3D()
    view.resize(900, 700)
    view.show()
    for _ in range(15):
        app.processEvents()
    for name, pose in POSES.items():
        view.set_pose(*pose)
        for _ in range(8):
            app.processEvents()
        img = view.grabFramebuffer()
        path = os.path.join(out, f"arm_{name}.png")
        img.save(path)
        print(f"saved {path}  ({img.width()}x{img.height()})")
    view.close()


if __name__ == "__main__":
    main()
