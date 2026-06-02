"""Render the moderngl studio viewport at a few poses to PNG (visual validation)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PySide6.QtWidgets import QApplication

from kia_studio.ui.widgets.viewport_gl import GLViewport, ensure_gl_format

POSES = {"home": (0, 90, 0, 0, 0), "reach": (35, 70, 40, -10, 30)}


def main():
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "render_out")
    os.makedirs(out, exist_ok=True)
    ensure_gl_format()
    app = QApplication.instance() or QApplication([])
    v = GLViewport(); v.resize(960, 720); v.show()
    for _ in range(25):
        app.processEvents()
    for name, pose in POSES.items():
        v.set_pose(*pose)
        for _ in range(8):
            app.processEvents()
        v.grabFramebuffer().save(os.path.join(out, f"gl_{name}.png"))
        print("saved", name)
    v.close()


if __name__ == "__main__":
    main()
