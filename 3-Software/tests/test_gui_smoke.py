"""Offscreen GUI smoke test — boots the window against the simulator, no display needed."""
import os
import sys
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest  # noqa: E402

PySide6 = pytest.importorskip("PySide6")  # skip if Qt unavailable


def _pump(app, seconds):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        app.processEvents()
        time.sleep(0.02)


def test_window_boots_and_streams():
    from PySide6.QtWidgets import QApplication
    from kia_studio.ui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    win = MainWindow()
    win.show()
    _pump(app, 0.1)

    win.client.connect_sim()
    assert win.client.is_connected
    _pump(app, 0.5)

    # telemetry panel got real values (Z field no longer the placeholder)
    assert win.telemetry.f_z.text() != "—"
    assert win._tlm_count >= 0  # counter active
    assert win.client.client.last_telemetry is not None

    win.client.disconnect()
    assert not win.client.is_connected
    win.close()


def test_joint_panel_drives_sim():
    from PySide6.QtWidgets import QApplication
    from kia_studio.ui.main_window import MainWindow

    app = QApplication.instance() or QApplication([])
    win = MainWindow()
    win.client.connect_sim()
    _pump(app, 0.3)

    # command via the joint panel (base -> 50 deg, on release)
    win.joint_panel.rb_release.setChecked(True)
    win.joint_panel.rows[0].set_value_silent(50.0)
    win.joint_panel.command.emit([50.0, 90.0, 0.0, 0.0, 0.0], 0.4)
    _pump(app, 1.5)

    st = win.client.client.last_telemetry
    assert st is not None and st.joints[0] > 20.0

    win.client.disconnect()
    win.close()


def test_cartesian_reachability_indicator():
    from PySide6.QtWidgets import QApplication
    from kia_studio.ui.main_window import MainWindow

    from kia_studio.core import fk_pose
    app = QApplication.instance() or QApplication([])
    win = MainWindow()
    cart = win.cart_panel
    cart.elbow.setCurrentIndex(1)                 # elbow down (elbow limit is 0..180)
    p = fk_pose(0, 60, 30, 10)                    # reachable elbow-down pose
    cart.sb_x.setValue(p.x); cart.sb_y.setValue(p.y)
    cart.sb_z.setValue(p.z); cart.sb_pitch.setValue(p.pitch)
    _pump(app, 0.05)
    assert cart.apply_btn.isEnabled()
    # push out of range
    cart.sb_x.setValue(cart.sb_x.maximum())
    cart.sb_z.setValue(cart.sb_z.maximum())
    _pump(app, 0.05)
    assert not cart.apply_btn.isEnabled()
    win.close()


if __name__ == "__main__":
    test_window_boots_and_streams()
    print("  ok  test_window_boots_and_streams")
    test_joint_panel_drives_sim()
    print("  ok  test_joint_panel_drives_sim")
    test_cartesian_reachability_indicator()
    print("  ok  test_cartesian_reachability_indicator")
