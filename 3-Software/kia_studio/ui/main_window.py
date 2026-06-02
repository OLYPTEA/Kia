"""MainWindow — assembles connection bar, live telemetry panel and an event log.

M2 scope: connect (USB or simulator), stream telemetry, surface ACK/NACK/PONG/text.
Teleop (M3) and the 3D viewport (M4) dock into the same window later.
"""
from __future__ import annotations

import os
import sys
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QIcon
from PySide6.QtWidgets import (
    QApplication, QHBoxLayout, QLabel, QMainWindow, QPlainTextEdit,
    QSplitter, QTabWidget, QVBoxLayout, QWidget,
)

from ..proto.codec import Status
from ..proto.opcodes import NACK_REASON
from .qt_client import QtKiaClient
from .theme import STYLESHEET
from .widgets.cartesian_panel import CartesianPanel
from .widgets.connection_bar import ConnectionBar
from .widgets.control_bar import ControlBar
from .widgets.joint_panel import JointPanel
from .widgets.telemetry_panel import TelemetryPanel
from .widgets.timeline_panel import TimelinePanel


def _make_viewport():
    """Create the 3D viewport: moderngl (studio) first, pyqtgraph fallback, else None.

    GL is unsafe under the 'offscreen' Qt platform (used by the headless tests)."""
    if os.environ.get("QT_QPA_PLATFORM", "") == "offscreen":
        return None
    try:
        from .widgets.viewport_gl import GLViewport
        return GLViewport()
    except Exception as exc:  # pragma: no cover - GL driver issues
        print(f"[kia] moderngl viewport unavailable, trying pyqtgraph: {exc}", file=sys.stderr)
    try:
        from .widgets.viewport3d import Viewport3D
        return Viewport3D()
    except Exception as exc:  # pragma: no cover
        print(f"[kia] 3D viewport disabled: {exc}", file=sys.stderr)
    return None


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Kia Studio")
        self.resize(1320, 760)

        self.client = QtKiaClient(self)
        self._tlm_count = 0
        self._tlm_t0 = time.monotonic()

        self.conn_bar = ConnectionBar(self.client)
        self.control_bar = ControlBar(self.client)
        self.telemetry = TelemetryPanel()

        # teleop tabs
        self.joint_panel = JointPanel()
        self.cart_panel = CartesianPanel()
        self.teleop = QTabWidget()
        self.teleop.addTab(self.joint_panel, "Joints")
        self.teleop.addTab(self.cart_panel, "Cartesian")

        # 3D viewport (headline) — moderngl studio renderer, guarded for offscreen/CI
        self.viewport = _make_viewport()
        self.calib_panel = None
        if self.viewport is not None:
            from .widgets.calib_panel import CalibrationPanel
            self.calib_panel = CalibrationPanel(self.viewport)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(2000)
        self.log.setFont(QFont("Consolas", 9))
        self.log.setObjectName("eventLog")

        log_wrap = QWidget()
        lv = QVBoxLayout(log_wrap)
        lv.setContentsMargins(8, 8, 8, 8)
        lv.addWidget(QLabel("Event log"))
        lv.addWidget(self.log)

        # right-hand control stack
        self.tabs = QTabWidget()
        self.tabs.addTab(self.teleop, "Téléop")
        if self.calib_panel is not None:
            self.tabs.addTab(self.calib_panel, "Calibration 3D")
        self.tabs.addTab(log_wrap, "Log")

        split = QSplitter(Qt.Horizontal)
        split.addWidget(self.telemetry)
        center = self.viewport if self.viewport is not None else self._viewport_placeholder()
        split.addWidget(center)
        split.addWidget(self.tabs)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setStretchFactor(2, 0)
        split.setSizes([280, 640, 400])

        # keyframe timeline (bottom bar) — preview drives the 3D view, run streams to robot
        self.timeline = TimelinePanel(pose_provider=self.joint_panel.current_pose)
        self.timeline.setObjectName("timeline")

        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self.conn_bar)
        root.addWidget(self.control_bar)
        root.addWidget(split, 1)
        root.addWidget(self.timeline)
        self.setCentralWidget(central)

        self._rate_lbl = QLabel("0.0 Hz")
        self.statusBar().addPermanentWidget(self._rate_lbl)
        self.statusBar().showMessage("Ready — connect to a device or the simulator")

        # wire client -> UI
        self.client.telemetry.connect(self._on_telemetry)
        self.client.status.connect(self.telemetry.update_status)
        self.client.ack.connect(lambda seq: self._append(f"ACK   seq={seq}"))
        self.client.nack.connect(
            lambda seq, r: self._append(f"NACK  seq={seq} reason={NACK_REASON.get(r, hex(r))}")
        )
        self.client.pong.connect(lambda v: self._append(f"PONG  fw={v[0]}.{v[1]}.{v[2]}"))
        self.client.text.connect(lambda s: self._append(f"<     {s}"))
        self.client.connected.connect(lambda lbl: self._append(f"--- connected: {lbl}"))
        self.client.disconnected.connect(lambda lbl: self._on_disconnected(lbl))
        self.client.error.connect(lambda m: self._append(f"!!!   {m}"))

        # teleop -> client
        self.joint_panel.command.connect(self._send_joints)
        self.cart_panel.command.connect(self._send_xyz)

        # timeline -> preview + robot
        self.timeline.poseChanged.connect(self._on_anim_pose)
        self.timeline.playbackActive.connect(self._on_anim_active)
        self._last_robot_send = 0.0

        # telemetry-rate ticker
        self._rate_timer = QTimer(self)
        self._rate_timer.setInterval(1000)
        self._rate_timer.timeout.connect(self._update_rate)
        self._rate_timer.start()

    def _viewport_placeholder(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lbl = QLabel("Vue 3D indisponible\n(OpenGL non initialisé)")
        lbl.setAlignment(Qt.AlignCenter)
        lay.addWidget(lbl)
        return w

    # ---- slots ----
    def _on_telemetry(self, st: Status) -> None:
        self.telemetry.update_status(st)
        if not self.timeline.is_playing():        # don't fight the animation while playing
            self.joint_panel.update_from_status(st)
        if self.viewport is not None:
            self.viewport.update_from_status(st)   # no-op while frozen during playback
        self._tlm_count += 1

    def _send_joints(self, vals: list, dur: float) -> None:
        self.client.set_joints(vals[0], vals[1], vals[2], vals[3], vals[4], dur)

    def _send_xyz(self, pose: list, dur: float) -> None:
        self.client.set_xyz(pose[0], pose[1], pose[2], pose[3], dur)

    # ---- timeline animation ----
    def _on_anim_pose(self, pose: list) -> None:
        """A sampled trajectory pose: animate the 3D view + joints, and (live) drive the robot."""
        self.joint_panel.apply_pose(pose)
        if self.viewport is not None:
            self.viewport.set_pose(pose[0], pose[1], pose[2], pose[3], pose[4])
        # stream to the connected device, live, throttled to ~20 Hz
        if (self.timeline.is_playing() and self.timeline.robot_enabled()
                and self.client.is_connected):
            now = time.monotonic()
            if now - self._last_robot_send >= 0.045:
                self.client.set_joints(pose[0], pose[1], pose[2], pose[3], pose[4], 0.09)
                self._last_robot_send = now

    def _on_anim_active(self, active: bool) -> None:
        # while playing, the 3D view shows the exact trajectory (ignore telemetry)
        if self.viewport is not None:
            self.viewport.set_telemetry_frozen(active)

    def _on_disconnected(self, label: str) -> None:
        self._append(f"--- disconnected: {label}")
        self._rate_lbl.setText("0.0 Hz")

    def _update_rate(self) -> None:
        now = time.monotonic()
        dt = now - self._tlm_t0
        if dt > 0:
            self._rate_lbl.setText(f"{self._tlm_count / dt:.1f} Hz")
        self._tlm_count = 0
        self._tlm_t0 = now

    def _append(self, line: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log.appendPlainText(f"{ts}  {line}")

    def closeEvent(self, event) -> None:
        self.client.disconnect()
        super().closeEvent(event)


def run() -> int:
    # request a 3.3 core GL context for the moderngl viewport (before any QApplication)
    try:
        from .widgets.viewport_gl import ensure_gl_format
        ensure_gl_format()
    except Exception:
        pass
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("Kia Studio")
    _icon = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "kia.ico")
    if os.path.exists(_icon):
        app.setWindowIcon(QIcon(_icon))
    app.setStyleSheet(STYLESHEET)
    win = MainWindow()
    win.show()
    return app.exec()
