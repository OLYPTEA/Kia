"""Live 3D viewport — pyqtgraph GLViewWidget driven by the FK chain.

Renders a ground grid, the FK skeleton (joint pivots + links) as the kinematic ground
truth, and the real STL link meshes attached to their joint frames. Skeleton and meshes
toggle independently, and per-link calibration can be updated live so the meshes can be
aligned against the exact skeleton.
"""
from __future__ import annotations

import os

import numpy as np
import pyqtgraph.opengl as gl
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QMatrix4x4

from ...core.kinematics import (Geometry, fk_frames, chain_transforms, chain_joint_points,
                                DEFAULT_PIVOTS)
from ...core.mesh_loader import load_stl
from ..mesh_config import (LINK_META, LINK_ORDER, calib_matrix, hinge_matrix,
                           load_calibration)

_MESH_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "resources", "meshes")

_JOINT_COLOR = (0.55, 0.80, 1.0, 1.0)
_LINK_COLOR = (0.35, 0.55, 0.75, 1.0)
_TCP_COLOR = (1.0, 0.55, 0.35, 1.0)
_PIVOT_COLOR = (1.0, 0.85, 0.20, 1.0)   # calibration pivot centre
_AXIS_RGBA = ((1.0, 0.30, 0.30, 1.0), (0.35, 1.0, 0.35, 1.0), (0.40, 0.55, 1.0, 1.0))  # X,Y,Z
_GIZMO_LEN = 50.0       # mm, axis handle length
_GIZMO_PICK_PX = 22.0   # screen-space grab radius
_PIVOT_LINKS = ("base", "shoulder", "elbow", "wrist")


def _to_qmatrix(m: np.ndarray) -> QMatrix4x4:
    return QMatrix4x4(*[float(v) for v in m.flatten()])


class Viewport3D(gl.GLViewWidget):
    pivotMoved = Signal(str)   # emitted while dragging a pivot gizmo (link name)

    def __init__(self, geo: Geometry | None = None, parent=None):
        super().__init__(parent)
        self.geo = geo or Geometry()
        self.setBackgroundColor((18, 20, 26))
        self.setCameraPosition(distance=420, elevation=22, azimuth=-60)

        self.calib = load_calibration()
        self._pose = (0.0, 90.0, 0.0, 0.0)
        self._grip = 0.0
        self._ignore_telemetry = False     # frozen while calibrating
        self._pivot_link: str | None = None
        self._meshes: dict[str, gl.GLMeshItem] = {}
        self._centers: dict[str, np.ndarray] = {}

        # pivot gizmo state
        self._giz_center = np.zeros(3)
        self._giz_dirs = np.eye(3)         # world unit axes (rows)
        self._giz_handles = np.zeros((3, 3))
        self._drag_axis: int | None = None
        self._drag_pv0 = np.zeros(3)
        self._drag_s0 = 0.0

        # fixed home anchor frames: meshes are placed relative to these, then carried by
        # the pivot-driven chain. Keeps existing home calibration valid.
        hf = fk_frames(0.0, 90.0, 0.0, 0.0, self.geo)
        self._home_frames = {"bati": np.eye(4), "base": hf[0], "shoulder": hf[1],
                             "elbow": hf[2], "wrist": hf[3], "grip1": hf[3], "grip2": hf[3]}

        self._build_static()
        self._build_skeleton()
        self._load_meshes()
        self._build_gizmo()
        self.set_pose(*self._pose)

    def _pivots(self) -> dict:
        return {n: (self.calib[n].get("px", DEFAULT_PIVOTS[n][0]),
                    self.calib[n].get("py", DEFAULT_PIVOTS[n][1]),
                    self.calib[n].get("pz", DEFAULT_PIVOTS[n][2])) for n in DEFAULT_PIVOTS}

    def _placement(self, name: str) -> np.ndarray:
        return self._home_frames[name] @ calib_matrix(self._centers[name], self.calib[name])

    def _tcp_home(self) -> np.ndarray:
        """Tool-tip in world-home: centroid of the gripper jaws, else the wrist pivot."""
        cs = [(self._placement(n) @ np.append(self._centers[n], 1.0))[:3]
              for n in ("grip1", "grip2") if n in self._meshes]
        if cs:
            return np.mean(cs, axis=0)
        return np.asarray(self._pivots()["wrist"], float)

    def _build_gizmo(self):
        self._giz_axes = []
        for rgba in _AXIS_RGBA:
            it = gl.GLLinePlotItem(width=3.5, antialias=True, color=rgba)
            it.setVisible(False)
            self.addItem(it)
            self._giz_axes.append(it)
        self._giz_handle_items = []
        for rgba in _AXIS_RGBA:
            h = gl.GLScatterPlotItem(size=15.0, color=rgba, pxMode=True)
            h.setVisible(False)
            self.addItem(h)
            self._giz_handle_items.append(h)
        self._pivot_marker = gl.GLScatterPlotItem(size=13.0, color=_PIVOT_COLOR, pxMode=True)
        self._pivot_marker.setVisible(False)
        self.addItem(self._pivot_marker)

    # ---- static scene -------------------------------------------------
    def _build_static(self):
        grid = gl.GLGridItem()
        grid.setSize(600, 600)
        grid.setSpacing(20, 20)
        grid.setColor((80, 90, 110, 120))
        self.addItem(grid)
        axis = gl.GLAxisItem()
        axis.setSize(80, 80, 80)
        self.addItem(axis)

    def _build_skeleton(self):
        self._link_line = gl.GLLinePlotItem(width=3.0, antialias=True, color=_LINK_COLOR)
        self.addItem(self._link_line)
        self._joints = gl.GLScatterPlotItem(size=11.0, color=_JOINT_COLOR, pxMode=True)
        self.addItem(self._joints)
        self._tcp = gl.GLScatterPlotItem(size=14.0, color=_TCP_COLOR, pxMode=True)
        self.addItem(self._tcp)

    def _load_meshes(self):
        for name in LINK_ORDER:
            path = os.path.join(_MESH_DIR, LINK_META[name]["file"])
            if not os.path.exists(path):
                continue
            mesh = load_stl(path)
            md = gl.MeshData(vertexes=mesh.vertices, faces=mesh.faces)
            item = gl.GLMeshItem(meshdata=md, smooth=True, shader="shaded",
                                 color=LINK_META[name]["color"], glOptions="opaque")
            self.addItem(item)
            self._meshes[name] = item
            self._centers[name] = mesh.center

    # ---- live update --------------------------------------------------
    def set_pose(self, base: float, shoulder: float, elbow: float, wrist: float,
                 grip: float | None = None):
        self._pose = (base, shoulder, elbow, wrist)
        if grip is not None:
            self._grip = grip
        pivots = self._pivots()
        M = chain_transforms(base, shoulder, elbow, wrist, pivots)

        pts = np.array(chain_joint_points(base, shoulder, elbow, wrist, pivots,
                                          self._tcp_home()), dtype=float)
        self._link_line.setData(pos=pts)
        self._joints.setData(pos=pts[:4])
        self._tcp.setData(pos=pts[4:5])

        for name in LINK_ORDER:
            item = self._meshes.get(name)
            if item is not None:
                cfg = self.calib[name]
                Mm = (M[name] @ self._home_frames[name] @ hinge_matrix(cfg, self._grip)
                      @ calib_matrix(self._centers[name], cfg))
                item.setTransform(_to_qmatrix(Mm))
        self._chain_M = M
        self._refresh_pivot_marker()
        self.update()   # QOpenGLWidget: force an immediate repaint of the scene

    def update_from_status(self, st):
        if self._ignore_telemetry:
            return
        j = st.joints
        self.set_pose(j[0], j[1], j[2], j[3], st.grip)

    def set_link_calib(self, name: str, cfg: dict):
        """Replace one link's calibration and re-apply at the current pose."""
        self.calib[name].update(cfg)
        self.set_pose(*self._pose)

    # ---- calibration support -----------------------------------------
    def set_telemetry_frozen(self, frozen: bool):
        """Stop telemetry from driving the view (e.g. during timeline preview)."""
        self._ignore_telemetry = frozen

    def set_calib_frozen(self, frozen: bool):
        """Freeze the view on the home reference (ignore telemetry) while calibrating."""
        self._ignore_telemetry = frozen
        if frozen:
            self.set_pose(0.0, 90.0, 0.0, 0.0, 0.0)

    def show_pivot_marker(self, name: str | None):
        """Show the draggable pivot gizmo for `name` (None hides it)."""
        self._pivot_link = name if name in _PIVOT_LINKS else None
        self._refresh_pivot_marker()
        self.update()

    _PIVOT_PARENT = {"base": "bati", "shoulder": "base", "elbow": "shoulder", "wrist": "elbow"}

    def _refresh_pivot_marker(self):
        name = self._pivot_link
        vis = name is not None
        for it in (*self._giz_axes, *self._giz_handle_items):
            it.setVisible(vis)
        self._pivot_marker.setVisible(vis)
        if not vis:
            return
        Mp = getattr(self, "_chain_M", {}).get(self._PIVOT_PARENT[name], np.eye(4))
        pv = np.asarray(self._pivots()[name], float)
        center = (Mp @ np.append(pv, 1.0))[:3]
        dirs = Mp[:3, :3].T                      # world axes carried by the parent (unit)
        dirs = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)
        handles = center + _GIZMO_LEN * dirs     # (3,3)
        self._giz_center = center
        self._giz_dirs = dirs
        self._giz_handles = handles
        self._pivot_marker.setData(pos=center.reshape(1, 3))
        for i in range(3):
            self._giz_axes[i].setData(pos=np.vstack([center, handles[i]]))
            self._giz_handle_items[i].setData(pos=handles[i].reshape(1, 3))

    # ---- 3D picking / drag for the pivot gizmo -----------------------
    def _vp_matrix(self) -> np.ndarray:
        vp = self.getViewport()
        proj = np.array(self.projectionMatrix(vp, vp).data(), float).reshape(4, 4).T
        view = np.array(self.viewMatrix().data(), float).reshape(4, 4).T
        return proj @ view

    def _project(self, M: np.ndarray, p: np.ndarray):
        """World point -> (screen_x, screen_y) in logical px, or None if behind camera."""
        clip = M @ np.array([p[0], p[1], p[2], 1.0])
        if clip[3] <= 1e-6:
            return None
        ndc = clip[:3] / clip[3]
        return ((ndc[0] + 1) * 0.5 * self.width(), (1 - ndc[1]) * 0.5 * self.height())

    def _ray_from_pixel(self, x: float, y: float):
        inv = np.linalg.inv(self._vp_matrix())
        nx, ny = 2 * x / self.width() - 1, 1 - 2 * y / self.height()
        near = inv @ np.array([nx, ny, -1.0, 1.0]); near = near[:3] / near[3]
        far = inv @ np.array([nx, ny, 1.0, 1.0]); far = far[:3] / far[3]
        d = far - near
        return near, d / (np.linalg.norm(d) or 1.0)

    def _axis_param_at_ray(self, axis_i: int, x: float, y: float) -> float:
        """Signed distance along axis `axis_i` (from gizmo centre) closest to the cursor ray."""
        o, r = self._ray_from_pixel(x, y)
        u = self._giz_dirs[axis_i]
        w0 = self._giz_center - o
        b = float(u @ r)
        denom = 1.0 - b * b
        if abs(denom) < 1e-9:
            return self._drag_s0
        return float((b * (r @ w0) - (u @ w0)) / denom)

    def _pick_axis(self, x: float, y: float):
        M = self._vp_matrix()
        best, best_d = None, _GIZMO_PICK_PX
        for i in range(3):
            s = self._project(M, self._giz_handles[i])
            if s is None:
                continue
            d = ((s[0] - x) ** 2 + (s[1] - y) ** 2) ** 0.5
            if d < best_d:
                best, best_d = i, d
        return best

    def mousePressEvent(self, ev):
        if self._pivot_link is not None and ev.button() == Qt.LeftButton:
            p = ev.position()
            axis = self._pick_axis(p.x(), p.y())
            if axis is not None:
                cfg = self.calib[self._pivot_link]
                self._drag_axis = axis
                self._drag_pv0 = np.array([cfg["px"], cfg["py"], cfg["pz"]], float)
                self._drag_s0 = self._axis_param_at_ray(axis, p.x(), p.y())
                ev.accept()
                return
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if self._drag_axis is not None:
            p = ev.position()
            s = self._axis_param_at_ray(self._drag_axis, p.x(), p.y())
            pv = self._drag_pv0.copy()
            pv[self._drag_axis] = self._drag_pv0[self._drag_axis] + (s - self._drag_s0)
            cfg = self.calib[self._pivot_link]
            cfg["px"], cfg["py"], cfg["pz"] = float(pv[0]), float(pv[1]), float(pv[2])
            self.set_pose(*self._pose)
            self.pivotMoved.emit(self._pivot_link)
            ev.accept()
            return
        super().mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev):
        if self._drag_axis is not None:
            self._drag_axis = None
            ev.accept()
            return
        super().mouseReleaseEvent(ev)

    # ---- toggles ------------------------------------------------------
    def set_skeleton_visible(self, on: bool):
        for it in (self._link_line, self._joints, self._tcp):
            it.setVisible(on)
        self.update()

    def set_meshes_visible(self, on: bool):
        for it in self._meshes.values():
            it.setVisible(on)
        self.update()

    def highlight_link(self, name: str | None):
        """Dim all meshes except `name` (visual aid while calibrating)."""
        for n, it in self._meshes.items():
            col = list(LINK_META[n]["color"])
            if name is not None and n != name:
                col[3] = 0.25
            it.setColor(tuple(col))
        self.update()
