"""Forward kinematics (host side) — byte-faithful to firmware dom_kinematics/kinematics.c.

Convention (planar 3R rotated by base yaw q0 about +Z):
    r = L1 sin(q1) + L2 sin(q1+q2) + Ltool sin(q1+q2+q3)
    z = h_base + L1 cos(q1) + L2 cos(q1+q2) + Ltool cos(q1+q2+q3)
    x = r cos(q0),  y = r sin(q0)
    pitch = (q1+q2+q3) - 90deg   (from horizontal)

`fk_chain` additionally returns each link joint position for 3D rendering.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np


@dataclass(slots=True)
class Geometry:
    """Link dimensions in mm — defaults == KIA_LINK_* in app_config.h."""
    h_base: float = 60.0
    L1: float = 105.0      # upper arm
    L2: float = 100.0      # forearm
    L_tool: float = 85.0


@dataclass(slots=True)
class Pose:
    x: float
    y: float
    z: float
    pitch: float           # deg, from horizontal


def fk_pose(base: float, shoulder: float, elbow: float, wrist: float,
            geo: Geometry | None = None) -> Pose:
    """TCP pose from joint angles (deg)."""
    g = geo or Geometry()
    q0 = math.radians(base)
    q1 = math.radians(shoulder)
    q2 = math.radians(elbow)
    q3 = math.radians(wrist)
    r = g.L1 * math.sin(q1) + g.L2 * math.sin(q1 + q2) + g.L_tool * math.sin(q1 + q2 + q3)
    z = g.h_base + g.L1 * math.cos(q1) + g.L2 * math.cos(q1 + q2) + g.L_tool * math.cos(q1 + q2 + q3)
    return Pose(
        x=r * math.cos(q0),
        y=r * math.sin(q0),
        z=z,
        pitch=math.degrees(q1 + q2 + q3) - 90.0,
    )


def fk_chain(base: float, shoulder: float, elbow: float, wrist: float,
             geo: Geometry | None = None) -> list[tuple[float, float, float]]:
    """Ordered 3D points: [floor, shoulder, elbow, wrist, TCP] for line/mesh rendering."""
    g = geo or Geometry()
    q0 = math.radians(base)
    q1 = math.radians(shoulder)
    q2 = math.radians(elbow)
    q3 = math.radians(wrist)
    c0, s0 = math.cos(q0), math.sin(q0)

    def world(r: float, z: float) -> tuple[float, float, float]:
        return (r * c0, r * s0, z)

    p_floor = (0.0, 0.0, 0.0)
    r = 0.0
    z = g.h_base
    p_shoulder = world(r, z)
    r += g.L1 * math.sin(q1);            z += g.L1 * math.cos(q1)
    p_elbow = world(r, z)
    r += g.L2 * math.sin(q1 + q2);       z += g.L2 * math.cos(q1 + q2)
    p_wrist = world(r, z)
    r += g.L_tool * math.sin(q1 + q2 + q3); z += g.L_tool * math.cos(q1 + q2 + q3)
    p_tcp = world(r, z)
    return [p_floor, p_shoulder, p_elbow, p_wrist, p_tcp]


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], float)


def _trans(p) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = p
    return m


def fk_frames(base: float, shoulder: float, elbow: float, wrist: float,
              geo: Geometry | None = None) -> list[np.ndarray]:
    """4x4 world transforms of each link frame: [base, shoulder, elbow, wrist, grip].

    Each frame's origin sits at the link's proximal joint; its local +Z points along
    the link (the direction the link extends), and the joint rotation axis is local +Y.
    A mesh attached via ``frame @ M_calib`` therefore rotates rigidly with its joint.
    The grip shares the wrist frame (it is rigid on the wrist).
    """
    pts = fk_chain(base, shoulder, elbow, wrist, geo)
    q0 = math.radians(base)
    q1 = math.radians(shoulder)
    q2 = math.radians(elbow)
    q3 = math.radians(wrist)
    rz = _rot_z(q0)
    f_base = rz.copy()                                            # origin at floor
    f_shoulder = _trans(pts[1]) @ rz @ _rot_y(q1)
    f_elbow = _trans(pts[2]) @ rz @ _rot_y(q1 + q2)
    f_wrist = _trans(pts[3]) @ rz @ _rot_y(q1 + q2 + q3)
    return [f_base, f_shoulder, f_elbow, f_wrist, f_wrist.copy()]


# Joint home references (deg) == app_config KIA_LIM_* home: base 0, shoulder 90, elbow 0, wrist 0.
_HOME_DEG = (0.0, 90.0, 0.0, 0.0)

# Joint rotation axes at the home pose (world): base = vertical Z, the others = horizontal Y.
_JOINT_AXIS = {"base": "z", "shoulder": "y", "elbow": "y", "wrist": "y"}
_CHAIN_JOINTS = ("base", "shoulder", "elbow", "wrist")

# Default pivot points (world, mm) at home == the joint positions of the nominal geometry,
# i.e. fk_chain(0, 90, 0, 0): a sensible starting point the user then drags onto the real hinges.
DEFAULT_PIVOTS = {
    "base": (0.0, 0.0, 0.0),
    "shoulder": (0.0, 0.0, 60.0),
    "elbow": (105.0, 0.0, 60.0),
    "wrist": (205.0, 0.0, 60.0),
}


def _rot_axis(axis: str, a: float) -> np.ndarray:
    return _rot_z(a) if axis == "z" else _rot_y(a)


def chain_transforms(base: float, shoulder: float, elbow: float, wrist: float,
                     pivots: dict) -> dict[str, np.ndarray]:
    """Pivot-driven kinematic chain: cumulative 4x4 transforms per link.

    Each joint rotates its link AND all downstream links about its pivot point (world,
    home coords) and home axis. Composition (outer = parent) means moving one joint
    recomputes everything below it — Base ▸ Épaule ▸ Coude ▸ Poignet ▸ Grip:

        M_base     = R(pivot_base,  Z, q0)
        M_shoulder = M_base     @ R(pivot_shoulder, Y, q1 - 90)
        M_elbow    = M_shoulder @ R(pivot_elbow,    Y, q2)
        M_wrist    = M_elbow    @ R(pivot_wrist,    Y, q3)

    At home every rotation is identity (M = I), so meshes stay at their calibrated home
    placement; link lengths are implicit in the pivot spacing (no fixed geometry).
    """
    q = {"base": base, "shoulder": shoulder, "elbow": elbow, "wrist": wrist}
    qh = dict(zip(_CHAIN_JOINTS, _HOME_DEG))
    M = np.eye(4)
    out = {"bati": np.eye(4)}
    for name in _CHAIN_JOINTS:
        p = np.asarray(pivots.get(name, DEFAULT_PIVOTS[name]), float)
        ang = math.radians(q[name] - qh[name])
        R = _trans(p) @ _rot_axis(_JOINT_AXIS[name], ang) @ _trans(-p)
        M = M @ R
        out[name] = M
    out["grip1"] = out["wrist"]
    out["grip2"] = out["wrist"].copy()
    return out


def chain_joint_points(base: float, shoulder: float, elbow: float, wrist: float,
                       pivots: dict, tcp_home=None) -> list[np.ndarray]:
    """World skeleton points [base, shoulder, elbow, wrist, tcp] from the pivot chain.

    Each joint sits at its pivot carried by its PARENT transform; `tcp_home` (world, home)
    is carried by the wrist transform (defaults to the wrist pivot if not given).
    """
    M = chain_transforms(base, shoulder, elbow, wrist, pivots)
    parents = {"base": "bati", "shoulder": "base", "elbow": "shoulder", "wrist": "elbow"}
    pts = []
    for name in _CHAIN_JOINTS:
        p = np.asarray(pivots.get(name, DEFAULT_PIVOTS[name]), float)
        pts.append((M[parents[name]] @ np.append(p, 1.0))[:3])
    tcp = np.asarray(tcp_home if tcp_home is not None else pivots.get("wrist", DEFAULT_PIVOTS["wrist"]), float)
    pts.append((M["wrist"] @ np.append(tcp, 1.0))[:3])
    return pts


# --- geometry calibration: derive link lengths from the calibrated joint pivots ---
_GEOM_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "resources", "geometry.json")


def _dist(a, b) -> float:
    return float(np.linalg.norm(np.asarray(a, float) - np.asarray(b, float)))


def geometry_from_pivots(pivots: dict, tcp) -> Geometry:
    """Real link lengths from the world-home pivots + tool tip (the 3R-planar projection)."""
    return Geometry(
        h_base=_dist(pivots["base"], pivots["shoulder"]),
        L1=_dist(pivots["shoulder"], pivots["elbow"]),
        L2=_dist(pivots["elbow"], pivots["wrist"]),
        L_tool=_dist(pivots["wrist"], tcp),
    )


def save_geometry(geo: Geometry, path: str = _GEOM_PATH) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"h_base": geo.h_base, "L1": geo.L1, "L2": geo.L2, "L_tool": geo.L_tool},
                  f, indent=2)
    return path


def load_geometry(path: str = _GEOM_PATH) -> Geometry:
    """Calibrated geometry from JSON if present, else the nominal defaults."""
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                d = json.load(f)
            return Geometry(float(d["h_base"]), float(d["L1"]), float(d["L2"]), float(d["L_tool"]))
        except (OSError, ValueError, KeyError):
            pass
    return Geometry()


def firmware_geometry_header(geo: Geometry) -> str:
    """C snippet ready to paste into firmware main/app_config.h (KIA_LINK_* macros)."""
    return (
        "// --- Kia Studio calibrated link geometry (mm) — paste into app_config.h ---\n"
        f"#define KIA_LINK_BASE_HEIGHT_MM  {geo.h_base:.1f}f\n"
        f"#define KIA_LINK_UPPER_ARM_MM    {geo.L1:.1f}f\n"
        f"#define KIA_LINK_FOREARM_MM      {geo.L2:.1f}f\n"
        f"#define KIA_LINK_TOOL_MM         {geo.L_tool:.1f}f\n"
    )


class IkStatus(Enum):
    OK = "ok"
    UNREACHABLE = "unreachable"   # outside the 2R annulus
    LIMIT = "limit"               # solvable but a joint exceeds its software limit


@dataclass(slots=True)
class IkResult:
    status: IkStatus
    base: float = 0.0
    shoulder: float = 0.0
    elbow: float = 0.0
    wrist: float = 0.0

    @property
    def ok(self) -> bool:
        return self.status is IkStatus.OK

    def joints(self) -> tuple[float, float, float, float]:
        return (self.base, self.shoulder, self.elbow, self.wrist)


def ik(x: float, y: float, z: float, pitch_deg: float,
       elbow_up: bool = True, geo: Geometry | None = None,
       check_limits: bool = True) -> IkResult:
    """Closed-form 4DOF inverse kinematics — the exact inverse of `fk_pose`.

    Angle convention matches the host FK: joint angles are measured from vertical,
    and pitch is from horizontal with sum(q1..q3) = pitch + 90deg. (This corrects a
    pitch-sign mismatch present in firmware kin_ik vs kin_fk — see docs.)
    """
    g = geo or Geometry()
    q0 = math.atan2(y, x)
    r_tcp = math.hypot(x, y)
    pitch = math.radians(pitch_deg)
    sum_q = pitch + math.pi / 2          # q1+q2+q3, from vertical

    # subtract the tool link to get the wrist (2R end) point in the (r,z) plane
    r_w = r_tcp - g.L_tool * math.sin(sum_q)
    z_w = z - g.h_base - g.L_tool * math.cos(sum_q)

    d2 = r_w * r_w + z_w * z_w
    d = math.sqrt(d2)
    if d > (g.L1 + g.L2) + 1e-6 or d < abs(g.L1 - g.L2) - 1e-6:
        return IkResult(IkStatus.UNREACHABLE)

    c2 = (d2 - g.L1 * g.L1 - g.L2 * g.L2) / (2 * g.L1 * g.L2)
    c2 = max(-1.0, min(1.0, c2))
    s2 = (-1.0 if elbow_up else 1.0) * math.sqrt(max(0.0, 1 - c2 * c2))
    q2 = math.atan2(s2, c2)

    k1 = g.L1 + g.L2 * c2
    k2 = g.L2 * s2
    q1 = math.atan2(r_w, z_w) - math.atan2(k2, k1)
    q3 = sum_q - q1 - q2

    res = IkResult(IkStatus.OK,
                   base=math.degrees(q0), shoulder=math.degrees(q1),
                   elbow=math.degrees(q2), wrist=math.degrees(q3))

    if check_limits:
        from .joints import JOINTS
        vals = res.joints()
        for spec, v in zip(JOINTS, vals):
            if not (spec.min_deg - 1e-6 <= v <= spec.max_deg + 1e-6):
                return IkResult(IkStatus.LIMIT, *vals)
    return res


def solve_ik(x: float, y: float, z: float, pitch_deg: float,
             geo: Geometry | None = None) -> IkResult:
    """Try elbow-up then elbow-down; return the first valid solution (or last failure)."""
    up = ik(x, y, z, pitch_deg, elbow_up=True, geo=geo)
    if up.ok:
        return up
    down = ik(x, y, z, pitch_deg, elbow_up=False, geo=geo)
    return down if down.ok else up
