"""Trajectory tests — smoothness, joint limits, and keyframe dwell (stop-and-hold)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kia_studio.core.animation import Sequence  # noqa: E402
from kia_studio.core.trajectory import (build_trajectory, _min_clearance,  # noqa: E402
                                        _floor_project, _FLOOR_Z)


def _seq():
    # floor-safe, in-limit poses (shoulder -90..90, elbow 0..180), all links above z=0
    s = Sequence("t")
    s.add(0.0, [0, 80, 20, 0, 0])
    s.add(1.0, [30, 70, 30, -10, 60])
    s.add(2.0, [-25, 60, 40, 10, 0])
    return s


def _dense(tr, n=1500):
    ts = np.linspace(0, tr.duration, n)
    return ts, np.array([tr.sample(t) for t in ts])


def test_passes_through_keyframes():
    s = _seq()
    tr = build_trajectory(s)
    _, pos = _dense(tr, 3000)
    for kf in s.keyframes:
        d = np.abs(pos - np.array(kf.pose)).max(axis=1).min()
        assert d < 1.0, (kf.t, d)


def test_zero_velocity_at_endpoints():
    tr = build_trajectory(_seq())
    dt = 1e-3
    v0 = (np.array(tr.sample(dt)) - np.array(tr.sample(0.0))) / dt
    v1 = (np.array(tr.sample(tr.duration)) - np.array(tr.sample(tr.duration - dt))) / dt
    assert np.all(np.abs(v0) < 1.0) and np.all(np.abs(v1) < 1.0)


def test_velocity_is_continuous():
    tr = build_trajectory(_seq())
    ts, pos = _dense(tr, 800)
    vel = np.diff(pos, axis=0) / (ts[1] - ts[0])
    assert np.max(np.abs(np.diff(vel, axis=0))) < 5.0


def test_velocity_limit_respected():
    s = Sequence("fast")
    s.add(0.0, [0, 90, 0, 0, 0])
    s.add(0.1, [90, 0, 120, 80, 90])     # huge move in 0.1 s -> must be slowed down
    vmax = (150, 150, 150, 150, 220)
    tr = build_trajectory(s, vmax=vmax)
    assert tr.duration > 0.1
    ts, pos = _dense(tr, 600)
    vel = np.abs(np.diff(pos, axis=0) / (ts[1] - ts[0]))
    for j in range(5):
        assert vel[:, j].max() <= vmax[j] * 1.06, (j, vel[:, j].max())


def test_single_keyframe_is_constant():
    s = Sequence(); s.add(0.0, [1, 2, 3, 4, 5])
    tr = build_trajectory(s)
    assert tr.sample(0.0) == [1, 2, 3, 4, 5] and tr.sample(5.0) == [1, 2, 3, 4, 5]


def test_dwell_extends_duration_and_holds():
    base = build_trajectory(_seq())
    s = _seq()
    s.keyframes[1].dwell = 1.5            # hold 1.5 s on the middle keyframe
    held = build_trajectory(s)
    assert abs(held.duration - (base.duration + 1.5)) < 0.05
    # find the hold window: the middle pose must stay constant for ~1.5 s
    mid_pose = np.array(s.keyframes[1].pose)
    ts, pos = _dense(held, 4000)
    on_pose = np.where(np.abs(pos - mid_pose).max(axis=1) < 0.05)[0]
    assert len(on_pose) > 0
    held_span = ts[on_pose[-1]] - ts[on_pose[0]]
    assert held_span >= 1.4               # roughly the dwell duration


def test_dwell_keyframe_is_a_full_stop():
    s = _seq()
    s.keyframes[1].dwell = 1.0
    tr = build_trajectory(s)
    # velocity right around the middle hold must be ~zero (full stop)
    mid_pose = np.array(s.keyframes[1].pose)
    ts, pos = _dense(tr, 4000)
    i = int(np.abs(pos - mid_pose).max(axis=1).argmin())
    dt = ts[1] - ts[0]
    v = (pos[min(i + 1, len(pos) - 1)] - pos[i]) / dt
    assert np.all(np.abs(v) < 2.0)


def test_floor_clearance_excludes_base_mount():
    # the fixed base point sits at z=0; the moving links of the home pose are well above
    assert _min_clearance([0, 90, 0, 0, 0]) > 50.0


def test_floor_project_lifts_a_below_floor_pose():
    bad = [0.0, 90.0, 180.0, -90.0, 0.0]      # TCP dips ~25 mm under the floor
    assert _min_clearance(bad) < _FLOOR_Z
    fixed = _floor_project(bad, [0.0, 90.0, 0.0, 0.0, 0.0])
    assert _min_clearance(fixed) >= _FLOOR_Z


def test_smooth_trajectory_never_goes_under_floor():
    # elbow 0 -> 15 -> 0: the spline would overshoot the elbow and dip the TCP under z=0
    s = Sequence()
    s.add(0.0, [0, 90, 0, 0, 0])
    s.add(1.0, [0, 90, 15, 0, 0])
    s.add(2.0, [0, 90, 0, 0, 0])
    tr = build_trajectory(s)
    ts = np.linspace(0, tr.duration, 1200)
    worst = min(_min_clearance(tr.sample(t)) for t in ts)
    assert worst >= _FLOOR_Z - 1e-6, worst


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn(); print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
