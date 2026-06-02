"""Smooth multi-joint trajectory generation from keyframes (pure, no Qt/hardware).

A keyframe may carry a ``dwell`` (delay): the arm reaches that pose, STOPS on it for the
dwell duration, then resumes. Dwell keyframes (and the endpoints) are therefore stop
points; the motion between two stop points is one clamped **cubic spline** (C², passing
through any intermediate pass-through keyframes, zero velocity at the segment ends), or
linear when ``smooth=False``. Each moving segment is time-scaled so no joint exceeds its
velocity/acceleration limit. The result is a piecewise trajectory of moves and holds.
"""
from __future__ import annotations

import numpy as np

from .animation import Sequence
from .kinematics import fk_chain

POSE_LEN = 5
DEFAULT_VMAX = (150.0, 150.0, 150.0, 150.0, 220.0)   # deg/s, per joint
DEFAULT_AMAX = (500.0, 500.0, 500.0, 500.0, 900.0)   # deg/s^2, per joint
_SAMPLES = 200

# Hard floor constraint: no moving link may pass under the ground (z = 0).
_FLOOR_Z = 0.0
_FLOOR_MARGIN = 2.0          # mm clearance kept above the floor


def _min_clearance(pose) -> float:
    """Lowest z of the moving links (excludes the fixed base mount at z=0)."""
    pts = fk_chain(pose[0], pose[1], pose[2], pose[3])
    return min(p[2] for p in pts[1:])


def _floor_project(pose, anchor):
    """Pull a below-floor pose toward a floor-safe anchor until it clears the ground."""
    if _min_clearance(pose) >= _FLOOR_Z + _FLOOR_MARGIN:
        return pose
    for k in range(1, 11):
        a = k / 10.0
        b = [pose[j] * (1 - a) + anchor[j] * a for j in range(POSE_LEN)]
        if _min_clearance(b) >= _FLOOR_Z + _FLOOR_MARGIN:
            return b
    return list(anchor)


class _CubicSpline:
    """Clamped (zero end-velocity) cubic spline of one scalar joint over knot times."""

    def __init__(self, t, y):
        self.t = t; self.y = y
        n = len(t) - 1
        h = np.diff(t)
        A = np.zeros((n + 1, n + 1)); d = np.zeros(n + 1)
        A[0, 0] = 2 * h[0]; A[0, 1] = h[0]; d[0] = 6 * ((y[1] - y[0]) / h[0])
        A[n, n - 1] = h[n - 1]; A[n, n] = 2 * h[n - 1]; d[n] = 6 * (-(y[n] - y[n - 1]) / h[n - 1])
        for i in range(1, n):
            A[i, i - 1] = h[i - 1]; A[i, i] = 2 * (h[i - 1] + h[i]); A[i, i + 1] = h[i]
            d[i] = 6 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])
        self.M = np.linalg.solve(A, d); self.h = h

    def _seg(self, tq):
        return min(max(int(np.searchsorted(self.t, tq) - 1), 0), len(self.t) - 2)

    def pos(self, tq):
        i = self._seg(tq); h = self.h[i]
        a = (self.t[i + 1] - tq) / h; b = (tq - self.t[i]) / h
        return (a * self.y[i] + b * self.y[i + 1]
                + ((a**3 - a) * self.M[i] + (b**3 - b) * self.M[i + 1]) * h * h / 6.0)

    def grid_deriv(self, order):
        ts = np.linspace(self.t[0], self.t[-1], _SAMPLES); out = np.empty_like(ts)
        for k, tq in enumerate(ts):
            i = self._seg(tq); h = self.h[i]
            a = (self.t[i + 1] - tq) / h; b = (tq - self.t[i]) / h
            if order == 1:
                out[k] = ((self.y[i + 1] - self.y[i]) / h
                          - (3 * a * a - 1) / 6 * h * self.M[i] + (3 * b * b - 1) / 6 * h * self.M[i + 1])
            else:
                out[k] = a * self.M[i] + b * self.M[i + 1]
        return out


def _segment_sampler(kfs, smooth, vmax, amax):
    """Return (duration, sampler(local_t)->pose) for one moving segment (>=2 keyframes).

    The sampler is wrapped with a floor guard so no pose ever sends a link under z=0.
    """
    t = np.array([k.t for k in kfs], float)
    t = t - t[0]
    for i in range(1, len(t)):
        if t[i] <= t[i - 1]:
            t[i] = t[i - 1] + 1e-3
    span = float(t[-1])
    poses = [np.array([k.pose[j] for k in kfs], float) for j in range(POSE_LEN)]

    if smooth and len(kfs) >= 2:
        splines = [_CubicSpline(t, poses[j]) for j in range(POSE_LEN)]
        scale = 1.0
        for j in range(POSE_LEN):
            pv = float(np.max(np.abs(splines[j].grid_deriv(1))))
            pa = float(np.max(np.abs(splines[j].grid_deriv(2))))
            if vmax[j] > 0 and pv > 0:
                scale = max(scale, pv / vmax[j])
            if amax[j] > 0 and pa > 0:
                scale = max(scale, (pa / amax[j]) ** 0.5)
        dur = span * scale

        def raw(lt, splines=splines, scale=scale, dur=dur):
            tq = min(max(lt, 0.0), dur) / scale
            return [float(s.pos(tq)) for s in splines]
    else:
        dur = span

        def raw(lt, t=t, poses=poses, span=span):
            tq = min(max(lt, 0.0), span)
            return [float(np.interp(tq, t, poses[j])) for j in range(POSE_LEN)]

    # floor guard: blend toward whichever segment endpoint sits higher above the floor
    c0, c1 = _min_clearance(kfs[0].pose), _min_clearance(kfs[-1].pose)
    anchor = list(kfs[0].pose) if c0 >= c1 else list(kfs[-1].pose)

    def guarded(lt, raw=raw, anchor=anchor):
        return _floor_project(raw(lt), anchor)
    return dur, guarded


class Trajectory:
    """Piecewise trajectory of moves and holds; ``sample(t)`` returns a 5-joint pose."""

    def __init__(self, pose):
        self._const = list(pose)
        self.pieces: list[tuple[float, float, object]] = []   # (t0, dur, sampler)
        self.duration = 0.0

    def sample(self, t):
        if not self.pieces:
            return list(self._const)
        t = min(max(float(t), 0.0), self.duration)
        for t0, dur, fn in self.pieces:
            if t <= t0 + dur:
                return fn(t - t0)
        return self.pieces[-1][2](self.pieces[-1][1])


def _hold(pose):
    p = list(pose)
    return lambda lt, p=p: list(p)


def build_trajectory(seq: Sequence, smooth: bool = True,
                     vmax=DEFAULT_VMAX, amax=DEFAULT_AMAX) -> Trajectory:
    kfs = seq.keyframes
    if not kfs:
        return Trajectory([0.0, 90.0, 0.0, 0.0, 0.0])
    if len(kfs) == 1:
        traj = Trajectory(list(kfs[0].pose))
        if kfs[0].dwell > 0:
            traj.pieces = [(0.0, kfs[0].dwell, _hold(kfs[0].pose))]
            traj.duration = kfs[0].dwell
        return traj

    n = len(kfs)
    stops = sorted(set([0, n - 1] + [i for i, k in enumerate(kfs) if k.dwell > 0]))
    traj = Trajectory(list(kfs[0].pose))
    pieces = []
    cursor = 0.0
    if kfs[0].dwell > 0:                      # hold at the very start
        pieces.append((cursor, kfs[0].dwell, _hold(kfs[0].pose))); cursor += kfs[0].dwell
    for a, b in zip(stops, stops[1:]):
        dur, fn = _segment_sampler(kfs[a:b + 1], smooth, vmax, amax)
        pieces.append((cursor, dur, fn)); cursor += dur
        if kfs[b].dwell > 0:                  # hold on the stop pose
            pieces.append((cursor, kfs[b].dwell, _hold(kfs[b].pose))); cursor += kfs[b].dwell
    traj.pieces = pieces
    traj.duration = cursor
    return traj
