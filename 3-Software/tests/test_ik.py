"""Inverse kinematics tests — host IK must be the exact inverse of host FK."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kia_studio.core import Geometry, IkStatus, fk_pose, ik, solve_ik  # noqa: E402

G = Geometry()


def test_fk_ik_roundtrip():
    # valid joint sets under the limits (shoulder -90..90, elbow 0..180); elbow >= 0 here
    for (b, s, e, w) in [(0, 60, 30, 10), (30, 70, 40, -20), (-45, 80, 60, 30),
                         (10, 45, 20, 0), (0, 90, 0, 0)]:
        p = fk_pose(b, s, e, w)
        res = ik(p.x, p.y, p.z, p.pitch, elbow_up=False, geo=G)
        assert res.status is IkStatus.OK, (b, s, e, w, res.status)
        q = fk_pose(*res.joints())
        assert math.isclose(q.x, p.x, abs_tol=1e-3)
        assert math.isclose(q.y, p.y, abs_tol=1e-3)
        assert math.isclose(q.z, p.z, abs_tol=1e-3)
        assert math.isclose(q.pitch, p.pitch, abs_tol=1e-3)


def test_unreachable_far():
    res = ik(1000.0, 0.0, 1000.0, 0.0, geo=G)
    assert res.status is IkStatus.UNREACHABLE


def test_limit_violation():
    # reachable in space but base yaw forced beyond +/-90 deg -> LIMIT
    res = ik(-150.0, -10.0, 150.0, 0.0, geo=G)
    assert res.status is IkStatus.LIMIT


def test_solve_tries_both_configs():
    p = fk_pose(0, 70, 50, -10)   # elbow-down solution
    res = solve_ik(p.x, p.y, p.z, p.pitch, geo=G)
    assert res.ok


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn(); print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
