"""FK tests — verify host kinematics matches firmware convention. Run: pytest."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kia_studio.core import Geometry, fk_pose, fk_chain  # noqa: E402

G = Geometry()


def test_fully_vertical():
    # all zero -> arm straight up, no reach
    p = fk_pose(0, 0, 0, 0)
    assert math.isclose(p.x, 0.0, abs_tol=1e-6)
    assert math.isclose(p.y, 0.0, abs_tol=1e-6)
    assert math.isclose(p.z, G.h_base + G.L1 + G.L2 + G.L_tool, abs_tol=1e-6)
    assert math.isclose(p.pitch, -90.0, abs_tol=1e-6)


def test_horizontal_reach():
    # shoulder 90 -> first link horizontal; chain stays in +x at base yaw 0
    p = fk_pose(0, 90, 0, 0)
    assert math.isclose(p.z, G.h_base, abs_tol=1e-6)
    assert math.isclose(p.x, G.L1 + G.L2 + G.L_tool, abs_tol=1e-6)
    assert math.isclose(p.pitch, 0.0, abs_tol=1e-6)


def test_base_yaw():
    p = fk_pose(90, 90, 0, 0)
    assert math.isclose(p.x, 0.0, abs_tol=1e-5)
    assert math.isclose(p.y, G.L1 + G.L2 + G.L_tool, abs_tol=1e-5)


def test_chain_endpoint_matches_pose():
    for angles in [(10, 60, -30, 20), (-45, 120, 40, -10), (0, 0, 0, 0)]:
        chain = fk_chain(*angles)
        p = fk_pose(*angles)
        assert len(chain) == 5
        tcp = chain[-1]
        assert math.isclose(tcp[0], p.x, abs_tol=1e-5)
        assert math.isclose(tcp[1], p.y, abs_tol=1e-5)
        assert math.isclose(tcp[2], p.z, abs_tol=1e-5)


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn(); print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
