"""Geometry calibration tests — derive link lengths from pivots, persist, firmware export."""
import math
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kia_studio.core.kinematics import (geometry_from_pivots, save_geometry,  # noqa: E402
                                        load_geometry, firmware_geometry_header, Geometry)


def test_geometry_from_pivots_matches_distances():
    pivots = {"base": (0, 0, 0), "shoulder": (0, 0, 60),
              "elbow": (105, 0, 60), "wrist": (205, 0, 60)}
    geo = geometry_from_pivots(pivots, (290, 0, 60))
    assert math.isclose(geo.h_base, 60, abs_tol=1e-6)
    assert math.isclose(geo.L1, 105, abs_tol=1e-6)
    assert math.isclose(geo.L2, 100, abs_tol=1e-6)
    assert math.isclose(geo.L_tool, 85, abs_tol=1e-6)


def test_geometry_uses_3d_distance():
    pivots = {"base": (0, 0, 0), "shoulder": (0, 0, 50),
              "elbow": (30, 0, 90), "wrist": (30, 0, 90)}   # shoulder->elbow = 3-4-5 -> 50
    geo = geometry_from_pivots(pivots, (30, 0, 90))
    assert math.isclose(geo.L1, 50.0, abs_tol=1e-6)


def test_save_load_roundtrip():
    geo = Geometry(61.2, 104.5, 99.1, 87.3)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "geometry.json")
        save_geometry(geo, p)
        r = load_geometry(p)
    assert (round(r.h_base, 1), round(r.L1, 1), round(r.L2, 1), round(r.L_tool, 1)) \
        == (61.2, 104.5, 99.1, 87.3)


def test_load_missing_returns_nominal():
    geo = load_geometry(os.path.join(tempfile.gettempdir(), "no_such_geom_kia.json"))
    assert (geo.h_base, geo.L1, geo.L2, geo.L_tool) == (60.0, 105.0, 100.0, 85.0)


def test_firmware_header_has_macros():
    h = firmware_geometry_header(Geometry(60, 105, 100, 85))
    for macro in ("KIA_LINK_BASE_HEIGHT_MM", "KIA_LINK_UPPER_ARM_MM",
                  "KIA_LINK_FOREARM_MM", "KIA_LINK_TOOL_MM"):
        assert macro in h
    assert "105.0f" in h and "85.0f" in h


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn(); print(f"  ok  {fn.__name__}")
    print(f"\n{len(fns)} passed")
