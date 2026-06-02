"""STL loader (binary + ASCII) → indexed vertices/faces, with bbox helpers.

Pure-numpy, no Qt/GL coupling so it stays unit-testable. STL stores no units and no
shared origin convention, so callers recenter/scale via the mesh calibration table.
"""
from __future__ import annotations

import os
import struct
from dataclasses import dataclass

import numpy as np


@dataclass(slots=True)
class Mesh:
    vertices: np.ndarray   # (N, 3) float32, deduplicated
    faces: np.ndarray      # (M, 3) int32 indices into vertices

    @property
    def bbox(self) -> tuple[np.ndarray, np.ndarray]:
        return self.vertices.min(axis=0), self.vertices.max(axis=0)

    @property
    def center(self) -> np.ndarray:
        lo, hi = self.bbox
        return (lo + hi) * 0.5

    @property
    def extent(self) -> np.ndarray:
        lo, hi = self.bbox
        return hi - lo


def _load_raw_triangles(path: str) -> np.ndarray:
    """Return (T, 3, 3) array of triangle vertices, auto-detecting binary vs ASCII STL."""
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        header = f.read(80)
        count_bytes = f.read(4)
        if len(count_bytes) == 4:
            n = struct.unpack("<I", count_bytes)[0]
            if size == 84 + n * 50:                  # exact binary STL size
                data = f.read()
                rec = np.frombuffer(data, dtype=np.uint8).reshape(n, 50)
                tris = np.frombuffer(rec[:, 12:48].copy().tobytes(), dtype="<f4")
                return tris.reshape(n, 3, 3).astype(np.float32)
    # ASCII fallback
    pts: list[tuple[float, float, float]] = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            s = line.strip()
            if s.startswith("vertex"):
                _, x, y, z = s.split()
                pts.append((float(x), float(y), float(z)))
    arr = np.asarray(pts, dtype=np.float32)
    return arr.reshape(-1, 3, 3)


def load_stl(path: str) -> Mesh:
    """Load an STL and return a deduplicated indexed Mesh."""
    tris = _load_raw_triangles(path)
    flat = tris.reshape(-1, 3)
    uniq, inv = np.unique(flat, axis=0, return_inverse=True)
    faces = inv.reshape(-1, 3).astype(np.int32)
    return Mesh(vertices=uniq.astype(np.float32), faces=faces)
