from __future__ import annotations

from pathlib import Path

import numpy as np


def ensure_dir(path: str | Path) -> Path:
    p = Path(path)
    p.mkdir(parents=True, exist_ok=True)
    return p


def write_ply_xyz(path: str | Path, points: np.ndarray) -> None:
    p = Path(path)
    with p.open("w", encoding="ascii") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for x, y, z in points[:, :3]:
            f.write(f"{float(x):.6f} {float(y):.6f} {float(z):.6f}\n")
