from __future__ import annotations

from typing import Protocol

import numpy as np


class _TransformLike(Protocol):
    def get_matrix(self): ...


def transform_to_world(points_xyz: np.ndarray, transform: _TransformLike) -> np.ndarray:
    """Convert LiDAR-local XYZ points to WORLD coordinates."""
    mat = np.asarray(transform.get_matrix(), dtype=np.float32)
    ones = np.ones((points_xyz.shape[0], 1), dtype=np.float32)
    pts_h = np.hstack((points_xyz.astype(np.float32), ones))
    return (pts_h @ mat.T)[:, :3]


def transform_world_to_ego(points_world: np.ndarray, ego_transform: _TransformLike) -> np.ndarray:
    """Convert WORLD XYZ points to Ego vehicle coordinates."""
    mat_ego_w = np.asarray(ego_transform.get_matrix(), dtype=np.float32)
    mat_w_ego = np.linalg.inv(mat_ego_w)
    ones = np.ones((points_world.shape[0], 1), dtype=np.float32)
    pts_h = np.hstack((points_world.astype(np.float32), ones))
    return (pts_h @ mat_w_ego.T)[:, :3]


def rotate_xy(x: float, y: float, yaw_deg: float) -> tuple[float, float]:
    th = np.deg2rad(yaw_deg)
    c = np.cos(th)
    s = np.sin(th)
    return float(c * x - s * y), float(s * x + c * y)


def ego_to_world_xy(xe: float, ye: float, ego_transform) -> tuple[float, float]:
    xr, yr = rotate_xy(xe, ye, ego_transform.rotation.yaw)
    return float(ego_transform.location.x + xr), float(ego_transform.location.y + yr)
