from __future__ import annotations

import zlib

import numpy as np


L0 = 0.0
L_MIN = -4.0
L_MAX = 4.0
OCC_TH = 0.60
FREE_TH = 0.48


def decay_logodds(arr: np.ndarray, dt: float, rate: float) -> None:
    if rate <= 0.0 or dt <= 0.0:
        return
    arr += (L0 - arr) * (rate * dt)
    np.clip(arr, L_MIN, L_MAX, out=arr)


def probability_from_logodds(logodds: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-logodds))


def encode_grid_q8(logodds_arr: np.ndarray) -> bytes:
    arr = np.clip(logodds_arr, L_MIN, L_MAX)
    q = ((arr - L_MIN) / (L_MAX - L_MIN) * 255.0).astype(np.uint8)
    return zlib.compress(q.tobytes(), level=6)


def decode_grid_q8(payload: bytes, ny: int, nx: int) -> np.ndarray:
    q = np.frombuffer(zlib.decompress(payload), dtype=np.uint8).reshape(ny, nx)
    return (q.astype(np.float32) / 255.0) * (L_MAX - L_MIN) + L_MIN


def known_cells(logodds: np.ndarray) -> np.ndarray:
    p = probability_from_logodds(logodds)
    return (p >= OCC_TH) | (p <= FREE_TH)


def fuse_logodds_prefer_ego(ego_logodds: np.ndarray, rsu_logodds: np.ndarray) -> np.ndarray:
    ego_known = known_cells(ego_logodds)
    rsu_known = known_cells(rsu_logodds)
    fused = np.full_like(ego_logodds, L0, dtype=np.float32)
    fused[ego_known] = ego_logodds[ego_known]
    use_rsu = (~ego_known) & rsu_known
    fused[use_rsu] = rsu_logodds[use_rsu]
    return fused
