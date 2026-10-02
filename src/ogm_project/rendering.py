from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image

from .logodds import FREE_TH, OCC_TH, probability_from_logodds


def labels_from_logodds(logodds: np.ndarray) -> np.ndarray:
    p = probability_from_logodds(logodds)
    labels = np.zeros(logodds.shape, dtype=np.uint8)
    labels[p >= OCC_TH] = 255
    labels[p <= FREE_TH] = 80
    labels[(p > FREE_TH) & (p < OCC_TH)] = 160
    return labels


def save_label_png(path: str | Path, logodds: np.ndarray) -> None:
    Image.fromarray(labels_from_logodds(logodds)).save(path)


def save_heatmap_png(path: str | Path, logodds: np.ndarray) -> None:
    clipped = np.clip(logodds, -4.0, 4.0)
    norm = ((clipped + 4.0) / 8.0 * 255.0).astype(np.uint8)
    Image.fromarray(norm).save(path)
