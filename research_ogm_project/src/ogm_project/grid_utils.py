from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GridSpec:
    res: float = 0.2
    x_min: float = -30.0
    x_max: float = 30.0
    y_min: float = -30.0
    y_max: float = 30.0
    origin_x: float = 42.0
    origin_y: float = 56.0

    @property
    def nx(self) -> int:
        import math

        return int(math.ceil((self.x_max - self.x_min) / self.res))

    @property
    def ny(self) -> int:
        import math

        return int(math.ceil((self.y_max - self.y_min) / self.res))

    def world_to_grid(self, xw: float, yw: float) -> tuple[int, int]:
        ix = int((xw - (self.origin_x + self.x_min)) / self.res)
        iy = int((yw - (self.origin_y + self.y_min)) / self.res)
        return ix, iy

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny


def bresenham(ix0: int, iy0: int, ix1: int, iy1: int) -> list[tuple[int, int]]:
    cells: list[tuple[int, int]] = []
    dx = abs(ix1 - ix0)
    dy = -abs(iy1 - iy0)
    sx = 1 if ix0 < ix1 else -1
    sy = 1 if iy0 < iy1 else -1
    err = dx + dy
    ix, iy = ix0, iy0
    while True:
        cells.append((ix, iy))
        if ix == ix1 and iy == iy1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            ix += sx
        if e2 <= dx:
            err += dx
            iy += sy
    return cells
