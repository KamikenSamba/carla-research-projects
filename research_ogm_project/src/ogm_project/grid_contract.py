"""Strict spatial contracts for offline Risk, OGM and road-mask layers."""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, fields
from pathlib import Path


class GridMismatchError(ValueError):
    pass


def read_metadata(path):
    try:
        value = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    except (OSError, ValueError) as exc:
        raise GridMismatchError(f'{path}: metadata required and must be valid JSON: {exc}') from exc
    if not isinstance(value, dict):
        raise GridMismatchError(f'{path}: metadata must be an object')
    return value


@dataclass(frozen=True)
class GridContract:
    schema_version: str
    map_name: str
    frame: str
    resolution_m: float
    nx: int
    ny: int
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    center_x: float
    center_y: float
    array_axis_0: str
    array_axis_1: str
    cell_coordinate: str

    def __post_init__(self):
        required = dict(schema_version='1.0', frame='CARLA_WORLD', array_axis_0='WORLD_Y',
                        array_axis_1='WORLD_X', cell_coordinate='center')
        for key, expected in required.items():
            if getattr(self, key) != expected:
                raise GridMismatchError(f'{key}: expected {expected!r}, got {getattr(self, key)!r}')
        if not isinstance(self.map_name, str) or not self.map_name.strip():
            raise GridMismatchError('map_name must be nonempty')
        for key in ('resolution_m', 'x_min_m', 'x_max_m', 'y_min_m', 'y_max_m', 'center_x', 'center_y'):
            v = getattr(self, key)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise GridMismatchError(f'{key}: finite number required')
        if self.resolution_m <= 0:
            raise GridMismatchError('resolution_m must be positive')
        for axis in ('x', 'y'):
            n = getattr(self, 'n' + axis)
            lo, hi = getattr(self, axis + '_min_m'), getattr(self, axis + '_max_m')
            if type(n) is not int or n <= 0:
                raise GridMismatchError(f'n{axis}: positive integer required')
            if hi <= lo or not math.isclose(hi-lo, n*self.resolution_m, abs_tol=1e-8, rel_tol=0):
                raise GridMismatchError(f'{axis}: extent does not match resolution * size')
            if not math.isclose((lo+hi)/2, getattr(self, 'center_'+axis), abs_tol=1e-8, rel_tol=0):
                raise GridMismatchError(f'center_{axis}: inconsistent WORLD bounds')

    @property
    def shape(self):
        return self.ny, self.nx

    @property
    def extent(self):
        return self.x_min_m, self.x_max_m, self.y_min_m, self.y_max_m

    def metadata(self):
        return asdict(self)

    @classmethod
    def from_metadata(cls, meta, *, risk_v1=False):
        meta = dict(meta)
        if risk_v1:
            # Explicit adapter for the already-published Heatmap v1 schema.
            # Missing/unknown conventions are never guessed from array shape.
            expected = {
                'array_convention': 'risk_time[k, iy, ix]; axis 0 of spatial maps is WORLD Y',
                'cell_sampling': 'cell_center',
                'bounds_convention': 'absolute WORLD meters; min inclusive, max exclusive',
            }
            for key, value in expected.items():
                if meta.get(key) != value:
                    raise GridMismatchError(f'Risk v1 {key}: unsupported or missing convention')
            for key, value in dict(array_axis_0='WORLD_Y', array_axis_1='WORLD_X', cell_coordinate='center').items():
                if key in meta and meta[key] != value:
                    raise GridMismatchError(f'Risk v1 {key}: conflicting convention')
                meta[key] = value
        try:
            grid = cls(**{f.name: meta[f.name] for f in fields(cls)})
        except (KeyError, TypeError) as exc:
            raise GridMismatchError(f'incomplete grid metadata: {exc}') from exc
        # OGM runtime origin is an anchor; bounds may be asymmetric around it.
        # Validate the anchor+offset relationship rather than assuming center=origin.
        anchor_keys = ['origin_x', 'origin_y', 'relative_x_min', 'relative_x_max', 'relative_y_min', 'relative_y_max']
        if any(k in meta for k in anchor_keys):
            if not all(k in meta for k in anchor_keys):
                raise GridMismatchError('incomplete origin/relative-bounds metadata')
            for axis in ('x', 'y'):
                for edge in ('min', 'max'):
                    a, b = meta['origin_'+axis], meta[f'relative_{axis}_{edge}']
                    if (isinstance(a, bool) or isinstance(b, bool) or
                            not isinstance(a, (int, float)) or not isinstance(b, (int, float)) or
                            not math.isfinite(a) or not math.isfinite(b) or
                            not math.isclose(a+b, getattr(grid, f'{axis}_{edge}_m'), rel_tol=0, abs_tol=1e-8)):
                        raise GridMismatchError(f'origin_{axis} + relative_{axis}_{edge} conflicts with WORLD bounds')
        return grid


def validate_same_grid(*grids):
    if not grids:
        raise GridMismatchError('at least one GridContract is required')
    first = grids[0]
    if not all(isinstance(g, GridContract) for g in grids):
        raise GridMismatchError('validated GridContract objects required')
    for index, grid in enumerate(grids[1:], 1):
        for f in fields(first):
            a, b = getattr(first, f.name), getattr(grid, f.name)
            equal = math.isclose(a, b, rel_tol=0, abs_tol=1e-8) if f.name in {
                'resolution_m', 'x_min_m', 'x_max_m', 'y_min_m', 'y_max_m', 'center_x', 'center_y'} else a == b
            if not equal:
                raise GridMismatchError(f'grid[{index}] {f.name}: {a!r} != {b!r}')


validate_grid_compatibility = validate_same_grid


def runtime_grid_metadata(*, map_name, origin_x, origin_y, x_min, x_max, y_min, y_max, resolution, nx, ny):
    lo_x, hi_x, lo_y, hi_y = origin_x+x_min, origin_x+x_max, origin_y+y_min, origin_y+y_max
    grid = GridContract('1.0', map_name, 'CARLA_WORLD', resolution, nx, ny,
                        lo_x, hi_x, lo_y, hi_y, (lo_x+hi_x)/2, (lo_y+hi_y)/2,
                        'WORLD_Y', 'WORLD_X', 'center')
    return dict(grid.metadata(), origin_x=origin_x, origin_y=origin_y,
                relative_x_min=x_min, relative_x_max=x_max, relative_y_min=y_min, relative_y_max=y_max,
                bounds_convention='absolute WORLD meters; min inclusive, max exclusive',
                ogm_indexing='legacy int truncation; lower outside strip may enter index 0')
