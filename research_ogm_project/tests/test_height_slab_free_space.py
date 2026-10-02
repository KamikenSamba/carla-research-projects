from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ogm_project.height_slab_free_space import (
    ray_segment_in_height_slab,
    trace_height_slab_ray,
    update_from_points_height_slab,
)


def bresenham(ix0, iy0, ix1, iy1):
    dx = abs(ix1 - ix0); sx = 1 if ix0 < ix1 else -1
    dy = -abs(iy1 - iy0); sy = 1 if iy0 < iy1 else -1
    err = dx + dy; x, y = ix0, iy0
    while True:
        yield x, y
        if (x, y) == (ix1, iy1): break
        e2 = 2 * err
        if e2 >= dy: err += dy; x += sx
        if e2 <= dx: err += dx; y += sy


world_to_grid = lambda x, y: (int(x), int(y))
in_bounds = lambda x, y: 0 <= x < 6 and 0 <= y < 6


def trace(sensor, hit):
    return trace_height_slab_ray(sensor, hit, .1, 2., 100.,
                                 world_to_grid, in_bounds, bresenham)


def update(points, sensor):
    arr = np.zeros((6, 6), dtype=np.float32)
    update_from_points_height_slab(
        np.asarray(points, dtype=float), sensor, arr, z_min=.1, z_max=2.,
        lidar_range=100., free_logodds=-.2, occupied_logodds=1.4,
        free_scale=1., logodds_min=-4., logodds_max=4.,
        world_to_grid=world_to_grid, in_bounds=in_bounds, bresenham=bresenham)
    return arr


def test_ray_wholly_above_or_below_slab_has_no_free_update():
    assert ray_segment_in_height_slab((1, 1, 3), (4, 1, 4), .1, 2.) is None
    assert ray_segment_in_height_slab((1, 1, 0), (4, 1, .05), .1, 2.) is None
    assert trace((1.2, 1.2, 3), (4.2, 1.2, 4)).free_cells == ()
    assert trace((1.2, 1.2, 0), (4.2, 1.2, .05)).free_cells == ()


def test_ray_entering_from_above_and_leaving_below_only_carves_slab_segment():
    result = trace((0.2, 1.2, 3.), (5.2, 1.2, 0.))
    assert result.occupied_cell is None
    assert result.free_cells == ((1, 1), (2, 1), (3, 1), (4, 1), (5, 1))


def test_endpoint_inside_has_intermediate_free_and_one_occupied_endpoint():
    result = trace((1.2, 1.2, 1.), (4.2, 1.2, 1.))
    assert result.free_cells == ((1, 1), (2, 1), (3, 1))
    assert result.occupied_cell == (4, 1)
    assert (4, 1) not in result.free_cells
    arr = update([[4.2, 1.2, 1.]], (1.2, 1.2, 1.))
    assert arr[1, 4] == pytest.approx(1.4)
    assert arr[1, 3] == pytest.approx(-.2)


def test_endpoint_below_adds_slab_free_without_occupied():
    result = trace((0.2, 1.2, 3.), (5.2, 1.2, 0.))
    assert result.free_cells
    assert result.occupied_cell is None


def test_endpoint_above_adds_slab_free_without_occupied():
    result = trace((0.2, 1.2, 0.), (5.2, 1.2, 3.))
    assert result.free_cells
    assert result.occupied_cell is None


def test_horizontal_ray_inside_matches_legacy_intermediate_free_endpoint_occupied():
    result = trace((1.2, 2.2, 1.), (4.2, 2.2, 1.))
    assert result.free_cells == tuple(bresenham(1, 2, 4, 2))[:-1]
    assert result.occupied_cell == (4, 2)


def test_horizontal_ray_outside_has_no_updates():
    result = trace((1.2, 2.2, 2.5), (4.2, 2.2, 2.5))
    assert result.free_cells == () and result.occupied_cell is None


def test_grid_boundary_crossing_is_safe_and_preserves_legacy_skip():
    result = trace((1.2, 1.2, 1.), (8.2, 1.2, 0.))
    assert result.free_cells == () and result.occupied_cell is None
    assert result.endpoint_out_of_bounds and result.ray_out_of_bounds
    arr = update([[8.2, 1.2, 0.]], (1.2, 1.2, 1.))
    assert not arr.any()


def test_phase1_off_equivalent_is_covered_by_unchanged_legacy_function():
    # The optional Phase-1 function is separate; importing it cannot alter an
    # existing array or invoke the legacy updater.
    arr = np.arange(36, dtype=np.float32).reshape(6, 6)
    before = arr.copy()
    ray_segment_in_height_slab((1, 1, 1), (2, 2, 1), .1, 2.)
    np.testing.assert_array_equal(arr, before)


def test_phase1_on_adds_free_from_z_rejected_return():
    # Endpoint below z_min: Phase 0 would reject the whole return.
    arr = update([[5.2, 1.2, 0.]], (0.2, 1.2, 3.))
    assert np.count_nonzero(arr < 0) > 0
    assert np.count_nonzero(arr > 0) == 0


def test_occupied_endpoint_never_receives_free_in_same_ray():
    arr = update([[4.2, 1.2, 1.]], (1.2, 1.2, 1.))
    assert arr[1, 4] == pytest.approx(1.4), "endpoint must not contain a -0.2 Free update"


def test_nan_and_inf_are_rejected():
    with pytest.raises(ValueError, match="finite"):
        ray_segment_in_height_slab((0, 0, np.nan), (1, 1, 1), .1, 2.)
    with pytest.raises(ValueError, match="NaN or Inf"):
        update([[1, 1, np.inf]], (0, 0, 1))
