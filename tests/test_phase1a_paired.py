from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ogm_project.grid_contract import GridContract
from ogm_project.phase1a_paired import Phase1aPairedComparison, transition_matrix


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


def grid():
    return GridContract("1.0", "TownTest", "CARLA_WORLD", 1., 6, 6,
                        0., 6., 0., 6., 3., 3., "WORLD_Y", "WORLD_X", "center")


def paired(tmp_path, road=None):
    road = np.ones((6, 6), dtype=bool) if road is None else road
    return Phase1aPairedComparison(
        tmp_path, grid(), road, z_min=.1, z_max=2., lidar_range=100.,
        free_logodds=-.2, occupied_logodds=1.4, logodds_min=-4., logodds_max=4.,
        world_to_grid=lambda x, y: (int(x), int(y)),
        in_bounds=lambda x, y: 0 <= x < 6 and 0 <= y < 6,
        bresenham=bresenham)


def legacy(points, origin, arr, free_scale=1.):
    ox, oy = origin; start = (int(ox), int(oy))
    if not (0 <= start[0] < 6 and 0 <= start[1] < 6): return
    for hit in points:
        end = (int(hit[0]), int(hit[1]))
        if not (0 <= end[0] < 6 and 0 <= end[1] < 6): continue
        cells = tuple(bresenham(*start, *end))
        for x, y in cells[:-1]: arr[y, x] = np.clip(arr[y, x] - .2 * free_scale, -4, 4)
        x, y = cells[-1]; arr[y, x] = np.clip(arr[y, x] + 1.4, -4, 4)


def test_valid_returns_are_bit_exact_for_ego_and_rsu_with_duplicates_and_boundary(tmp_path):
    p = paired(tmp_path)
    points = np.array([[4.2, 1.2, 1.], [4.2, 1.2, 1.], [5.2, 2.2, 1.], [8., 1., 1.]])
    mask = np.ones(4, dtype=bool)
    for sensor, scale in (("ego", 1.), ("rsu", .15)):
        p.update(sensor, points, mask, (1.2, 1.2, 2.5), scale, legacy, frame=1, timestamp=.1)
        assert np.array_equal(p.phase0[sensor], p.phase1a[sensor])


def test_rejected_crossing_ray_adds_only_free_and_nonpositive_delta(tmp_path):
    p = paired(tmp_path)
    points = np.array([[5.2, 1.2, 0.]])
    p.update("ego", points, np.array([False]), (0.2, 1.2, 3.), 1., legacy,
             frame=1, timestamp=.1)
    assert not p.phase0["ego"].any()
    assert np.count_nonzero(p.phase1a["ego"] < 0) > 0
    assert not np.any(p.phase1a["ego"] > 0)
    assert np.all(p.phase1a["ego"] - p.phase0["ego"] <= 0)


def test_rejected_ray_missing_slab_keeps_shadows_equal(tmp_path):
    p = paired(tmp_path)
    p.update("ego", np.array([[5.2, 1.2, 3.5]]), np.array([False]),
             (1.2, 1.2, 3.), 1., legacy, frame=1, timestamp=.1)
    assert np.array_equal(p.phase0["ego"], p.phase1a["ego"])


def test_shared_decay_uses_one_dt_and_is_independent_of_phase1a_work_time(tmp_path):
    p = paired(tmp_path)
    p.phase0["ego"][1, 1] = p.phase1a["ego"][1, 1] = 2.
    calls = []
    def decay(arr, dt, rate):
        calls.append((dt, rate)); arr += -arr * rate * dt
    p.decay("ego", .25, .4, decay)
    assert calls == [(.25, .4), (.25, .4)]
    assert np.array_equal(p.phase0["ego"], p.phase1a["ego"])


def test_transition_matrix_and_road_subset():
    before = np.array([[0, 1, 2], [0, 1, 2]], dtype=np.uint8)
    after = np.array([[0, 0, 1], [1, 2, 0]], dtype=np.uint8)
    full = transition_matrix(before, after)
    assert full["free"] == {"free": 1, "unknown": 1, "occupied": 0}
    assert full["unknown"] == {"free": 1, "unknown": 0, "occupied": 1}
    assert full["occupied"] == {"free": 1, "unknown": 1, "occupied": 0}
    road = np.array([[True, True, False], [False, False, False]])
    subset = transition_matrix(before, after, road)
    assert subset["free"]["free"] == 1 and subset["unknown"]["free"] == 1
    assert sum(sum(row.values()) for row in subset.values()) == 2


def test_phase1a_off_does_not_touch_production_array(tmp_path):
    production = np.arange(36, dtype=np.float32).reshape(6, 6)
    before = production.copy()
    paired(tmp_path)  # constructing optional diagnostics has no production reference
    np.testing.assert_array_equal(production, before)
