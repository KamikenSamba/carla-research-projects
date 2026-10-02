from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ogm_project.logodds import encode_grid_q8, fuse_logodds_prefer_ego
from ogm_project.ogm_engine import (
    OGMGridContract, OGMUpdateConfig, apply_decay, classify_logodds,
    known_masks, update_lidar_measurement, update_valid_returns,
)
from ogm_project.ogm_engine_shadow import OGMEngineShadowComparison, config_diff


SHAPE = (8, 8)


def world_to_grid(x, y):
    return int(x), int(y)


def in_bounds(x, y):
    return 0 <= x < SHAPE[1] and 0 <= y < SHAPE[0]


def bresenham(ix0, iy0, ix1, iy1):
    dx = abs(ix1 - ix0); sx = 1 if ix0 < ix1 else -1
    dy = -abs(iy1 - iy0); sy = 1 if iy0 < iy1 else -1
    err = dx + dy; x, y = ix0, iy0
    while True:
        yield x, y
        if x == ix1 and y == iy1: break
        e2 = 2 * err
        if e2 >= dy: err += dy; x += sx
        if e2 <= dx: err += dx; y += sy


GRID = OGMGridContract(world_to_grid, in_bounds, bresenham)
EGO = OGMUpdateConfig(.1, 2., 100., float(np.log(.45/.55)),
                      float(np.log(.8/.2)), 1., -4., 4., .48, .60, .4)
RSU = replace(EGO, free_scale=.15, decay_rate=.3)


def legacy_update(points, origin, arr, free_scale=1.):
    ox, oy = origin
    start = world_to_grid(ox, oy)
    if not in_bounds(*start): return
    for xw, yw, _ in points:
        dx, dy = xw-ox, yw-oy
        if dx*dx+dy*dy > 100.*100.: continue
        end = world_to_grid(xw, yw)
        if not in_bounds(*end): continue
        for cx, cy in bresenham(*start, *end):
            delta = EGO.occupied_logodds if (cx, cy) == end else free_scale*EGO.free_logodds
            arr[cy, cx] = np.clip(arr[cy, cx]+delta, -4., 4.)


def legacy_decay(arr, dt, rate):
    if rate <= 0 or dt <= 0: return
    arr += (0.-arr)*(rate*dt)
    np.clip(arr, -4., 4., out=arr)


def points():
    return np.array([[5.2, 1.1, .5], [5.8, 1.2, 2.5], [3.1, 6.2, .1],
                     [7.2, 7.2, 1.], [5.4, 1.4, .7]], dtype=np.float64)


def test_legacy_ego_phase0_bit_exact():
    old = np.zeros(SHAPE, np.float32); new = old.copy(); p = points()
    mask = (p[:, 2] > .1) & (p[:, 2] < 2.)
    legacy_update(p[mask], (1., 1.), old)
    update_lidar_measurement(new, p, (1., 1., 2.), GRID, EGO)
    np.testing.assert_array_equal(old, new)


def test_legacy_rsu_phase0_bit_exact():
    old = np.zeros(SHAPE, np.float32); new = old.copy(); p = points()
    mask = (p[:, 2] > .1) & (p[:, 2] < 2.)
    legacy_update(p[mask], (1., 1.), old, free_scale=.15)
    update_lidar_measurement(new, p, (1., 1., 2.), GRID, RSU)
    np.testing.assert_array_equal(old, new)


def test_shadow_phase0_phase1a_ego_rsu_and_counts(tmp_path):
    shadow = OGMEngineShadowComparison(tmp_path, SHAPE, GRID, EGO, RSU,
                                       legacy_update, legacy_decay)
    for name in ("ego", "rsu"):
        shadow.decay(name, .05)
        shadow.update(name, points(), (1., 1., 2.))
    result = shadow.finalize(encode_grid=encode_grid_q8,
                             fuse=fuse_logodds_prefer_ego,
                             road_mask=np.ones(SHAPE, bool),
                             risk=np.ones(SHAPE, np.float32))
    assert result["all_pass"]
    for name in ("ego", "rsu"):
        assert result["checks"][f"{name}_phase0_bit_exact"]
        assert result["checks"][f"{name}_phase1a_bit_exact"]
        assert result["checks"][f"{name}_phase0_counts_identical"]
        assert result["checks"][f"{name}_phase1a_counts_identical"]


def test_sensor_label_invariance_same_config_same_input():
    ego_named = np.zeros(SHAPE, np.float32); rsu_named = ego_named.copy()
    update_lidar_measurement(ego_named, points(), (1., 1., 2.), GRID, EGO, include_rejected_free=True)
    update_lidar_measurement(rsu_named, points(), (1., 1., 2.), GRID, EGO, include_rejected_free=True)
    np.testing.assert_array_equal(ego_named, rsu_named)


def test_config_difference_is_explicit_and_free_scale_changes_only_free_cells():
    scaled = replace(EGO, free_scale=.15)
    assert config_diff(EGO, scaled) == {"free_scale": {"ego": 1., "rsu": .15}}
    a = np.zeros(SHAPE, np.float32); b = a.copy()
    one = np.array([[5.2, 1.1, .5]])
    update_lidar_measurement(a, one, (1., 1., 2.), GRID, EGO)
    update_lidar_measurement(b, one, (1., 1., 2.), GRID, scaled)
    assert a[1, 5] == b[1, 5]
    assert not np.array_equal(a[1, 1:5], b[1, 1:5])


def test_valid_and_rejected_returns_keep_rejected_endpoint_nonoccupied():
    arr = np.zeros(SHAPE, np.float32)
    p = np.array([[5., 1., .5], [5., 2., 0.]])
    result = update_lidar_measurement(arr, p, (1., 1., 2.), GRID, EGO, include_rejected_free=True)
    assert result.stats.z_pass == 1 and result.stats.z_reject == 1
    assert arr[1, 5] > 0
    assert arr[2, 5] <= 0
    assert result.stats.extra_free_updates > 0


def test_grid_boundary_and_out_of_bounds_are_unchanged():
    old = np.zeros(SHAPE, np.float32); new = old.copy()
    p = np.array([[7.9, 7.9, .5], [8., 3., .5]])
    legacy_update(p, (0., 0.), old)
    update_valid_returns(new, p, (0., 0., 2.), GRID, EGO)
    np.testing.assert_array_equal(old, new)


def test_multiple_rays_same_cell_and_clipping_bit_exact():
    p = np.repeat(np.array([[5.2, 1.1, .5]]), 30, axis=0)
    old = np.zeros(SHAPE, np.float32); new = old.copy()
    legacy_update(p, (1., 1.), old)
    update_lidar_measurement(new, p, (1., 1., 2.), GRID, EGO)
    np.testing.assert_array_equal(old, new)
    assert new[1, 5] == EGO.logodds_max


def test_decay_bit_exact_and_noop():
    old = np.array([[4., -4.]], np.float32); new = old.copy()
    legacy_decay(old, .125, .4); apply_decay(new, .125, EGO)
    np.testing.assert_array_equal(old, new)
    frozen = new.copy(); apply_decay(new, 0., EGO)
    np.testing.assert_array_equal(frozen, new)


def test_known_free_occupied_unknown_thresholds():
    p = np.array([[.47, .48, .5, .60, .61]], np.float64)
    logodds = np.log(p/(1-p))
    free, occupied, unknown = known_masks(logodds, EGO)
    np.testing.assert_array_equal(free, [[True, True, False, False, False]])
    np.testing.assert_array_equal(occupied, [[False, False, False, True, True]])
    np.testing.assert_array_equal(unknown, [[False, False, True, False, False]])
    np.testing.assert_array_equal(classify_logodds(logodds, EGO), [[1, 1, 0, 2, 2]])


def test_communication_fusion_and_priority_invariants(tmp_path):
    shadow = OGMEngineShadowComparison(tmp_path, SHAPE, GRID, EGO, RSU,
                                       legacy_update, legacy_decay)
    shadow.update("ego", points(), (1., 1., 2.)); shadow.update("rsu", points(), (1., 1., 2.))
    result = shadow.finalize(encode_grid=encode_grid_q8, fuse=fuse_logodds_prefer_ego,
                             road_mask=np.ones(SHAPE, bool), risk=np.arange(64).reshape(SHAPE))
    assert result["checks"]["communication_payload_bit_exact"]
    assert result["checks"]["fusion_bit_exact"]
    assert result["checks"]["ego_unknown_identical"]
    assert result["checks"]["rsu_known_identical"]
    assert result["checks"]["priority_bit_exact"]


def test_diagnostics_stats_do_not_change_grid_and_are_finite():
    a = np.zeros(SHAPE, np.float32); b = a.copy()
    result = update_lidar_measurement(a, points(), (1., 1., 2.), GRID, EGO)
    update_valid_returns(b, points()[result.z_pass_mask], (1., 1., 2.), GRID, EGO)
    np.testing.assert_array_equal(a, b)
    assert np.isfinite(a).all()
    assert all(v >= 0 for v in result.stats.as_dict().values())


def test_nan_inf_rejected_without_grid_mutation():
    for bad in (np.nan, np.inf, -np.inf):
        arr = np.zeros(SHAPE, np.float32)
        with np.testing.assert_raises(ValueError):
            update_lidar_measurement(arr, np.array([[2., 2., bad]]), (1., 1., 2.), GRID, EGO)
        assert not arr.any()
