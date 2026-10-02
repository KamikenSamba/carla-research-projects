from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ogm_project.grid_contract import GridContract, GridMismatchError
from ogm_project.ogm_diagnostics import OGMDiagnostics
from ogm_project.risk_priority import sha256


def bresenham(ix0, iy0, ix1, iy1):
    dx = abs(ix1 - ix0); sx = 1 if ix0 < ix1 else -1
    dy = -abs(iy1 - iy0); sy = 1 if iy0 < iy1 else -1
    err = dx + dy
    x, y = ix0, iy0
    while True:
        yield x, y
        if x == ix1 and y == iy1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy; x += sx
        if e2 <= dx:
            err += dx; y += sy


def grid():
    return GridContract("1.0", "TownTest", "CARLA_WORLD", 1.0, 5, 5,
                        0.0, 5.0, 0.0, 5.0, 2.5, 2.5,
                        "WORLD_Y", "WORLD_X", "center")


def write_road(tmp_path: Path, road=None, *, center_x=2.5):
    tmp_path.mkdir(parents=True, exist_ok=True)
    road = np.ones((5, 5), dtype=bool) if road is None else np.asarray(road, dtype=bool)
    path = tmp_path / "road.npy"
    meta = tmp_path / "road.json"
    np.save(path, road)
    data = grid().metadata()
    data["center_x"] = center_x
    data.update(mask_semantics="road_true", array_sha256=sha256(path),
                provenance="synthetic unit-test road mask")
    meta.write_text(json.dumps(data), encoding="utf-8")
    return path, meta


def collector(tmp_path, road=None):
    path, meta = write_road(tmp_path, road)
    return OGMDiagnostics(
        tmp_path / "out", grid(), path, meta,
        z_min_world=0.1, z_max_world=2.0, lidar_range=100.0,
        world_to_grid=lambda x, y: (int(x), int(y)),
        in_bounds=lambda x, y: 0 <= x < 5 and 0 <= y < 5,
        bresenham=bresenham,
    )


def observe(diagnostics, sensor="ego"):
    # Valid (1,1)->(3,1): two free cells + one occupied endpoint.
    # Rejected (1,1)->(1,4): three potential-free cells, endpoint excluded.
    points = np.array([[3.2, 1.2, 1.0], [1.2, 4.2, 2.5]], dtype=np.float64)
    diagnostics.observe_measurement(
        sensor, carla_frame=10, timestamp=0.5, raw_return_count=2,
        points_world=points, z_pass_mask=np.array([True, False]),
        origin_xy_world=(1.2, 1.2),
    )


def test_masks_counts_occupied_observed_and_rejected_is_read_only(tmp_path):
    d = collector(tmp_path)
    ego = np.zeros((5, 5), dtype=np.float32)
    rsu = np.zeros((5, 5), dtype=np.float32)
    before_ego = ego.copy(); before_rsu = rsu.copy()
    observe(d, "ego")

    state = d.sensors["ego"]
    assert state.free_touched[1, 1] and state.free_touched[1, 2]
    assert state.free_update_count[1, 1] == 1
    assert state.free_update_count[1, 2] == 1
    assert state.occupied_touched[1, 3]
    assert (state.potential_free[1:4, 1] == [True, True, True]).all()
    np.testing.assert_array_equal(ego, before_ego)
    np.testing.assert_array_equal(rsu, before_rsu)

    summary = d.finalize(ego, rsu)
    np.testing.assert_array_equal(ego, before_ego)
    np.testing.assert_array_equal(rsu, before_rsu)
    assert summary["invariance"] == {
        "ego_logodds_unchanged_by_finalize": True,
        "rsu_logodds_unchanged_by_finalize": True,
    }
    s = summary["sensors"]["ego"]
    assert s["free_cell_update_count_total"] == 2
    assert s["free_cell_update_count_unique"] == 2
    assert s["occupied_cell_update_count_total"] == 1
    assert s["unknown_but_observed"] == 3
    assert s["unknown_untouched_by_current_model"] == 22
    assert s["rejected_ray_potential_free_cells_unique"] == 3
    assert s["potential_free_but_current_unknown_cells"] == 3


def test_free_count_accumulates_and_final_classification_is_correct(tmp_path):
    d = collector(tmp_path)
    points = np.array([[3.2, 1.2, 1.0]], dtype=np.float64)
    for frame in (1, 2):
        d.observe_measurement("rsu", carla_frame=frame, timestamp=frame / 10,
                              raw_return_count=1, points_world=points,
                              z_pass_mask=np.array([True]), origin_xy_world=(1.2, 1.2))
    rsu = np.zeros((5, 5), dtype=np.float32)
    rsu[1, 1] = -1.0       # touched Known Free
    rsu[1, 2] = 0.0        # touched Unknown
    rsu[1, 3] = 1.0        # endpoint Occupied
    summary = d.finalize(np.zeros_like(rsu), rsu)["sensors"]["rsu"]
    assert d.sensors["rsu"].free_update_count[1, 1] == 2
    assert summary["free_touched_and_known_free"] == 1
    assert summary["free_touched_but_unknown"] == 1
    assert summary["free_update_count_final_state"]["2"] == {
        "known_free": 1, "unknown": 1, "occupied": 0}


def test_road_limited_statistics(tmp_path):
    road = np.zeros((5, 5), dtype=bool)
    road[1, 1:4] = True
    d = collector(tmp_path, road)
    observe(d)
    ego = np.zeros((5, 5), dtype=np.float32)
    ego[1, 1] = -1.0
    ego[1, 3] = 1.0
    result = d.finalize(ego, np.zeros_like(ego))["sensors"]["ego"]["road"]
    assert result == {
        "road_total_cells": 3, "road_unknown": 1, "road_known_free": 1,
        "road_occupied": 1, "road_free_touched_but_unknown": 1,
        "road_potential_free_but_current_unknown": 0,
        "road_unknown_untouched_by_current_model": 0,
        "road_phase1_added_free_cells_unique": 0,
        "road_phase1_added_free_to_known_free": 0,
        "road_phase1_added_free_still_unknown": 0,
        "road_phase1_added_free_to_occupied": 0,
    }


def test_shape_and_metadata_mismatch_are_errors(tmp_path):
    path, meta = write_road(tmp_path, center_x=99.0)
    with pytest.raises(GridMismatchError):
        OGMDiagnostics(tmp_path / "out", grid(), path, meta,
                       z_min_world=.1, z_max_world=2, lidar_range=10,
                       world_to_grid=lambda x, y: (int(x), int(y)),
                       in_bounds=lambda x, y: True, bresenham=bresenham)

    d = collector(tmp_path / "shape")
    with pytest.raises(ValueError, match="shape mismatch"):
        d.finalize(np.zeros((4, 5)), np.zeros((5, 5)))


def test_nan_inf_rejected_and_outputs_are_finite(tmp_path):
    d = collector(tmp_path)
    with pytest.raises(ValueError, match="NaN or Inf"):
        d.observe_measurement("ego", carla_frame=1, timestamp=0,
                              raw_return_count=1,
                              points_world=np.array([[1.0, 1.0, np.nan]]),
                              z_pass_mask=np.array([False]), origin_xy_world=(1, 1))
    with pytest.raises(ValueError, match="NaN or Inf"):
        d.finalize(np.full((5, 5), np.inf), np.zeros((5, 5)))


def test_frame_csv_has_measurement_level_values(tmp_path):
    d = collector(tmp_path)
    observe(d)
    d.finalize(np.zeros((5, 5)), np.zeros((5, 5)))
    with (tmp_path / "out" / "ogm_diagnostics_frames.csv").open(encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) == 1
    assert rows[0]["raw_return_count"] == "2"
    assert rows[0]["z_filter_pass_count"] == "1"
    assert rows[0]["z_filter_reject_count"] == "1"
    assert rows[0]["free_cell_update_count_total"] == "2"


def test_phase1_added_free_and_vehicle_false_free_are_separate(tmp_path):
    path, meta = write_road(tmp_path)
    d = OGMDiagnostics(
        tmp_path / "out", grid(), path, meta, z_min_world=.1, z_max_world=2.,
        lidar_range=100., world_to_grid=lambda x, y: (int(x), int(y)),
        in_bounds=lambda x, y: 0 <= x < 5 and 0 <= y < 5,
        bresenham=bresenham, free_space_height_slab=True)
    # Endpoint below the slab: rejected for Occupied, but its ray crosses the slab.
    points = np.array([[4.2, 1.2, 0.0]])
    d.observe_measurement(
        "ego", carla_frame=1, timestamp=.1, raw_return_count=1,
        points_world=points, z_pass_mask=np.array([False]),
        origin_xy_world=(0.2, 1.2), origin_xyz_world=(0.2, 1.2, 3.0))
    ego = np.zeros((5, 5), dtype=np.float32)
    added = d.sensors["ego"].phase1_added_free
    ego[added] = -1.0
    gt = np.zeros((5, 5), dtype=bool)
    gt[1, 2] = True
    summary = d.finalize(ego, np.zeros_like(ego), gt_occupied_mask=gt)["sensors"]["ego"]
    assert summary["phase1_added_free_cells_unique"] == int(added.sum()) > 0
    assert summary["phase1_added_free_to_known_free"] == int(added.sum())
    assert summary["phase1_added_free_still_unknown"] == 0
    assert summary["phase1_added_free_to_occupied"] == 0
    assert summary["vehicle_ground_truth"] == {
        "available": True, "gt_occupied_cells": 1,
        "false_free_cells": 1, "false_free_rate": 1.0}


def test_diagnostics_off_equivalent_path_is_bit_identical(tmp_path, monkeypatch):
    from ogm_project import coop_comm_compat as production
    for name, value in (("ORIGIN_X", 0.0), ("ORIGIN_Y", 0.0),
                        ("X_MIN", 0.0), ("Y_MIN", 0.0), ("RES", 1.0),
                        ("nx", 5), ("ny", 5), ("LIDAR_RANGE", 100.0)):
        monkeypatch.setattr(production, name, value)
    points = np.array([[3.2, 1.2, 1.0]])
    off = {name: np.zeros((5, 5), dtype=np.float32) for name in ("ego", "rsu")}
    on = {name: np.zeros((5, 5), dtype=np.float32) for name in ("ego", "rsu")}
    production.update_from_points(points, (1.2, 1.2), off["ego"], free_scale=1.0)
    production.update_from_points(points, (1.2, 1.2), off["rsu"], free_scale=.15)
    d = collector(tmp_path)
    production.update_from_points(points, (1.2, 1.2), on["ego"], free_scale=1.0)
    production.update_from_points(points, (1.2, 1.2), on["rsu"], free_scale=.15)
    d.observe_measurement("ego", carla_frame=1, timestamp=.1, raw_return_count=1,
                          points_world=points, z_pass_mask=np.array([True]),
                          origin_xy_world=(1.2, 1.2))
    d.observe_measurement("rsu", carla_frame=1, timestamp=.1, raw_return_count=1,
                          points_world=points, z_pass_mask=np.array([True]),
                          origin_xy_world=(1.2, 1.2))
    assert np.array_equal(off["ego"], on["ego"])
    assert np.array_equal(off["rsu"], on["rsu"])
    static = np.zeros((5, 5), dtype=bool)
    assert production.compute_label_counts(off["ego"], off["rsu"], static) == production.compute_label_counts(on["ego"], on["rsu"], static)
    np.testing.assert_array_equal(production.fuse_logodds_prefer_ego(off["ego"], off["rsu"]),
                                  production.fuse_logodds_prefer_ego(on["ego"], on["rsu"]))
    assert production.encode_grid_q8(off["rsu"]) == production.encode_grid_q8(on["rsu"])
