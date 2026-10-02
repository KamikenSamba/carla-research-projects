from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ogm_project.phase1c_height_band import (
    HeightBand,
    benefit_retention,
    dominates,
    false_free_reduction,
    pareto_frontier,
    threshold_summary,
    GroundedRSUReplay,
)
from ogm_project.phase1a_paired import classify


OFFICIAL_RUN = Path(r"D:\CARLA_DATA\outputs\priority_crossing_v2_grounded_20260929_r2")


def candidate(z_min, z_max, benefit, false_free):
    return {
        "z_free_min": z_min,
        "z_free_max": z_max,
        "road_unknown_to_free": benefit,
        "road_benefit_retention": benefit / 4091,
        "target_new_false_free": false_free,
        "false_free_reduction": 1 - false_free / 111,
    }


def test_height_band_requires_finite_increasing_limits():
    assert HeightBand(0.1, 2.0).key == "0.100_2.000"
    for limits in ((1.0, 1.0), (2.0, 1.0), (float("nan"), 2.0)):
        with pytest.raises(ValueError):
            HeightBand(*limits)


def test_normalized_metrics_use_official_baselines():
    assert benefit_retention(4091, 4091) == 1.0
    assert benefit_retention(3272.8, 4091) == pytest.approx(0.8)
    assert false_free_reduction(0, 111) == 1.0
    assert false_free_reduction(111, 111) == 0.0


def test_pareto_frontier_drops_only_dominated_candidates():
    safe = candidate(1.0, 1.5, 3000, 0)
    balanced = candidate(0.8, 1.8, 3800, 10)
    dominated = candidate(0.7, 1.9, 3700, 12)
    benefit = candidate(0.1, 2.0, 4091, 111)

    assert dominates(balanced, dominated)
    assert pareto_frontier([safe, balanced, dominated, benefit]) == [
        safe,
        balanced,
        benefit,
    ]


def test_threshold_summary_respects_safety_and_retention_constraints():
    rows = [
        candidate(1.0, 1.5, 3000, 0),
        candidate(0.8, 1.8, 3700, 8),
        candidate(0.1, 2.0, 4091, 111),
    ]
    summary = threshold_summary(rows)

    assert summary["benefit_thresholds"]["retention_gte_90pct"]["target_new_false_free"] == 8
    assert summary["false_free_thresholds"]["false_free_lte_10"]["road_unknown_to_free"] == 3700


@pytest.fixture(scope="module")
def official_replays():
    if not OFFICIAL_RUN.exists():
        pytest.skip("official grounded Phase 1b.7 capture is not available")
    replay = GroundedRSUReplay(OFFICIAL_RUN)
    current = replay.replay(HeightBand(0.1, 2.0))
    repeated = replay.replay(HeightBand(0.1, 2.0))
    narrower = replay.replay(HeightBand(0.1, 0.65))
    return replay, current, repeated, narrower


def test_official_current_band_is_bit_exact_and_deterministic(official_replays):
    replay, current, repeated, _ = official_replays
    assert np.array_equal(current.logodds, replay.official_phase1a)
    assert np.array_equal(current.logodds, repeated.logodds)
    assert np.array_equal(current.extra_count, repeated.extra_count)
    assert current.metrics == repeated.metrics
    assert current.metrics["road_unknown_to_free"] == 4091
    assert current.metrics["target_new_false_free"] == 111


def test_official_candidate_preserves_valid_and_occupied_updates(official_replays):
    replay, current, _, narrower = official_replays
    assert narrower.metrics["valid_free_updates"] == current.metrics["valid_free_updates"]
    assert narrower.metrics["occupied_endpoint_updates"] == current.metrics["occupied_endpoint_updates"]
    assert narrower.metrics["occupied_to_unknown"] == 0
    assert narrower.metrics["occupied_to_free"] == 0
    assert narrower.metrics["phase0_free_to_unknown"] == 0
    assert np.isfinite(narrower.logodds).all()
    assert np.all(narrower.logodds - replay.phase0 <= 0)


def test_official_target_and_road_metrics_follow_mask_definitions(official_replays):
    replay, _, _, candidate_result = official_replays
    labels, _ = classify(candidate_result.logodds)
    target_new = replay.target & (replay.labels0 != 0) & (labels == 0)
    road_new = replay.road & (replay.labels0 == 1) & (labels == 0)
    assert int(target_new.sum()) == candidate_result.metrics["target_new_false_free"]
    assert int(road_new.sum()) == candidate_result.metrics["road_unknown_to_free"]

