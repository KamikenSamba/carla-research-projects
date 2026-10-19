from dataclasses import replace
import csv
import json
import math
import random

import pytest

from Estimate_IDM_T import main
from idm_history import FitConfig, estimate_headway, estimate_traits, read_history
from idm_model import IDMParameters, compute_idm_acceleration
from idm_snapshot import load_traits, resolve_parameters
import Run_DT_Risk_V2_IDM as runner


PARAMETERS = IDMParameters(desired_speed_mps=18)


def following(t=1.5, logical="follower", count=240, measured=True, noise=0.0):
    """Integrate IDM following a sinusoidally changing leader; deterministic noise."""
    p, rng = replace(PARAMETERS, desired_time_headway_s=t), random.Random(42)
    speed, gap, rows, dt = 8.0, 22.0, [], 0.05
    for i in range(count):
        leader_speed = 8 + 1.3 * math.sin(i * dt / 2)
        acceleration = compute_idm_acceleration(speed, p, gap, leader_speed)
        row = dict(logical_actor_id=logical, source_actor_id=1, carla_frame=i,
                   sim_time_s=i * dt, speed_mps=speed + rng.gauss(0, noise),
                   leader_id="leader", leader_speed_mps=leader_speed, gap_m=gap,
                   road_id=1, section_id=0, lane_id=-1, signal_stop=False,
                   lane_change=False, is_junction=False, fallback_reason="")
        if measured:
            row["observed_acceleration_mps2"] = acceleration
        rows.append(row)
        next_speed = max(0, speed + acceleration * dt)
        gap += (leader_speed - (speed + next_speed) / 2) * dt
        speed = next_speed
    return rows


@pytest.mark.parametrize("t", [0.8, 1.5, 2.5])
def test_recovers_known_headway(t):
    result = estimate_headway(following(t), PARAMETERS)
    assert result["accepted"] and result["identifiable"]
    assert result["estimated_t_s"] == pytest.approx(t, abs=0.011)
    assert result["rmse_mps2"] < 1e-9
    assert result["valid_samples"] == 234


def test_distinguishes_vehicles_and_exports_existing_v2_traits(tmp_path):
    rows = following(0.8, "short") + following(2.5, "long")
    traits, diagnostics = estimate_traits(rows, {key: PARAMETERS for key in ["short", "long"]})
    path = tmp_path / "traits.json"
    path.write_text(json.dumps(traits), encoding="utf-8")
    args = runner.parse_args(["--mode", "predict", "--snapshot", "unused.csv",
                              "--actor-traits", str(path)])
    resolved = {logical: resolve_parameters(dict(logical_actor_id=logical, speed_mps=8), args, args.traits)
                for logical in traits}
    assert resolved["short"].desired_time_headway_s == pytest.approx(0.8)
    assert resolved["long"].desired_time_headway_s == pytest.approx(2.5)
    assert resolved["short"].desired_speed_mps == 18
    assert compute_idm_acceleration(8, resolved["short"], 22, 8) > compute_idm_acceleration(8, resolved["long"], 22, 8)
    assert all(d["adopted"] for d in diagnostics["actors"].values())
    assert load_traits(path) == traits


def test_smoothed_speed_derivative_with_noise():
    result = estimate_headway(following(1.8, measured=False, noise=0.025), PARAMETERS)
    assert result["accepted"], result
    assert result["estimated_t_s"] == pytest.approx(1.8, abs=0.12)
    assert result["rmse_mps2"] < 0.2


@pytest.mark.parametrize("field,value,reason", [
    ("speed_mps", 0, "low_speed"), ("signal_stop", True, "signal_stop"),
    ("lane_change", True, "lane_change"), ("is_junction", True, "junction"),
    ("leader_id", "", "no_leader"), ("gap_m", -1, "overlap"),
    ("fallback_reason", "route_end", "fallback"),
    ("leader_speed_mps", "nan", "invalid_context"),
    ("signal_stop", "", "invalid_context"),
])
def test_exclusions_fall_back(field, value, reason):
    rows = following()
    for row in rows:
        row[field] = value
    result = estimate_headway(rows, PARAMETERS)
    assert not result["accepted"] and result["used_t_s"] == 1.5
    assert result["exclusions"][reason] == len(rows)


def test_missing_context_and_single_snapshot_do_not_estimate():
    rows = [dict(logical_actor_id="follower", sim_time_s=0, speed_mps=8)]
    result = estimate_headway(rows, PARAMETERS)
    assert not result["accepted"] and result["exclusions"] == {"missing_context": 1}
    assert not estimate_headway(following(count=1), PARAMETERS)["accepted"]


def test_duplicates_excluded_including_conflicts():
    rows = following()
    result = estimate_headway(rows + [dict(row, speed_mps=2) for row in rows], PARAMETERS)
    assert not result["accepted"] and result["exclusions"]["duplicate_time"] == 480


@pytest.mark.parametrize("change", ["lane_id", "leader_id", "source_actor_id"])
def test_derivative_does_not_cross_identity_or_lane_boundaries(change):
    rows = following(measured=False)
    for i, row in enumerate(rows):
        row[change] = i // 5
    result = estimate_headway(rows, PARAMETERS)
    assert not result["accepted"]
    assert result["exclusions"]["discontinuity"] > 0


def test_missing_frames_dont_bridge_derivative_window():
    rows = following(measured=False)
    result = estimate_headway([r for i, r in enumerate(rows) if i % 8 < 4], PARAMETERS)
    assert not result["accepted"]
    assert result["exclusions"]["discontinuity"] > 0


def test_sparse_missing_values_excluded_with_surrounding_windows():
    rows = following()
    rows[100]["gap_m"] = ""
    result = estimate_headway(rows, PARAMETERS)
    assert result["accepted"]
    assert result["exclusions"]["invalid_context"] == 1
    assert result["valid_samples"] == 227


@pytest.mark.parametrize("bad", [None, "nan", "inf"])
def test_unlocatable_time_fails_closed(bad):
    rows = following()
    rows[100]["sim_time_s"] = bad
    result = estimate_headway(rows, PARAMETERS)
    assert result["reason"] == "invalid_time_or_speed" and not result["accepted"]


def test_saturation_not_mistaken_for_identifiability():
    rows = following()
    for row in rows:
        row.update(gap_m=0.01, observed_acceleration_mps2=-4)
    result = estimate_headway(rows, PARAMETERS)
    assert result["rmse_mps2"] == 0
    assert not result["identifiable"] and not result["accepted"]


def test_boundary_fit_rejected():
    result = estimate_headway(following(4.5), PARAMETERS)
    assert result["boundary_reached"] and result["reason"] == "search_boundary"
    assert result["used_t_s"] == 1.5


def test_weak_interaction_is_unidentifiable_despite_low_rmse():
    rows = following()
    for row in rows:
        row["gap_m"] = 10000
        row["observed_acceleration_mps2"] = compute_idm_acceleration(
            row["speed_mps"], PARAMETERS, row["gap_m"], row["leader_speed_mps"])
    result = estimate_headway(rows, PARAMETERS)
    assert result["rmse_mps2"] == 0
    assert result["reason"] == "unidentifiable" and not result["accepted"]


def test_sample_count_alone_does_not_replace_duration():
    rows = following()
    for row in rows:
        row["sim_time_s"] /= 10
    result = estimate_headway(rows, PARAMETERS)
    assert result["valid_samples"] > 20
    assert result["valid_duration_s"] < 2 and not result["accepted"]


def test_unknown_history_is_reported_unknown_explicit_actor_rejected():
    _, diagnostic = estimate_traits(following(logical="external"), {"follower": PARAMETERS})
    assert diagnostic["ignored_history_ids"] == ["external"]
    with pytest.raises(ValueError, match="unknown"):
        estimate_traits([], {"follower": PARAMETERS}, {"external": {"desired_time_headway_s": 2}})


def test_explicit_v0_must_match_fixed_fitting_parameter():
    with pytest.raises(ValueError, match="resolved explicit desired speed"):
        estimate_traits(following(), {"follower": PARAMETERS}, {"follower": {"desired_speed_mps": 20}})


def test_model_mismatch_rejected():
    rows = following()
    for i, row in enumerate(rows):
        row["observed_acceleration_mps2"] += 2 if i % 2 else -2
    result = estimate_headway(rows, PARAMETERS)
    assert not result["accepted"] and result["rmse_mps2"] > 1


def test_numerical_failure_does_not_emit_nonfinite_json():
    rows = following()
    for row in rows:
        row["observed_acceleration_mps2"] = 1e300
    result = estimate_headway(rows, PARAMETERS)
    assert result["reason"] == "numerical_failure"
    json.dumps(result, allow_nan=False)


def test_trait_snapshot_and_estimate_precedence():
    rows = following(2)
    parameters = {"follower": PARAMETERS}
    for explicit, snapshot, replace_snapshot, expected, reason in [
        ({"follower": {"desired_time_headway_s": 0.9, "desired_speed_mps": 18}}, {"follower": 1.2}, True, 0.9, "explicit_trait"),
        ({}, {"follower": 1.2}, False, 1.2, "snapshot_headway"),
        ({}, {"follower": 1.2}, True, 2.0, "accepted"),
    ]:
        traits, diagnostic = estimate_traits(rows, parameters, explicit, snapshot, replace_snapshot)
        assert traits["follower"]["desired_time_headway_s"] == pytest.approx(expected)
        assert diagnostic["actors"]["follower"]["adoption_reason"] == reason


def test_future_history_excluded_and_absent_actor_falls_back():
    rows = following(0.8)
    for row in following(2.5):
        rows.append(dict(row, sim_time_s=row["sim_time_s"] + 12))
    traits, diagnostics = estimate_traits(rows, {"follower": PARAMETERS, "missing": PARAMETERS},
                                         cutoff_times={"follower": 11.95, "missing": 11.95})
    assert traits["follower"]["desired_time_headway_s"] == pytest.approx(0.8)
    assert diagnostics["actors"]["follower"]["future_samples_excluded"] == 240
    assert traits["missing"]["desired_time_headway_s"] == 1.5
    assert not diagnostics["actors"]["missing"]["adopted"]


def write_csv(path, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def test_context_join_exact_and_never_overwrites_observed_speed(tmp_path):
    history, context = tmp_path / "capture.csv", tmp_path / "context.csv"
    write_csv(history, [dict(logical_actor_id="follower", carla_frame=1, sim_time_s=0.05, speed_mps=8)])
    write_csv(context, [dict(logical_actor_id="follower", carla_frame=1, speed_mps=99, gap_m=12)])
    rows = read_history(history, context)
    assert rows[0]["speed_mps"] == "8" and rows[0]["gap_m"] == "12"
    assert not estimate_headway(rows, PARAMETERS)["accepted"]
    write_csv(context, [dict(logical_actor_id="follower", carla_frame=1)] * 2)
    with pytest.raises(ValueError, match="duplicate"):
        read_history(history, context)


def test_cli_exports_complete_files_and_no_simulator_import(tmp_path, monkeypatch):
    import builtins
    original = builtins.__import__
    def guard(name, *args, **kwargs):
        if name == "carla":
            raise AssertionError("offline estimator must not import CARLA")
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", guard)
    from test_idm import old_row
    snapshot = tmp_path / "snapshot.json"
    snapshot.write_text(json.dumps([dict(old_row(), logical_actor_id="follower", sim_time_s=12)]), encoding="utf-8")
    history, output = tmp_path / "history.csv", tmp_path / "fit"
    write_csv(history, following(1.8))
    argv = ["--history", str(history), "--snapshot", str(snapshot), "--output-dir", str(output),
            "--idm-desired-speed-mps", "18"]
    assert main(argv) == 0
    assert load_traits(output / "actor_traits.json")["follower"]["desired_time_headway_s"] == pytest.approx(1.8)
    diagnostics = json.loads((output / "headway_diagnostics.json").read_text())
    assert diagnostics["actors"]["follower"]["adopted"]
    assert len(diagnostics["input_sha256"]["history"]) == 64
    args = runner.parse_args(["--mode", "predict", "--snapshot", str(snapshot),
                             "--actor-traits", str(output / "actor_traits.json")])
    received = {}
    def predict(a, directory, path, logger, parameters):
        received.update(parameters)
        return {"tested": True}
    monkeypatch.setattr(runner, "run_idm_prediction", predict)
    import logging
    assert runner.run_prediction(args, output, snapshot, logging.getLogger("test")) == {"tested": True}
    assert received["follower"].desired_time_headway_s == pytest.approx(1.8)
    resolved = json.loads((output / "resolved_idm_parameters.json").read_text())
    assert resolved["follower"]["desired_time_headway_s"] == pytest.approx(1.8)
    with pytest.raises(SystemExit):
        main(argv)  # Never overwrite a previous run.


@pytest.mark.parametrize("settings", [dict(minimum_t_s=4, maximum_t_s=3),
    dict(grid_step_s=0), dict(derivative_radius=0), dict(maximum_step_s=float("nan"))])
def test_invalid_fit_config(settings):
    with pytest.raises(ValueError):
        FitConfig(**settings)
