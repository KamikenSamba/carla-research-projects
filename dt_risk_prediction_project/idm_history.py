"""Offline headway fitting. No CARLA, numerical packages or simulator calls."""
from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, replace
import csv
import json
import math
from pathlib import Path
from statistics import fmean

from idm_model import IDMParameters, compute_idm_acceleration, finite


CONTEXT_FIELDS = (
    "leader_id", "leader_speed_mps", "gap_m", "road_id", "section_id", "lane_id",
    "signal_stop", "lane_change", "is_junction", "fallback_reason",
)


@dataclass(frozen=True)
class FitConfig:
    minimum_t_s: float = 0.3
    maximum_t_s: float = 4.0
    grid_step_s: float = 0.01
    minimum_speed_mps: float = 1.0
    minimum_samples: int = 20
    minimum_duration_s: float = 2.0
    maximum_step_s: float = 0.15
    derivative_radius: int = 3
    maximum_rmse_mps2: float = 0.5
    minimum_response_mps2: float = 0.1
    profile_tolerance_mps2: float = 0.05
    maximum_profile_width_s: float = 1.0

    def __post_init__(self):
        for name, value in asdict(self).items():
            finite(value, name, 0.0)
        if (self.maximum_t_s <= self.minimum_t_s or self.grid_step_s <= 0
                or self.maximum_step_s <= 0 or self.minimum_samples < 2
                or self.derivative_radius < 1 or self.minimum_duration_s <= 0
                or self.maximum_profile_width_s <= 0):
            raise ValueError("invalid fit configuration")
        if (not isinstance(self.minimum_samples, int) or not isinstance(self.derivative_radius, int)
                or (self.maximum_t_s - self.minimum_t_s) / self.grid_step_s > 10000):
            raise ValueError("sample/radius must be integers; grid limited to 10001 points")


def read_history(path: Path, context_path: Path | None = None) -> list[dict]:
    """Read enriched CSV or exact (logical ID, CARLA frame) sidecar join.

    Sidecar may supply context and measured longitudinal acceleration, never
    replace observed speed/time. Duplicate context keys are a hard error.
    Missing context remains missing and is rejected by the quality filters.
    """
    def read(p):
        with p.open(encoding="utf-8-sig", newline="") as stream:
            return list(csv.DictReader(stream))

    rows = read(path)
    if context_path is None:
        return rows
    metadata_path = context_path.with_suffix(".meta.json")
    metadata = None
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if not isinstance(metadata, dict) or metadata.get("status") != "completed":
            raise ValueError("context capture is incomplete or failed")
    context = {}
    for row in read(context_path):
        key = (row.get("logical_actor_id"), row.get("carla_frame"))
        if not all(key) or key in context:
            raise ValueError("missing or duplicate history-context key")
        context[key] = row
    if metadata is not None:
        capture_keys = {(row.get("logical_actor_id"), row.get("carla_frame")) for row in rows}
        if (metadata.get("rows") != len(context) or len(rows) != len(context)
                or capture_keys != set(context)):
            raise ValueError("capture/context keys or metadata row count differ")
    for row in rows:
        extra = context.get((row.get("logical_actor_id"), row.get("carla_frame")), {})
        for name in (*CONTEXT_FIELDS, "observed_acceleration_mps2"):
            if name in extra:
                row[name] = extra[name]
    return rows


def _number(row, name):
    return finite(row.get(name), name)


def _flag(value):
    if str(value).lower() in {"false", "0"}:
        return False
    if str(value).lower() in {"true", "1"}:
        return True
    raise ValueError("missing or invalid context flag")


def _parse(row, config):
    try:
        time = _number(row, "sim_time_s")
        speed = _number(row, "speed_mps")
    except (ValueError, TypeError):
        return None, "invalid_time_or_speed"
    base = dict(time=time, speed=speed, source=str(row.get("source_actor_id", "")))
    if any(name not in row for name in CONTEXT_FIELDS):
        return base, "missing_context"
    try:
        flags = {name: _flag(row[name]) for name in ("signal_stop", "lane_change", "is_junction")}
        lane = tuple(int(row[name]) for name in ("road_id", "section_id", "lane_id"))
        if row["fallback_reason"] is None:
            raise ValueError("unknown fallback state")
        if flags["signal_stop"]:
            return base, "signal_stop"
        if flags["lane_change"]:
            return base, "lane_change"
        if flags["is_junction"]:
            return base, "junction"
        if row["fallback_reason"]:
            return base, "fallback"
        if speed < config.minimum_speed_mps:
            return base, "low_speed"
        if not str(row["leader_id"]).strip() or row["leader_id"] is None:
            return base, "no_leader"
        gap, leader_speed = _number(row, "gap_m"), _number(row, "leader_speed_mps")
        if gap <= 0:
            return base, "overlap"
        if leader_speed < 0:
            raise ValueError("negative leader speed")
        acceleration = row.get("observed_acceleration_mps2")
        acceleration = None if acceleration in (None, "") else finite(acceleration, "acceleration")
        base.update(lane=lane, leader=str(row["leader_id"]), gap=gap,
                    leader_speed=leader_speed, acceleration=acceleration)
        return base, ""
    except (TypeError, ValueError, OverflowError):
        return base, "invalid_context"


def _samples(rows, config):
    parsed = [_parse(row, config) for row in rows]
    excluded = Counter(reason for _, reason in parsed if reason)
    # An unlocatable invalid timestamp could hide a discontinuity anywhere.
    if any(sample is None for sample, _ in parsed):
        return [], dict(excluded), "invalid_time_or_speed"
    ordered = sorted(parsed, key=lambda pair: pair[0]["time"])
    counts = Counter(sample["time"] for sample, _ in ordered)
    for i, (sample, reason) in enumerate(ordered):
        if counts[sample["time"]] > 1:
            if reason:
                excluded[reason] -= 1
            ordered[i] = sample, "duplicate_time"
            excluded["duplicate_time"] += 1
    radius, result = config.derivative_radius, []
    for i, (sample, reason) in enumerate(ordered):
        if reason:
            continue
        window = ordered[max(0, i - radius):i + radius + 1]
        if i < radius or i + radius >= len(ordered):
            excluded["window_edge"] += 1
            continue
        if any(why for _, why in window):
            excluded["invalid_neighbor"] += 1
            continue
        points = [p for p, _ in window]
        if (any((p["lane"], p["leader"], p["source"]) !=
                (sample["lane"], sample["leader"], sample["source"]) for p in points)
                or any(b["time"] - a["time"] > config.maximum_step_s
                       for a, b in zip(points, points[1:]))):
            excluded["discontinuity"] += 1
            continue
        acceleration = sample["acceleration"]
        if acceleration is None:
            # Centred local linear regression: smoothed derivative on actual
            # timestamps, never across a lane/leader change or rejected sample.
            mean_t, mean_v = fmean(p["time"] for p in points), fmean(p["speed"] for p in points)
            denominator = sum((p["time"] - mean_t) ** 2 for p in points)
            acceleration = sum((p["time"] - mean_t) * (p["speed"] - mean_v)
                               for p in points) / denominator
        result.append(dict(sample, observed=acceleration))
    return result, {key: count for key, count in excluded.items() if count}, ""


def estimate_headway(rows: list[dict], parameters: IDMParameters,
                     config: FitConfig = FitConfig()) -> dict:
    """Fit one actor's T with all other parameters fixed; return diagnostics.

    The profile interval is a loss-tolerance diagnostic, NOT a statistical CI.
    The reused IDM function includes its existing acceleration bounds.
    """
    samples, excluded, input_failure = _samples(rows, config)
    duration = sum(b["time"] - a["time"] for a, b in zip(samples, samples[1:])
                   if 0 < b["time"] - a["time"] <= config.maximum_step_s
                   and (a["lane"], a["leader"], a["source"]) ==
                       (b["lane"], b["leader"], b["source"]))
    result = dict(input_samples=len(rows), valid_samples=len(samples), valid_duration_s=duration,
                  measured_acceleration_samples=sum(s["acceleration"] is not None for s in samples),
                  derived_acceleration_samples=sum(s["acceleration"] is None for s in samples),
                  exclusions=excluded, accepted=False, reason="insufficient_samples",
                  estimated_t_s=None, used_t_s=parameters.desired_time_headway_s,
                  rmse_mps2=None, boundary_reached=False, identifiable=False,
                  profile_interval_s=None, response_span_mps2=None,
                  fixed_parameters={key: value for key, value in asdict(parameters).items()
                                    if key != "desired_time_headway_s"}, config=asdict(config))
    if input_failure:
        result["reason"] = input_failure
        return result
    if len(samples) < config.minimum_samples or duration < config.minimum_duration_s:
        return result
    steps = math.ceil((config.maximum_t_s - config.minimum_t_s) / config.grid_step_s)
    grid = [min(config.maximum_t_s, config.minimum_t_s + i * config.grid_step_s)
            for i in range(steps + 1)]

    def predictions(t):
        p = replace(parameters, desired_time_headway_s=t)
        return [compute_idm_acceleration(s["speed"], p, s["gap"], s["leader_speed"]) for s in samples]

    try:
        losses = [fmean((predicted - s["observed"]) ** 2
                        for predicted, s in zip(predictions(t), samples)) for t in grid]
        if not all(math.isfinite(loss) for loss in losses):
            raise ValueError("nonfinite loss")
    except (OverflowError, ValueError):
        result["reason"] = "numerical_failure"
        return result
    best = min(range(len(grid)), key=losses.__getitem__)
    near = [t for t, loss in zip(grid, losses)
            if loss <= losses[best] + config.profile_tolerance_mps2 ** 2]
    response = math.sqrt(fmean((a - b) ** 2 for a, b in
                              zip(predictions(grid[0]), predictions(grid[-1]))))
    identifiable = (response >= config.minimum_response_mps2
                    and near[-1] - near[0] <= config.maximum_profile_width_s)
    boundary = best in (0, len(grid) - 1)
    rmse = math.sqrt(losses[best])
    reason = ("search_boundary" if boundary else "unidentifiable" if not identifiable
              else "poor_fit" if rmse > config.maximum_rmse_mps2 else "accepted")
    result.update(estimated_t_s=grid[best], rmse_mps2=rmse, boundary_reached=boundary,
                  identifiable=identifiable, profile_interval_s=[near[0], near[-1]],
                  response_span_mps2=response, accepted=reason == "accepted", reason=reason)
    if result["accepted"]:
        result["used_t_s"] = grid[best]
    return result


def estimate_traits(rows: list[dict], parameters: dict[str, IDMParameters],
                    explicit_traits: dict | None = None, snapshot_headways: dict | None = None,
                    replace_snapshot_headway: bool = False,
                    config: FitConfig = FitConfig(),
                    cutoff_times: dict[str, float] | None = None) -> tuple[dict, dict]:
    """Export existing V2 traits schema, retaining explicit/snapshot precedence.

    Only actors in the target snapshot are exported. Other history actors are
    counted in diagnostics; explicit traits with unknown target IDs are errors.
    """
    explicit_traits, snapshot_headways = explicit_traits or {}, snapshot_headways or {}
    if set(explicit_traits) - set(parameters):
        raise ValueError("explicit traits reference unknown snapshot actors")
    grouped = defaultdict(list)
    for row in rows:
        logical = row.get("logical_actor_id")
        if not isinstance(logical, str) or not logical.strip():
            raise ValueError("history row requires logical_actor_id")
        grouped[logical].append(row)
    traits, diagnostics = {}, {}
    for logical, p in sorted(parameters.items()):
        explicit_speed = explicit_traits.get(logical, {}).get("desired_speed_mps")
        if explicit_speed is not None and finite(explicit_speed, "explicit v0", 0) != p.desired_speed_mps:
            raise ValueError("parameters must use the resolved explicit desired speed")
        actor_rows, future_count = [], 0
        cutoff = None if cutoff_times is None else finite(cutoff_times[logical], "snapshot cutoff")
        for row in grouped.get(logical, []):
            try:
                future = cutoff is not None and _number(row, "sim_time_s") - cutoff > 1e-9
            except (ValueError, TypeError):
                future = False  # Invalid timestamp must trigger fallback, not be silently removed.
            if future:
                future_count += 1
            else:
                actor_rows.append(row)
        diagnostic = estimate_headway(actor_rows, p, config)
        diagnostic.update(future_samples_excluded=future_count, cutoff_time_s=cutoff)
        values = dict(explicit_traits.get(logical, {}))
        protected = ("explicit_trait" if "desired_time_headway_s" in values else
                     "snapshot_headway" if snapshot_headways.get(logical) is not None
                     and not replace_snapshot_headway else None)
        if protected:
            used = values.get("desired_time_headway_s", snapshot_headways.get(logical))
            diagnostic.update(adopted=False, adoption_reason=protected, used_t_s=float(used))
        else:
            diagnostic.update(adopted=diagnostic["accepted"], adoption_reason=diagnostic["reason"])
        values["desired_time_headway_s"] = diagnostic["used_t_s"]
        # Preserve the fixed v0 used in estimation for the subsequent prediction.
        values.setdefault("desired_speed_mps", p.desired_speed_mps)
        traits[logical], diagnostics[logical] = values, diagnostic
    return traits, dict(schema_version=1, actors=diagnostics,
                        ignored_history_ids=sorted(set(grouped) - set(parameters)))
