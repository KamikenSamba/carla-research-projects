"""V2 snapshot schema; V1's dataclass and CSV columns remain unchanged."""
from __future__ import annotations

from dataclasses import dataclass, asdict, fields
import csv
import json
from pathlib import Path

from dt_risk_common import ActorState
from idm_model import IDMParameters, finite


@dataclass
class IDMActorState(ActorState):
    desired_time_headway_s: float | None = None
    desired_speed_mps: float | None = None


def read_snapshot(path: Path) -> list[dict]:
    if path.suffix.lower() == ".json":
        payload = json.loads(path.read_text(encoding="utf-8-sig"))
        rows = payload.get("actors") if isinstance(payload, dict) else payload
    else:
        with path.open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
    if not isinstance(rows, list):
        raise ValueError("snapshot must contain an actors list")
    result, seen = [], set()
    optional = {"desired_time_headway_s", "desired_speed_mps"}
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("snapshot actor must be an object")
        data = {}
        for field in fields(IDMActorState):
            value = row.get(field.name)
            if field.name in optional:
                data[field.name] = None if value in (None, "") else finite(value, field.name, 0.0)
            elif value is None:
                raise ValueError(f"missing snapshot field: {field.name}")
            elif field.name in {"phase", "logical_actor_id", "role_name", "blueprint_id"}:
                data[field.name] = str(value)
            elif field.name == "autopilot_enabled":
                if str(value).lower() not in {"true", "false", "1", "0"}:
                    raise ValueError("invalid autopilot_enabled")
                data[field.name] = str(value).lower() in {"true", "1"}
            elif field.name in {"carla_frame", "source_actor_id"}:
                data[field.name] = int(value)
            else:
                data[field.name] = finite(value, field.name)
        logical = data["logical_actor_id"]
        if not logical or logical in seen:
            raise ValueError(f"empty or duplicate logical_actor_id: {logical}")
        seen.add(logical)
        finite(data["speed_mps"], "speed_mps", 0.0)
        result.append(asdict(IDMActorState(**data)))
    return result


def load_traits(path: Path | None) -> dict:
    """Mapping from logical actor ID to numeric trait overrides (SI units)."""
    traits = {} if path is None else json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(traits, dict):
        raise ValueError("traits config must be an object")
    for logical, values in traits.items():
        if not isinstance(values, dict) or set(values) - {"desired_time_headway_s", "desired_speed_mps"}:
            raise ValueError(f"invalid trait configuration for {logical}")
        for key, value in values.items():
            finite(value, key, 0.0)
            if key == "desired_speed_mps" and float(value) == 0:
                raise ValueError("desired_speed_mps must be positive")
    return traits


def resolve_parameters(row: dict, args, traits: dict) -> IDMParameters:
    override = traits.get(row["logical_actor_id"], {})
    headway = override.get("desired_time_headway_s", row.get("desired_time_headway_s"))
    desired = override.get("desired_speed_mps", row.get("desired_speed_mps"))
    if headway is None:
        headway = args.default_idm_headway
    if desired is None:
        desired = args.idm_desired_speed_mps
    if desired is None:
        # Snapshot speed is an observation, not an inferred free-flow speed.
        desired = max(float(row["speed_mps"]), args.idm_min_desired_speed_mps)
    return IDMParameters(float(desired), float(headway), args.idm_minimum_gap_m,
                         args.idm_max_accel_mps2, args.idm_comfortable_decel_mps2,
                         args.idm_acceleration_exponent, args.idm_max_decel_mps2)
