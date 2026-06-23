from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import Any


def load_scenarios_json(path: str | Path) -> dict[str, dict[str, Any]]:
    with Path(path).open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Scenario file must contain an object: {path}")
    return data


def list_scenario_names(path: str | Path) -> list[str]:
    return list(load_scenarios_json(path).keys())


def load_scenario_actors(actors_file: str | Path) -> list[dict[str, Any]]:
    path = Path(actors_file)
    spec = importlib.util.spec_from_file_location("scenario_actors_module", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load scenario actor file: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actors = getattr(module, "SCENARIO_ACTORS", [])
    if not isinstance(actors, list):
        raise ValueError(f"SCENARIO_ACTORS must be a list: {path}")
    return actors
