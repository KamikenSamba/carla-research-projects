from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .paths import CONFIG_DIR


@dataclass(frozen=True)
class RuntimeConfig:
    scenario_file: Path = CONFIG_DIR / "scenarios.json"
    scenario: str | None = None
    host: str = "127.0.0.1"
    port: int = 2000


DEFAULT_SCENARIO_FILE = CONFIG_DIR / "scenarios.json"
DEFAULT_SCENARIO = "scenario_A"
