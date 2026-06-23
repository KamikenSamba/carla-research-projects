from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from .config import DEFAULT_SCENARIO
from .paths import (
    CONFIG_DIR,
    COOP_OUTPUT_ROOT,
    PROJECT_ROOT,
    STATIC_MASK_PATH,
    carla_data_root,
    ensure_data_dirs,
    resolve_project_path,
)
from .scenario_loader import list_scenario_names
from .spectator_utils import SpectatorConfig


def _print_scenarios(scenario_file: Path) -> None:
    for name in list_scenario_names(scenario_file):
        print(name)


def _patch_runtime(module) -> None:
    module.OUT_DIR = str(COOP_OUTPUT_ROOT)
    module.STATIC_MASK_PATH = str(STATIC_MASK_PATH)
    module.ENABLE_SPECTATOR_FOLLOW = True
    module.SPECTATOR_VIEW_MODE = "topdown"
    module.SPECTATOR_HEIGHT_M = 35.0
    module.SPECTATOR_CHASE_DISTANCE_M = 20.0
    module.SPECTATOR_CHASE_HEIGHT_M = 18.0
    module.ENABLE_REALTIME_PREVIEW = False
    module.SPECTATOR_CONFIG = SpectatorConfig()


def run_coop_comm(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Run the cooperative Ego/RSU OGM experiment with spectator follow."
    )
    parser.add_argument("--scenario-file", default=str(CONFIG_DIR / "scenarios.json"))
    parser.add_argument("--scenario", default=DEFAULT_SCENARIO)
    parser.add_argument("--list-scenarios", action="store_true")
    args, passthrough = parser.parse_known_args(argv)

    scenario_file = resolve_project_path(args.scenario_file, CONFIG_DIR / "scenarios.json")
    if args.list_scenarios:
        _print_scenarios(scenario_file)
        return

    os.environ.setdefault("CARLA_DATA_ROOT", str(carla_data_root()))
    ensure_data_dirs()

    old_argv = sys.argv[:]
    old_cwd = Path.cwd()
    try:
        sys.argv = [
            str(Path(__file__).with_name("coop_comm_compat.py")),
            "--scenario-file",
            str(scenario_file),
        ]
        if args.scenario:
            sys.argv += ["--scenario", args.scenario]
        sys.argv += passthrough

        os.chdir(PROJECT_ROOT)
        from . import coop_comm_compat

        _patch_runtime(coop_comm_compat)
        coop_comm_compat.main()
    finally:
        sys.argv = old_argv
        os.chdir(old_cwd)


__all__ = ["run_coop_comm"]
