from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from .config import DEFAULT_SCENARIO
from .paths import (
    CONFIG_DIR,
    DATA_ROOT,
    EGO_OUTPUT_ROOT,
    LOG_ROOT,
    MASK_ROOT,
    OUTPUT_ROOT,
    PROJECT_ROOT,
    carla_data_root,
    ensure_data_dirs,
    resolve_project_path,
)
from .scenario_loader import list_scenario_names


def _print_scenarios(scenario_file: Path) -> None:
    for name in list_scenario_names(scenario_file):
        print(name)


def _patch_ego_runtime(module) -> None:
    module.DATA_ROOT = str(DATA_ROOT)
    module.OUTPUT_ROOT = str(OUTPUT_ROOT)
    module.OUT_DIR = str(EGO_OUTPUT_ROOT)
    module.MASK_ROOT = str(MASK_ROOT)
    module.LOG_ROOT = str(LOG_ROOT)


def run_ego_ogm(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Run the Ego-only OGM experiment with Ego/World outputs."
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
            str(Path(__file__).with_name("ego_ogm_compat.py")),
            "--scenario-file",
            str(scenario_file),
        ]
        if args.scenario:
            sys.argv += ["--scenario", args.scenario]
        sys.argv += passthrough

        os.chdir(PROJECT_ROOT)
        from . import ego_ogm_compat

        _patch_ego_runtime(ego_ogm_compat)
        ego_ogm_compat.main()
    finally:
        sys.argv = old_argv
        os.chdir(old_cwd)


__all__ = ["run_ego_ogm"]
