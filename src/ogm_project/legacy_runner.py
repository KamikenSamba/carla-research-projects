from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import uuid
from pathlib import Path

from .config import DEFAULT_SCENARIO
from .paths import (
    CONFIG_DIR,
    COOP_OUTPUT_ROOT,
    DATA_ROOT,
    EGO_OUTPUT_ROOT,
    LOG_ROOT,
    MASK_ROOT,
    OUTPUT_ROOT,
    PROJECT_ROOT,
    STATIC_MASK_PATH,
    carla_data_root,
    ensure_data_dirs,
    legacy_script,
    resolve_project_path,
)
from .scenario_loader import list_scenario_names
from .spectator_utils import SpectatorConfig


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--scenario-file", default=str(CONFIG_DIR / "scenarios.json"))
    parser.add_argument("--scenario", default=DEFAULT_SCENARIO)
    parser.add_argument("--list-scenarios", action="store_true")


def _print_scenarios(scenario_file: Path) -> None:
    for name in list_scenario_names(scenario_file):
        print(name)


def _patch_legacy_paths(script_name: str, namespace) -> None:
    if script_name == "Run_Coop_Comm_V5.py":
        namespace.OUT_DIR = str(COOP_OUTPUT_ROOT)
        namespace.STATIC_MASK_PATH = str(STATIC_MASK_PATH)
    elif script_name == "Run_Ego_OGM_V1.py":
        namespace.DATA_ROOT = str(DATA_ROOT)
        namespace.OUTPUT_ROOT = str(OUTPUT_ROOT)
        namespace.OUT_DIR = str(EGO_OUTPUT_ROOT)
        namespace.MASK_ROOT = str(MASK_ROOT)
        namespace.LOG_ROOT = str(LOG_ROOT)
    elif script_name == "build_static_mask_from_hdmap_V3.py":
        namespace.DATA_ROOT = str(DATA_ROOT)
        namespace.MASK_ROOT = str(MASK_ROOT)
        namespace.SAVE_MASK_PATH = str(STATIC_MASK_PATH)


def _load_legacy_module(script_name: str):
    script_path = legacy_script(script_name)
    module_name = f"_ogm_legacy_{script_path.stem}_{uuid.uuid4().hex}"
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load legacy script: {script_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _patch_spectator_defaults(namespace) -> None:
    namespace.ENABLE_SPECTATOR_FOLLOW = True
    namespace.SPECTATOR_VIEW_MODE = "topdown"
    namespace.SPECTATOR_HEIGHT_M = 35.0
    namespace.SPECTATOR_CHASE_DISTANCE_M = 20.0
    namespace.SPECTATOR_CHASE_HEIGHT_M = 18.0
    namespace.ENABLE_REALTIME_PREVIEW = False
    namespace.SPECTATOR_CONFIG = SpectatorConfig()


def _run_legacy(script_name: str, argv: list[str], scenario_file: Path | None = None) -> None:
    os.environ.setdefault("CARLA_DATA_ROOT", str(carla_data_root()))
    ensure_data_dirs()
    old_argv = sys.argv[:]
    old_cwd = Path.cwd()
    try:
        sys.argv = [str(legacy_script(script_name)), *argv]
        os.chdir(PROJECT_ROOT)
        module = _load_legacy_module(script_name)
        _patch_legacy_paths(script_name, module)
        _patch_spectator_defaults(module)
        module.main()
    finally:
        sys.argv = old_argv
        os.chdir(old_cwd)


def run_coop_comm(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Run the cooperative Ego/RSU OGM experiment through the V5 legacy implementation."
    )
    add_common_args(parser)
    args, passthrough = parser.parse_known_args(argv)
    scenario_file = resolve_project_path(args.scenario_file, CONFIG_DIR / "scenarios.json")
    if args.list_scenarios:
        _print_scenarios(scenario_file)
        return
    legacy_argv = ["--scenario-file", str(scenario_file)]
    if args.scenario:
        legacy_argv += ["--scenario", args.scenario]
    legacy_argv += passthrough
    from .cooperative_runner import run_coop_comm as run_coop_comm_compat

    run_coop_comm_compat([*legacy_argv, *passthrough])


def run_ego_ogm(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Run the Ego-only OGM experiment through the V1 legacy implementation."
    )
    add_common_args(parser)
    args, passthrough = parser.parse_known_args(argv)
    scenario_file = resolve_project_path(args.scenario_file, CONFIG_DIR / "scenarios.json")
    if args.list_scenarios:
        _print_scenarios(scenario_file)
        return
    legacy_argv = ["--scenario-file", str(scenario_file)]
    if args.scenario:
        legacy_argv += ["--scenario", args.scenario]
    legacy_argv += passthrough
    _run_legacy("Run_Ego_OGM_V1.py", legacy_argv, scenario_file)


def run_static_mask(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Build static_mask.npy under CARLA_DATA_ROOT/masks using the V3 HD-map implementation."
    )
    parser.parse_args(argv)
    _run_legacy("build_static_mask_from_hdmap_V3.py", [])


def run_show_spectator_pose(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Print the current CARLA spectator pose.")
    parser.parse_args(argv)
    _run_legacy("show_spectator_pose.py", [])
