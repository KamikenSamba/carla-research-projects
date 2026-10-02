from __future__ import annotations

import os
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
LEGACY_DIR = PROJECT_ROOT / "legacy"
CONFIG_DIR = PROJECT_ROOT / "configs"
SCRIPT_DIR = PROJECT_ROOT / "scripts"
DEFAULT_DATA_ROOT = Path(r"D:\CARLA_DATA")


def carla_data_root() -> Path:
    """Return CARLA_DATA_ROOT, falling back to the default data root."""
    return Path(os.environ.get("CARLA_DATA_ROOT", str(DEFAULT_DATA_ROOT)))


DATA_ROOT = carla_data_root()
OUTPUT_ROOT = DATA_ROOT / "outputs"
COOP_OUTPUT_ROOT = OUTPUT_ROOT / "coop_comm"
EGO_OUTPUT_ROOT = OUTPUT_ROOT / "ego_ogm"
MASK_ROOT = DATA_ROOT / "masks"
STATIC_MASK_PATH = MASK_ROOT / "static_mask.npy"
LOG_ROOT = DATA_ROOT / "logs"
OUTPUT_DIR = OUTPUT_ROOT


def ensure_output_dir(path: Path | None = None) -> Path:
    target = path or OUTPUT_ROOT
    target.mkdir(parents=True, exist_ok=True)
    return target


def ensure_data_dirs() -> None:
    for path in (OUTPUT_ROOT, COOP_OUTPUT_ROOT, EGO_OUTPUT_ROOT, MASK_ROOT, LOG_ROOT):
        path.mkdir(parents=True, exist_ok=True)


def resolve_project_path(path_value: str | os.PathLike[str] | None, default: Path) -> Path:
    """Resolve a CLI path relative to the project root."""
    if path_value is None:
        return default.resolve()
    path = Path(path_value)
    if path.is_absolute():
        return path
    return (PROJECT_ROOT / path).resolve()


def legacy_script(name: str) -> Path:
    path = LEGACY_DIR / name
    if not path.exists():
        raise FileNotFoundError(f"Legacy script not found: {path}")
    return path
