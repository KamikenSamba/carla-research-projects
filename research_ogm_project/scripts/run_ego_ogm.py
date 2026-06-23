from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from ogm_project.ego_ogm_runner import run_ego_ogm


if __name__ == "__main__":
    run_ego_ogm()
