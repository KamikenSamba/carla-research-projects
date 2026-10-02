from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from ogm_project.cooperative_runner import run_coop_comm


if __name__ == "__main__":
    run_coop_comm()
