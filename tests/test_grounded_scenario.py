from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"src"))
from ogm_project.vehicle_grounding import transforms_reproducible


def load(name):return json.loads((ROOT/"configs"/name).read_text(encoding="utf-8"))


def test_legacy_priority_crossing_v1_is_byte_unchanged():
    digest=hashlib.sha256((ROOT/"configs"/"priority_crossing_v1.json").read_bytes()).hexdigest()
    assert digest=="369d6350fc65e7a735d9aa4cd45ec4141cb10cf0d8381148d77fdc7a879e2b19"


def test_grounded_scenario_has_separate_identity_and_version():
    old,new=load("priority_crossing_v1.json"),load("priority_crossing_v2_grounded.json")
    assert new["scenario_name"]=="priority_crossing_v2_grounded"
    assert new["experiment_id"]!=old["experiment_id"]
    assert new["scenario_version"]=="2.0" and new["grounded"] is True


def test_only_scenario_initialization_values_changed():
    old,new=load("priority_crossing_v1.json"),load("priority_crossing_v2_grounded.json")
    for key in ("map_name","fixed_delta_seconds","prediction_horizon_s","warmup_seconds","random_seed","tm_seed","grid","rsu","lidar","risk"):
        assert new[key]==old[key]
    for role in ("ego","target"):
        for key in ("blueprint","x","y","yaw"):
            assert new[role][key]==old[role][key]


def test_all_configured_vehicle_roles_have_grounding_provenance():
    cfg=load("priority_crossing_v2_grounded.json")
    for role in ("ego","target"):
        provenance=cfg[role]["grounding_provenance"]
        assert set(provenance)=={"legacy_original_transform","calibrated_settled_transform"}
        assert provenance["legacy_original_transform"]["z"]==.6
        assert cfg[role]["z"]==provenance["calibrated_settled_transform"]["z"]


def test_grounding_thresholds_match_phase1b5_contract():
    grounding=load("priority_crossing_v2_grounded.json")["grounding"]
    assert grounding["velocity_threshold_mps"]==.01
    assert grounding["delta_z_threshold_m"]==.001
    assert grounding["consecutive_required_frames"]==20
    assert grounding["max_seconds"]==10.
    assert grounding["physics_during_measurement"] is False


def test_settled_transform_reproducibility_tolerances():
    base=dict(x=1.,y=2.,z=-.01,yaw=89.)
    passed,delta=transforms_reproducible(base,dict(x=1.0005,y=2.0005,z=-.0105,yaw=89.005))
    assert passed and all(np.isfinite(list(delta.values())))
    assert not transforms_reproducible(base,dict(x=1.,y=2.,z=.1,yaw=89.))[0]


def test_calibration_artifact_has_distinct_provenance_hash():
    grounding=load("priority_crossing_v2_grounded.json")["grounding"]
    assert len(grounding["calibration_artifact_sha256"])==64
    assert grounding["calibration_artifact_sha256"]!=grounding["source_legacy_config_sha256"]

