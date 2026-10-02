"""Create and validate the official Phase 1b.7 grounded baseline artifacts."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


LEGACY_CONFIG_SHA256 = "369d6350fc65e7a735d9aa4cd45ec4141cb10cf0d8381148d77fdc7a879e2b19"


def read(path): return json.loads(Path(path).read_text(encoding="utf-8-sig"))
def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sensor_summary(paired, sensor):
    item=paired["sensors"][sensor]; target=item["false_free"]["by_actor"]["priority_target"]
    return {
        "phase0": item["phase0"], "phase1a": item["phase1a"],
        "road_phase0": item["road_phase0"], "road_phase1a": item["road_phase1a"],
        "unknown_to_free": item["unknown_to_free"],
        "road_unknown_to_free": item["road_unknown_to_free"],
        "occupied_to_unknown": item["occupied_to_unknown"],
        "occupied_to_free": item["occupied_to_free"],
        "target_phase0_false_free": target["phase0_false_free_cells"],
        "target_phase1a_false_free": target["phase1a_false_free_cells"],
        "target_new_false_free": target["phase1a_false_free_cells"]-target["phase0_false_free_cells"],
    }


def render_grounding(root, metadata):
    import matplotlib.pyplot as plt
    roles=("ego","target");fig,axes=plt.subplots(1,2,figsize=(11,5))
    for ax,role in zip(axes,roles):
        item=metadata["actors"][role]
        for x,key,color,label in ((0,"observed_before","tab:red","floating"),(1,"observed_after","tab:green","grounded")):
            state=item[key];ax.plot([x,x],[state["obb_bottom_z"],state["obb_top_z"]],lw=14,color=color,alpha=.55,label=label)
            ax.scatter(x,state["transform"]["z"],marker="x",s=90,color=color)
        ax.axhline(item["road_surface_z"],color="black",label="road surface")
        ax.set_xticks([0,1],["legacy spawn","grounded"]);ax.set_ylabel("WORLD z [m]")
        ax.set_title(f"{role}: origin (x), OBB span");ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(root/"all_vehicle_grounding.png",dpi=160);plt.close(fig)


def render_ogm(root):
    import matplotlib.pyplot as plt
    target=np.load(root/"phase1b_capture"/"priority_target_gt.npy")
    fig,axes=plt.subplots(2,3,figsize=(14,9))
    for row,sensor in enumerate(("ego","rsu")):
        p0=np.load(root/f"{sensor}_phase0.npy");p1=np.load(root/f"{sensor}_phase1a.npy")
        prob0=1/(1+np.exp(-p0));prob1=1/(1+np.exp(-p1))
        transition=((prob0>.48)&(prob0<.60)&(prob1<=.48))
        for ax,data,title,cmap in zip(axes[row],(prob0,prob1,transition),
                (f"{sensor} Phase0 probability",f"{sensor} Phase1a probability",f"{sensor} Unknown→Free"),
                ("viridis","viridis","magma")):
            ax.imshow(data,origin="lower",cmap=cmap,vmin=0 if data.dtype==bool else None,vmax=1)
            ax.contour(target,levels=[.5],colors="red",linewidths=1.2);ax.set_title(title)
    fig.tight_layout();fig.savefig(root/"grounded_ogm_phase0_phase1a.png",dpi=160);plt.close(fig)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--floating",type=Path,required=True);p.add_argument("--grounded",type=Path,required=True)
    p.add_argument("--repro",type=Path,required=True);p.add_argument("--legacy-config",type=Path,required=True)
    p.add_argument("--grounded-config",type=Path,required=True);args=p.parse_args(argv)
    floating=args.floating.resolve();root=args.grounded.resolve()
    cfg_old=read(args.legacy_config);cfg_new=read(args.grounded_config)
    grounding=read(root/"grounding_metadata.json");repro=read(args.repro/"grounding_reproducibility.json")
    paired_old=read(floating/"phase1a_paired"/"phase1a_paired_comparison.json")
    paired_new=read(root/"phase1a_paired"/"phase1a_paired_comparison.json")
    integ_old=read(floating/"integration_summary.json");integ_new=read(root/"integration_summary.json")
    shadow=read(root/"ogm_engine_shadow"/"shadow_comparison.json")
    experiment=read(root/"experiment_metadata.json")
    risk_experiment=read(root/"risk"/"experiment_metadata.json")
    ogm_experiment=read(root/"ogm"/"priority_debug"/"experiment_metadata.json")
    risk_map_experiment=read(root/"risk"/"risk_map"/"metadata.json")["experiment"]
    priority_metadata=read(root/"priority"/"priority_metadata.json")
    old_s={s:sensor_summary(paired_old,s) for s in ("ego","rsu")}
    new_s={s:sensor_summary(paired_new,s) for s in ("ego","rsu")}
    comparison={
        "schema_version":"1.0","floating_scenario":cfg_old["scenario_name"],
        "grounded_scenario":cfg_new["scenario_name"],
        "actor_transforms_and_clearance":grounding["actors"],
        "ego":{"floating":old_s["ego"],"grounded":new_s["ego"]},
        "rsu":{"floating":old_s["rsu"],"grounded":new_s["rsu"]},
        "risk":{"floating":{"event_count":integ_old["risk_event_count"],"positive_cells":integ_old["priority"]["risk_positive_cells"],"max":integ_old["risk_max"]},
                "grounded":{"event_count":integ_new["risk_event_count"],"positive_cells":integ_new["priority"]["risk_positive_cells"],"max":integ_new["risk_max"]}},
        "priority":{"floating":{"baseline_candidates":integ_old["priority"]["baseline_candidate_cells"],"positive_cells":integ_old["priority"]["priority_positive_cells"],"max":integ_old["priority"]["priority_max"]},
                    "grounded":{"baseline_candidates":integ_new["priority"]["baseline_candidate_cells"],"positive_cells":integ_new["priority"]["priority_positive_cells"],"max":integ_new["priority"]["priority_max"]}},
    }
    (root/"floating_vs_grounded_baseline.json").write_text(json.dumps(comparison,indent=2,ensure_ascii=False),encoding="utf-8")
    arrays=[np.load(path) for path in (root/"ego_phase0.npy",root/"ego_phase1a.npy",root/"rsu_phase0.npy",root/"rsu_phase1a.npy",root/"future_risk.npy",root/"priority.npy")]
    checks={
        "legacy_config_unchanged":sha(args.legacy_config)==LEGACY_CONFIG_SHA256,
        "scenario_separate_id_version":cfg_new["scenario_name"]!=cfg_old["scenario_name"] and cfg_new["experiment_id"]!=cfg_old["experiment_id"] and cfg_new["scenario_version"]=="2.0",
        "config_sha_distinct":sha(args.grounded_config)!=sha(args.legacy_config),
        "all_vehicle_roles_audited":set(grounding["actors"])=={"ego","target"} and grounding["vehicle_actor_count"]==2,
        "grounding_provenance_present":all(all(k in grounding["actors"][r] for k in ("original_spawn_transform","settled_transform","road_surface_z","clearance_before","clearance_after","settle_elapsed_frames","settle_elapsed_seconds")) for r in ("ego","target")),
        "clearance_reasonable":all(abs(grounding["actors"][r]["clearance_after"])<.05 for r in ("ego","target")),
        "legacy_actors_were_floating":all(grounding["actors"][r]["clearance_before"]>.5 for r in ("ego","target")),
        "settled_reproducible_twice":bool(repro["all_reproducible"]) and repro["source_scenario"]==cfg_new["scenario_name"],
        "common_legacy_all_pass":bool(shadow["all_pass"]),
        "paired_same_measurements":bool(paired_new["paired_same_measurements"]),
        "paired_same_decay":bool(paired_new["paired_same_decay"]),
        "ego_new_false_free_zero":new_s["ego"]["target_new_false_free"]==0,
        "rsu_new_false_free_111":new_s["rsu"]["target_new_false_free"]==111,
        "experiment_contract_exact_across_risk_ogm":experiment==risk_experiment==ogm_experiment==risk_map_experiment,
        "priority_experiment_alignment":priority_metadata["experiment_alignment"]["compatible"],
        "arrays_finite":all(np.isfinite(a).all() for a in arrays),
        "communication_payload_bit_exact":shadow["checks"]["communication_payload_bit_exact"],
        "fusion_bit_exact":shadow["checks"]["fusion_bit_exact"],
        "full_grid_q8_zlib_unchanged":priority_metadata["selective_transmission"]=="NOT IMPLEMENTED",
    }
    audit={"schema_version":"1.0","phase":"Phase 1b.7 grounded baseline audit",
           "legacy_config_sha256":sha(args.legacy_config),"grounded_config_sha256":sha(args.grounded_config),
           "checks":checks,"passed":all(checks.values()),"grounded_sensor_summary":new_s}
    (root/"grounded_baseline_audit.json").write_text(json.dumps(audit,indent=2,ensure_ascii=False),encoding="utf-8")
    render_grounding(root,grounding);render_ogm(root)
    print(json.dumps(audit,indent=2,ensure_ascii=False))
    if not audit["passed"]:raise SystemExit(1)


if __name__=="__main__":main()
