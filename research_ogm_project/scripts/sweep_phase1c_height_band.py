"""Offline Phase 1c-A RSU extra-Free height-band sweep."""
from __future__ import annotations

import argparse
import csv
import json
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

PROJECT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(PROJECT/"src"))
from ogm_project.phase1c_height_band import (GroundedRSUReplay,HeightBand,
    benefit_retention,categorized_candidates,compact,false_free_reduction,
    pareto_frontier,threshold_summary)

BASELINE_BENEFIT=4091
BASELINE_FALSE_FREE=111


def evaluate(task):
    root,zmin,zmax,stage=task
    replay=GroundedRSUReplay(root);result=replay.replay(HeightBand(zmin,zmax));row=result.metrics
    row["road_benefit_retention"]=benefit_retention(row["road_unknown_to_free"],BASELINE_BENEFIT)
    row["false_free_reduction"]=false_free_reduction(row["target_new_false_free"],BASELINE_FALSE_FREE)
    row["sweep_stage"]=stage
    return row


def decimal_values(start,stop,step):
    n=round((stop-start)/step)
    return [round(start+i*step,6) for i in range(n+1)]


def coarse_bands():
    return [(lo,hi) for lo in decimal_values(.1,1.5,.1)
            for hi in decimal_values(.5,2.,.1) if lo<hi]


def fine_bands(coarse):
    frontier=pareto_frontier(coarse);categories=categorized_candidates(coarse)
    anchors=[]
    for key in ("maximum_safety","high_safety","high_benefit"):
        if categories[key]:anchors.append(categories[key])
    balanced=categories["balanced_candidates"]
    if balanced:
        anchors.extend((balanced[0],balanced[-1]))
    if frontier:
        anchors.extend((compact(frontier[0]),compact(frontier[-1])))
    unique={(a["z_free_min"],a["z_free_max"]):a for a in anchors}
    bands=set()
    for lo,hi in unique:
        for a in decimal_values(max(.1,lo-.05),min(1.5,lo+.05),.025):
            for b in decimal_values(max(.5,hi-.05),min(2.,hi+.05),.025):
                if a<b:bands.add((a,b))
    coarse_keys={(r["z_free_min"],r["z_free_max"]) for r in coarse}
    return sorted(bands-coarse_keys),{
        "selection":"±0.05 m around coarse Pareto/category anchors",
        "step_m":.025,"anchors":[{"z_free_min":a,"z_free_max":b} for a,b in sorted(unique)]}


def write_csv(path,rows):
    fields=list(rows[0])
    with path.open("w",newline="",encoding="utf-8-sig") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)


def render_heatmaps(out,rows):
    import matplotlib.pyplot as plt
    coarse=[r for r in rows if r["sweep_stage"]=="coarse"]
    xs=sorted({r["z_free_max"] for r in coarse});ys=sorted({r["z_free_min"] for r in coarse})
    for field,name,title in (("road_unknown_to_free","road_unknown_to_free_heatmap.png","Road Unknown → Free"),
            ("target_new_false_free","target_false_free_heatmap.png","Target new False Free"),
            ("road_benefit_retention","road_benefit_retention_heatmap.png","Road benefit retention"),
            ("false_free_reduction","false_free_reduction_heatmap.png","False Free reduction")):
        data=np.full((len(ys),len(xs)),np.nan)
        for r in coarse:data[ys.index(r["z_free_min"]),xs.index(r["z_free_max"])]=r[field]
        fig,ax=plt.subplots(figsize=(11,7));im=ax.imshow(data,origin="lower",aspect="auto",
            extent=[min(xs)-.05,max(xs)+.05,min(ys)-.05,max(ys)+.05],cmap="viridis")
        ax.set_xlabel("z_free_max [m]");ax.set_ylabel("z_free_min [m]");ax.set_title(title+" (coarse sweep)")
        fig.colorbar(im,ax=ax);fig.tight_layout();fig.savefig(out/name,dpi=160);plt.close(fig)


def render_pareto(out,rows,frontier,categories):
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(10,7));ax.scatter([r["target_new_false_free"] for r in rows],
        [r["road_unknown_to_free"] for r in rows],s=14,alpha=.35,label="candidates")
    ax.plot([r["target_new_false_free"] for r in frontier],[r["road_unknown_to_free"] for r in frontier],
        "o-",color="tab:red",label="Pareto frontier")
    ax.scatter([111],[4091],marker="*",s=180,color="black",label="current 0.10–2.00")
    labels={"A":categories["maximum_safety"],"B":categories["high_safety"],"D":categories["high_benefit"]}
    if categories["balanced_candidates"]:labels["C"]=categories["balanced_candidates"][0]
    for label,row in labels.items():
        if row:ax.annotate(label,(row["target_new_false_free"],row["road_unknown_to_free"]),xytext=(5,5),textcoords="offset points")
    ax.set_xlabel("Target new False Free [cells] (lower is better)");ax.set_ylabel("Road Unknown → Free [cells] (higher is better)")
    ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(out/"phase1c_pareto_plot.png",dpi=170);plt.close(fig)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run-dir",type=Path,required=True);p.add_argument("--output-dir",type=Path,required=True)
    p.add_argument("--workers",type=int,default=4);args=p.parse_args(argv)
    root=args.run_dir.resolve();out=args.output_dir.resolve()
    if out.exists() and any(out.iterdir()):raise ValueError(f"output directory must be empty: {out}")
    out.mkdir(parents=True,exist_ok=True);replay=GroundedRSUReplay(root)
    baseline=replay.replay(HeightBand(.1,2.));baseline_repeat=replay.replay(HeightBand(.1,2.))
    official_equal=bool(np.array_equal(baseline.logodds,replay.official_phase1a))
    deterministic=bool(np.array_equal(baseline.logodds,baseline_repeat.logodds) and
                       np.array_equal(baseline.extra_count,baseline_repeat.extra_count) and
                       baseline.metrics==baseline_repeat.metrics)
    gate=(official_equal and baseline.metrics["road_unknown_to_free"]==4091 and
          baseline.metrics["target_new_false_free"]==111 and
          baseline.metrics["occupied_to_unknown"]==0 and baseline.metrics["occupied_to_free"]==0)
    if not gate:raise RuntimeError(f"baseline reproduction failed: {baseline.metrics}")
    coarse=coarse_bands();tasks=[(str(root),a,b,"coarse") for a,b in coarse]
    with ProcessPoolExecutor(max_workers=max(1,args.workers)) as pool:coarse_rows=list(pool.map(evaluate,tasks))
    fine,fine_reason=fine_bands(coarse_rows)
    with ProcessPoolExecutor(max_workers=max(1,args.workers)) as pool:
        fine_rows=list(pool.map(evaluate,[(str(root),a,b,"fine") for a,b in fine]))
    combined={};
    for row in coarse_rows+fine_rows:combined[(row["z_free_min"],row["z_free_max"])]=row
    rows=sorted(combined.values(),key=lambda r:(r["z_free_min"],r["z_free_max"]))
    if len({r["valid_free_updates"] for r in rows})!=1 or len({r["occupied_endpoint_updates"] for r in rows})!=1:
        raise RuntimeError("valid-return invariant failed")
    frontier=pareto_frontier(rows);frontier_keys={(r["z_free_min"],r["z_free_max"]) for r in frontier}
    for row in rows:row["pareto"]=(row["z_free_min"],row["z_free_max"]) in frontier_keys
    categories=categorized_candidates(rows);thresholds=threshold_summary(rows)
    selected={}
    for name in ("maximum_safety","high_safety","high_benefit"):
        if categories[name]:selected[name]=(categories[name]["z_free_min"],categories[name]["z_free_max"])
    if categories["balanced_candidates"]:
        c=min(categories["balanced_candidates"],key=lambda r:(r["target_new_false_free"],-r["road_unknown_to_free"]))
        selected["balanced"]=c["z_free_min"],c["z_free_max"]
    minimum_false_free=min(rows,key=lambda r:(r["target_new_false_free"],-r["road_unknown_to_free"]))
    selected["minimum_false_free"]=(minimum_false_free["z_free_min"],minimum_false_free["z_free_max"])
    selected["current_baseline"]=(.1,2.)
    target_vertices=np.asarray(replay.records[0]["actors"]["priority_target"]["vertices"])
    causes={};saved={}
    for name,(lo,hi) in selected.items():
        result=replay.replay(HeightBand(lo,hi));repeat=replay.replay(HeightBand(lo,hi))
        if not np.array_equal(result.logodds,repeat.logodds) or result.metrics!=repeat.metrics:raise RuntimeError(f"nondeterministic candidate {name}")
        np.save(out/f"{name}_rsu_logodds.npy",result.logodds);np.save(out/f"{name}_delta_from_phase0.npy",result.logodds-replay.phase0)
        causes[name]=replay.classify_causes(result);saved[name]={"band":[lo,hi],"metrics":result.metrics,
            "target_obb_relation":{"bottom_z":float(target_vertices[:,2].min()),
                "top_z":float(target_vertices[:,2].max()),
                "vertical_intervals_overlap":bool(hi>float(target_vertices[:,2].min()) and lo<float(target_vertices[:,2].max())),
                "evaluation_only":True}}
    summary={"schema_version":"1.0","phase":"Phase 1c-A offline height-band sweep",
        "source_run":str(root),"source_experiment":replay.manifest["experiment"],
        "production_changed":False,"current_baseline":compact(next(r for r in rows if r["z_free_min"]==.1 and r["z_free_max"]==2.)),
        **categories,"threshold_summary":thresholds,"fine_selection":fine_reason,
        "target_obb":{"bottom_z":float(target_vertices[:,2].min()),"top_z":float(target_vertices[:,2].max()),
                      "usage":"offline evaluation context only; never used by carving"},
        "baseline_reproduction":{"official_array_bit_exact":official_equal,"deterministic_repeat":deterministic,
            "road_unknown_to_free":baseline.metrics["road_unknown_to_free"],"target_new_false_free":baseline.metrics["target_new_false_free"],
            "occupied_to_unknown":baseline.metrics["occupied_to_unknown"],"occupied_to_free":baseline.metrics["occupied_to_free"]},
        "candidate_count":{"coarse":len(coarse_rows),"fine":len(fine_rows),"unique_total":len(rows)},
        "valid_return_invariant":{"valid_free_updates":rows[0]["valid_free_updates"],"occupied_endpoint_updates":rows[0]["occupied_endpoint_updates"],"all_candidates_identical":True},
        "major_candidate_causes":causes,"saved_candidates":saved}
    write_csv(out/"phase1c_height_band_sweep.csv",rows);write_csv(out/"phase1c_pareto_frontier.csv",frontier)
    (out/"phase1c_height_band_sweep.json").write_text(json.dumps({"rows":rows},indent=2,ensure_ascii=False),encoding="utf-8")
    (out/"phase1c_candidate_summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False),encoding="utf-8")
    render_heatmaps(out,rows);render_pareto(out,rows,frontier,categories)
    print(json.dumps(summary,indent=2,ensure_ascii=False))


if __name__=="__main__":main()
