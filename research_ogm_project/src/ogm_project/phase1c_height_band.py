"""Offline-only Phase 1c-A height-band replay and Pareto analysis.

Nothing in this module is imported by the production OGM path.  Target and
road masks are used only after replay for evaluation, never for ray carving.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from .height_slab_free_space import trace_height_slab_ray
from .ogm_engine import OGMGridContract, OGMUpdateConfig, apply_decay, update_valid_returns
from .phase1a_paired import classify, transition_matrix
from .phase1b_geometry import CAUSES, aggregate_cell, ray_geometry
from .logodds import FREE_TH, OCC_TH


@dataclass(frozen=True)
class OfflineGridOps:
    origin_x: float; origin_y: float; x_min: float; y_min: float
    resolution: float; nx: int; ny: int

    def world_to_grid(self,xw,yw):
        return (int((xw-(self.origin_x+self.x_min))/self.resolution),
                int((yw-(self.origin_y+self.y_min))/self.resolution))

    def in_bounds(self,ix,iy): return 0<=ix<self.nx and 0<=iy<self.ny

    def bresenham(self,ix0,iy0,ix1,iy1):
        dx=abs(ix1-ix0);sx=1 if ix0<ix1 else -1
        dy=-abs(iy1-iy0);sy=1 if iy0<iy1 else -1
        err=dx+dy;x,y=ix0,iy0
        while True:
            yield x,y
            if x==ix1 and y==iy1:break
            e2=2*err
            if e2>=dy:err+=dy;x+=sx
            if e2<=dx:err+=dx;y+=sy


@dataclass(frozen=True)
class HeightBand:
    z_free_min: float
    z_free_max: float

    def __post_init__(self):
        if not np.isfinite([self.z_free_min,self.z_free_max]).all() or self.z_free_min>=self.z_free_max:
            raise ValueError("height band must be finite and increasing")

    @property
    def key(self): return f"{self.z_free_min:.3f}_{self.z_free_max:.3f}"


@dataclass
class ReplayResult:
    band: HeightBand
    logodds: np.ndarray
    extra_count: np.ndarray
    metrics: dict


def benefit_retention(value, baseline):
    if baseline <= 0: raise ValueError("baseline benefit must be positive")
    return float(value/baseline)


def false_free_reduction(value, baseline):
    if baseline <= 0: raise ValueError("baseline false Free must be positive")
    return float(1-value/baseline)


def dominates(a,b):
    return (a["target_new_false_free"]<=b["target_new_false_free"] and
            a["road_unknown_to_free"]>=b["road_unknown_to_free"] and
            (a["target_new_false_free"]<b["target_new_false_free"] or
             a["road_unknown_to_free"]>b["road_unknown_to_free"]))


def pareto_frontier(rows):
    result=[row for row in rows if not any(dominates(other,row) for other in rows if other is not row)]
    return sorted(result,key=lambda r:(r["target_new_false_free"],-r["road_unknown_to_free"],r["z_free_min"],r["z_free_max"]))


def threshold_summary(rows):
    benefit={}
    for level in (1.,.95,.90,.80,.70,.50):
        valid=[r for r in rows if r["road_benefit_retention"]>=level]
        best=min(valid,key=lambda r:(r["target_new_false_free"],-r["road_unknown_to_free"])) if valid else None
        benefit[f"retention_gte_{int(level*100)}pct"]=None if best is None else compact(best)
    safety={}
    for limit in (0,5,10,25,50):
        valid=[r for r in rows if r["target_new_false_free"]<=limit]
        best=max(valid,key=lambda r:(r["road_unknown_to_free"],-r["target_new_false_free"])) if valid else None
        safety[f"false_free_lte_{limit}"]=None if best is None else compact(best)
    return {"benefit_thresholds":benefit,"false_free_thresholds":safety}


def compact(row):
    keys=("z_free_min","z_free_max","road_unknown_to_free","road_benefit_retention",
          "target_new_false_free","false_free_reduction")
    return {key:row[key] for key in keys}


def categorized_candidates(rows):
    def best(items):
        return None if not items else compact(max(items,key=lambda r:(r["road_unknown_to_free"],-r["target_new_false_free"],-r["z_free_min"])))
    zero=[r for r in rows if r["target_new_false_free"]==0]
    high=[r for r in rows if r["target_new_false_free"]<=10]
    balanced=[r for r in pareto_frontier(rows) if r["road_benefit_retention"]>=.80]
    high_benefit=[r for r in rows if r["road_benefit_retention"]>=.90]
    hb=None if not high_benefit else compact(min(high_benefit,key=lambda r:(r["target_new_false_free"],-r["road_unknown_to_free"])))
    return {"maximum_safety":best(zero),"high_safety":best(high),
            "balanced_candidates":[compact(r) for r in balanced],"high_benefit":hb}


class GroundedRSUReplay:
    def __init__(self, run_dir, *, world_to_grid=None, in_bounds=None, bresenham=None):
        self.root=Path(run_dir);self.capture=self.root/"phase1b_capture"
        self.manifest=json.loads((self.capture/"capture_manifest.json").read_text(encoding="utf-8"))
        self.cfg=self.manifest["config"];self.lidar=self.cfg["lidar"];self.constants=self.manifest["constants"]
        self.records=[m for m in self.manifest["measurements"] if m["sensor"]=="rsu"]
        g=self.cfg["grid"]
        ops=OfflineGridOps(g["center_x"],g["center_y"],-g["nx"]*g["resolution_m"]/2,
            -g["ny"]*g["resolution_m"]/2,g["resolution_m"],g["nx"],g["ny"])
        self.grid=OGMGridContract(world_to_grid or ops.world_to_grid,
            in_bounds or ops.in_bounds,bresenham or ops.bresenham)
        self.shape=(self.cfg["grid"]["ny"],self.cfg["grid"]["nx"])
        self.road=np.load(self.root/"road_mask_source.npy").astype(bool)
        self.target=np.load(self.capture/"priority_target_gt.npy").astype(bool)
        self.ego=np.load(self.capture/"priority_ego_gt.npy").astype(bool)
        self.phase0=np.load(self.root/"rsu_phase0.npy")
        self.official_phase1a=np.load(self.root/"rsu_phase1a.npy")
        self.labels0,_=classify(self.phase0)
        if self.road.shape!=self.shape or self.target.shape!=self.shape:return self._bad_shape()
        if self.manifest["experiment"]["scenario_name"]!="priority_crossing_v2_grounded":
            raise ValueError("official grounded measurements required")

    def _bad_shape(self): raise ValueError("saved mask/Grid shape mismatch")

    def _config(self,record):
        c=self.constants
        return OGMUpdateConfig(self.lidar["z_min_world"],self.lidar["z_max_world"],self.lidar["range_m"],
            c["free"],c["occupied"],record["free_scale"],c["minimum"],c["maximum"],FREE_TH,OCC_TH,
            record["decay_rate"],0.)

    def replay(self,band:HeightBand):
        arr=np.zeros(self.shape,np.float32);extra_count=np.zeros(self.shape,np.uint32)
        valid_free=valid_occ=extra_updates=0
        for record in self.records:
            data=np.load(self.capture/record["file"]);points=data["points_world"][:,:3]
            origin=np.asarray(record["used_sensor_matrix"],dtype=float)[:3,3]
            config=self._config(record)
            if record["decay_dt"] is not None:apply_decay(arr,record["decay_dt"],config)
            valid=(points[:,2]>config.z_min)&(points[:,2]<config.z_max)
            stats=update_valid_returns(arr,points[valid],origin,self.grid,config)
            valid_free+=stats.free_updates;valid_occ+=stats.occupied_updates
            amount=config.free_scale*config.free_logodds
            for hit in points[~valid]:
                updates=trace_height_slab_ray(origin,hit,band.z_free_min,band.z_free_max,
                    config.lidar_range,self.grid.world_to_grid,self.grid.in_bounds,self.grid.bresenham)
                for ix,iy in updates.free_cells:
                    arr[iy,ix]=np.clip(arr[iy,ix]+amount,config.logodds_min,config.logodds_max)
                    extra_count[iy,ix]+=1;extra_updates+=1
        labels,_=classify(arr);trans=transition_matrix(self.labels0,labels)
        road_trans=transition_matrix(self.labels0,labels,self.road)
        target_new=self.target&(self.labels0!=0)&(labels==0)
        all_vehicle=(self.target|self.ego)&(self.labels0!=0)&(labels==0)
        counts={name:int((labels==i).sum()) for i,name in enumerate(("known_free","unknown","occupied"))}
        road_counts={f"road_{name}":int((self.road&(labels==i)).sum()) for i,name in enumerate(("known_free","unknown","occupied"))}
        phase0_target_free=int((self.target&(self.labels0==0)).sum())
        candidate_target_free=int((self.target&(labels==0)).sum())
        delta=arr-self.phase0
        metrics={"z_free_min":band.z_free_min,"z_free_max":band.z_free_max,**counts,**road_counts,
            "unknown_to_free":trans["unknown"]["free"],"road_unknown_to_free":road_trans["unknown"]["free"],
            "occupied_to_unknown":trans["occupied"]["unknown"],"occupied_to_free":trans["occupied"]["free"],
            "phase0_free_to_unknown":trans["free"]["unknown"],
            "target_phase0_false_free":phase0_target_free,"target_false_free_total":candidate_target_free,
            "target_new_false_free":int(target_new.sum()),"all_vehicle_new_false_free":int(all_vehicle.sum()),
            "extra_free_updates":extra_updates,"extra_free_unique_cells":int((extra_count>0).sum()),
            "road_extra_free_unique_cells":int((self.road&(extra_count>0)).sum()),
            "valid_free_updates":valid_free,"occupied_endpoint_updates":valid_occ,
            "delta_positive_cells":int((delta>0).sum()),"delta_max":float(delta.max()),"delta_min":float(delta.min())}
        if not np.isfinite(arr).all():raise ValueError("candidate contains NaN or Inf")
        return ReplayResult(band,arr,extra_count,metrics)

    def classify_causes(self,result:ReplayResult):
        labels,_=classify(result.logodds);new=self.target&(self.labels0!=0)&(labels==0)
        by_cell={}
        for record in self.records:
            data=np.load(self.capture/record["file"]);points=data["points_world"][:,:3]
            origin=np.asarray(record["used_sensor_matrix"],dtype=float)[:3,3]
            config=self._config(record);valid=(points[:,2]>config.z_min)&(points[:,2]<config.z_max)
            target_box=record["actors"]["priority_target"]
            for raw_index in np.flatnonzero(~valid):
                hit=points[raw_index]
                updates=trace_height_slab_ray(origin,hit,result.band.z_free_min,result.band.z_free_max,
                    config.lidar_range,self.grid.world_to_grid,self.grid.in_bounds,self.grid.bresenham)
                relevant=[(ix,iy) for ix,iy in updates.free_cells if new[iy,ix]]
                for ix,iy in relevant:
                    g=self.manifest["grid_metadata"];x=g["x_min_m"]+(ix+.5)*g["resolution_m"];y=g["y_min_m"]+(iy+.5)*g["resolution_m"]
                    row=ray_geometry(origin,hit,(x,y),target_box);row["ray_id"]=f"rsu:{record['frame']}:{raw_index}"
                    row["free_logodds_delta"]=config.free_scale*config.free_logodds
                    row["endpoint_z_class"]="LOW" if float(hit[2]) <= float(config.z_min) else "HIGH"
                    by_cell.setdefault((ix,iy),[]).append(row)
        counts={cause:0 for cause in CAUSES}
        for rows in by_cell.values():counts[aggregate_cell(rows)["cause_class"]]+=1
        return {"new_false_free_cells":int(new.sum()),"classified_cells":len(by_cell),"cause":counts}

