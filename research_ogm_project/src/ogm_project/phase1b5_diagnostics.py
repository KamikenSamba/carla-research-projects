"""Ground-contact and collision-query diagnostics. No OGM update policy."""
from __future__ import annotations

from dataclasses import dataclass, asdict
import numpy as np


def clearance(bottom_z, road_surface_z):
    if road_surface_z is None:return None
    if not np.isfinite([bottom_z,road_surface_z]).all():raise ValueError('nonfinite clearance')
    return float(bottom_z-road_surface_z)


@dataclass
class SettleMonitor:
    velocity_threshold: float = .01
    delta_z_threshold: float = .001
    consecutive_required: int = 20
    minimum_frames: int = 20
    consecutive: int = 0
    frames: int = 0
    previous_z: float | None = None

    def update(self,z,vz):
        if not np.isfinite([z,vz]).all():raise ValueError('nonfinite settle state')
        stable=self.previous_z is not None and abs(vz)<self.velocity_threshold and abs(z-self.previous_z)<self.delta_z_threshold
        self.consecutive=self.consecutive+1 if stable else 0
        self.previous_z=float(z);self.frames+=1
        return self.frames>=self.minimum_frames and self.consecutive>=self.consecutive_required

    def metadata(self):return asdict(self)


def transform_record(transform):
    result={k:float(getattr(transform.location,k)) for k in ('x','y','z')}
    result.update({k:float(getattr(transform.rotation,k)) for k in ('pitch','yaw','roll')})
    if not np.isfinite(list(result.values())).all():raise ValueError('nonfinite transform')
    return result


def query_hits(world,start,end):
    available=hasattr(world,'cast_ray')
    if not available:return dict(available=False,hits=[],project_available=False,project_hit=None)
    import carla
    location=lambda xyz:carla.Location(x=float(xyz[0]),y=float(xyz[1]),z=float(xyz[2]))
    try:
        hits=[dict(location=[float(h.location.x),float(h.location.y),float(h.location.z)],label=str(h.label),
                   distance=float(np.linalg.norm(np.array([h.location.x,h.location.y,h.location.z])-start)))
              for h in world.cast_ray(location(start),location(end))]
        hits.sort(key=lambda h:h['distance'])
        length=float(np.linalg.norm(np.asarray(end)-start))
        project=None
        if hasattr(world,'project_point') and length>0:
            h=world.project_point(location(start),carla.Vector3D(*map(float,np.asarray(end)-start)),length)
            if h is not None:project=dict(location=[float(h.location.x),float(h.location.y),float(h.location.z)],label=str(h.label),
                distance=float(np.linalg.norm(np.array([h.location.x,h.location.y,h.location.z])-start)))
        return dict(available=True,hits=hits,project_available=hasattr(world,'project_point'),project_hit=project)
    except RuntimeError as exc:
        return dict(available=False,hits=[],project_available=False,project_hit=None,error=str(exc))


def infer_target_hit(hit,geometry,all_vehicles):
    """LabelledPoint has no actor ID. Preserve this explicitly as inference."""
    if hit is None:return False
    if hit['label'] not in ('Car','Truck','Bus','Motorcycle','Bicycle'):return False
    p=np.asarray(hit['location']);candidates=[]
    for box in all_vehicles:
        local=(np.linalg.inv(box['box_matrix'])@np.r_[p,1])[:3]
        if np.all(abs(local)<=np.array(box['extent'])+.02):candidates.append(box['actor_id'])
    if candidates==[geometry['actor_id']]:return True
    if not candidates or len(candidates)>1:return None
    return False


def classify_query(*,available,same_frame,first_hit_is_target,first_hit_distance,
                   target_exit_distance,endpoint_distance):
    """A query blocked before the box does not demonstrate box-only geometry."""
    if not available or not same_frame:return 'UNRESOLVED'
    finite=[v for v in (first_hit_distance,target_exit_distance,endpoint_distance) if v is not None]
    if not np.isfinite(finite).all():raise ValueError('nonfinite query distance')
    if first_hit_is_target is True:return 'OBB_AND_TARGET_GEOMETRY'
    if first_hit_is_target is None:return 'UNRESOLVED'
    if first_hit_distance is None or first_hit_distance>=target_exit_distance-.001:
        return 'OBB_ONLY'
    return 'UNRESOLVED'


def variant_sensor_summary(paired,provenance):
    return dict(new_false_free=provenance['new_target_false_free_cells'],
        xy_only_below=provenance['cause']['XY_ONLY_BELOW'],
        xy_only_above=provenance['cause']['XY_ONLY_ABOVE'],
        **{'3d_intersection':provenance['cause']['3D_INTERSECTION']},
        rasterization_edge=provenance['cause']['RASTERIZATION_OR_EDGE'],
        unresolved=provenance['cause']['UNRESOLVED'],
        road_unknown_to_free=paired['road_unknown_to_free'],
        occupied_to_unknown=paired['occupied_to_unknown'],occupied_to_free=paired['occupied_to_free'])
