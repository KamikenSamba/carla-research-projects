"""Read-only finite-segment geometry for Phase 1b; no occupancy policies."""
from __future__ import annotations

import numpy as np

from .actor_ground_truth import _convex_hull

CAUSES = ('3D_INTERSECTION', 'XY_ONLY_ABOVE', 'XY_ONLY_BELOW',
          'RASTERIZATION_OR_EDGE', 'UNRESOLVED')


def finite(value):
    result = np.asarray(value, dtype=np.float64)
    if not np.isfinite(result).all():
        raise ValueError('geometry contains NaN or Inf')
    return result


def segment_box_interval(start, end, box_matrix, extent):
    """Return finite segment entry/exit parameters in an oriented box, or None."""
    start, end, matrix, extent = map(finite, (start, end, box_matrix, extent))
    if start.shape != (3,) or end.shape != (3,) or matrix.shape != (4, 4) or extent.shape != (3,):
        raise ValueError('invalid OBB geometry shape')
    if np.any(extent <= 0):
        raise ValueError('OBB extents must be positive')
    inverse = np.linalg.inv(matrix)
    a = (inverse @ np.r_[start, 1.])[:3]
    b = (inverse @ np.r_[end, 1.])[:3]
    direction = b-a
    lo, hi = 0., 1.
    for axis in range(3):
        if abs(direction[axis]) < 1e-12:
            if abs(a[axis]) > extent[axis] + 1e-10:
                return None
        else:
            t = sorted(((-extent[axis]-a[axis])/direction[axis],
                        (extent[axis]-a[axis])/direction[axis]))
            lo, hi = max(lo, t[0]), min(hi, t[1])
            if lo > hi + 1e-10:
                return None
    return float(lo), float(hi)


def segment_intersects_actor_obb(start_xyz, end_xyz, actor_transform, bounding_box):
    """CARLA adapter; includes bounding-box offset and its own rotation."""
    import carla
    box_tf = carla.Transform(bounding_box.location, bounding_box.rotation)
    matrix = finite(actor_transform.get_matrix()) @ finite(box_tf.get_matrix())
    e = bounding_box.extent
    return segment_box_interval(start_xyz, end_xyz, matrix, (e.x, e.y, e.z)) is not None


def box_vertices(matrix, extent):
    import itertools
    local = np.array(list(itertools.product((-1., 1.), repeat=3))) * finite(extent)
    return (np.c_[local, np.ones(8)] @ finite(matrix).T)[:, :3]


def footprint(vertices):
    return np.asarray(_convex_hull([tuple(p[:2]) for p in finite(vertices)]))


def vertical_ray_plane_box_section(start, end, matrix, extent):
    """Exact OBB slice in the ray's vertical plane, as (XY distance, WORLD Z).

    A projection of the entire box onto this plane would wrongly appear to
    intersect some rays that actually pass its side or roof corner.
    """
    s,h=finite(start),finite(end)
    direction=h[:2]-s[:2];length=np.linalg.norm(direction)
    if length<1e-12:return np.empty((0,2))
    unit=direction/length;normal=np.array([-unit[1],unit[0]])
    vertices=box_vertices(matrix,extent)
    lateral=(vertices[:,:2]-s[:2])@normal
    intersections=[]
    for i in range(8):
        for bit in (1,2,4):
            j=i^bit
            if j<i:continue
            a,b=vertices[i],vertices[j];da,db=lateral[i],lateral[j]
            if abs(da)<1e-10:intersections.append(a)
            if abs(db)<1e-10:intersections.append(b)
            if da*db<0:intersections.append(a+da/(da-db)*(b-a))
    if not intersections:return np.empty((0,2))
    points=np.asarray(intersections)
    return np.asarray(_convex_hull(list(zip((points[:,:2]-s[:2])@unit,points[:,2]))))


def segment_polygon_interval(start, end, polygon):
    """Convex CCW XY polygon clipping, including boundary contact."""
    a, b, poly = finite(start)[:2], finite(end)[:2], finite(polygon)
    lo, hi = 0., 1.
    cross = lambda u, v: u[0]*v[1]-u[1]*v[0]
    for p, q in zip(poly, np.roll(poly, -1, axis=0)):
        edge = q-p
        value, slope = cross(edge, a-p), cross(edge, b-a)
        if abs(slope) < 1e-12:
            if value < -1e-10:
                return None
        elif slope > 0:
            lo = max(lo, -value/slope)
        else:
            hi = min(hi, -value/slope)
        if lo > hi + 1e-10:
            return None
    return float(lo), float(hi)


def endpoint_class(z, z_min, z_max):
    finite([z, z_min, z_max])
    if z <= z_min: return 'LOW'
    if z >= z_max: return 'HIGH'
    return 'VALID'


def new_target_false_free(target, labels0, labels1):
    return np.asarray(target, dtype=bool) & (np.asarray(labels0) != 0) & (np.asarray(labels1) == 0)


def ray_geometry(start, end, cell_xy, target):
    s, h = finite(start), finite(end)
    vertices = finite(target['vertices'])
    poly = footprint(vertices)
    obb = segment_box_interval(s, h, target['box_matrix'], target['extent'])
    xy = segment_polygon_interval(s, h, poly)
    d = h-s
    denom = float(d[:2] @ d[:2])
    t = float(np.clip((finite(cell_xy)-s[:2]) @ d[:2]/denom, 0, 1)) if denom else 0.
    z = float((s+t*d)[2])
    zmin, zmax = float(vertices[:, 2].min()), float(vertices[:, 2].max())
    height = 'BELOW_TARGET' if z < zmin else ('ABOVE_TARGET' if z > zmax else 'THROUGH_TARGET_HEIGHT')
    # Above/below classification uses the entire ray interval over the footprint,
    # not just a raster cell-center approximation.
    zxy = sorted([float((s+u*d)[2]) for u in xy]) if xy else None
    if obb is not None: cause = CAUSES[0]
    elif xy is None: cause = CAUSES[3]
    elif zxy[0] > zmax: cause = CAUSES[1]
    elif zxy[1] < zmin: cause = CAUSES[2]
    else: cause = CAUSES[4]
    distance = float(np.linalg.norm(d))
    return dict(cause_class=cause,z_at_cell=z,closest_xy_t=t,height_class=height,
                target_z_min=zmin,target_z_max=zmax,
                xy_footprint_intersection=xy is not None,obb_3d_intersection=obb is not None,
                d_target_entry=None if obb is None else obb[0]*distance,
                d_target_exit=None if obb is None else obb[1]*distance,
                d_endpoint=distance,
                endpoint_beyond_target_entry=bool(obb is not None and obb[0] < 1.-1e-8))


def aggregate_cell(rows):
    counts = {cause: sum(r['cause_class'] == cause for r in rows) for cause in CAUSES}
    # Exclusive primary label = most numerous evidence updates. Keep all counts
    # so a mixed cell is not mistaken for single-cause evidence.
    primary = max(CAUSES, key=lambda c: counts[c]) if rows else 'UNRESOLVED'
    return dict(cause_class=primary,cause_ray_counts=counts,
                extra_ray_count=len({r['ray_id'] for r in rows}),
                extra_free_update_count=len(rows),
                extra_logodds_total=sum(r['free_logodds_delta'] for r in rows),
                low_ray_count=len({r['ray_id'] for r in rows if r['endpoint_z_class']=='LOW'}),
                high_ray_count=len({r['ray_id'] for r in rows if r['endpoint_z_class']=='HIGH'}),
                obb_intersect_ray_count=counts[CAUSES[0]],
                xy_only_above_ray_count=counts[CAUSES[1]],
                xy_only_below_ray_count=counts[CAUSES[2]])


def distribution(values):
    a = finite(values)
    if not a.size: return dict(count=0,min=None,median=None,p95=None,max=None)
    return dict(count=int(a.size),min=float(a.min()),median=float(np.median(a)),
                p95=float(np.percentile(a,95)),max=float(a.max()))
