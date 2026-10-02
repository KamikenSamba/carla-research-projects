from __future__ import annotations

import numpy as np
import pytest
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))

from ogm_project.phase1b_geometry import (aggregate_cell, box_vertices, endpoint_class,
    footprint, new_target_false_free, ray_geometry, segment_box_interval, segment_polygon_interval)


def target(matrix=None, extent=(1.,1.,1.)):
    matrix=np.eye(4) if matrix is None else matrix
    return dict(box_matrix=matrix,extent=extent,vertices=box_vertices(matrix,extent))


@pytest.mark.parametrize('height,xy,obb,cause',[
    (0.,True,True,'3D_INTERSECTION'),
    (2.,True,False,'XY_ONLY_ABOVE'),
    (-2.,True,False,'XY_ONLY_BELOW')])
def test_finite_ray_height(height,xy,obb,cause):
    result=ray_geometry((-3,0,height),(3,0,height),(0,0),target())
    assert result['xy_footprint_intersection']==xy
    assert result['obb_3d_intersection']==obb
    assert result['cause_class']==cause
    assert result['z_at_cell']==height


def test_outside_and_segment_ends_before_box():
    box=target()
    result=ray_geometry((-3,3,0),(3,3,0),(0,0),box)
    assert not result['xy_footprint_intersection'] and not result['obb_3d_intersection']
    assert result['cause_class']=='RASTERIZATION_OR_EDGE'
    assert segment_box_interval((-3,0,0),(-2,0,0),box['box_matrix'],box['extent']) is None


def test_rotated_offset_box_with_pitch_roll():
    # Independent rigid rotation (yaw and pitch), plus bbox-local translation.
    yaw=np.deg2rad(37);pitch=np.deg2rad(23);roll=np.deg2rad(11)
    rz=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    ry=np.array([[np.cos(pitch),0,np.sin(pitch)],[0,1,0],[-np.sin(pitch),0,np.cos(pitch)]])
    rx=np.array([[1,0,0],[0,np.cos(roll),-np.sin(roll)],[0,np.sin(roll),np.cos(roll)]])
    actor=np.eye(4);actor[:3,:3]=rz@ry@rx;actor[:3,3]=[10,20,3]
    local=np.eye(4);local[:3,3]=[2,.4,.8]
    matrix=actor@local
    a=(matrix@[-3,0,0,1])[:3];b=(matrix@[3,0,0,1])[:3]
    assert np.allclose(segment_box_interval(a,b,matrix,(1,1,1)),(1/3,2/3))
    a=(matrix@[-3,0,2,1])[:3];b=(matrix@[3,0,2,1])[:3]
    assert segment_box_interval(a,b,matrix,(1,1,1)) is None


@pytest.mark.parametrize('z,expected',[(.1,'LOW'),(-1,'LOW'),(2,'HIGH'),(3,'HIGH'),(1,'VALID')])
def test_endpoint_boundary(z,expected):
    assert endpoint_class(z,.1,2)==expected


def test_new_false_free_definition():
    assert new_target_false_free([True,True,True,False],[0,1,2,1],[0,0,0,0]).tolist()==[False,True,True,False]


def test_aggregation_counts_and_mixed_primary():
    rows=[dict(ray_id=str(i),cause_class=c,endpoint_z_class='LOW',free_logodds_delta=-.2)
          for i,c in enumerate(['XY_ONLY_BELOW','XY_ONLY_BELOW','3D_INTERSECTION'])]
    result=aggregate_cell(rows)
    assert result['cause_class']=='XY_ONLY_BELOW'
    assert result['extra_ray_count']==result['extra_free_update_count']==3
    assert result['extra_logodds_total']==pytest.approx(-.6)
    assert result['obb_intersect_ray_count']==1
    assert aggregate_cell([])['cause_class']=='UNRESOLVED'


@pytest.mark.parametrize('bad',[float('nan'),float('inf')])
def test_nonfinite_rejected(bad):
    with pytest.raises(ValueError):segment_box_interval((bad,0,0),(3,0,0),np.eye(4),(1,1,1))


def test_point_polygon_boundary():
    poly=footprint(target()['vertices'])
    assert segment_polygon_interval((1,0),(1,0),poly) is not None
    assert segment_polygon_interval((1.1,0),(1.1,0),poly) is None


def test_true_vertical_section_not_projected_envelope():
    from ogm_project.phase1b_geometry import vertical_ray_plane_box_section
    section=vertical_ray_plane_box_section((-3,0,2),(3,0,2),np.eye(4),(1,1,1))
    assert set(map(tuple,np.round(section,6)))=={(2.,-1.),(2.,1.),(4.,-1.),(4.,1.)}
    assert len(vertical_ray_plane_box_section((-3,2,0),(3,2,0),np.eye(4),(1,1,1)))==0


def test_carla_box_adapter_matches_world_vertices_with_box_rotation():
    carla=pytest.importorskip('carla')
    from ogm_project.phase1b_geometry import segment_intersects_actor_obb
    tf=carla.Transform(carla.Location(x=7,y=-3,z=2),carla.Rotation(pitch=13,yaw=37,roll=9))
    box=carla.BoundingBox(carla.Location(x=.4,y=.7,z=.8),carla.Vector3D(x=2,y=1,z=.6))
    box.rotation=carla.Rotation(pitch=5,yaw=11,roll=3)
    matrix=np.array(tf.get_matrix())@np.array(carla.Transform(box.location,box.rotation).get_matrix())
    expected=box_vertices(matrix,(2,1,.6))
    actual=np.array([[v.x,v.y,v.z] for v in box.get_world_vertices(tf)])
    assert max(min(np.linalg.norm(v-actual,axis=1)) for v in expected)<2e-5
    start=(matrix@[-4,0,0,1])[:3];end=(matrix@[4,0,0,1])[:3]
    assert segment_intersects_actor_obb(start,end,tf,box)
    end=(matrix@[-3,0,0,1])[:3]
    assert not segment_intersects_actor_obb(start,end,tf,box)


def test_diagnostics_on_off_preserves_both_ogms(tmp_path):
    from test_phase1a_paired import paired,legacy
    from ogm_project.height_slab_free_space import trace_height_slab_ray
    a=paired(tmp_path/'off');b=paired(tmp_path/'on')
    points=np.array([[5.2,1.2,0.],[4.2,1.2,1.],[4.2,1.2,1.],[5.2,2.2,4.]])
    mask=(points[:,2]>.1)&(points[:,2]<2.)
    original=points.copy()
    for sensor,scale in [('ego',1.),('rsu',.15)]:
        production=np.zeros((6,6),dtype=np.float32)
        for frame in range(3):
            legacy(points[mask],(.2,1.2),production,free_scale=scale)
            for obj in (a,b):obj.update(sensor,points,mask,(.2,1.2,3.),scale,legacy,frame=frame,timestamp=.1*frame)
            for hit in points[~mask]:
                trace_height_slab_ray((.2,1.2,3.),hit,.1,2.,100,b.world_to_grid,b.in_bounds,b.bresenham)
                ray_geometry((.2,1.2,3.),hit,(2,1),target())
        np.testing.assert_array_equal(a.phase0[sensor],b.phase0[sensor])
        np.testing.assert_array_equal(a.phase1a[sensor],b.phase1a[sensor])
        np.testing.assert_array_equal(production,b.phase0[sensor])
    np.testing.assert_array_equal(points,original)
