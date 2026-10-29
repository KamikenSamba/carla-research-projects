import math
import pytest
from idm_reverse_motion import ReverseMotion


def row(frame,x,v,*,time=None,forward=(1,0,0)):
    return dict(frame=frame,world_time_s=.05*frame if time is None else time,
        x=x,y=0,z=0,vx=v,vy=0,vz=0,**{'forward_'+k:value for k,value in zip(('x','y','z'),forward)})


def test_one_endpoint_negative_with_forward_displacement_then_recovery():
    monitor=ReverseMotion((1,0,0));monitor.update(row(1,0,0))
    r=monitor.update(row(2,.0007,-.037))
    assert r['reason'] is None and r['signed_road_displacement_m']>0
    assert r['classification']=='transient_disagreement_under_observation'
    r=monitor.update(row(3,.003,.065));assert r['reason'] is None
    assert r['consecutive_negative_api_frames']==0


def test_true_reverse_position_detected_even_when_api_velocity_positive():
    m=ReverseMotion((1,0,0));m.update(row(1,0,0))
    assert m.update(row(2,-.0015,.03))['reason']=='reverse_signed_displacement'


def test_small_sustained_actual_reverse_accumulates():
    m=ReverseMotion((1,0,0));m.update(row(1,0,0))
    for i in range(2,7):assert m.update(row(i,-.00035*(i-1),-.007))['reason'] is None
    assert m.update(row(7,-.00035*6,-.007))['reason']=='reverse_cumulative_retreat'


def test_persistent_api_disagreement_cannot_pass_on_positive_position():
    m=ReverseMotion((1,0,0));m.update(row(1,0,0))
    assert m.update(row(2,.001,-.04))['reason'] is None
    assert m.update(row(3,.002,-.04))['reason'] is None
    assert m.update(row(4,.003,-.04))['reason']=='reverse_velocity_persistent_or_unexplained'


def test_strong_negative_api_aborts_without_waiting_for_position():
    assert ReverseMotion((1,0,0)).update(row(1,0,-.15))['reason']=='reverse_api_severe'


def test_northbound_signed_projection_not_absolute_distance():
    m=ReverseMotion((0,-1,0));a=row(1,0,0,forward=(0,-1,0));m.update(a)
    b=row(2,0,0,forward=(0,-1,0));b.update(y=.003,vy=-.02)
    r=m.update(b);assert r['signed_road_displacement_m']==-.003
    assert r['reason']=='reverse_signed_displacement'


@pytest.mark.parametrize('change',[dict(frame=4),dict(world_time_s=.2),dict(vx=float('nan')),dict(forward_x=0,forward_y=0)])
def test_missing_or_wrong_time_and_direction_fail_closed(change):
    m=ReverseMotion((1,0,0));m.update(row(1,0,0));r=row(2,.001,.02);r.update(change)
    with pytest.raises(ValueError):m.update(r)


def test_stationary_quantization_does_not_accumulate_absolute_distance():
    m=ReverseMotion((1,0,0))
    for f in range(1,100):assert m.update(row(f,.00003*(-1)**f,0))['reason'] is None
