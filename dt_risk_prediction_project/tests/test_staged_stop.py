import pytest
from idm_staged_stop import StagedStop,stopped


def test_taper_is_bounded_and_does_not_mix_brake_with_cruise_throttle():
    for initial in (0.,.0005):assert StagedStop(initial).step(.4)['throttle']<=initial
    c=StagedStop(.075);previous=.075
    for _ in range(100):
        r=c.step(.45)
        assert r['brake']==0 and .001<=r['throttle']<=previous
        assert previous-r['throttle']<=.001000001
        previous=r['throttle']


def test_brake_only_starts_low_and_is_ramped_to_small_hold():
    c=StagedStop(.06);c.step(.4)
    values=[c.step(.09) for _ in range(10)]
    assert values[0]['brake']==pytest.approx(.005)
    assert all(r['throttle']==0 and 0<r['brake']<=.02 for r in values)
    assert values[-1]['stage']=='stop_hold' and values[-1]['brake']==.02


@pytest.mark.parametrize('speed',[float('nan'),float('inf'),-.01])
def test_unknown_or_invalid_speed_rejected(speed):
    with pytest.raises(ValueError):StagedStop(.07).step(speed)


def test_stop_uses_observed_low_motion_and_actual_hold_control():
    r=dict(speed_planar_mps=0,v_long_mps=0,vz=0,a_long_mps2=0,throttle=0,brake=.02,hand_brake=False)
    assert stopped(r)
    for key,value in [('speed_planar_mps',.05),('v_long_mps',-.06),('a_long_mps2',.6),
                      ('throttle',.01),('brake',.005),('hand_brake',True)]:
        assert not stopped(dict(r,**{key:value}))
