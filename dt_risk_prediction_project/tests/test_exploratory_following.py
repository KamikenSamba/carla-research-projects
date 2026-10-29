"""Bounded exploratory control; synthetic plant tests are not physics calibration."""
from dataclasses import asdict, replace
import json
import math
from types import SimpleNamespace as NS
import pytest

from idm_exploratory_following import ResearchConfig, ApproachPI, StabilityHarness, ready, following_gap_gate
from idm_low_speed import config_for
from idm_experiment_harness import EarlyStop
import Run_Exploratory_Following as entry


def test_fixed_protocol_preserves_every_safety_gate():
    old=asdict(config_for('trial2'));new=asdict(ResearchConfig())
    differences={k for k in old if old[k]!=new[k]}
    assert differences=={'mode','trial','drive_distance'}
    for change in ({'acceleration_peak':6},{'speed_limit':2},{'route_audit':{}},{'lane':1}):
        with pytest.raises(ValueError):replace(ResearchConfig(),**change)


def test_approach_pi_damps_alternating_feedback_without_filtering_gates():
    c=ResearchConfig();ctl=ApproachPI(c)
    for _ in range(70):ctl.step(.3)
    last=ctl.throttle;outputs=[]
    for i in range(100):
        value=ctl.step(.95 if i%2 else 1.03)
        assert 0<=value<=.1 and abs(value-last)<=.00100001
        last=value;outputs.append(value)
    assert max(abs(b-a) for a,b in zip(outputs,outputs[1:]))<.0003
    assert not ready(dict(speed_planar_mps=1.,a_long_mps2=.501,vz=0))


@pytest.mark.parametrize('v',[math.nan,math.inf,-1,1.3])
def test_pi_invalid_feedback_not_zero_filled(v):
    with pytest.raises(ValueError):ApproachPI(ResearchConfig()).step(v)


@pytest.mark.parametrize('gap,closing,reason',[(15,0,None),(7.99,0,'following_gap_below_8m'),
    (10,3,'following_ttc_below_5s'),(math.nan,0,'gap_unknown')])
def test_independent_gap_ttc_gate(gap,closing,reason):
    assert following_gap_gate(gap,closing)==reason


def test_offline_entry_never_accesses_native_or_carla(tmp_path,monkeypatch,capsys):
    c=ResearchConfig();p=tmp_path/'config.json';p.write_text(json.dumps(dict(trial='stability',scenario=asdict(c))))
    monkeypatch.setattr(entry,'native_identity',lambda *_:pytest.fail('native access offline'))
    assert entry.main(['--config',str(p)])==0
    assert json.loads(capsys.readouterr().out)['route_audit'] is None


@pytest.mark.parametrize('unstable',[False,True])
def test_capture_synthetic_ready_coast_and_stop_or_bounded_rejection(tmp_path,unstable):
    h=StabilityHarness.__new__(StabilityHarness);h.c=ResearchConfig();h.directory=tmp_path
    h.actor=NS(id=1);h.rows=[];h.last_frame=0;h.t=0;h.reverse_monitor=NS(road=(1,0,0))
    h.ledger=lambda *a,**kw:None
    speed=0.;x=0.
    def step(phase,throttle,brake,previous=None):
        nonlocal speed,x
        old=speed
        if phase=='launch':speed=1.
        elif phase=='braking':speed=max(0,speed-.07)
        else:speed=0.
        h.last_frame+=1;h.t+=.05;x+=speed*.05
        r=dict(phase=phase,frame=h.last_frame,sim_time_s=h.t,x=x,y=0,z=0,vz=0,pitch=0,roll=0,
            speed_planar_mps=speed,v_long_mps=speed,a_long_mps2=(1. if unstable and phase=='launch' else 0.),
            route_progress_m=x,throttle=throttle,brake=brake,hand_brake=False)
        h.rows.append(r);return r
    h.step=step
    if unstable:
        with pytest.raises(EarlyStop,match='stability_not_ready|stability_distance_limit'):h.capture()
        assert not (tmp_path/'separated_stop_result.json').exists()
    else:
        h.capture()
        assert json.loads((tmp_path/'stability_result.json').read_text())['continuous_ready_s']==2.
        stop=json.loads((tmp_path/'separated_stop_result.json').read_text())
        assert stop['zero_throttle_coast_frames']==4 and stop['continuous_stop_s']==.5
        assert stop['physical_bounds'] is None and stop['calibration_valid'] is False
