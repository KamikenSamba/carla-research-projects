"""Two-actor sequencing and entry rejection, without a server or claimed physics."""
import csv
import json
from types import SimpleNamespace as NS
import pytest

from idm_pair_exploration import PairHarness, pair_budget, single_admission
from idm_exploratory_following import ResearchConfig
from idm_brake_metrics import FIELDS
from idm_experiment_harness import EarlyStop
import Run_Exploratory_Following as entry


def test_pair_budget_uses_actual_bbox_and_measured_stop():
    report=pair_budget(2.809387,(2.4,2.4))
    assert report['admissible'] and report['initial_gap_m']==15.2
    assert report['required_route_m']<60 and report['accepted_physical_bounds'] is None
    assert not pair_budget(5.,(2.4,2.4))['admissible']
    with pytest.raises(ValueError):pair_budget(float('nan'),(2.4,2.4))
    with pytest.raises(ValueError):pair_budget(2.,(3.,2.4))


def test_safe_early_stop_cannot_admit_pair():
    result=dict(status='completed',supervised=True,worker_exit_code=0,collision_events=0,cleanup=dict(complete=True))
    stable=dict(continuous_ready_s=2.);stop=dict(continuous_stop_s=.5,physical_bounds=None)
    assert single_admission(result,stable,stop)
    for change in (dict(status='safe_early_stop'),dict(collision_events=1),dict(cleanup={}),dict(supervised=False)):
        assert not single_admission(result|change,stable,stop)
    assert not single_admission(result,stable,{})


def test_pair_execute_rejects_failed_single_before_native_or_rpc(tmp_path,monkeypatch):
    run=tmp_path/'single';run.mkdir()
    (run/'result.json').write_text(json.dumps(dict(status='safe_early_stop')))
    (run/'stability_result.json').write_text(json.dumps(dict(continuous_ready_s=2.)))
    from dataclasses import asdict
    c=ResearchConfig(trial='following');config=tmp_path/'config.json'
    config.write_text(json.dumps(dict(trial='following',scenario=asdict(c),single_run=str(run))))
    monkeypatch.setattr(entry,'native_identity',lambda *_:pytest.fail('native/RPC path reached'))
    with pytest.raises(ValueError,match='completed stable single'):
        entry.main(['--config',str(config),'--execute'])


def pair_fixture(tmp_path, gap=20.):
    h=PairHarness.__new__(PairHarness);h.c=ResearchConfig(trial='following');h.directory=tmp_path
    h.rows=[];h.pair_rows=[];h.pair_frames=0;h.last_frame=100;h.t=0.;h.collisions=[]
    h.slots={}
    for i,role in enumerate(('leader','follower')):
        box=NS(location=NS(x=0,y=0),extent=NS(x=2.,y=1.),rotation=NS(yaw=0.))
        h.slots[role]=dict(actor=NS(id=i+1,bounding_box=box),sensor=NS(is_listening=True),
            previous_velocity=None,accel_count=0,reverse_monitor=None,
            controller=NS(step=lambda *a: .05))
    events=[];commands={};positions=dict(leader=gap,follower=0.);speeds=dict(leader=0.,follower=0.);accel={}
    h.collision_gate=lambda:None
    def issue(phase,throttle,brake,steer):
        role='leader' if h.actor.id==1 else 'follower';events.append(('issue',role,h.last_frame))
        cmd=dict(based_on_frame=h.last_frame,sequence=len(events),phase=phase,
                 control=dict(throttle=throttle,brake=brake,steer=steer))
        commands[role]=cmd;return cmd
    h.issue=issue
    def tick():
        events.append(('tick',));h.last_frame+=1
        for role,cmd in commands.items():
            old=speeds[role]
            speeds[role]=(min(1.,old+.05) if cmd['phase']=='launch' else
                          max(0.,old-.05) if cmd['phase']=='braking' else 0.)
            accel[role]=(speeds[role]-old)/.05;positions[role]+=speeds[role]*.05
    h.tick=tick
    def observe(phase,cmd):
        role='leader' if h.actor.id==1 else 'follower';events.append(('observe',role,h.last_frame))
        r=dict.fromkeys(FIELDS,None)
        r.update(phase=phase,frame=h.last_frame,sim_time_s=h.t,world_time_s=h.t,actor_id=h.actor.id,
            x=positions[role],y=0.,z=0.,vz=0.,roll=0.,pitch=0.,speed_planar_mps=speeds[role],
            v_long_mps=speeds[role],a_long_mps2=accel[role],heading_error_deg=0.,signed_lateral_m=0.,
            route_progress_m=positions[role],gear=1,hand_brake=False,command_based_on_frame=cmd['based_on_frame'])
        r.update(cmd['control']);r.update({'command_'+k:v for k,v in cmd['control'].items()})
        h.rows.append(r);return r
    h.observe=observe
    return h,events


def test_both_commands_then_one_tick_then_two_same_frame_observations(tmp_path):
    h,events=pair_fixture(tmp_path)
    last=h.pair_step('settle',dict(leader=(0.,0.),follower=(0.,0.)),{})
    assert [x[0] for x in events]==['issue','issue','tick','observe','observe']
    assert last['leader']['frame']==last['follower']['frame']==101
    assert len(h.pair_rows)==2 and h.pair_rows[-1]['gap_m']==16.
    assert h.pair_rows[-1]['leader_id']==1


def test_gap_failure_preserves_partial_pair_rows(tmp_path):
    h,events=pair_fixture(tmp_path,gap=10.)
    with pytest.raises(EarlyStop,match='gap_below'):
        h.pair_step('settle',dict(leader=(0.,0.),follower=(0.,0.)),{})
    assert sum(x[0]=='tick' for x in events)==1
    assert len((tmp_path/'pair_raw.jsonl').read_text().splitlines())==1


def test_full_pair_synthetic_capture_and_replay(tmp_path,monkeypatch):
    monkeypatch.setattr('idm_pair_exploration.time.sleep',lambda *_:None)
    h,events=pair_fixture(tmp_path)
    h.capture()
    completion=json.loads((tmp_path/'pair_completion.json').read_text())
    assert completion['following_capture_frames']==40
    assert completion['continuous_stop_s']==.5
    result=dict(collision_events=0,cleanup=dict(complete=True),route_audit=None,physical_bounds=None,
        motion_samples=len(h.rows),final_frame=h.last_frame)
    entry.verify_completed(tmp_path,result,h.c)
    assert len(h.rows)==2*sum(x[0]=='tick' for x in events)
    with (tmp_path/'pair_observations.csv').open() as f:rows=list(csv.DictReader(f))
    assert len({r['frame'] for r in rows})==completion['pair_frames']


@pytest.mark.parametrize('poisoned',[False,True])
def test_pair_cleanup_preserves_external_actors_and_restores_settings_or_suppresses_rpc(tmp_path,poisoned):
    events=[];actors=[]
    class Actor:
        def __init__(self,i):self.id=i;self.is_listening=True;actors.append(self)
        def stop(self):events.append(('stop',self.id))
        def destroy(self):events.append(('destroy',self.id));actors.remove(self);return True
    external=Actor(99);s1,s2,a1,a2=[Actor(i) for i in (1,2,3,4)]
    def call(fn,*a,**kw):
        if poisoned:raise RuntimeError('no RPC after poison')
        return fn(*a,**kw)
    gate=NS(poisoned=poisoned,call=call,check=lambda:(_ for _ in ()).throw(RuntimeError('poison')) if poisoned else None)
    h=PairHarness(None,None,ResearchConfig(trial='following'),tmp_path,gate,single_stop_m=2.)
    h.last_frame=1;h.original=NS(synchronous_mode=False);state=NS(settings=NS(synchronous_mode=True))
    h.world=NS(get_snapshot=lambda:NS(frame=1),get_actors=lambda:list(actors),get_settings=lambda:state.settings,
        apply_settings=lambda s:setattr(state,'settings',s))
    h.settings_touched=True;h.initial_ids={99};h.owned=dict(s1=s1,s2=s2,a1=a1,a2=a2)
    h.slots=dict(leader=dict(actor=a1,sensor=s1),follower=dict(actor=a2,sensor=s2))
    h.cleanup()
    assert external in actors
    if poisoned:
        assert not events and not h.result['cleanup']['complete']
    else:
        assert [e[1] for e in events if e[0]=='destroy']==[1,2,3,4]
        assert state.settings is h.original and h.result['cleanup']['complete']
