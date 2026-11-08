"""Design T01--T36: mocked APIs only; no CARLA physics/admission claim."""
import copy
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest

from idm_abort_state import AbortState, AbortRejected, StopWindow, bounded_stop
from idm_carla_validation import V2FleetHarness, ValidationConfig
from idm_low_speed import ExploratoryHarness
from idm_experiment_harness import EarlyStop
from idm_harness_host import HealthGate, ServerUnavailable
from test_gt_observation import collect
from test_idm_harness import Gate


def control(brake=0.,throttle=0.):
    return dict(throttle=throttle,brake=brake,steer=0.,hand_brake=False,
                reverse=False,manual_gear_shift=False)


def row(frame=10,brake=0.,speed=0.):
    return dict(frame=frame,world_time_s=frame*.05,speed_planar_mps=speed,v_long_mps=speed,vz=0.,a_long_mps2=0.,
        gear=1,control_read_at_frame=frame,command_based_on_frame=frame-1,
        road_id=8,section_id=0,lane_id=-1,roll=0.,pitch=0.,heading_error_deg=0.,
        route_lateral_m=0.,front_clearance_m=40.,
        **control(brake),command_throttle=0.,command_brake=brake,command_steer=0.)


def callback(h):
    owner=next(iter(h.slots.values()))['actor'].id
    h.record_collision(NS(frame=h.last_frame,timestamp=h.last_time,normal_impulse=NS(x=1.,y=0.,z=0.),
        other_actor=NS(id=999,type_id='vehicle.synthetic')),owner)


def no_drive_after_latch(h):
    events=h.abort_state().snapshot()['events']
    first=next(e['sequence'] for e in events if e['kind']=='ABORT_REQUESTED')
    assert not [e for e in events if e['kind']=='CONTROL_ADMITTED' and not e['abort_control'] and e['sequence']>first]


def fixture(tmp_path,pair=True):
    from idm_carla_validation import V2SingleHarness
    klass=V2FleetHarness if pair else V2SingleHarness
    args=dict(single_stop_m=.5) if pair else {}
    h=klass(None,NS(VehicleControl=lambda **kw:NS(**kw)),ValidationConfig(trial='fleet' if pair else 'single'),tmp_path,Gate(),**args)
    calls=[];actor=NS(id=1,apply_control=lambda c:calls.append(dict(vars(c))))
    h.actor=actor;h.owned={'vehicle':actor};h.last_frame=9
    h.issue('launch',.02,0.,0.)
    h.last_frame=10
    return h,calls


def acknowledged(actors=(1,2),frame=100):
    s=AbortState(pair=True)
    for a in actors:s.register(a)
    s.request('synthetic')
    for a in actors:
        c=control(.02);p=s.reserve_stop(a,c);t=s.admit(a,frame,c,p)
        s.rpc_started(t);s.finish(t)
    return s


def stop_rows(frame,speed=0.):
    rows={a:row(frame,.02,speed) for a in (1,2)}
    for r in rows.values():r['command_based_on_frame']=100
    return rows


def test_T01_callback_before_first_drive(tmp_path,monkeypatch):
    def fault(h):
        old=h.initialize_fleet
        def init(*a):old(*a);callback(h)
        h.initialize_fleet=init
    h,r,*_=collect(tmp_path/'run',monkeypatch,False,fault=fault)
    no_drive_after_latch(h)
    assert r['reason']=='collision_event' and h.pair_frames==40


def test_T02_callback_between_actors(tmp_path,monkeypatch):
    def fault(h):
        old=h._apply_control_rpc;fired=[]
        def apply(c,v,p=None):
            result=old(c,v,p)
            if v['throttle']>0 and hasattr(h,'fleet') and not fired:fired.append(True);callback(h)
            return result
        h._apply_control_rpc=apply
    h,r,*_=collect(tmp_path/'run',monkeypatch,False,fault=fault)
    no_drive_after_latch(h)
    drives=[e for e in r['abort']['events'] if e['kind']=='CONTROL_ADMITTED' and not e['abort_control'] and e['frame']==140]
    assert len(drives)==1 and h.pair_frames==40


def test_T03_callback_during_ledger(tmp_path):
    h,calls=fixture(tmp_path);old=h.ledger
    def ledger(op,**kw):
        n=old(op,**kw)
        if op=='manual_control' and kw.get('outcome')=='requested':h.abort_state().request('collision_event')
        return n
    h.ledger=ledger
    with pytest.raises(EarlyStop):h.issue('launch',.03,0,0)
    assert len(calls)==1;no_drive_after_latch(h)


def test_T04_callback_during_health_check(tmp_path):
    h,calls=fixture(tmp_path)
    h.gate.check=lambda:h.abort_state().request('collision_event')
    with pytest.raises(EarlyStop):h.issue('launch',.03,0,0)
    assert len(calls)==1 and not h.gate.poisoned


def test_T05_admitted_rpc_cannot_be_cancelled_and_single_inflight(tmp_path):
    s=AbortState();s.register(1);ticket=s.admit(1,10,control())
    s.request('collision_event',received_ns=1)
    s.rpc_started(ticket)
    errors=[]
    def other():
        try:s.admit(2,10,control())
        except AbortRejected as e:errors.append(str(e))
    thread=threading.Thread(target=other);thread.start();thread.join(2)
    assert not thread.is_alive() and errors
    s.finish(ticket)
    with pytest.raises(AbortRejected):s.admit(1,10,control())
    e=s.snapshot()['events'];admit=next(x for x in e if x['kind']=='CONTROL_ADMITTED')
    abort=next(x for x in e if x['kind']=='ABORT_REQUESTED');start=next(x for x in e if x['kind']=='RPC_CALL_STARTED')
    assert admit['sequence']<abort['sequence']<start['sequence']
    h,calls=fixture(tmp_path);entered=threading.Event();release=threading.Event()
    active=[];maximum=[];rejections=[]
    def blocking_rpc(c):
        assert h.abort_state().lock.acquire(blocking=False)
        h.abort_state().lock.release()
        active.append(1);maximum.append(len(active));entered.set()
        assert release.wait(2)
        active.pop();calls.append(dict(vars(c)))
    h.actor.apply_control=blocking_rpc
    def contender():
        assert entered.wait(2)
        try:h.issue('launch',.03,0,0)
        except EarlyStop as error:rejections.append(str(error))
        h.abort_state().request('collision_event');release.set()
    thread=threading.Thread(target=contender);thread.start()
    h.issue('launch',.03,0,0);thread.join(2)
    assert not thread.is_alive() and rejections and max(maximum)==1 and len(calls)==2
    no_drive_after_latch(h)


def test_T06_callback_io_failure_latches_first(tmp_path,monkeypatch):
    h,_=fixture(tmp_path);h.collisions=[]
    old=Path.open
    def broken(p,*a,**kw):
        if p.name=='collision_events.jsonl':
            assert h.abort_state().lock.acquire(blocking=False)
            h.abort_state().lock.release()
            assert h.collision_lock.acquire(blocking=False)
            h.collision_lock.release()
            raise OSError('synthetic disk error')
        return old(p,*a,**kw)
    monkeypatch.setattr(Path,'open',broken)
    with pytest.raises(OSError):ExploratoryHarness.collision(h,NS(frame=10,normal_impulse=NS(x=1,y=0,z=0),other_actor=NS(id=2,type_id='vehicle.fake')))
    assert h.abort_state().requested
    with pytest.raises(EarlyStop):h.issue('launch',.03,0,0)


def test_T07_malformed_callback_latches_before_attributes(tmp_path):
    h,_=fixture(tmp_path)
    with pytest.raises(AttributeError):h.record_collision(object(),1)
    assert h.abort_state().reason=='collision_event'


def test_T08_multiple_same_frame_callbacks(tmp_path):
    h,_=fixture(tmp_path)
    for owner in (1,2,1):h.record_collision(NS(frame=10,normal_impulse=NS(x=1,y=0,z=0),other_actor=NS(id=3,type_id='vehicle.fake')),owner)
    s=h.abort_state().snapshot()
    assert s['reason']=='collision_event' and len(h.collisions)==3
    assert len([e for e in s['events'] if e['kind']=='ABORT_REQUESTED' and e['actor_id'] is None])==1


def test_T09_aborted_all_normal_phases_rejected(tmp_path):
    h,calls=fixture(tmp_path);h.abort_state().request('collision_event')
    for phase in ('launch','settle','braking','abort'):
        with pytest.raises(EarlyStop):h.issue(phase,.01,0,0)
    assert len(calls)==1


def test_T10_normal_on_off_control_pid_actuator_tick_equivalence(tmp_path,monkeypatch):
    off=collect(tmp_path/'off',monkeypatch,False);on=collect(tmp_path/'on',monkeypatch,True)
    assert off[1]['status']==on[1]['status']=='completed'
    assert off[2:4]==on[2:4]
    for k in off[0].actuators:
        assert vars(off[0].actuators[k])==vars(on[0].actuators[k])
        for name in ('speed_pid','steering_pid'):
            assert vars(getattr(off[0].fleet.controllers[k],name))==vars(getattr(on[0].fleet.controllers[k],name))


def test_T11_pair_low_speed_abort_requests_and_actor_facts(tmp_path,monkeypatch):
    def fault(h):
        old=h.initialize_fleet
        def init(*a):old(*a);callback(h)
        h.initialize_fleet=init
    h,r,calls,commands,*_=collect(tmp_path/'run',monkeypatch,False,fault=fault)
    aborts=[c for c in commands if c[2]['brake']>0]
    assert len(aborts)==2 and all(c[2]['brake']==pytest.approx(.005) for c in aborts)
    for a in r['abort']['actors'].values():
        assert {'ABORT_REQUESTED','DRIVE_COMMANDS_BLOCKED','STOP_COMMAND_ATTEMPTED','STOP_COMMAND_ACKNOWLEDGED',
                'STOP_UNVERIFIED','CLEANUP_ATTEMPTED','CLEANUP_COMPLETED'}<=set(a['facts'])
        assert 'STOP_OBSERVED' not in a['facts']


def test_T12_bounded_cap_and_slew():
    r=row(brake=.019);c,reason=bounded_stop(r,dict(frame=9,control=control(.019)),10)
    assert not reason and c['brake']==.02 and c['brake']-.019<=.005


def test_T13_high_speed_skips_all_unqualified_control():
    assert bounded_stop(row(speed=.11),dict(frame=9,control=control()),10)[1]=='unqualified_speed'


def test_T14_unknown_state_and_control_rejected():
    for key,value in [('gear',None),('speed_planar_mps',float('nan')),('brake',.1),('hand_brake',None)]:
        r=row();r[key]=value
        assert bounded_stop(r,dict(frame=9,control=control()),10)[0] is None
    assert bounded_stop(None,None,10)[0] is None


def test_T15_none_api_return_is_not_stop():
    s=acknowledged((1,));f=s.snapshot()['actors']['1']['facts']
    assert f['STOP_COMMAND_ACKNOWLEDGED']['ack_kind']=='api_call_returned' and 'STOP_OBSERVED' not in f


def test_T16_false_return_not_acknowledged():
    s=AbortState();s.register(1);s.request('test');c=control(.02)
    t=s.admit(1,10,c,s.reserve_stop(1,c));s.rpc_started(t);s.finish(t,returned=False)
    f=s.snapshot()['actors']['1']['facts']
    assert 'STOP_UNVERIFIED' in f and 'STOP_COMMAND_ACKNOWLEDGED' not in f


def test_T17_first_stop_rpc_loss_prevents_second_and_cleanup_rpc(tmp_path,monkeypatch):
    original_open=Path.open
    def broken(path,*a,**kw):
        if path.name=='collision_events.jsonl':raise OSError('concurrent callback disk failure')
        return original_open(path,*a,**kw)
    monkeypatch.setattr(Path,'open',broken)
    def fault(h):
        old=h.initialize_fleet
        def init(*a):
            old(*a)
            with pytest.raises(OSError):callback(h)
            actor=next(iter(h.slots.values()))['actor']
            actor.apply_control=lambda c:(_ for _ in ()).throw(RuntimeError('synthetic RPC timeout'))
        h.initialize_fleet=init
    h,r,calls,commands,w,live=collect(tmp_path/'run',monkeypatch,False,fault=fault)
    events=r['abort']['events']
    assert len([e for e in events if e['kind']=='STOP_COMMAND_ATTEMPTED'])==1
    assert h.gate.poisoned and h.gate.calls_after_poison==0 and not r['cleanup']['complete']
    assert len(live)>1 and w.settings.synchronous_mode


def test_T18_poison_no_control_rpc(tmp_path):
    h,calls=fixture(tmp_path);h.abort_state().request('test');h.gate.poisoned=True
    c=control(.005);p=h.abort_state().reserve_stop(1,c)
    with pytest.raises(ServerUnavailable):h.issue('abort',0,.005,0,_abort_permit=p)
    assert len(calls)==1


def test_T19_partial_actor_state_is_independent():
    s=acknowledged((1,));s.register(2);s.unverified(2,'missing_control')
    d=s.snapshot();assert 'STOP_COMMAND_ACKNOWLEDGED' in d['actors']['1']['facts']
    assert 'STOP_COMMAND_ACKNOWLEDGED' not in d['actors']['2']['facts'] and not d['stop_verified']


def test_T20_cleanup_not_stop_and_no_later_promotion():
    s=acknowledged((1,));s.cleanup_result(dict(attempted=True,complete=True))
    f=s.snapshot()['actors']['1']['facts']
    assert 'CLEANUP_COMPLETED' in f and 'STOP_UNVERIFIED' in f and 'STOP_OBSERVED' not in f
    with pytest.raises(AbortRejected):s.observed(1,101,111)
    with pytest.raises(AbortRejected):s.reserve_stop(1,control(.02))


def test_T21_valid_supplied_post_request_window_only():
    s=acknowledged();w=StopWindow(s,(1,2),100)
    for f in range(101,112):observed=w.feed(f,f*.05,stop_rows(f))
    assert observed and s.snapshot()['stop_verified']


def test_T22_duplicate_missing_or_invalid_clock():
    for frame,time in [(101,5.05),(103,5.15),(102,float('nan'))]:
        s=acknowledged();w=StopWindow(s,(1,2),100)
        w.feed(101,5.05,stop_rows(101))
        assert not w.feed(frame,time,stop_rows(frame)) and w.terminal
    for key,value in [('world_time_s',123.),('command_brake',.01),('control_read_at_frame',100)]:
        s=acknowledged();w=StopWindow(s,(1,2),100);r=stop_rows(101);r[1][key]=value
        assert not w.feed(101,5.05,r) and w.terminal
    s=acknowledged();w=StopWindow(s,(1,2),100)
    assert not w.feed(103,5.15,stop_rows(103)) and w.terminal


def test_T23_moving_sample_cannot_certify_stop():
    s=acknowledged();w=StopWindow(s,(1,2),100)
    for f in range(101,112):
        r=stop_rows(f);r[1]['vz']=.1
        assert not w.feed(f,f*.05,r)
    assert not s.snapshot()['stop_verified'] and not w.window


def test_T24_stop_timeout_no_live_clock_driver():
    s=acknowledged();w=StopWindow(s,(1,2),100)
    for f in range(101,201):w.feed(f,f*.05,stop_rows(f,.2))
    assert w.terminal and not s.snapshot()['stop_verified']
    assert s.snapshot()['actors']['1']['facts']['STOP_UNVERIFIED']['reason']=='stop_observation_timeout'


def test_T25_abort_has_no_additional_ticks(tmp_path,monkeypatch):
    def fault(h):
        h.capture=lambda:(_ for _ in ()).throw(EarlyStop('synthetic abort'))
    h,r,calls,*_=collect(tmp_path/'run',monkeypatch,False,fault=fault)
    assert calls.count('tick')==0 and not r['abort']['stop_verified']


def test_T26_destroy_failure_is_cleanup_failure(tmp_path,monkeypatch):
    def fault(h):
        old=h.prepare
        def prepare():
            old()
            for slot in h.slots.values():slot['actor'].destroy=lambda:False
        h.prepare=prepare;h.capture=lambda:(_ for _ in ()).throw(EarlyStop('abort'))
    h,r,*_=collect(tmp_path/'run',monkeypatch,False,fault=fault)
    assert r['exit_code']==5 and not r['cleanup']['complete']
    assert all('CLEANUP_COMPLETED' not in a['facts'] for a in r['abort']['actors'].values())
    h.abort_state().cleanup_result(dict(attempted=True,complete=True))
    assert all('CLEANUP_COMPLETED' not in a['facts'] for a in h.abort_state().snapshot()['actors'].values())


def test_T27_restore_failure_recorded(tmp_path,monkeypatch):
    def fault(h):
        old=h.prepare
        def prepare():old();h.world.apply_settings=lambda s:None
        h.prepare=prepare;h.capture=lambda:(_ for _ in ()).throw(EarlyStop('abort'))
    h,r,*_=collect(tmp_path/'run',monkeypatch,False,fault=fault)
    assert r['exit_code']==5 and not r['cleanup']['settings_restored']


def test_T28_single_legacy_abort_preserved(tmp_path,monkeypatch):
    def fault(h):h.capture=lambda:(_ for _ in ()).throw(EarlyStop('abort'))
    h,r,calls,commands,*_=collect(tmp_path/'run',monkeypatch,False,fault=fault,single=True)
    assert len(commands)==1 and commands[0][2]['brake']==.7 and calls.count('tick')==0
    assert r['cleanup']['complete'] and not r['abort']['stop_verified']


def test_T29_normal_stop_and_owned_partial_slots(tmp_path,monkeypatch):
    s=AbortState(pair=True);s.register(1);s.request('partial')
    with pytest.raises(AbortRejected):s.reserve_stop(1,control(.7),legacy=True)
    with pytest.raises(AbortRejected):s.reserve_stop(99,control(.005))
    def fault(h):
        original=h.prepare
        def prepare():
            original();del h.slots['leader']
            raise EarlyStop('synthetic partial preparation')
        h.prepare=prepare
    h,r,calls,commands,*_=collect(tmp_path/'partial',monkeypatch,False,fault=fault)
    assert len(r['abort']['actors'])==2 and not commands
    assert all('STOP_UNVERIFIED' in a['facts'] for a in r['abort']['actors'].values())


def test_T30_callback_observer_on_off_latch_equivalence(tmp_path,monkeypatch):
    def fault(h):
        old=h.initialize_fleet
        def init(*a):old(*a);callback(h)
        h.initialize_fleet=init
    off=collect(tmp_path/'off',monkeypatch,False,fault=fault)
    on=collect(tmp_path/'on',monkeypatch,True,fault=fault)
    assert off[2:4]==on[2:4]
    assert off[1]['abort']['stop_verified'] is on[1]['abort']['stop_verified'] is False


def test_T31_persistent_io_keeps_latch_and_reports_incomplete(tmp_path,monkeypatch):
    def fault(h):
        def capture():h.abort_state().request('disk_failure');raise OSError('synthetic disk')
        h.capture=capture
        h.ledger=lambda *a,**kw:(_ for _ in ()).throw(OSError('ledger disk failure'))
    h,r,*_=collect(tmp_path/'run',monkeypatch,False,fault=fault)
    assert h.abort_state().requested and r['exit_code']==5 and not r['abort']['stop_verified']


def test_T32_native_or_worker_loss_never_proves_stop():
    from Run_IDM_Following_Experiment import classify_exit
    assert classify_exit(0xC0000005,None)[0]=='native_abnormal_exit'
    s=acknowledged((1,));s.unverified(1,'worker_timeout')
    assert not s.snapshot()['stop_verified']


def test_T33_permit_identity_scope_and_replay_rejected():
    s=AbortState(pair=True);s.register(1);s.register(2);s.request('abort');c=control(.005)
    p=s.reserve_stop(1,c)
    with pytest.raises(AbortRejected):s.admit(2,10,c,p)
    with pytest.raises(AbortRejected):s.admit(1,10,c,p)
    with pytest.raises(AbortRejected):s.reserve_stop(1,c)


def test_T34_requested_ledger_not_rpc_delivery(tmp_path):
    h,calls=fixture(tmp_path);old=h.ledger
    def ledger(op,**kw):
        n=old(op,**kw)
        if kw.get('outcome')=='requested':h.abort_state().request('collision')
        return n
    h.ledger=ledger
    with pytest.raises(EarlyStop):h.issue('launch',.03,0,0)
    led=[json.loads(x) for x in (tmp_path/'command_ledger.jsonl').read_text().splitlines()]
    assert sum(e.get('outcome')=='requested' for e in led)==2 and len(calls)==1


def test_T35_same_frame_no_double_slew_and_per_actor_elapsed():
    assert bounded_stop(row(),dict(frame=10,control=control()),10)[1]=='no_slew_budget'
    assert bounded_stop(row(),dict(frame=9,control=control()),10)[0]['brake']==pytest.approx(.005)
    assert bounded_stop(row(),dict(frame=None,control=control()),10)[0] is None


def test_T36_local_rejection_does_not_poison_rpc_gate(tmp_path,monkeypatch):
    h,calls=fixture(tmp_path)
    h.gate=HealthGate(1,2,tmp_path/'unused_marker')
    h.call=h.gate.call
    monkeypatch.setattr(h.gate,'check',lambda:h.abort_state().request('collision'))
    with pytest.raises(EarlyStop):h.issue('launch',.03,0,0)
    assert not h.gate.poisoned and len(calls)==1
