"""Math, nonsmooth boundaries and independent ballistic/event checks, no CARLA."""
import builtins
from dataclasses import replace
import json
import math

import pytest

from idm_model import IDMParameters, compute_idm_acceleration
from idm_t_sensitivity import State1D, T_VALUES, advance, gap_of, rollout, step_geometry, t_sensitivity


def test_analytic_derivative_matches_independent_expression_and_both_differences():
    p=IDMParameters(30.)
    r=t_sensitivity(10.,10.,30.,p)
    expected=-2*1.5*(2+10*1.5)*10/30**2
    assert r['analytic_derivative']==pytest.approx(expected)
    assert r['fd_central']==pytest.approx(expected,abs=1e-9)
    assert r['fd_left']==pytest.approx(expected,abs=2e-6)
    assert r['fd_right']==pytest.approx(expected,abs=2e-6)


@pytest.mark.parametrize('v,vl,g',[(0,0,20),(5,20,20),(15,15,None),(15,0,0),(15,0,-2),(30,0,.5)])
def test_insensitive_regimes(v,vl,g):
    p=IDMParameters(30.);r=t_sensitivity(v,vl,g,p)
    assert r['analytic_derivative']==0
    assert r['fd_left']==r['fd_right']==0
    assert len({compute_idm_acceleration(v,replace(p,desired_time_headway_s=T),g,vl) for T in T_VALUES})==1


def test_max_boundary_has_distinct_one_sided_derivatives():
    p=IDMParameters(30.,desired_time_headway_s=1.)
    r=t_sensitivity(10.,10.+2*math.sqrt(3),30.,p)
    assert r['regime']=='max_boundary'
    assert r['analytic_derivative'] is None
    assert r['analytic_left']==0 and r['analytic_right']==pytest.approx(-2*1.5*2*10/900)
    assert r['fd_left']==pytest.approx(0.,abs=1e-9)
    assert r['fd_right']==pytest.approx(r['analytic_right'],abs=2e-6)


def test_lower_clamp_boundary_has_distinct_one_sided_derivatives():
    p=IDMParameters(30.);q=17.
    gap=q/math.sqrt(1-(10/30)**4+4/1.5)
    r=t_sensitivity(10.,10.,gap,p)
    assert r['regime']=='lower_clamp_boundary' and r['analytic_derivative'] is None
    assert r['analytic_left']<0 and r['analytic_right']==0
    assert r['fd_left']==pytest.approx(r['analytic_left'],rel=1e-5)
    assert r['fd_right']==pytest.approx(0.,abs=1e-8)
    assert r['fd_central']!=pytest.approx(r['analytic_left'])


def test_T_domain_boundary_and_zero_step_rejection():
    p=IDMParameters(30.,desired_time_headway_s=0.)
    r=t_sensitivity(10.,10.,30.,p)
    assert r['fd_left'] is None and r['analytic_left'] is None
    assert r['analytic_derivative']==r['analytic_right']
    with pytest.raises(ValueError):t_sensitivity(10,10,20,p,h=0)


def test_T_monotonic_acceleration_over_full_fixed_state_grid():
    for v in (0,.5,5.,15.,30.):
        for vl in (0.,5.,15.,30.):
            for g in (None,.001,2.,20.,100.):
                a=[compute_idm_acceleration(v,IDMParameters(30.,desired_time_headway_s=T),g,vl) for T in T_VALUES]
                assert all(x>=y for x,y in zip(a,a[1:]))


def test_ballistic_update_stops_exactly_and_never_moves_backwards():
    s=State1D(1.,2.)
    assert advance(s,-4.,2.)==State1D(1.5,0.)
    assert advance(s,2.,.5)==State1D(2.25,3.)
    assert advance(State1D(0.,0.),-4.,2.)==State1D(0.,0.)


def test_contact_is_located_inside_step_using_closed_form_reference():
    t,g=step_geometry(State1D(0.,30.),State1D(9.8,0.),-4.,0.,1.)
    assert t==pytest.approx((30-math.sqrt(860))/4,abs=1e-12)
    assert g==0


def test_hidden_overlap_then_separation_is_detected():
    f,l=State1D(0.,10.),State1D(5.8,0.)
    assert gap_of(advance(f,-10.,2.),advance(l,10.,2.))>0
    t,g=step_geometry(f,l,-10.,10.,2.)
    assert t==pytest.approx((10-math.sqrt(60))/20,abs=1e-12) and g==0


def test_initial_overlap_is_censored_and_no_postcontact_trajectory_exists():
    p=IDMParameters(30.)
    r=rollout(State1D(0.,10.),State1D(4.,0.),p,p)
    assert r['status']=='initial_overlap' and len(r['rows'])==1
    assert r['contact_time_s']==0 and r['minimum_continuous_gap_m']<0
    r=rollout(State1D(0.,30.),State1D(9.8,0.),p,p,dt=1.,leader_acceleration=lambda t:0.)
    assert r['status']=='contact' and len(r['rows'])==2
    assert r['rows'][-1]['time_s']==r['contact_time_s']<1.
    assert abs(r['rows'][-1]['gap_m'])<1e-10


def test_simultaneous_old_state_evaluation_and_matched_T_only_changes():
    p=IDMParameters(30.);f,l=State1D(0.,15.),State1D(34.8,15.)
    a=compute_idm_acceleration(15.,p,30.,15.)
    runs=[rollout(f,l,replace(p,desired_time_headway_s=T),p,dt=.05,horizon=3.,leader_acceleration=lambda t:0.) for T in T_VALUES]
    for r in runs:
        assert r['initial_follower']==runs[0]['initial_follower'] and r['initial_leader']==runs[0]['initial_leader']
        assert r['leader_parameters']==runs[0]['leader_parameters']
        assert [x['leader_x_m'] for x in r['rows']]==[x['leader_x_m'] for x in runs[0]['rows']]
    reference=runs[2]
    assert reference['rows'][1]['follower_x_m']==pytest.approx(15*.05+.5*a*.05**2)
    assert runs[0]['follower_distance_m']>runs[-1]['follower_distance_m']


def test_step_refinement_and_contact_event_convergence():
    p=IDMParameters(30.)
    rs=[rollout(State1D(0.,15.),State1D(29.8,10.),p,p,dt=d,horizon=6.,leader_acceleration=lambda t:0.)
        for d in (.1,.05,.025,.0125)]
    ref=rs[-1]['follower_distance_m'];err=[abs(r['follower_distance_m']-ref) for r in rs[:-1]]
    assert err[0]>err[1]>err[2]
    ts=[rollout(State1D(0.,30.),State1D(9.8,0.),p,p,dt=d,leader_acceleration=lambda t:0.)['contact_time_s'] for d in (.2,.1,.05)]
    assert max(ts)-min(ts)<1e-12


def test_fractional_horizon_and_nonclosing_ttc_are_not_filled_with_risk_values():
    p=IDMParameters(30.)
    r=rollout(State1D(0.,0.),State1D(24.8,10.),p,p,dt=.05,horizon=.13,leader_acceleration=lambda t:0.)
    assert r['duration_s']==.13 and len(r['rows'])==4
    assert r['minimum_sampled_ttc_s'] is None
    assert all(x['ttc_s'] is None and x['follower_speed_mps']>=0 for x in r['rows'])


@pytest.mark.parametrize('dt,horizon',[(0,6),(-1,6),(.05,0),(1e-8,6),(.05,float('inf'))])
def test_invalid_or_unbounded_design_rejected(dt,horizon):
    with pytest.raises(ValueError):rollout(State1D(0.,0.),State1D(20.,0.),IDMParameters(30.),IDMParameters(30.),dt=dt,horizon=horizon)


def test_offline_cli_without_evidence_never_imports_carla_and_rejects_overwrite(tmp_path,monkeypatch):
    real=builtins.__import__
    def guarded(name,*a,**kw):
        if name=='carla':pytest.fail('CARLA import forbidden')
        return real(name,*a,**kw)
    monkeypatch.setattr(builtins,'__import__',guarded)
    from Analyze_IDM_T import main
    out=tmp_path/'new'
    assert main(['--output-root',str(out)])==0
    s=json.loads((out/'summary.json').read_text())
    assert s['offline_only'] and s['route_audit'] is None and s['physical_bounds'] is None
    assert s['carla_identifiability_verified'] is False and s['risk_accuracy_verified'] is False
    with pytest.raises(ValueError,match='new output'):main(['--output-root',str(out)])
