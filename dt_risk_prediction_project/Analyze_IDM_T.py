"""Offline T sensitivity and ideal 1D trajectories. Never creates a CARLA client."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path

from idm_model import IDMParameters, compute_idm_acceleration
from idm_snapshot import read_snapshot
from idm_fidelity import csv_rows, write_csv
from idm_t_sensitivity import T_VALUES, State1D, rollout, t_sensitivity


def cases():
    normal=IDMParameters(30.)
    return [dict(name='equal_speed',follower=State1D(0.,15.),leader=State1D(34.8,15.),
                 parameters=normal,leader_parameters=normal,horizon=20.,leader_mode='constant_speed'),
            dict(name='closing',follower=State1D(0.,15.),leader=State1D(29.8,10.),
                 parameters=normal,leader_parameters=normal,horizon=20.,leader_mode='constant_speed'),
            dict(name='leader_braking',follower=State1D(0.,20.),leader=State1D(44.8,20.),
                 parameters=normal,leader_parameters=normal,horizon=20.,leader_mode='brake_2_to_6s'),
            dict(name='contact_stress',follower=State1D(0.,30.),leader=State1D(9.8,0.),
                 parameters=normal,leader_parameters=normal,horizon=6.,leader_mode='constant_speed')]


def profile(mode):
    if mode=='free_idm':return None
    if mode=='constant_speed':return lambda t:0.
    if mode=='brake_2_to_6s':return lambda t:-2. if 2.<=t<6. else 0.
    raise ValueError('unknown declared leader profile')


def saved_case(root, hashes):
    """Project the saved flat two-vehicle V2 snapshot, using its existing reader.

    Same V2 speed magnitude convention; the ideal model cannot restore CARLA
    drivetrain/contact/steering states. This is not a physical snapshot replay.
    """
    receipt=Path(root)/'fleet_idm_once/controller_result.json'
    hashes[str(receipt)]=hashlib.sha256(receipt.read_bytes()).hexdigest()
    data=json.loads(receipt.read_text(encoding='utf-8-sig'))
    if data.get('successful') is not True:raise ValueError('source fleet run incomplete')
    run=Path(data['run_dir'])
    def track(path):
        hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest();return path
    snapshot=read_snapshot(track(run/'initial_snapshot.json'))
    actors={r['logical_actor_id']:r for r in snapshot}
    if set(actors)!={'leader','follower'}:raise ValueError('expected saved two-vehicle snapshot')
    parameters=json.loads(track(run/'resolved_idm_parameters.json').read_text())
    route=json.loads(track(run/'local_route.json').read_text())['points']
    axis=[route[-1][i]-route[0][i] for i in range(3)];norm=math.hypot(*axis);axis=[v/norm for v in axis]
    boxes={}
    for line in track(run/'calibration_raw.jsonl').read_text(encoding='utf-8-sig').splitlines():
        for r in json.loads(line)['rows']:boxes.setdefault(r['actor_id'],r['bounding_box'])
    states={}
    for key,r in actors.items():
        if abs(r['pitch'])>1e-8 or abs(r['roll'])>1e-8:raise ValueError('projection requires flat saved snapshot')
        b=boxes[r['source_actor_id']];yaw=math.radians(r['yaw']);c,s=math.cos(yaw),math.sin(yaw)
        if c*axis[0]+s*axis[1]<.99:raise ValueError('snapshot vehicle is not aligned with route')
        center=[r['x']+c*b['location']['x']-s*b['location']['y'],
                r['y']+s*b['location']['x']+c*b['location']['y'],r['z']+b['location']['z']]
        states[key]=State1D(sum(center[i]*axis[i] for i in range(3)),
                           math.hypot(r['vx'],r['vy'],r['vz']),2*b['extent']['x'])
    origin=states['follower'].x
    initial_gap=states['leader'].x-states['follower'].x-(states['leader'].length+states['follower'].length)/2
    recorded=next(r for r in csv_rows(track(run/'v2_validation.csv')) if r['logical_actor_id']=='follower')
    if abs(initial_gap-float(recorded['v2_gap_m']))>1e-5:
        raise ValueError('projected snapshot gap differs from recorded V2 initial gap')
    return dict(name='saved_low_snapshot',follower=replace(states['follower'],x=0.),
        leader=replace(states['leader'],x=states['leader'].x-origin),
        parameters=IDMParameters(**parameters['follower']),leader_parameters=IDMParameters(**parameters['leader']),
        horizon=6.,leader_mode='free_idm',projection='flat bbox-center projection; initial speed uses V2 3D magnitude, not exact signed/vertical dynamics')


def historical_rows(root, hashes):
    path=Path(root)/'all_idm_observations.csv'
    hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    result=[]
    for r in csv_rows(path):
        p=IDMParameters(float(r['v0_mps']),float(r['T_s']),float(r['s0_m']),float(r['a_mps2']),
                        float(r['b_mps2']),4.,float(r['maximum_deceleration_parameter_mps2']))
        v=float(r['v2_speed_mps']);vl=float(r['v2_leader_speed_mps']) if r['v2_leader_speed_mps'] else 0.
        gap=float(r['v2_gap_m']) if r['v2_gap_m'] else None
        if abs(compute_idm_acceleration(v,p,gap,vl)-float(r['v2_idm_acceleration_mps2']))>1e-10:
            raise ValueError('saved model parameters/state do not reproduce recorded request')
        for T in T_VALUES:
            result.append(dict(provenance='counterfactual_on_saved_states_not_new_measurement',
                trial=r['trial'],logical_actor_id=r['logical_actor_id'],source_frame=r['command_based_on_frame'],
                **{k:v for k,v in asdict(p).items() if k!='desired_time_headway_s'},
                **t_sensitivity(v,vl,gap,replace(p,desired_time_headway_s=T))))
    return result


def run(output, evidence_root=None, plots=False):
    output=Path(output)
    if output.exists() or (evidence_root and output.resolve().is_relative_to(Path(evidence_root).resolve())):
        raise ValueError('use a new output directory outside original evidence')
    hashes={};conditions=cases()
    historic=[]
    if evidence_root:
        conditions.insert(0,saved_case(evidence_root,hashes))
        historic=historical_rows(evidence_root,hashes)
    grid=[]
    for design,p,velocities,gaps in [
        ('low',IDMParameters(.6,2.,2.,.3,1.5,4.,2.),(0.,.1,.3,.5,.6),(2.,5.,15.2,30.)),
        ('normal',IDMParameters(30.),(0.,.1,.3,.5,1.,3.,5.,10.,15.,20.,25.,30.),
         (.5,1.,2.,3.,5.,8.,10.,15.2,20.,30.,40.,60.,100.))]:
        for v in velocities:
            for gap in gaps:
                for dv in (-3.,0.,3.):
                    vl=max(0.,v-dv)
                    for T in T_VALUES:
                        grid.append(dict(design=design,**{k:x for k,x in asdict(p).items() if k!='desired_time_headway_s'},
                                         **t_sensitivity(v,vl,gap,replace(p,desired_time_headway_s=T))))
    summaries=[];trajectories=[];convergence=[];stored={}
    for c in conditions:
        for T in T_VALUES:
            p=replace(c['parameters'],desired_time_headway_s=T)
            for dt in (.2,.1,.05,.025,.0125):
                result=rollout(c['follower'],c['leader'],p,c['leader_parameters'],dt=dt,horizon=c['horizon'],
                               leader_acceleration=profile(c['leader_mode']))
                rows=result.pop('rows')
                convergence.append(dict(scenario=c['name'],T_s=T,dt_s=dt,status=result['status'],
                    distance_m=result['follower_distance_m'],final_speed_mps=result['final_follower_speed_mps'],
                    minimum_gap_m=result['minimum_continuous_gap_m'],contact_time_s=result['contact_time_s']))
                if dt==.05:
                    summaries.append(dict(scenario=c['name'],leader_mode=c['leader_mode'],T_s=T,**result))
                    trajectories.extend(dict(scenario=c['name'],**r) for r in rows)
                    stored[(c['name'],T)]=rows
    for c in convergence:
        ref=next(x for x in convergence if x['scenario']==c['scenario'] and x['T_s']==c['T_s'] and x['dt_s']==.0125)
        # Never compare endpoints at different contact/horizon times as equal-time errors.
        same_time=(c['status']==ref['status'] and
                   (c['contact_time_s'] is None and ref['contact_time_s'] is None))
        c['distance_difference_to_finest_m']=c['distance_m']-ref['distance_m'] if same_time else None
        c['contact_time_difference_to_finest_s']=(c['contact_time_s']-ref['contact_time_s']
              if c['contact_time_s'] is not None and ref['contact_time_s'] is not None else None)
    description=[]
    for c in conditions:
        description.append({k:asdict(v) if k in ('follower','leader','parameters','leader_parameters') else v for k,v in c.items()})
    summary=dict(offline_only=True,model='explicit simultaneous IDM + ballistic 1D with exact stop and first-contact termination',
        T_values_s=T_VALUES,base_dt_s=.05,conditions=description,results=summaries,
        historical_actor_states=len(historic)//5 if historic else 0,source_sha256=hashes,
        source_code_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                           (Path(__file__),Path(__file__).with_name('idm_t_sensitivity.py'),Path(__file__).with_name('idm_model.py'))},
        route_audit=None,physical_bounds=None,carla_identifiability_verified=False,risk_accuracy_verified=False)
    output.mkdir(parents=True,exist_ok=False)
    write_csv(output/'sensitivity_grid.csv',grid)
    write_csv(output/'trajectories.csv',trajectories)
    write_csv(output/'time_step_comparison.csv',convergence)
    write_csv(output/'scenario_results.csv',[
        {**{k:v for k,v in r.items() if not isinstance(v,(dict,list))},**r['parameters'],
         **{'initial_follower_'+k:v for k,v in r['initial_follower'].items()},
         **{'initial_leader_'+k:v for k,v in r['initial_leader'].items()}} for r in summaries])
    if historic:write_csv(output/'historical_T_counterfactual.csv',historic)
    (output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False),encoding='utf-8')
    if plots:plot_results(output,stored)
    return summary


def plot_results(output, trajectories):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    normal=IDMParameters(30.)
    fig,ax=plt.subplots(figsize=(9,5),layout='constrained')
    for name,v,vl,g,p in [('past low-speed range',.5,.45,15.2,IDMParameters(.6,2.,2.,.3,1.5,4.,2.)),
                         ('equal speed 15 m/s',15.,15.,30.,normal),('closing 20->15 m/s',20.,15.,30.,normal),
                         ('opening max inactive',5.,20.,10.,normal)]:
        ax.plot(T_VALUES,[compute_idm_acceleration(v,replace(p,desired_time_headway_s=T),g,vl) for T in T_VALUES],marker='o',label=name)
    ax.set(xlabel='T [s]',ylabel='Clamped IDM acceleration [m/s2]',title='Fixed states and other parameters within each comparison')
    ax.legend();ax.grid(alpha=.2);fig.savefig(output/'T_acceleration.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    velocities=list(range(31));gaps=list(range(2,102,2))
    map_rows=[]
    for ax,closing in zip(axes,(0,3)):
        data=[]
        for v in velocities:
            values=[t_sensitivity(v,max(0,v-closing),g,normal) for g in gaps]
            data.append([r['analytic_derivative'] for r in values])
            map_rows.extend(dict(closing_design_mps=closing,**r) for r in values)
        # None at nonsmooth points is displayed as NaN only in the plot, never saved as a numeric observation.
        numeric=[[float('nan') if x is None else x for x in row] for row in data]
        im=ax.imshow(numeric,origin='lower',aspect='auto',extent=(1,101,-.5,30.5),vmin=-3,vmax=0,cmap='viridis')
        ax.set(xlabel='Bumper gap [m]',ylabel='Follower speed [m/s]',title=f'T=1.5s, vL=max(0,v-{closing}) [m/s]')
        fig.colorbar(im,ax=ax,label='da/dT [m/s3]; color clipped below -3')
    fig.savefig(output/'T_sensitivity_maps.png',dpi=160);plt.close(fig)
    write_csv(output/'heatmap_data.csv',map_rows)
    for scenario in sorted({k[0] for k in trajectories}):
        fig,axes=plt.subplots(4,1,figsize=(10,11),sharex=True,layout='constrained')
        for T in T_VALUES:
            rows=trajectories[(scenario,T)];t=[r['time_s'] for r in rows]
            for ax,key in zip(axes,('follower_speed_mps','gap_m','follower_x_m','ttc_s')):
                ax.plot(t,[r[key] for r in rows],label=f'T={T}s')
        for ax,label in zip(axes,('Speed [m/s]','Bumper gap [m]','Position [m]','Sampled TTC [s]')):
            ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(fontsize=7)
        axes[3].set_ylim(0,40);axes[3].set_xlabel('Time [s]; curve stops at first contact')
        fig.suptitle(scenario+': ideal 1D model, not CARLA physics; TTC absent when not closing')
        fig.savefig(output/(scenario+'_trajectories.png'),dpi=160);plt.close(fig)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',required=True,type=Path)
    parser.add_argument('--evidence-root',type=Path,help='optional saved V2 experiment root; read only')
    parser.add_argument('--plots',action='store_true',help='optional matplotlib; core analysis requires only stdlib')
    args=parser.parse_args(argv)
    result=run(args.output_root,args.evidence_root,args.plots)
    print(json.dumps(dict(scenarios=len(result['conditions']),runs=len(result['results']),offline_only=True)))
    return 0


if __name__=='__main__':raise SystemExit(main())
