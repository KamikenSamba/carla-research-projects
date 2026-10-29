"""Server-free fidelity diagnostics for saved V2 validation runs.

Extends the existing experiment's analyze_v2_runs.py calculation, without its
live-supervisor import or risk/snapshot analyses. Never estimates missing data.
Controller internals replayed from a fresh controller are explicitly labelled.
"""
from collections import defaultdict
import csv
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import struct
from types import SimpleNamespace

from idm_controller import ControlConfig, VehicleController
from idm_model import IDMParameters, compute_idm_acceleration
from idm_carla_validation import BoundedV2Actuator


def number(value):
    if isinstance(value, bool):
        raise ValueError('boolean is not a measurement')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('nonfinite measurement')
    return value


def stats(values):
    values = [number(v) for v in values]
    if not values:
        return dict(n=0, minimum=None, maximum=None, mean=None, median=None, std=None)
    return dict(n=len(values), minimum=min(values), maximum=max(values),
                mean=statistics.mean(values), median=statistics.median(values),
                std=statistics.pstdev(values))


def errors(measured, requested):
    if len(measured) != len(requested):
        raise ValueError('unpaired errors')
    e = [number(a) - number(b) for a, b in zip(measured, requested)]
    if not e:
        return dict(n=0, rmse=None, mae=None, bias=None, residual_std=None)
    return dict(n=len(e), rmse=math.sqrt(statistics.mean(x*x for x in e)),
                mae=statistics.mean(abs(x) for x in e), bias=statistics.mean(e),
                residual_std=statistics.pstdev(e))


def csv_rows(path):
    with Path(path).open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def unique_index(rows, keys):
    result = {}
    for row in rows:
        key = tuple(row[k] for k in keys)
        if key in result:
            raise ValueError('duplicate observation key: '+str(key))
        result[key] = row
    return result


def close(a, b, tolerance=1e-9):
    if abs(number(a)-number(b)) > tolerance:
        raise ValueError(f'inconsistent recorded/replayed value: {a} versus {b}')


def control_float(value):
    """CARLA VehicleControl stores float32; replay the sink input, not a double."""
    return struct.unpack('f', struct.pack('f', number(value)))[0]


def acceleration_definitions(previous, current):
    """Interval FD projected onto CURRENT body direction; magnitude FD is distinct."""
    dt = number(current['world_time_s'])-number(previous['world_time_s'])
    if dt <= 0:
        raise ValueError('nonpositive observation interval')
    v = [number(current['v'+k]) for k in 'xyz']
    old_v = [number(previous['v'+k]) for k in 'xyz']
    f = [number(current['forward_'+k]) for k in 'xyz']
    old_f = [number(previous['forward_'+k]) for k in 'xyz']
    dot = lambda a, b: sum(x*y for x, y in zip(a, b))
    dv = [(a-b)/dt for a, b in zip(v, old_v)]
    displacement = [number(current[k])-number(previous[k]) for k in 'xyz']
    return dict(interval_s=dt, api_projection=dot([number(current['a'+k]) for k in 'xyz'], f),
        vector_fd=dot(dv, f), magnitude_fd=(math.hypot(*v)-math.hypot(*old_v))/dt,
        signed_speed_fd=(dot(v, f)-dot(old_v, old_f))/dt,
        signed_position_interval_speed=dot(displacement, f)/dt,
        body_endpoint_speed=dot(v, f))


def lag_metrics(rows, max_lag=10):
    """k compares command F with response F+1+k. Fixed interior command support.

    This is a descriptive shift scan, not an estimate of physical control latency.
    """
    if len(rows) <= 2*max_lag:
        return []
    result = []
    for k in range(-max_lag, max_lag+1):
        indices = range(max_lag, len(rows)-max_lag)
        result.append(dict(response_shift_frames=k, **errors(
            [rows[i+k]['api_a'] for i in indices], [rows[i]['raw_idm_a'] for i in indices])))
    return result


def analyze_run(run_dir, label):
    run_dir = Path(run_dir)
    required = ['v2_validation.csv', 'calibration_observations.csv', 'command_ledger.jsonl',
                'resolved_control_config.json', 'resolved_idm_parameters.json', 'config_used.json',
                'launch.json', 'calibration_raw.jsonl', 'longitudinal_diagnostics.jsonl']
    hashes = {name: hashlib.sha256((run_dir/name).read_bytes()).hexdigest() for name in required}
    scenario = read_json(run_dir/'config_used.json')['scenario']
    dt = number(scenario['dt'])
    close(dt, .05)
    config = ControlConfig(**read_json(run_dir/'resolved_control_config.json'))
    params = read_json(run_dir/'resolved_idm_parameters.json')
    motion = csv_rows(run_dir/'calibration_observations.csv')
    index = unique_index(motion, ('actor_id', 'frame'))
    requests = csv_rows(run_dir/'v2_validation.csv')
    unique_index(requests, ('actor_id', 'observation_frame'))
    ledger = [json.loads(line) for line in (run_dir/'command_ledger.jsonl').read_text(encoding='utf-8-sig').splitlines()]
    sent = [r for r in ledger if r.get('operation') == 'manual_control' and r.get('outcome') == 'requested']
    sent = unique_index(sent, ('actor_id', 'sequence'))
    groups = defaultdict(list)
    for r in requests:
        groups[r['logical_actor_id']].append(r)
    if not groups:
        raise ValueError('empty IDM observations')
    summary, details, phases = {}, [], []
    for logical, rows in sorted(groups.items()):
        rows.sort(key=lambda r: int(r['observation_frame']))
        actor_motion = [r for r in motion if r['actor_id'] == rows[0]['actor_id']]
        settle = [r for r in actor_motion if r['phase']=='settle']
        launch = [r for r in actor_motion if r['phase']=='launch']
        if (len(rows) != round(number(scenario['drive_seconds'])/dt) or len(launch) != len(rows)
                or not settle or int(rows[0]['command_based_on_frame']) != max(int(r['frame']) for r in settle)):
            raise ValueError('incomplete controller history from initialization')
        p = IDMParameters(**params[logical])
        controller = VehicleController(p, dt, config)
        actuator = BoundedV2Actuator(SimpleNamespace(**scenario))
        local = []
        for i, r in enumerate(rows):
            source_frame, response_frame = int(r['command_based_on_frame']), int(r['observation_frame'])
            if response_frame != source_frame+1 or (i and source_frame != int(rows[i-1]['observation_frame'])):
                raise ValueError('noncontiguous F -> F+1 comparison')
            previous, current = (index[(r['actor_id'], str(f))] for f in (source_frame, response_frame))
            close(number(r['observation_time_s'])-number(r['v2_simulation_time']), dt)
            close(r['v2_simulation_time'], previous['sim_time_s'])
            close(r['observation_time_s'], current['sim_time_s'])
            close(r['v2_speed_mps'], previous['speed_mps'])
            if int(current['control_read_at_frame']) != response_frame:
                raise ValueError('control read frame mismatch')
            gap = None if r['v2_gap_m'] == '' else number(r['v2_gap_m'])
            leader_v = 0.0 if r['v2_leader_speed_mps'] == '' else number(r['v2_leader_speed_mps'])
            raw = number(r['v2_idm_acceleration_mps2'])
            close(raw, compute_idm_acceleration(number(r['v2_speed_mps']), p, gap, leader_v))
            # Longitudinal computation is independent of steering PID state.
            target, th, br, _ = controller.command(number(r['v2_speed_mps']), raw, 0., bool(r['v2_fallback_reason']))
            for a, key in ((target, 'v2_target_speed_mps'), (th, 'v2_throttle'), (br, 'v2_brake'),
                           (controller.acceleration, 'v2_filtered_acceleration_mps2')):
                close(a, r[key])
            bounded = actuator.step(SimpleNamespace(throttle=control_float(th), brake=control_float(br),
                                    steer=control_float(r['v2_steer'])),
                                    number(previous['speed_planar_mps']), r['v2_fallback_reason'])
            command = sent[(int(r['actor_id']), int(current['command_sequence']))]
            if command['based_on_frame'] != source_frame:
                raise ValueError('ledger command frame mismatch')
            for value, key in zip(bounded, ('throttle', 'brake', 'steer')):
                close(value, command['control'][key])
                close(value, current['command_'+key])
                close(value, current[key], 1e-5)
                close(value, r['actual_'+key], 1e-5)
            a = acceleration_definitions(previous, current)
            close(a['interval_s'], dt, 1e-6)
            close(a['api_projection'], r['a_long_mps2'])
            close(a['vector_fd'], r['a_fd_long_mps2'], 1e-6)
            cd, ad = controller.last_diagnostics, actuator.last_diagnostics
            local.append(dict(trial=label, logical_actor_id=logical, actor_id=int(r['actor_id']),
                source_frame=source_frame, response_frame=response_frame,
                elapsed_s=number(r['observation_time_s'])-number(rows[0]['v2_simulation_time']),
                response_time_s=number(r['observation_time_s']), state_time_s=number(r['v2_simulation_time']),
                raw_idm_a=raw, filtered_idm_a=controller.acceleration, api_a=a['api_projection'],
                vector_fd_a=a['vector_fd'], magnitude_fd_a=a['magnitude_fd'], signed_speed_fd_a=a['signed_speed_fd'],
                interval_s=a['interval_s'], signed_position_interval_speed=a['signed_position_interval_speed'],
                body_endpoint_speed=a['body_endpoint_speed'], speed=number(current['speed_planar_mps']),
                requested_throttle=th, actual_throttle=number(current['throttle']), actual_brake=number(current['brake']),
                pid_p_replayed=cd['pid_p'], pid_i_replayed=cd['pid_i'], pid_d_replayed=cd['pid_d'],
                pid_output_replayed=cd['pid_output'], feed_forward_replayed=cd['feed_forward'],
                effort_replayed=cd['effort_before_clamp'], throttle_scaled=ad['throttle_scaled'],
                throttle_capped=ad['throttle_after_cap'], throttle_slew_limited=ad['throttle_slew_limited'],
                replay_provenance='offline_controller_replay_not_historical_internal_measurement'))
        get = lambda k: [r[k] for r in local]
        calc, actual = get('raw_idm_a'), get('api_a')
        mean_a = statistics.mean(actual)
        denom = sum((a-mean_a)**2 for a in actual)
        corr1 = None if not denom else sum((a-mean_a)*(b-mean_a) for a,b in zip(actual,actual[1:]))/denom
        metrics = dict(actor_id=int(rows[0]['actor_id']), parameters=asdict(p), frames=len(local),
            frame_alignment='validated F state/command -> F+1 response; one tick; ledger and control echo matched',
            raw_request_api_error=errors(actual, calc), filtered_request_api_error=errors(actual, get('filtered_idm_a')),
            raw_request_vector_fd_error=errors(get('vector_fd_a'), calc),
            api_vs_vector_fd=errors(actual, get('vector_fd_a')),
            api_vs_magnitude_fd=errors(actual, get('magnitude_fd_a')),
            api_vs_signed_speed_fd=errors(actual, get('signed_speed_fd_a')),
            position_interval_vs_endpoint_speed=errors(get('signed_position_interval_speed'), get('body_endpoint_speed')),
            raw_request=stats(calc), api_acceleration=stats(actual), speed=stats(get('speed')),
            acceleration_lag1_autocorrelation=corr1,
            negative_body_endpoint_speed_count=sum(r['body_endpoint_speed']<0 for r in local),
            negative_signed_position_interval_count=sum(r['signed_position_interval_speed']<0 for r in local),
            adjacent_acceleration_sign_flip_fraction=sum(a*b<0 for a,b in zip(actual,actual[1:]))/max(1,len(actual)-1),
            negative_idm_count=sum(a<0 for a in calc), actual_brake_nonzero_count=sum(a>0 for a in get('actual_brake')),
            raw_effort_over_one_count=sum(r['effort_replayed']>1 for r in local),
            post_scale_throttle_cap_count=sum(r['throttle_scaled']>scenario['throttle_cap'] for r in local),
            throttle_slew_limited_count=sum(r['throttle_slew_limited'] for r in local),
            pid_output_replayed=stats(get('pid_output_replayed')), feed_forward_replayed=stats(get('feed_forward_replayed')),
            lag_scan=lag_metrics(local), time_bands={})
        for name, subset in [('first_4s', [r for r in local if r['elapsed_s']<=4.+1e-9]),
                             ('after_4s', [r for r in local if r['elapsed_s']>4.+1e-9])]:
            metrics['time_bands'][name] = dict(note='time band, not a claim of settled driving',
                **errors([r['api_a'] for r in subset], [r['raw_idm_a'] for r in subset]))
        # Nonoverlapping 0.5 s interval means. Descriptive resolution comparison,
        # never a replacement for raw safety monitoring or a new latency fit.
        blocks = [local[i:i+10] for i in range(0, len(local)-9, 10)]
        metrics['half_second_block_error'] = errors(
            [statistics.mean(r['vector_fd_a'] for r in b) for b in blocks],
            [statistics.mean(r['raw_idm_a'] for r in b) for b in blocks])
        summary[logical] = metrics
        details.extend(local)
        by_phase = defaultdict(list)
        for r in actor_motion:
            by_phase[r['phase']].append(r)
        for phase, phase_rows in sorted(by_phase.items()):
            phases.append(dict(trial=label, logical_actor_id=logical, phase=phase,
                frames=len(phase_rows), speed=stats([r['speed_planar_mps'] for r in phase_rows]),
                acceleration=stats([r['a_long_mps2'] for r in phase_rows]),
                idm_error=metrics['raw_request_api_error'] if phase=='launch' else None,
                note='No IDM commands during settle/staged stopping; IDM error is unavailable there'))
    return dict(trial=label, run_dir=str(run_dir), source_commit=read_json(run_dir/'launch.json')['source_commit'],
                source_sha256=hashes, groups=summary, phase_statistics=phases,
                aggregate_error=errors([r['api_a'] for r in details],[r['raw_idm_a'] for r in details])), details


def write_csv(path, rows):
    with Path(path).open('x', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def plot_fidelity(output, details):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    groups = defaultdict(list)
    for r in details:
        groups[(r['trial'], r['logical_actor_id'])].append(r)
    for (trial, logical), rows in groups.items():
        fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, layout='constrained')
        x = [r['elapsed_s'] for r in rows]
        for key, style in [('api_a','-'), ('raw_idm_a','--'), ('filtered_idm_a',':')]:
            axes[0].plot(x,[r[key] for r in rows],style,label=key,lw=1)
        axes[0].set_ylabel('Acceleration [m/s2]')
        for key in ('requested_throttle','throttle_scaled','actual_throttle'):
            axes[1].plot(x,[r[key] for r in rows],label=key)
        axes[1].set_ylabel('Control input')
        for key in ('pid_output_replayed','feed_forward_replayed','effort_replayed'):
            axes[2].plot(x,[r[key] for r in rows],label=key)
        axes[2].axhline(1.,color='k',ls=':',lw=.6)
        axes[2].set_ylabel('Offline replay (not logged)')
        axes[3].plot(x,[r['speed'] for r in rows],label='planar speed')
        axes[3].plot(x,[r['body_endpoint_speed'] for r in rows],label='signed API body endpoint speed',alpha=.6)
        axes[3].plot(x,[r['signed_position_interval_speed'] for r in rows],label='signed position interval mean')
        axes[3].set_ylabel('Speed [m/s]');axes[3].set_xlabel('Time from first command [s]')
        for ax in axes:
            ax.legend(fontsize=7);ax.grid(alpha=.2)
        fig.suptitle(f'{trial}/{logical}: raw response vs constrained V2 control; no fitted delay')
        fig.savefig(Path(output)/f'{trial}_{logical}_fidelity.png',dpi=160)
        plt.close(fig)
    fig, ax = plt.subplots(figsize=(9,5),layout='constrained')
    for key, rows in groups.items():
        scan=lag_metrics(rows)
        ax.plot([r['response_shift_frames'] for r in scan],[r['rmse'] for r in scan],label='/'.join(key))
    ax.set(xlabel='Response shift k [frame]: F+1+k (fixed command support)',ylabel='RMSE [m/s2]',
           title='Descriptive shift scan; minimum is not proof of physical latency')
    ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.savefig(Path(output)/'lag_scan.png',dpi=160);plt.close(fig)


def analyze(evidence_root, output_root, plots=False):
    root, output = Path(evidence_root), Path(output_root)
    if output.exists() or output.resolve().is_relative_to(root.resolve()):
        raise ValueError('output must be new and outside original evidence directory')
    summaries, details = [], []
    for trial, folder in [('single','single_idm_once'),('fleet','fleet_idm_once'),('replay','snapshot_replay_once')]:
        receipt = read_json(root/folder/'controller_result.json')
        if receipt.get('successful') is not True:
            raise ValueError('source run did not complete')
        summary, rows = analyze_run(receipt['run_dir'], trial)
        summaries.append(summary);details.extend(rows)
    summary = dict(schema_version=1, offline_only=True, response_comparison='a_api(F+1) - raw IDM a(F)',
        controller_internal_provenance='deterministic offline replay from zero PID/filter state, output validated; not historically measured',
        aggregate_error=errors([r['api_a'] for r in details],[r['raw_idm_a'] for r in details]),
        total_rows=len(details), runs=summaries, route_audit=None, physical_bounds=None)
    output.mkdir(parents=True, exist_ok=False)
    (output/'fidelity_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False),encoding='utf-8')
    write_csv(output/'fidelity_frames.csv',details)
    if plots:
        plot_fidelity(output,details)
    return summary
