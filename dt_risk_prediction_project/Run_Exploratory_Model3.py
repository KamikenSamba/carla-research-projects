"""Bounded exploratory runs, separate from strict route certification. Offline default."""
import argparse
import csv
import ctypes
from ctypes import wintypes as W
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys

from idm_low_speed import config_for,ExploratoryHarness
from idm_harness_host import HealthGate,ServerUnavailable,listener_pids
from Run_IDM_Following_Experiment import supervise


def native_identity(pid):
    if sys.platform!='win32':raise ValueError('native identity adapter required for this platform')
    k=ctypes.WinDLL('kernel32',use_last_error=True)
    k.OpenProcess.argtypes=[W.DWORD,W.BOOL,W.DWORD];k.OpenProcess.restype=W.HANDLE
    k.CloseHandle.argtypes=[W.HANDLE]
    k.GetProcessTimes.argtypes=[W.HANDLE]+[ctypes.POINTER(W.FILETIME)]*4
    k.QueryFullProcessImageNameW.argtypes=[W.HANDLE,W.DWORD,W.LPWSTR,ctypes.POINTER(W.DWORD)]
    h=k.OpenProcess(0x1000,False,pid)
    if not h:raise OSError('cannot open expected server process')
    try:
        times=[W.FILETIME() for _ in range(4)];buf=ctypes.create_unicode_buffer(32768);n=W.DWORD(len(buf))
        if not k.GetProcessTimes(h,*[ctypes.byref(t) for t in times]) or not k.QueryFullProcessImageNameW(h,0,buf,ctypes.byref(n)):
            raise OSError('server identity unavailable')
        return dict(pid=pid,creation_filetime=(times[0].dwHighDateTime<<32)|times[0].dwLowDateTime,path=buf.value)
    finally:k.CloseHandle(h)


def same_identity(a,b):
    return (a['pid']==b['pid'] and a['creation_filetime']==b['creation_filetime']
        and os.path.normcase(os.path.realpath(a['path']))==os.path.normcase(os.path.realpath(b['path'])))


def session_valid(session,pid,port):
    if (session.get('exploratory_low_speed_authorized') is not True or session.get('dedicated_session_declared') is not True
        or not session.get('operator') or not session.get('reviewed_at') or not session.get('approval_reference')
        or session.get('host')!='127.0.0.1' or session.get('port')!=port or session.get('identity',{}).get('pid')!=pid):
        raise ValueError('fresh authorized research session attestation required')


class IdentityGate(HealthGate):
    def __init__(self,pid,parent,marker,witness):
        super().__init__(pid,parent,marker);self.witness=witness
    def check(self):
        super().check()
        try:matches=same_identity(native_identity(self.server_pid),self.witness)
        except (OSError,ValueError):matches=False
        if not matches:
            self.poisoned=True
            raise ServerUnavailable('server native identity changed; no further RPC')


def verify_completed(directory,result,c):
    """Offline frame/control replay; a completed label alone is not evidence."""
    if (result.get('status')!='completed' or not result.get('cleanup',{}).get('complete')
        or result.get('collision_events')!=0 or result.get('route_audit') is not None
        or result.get('physical_bounds') is not None):raise ValueError('incomplete exploratory result')
    with (directory/'calibration_observations.csv').open(encoding='utf-8',newline='') as f:
        rows=list(csv.DictReader(f))
    raw=[json.loads(s) for s in (directory/'calibration_raw.jsonl').read_text(encoding='utf-8').splitlines()]
    if not rows or len(rows)!=len(raw) or len(rows)!=result['motion_samples']:
        raise ValueError('raw/CSV/result sample mismatch')
    frames=[int(r['frame']) for r in rows]
    if frames!=list(range(frames[0],frames[0]+len(frames))) or frames[-1]!=result['final_frame']:
        raise ValueError('noncontiguous or wrong final frame')
    for r,envelope in zip(rows,raw):
        if (int(r['frame'])!=envelope['carla_frame'] or int(r['command_based_on_frame'])!=int(r['frame'])-1
            or not envelope['frame_validated'] or envelope['acquisition_status']!='complete'):
            raise ValueError('observation/command frame mismatch')
        observed=envelope['rows'][0]
        for key in ('throttle','brake','steer'):
            if float(r[key])!=observed['control'][key]:raise ValueError('control log mismatch')
    collision=json.loads((directory/'collision_summary.json').read_text(encoding='utf-8'))
    if collision['events']:raise ValueError('observed collision')
    if c.trial=='calibration' and result.get('calibration_valid') is not True:
        raise ValueError('incomplete brake calibration')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--server-pid',type=int);p.add_argument('--port',type=int,default=2100)
    p.add_argument('--output-root',type=Path,default=Path('runs'))
    p.add_argument('--wall-timeout',type=float,default=120)
    p.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--stage',default='exploratory_low_speed',choices=['exploratory_low_speed'])
    p.add_argument('--run-dir',type=Path,help=argparse.SUPPRESS)
    p.add_argument('--supervisor-pid',type=int,help=argparse.SUPPRESS)
    a=p.parse_args(argv);a.stage1_result=None
    payload=json.loads(a.config.read_text(encoding='utf-8-sig'));c=config_for(payload['trial'])
    if payload.get('scenario')!=asdict(c):raise ValueError('scenario differs from fixed exploratory protocol')
    if not a.execute and not a.worker:
        print(json.dumps(dict(status='offline_plan_only',scenario=asdict(c),strict_scene_audit='BLOCKED',
            route_audit=None,physical_bounds=None,requires_authorized_session=True),indent=2));return 0
    if a.wall_timeout!=120:raise ValueError('fixed 120s worker supervisor limit')
    session=payload['session'];session_valid(session,a.server_pid,a.port)
    if not same_identity(native_identity(a.server_pid),session['identity']):raise ValueError('server identity mismatch')
    if a.worker:
        if not a.run_dir or not a.supervisor_pid:raise ValueError('worker requires supervisor')
        gate=IdentityGate(a.server_pid,a.supervisor_pid,a.run_dir/'halt_rpc',session['identity']);gate.check()
        if listener_pids(a.port)!={a.server_pid}:raise ValueError('listener identity mismatch')
        import carla
        client=carla.Client('127.0.0.1',a.port);client.set_timeout(5.)
        result=ExploratoryHarness(client,carla,c,a.run_dir,gate).run()
        # Do not let a binding destructor attempt RPC after server failure.
        sys.stdout.flush();sys.stderr.flush();os._exit(result['exit_code'])
    return supervise(a,c,entrypoint=Path(__file__),source_files=(
        'Run_Exploratory_Model3.py','idm_low_speed.py','idm_reverse_motion.py','idm_brake_harness.py','idm_brake_config.py',
        'idm_brake_metrics.py','idm_experiment_harness.py','idm_harness_host.py'),
        verify_completed=lambda directory,result:verify_completed(directory,result,c))


if __name__=='__main__':raise SystemExit(main())
