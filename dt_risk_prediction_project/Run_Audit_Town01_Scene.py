"""Independent Town01 scene auditor. Default: offline, no CARLA import/connection."""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from idm_scene_audit import ALLOWLIST, UNKNOWN, Evidence, SceneAudit, utc, hash_manifest
from idm_scene_reference import read_policy, read_reference
from idm_scene_os import validate_target, inspect_os, verify_os


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default=str(Path(__file__).parent/'configs/brake_calibration_town01_v2.json'))
    p.add_argument('--reference-opendrive')
    p.add_argument('--reference-geometry')
    p.add_argument('--connect-readonly',action='store_true',help='Requires separate explicit operator approval; never starts/changes a server')
    p.add_argument('--host')
    p.add_argument('--port',type=int)
    p.add_argument('--server-pid',type=int)
    p.add_argument('--server-started-utc')
    p.add_argument('--tm-port',type=int)
    p.add_argument('--session-reference',help='JSON declaration of matching endpoint/PID/start/operator/dedicated session')
    p.add_argument('--output-root',default=tempfile.gettempdir(),help='Existing directory outside repository; a unique child is created')
    return p


def target_of(args):
    return {k:getattr(args,k) for k in ('host','port','server_pid','server_started_utc','tm_port','session_reference')}


def output_directory(root):
    root = Path(root).resolve(strict=True)
    repo = Path(__file__).resolve().parents[1]
    if root==repo or repo in root.parents or any((p/'.git').exists() for p in (root,*root.parents)):
        raise ValueError('evidence output must be outside repository')
    return Path(tempfile.mkdtemp(prefix='town01_scene_audit_',dir=root))


def source_commit():
    return subprocess.check_output(['git','rev-parse','HEAD'],cwd=Path(__file__).parent,text=True).strip()


def prepare(args,e):
    c = read_policy(args.config)
    target = target_of(args)
    declaration = None
    if any(v is not None for v in target.values()) or args.connect_readonly:
        declaration = validate_target(target)
    reference = None
    if args.reference_opendrive or args.reference_geometry:
        reference = read_reference(c,args.reference_opendrive,args.reference_geometry)
        e.write('offline_reference',reference)
    if args.connect_readonly and reference is None:
        raise ValueError('validated independent offline reference required before any connection')
    plan = dict(status='unknown',mode='offline',policy=asdict(c),
        policy_sha256=c.policy_sha256,config_fingerprint=c.fingerprint,
        config_file_sha256=hashlib.sha256(Path(args.config).read_bytes()).hexdigest(),
        target=target,session_operator_declaration=declaration,
        reference_status='pass' if reference else 'unknown',
        reference_reason=None if reference else 'reference XML/196-coordinate file not supplied',
        api_allowlist=ALLOWLIST,unconfirmed=UNKNOWN,connected=False)
    e.write('offline_plan',plan)
    e.write('environment',plan)
    return c,reference,target


def supervise(payload_path,directory, *, command=None, timeout=120):
    """Only child-process/file operations. Native failures never cause a recovery RPC."""
    started = utc()
    command = command or [sys.executable,'-X','utf8','-c',
                           'from Run_Audit_Town01_Scene import worker; worker()',str(payload_path)]
    with (directory/'worker_stdout.log').open('wb') as out, (directory/'worker_stderr.log').open('wb') as err:
        proc = subprocess.Popen(command,cwd=Path(__file__).parent,stdout=out,stderr=err)
        timed_out = False
        try:
            code = proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            # This PID is the audit child, never the CARLA server.
            proc.kill()
            code = proc.wait(timeout=10)
    stderr = (directory/'worker_stderr.log').read_text(encoding='utf-8',errors='replace')
    native = not timed_out and (code<0 or code>255 or any(s in stderr.lower() for s in ('fatal','segmentation fault','access violation')))
    result = dict(worker_pid=proc.pid,exit_code=code,timed_out=timed_out,
                  native_failure=native,stderr_nonempty=bool(stderr),recorded_utc=utc(),
                  started_utc=started,recovery_rpc_attempted=False)
    (directory/'supervisor_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    completed = (directory/'worker_completed.json').is_file()
    if code!=0 or native or timed_out or not completed:
        # Preserve every partial JSON and ledger; only invalidate the final assessment.
        path = directory/'audit_results.json'
        record = json.loads(path.read_text(encoding='utf-8'))
        previous = record['data']
        record['data'] = dict(status='unknown' if native or timed_out or not completed else
                             previous.get('status') if previous.get('status') in ('fail','unknown') else 'fail',
            reason='worker native failure' if native else 'worker timeout' if timed_out else 'worker nonzero exit' if code else 'worker incomplete',
            worker_assessment=previous,supervisor=result,route_audit=None,physical_bounds=None,execute_allowed=False)
        path.write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
        (directory/'report_ja.md').write_text('監査未完了: worker異常終了。部分証跡を保全。追加RPC・復旧操作なし。\n',encoding='utf-8')
    hash_manifest(directory)
    return 1 if code!=0 or native or timed_out or not completed else 0


def worker():
    """Private subprocess entry; repeat preconditions before the sole Client constructor."""
    payload = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
    args = argparse.Namespace(**payload['args'])
    e = Evidence(payload['directory'],payload['commit'],synthetic=payload.get('synthetic',False))
    try:
        if not args.connect_readonly:
            raise ValueError('explicit --connect-readonly required')
        c,ref,target = prepare(args,e)
        observed = inspect_os(target)
        e.write('server_status',dict(status='unknown',observed=observed))
        os_result = verify_os(target,observed)
        e.write('server_status',os_result)
        from idm_harness_host import HealthGate
        health = HealthGate(target['server_pid'],payload['supervisor_pid'],Path(payload['directory'])/'rpc_prohibited')
        health.check()
        import carla  # The only SDK import, after all offline and OS checks.
        constructor = dict(sequence=0,api='transport.Client',started_utc=utc(),status='requested',
                           host=target['host'],port=target['port'])
        e.write('api_call_ledger',[constructor])
        try:
            client = carla.Client(target['host'],target['port'])
        except BaseException:
            constructor.update(status='failed',ended_utc=utc())
            e.write('api_call_ledger',[constructor])
            raise
        constructor.update(status='returned',ended_utc=utc())
        e.write('api_call_ledger',[constructor])
        result = SceneAudit(client,carla,c,ref,e,health).run()
        code = 0 if result['status']=='pass' else 2
    except BaseException as exc:
        e.write('audit_results',dict(status='unknown',reason=type(exc).__name__+': '+str(exc),
                                   route_audit=None,physical_bounds=None,execute_allowed=False))
        e.report()
        code = 2
    (e.directory/'worker_completed.json').write_text(json.dumps(dict(exit_code=code,utc=utc())),encoding='utf-8')
    # Skip SDK client teardown after a poisoned RPC/native-risk path. OS reclaims this child.
    sys.stdout.flush();sys.stderr.flush()
    os._exit(code)


def main(argv=None):
    args = parser().parse_args(argv)
    directory = output_directory(args.output_root)
    e = Evidence(directory,source_commit())
    try:
        c,reference,target = prepare(args,e)
        e.write('audit_results',dict(status='pass',mode='offline',reason='offline plan validated; no connection made',
            reference_status='pass' if reference else 'unknown',route_audit=None,physical_bounds=None,
            execute_allowed=False,unknown=UNKNOWN))
        e.report()
        print(json.dumps(dict(directory=str(directory),mode='connect-readonly' if args.connect_readonly else 'offline',
                              reference_validated=reference is not None,api_allowlist=ALLOWLIST,unconfirmed=UNKNOWN)))
        if not args.connect_readonly:
            return 0
        # No CARLA calls in the supervisor, including on native child termination.
        payload = directory/'worker_input.json'
        payload.write_text(json.dumps(dict(args=vars(args),directory=str(directory),commit=e.meta['source_commit'],
                                           supervisor_pid=os.getpid())),encoding='utf-8')
        return supervise(payload,directory)
    except Exception as exc:
        e.write('audit_results',dict(status='fail',mode='offline',reason=str(exc),route_audit=None,
                                   physical_bounds=None,execute_allowed=False))
        e.report()
        print(str(exc),file=sys.stderr)
        return 2


if __name__=='__main__':
    raise SystemExit(main())
