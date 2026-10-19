"""Validate offline by default; --execute starts one supervised CARLA worker."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from idm_harness_config import load_scenario
from idm_harness_host import HealthGate, PortOwnership, listener_pids, process_alive
from idm_experiment_harness import Harness, save


def source_metadata():
    root=Path(__file__).resolve().parent
    payload=dict(source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                 [root/name for name in ('Run_IDM_Following_Experiment.py','idm_experiment_harness.py',
                  'idm_harness_config.py','idm_harness_host.py','idm_context.py','idm_normal_stage1.py')]})
    try:
        payload['source_commit']=subprocess.run(['git','rev-parse','HEAD'],cwd=root,
            check=True,capture_output=True,text=True,timeout=5).stdout.strip()
        payload['source_dirty']=bool(subprocess.run(['git','status','--porcelain'],cwd=root,
            check=True,capture_output=True,text=True,timeout=5).stdout.strip())
    except (OSError,subprocess.SubprocessError):
        payload.update(source_commit=None,source_dirty=None)
    return payload


def stage_gate(stage, scenario, stage1_path):
    if stage != "stage2":
        return
    if stage1_path is None:
        raise ValueError("stage2 requires --stage1-result from a completed real stage1")
    result = json.loads(Path(stage1_path).read_text(encoding="utf-8"))
    if (result.get("status") != "completed" or result.get("stage") != "stage1"
            or result.get("config_sha256") != scenario.fingerprint or not result.get("cleanup",{}).get("complete")
            or result.get("worker_exit_code") != 0 or not result.get("supervised")):
        raise ValueError("stage1 result is incomplete, incompatible or not supervised")


def classify_exit(code, result, server_lost=False):
    if server_lost:
        return "server_unavailable", 4
    if code < 0 or code > 255:
        return "native_abnormal_exit", 3
    expected = {0:"completed",1:"exception",2:"safe_early_stop",4:"server_unavailable",5:"cleanup_incomplete"}
    if result is None or result.get("exit_code") != code or expected.get(code) != result.get("status"):
        return "abnormal_exit", 1
    return result['status'], code


def supervise(args, scenario):
    if hasattr(scenario,'require_executable'):
        scenario.require_executable(args.stage)
    stage_gate(args.stage, scenario, args.stage1_result)
    if not args.server_pid or not process_alive(args.server_pid):
        raise ValueError("--execute requires an existing live --server-pid; never starts a server")
    if listener_pids(args.port) != {args.server_pid}:
        raise ValueError("CARLA endpoint listener does not match --server-pid")
    root = args.output_root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    # Exclusive cooperative lease, including across different TM ports.
    lock = Path(tempfile.gettempdir()) / f"carla-idm-harness-{args.port}.lock"
    with lock.open("x",encoding="utf-8") as lease:
        lease.write(str(os.getpid())); lease.flush()
        try:
            directory = Path(tempfile.mkdtemp(prefix="idm_harness_",dir=root))
            marker = directory/'halt_rpc'
            save(directory/'launch.json',dict(stage=args.stage,server_pid=args.server_pid,
                 supervisor_pid=os.getpid(),config_sha256=scenario.fingerprint,
                 config_file_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                 stage1_result=str(args.stage1_result) if args.stage1_result else None,
                 **source_metadata()))
            command = [sys.executable,'-X','utf8','-X','faulthandler',str(Path(__file__).resolve()),
                       '--worker','--config',str(args.config.resolve()),'--stage',args.stage,
                       '--run-dir',str(directory),'--server-pid',str(args.server_pid),
                       '--supervisor-pid',str(os.getpid()),'--port',str(args.port)]
            if args.stage1_result:
                command += ['--stage1-result',str(args.stage1_result.resolve())]
            flags = subprocess.CREATE_NO_WINDOW if sys.platform=='win32' else 0
            with (directory/'worker_stdout.txt').open('w',encoding='utf-8') as stdout, \
                 (directory/'worker_stderr.txt').open('w',encoding='utf-8') as stderr:
                worker = subprocess.Popen(command,stdout=stdout,stderr=stderr,creationflags=flags)
                save(directory/'worker_started.json',dict(pid=worker.pid,command=command))
                began=time.monotonic(); lost=False; timed_out=False
                try:
                    while worker.poll() is None:
                        if not process_alive(args.server_pid):
                            lost=True
                            marker.touch(exist_ok=True)
                        if time.monotonic()-began>args.wall_timeout:
                            timed_out=True
                            marker.touch(exist_ok=True)
                        if (lost or timed_out) and time.monotonic()-began>args.wall_timeout+5:
                            worker.kill()  # only this worker; supervisor never imports CARLA
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    marker.touch(exist_ok=True)
                    try:
                        worker.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        worker.kill()
                worker.wait()
            path=directory/'worker_result.json'
            try:
                result=json.loads(path.read_text()) if path.exists() else None
            except (OSError,ValueError):
                result=None
            status,exit_code=classify_exit(worker.returncode,result,lost)
            final=dict(result or {},status=status,exit_code=exit_code,worker_exit_code=worker.returncode,
                       stage=args.stage,config_sha256=scenario.fingerprint,supervised=True,
                       server_alive=process_alive(args.server_pid),timed_out=timed_out,
                       real_physics_verified=False)
            stderr=(directory/'worker_stderr.txt').read_text(encoding='utf-8',errors='replace')
            fatal=any(word in stderr.lower() for word in ('fatal python error','access violation','terminate called'))
            final['fatal_stderr']=fatal
            if fatal:
                final.update(status='native_abnormal_exit',exit_code=3,cleanup_verified=False)
            if timed_out:
                final.update(status='worker_timeout',exit_code=6)
            if status=='native_abnormal_exit':
                final.update(cleanup_verified=False,reason='native worker exit; supervisor sends no CARLA RPC')
            save(directory/'result.json',final)
            print(json.dumps(dict(run_dir=str(directory),**final),indent=2))
            return final['exit_code']
        finally:
            # On Windows close our lease before unlinking. Never remove someone else's lock.
            lease.close()
            lock.unlink()


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path(__file__).parent/'configs/idm_following_harness.json')
    parser.add_argument('--stage',choices=['stage1','stage2'],default='stage1')
    parser.add_argument('--stage1-result',type=Path)
    parser.add_argument('--execute',action='store_true',help='Explicit real-run opt-in; validation only by default')
    parser.add_argument('--server-pid',type=int)
    parser.add_argument('--port',type=int,default=2000)
    parser.add_argument('--output-root',type=Path,default=Path('runs'))
    parser.add_argument('--wall-timeout',type=float,default=120)
    parser.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--run-dir',type=Path,help=argparse.SUPPRESS)
    parser.add_argument('--supervisor-pid',type=int,help=argparse.SUPPRESS)
    args=parser.parse_args(argv)
    try:
        scenario=load_scenario(args.config)
        if not 1<=args.port<=65535 or not math.isfinite(args.wall_timeout) or args.wall_timeout<=0:
            raise ValueError('invalid server port or wall timeout')
        if (args.worker or args.execute) and hasattr(scenario,'require_executable'):
            scenario.require_executable(args.stage)
        if args.worker:
            if not args.run_dir or not args.supervisor_pid or not args.server_pid:
                raise ValueError('worker must be launched by supervisor')
            stage_gate(args.stage,scenario,args.stage1_result)
            gate=HealthGate(args.server_pid,args.supervisor_pid,args.run_dir/'halt_rpc')
            gate.check()
            if listener_pids(args.port)!={args.server_pid}:
                raise ValueError('server listener changed')
            import carla  # only explicit real worker, never validation/tests/supervisor
            client=carla.Client('127.0.0.1',args.port); client.set_timeout(5)
            harness=Harness(client,carla,scenario,args.stage,args.run_dir,gate,PortOwnership(scenario.tm_port))
            result=harness.run()
            if gate.poisoned:
                # Bypass C++ destructors: no implicit teardown RPC after server/runtime failure.
                sys.stdout.flush();sys.stderr.flush();os._exit(result['exit_code'])
            return result['exit_code']
        if not args.execute:
            print(json.dumps(dict(status='validated_offline_no_connection',stage=args.stage,
                 scenario=asdict_config(scenario),config_sha256=scenario.fingerprint,
                 clearance_budget_m=scenario.clearance_budget,
                 **(dict(executable_route=scenario.route_audit is not None,
                         distance_budget=scenario.distance_report(),initialization=scenario.initialization)
                    if hasattr(scenario,'distance_report') else {})),indent=2))
            return 0
        return supervise(args,scenario)
    except (ValueError,OSError,RuntimeError) as exc:
        parser.error(str(exc))


def asdict_config(scenario):
    from dataclasses import asdict
    return asdict(scenario)


if __name__=='__main__':
    raise SystemExit(main())
