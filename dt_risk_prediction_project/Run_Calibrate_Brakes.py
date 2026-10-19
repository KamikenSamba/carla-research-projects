"""Independent single-vehicle brake calibration; offline validation is the default."""
from dataclasses import asdict
import argparse
import json
import math
import os
from pathlib import Path
import sys

from idm_brake_config import load_config
from idm_brake_harness import BrakeHarness
from idm_brake_metrics import read_validated_run
from idm_harness_host import HealthGate,listener_pids
from Run_IDM_Following_Experiment import supervise

SOURCE_FILES=('Run_Calibrate_Brakes.py','idm_brake_config.py','idm_brake_harness.py','idm_brake_metrics.py')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path(__file__).parent/'configs/brake_calibration_v1.json')
    parser.add_argument('--execute',action='store_true',help='Explicit future real-run opt-in; requires new approval')
    parser.add_argument('--evaluate-run',type=Path,help='Read-only offline evaluation of a completed supervised run')
    parser.add_argument('--server-pid',type=int)
    parser.add_argument('--port',type=int,default=2000)
    parser.add_argument('--output-root',type=Path,default=Path('runs'))
    parser.add_argument('--wall-timeout',type=float,default=120)
    parser.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--stage',choices=['brake_calibration'],default='brake_calibration',help=argparse.SUPPRESS)
    parser.add_argument('--run-dir',type=Path,help=argparse.SUPPRESS)
    parser.add_argument('--supervisor-pid',type=int,help=argparse.SUPPRESS)
    args=parser.parse_args(argv);args.stage1_result=None
    try:
        if args.evaluate_run:
            if args.execute or args.worker:raise ValueError('evaluation cannot execute CARLA')
            print(json.dumps(read_validated_run(args.evaluate_run),indent=2,allow_nan=False));return 0
        c=load_config(args.config)
        if not 1<=args.port<=65535 or args.port==8000 or not math.isfinite(args.wall_timeout) or args.wall_timeout<=0:
            raise ValueError('invalid port or timeout')
        if not args.execute and not args.worker:
            print(json.dumps(dict(status='validated_offline_no_connection',scenario=asdict(c),
                config_sha256=c.fingerprint,policy_sha256=c.policy_sha256,
                executable_route=c.route_audit is not None,distance_budget=c.distance_report(),
                physical_bounds_qualified=False),indent=2));return 0
        c.require_executable('brake_calibration') # before OS/server/Client access
        if listener_pids(8000):raise ValueError('TM guard port 8000 already owned; dedicated session required')
        if args.worker:
            if not args.run_dir or not args.server_pid or not args.supervisor_pid:
                raise ValueError('worker must be launched by supervisor')
            gate=HealthGate(args.server_pid,args.supervisor_pid,args.run_dir/'halt_rpc');gate.check()
            if listener_pids(args.port)!={args.server_pid}:raise ValueError('server listener changed')
            import carla
            client=carla.Client('127.0.0.1',args.port);client.set_timeout(5)
            result=BrakeHarness(client,carla,c,args.run_dir,gate).run()
            if gate.poisoned:
                sys.stdout.flush();sys.stderr.flush();os._exit(result['exit_code'])
            return result['exit_code']
        return supervise(args,c,entrypoint=Path(__file__),source_files=SOURCE_FILES,
                         verify_completed=lambda directory,result:read_validated_run(directory,result_override=result))
    except (ValueError,OSError,RuntimeError,KeyError) as exc:
        parser.error(str(exc))


if __name__=='__main__':raise SystemExit(main())
