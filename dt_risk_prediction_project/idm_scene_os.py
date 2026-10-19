"""Read-only Windows PID/TCP observations; no socket connection or port binding."""
from datetime import datetime, timezone
import ipaddress
import json
from pathlib import Path
import subprocess
import sys


def instant(value):
    d = datetime.fromisoformat(value.replace('Z', '+00:00'))
    if d.tzinfo is None:
        raise ValueError('UTC offset required for server start time')
    return d.astimezone(timezone.utc)


def validate_target(target):
    required = ('host','port','server_pid','server_started_utc','tm_port','session_reference')
    if any(target.get(k) is None for k in required):
        raise ValueError('explicit host, port, PID, start time, TM port and session reference required')
    host = target['host']
    if host != 'localhost' and not ipaddress.ip_address(host).is_loopback:
        raise ValueError('only explicitly specified local endpoints support OS PID verification')
    for key in ('port','tm_port'):
        if type(target[key]) is not int or not 1 <= target[key] <= 65535:
            raise ValueError('invalid '+key)
    if target['port'] == target['tm_port'] or type(target['server_pid']) is not int or target['server_pid'] <= 0:
        raise ValueError('invalid server PID/port pair')
    instant(target['server_started_utc'])
    review = json.loads(Path(target['session_reference']).read_text(encoding='utf-8'))
    for k in required[:-1]:
        if review.get(k) != target[k]:
            raise ValueError('session declaration does not match '+k)
    if (review.get('dedicated_session_declared') is not True
            or not review.get('operator') or not review.get('reviewed_at')):
        raise ValueError('explicit dedicated-session operator declaration required')
    instant(review['reviewed_at'])
    return review


def inspect_os(target):
    if sys.platform != 'win32':
        raise ValueError('live PID/start-time audit currently supports Windows only')
    pid, port, tm = (int(target[k]) for k in ('server_pid','port','tm_port'))
    script = (f'$p=Get-CimInstance Win32_Process -Filter "ProcessId={pid}" -ErrorAction Stop; '
              'if (!$p) { throw "server PID absent" }; '
              '$started=([DateTime]$p.CreationDate).ToUniversalTime().ToString("o"); '
              f'$tcp=@(Get-NetTCPConnection -ErrorAction Stop | Where-Object {{ $_.LocalPort -in @({port},{tm}) -or $_.RemotePort -eq {port} }} | '
              'Select-Object LocalAddress,LocalPort,RemoteAddress,RemotePort,State,OwningProcess); '
              '@{pid=[int]$p.ProcessId;name=$p.Name;started_utc=$started;tcp=$tcp} | ConvertTo-Json -Depth 5 -Compress')
    result = subprocess.run(['powershell','-NoProfile','-NonInteractive','-Command',script],
                            capture_output=True, text=True, check=True, timeout=15,
                            creationflags=subprocess.CREATE_NO_WINDOW)
    return json.loads(result.stdout)


def verify_os(target, observed):
    if observed['pid'] != target['server_pid'] or abs((instant(observed['started_utc'])-
            instant(target['server_started_utc'])).total_seconds()) > .001:
        raise ValueError('server PID/start-time mismatch')
    if 'carla' not in observed['name'].lower():
        raise ValueError('PID is not a CARLA process')
    rows = observed['tcp']
    owners = {r['OwningProcess'] for r in rows if str(r['State']) in ('Listen','2')
              and r['LocalPort'] == target['port']}
    if owners != {target['server_pid']}:
        raise ValueError('server listener ownership mismatch')
    tm = [r for r in rows if str(r['State']) in ('Listen','2') and r['LocalPort'] == target['tm_port']]
    if tm:
        raise ValueError('existing TM listener; dedicated-session precondition not met')
    external = [r for r in rows if str(r['State']) in ('Established','5')
                and (r['LocalPort'] == target['port'] or r['RemotePort'] == target['port'])]
    if external:
        raise ValueError('existing client connection; dedicated-session precondition not met')
    return dict(status='pass', observations=observed, external_clients_absent='unknown',
                reason='OS sample and operator declaration cannot prove absence of all clients/tick owners')
