"""Local OS ownership/health checks. This module never imports CARLA."""
import ctypes
import json
import os
from pathlib import Path
import socket
import subprocess
import sys


def process_alive(pid):
    if not isinstance(pid,int) or pid<=0:
        return False
    if sys.platform == "win32":
        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        kernel.OpenProcess.restype = ctypes.c_void_p
        kernel.CloseHandle.argtypes = [ctypes.c_void_p]
        kernel.GetExitCodeProcess.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong)]
        handle = kernel.OpenProcess(0x1000, False, pid)
        if not handle:
            return False
        try:
            code = ctypes.c_ulong()
            return bool(kernel.GetExitCodeProcess(handle, ctypes.byref(code))) and code.value == 259
        finally:
            kernel.CloseHandle(handle)
    try:
        os.kill(pid, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def listener_pids(port):
    """Fail closed if OS inventory is unavailable; no connection to the port."""
    if sys.platform == "win32":
        script = (f"@(Get-NetTCPConnection -State Listen -ErrorAction Stop | "
                  f"Where-Object LocalPort -eq {int(port)} | "
                  "Select-Object -ExpandProperty OwningProcess -Unique) | ConvertTo-Json -Compress")
        result = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", script],
                                check=True, capture_output=True, text=True, timeout=10,
                                creationflags=subprocess.CREATE_NO_WINDOW)
        data = json.loads(result.stdout) if result.stdout.strip() else []
        return set(data if isinstance(data, list) else [data])
    if sys.platform.startswith("linux"):
        inodes = set()
        for name in ("tcp", "tcp6"):
            for line in Path(f"/proc/net/{name}").read_text().splitlines()[1:]:
                fields = line.split()
                if int(fields[1].split(":")[-1], 16) == port and fields[3] == "0A":
                    inodes.add(fields[9])
        owners, resolved = set(), set()
        for process in Path("/proc").iterdir():
            if not process.name.isdigit():
                continue
            try:
                for fd in (process / "fd").iterdir():
                    try:
                        target = os.readlink(fd)
                    except FileNotFoundError:
                        continue
                    if target.startswith("socket:[") and target[8:-1] in inodes:
                        owners.add(int(process.name))
                        resolved.add(target[8:-1])
            except (PermissionError, FileNotFoundError):
                continue
        if inodes - resolved:
            raise RuntimeError("listener owner could not be inspected")
        return owners
    raise RuntimeError("TM ownership inspection supports Windows/Linux only")


class PortOwnership:
    def __init__(self, port, inspect=listener_pids):
        self.port, self.inspect = port, inspect

    def require_unused(self):
        if self.inspect(self.port):
            raise ValueError("TM port already has an owner; do not acquire Remote TM")
        with socket.socket() as probe:
            if sys.platform == "win32":
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            probe.bind(("0.0.0.0", self.port))

    def require_self(self):
        if self.inspect(self.port) != {os.getpid()}:
            raise ValueError("TM listener is not exclusively owned by this worker; no TM mutation")


class ServerUnavailable(Exception):
    """Not a RuntimeError: collector read-error fallbacks must not swallow this."""


class HealthGate:
    def __init__(self, server_pid, supervisor_pid, marker):
        self.server_pid, self.supervisor_pid, self.marker = server_pid, supervisor_pid, Path(marker)
        self.poisoned = False

    def check(self):
        if (self.poisoned or self.marker.exists() or not process_alive(self.server_pid)
                or not process_alive(self.supervisor_pid)):
            self.poisoned = True
            raise ServerUnavailable("server/supervisor unavailable; all further CARLA calls prohibited")

    def call(self, function, *args, **kwargs):
        self.check()
        try:
            return function(*args, **kwargs)
        except (RuntimeError, OSError, ConnectionError, TimeoutError) as exc:
            self.poisoned = True
            raise ServerUnavailable(f"CARLA runtime/RPC failure, no retry or cleanup RPC: {exc}") from exc
