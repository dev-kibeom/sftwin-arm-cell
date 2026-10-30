"""Owned process-group lifecycle for live acceptance launch probes."""

import os
from pathlib import Path
import signal
import subprocess
import time


def _proc_metadata(pid):
    try:
        stat = (Path("/proc") / str(pid) / "stat").read_text()
        command_end = stat.rfind(") ")
        fields = stat[command_end + 2:].split()
        return {
            "state": fields[0],
            "ppid": int(fields[1]),
            "pgid": int(fields[2]),
            "sid": int(fields[3]),
        }
    except (FileNotFoundError, PermissionError, ValueError):
        return None


def _processes():
    for entry in Path("/proc").glob("[0-9]*"):
        metadata = _proc_metadata(int(entry.name))
        if metadata is not None:
            yield int(entry.name), metadata


def _descendants(root_pid):
    process_map = dict(_processes())
    descendants = {root_pid}
    changed = True
    while changed:
        changed = False
        for pid, metadata in process_map.items():
            if metadata["ppid"] in descendants and pid not in descendants:
                descendants.add(pid)
                changed = True
    return descendants


class ManagedLaunch:
    """Start and teardown only the launch process tree owned by this probe."""

    def __init__(self, command, *, cwd=None, env=None, output=None):
        self.command = list(command)
        self.cwd = cwd
        self.env = env
        self.output = output if output is not None else subprocess.DEVNULL
        self.process = None
        self.process_group_id = None
        self.session_id = None

    def __enter__(self):
        self.process = subprocess.Popen(
            self.command,
            cwd=self.cwd,
            env=self.env,
            stdout=self.output,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        metadata = _proc_metadata(self.process.pid)
        if (
            metadata is None
            or metadata["pgid"] != self.process.pid
            or metadata["sid"] != self.process.pid
        ):
            self.process.terminate()
            self.process.wait()
            raise RuntimeError(
                "managed launch did not receive an isolated process session"
            )
        self.process_group_id = metadata["pgid"]
        self.session_id = metadata["sid"]
        return self

    def _owned_pids(self):
        if self.process is None:
            return set()
        process_group = self.process_group_id
        owned = _descendants(self.process.pid)
        for pid, metadata in _processes():
            if metadata["pgid"] == process_group or metadata["sid"] == process_group:
                owned.add(pid)
        return {
            pid
            for pid in owned
            if (metadata := _proc_metadata(pid)) is not None
            and metadata["state"] != "Z"
        }

    @staticmethod
    def _signal_pids(pids, sig):
        for pid in sorted(pids, reverse=True):
            if pid == os.getpid():
                continue
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass

    def _signal_owned_group(self, sig):
        if self.process is None:
            return
        try:
            os.killpg(self.process_group_id, sig)
        except ProcessLookupError:
            pass
        self._signal_pids(self._owned_pids(), sig)

    def _wait_for_exit(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.process.wait()
            if self.process.poll() is not None and not self._owned_pids():
                return True
            time.sleep(0.05)
        if self.process.poll() is not None:
            self.process.wait()
        return self.process.poll() is not None and not self._owned_pids()

    def terminate(self, timeout=5.0):
        if self.process is None:
            return
        if self.process.poll() is None:
            self._signal_owned_group(signal.SIGINT)
        if not self._wait_for_exit(timeout):
            self._signal_owned_group(signal.SIGTERM)
        if not self._wait_for_exit(timeout):
            self._signal_owned_group(signal.SIGKILL)
        if not self._wait_for_exit(timeout):
            raise RuntimeError(
                f"owned launch process tree did not exit: pids={sorted(self._owned_pids())}"
            )

    def __exit__(self, exc_type, exc_value, traceback):
        self.terminate()
        return False
