"""Python 3.11 client for the Python 3.10 Material Supply ROS process."""

from __future__ import annotations

import json
import os
from pathlib import Path
import select
import shlex
import socket
import subprocess
import sys
import time
from typing import Any


def completion_payload(result: dict[str, Any]) -> dict[str, Any]:
    """Validate and encode the Isaac provider completion fact."""
    required = (
        "delivery_id",
        "material_type",
        "requested_quantity",
        "supplied_quantity",
        "completed",
        "valid",
        "failure_reason",
    )
    missing = [key for key in required if key not in result]
    if missing:
        raise ValueError(f"completion result missing fields: {missing}")
    return {
        "op": "transfer_completion",
        "result": {key: result[key] for key in required},
    }


class MaterialSupplySidecarClient:
    """Small nonblocking JSON-line IPC client owned by the Isaac process."""

    def __init__(self, sock: socket.socket, process: Any = None):
        self._socket = sock
        self._socket.setblocking(False)
        self._process = process
        self._receive_buffer = bytearray()
        self._outgoing: list[memoryview] = []
        self._available = True
        self._closed = False

    @classmethod
    def from_socket(cls, sock: socket.socket, process: Any = None):
        """Construct from an already connected socket (also used by unit tests)."""
        return cls(sock, process)

    @classmethod
    def launch(cls, project_root: Path, timeout_seconds: float = 10.0):
        root = Path(project_root).expanduser().resolve()
        cell_root = root / "infra/isaac_sim/scripts/m0609_cell"
        child_script = cell_root / "material_supply_ros_sidecar.py"
        if not child_script.is_file():
            raise RuntimeError(
                f"material supply ROS sidecar is missing: {child_script}"
            )
        parent, child = socket.socketpair()
        env = os.environ.copy()
        # Do not leak Kit's Python 3.11 module search path into system Python.
        env["PYTHONPATH"] = ""
        env.pop("PYTHONHOME", None)
        env["SFTWIN_MATERIAL_SIDECAR_FD"] = str(child.fileno())
        env["SFTWIN_PROJECT_ROOT"] = str(root)
        env["PYTHONUNBUFFERED"] = "1"
        python = "/usr/bin/python3.10"
        if not Path(python).is_file():
            python = "python3.10"
        command = (
            "source /opt/ros/humble/setup.bash && "
            'source "$SFTWIN_PROJECT_ROOT/ros2_ws/install/setup.bash" && '
            f"exec {shlex.quote(python)} {shlex.quote(str(child_script))} "
            '"$SFTWIN_MATERIAL_SIDECAR_FD"'
        )
        try:
            process = subprocess.Popen(
                ["/bin/bash", "-c", command],
                env=env,
                pass_fds=(child.fileno(),),
                close_fds=True,
                stdout=subprocess.DEVNULL,
                stderr=sys.stderr,
            )
        except Exception:
            parent.close()
            child.close()
            raise
        child.close()
        parent.setblocking(True)
        parent.settimeout(timeout_seconds)
        client = cls(parent, process)
        try:
            ready = client._read_startup_line(timeout_seconds)
            if ready.get("op") != "ready" or ready.get("ready") is not True:
                raise RuntimeError(ready.get("error", "sidecar failed to start"))
        except Exception:
            client.close()
            raise
        parent.settimeout(0.0)
        return client

    def _read_startup_line(self, timeout_seconds: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_seconds
        while b"\n" not in self._receive_buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("material supply ROS sidecar startup timed out")
            readable = select.select([self._socket], [], [], remaining)[0]
            if not readable:
                raise TimeoutError("material supply ROS sidecar startup timed out")
            try:
                chunk = self._socket.recv(4096)
            except BlockingIOError:
                continue
            if not chunk:
                raise RuntimeError(
                    "material supply ROS sidecar disconnected at startup"
                )
            self._receive_buffer.extend(chunk)
        line, _, remainder = self._receive_buffer.partition(b"\n")
        self._receive_buffer = bytearray(remainder)
        return json.loads(line)

    @property
    def available(self) -> bool:
        if self._closed or not self._available:
            return False
        if self._process is not None and self._process.poll() is not None:
            self._available = False
        return self._available

    def poll_messages(self) -> list[dict[str, Any]]:
        if not self.available:
            return []
        messages = []
        while True:
            try:
                chunk = self._socket.recv(65536)
            except BlockingIOError:
                break
            except OSError:
                self._available = False
                break
            if not chunk:
                self._available = False
                break
            self._receive_buffer.extend(chunk)
        while b"\n" in self._receive_buffer:
            line, _, remainder = self._receive_buffer.partition(b"\n")
            self._receive_buffer = bytearray(remainder)
            try:
                message = json.loads(line)
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._available = False
                return []
            if not isinstance(message, dict):
                self._available = False
                return []
            messages.append(message)
        return messages

    def send_completion(self, result: dict[str, Any]) -> None:
        if not self.available:
            return
        packet = json.dumps(completion_payload(result), separators=(",", ":"))
        self._outgoing.append(memoryview((packet + "\n").encode("utf-8")))
        self.flush_outgoing()

    def flush_outgoing(self) -> None:
        if not self.available:
            return
        while self._outgoing:
            try:
                sent = self._socket.send(self._outgoing[0])
            except BlockingIOError:
                return
            except OSError:
                self._available = False
                return
            if sent <= 0:
                self._available = False
                return
            self._outgoing[0] = self._outgoing[0][sent:]
            if not self._outgoing[0]:
                self._outgoing.pop(0)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._socket.close()
        except OSError:
            pass
        process = self._process
        if process is not None and process.poll() is None:
            try:
                process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    try:
                        process.kill()
                    except ProcessLookupError:
                        pass
                    process.wait()


def simulation_time_ns() -> int:
    """Read Isaac timeline time in the same simulation-time domain as /clock."""
    import omni.timeline

    seconds = omni.timeline.get_timeline_interface().get_current_time()
    return int(seconds * 1_000_000_000)
