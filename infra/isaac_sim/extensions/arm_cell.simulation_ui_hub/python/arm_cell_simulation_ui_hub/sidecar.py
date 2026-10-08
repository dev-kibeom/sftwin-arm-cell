"""Python 3.10 ROS client process for Isaac Sim's Python 3.11 Hub UI."""

from __future__ import annotations

import base64
import json
import logging
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import threading
import time
import uuid
from typing import Any

from .scenario_ports import Owner, OwnerScenarioPorts
from .runtime import (
    DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS,
    VDA_MOCK_FAULT_SCENARIOS,
)


_OBSERVATIONS = (
    "safety",
    "motion",
    "amr",
    "handoff",
    "readiness",
    "packml",
    "execute_cycle",
)
_LOGGER = logging.getLogger(__name__)
_UI_POLL_INTERVAL_SECONDS = 0.05
_FAULT_RPC_TIMEOUT_SECONDS = 2.0


class IsaacRosSidecar:
    """UI-facing proxy; the ROS node remains an observer/request client."""

    def __init__(
        self,
        *,
        default_observation_freshness_seconds: float,
        default_annotation_source_tolerance_seconds: float = (
            DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS
        ),
        scenario_ports: OwnerScenarioPorts | None = None,
    ):
        root = Path(os.environ["SFTWIN_PROJECT_ROOT"]).expanduser().resolve()
        extension_python = (
            root / "infra/isaac_sim/extensions/arm_cell.simulation_ui_hub/python"
        )
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join(
            [str(extension_python), env.get("PYTHONPATH", "")]
        ).rstrip(os.pathsep)
        env["SFTWIN_HUB_OBSERVATION_FRESHNESS_SECONDS"] = str(
            default_observation_freshness_seconds
        )
        env["SFTWIN_HUB_ANNOTATION_SOURCE_TOLERANCE_SECONDS"] = str(
            default_annotation_source_tolerance_seconds
        )
        python = "/usr/bin/python3.10"
        if not Path(python).is_file():
            python = "python3.10"
        bootstrap = (
            "source /opt/ros/humble/setup.bash && "
            'source "$SFTWIN_PROJECT_ROOT/ros2_ws/install/setup.bash" && '
            f'exec "{python}" -m arm_cell_simulation_ui_hub.sidecar '
            '"$SFTWIN_HUB_SIDECAR_FD"'
        )
        parent_socket, child_socket = socket.socketpair()
        env["SFTWIN_HUB_SIDECAR_FD"] = str(child_socket.fileno())
        self._process = subprocess.Popen(
            ["/bin/bash", "-c", bootstrap],
            env=env,
            pass_fds=(child_socket.fileno(),),
            close_fds=True,
            stdout=subprocess.DEVNULL,
            stderr=sys.stderr,
        )
        child_socket.close()
        parent_socket.settimeout(10.0)
        self._socket = parent_socket
        self._reader = parent_socket.makefile("rb")
        self._writer = parent_socket.makefile("wb")
        self.scenario_ports = (
            scenario_ports or OwnerScenarioPorts.from_process_registry()
        )
        self.request_state: dict[str, str] = {}
        self.fault_active = {scenario: False for scenario in VDA_MOCK_FAULT_SCENARIOS}
        self._observations = {name: f"{name}: unavailable" for name in _OBSERVATIONS}
        self._target_pixel: tuple[str, tuple[float, float] | None] = (
            "unavailable",
            None,
        )
        self._vision_state = "unavailable"
        self._vision_overlay: dict[str, Any] | None = None
        self._camera_frame: tuple[Any, int, int] | None = None
        self._camera_identity: tuple[float, str] | None = None
        self._video_mode = "LIVE"
        self._display_source = "rgb"
        self._display_status = "RGB · LIVE"
        self._sidecar_available: bool | None = None
        self._sidecar_camera_frame_available: bool | None = None
        self._closed = False
        self._rpc_broken = False
        self._fault_rpc_in_flight = False
        self._last_poll_at = 0.0
        ready = self._read_message()
        if not ready.get("ready"):
            self.close()
            raise RuntimeError(ready.get("error", "Hub ROS sidecar failed to start"))

    def _read_message(self) -> dict[str, Any]:
        line = self._reader.readline()
        if not line:
            raise RuntimeError("Hub ROS sidecar disconnected")
        return json.loads(line)

    def _rpc(
        self, message: dict[str, Any], *, timeout: float | None = None
    ) -> dict[str, Any]:
        previous_timeout = self._socket.gettimeout() if timeout is not None else None
        if timeout is not None:
            self._socket.settimeout(timeout)
        try:
            self._writer.write((json.dumps(message) + "\n").encode("utf-8"))
            self._writer.flush()
            return self._read_message()
        finally:
            if timeout is not None and not self._rpc_broken:
                self._socket.settimeout(previous_timeout)

    def spin_once(self) -> None:
        if self._closed:
            return
        if self._fault_rpc_in_flight:
            return
        if self._rpc_broken:
            self._mark_sidecar_unavailable()
            return
        # The ROS sidecar collects frames continuously. Transfer the latest
        # presentation snapshot at a bounded rate instead of blocking Kit on
        # a socket round trip for every application update.
        now = time.monotonic()
        if now - self._last_poll_at < _UI_POLL_INTERVAL_SECONDS:
            return
        self._last_poll_at = now
        try:
            response = self._rpc(
                {
                    "op": "poll",
                    "force_camera_frame": self._camera_frame is None,
                    "display_source": getattr(self, "_display_source", "rgb"),
                }
            )
            if response.get("ready") is False:
                raise RuntimeError("Hub ROS observation process is unavailable")
        except (OSError, RuntimeError, json.JSONDecodeError):
            self._rpc_broken = True
            self._mark_sidecar_unavailable()
            return
        if self._sidecar_available is False:
            _LOGGER.info("Hub operator video sidecar recovered")
        self._sidecar_available = True
        self._observations.update(response.get("observations", {}))
        self.request_state.update(response.get("requests", {}))
        self.fault_active.update(response.get("fault_active", {}))
        target = response.get("target_pixel", {})
        pixel = target.get("pixel")
        self._target_pixel = (
            target.get("state", "unavailable"),
            tuple(pixel) if pixel is not None else None,
        )
        self._vision_state = response.get("vision_state", "unavailable")
        self._vision_overlay = response.get("vision_overlay")
        self._video_mode = response.get("video_mode", "LIVE")
        self._display_source = response.get(
            "display_source", getattr(self, "_display_source", "rgb")
        )
        self._display_status = response.get(
            "display_status", getattr(self, "_display_status", "RGB · LIVE")
        )
        image = response.get("camera_rgba_b64")
        if response.get("camera_frame_current") is not True:
            self._camera_frame = None
            self._camera_identity = None
        else:
            self._camera_identity = (
                float(
                    response.get("display_frame_stamp", response.get("camera_stamp"))
                ),
                str(response.get("display_frame_id", response.get("camera_frame_id"))),
            )
        if response.get("camera_frame_current") is True and image:
            try:
                import numpy as np

                width, height = (
                    int(response["camera_width"]),
                    int(response["camera_height"]),
                )
                pixels = np.frombuffer(base64.b64decode(image), dtype=np.uint8).reshape(
                    height, width, 4
                )
                self._camera_frame = (pixels, width, height)
                if self._sidecar_camera_frame_available is False:
                    _LOGGER.info("Hub raw camera frame recovered")
                self._sidecar_camera_frame_available = True
            except Exception as error:
                self._camera_frame = None
                self._camera_identity = None
                self._video_mode = "UNAVAILABLE"
                self._vision_state = "unavailable"
                self._vision_overlay = None
                if self._sidecar_camera_frame_available is not False:
                    _LOGGER.warning("Hub raw camera frame unavailable")
                    _LOGGER.debug("Hub raw camera frame transfer detail: %s", error)
                self._sidecar_camera_frame_available = False

    def _mark_sidecar_unavailable(self) -> None:
        for name in _OBSERVATIONS:
            self._observations[name] = f"{name}: unavailable"
        self._camera_frame = None
        self._target_pixel = ("unavailable", None)
        self._vision_state = "unavailable"
        self._vision_overlay = None
        self._video_mode = "UNAVAILABLE"
        source = getattr(self, "_display_source", "rgb")
        self._display_status = f"{source.upper()} · SIDECAR UNAVAILABLE"
        request_state = getattr(self, "request_state", {})
        for request_id, state in tuple(request_state.items()):
            if state == "pending":
                request_state[request_id] = "failed: ROS sidecar unavailable"
        if self._sidecar_available is not False:
            _LOGGER.warning("Hub operator video sidecar unavailable")
        self._sidecar_available = False

    def camera_frame_rgba(self) -> tuple[Any, int, int] | None:
        return self._camera_frame

    def camera_identity(self) -> tuple[float, str] | None:
        """Compatibility alias for the selected display frame identity."""
        return self.display_frame_identity()

    def display_frame_identity(self) -> tuple[float, str] | None:
        return self._camera_identity

    def selected_target_pixel(
        self, *, now: float | None = None
    ) -> tuple[str, tuple[float, float] | None]:
        return self._target_pixel

    def vision_display(self) -> tuple[str, dict[str, Any] | None]:
        return self._vision_state, self._vision_overlay

    def video_mode(self) -> str:
        return self._video_mode

    def set_display_source(self, source_id: str) -> None:
        if source_id not in {"rgb", "depth"}:
            raise ValueError(f"unsupported Hub display source: {source_id}")
        self._display_source = source_id

    def display_source(self) -> str:
        return self._display_source

    def display_status(self, *, now: float | None = None) -> str:
        return self._display_status

    def observation_text(self, name: str, *, now: float | None = None) -> str:
        return self._observations.get(name, f"{name}: unavailable")

    def request_material(self) -> tuple[str, None]:
        if self._fault_rpc_in_flight or self._rpc_broken:
            raise RuntimeError("Hub ROS sidecar is busy or unavailable")
        response = self._rpc({"op": "request_material"})
        request_id = response["request_id"]
        self.request_state[request_id] = response["state"]
        return request_id, None

    def acknowledge_and_reset(self) -> tuple[str, None]:
        if self._fault_rpc_in_flight or self._rpc_broken:
            raise RuntimeError("Hub ROS sidecar is busy or unavailable")
        response = self._rpc({"op": "acknowledge_and_reset"})
        request_id = response["request_id"]
        self.request_state[request_id] = response["state"]
        return request_id, None

    def set_external_fault(self, scenario: str, active: bool) -> tuple[str, None]:
        if scenario not in VDA_MOCK_FAULT_SCENARIOS:
            raise ValueError(f"unsupported VDA mock fault: {scenario}")
        request_id = str(uuid.uuid4())
        if self._rpc_broken:
            self.request_state[request_id] = "failed: sidecar communication unavailable"
            return request_id, None
        if self._fault_rpc_in_flight:
            self.request_state[request_id] = "failed: sidecar busy"
            return request_id, None

        self.request_state[request_id] = "pending"
        self._fault_rpc_in_flight = True

        def send_fault_request() -> None:
            try:
                response = self._rpc(
                    {
                        "op": "set_external_fault",
                        "request_id": request_id,
                        "scenario": scenario,
                        "active": active,
                    },
                    timeout=_FAULT_RPC_TIMEOUT_SECONDS,
                )
                if response.get("request_id") != request_id:
                    raise RuntimeError("Hub ROS sidecar returned a mismatched request")
                self.request_state[request_id] = response["state"]
            except (OSError, RuntimeError, KeyError, json.JSONDecodeError) as error:
                self.request_state[request_id] = (
                    f"failed: sidecar communication failed: {error}"
                )
                self._rpc_broken = True
                self._socket.close()
            finally:
                self._fault_rpc_in_flight = False

        threading.Thread(
            target=send_fault_request,
            name="arm-cell-hub-fault-request",
            daemon=True,
        ).start()
        return request_id, None

    def set_scenario_active(self, owner: Owner, scenario: str, active: bool) -> bool:
        return self.scenario_ports.set_active(owner, scenario, active)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if not self._fault_rpc_in_flight and not self._rpc_broken:
            try:
                self._rpc({"op": "shutdown"})
            except (OSError, RuntimeError, json.JSONDecodeError):
                pass
        for stream in (self._reader, self._writer):
            try:
                stream.close()
            except OSError:
                pass
        self._socket.close()
        try:
            self._process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._process.terminate()
            self._process.wait(timeout=2.0)


def _serve(fd: int) -> None:
    """Run the ROS client over an inherited local socket."""
    connection = socket.socket(fileno=fd)
    writer = connection.makefile("wb")
    runtime = None
    last_image_identity = None
    pending_request = b""
    try:
        from .runtime import HubRuntime

        runtime = HubRuntime(
            default_observation_freshness_seconds=float(
                os.environ.get("SFTWIN_HUB_OBSERVATION_FRESHNESS_SECONDS", "1.0")
            ),
            default_annotation_source_tolerance_seconds=float(
                os.environ.get(
                    "SFTWIN_HUB_ANNOTATION_SOURCE_TOLERANCE_SECONDS",
                    str(DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS),
                )
            ),
        )
        writer.write(b'{"ready":true}\n')
        writer.flush()
        while True:
            # Keep ROS subscriptions draining independently of Kit's UI update
            # cadence so source RGB frames remain available for diagnostic
            # identity matching.
            runtime.spin_once()
            readable, _, _ = select.select([connection], [], [], 0.01)
            if not readable:
                continue
            chunk = connection.recv(65536)
            if not chunk:
                break
            pending_request += chunk
            if b"\n" not in pending_request:
                continue
            line, pending_request = pending_request.split(b"\n", 1)
            request = json.loads(line)
            operation = request.get("op")
            if operation == "shutdown":
                writer.write(b'{"ok":true}\n')
                writer.flush()
                break
            if operation == "set_external_fault":
                request_id, _future = runtime.set_external_fault(
                    request["scenario"],
                    bool(request["active"]),
                    request_id=request["request_id"],
                )
                result = {
                    "request_id": request_id,
                    "state": runtime.request_state[request_id],
                }
            elif operation in ("request_material", "acknowledge_and_reset"):
                method = getattr(runtime, operation)
                request_id, _future = method()
                result = {
                    "request_id": request_id,
                    "state": runtime.request_state[request_id],
                }
            else:
                requested_source = request.get("display_source", "rgb")
                if hasattr(runtime, "set_display_source"):
                    runtime.set_display_source(requested_source)
                now = time.monotonic()
                result = {
                    "observations": {
                        name: runtime.observation_text(name, now=now)
                        for name in _OBSERVATIONS
                    },
                    "requests": dict(runtime.request_state),
                    "fault_active": dict(getattr(runtime, "fault_active", {})),
                }
                state, pixel = runtime.selected_target_pixel(now=now)
                result["target_pixel"] = {"state": state, "pixel": pixel}
                vision_state, vision_overlay = runtime.vision_display(now=now)
                result["vision_state"] = vision_state
                result["vision_overlay"] = vision_overlay
                result["video_mode"] = runtime.video_mode()
                result["display_source"] = (
                    runtime.display_source()
                    if hasattr(runtime, "display_source")
                    else requested_source
                )
                result["display_status"] = (
                    runtime.display_status(now=now)
                    if hasattr(runtime, "display_status")
                    else result["video_mode"]
                )
                frame = runtime.camera_frame_rgba()
                result["camera_frame_current"] = frame is not None
                identity_reader = getattr(
                    runtime, "display_frame_identity", runtime.camera_identity
                )
                camera_identity = identity_reader()
                if frame is not None and camera_identity is not None:
                    result["display_frame_stamp"] = camera_identity[0]
                    result["display_frame_id"] = camera_identity[1]
                identity = (
                    result["display_source"],
                    runtime.video_mode(),
                    camera_identity,
                )
                if frame is None:
                    last_image_identity = None
                if frame is not None and (
                    request.get("force_camera_frame") or identity != last_image_identity
                ):
                    import numpy as np

                    pixels, width, height = frame
                    result.update(
                        camera_rgba_b64=base64.b64encode(
                            np.asarray(pixels).tobytes()
                        ).decode("ascii"),
                        camera_width=width,
                        camera_height=height,
                    )
                    last_image_identity = identity
            writer.write((json.dumps(result) + "\n").encode("utf-8"))
            writer.flush()
    except Exception as error:
        try:
            writer.write(
                (json.dumps({"ready": False, "error": str(error)}) + "\n").encode(
                    "utf-8"
                )
            )
            writer.flush()
        except OSError:
            pass
    finally:
        if runtime is not None:
            runtime.close()
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        writer.close()
        connection.close()


def main() -> None:
    _serve(int(sys.argv[1]))


if __name__ == "__main__":
    main()
