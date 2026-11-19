"""Bounded, Isaac-owned capture evidence and PhysX read adapters."""

import json
import logging
import math
import os
import queue
import re
import threading
import time
from pathlib import Path

LOGGER = logging.getLogger(__name__)
MAX_CAPTURE_RECORD_BYTES = 1024 * 1024
MAX_CAPTURE_DIRECTORY_BYTES = 16 * 1024 * 1024


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        if math.isnan(value):
            return "NaN"
        return "Infinity" if value > 0 else "-Infinity"
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    return value


class BoundedCaptureDiagnosticWriter:
    """Write completed capture records on one bounded background worker."""

    def __init__(self, output_directory, max_pending=2, write_record=None):
        self.output_directory = Path(output_directory)
        self._queue = queue.Queue(maxsize=max(1, int(max_pending)))
        self._state_lock = threading.Lock()
        self._write_record = write_record or self._write_json
        self._thread = threading.Thread(
            target=self._run, name="capture-diagnostic-writer", daemon=True
        )
        self._started = False
        self._closed = False
        self._drop_cause = None
        self._drop_count = 0
        self._reported_drop_cause = None
        self.dropped_count = 0
        self.write_failure_count = 0
        self._write_log_cause = None
        self._write_log_count = 0
        self.written_count = 0
        self.shutdown_status = "open"
        self._retained_bytes = None
        self._capacity_reached = False
        self.instance_id = f'{os.getpid()}-{time.monotonic_ns()}'

    def submit(self, record):
        """Transfer ownership of a completed record to the bounded writer."""
        with self._state_lock:
            if self._closed:
                self.dropped_count += 1
                self._mark_submission(record, queued=False)
                self._drop_cause = 'writer_closed'
                self._drop_count += 1
                return False
            self._mark_submission(record, queued=True)
            if not self._started:
                self._started = True
                self._thread.start()
            try:
                self._queue.put_nowait(record)
                return True
            except queue.Full:
                self.dropped_count += 1
                self._mark_submission(record, queued=False)
                self._drop_cause = 'writer_queue_full'
                self._drop_count += 1
                return False

    def _mark_submission(self, record, *, queued):
        if not isinstance(record, dict):
            return
        coverage = record.get("coverage")
        if not isinstance(coverage, dict):
            return
        coverage["writer_queued"] = bool(queued)
        coverage["writer_dropped_count_at_submit"] = self.dropped_count
        coverage["producer_instance"] = self.instance_id

    @staticmethod
    def _log(level, message, *args):
        try:
            LOGGER.log(level, message, *args)
        except Exception:
            # A closed/broken log sink cannot affect control or the writer.
            pass

    def _report_drops(self, recovered=False):
        with self._state_lock:
            cause, count = self._drop_cause, self._drop_count
            onset = cause is not None and cause != self._reported_drop_cause
            self._reported_drop_cause = cause
            if recovered:
                self._drop_cause = self._reported_drop_cause = None
                self._drop_count = 0
        if onset:
            self._log(logging.WARNING,
                "Capture diagnostic evidence is being dropped / reason=%s", cause)
        if recovered and count:
            self._log(logging.INFO,
                "Capture diagnostic drop episode ended / reason=%s dropped=%d", cause, count)

    def close(self, timeout=1.0):
        with self._state_lock:
            self._closed = True
            started = self._started
        if not started:
            self.shutdown_status = "complete"
            self._report_drops(recovered=True)
            self._write_summary(True)
            return True
        self._thread.join(timeout=max(0.0, float(timeout)))
        complete = not self._thread.is_alive() and self._queue.empty()
        self.shutdown_status = "complete" if complete else "incomplete"
        self._write_summary(complete)
        return complete

    def _write_summary(self, drained):
        if self._write_record != self._write_json:
            return
        try:
            self.output_directory.mkdir(parents=True, exist_ok=True)
            with self._state_lock:
                summary = dict(schema_version=1, record_type='capture_summary',
                    producer_instance=self.instance_id,
                    written_count=self.written_count, dropped_count=self.dropped_count,
                    write_failure_count=self.write_failure_count,
                    pending_count=self._queue.unfinished_tasks,
                    capture_complete=bool(drained and not self.dropped_count and not self.write_failure_count),
                    shutdown_status=self.shutdown_status)
            path = self.output_directory / 'capture_summary.json'
            history = self.output_directory / 'coverage'
            history.mkdir(exist_ok=True)
            (history / f'{self.instance_id}.json').write_text(json.dumps(summary)+'\n', encoding='utf-8')
            temporary = path.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(summary) + '\n', encoding='utf-8')
            os.replace(temporary, path)
        except Exception as error:
            self._log(logging.WARNING, 'Capture diagnostic summary unavailable: %s', error)

    def _run(self):
        while True:
            try:
                record = self._queue.get(timeout=0.05)
            except queue.Empty:
                self._report_drops(recovered=True)
                if self._closed:
                    return
                continue
            self._report_drops()
            try:
                serialized = json.dumps(
                    record, indent=2, sort_keys=True, allow_nan=False
                )
                if self._write_record == self._write_json:
                    self._write_json(record, serialized)
                else:
                    self._write_record(record)
                self.written_count += 1
                if self._write_log_cause is not None:
                    self._log(logging.INFO,
                        "Capture diagnostic storage recovered after %d failed records",
                        self._write_log_count,
                    )
                    self._write_log_cause = None
                    self._write_log_count = 0
            except Exception as error:
                self.write_failure_count += 1
                detail = error.strerror if isinstance(error, OSError) and error.strerror else str(error)
                cause = (type(error).__name__, detail)
                self._write_log_count += 1
                if cause != self._write_log_cause:
                    self._log(logging.WARNING,
                        "Capture diagnostic storage unavailable; operation results are unchanged / %s", detail)
                    self._write_log_cause = cause
            finally:
                self._queue.task_done()

    def _write_json(self, record, serialized):
        record_id = record.get("motion_goal_uuid") or record.get("record_id")
        if not isinstance(record_id, str) or not re.fullmatch(
            r"[A-Za-z0-9_-]+", record_id
        ):
            raise ValueError("valid diagnostic record ID is unavailable")
        self.output_directory.mkdir(parents=True, exist_ok=True)
        encoded = serialized.encode("utf-8")
        if len(encoded) > MAX_CAPTURE_RECORD_BYTES:
            raise ValueError(
                f"Capture Diagnostic Record exceeds {MAX_CAPTURE_RECORD_BYTES} bytes"
            )
        if self._retained_bytes is None:
            self._retained_bytes = sum(item.stat().st_size
                for item in self.output_directory.iterdir() if item.is_file()
                and item.suffix in ('.json', '.jsonl'))
        execution = record.get('execution_id') or record.get('motion_goal_uuid')
        index_line = (json.dumps(dict(execution_id=execution, path=f'{record_id}.json'),
                      separators=(',', ':')) + '\n') if execution else ''
        required = len(encoded) + 1 + len(index_line.encode('utf-8'))
        if self._capacity_reached or self._retained_bytes + required > MAX_CAPTURE_DIRECTORY_BYTES:
            self._capacity_reached = True
            raise OSError(
                f"Capture Diagnostic Record directory limit of "
                f"{MAX_CAPTURE_DIRECTORY_BYTES} bytes would be exceeded"
            )
        destination = self.output_directory / f"{record_id}.json"
        temporary = destination.with_suffix(".json.tmp")
        if destination.exists():
            raise FileExistsError(
                "Capture Diagnostic Record already exists"
            )
        with temporary.open("w", encoding="utf-8") as stream:
            stream.write(serialized)
            stream.write("\n")
        if destination.exists():
            temporary.unlink(missing_ok=True)
            raise FileExistsError(
                "Capture Diagnostic Record already exists"
            )
        os.replace(temporary, destination)
        self._retained_bytes += len(encoded) + 1
        if index_line:
            with (self.output_directory / 'execution_index.jsonl').open('a', encoding='utf-8') as index:
                index.write(index_line)
            self._retained_bytes += len(index_line.encode('utf-8'))


class IsaacGripperDiagnosticRecord:
    """Bound one applied open command and its observed simulator outcome."""

    MAX_EVENTS = 32

    def __init__(self, writer, generation, width_mm, initial_actuator_state="unknown"):
        self.writer = writer
        self.started_monotonic_ns = time.monotonic_ns()
        self.record_id = f"release-{self.started_monotonic_ns}-{generation}"
        self.steps = 0
        self.finalized = False
        self.last_actuator_state = str(initial_actuator_state)
        self.data = {
            "schema_version": 1,
            "record_type": "isaac_gripper_release",
            "record_id": self.record_id,
            "action_graph_command_generation": int(generation),
            "requested_width_mm": float(width_mm),
            "time_bases": {
                "observed_monotonic_ns": "host monotonic clock",
                "physics_step": "per GripperRuntimeSession callback ordinal",
            },
            "events": [],
            "coverage": {"physics_step_limit": 128},
        }
        self.append(
            "actuator_state",
            {"state": self.last_actuator_state, "observation": "command_local_initial"},
        )
        self.append("command_applied", {"width_mm": width_mm})

    def append(self, kind, payload, *, physics_step=None):
        if self.finalized:
            return False
        if len(self.data["events"]) >= self.MAX_EVENTS - 1 and kind != "outcome":
            self.data["coverage"]["events_dropped"] = (
                self.data["coverage"].get("events_dropped", 0) + 1
            )
            return False
        self.data["events"].append(
            {
                "sequence": len(self.data["events"]) + 1,
                "kind": kind,
                "observed_monotonic_ns": time.monotonic_ns(),
                "physics_step": physics_step,
                "payload": _json_safe(payload),
            }
        )
        return True

    def finalize(self, outcome, details=None):
        if self.finalized:
            return False
        self.data["coverage"]["physics_steps_observed"] = self.steps
        self.append("outcome", {"result": outcome, **(details or {})})
        self.data["coverage"]["writer_queued"] = True
        self.finalized = True
        try:
            queued = bool(self.writer.submit(self.data))
        except Exception as error:
            self.data["coverage"]["writer_queued"] = False
            self.data["coverage"]["writer_submit_failed"] = True
            LOGGER.warning(
                "Isaac gripper release diagnostic submit failed record_id=%s; gripper operation is unchanged: %s",
                self.record_id,
                error,
            )
            return False
        self.data["coverage"]["writer_queued"] = queued
        return queued


class CaptureDiagnosticRecord:
    """Build one sparse Observation → Decision → Outcome record per capture."""

    def __init__(self, writer, motion_goal_uuid, capture_reference_id, max_events=64):
        self.writer = writer
        self.motion_goal_uuid = motion_goal_uuid
        self.capture_reference_id = capture_reference_id
        self._sequence = 0
        self._outcome = None
        self._finalized = False
        self._queued = False
        self._max_events = max(2, int(max_events))
        self.data = {
            "schema_version": 1,
            "motion_goal_uuid": motion_goal_uuid,
            "capture_reference_id": capture_reference_id,
            "time_bases": {
                "observed_monotonic_ns": "host monotonic clock",
                "physics_step": "per GripperRuntimeSession callback ordinal",
                "physics_dt_s": "physics callback dt; simulator absolute time is unavailable",
            },
            "events": [],
            "coverage": {"physics_step_limit": 4, "physics_steps_observed": 0},
        }

    def append(self, kind, payload, *, physics_step=None, dt_s=None):
        if self._finalized:
            return False
        if len(self.data["events"]) >= self._max_events - 1 and kind != "outcome":
            self.data["coverage"]["events_dropped"] = (
                self.data["coverage"].get("events_dropped", 0) + 1
            )
            return False
        self._sequence += 1
        self.data["events"].append(
            {
                "sequence": self._sequence,
                "kind": kind,
                "observed_monotonic_ns": time.monotonic_ns(),
                "physics_step": physics_step,
                "physics_dt_s": dt_s,
                "payload": _json_safe(payload),
            }
        )
        return True

    def set_outcome(
        self,
        state,
        reason,
        holding_sequence,
        observed_at,
        *,
        capture_decision=None,
        capture_result=None,
        transition_reason=None,
    ):
        outcome = dict(self._outcome or {})
        outcome.update(
            {
                "holding_state": state,
                "reason": reason,
                "holding_sequence": holding_sequence,
                "holding_observed_monotonic_s": observed_at,
            }
        )
        if capture_decision is not None:
            outcome["capture_decision"] = capture_decision
        if capture_result is not None:
            outcome["capture_result"] = capture_result
        if transition_reason is not None:
            outcome["holding_transition_reason"] = transition_reason
        outcome.setdefault("capture_decision", None)
        outcome.setdefault("capture_result", None)
        self._outcome = outcome

    def finalize(self):
        if self._finalized:
            return self._queued
        if self._outcome is not None:
            self.append("outcome", self._outcome)
        self.data["coverage"]["writer_queued"] = True
        self._finalized = True
        completed_record = self.data
        self.data = None
        queued = bool(self.writer.submit(completed_record))
        self._queued = queued
        return queued


class IsaacPhysXObserver:
    """Read actual initialized PhysX rigid-body state through Isaac Core 5.1."""

    def __init__(self, max_views=16):
        self._views = {}
        self._unavailable = {}
        self._max_views = max(1, int(max_views))

    def prepare(self, prim_paths):
        """Initialize direct PhysX views before the runtime subscribes callbacks."""
        for path in tuple(dict.fromkeys(str(item) for item in prim_paths)):
            if len(self._views) >= self._max_views and path not in self._views:
                self._unavailable[path] = (
                    f"bounded PhysX view limit ({self._max_views}) reached before callback subscription"
                )
                continue
            self._initialize_view(path)

    def observe_pair(self, tcp_path, rawpart_path):
        return {
            "source": "isaacsim.core.prims.SingleRigidPrim_physx",
            "tcp": self._observe(str(tcp_path)),
            "rawpart": self._observe(str(rawpart_path)),
        }

    def _observe(self, path):
        if path in self._unavailable:
            return {"status": "unavailable", "reason": self._unavailable[path]}
        view = self._views.get(path)
        if view is None:
            return {
                "status": "unavailable",
                "reason": "PhysX view was not initialized before the physics callback",
            }
        try:
            state = view.get_current_dynamic_state()
            return {
                "status": "observed",
                "pose_world": {
                    "position_m": self._sequence(state.position),
                    "orientation_wxyz": self._sequence(state.orientation),
                },
                "linear_velocity_m_s": self._sequence(state.linear_velocity),
                "angular_velocity_rad_s": self._sequence(state.angular_velocity),
            }
        except Exception as error:
            return {
                "status": "unavailable",
                "reason": f"{type(error).__name__}: {error}",
            }

    def _initialize_view(self, path):
        if path in self._views or path in self._unavailable:
            return path in self._views
        try:
            from pxr import UsdPhysics
            import omni.usd
            from isaacsim.core.prims import SingleRigidPrim

            prim = omni.usd.get_context().get_stage().GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError("rigid body prim is missing from Stage")
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                raise RuntimeError("RigidBodyAPI is not applied to prim")
            view = SingleRigidPrim(path, reset_xform_properties=False)
            view.initialize()
            self._views[path] = view
            return True
        except Exception as error:
            self._unavailable[path] = f"{type(error).__name__}: {error}"
            return False

    @staticmethod
    def _sequence(value):
        return tuple(float(item) for item in value)


class AttachmentPhysicsDiagnostics:
    """Fixed Attach's post-attachment diagnostic window; no holding decisions."""

    def __init__(self, manager):
        self.manager = manager
        self._last_record = None
        self._capture_physics_steps = 0
        self._previous_capture_relative = None

    def observe(self, dt, physics_step):
        record = getattr(self.manager, "capture_diagnostic_record", None)
        target_path = getattr(
            self.manager, "capture_diagnostic_target_path", None
        )
        if record is None or target_path is None:
            self._last_record = None
            return
        if record is not self._last_record:
            self._last_record = record
            self._capture_physics_steps = 0
            self._previous_capture_relative = None
            return
        self._capture_physics_steps += 1
        try:
            pair = self.manager.physx_observer.observe_pair(
                self.manager.tcp_path, target_path
            )
        except Exception as error:
            pair = {
                "status": "unavailable",
                "reason": f"{type(error).__name__}: {error}",
            }
        stage = self._stage_pair(self.manager, target_path)
        relative = self._relative_physx(pair)
        change = self._relative_change(self._previous_capture_relative, relative)
        self._previous_capture_relative = relative
        record.append(
            "observation",
            {
                "phase": "post_attach_physics_step",
                "source": "physx_rigid_body",
                "pair": pair,
                "relative_pose": relative,
                "relative_change_from_previous_step": change,
                "stage_world_pose": stage,
                "holding_state": self.manager.holding_observation().state.value,
            },
            physics_step=physics_step,
            dt_s=float(dt),
        )
        record.data["coverage"]["physics_steps_observed"] = self._capture_physics_steps
        if self._capture_physics_steps >= 4:
            self.manager.finalize_capture_diagnostic()
            self._capture_physics_steps = 0
            self._previous_capture_relative = None

    @staticmethod
    def _stage_pair(manager, target_path):
        try:
            source = manager.candidate_source
            world_transform = getattr(source, "_world_transform")
            pose_dict = source._pose_dict
            return {
                "source": "usd_stage_world",
                "tcp": pose_dict(world_transform(manager.tcp_path)),
                "rawpart": pose_dict(world_transform(target_path)),
            }
        except Exception as error:
            return {
                "source": "usd_stage_world",
                "status": "unavailable",
                "reason": f"{type(error).__name__}: {error}",
            }

    @staticmethod
    def _relative_physx(pair):
        try:
            tcp = pair["tcp"]
            rawpart = pair["rawpart"]
            if tcp.get("status") != "observed" or rawpart.get("status") != "observed":
                return {
                    "status": "unavailable",
                    "reason": "one or both PhysX rigid-body states are unavailable",
                }
            p0 = tcp["pose_world"]["position_m"]
            p1 = rawpart["pose_world"]["position_m"]
            q0 = tcp["pose_world"]["orientation_wxyz"]
            q1 = rawpart["pose_world"]["orientation_wxyz"]
            inv0 = (q0[0], -q0[1], -q0[2], -q0[3])
            delta = tuple(p1[i] - p0[i] for i in range(3))
            rotated_delta = AttachmentPhysicsDiagnostics._rotate(inv0, delta)
            qrel = AttachmentPhysicsDiagnostics._multiply(inv0, q1)
            v0, v1 = tcp["linear_velocity_m_s"], rawpart["linear_velocity_m_s"]
            w0, w1 = tcp["angular_velocity_rad_s"], rawpart["angular_velocity_rad_s"]
            return {
                "status": "observed",
                "position_tcp_frame_m": rotated_delta,
                "orientation_tcp_frame_wxyz": qrel,
                "linear_velocity_difference_world_m_s": tuple(
                    v1[i] - v0[i] for i in range(3)
                ),
                "angular_velocity_difference_world_rad_s": tuple(
                    w1[i] - w0[i] for i in range(3)
                ),
            }
        except Exception as error:
            return {
                "status": "unavailable",
                "reason": f"{type(error).__name__}: {error}",
            }

    @staticmethod
    def _relative_change(previous, current):
        if previous is None or current.get("status") != "observed":
            return {
                "status": "unavailable",
                "reason": "no prior comparable PhysX sample",
            }
        if previous.get("status") != "observed":
            return {"status": "unavailable", "reason": "prior PhysX sample unavailable"}
        before = previous["position_tcp_frame_m"]
        after = current["position_tcp_frame_m"]
        dq = sum(
            a * b
            for a, b in zip(
                previous["orientation_tcp_frame_wxyz"],
                current["orientation_tcp_frame_wxyz"],
            )
        )
        return {
            "status": "observed",
            "position_delta_m": tuple(after[i] - before[i] for i in range(3)),
            "orientation_delta_deg": math.degrees(2.0 * math.acos(min(1.0, abs(dq)))),
            "linear_velocity_difference_delta_world_m_s": tuple(
                current["linear_velocity_difference_world_m_s"][i]
                - previous["linear_velocity_difference_world_m_s"][i]
                for i in range(3)
            ),
            "angular_velocity_difference_delta_world_rad_s": tuple(
                current["angular_velocity_difference_world_rad_s"][i]
                - previous["angular_velocity_difference_world_rad_s"][i]
                for i in range(3)
            ),
        }

    @staticmethod
    def _multiply(a, b):
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    @staticmethod
    def _rotate(q, v):
        rotated = AttachmentPhysicsDiagnostics._multiply(
            AttachmentPhysicsDiagnostics._multiply(q, (0.0, *v)),
            (q[0], -q[1], -q[2], -q[3]),
        )
        return tuple(rotated[1:])
