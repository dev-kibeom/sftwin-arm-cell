"""Play-time lifecycle for the scripted Robotiq actuator and Isaac adapter."""

import logging
import math
import time
from pathlib import Path

from graph_builder.graph_contract import GRAPH_PATH
from shared.environment_setup import resolve_diagnostics_selection

from gripper_runtime.action_graph_command_adapter import SimGripperAdapter
from gripper_runtime.capture_diagnostics import (
    BoundedCaptureDiagnosticWriter,
    IsaacGripperDiagnosticRecord,
    IsaacPhysXObserver,
)
from gripper_runtime.grasp_attachment import GraspManager, HoldingState
from gripper_runtime.grasp_stage import EligibleObjectRegistry
from gripper_runtime.robotiq_actuator import Robotiq2F85Actuator

LOGGER = logging.getLogger(__name__)


def initial_status_sequence(previous_sequence=0, monotonic_ns=None):
    """Choose a sequence floor that outlives same-host runtime replacement."""
    current_monotonic_ns = (
        time.monotonic_ns() if monotonic_ns is None else int(monotonic_ns)
    )
    return max(int(previous_sequence), current_monotonic_ns)


class GripperRuntimeSession:
    """Own runtime objects and the retained physics-step subscription."""

    def __init__(
        self,
        physics_interface,
        actuator_cls=Robotiq2F85Actuator,
        adapter_cls=SimGripperAdapter,
        grasp_manager_cls=GraspManager,
        capture_config=None,
        capture_reference_config=None,
        registry=None,
        initial_status_sequence=0,
        diagnostic_writer=None,
        physx_observer=None,
        diagnostic_output_directory=None,
    ):
        self.grasp_config = capture_config
        self.registry = registry or EligibleObjectRegistry()
        self.gripper = actuator_cls(verbose=True)
        self.sim_adapter = adapter_cls(
            gripper=self.gripper,
            graph_path=GRAPH_PATH,
            move_duration=0.40,
            verbose=True,
        )
        prime_generation = getattr(self.sim_adapter, "prime_command_generation", None)
        if callable(prime_generation):
            prime_generation()
        self.grasp_manager = grasp_manager_cls(
            gripper=self.gripper,
            registry=self.registry,
            config=self.grasp_config,
            capture_reference_config=capture_reference_config,
            verbose=True,
        )
        selected_diagnostics = resolve_diagnostics_selection()
        diagnostics_enabled = True  # Required lightweight Capture/Release evidence.
        detailed_enabled = "motion" in selected_diagnostics
        self.capture_diagnostics_enabled = diagnostics_enabled
        self.capture_diagnostic_writer = diagnostic_writer
        self.physx_observer = physx_observer
        output_directory = diagnostic_output_directory or (
            Path(__file__).resolve().parents[5]
            / ".local_artifacts"
            / "capture_diagnostic_records"
        )
        if self.capture_diagnostic_writer is None:
            try:
                self.capture_diagnostic_writer = BoundedCaptureDiagnosticWriter(
                    output_directory
                )
            except Exception as error:
                LOGGER.warning(
                    "Capture diagnostic writer unavailable; gripper behavior is unchanged: %s",
                    error,
                )
        if detailed_enabled:
            try:
                self.physx_observer = self.physx_observer or IsaacPhysXObserver()
            except Exception as error:
                self.physx_observer = None
                LOGGER.warning(
                    "Detailed PhysX diagnostics unavailable; gripper behavior is unchanged: %s",
                    error,
                )
        LOGGER.debug(
            "Gripper lightweight capture diagnostics enabled; detailed PhysX %s "
            "in Isaac process (SFTWIN_DIAGNOSTICS selection=%s)",
            "enabled" if detailed_enabled else "disabled",
            ",".join(sorted(selected_diagnostics)) or "none",
        )
        prepare_physx = getattr(self.physx_observer, "prepare", None)
        if callable(prepare_physx):
            prim_paths = [getattr(self.grasp_manager, "tcp_path", "")]
            source = getattr(self.grasp_manager, "candidate_source", None)
            registry = getattr(source, "registry", None)
            stage = getattr(source, "stage", None)
            candidate_paths = getattr(registry, "candidate_paths", None)
            if callable(candidate_paths) and stage is not None:
                try:
                    registered = candidate_paths(stage)
                except Exception:
                    registered = None
                if registered:
                    prim_paths.extend(registered)
            elif callable(getattr(registry, "paths", None)):
                try:
                    prim_paths.extend(registry.paths())
                except Exception:
                    pass
            try:
                prepare_physx(prim_paths)
            except Exception as error:
                self.physx_observer = None
                LOGGER.warning(
                    "Detailed PhysX observation could not be prepared; gripper behavior is unchanged: %s",
                    error,
                )
        configure_diagnostics = getattr(
            self.grasp_manager, "set_diagnostic_observers", None
        )
        if callable(configure_diagnostics):
            try:
                configure_diagnostics(
                    self.capture_diagnostic_writer, self.physx_observer
                )
            except Exception as error:
                LOGGER.warning(
                    "Gripper diagnostics could not be attached; gripper behavior is unchanged: %s",
                    error,
                )
        restore_sequence = getattr(self.grasp_manager, "restore_holding_sequence", None)
        if callable(restore_sequence):
            self._status_sequence = int(restore_sequence(initial_status_sequence))
        else:
            self._status_sequence = int(initial_status_sequence)
        self._physics_interface = physics_interface
        self.physics_sub = None
        self._last_runtime_ready = None
        self._feedback_generation = 0
        self.gripper.feedback_generation = 0
        self._physics_step_sequence = 0
        self._capture_physics_steps = 0
        self._previous_capture_relative = None
        self._active_release_record = None
        self._last_release_joint_owned = None
        self._last_release_holding_state = None
        self.reconcile_stale_owned_joint()

    def on_physics_step(self, dt):
        self._physics_step_sequence += 1
        capture_record_before = getattr(
            self.grasp_manager, "capture_diagnostic_record", None
        )
        read_capture_reference = getattr(
            self.sim_adapter, "read_capture_reference", None
        )
        configure_reference = getattr(
            self.grasp_manager, "configure_selected_grasp_reference", None
        )
        reference_id = None
        if callable(read_capture_reference) and callable(configure_reference):
            packet = read_capture_reference()
            if packet is not None:
                configure_reference(packet)
                reference_id = getattr(self.grasp_manager, "capture_reference_id", None)
                mark_applied = getattr(
                    self.sim_adapter, "mark_capture_reference_applied", None
                )
                if callable(mark_applied):
                    mark_applied(reference_id)
        pre_command_width_mm = None
        get_width = getattr(self.gripper, "get_width", None)
        if callable(get_width):
            try:
                pre_command_width_mm = float(get_width())
            except Exception:
                pre_command_width_mm = None
        pre_command_state = getattr(
            getattr(self.gripper, "state", None), "value", "unknown"
        )
        command_applied = self.sim_adapter.update()
        command_width = getattr(self.sim_adapter, "last_command_width_mm", None)
        is_open_command = (
            command_applied
            and command_width is not None
            and float(command_width)
            >= getattr(self.sim_adapter, "MAX_WIDTH_MM", 85.0) - 1e-6
        )
        if is_open_command and self.capture_diagnostics_enabled:
            if self._active_release_record is not None:
                self._finalize_release_record("superseded_by_open_command")
            generation = getattr(self.sim_adapter, "last_command_generation", None)
            if generation is not None:
                self._active_release_record = IsaacGripperDiagnosticRecord(
                    self.capture_diagnostic_writer,
                    generation,
                    command_width,
                    initial_actuator_state=pre_command_state,
                )
                self._active_release_record.data["holding_sequence_before_command"] = (
                    self.grasp_manager.holding_observation().sequence
                )
                self._last_release_joint_owned = self._safe_attachment_owned()
                self._last_release_holding_state = None
                self._active_release_record.append(
                    "attachment_before_open",
                    {"owned_joint": self._last_release_joint_owned},
                    physics_step=self._physics_step_sequence,
                )
                self._active_release_record.append(
                    "open_command_boundary",
                    {
                        "phase": "before_apply",
                        "measured_width_mm": pre_command_width_mm,
                        "target_width_mm": command_width,
                        "actuator_state": pre_command_state,
                        "attachment_owned": self._last_release_joint_owned,
                    },
                    physics_step=self._physics_step_sequence,
                )
                LOGGER.info(
                    "Isaac gripper open command applied generation=%s width_mm=%.3f",
                    generation,
                    command_width,
                )
        if command_applied:
            note_command = getattr(
                self.grasp_manager, "note_gripper_command_applied", None
            )
            if callable(note_command) and not is_open_command:
                note_command()
            if is_open_command:
                note_open = getattr(
                    self.grasp_manager, "note_open_release_command_applied", None
                )
                generation = getattr(self.sim_adapter, "last_command_generation", None)
                if callable(note_open) and generation is not None:
                    note_open(generation, command_width)
        command_is_closing = getattr(self.sim_adapter, "last_command_is_closing", None)
        if command_applied and command_is_closing is True:
            close_id_reader = getattr(self.sim_adapter, "read_close_execution_id", None)
            close_id = close_id_reader() if callable(close_id_reader) else None
            arm_close = getattr(self.grasp_manager, "arm_close_transaction", None)
            if callable(arm_close):
                result = arm_close(
                    close_id,
                    getattr(self.sim_adapter, "last_command_generation", None),
                )
                if not result:
                    reason = (
                        getattr(
                            self.grasp_manager,
                            "_close_transaction_failure_reason",
                            None,
                        )
                        or "close_transaction_arm_failed"
                    )
                    self.grasp_manager._mark_unknown(reason)
                    outcome = getattr(self.grasp_manager, "_diagnostic_outcome", None)
                    if callable(outcome):
                        outcome(reason, capture_result="not_attempted")
                    return self.grasp_manager.state
                if getattr(self.sim_adapter, "verbose", False):
                    generation = getattr(
                        self.sim_adapter, "last_command_generation", None
                    )
                    LOGGER.debug(
                        "capture transaction arm generation=%s result=%s",
                        generation,
                        result,
                    )
        self.gripper.update(dt)
        if is_open_command and self._active_release_record is not None:
            get_width = getattr(self.gripper, "get_width", None)
            try:
                measured_width_mm = float(get_width()) if callable(get_width) else None
            except Exception:
                measured_width_mm = None
            self._active_release_record.append(
                "open_command_boundary",
                {
                    "phase": "after_apply",
                    "measured_width_mm": measured_width_mm,
                    "target_width_mm": command_width,
                    "actuator_state": getattr(
                        getattr(self.gripper, "state", None), "value", "unknown"
                    ),
                    "attachment_owned": self._safe_attachment_owned(),
                },
                physics_step=self._physics_step_sequence,
            )
        self._feedback_generation += 1
        self.gripper.feedback_generation = self._feedback_generation
        self.grasp_manager.update(dt)
        consume_detach = getattr(
            self.grasp_manager, "consume_release_detach_event", None
        )
        detach_event = consume_detach() if callable(consume_detach) else None
        if detach_event and self._active_release_record is not None:
            self._active_release_record.append(
                "detach_attempt",
                detach_event,
                physics_step=self._physics_step_sequence,
            )
        self._observe_release_step()
        capture_record_after = getattr(
            self.grasp_manager, "capture_diagnostic_record", None
        )
        if (
            capture_record_before is not None
            and capture_record_after is not capture_record_before
        ):
            self._capture_physics_steps = 0
            self._previous_capture_relative = None
        elif capture_record_before is not None:
            self._observe_capture_physics_step(dt)
        runtime_ready = bool(getattr(self.grasp_manager, "ready", False))
        observation = getattr(self.grasp_manager, "holding_observation", None)
        if not callable(observation):
            raise RuntimeError("grasp manager must provide holding_observation()")
        observation = observation()
        self._status_sequence = int(observation.sequence)
        if self._status_sequence <= 0:
            raise RuntimeError("gripper status sequence must be non-zero")
        held = runtime_ready and observation.state is HoldingState.HELD
        released = runtime_ready and observation.state is HoldingState.RELEASED
        attached = runtime_ready and held
        if runtime_ready != self._last_runtime_ready:
            LOGGER.info(
                "gripper runtime state=%s",
                "READY" if runtime_ready else "UNREADY",
            )
            self._last_runtime_ready = runtime_ready
        self.sim_adapter.publish_status(
            runtime_ready=runtime_ready,
            grasp_confirmed=held,
            attached=attached,
            released=released,
            width_mm=self.gripper.get_width(),
            sequence=self._status_sequence,
            capture_reference_id=getattr(
                self.grasp_manager, "capture_reference_id", None
            )
            or "",
        )

    def _observe_release_step(self):
        record = self._active_release_record
        if record is None:
            return
        gripper_state = getattr(self.gripper.state, "value", "unknown")
        observation = self.grasp_manager.holding_observation()
        joint_owned = self._safe_attachment_owned()
        record.steps += 1
        if observation.state.value != self._last_release_holding_state:
            record.append(
                "holding_observation",
                {
                    "state": observation.state.value,
                    "sequence": observation.sequence,
                    "fresh": observation.fresh(),
                },
                physics_step=self._physics_step_sequence,
            )
            self._last_release_holding_state = observation.state.value
        if gripper_state != record.last_actuator_state:
            record.append(
                "actuator_state",
                {
                    "state": gripper_state,
                    "width_mm": self._safe_gripper_width(),
                    "observation": "actuator_transition",
                },
                physics_step=self._physics_step_sequence,
            )
            LOGGER.info(
                "Isaac gripper actuator state=%s generation=%s",
                gripper_state,
                record.data["action_graph_command_generation"],
            )
            record.last_actuator_state = gripper_state
        if self._last_release_joint_owned is True and joint_owned is False:
            record.append(
                "joint_detached",
                {"holding_state": observation.state.value},
                physics_step=self._physics_step_sequence,
            )
            LOGGER.info(
                "Isaac gripper attachment absent after open generation=%s holding=%s",
                record.data["action_graph_command_generation"],
                observation.state.value,
            )
        self._last_release_joint_owned = joint_owned
        if (
            observation.state is HoldingState.RELEASED
            and observation.fresh()
            and observation.sequence > record.data["holding_sequence_before_command"]
        ):
            record.finalize(
                "released",
                {
                    "holding_state": observation.state.value,
                    "holding_sequence": observation.sequence,
                    "owned_joint": joint_owned,
                },
            )
            LOGGER.info(
                "Isaac gripper release confirmed generation=%s holding_sequence=%s",
                record.data["action_graph_command_generation"],
                observation.sequence,
            )
            self._clear_release_tracking()

        elif record.steps >= record.data["coverage"]["physics_step_limit"]:
            record.data["coverage"]["physics_steps_observed"] = record.steps
            record.finalize(
                "release_not_confirmed_within_diagnostic_window",
                {
                    "actuator_state": gripper_state,
                    "measured_width_mm": self._safe_gripper_width(),
                    "holding_state": observation.state.value,
                    "holding_sequence": observation.sequence,
                    "owned_joint": joint_owned,
                },
            )
            LOGGER.warning(
                "Isaac gripper release not confirmed generation=%s holding=%s",
                record.data["action_graph_command_generation"],
                observation.state.value,
            )
            self._clear_release_tracking()

    def _safe_attachment_owned(self):
        try:
            return bool(self.grasp_manager.joint_owner.has_owned_joint())
        except Exception as error:
            LOGGER.warning(
                "Isaac gripper release diagnostic attachment observation failed; gripper operation is unchanged: %s",
                error,
            )
            return None

    def _safe_gripper_width(self):
        try:
            return float(self.gripper.get_width())
        except Exception as error:
            LOGGER.warning(
                "Isaac gripper release diagnostic actuator observation failed; gripper operation is unchanged: %s",
                error,
            )
            return None

    def _finalize_release_record(self, outcome):
        record = self._active_release_record
        if record is None:
            self._clear_release_tracking()
            return False
        try:
            observation = self.grasp_manager.holding_observation()
            joint_owner = getattr(self.grasp_manager, "joint_owner", None)
            has_owned_joint = getattr(joint_owner, "has_owned_joint", None)
            owned_joint = bool(has_owned_joint()) if callable(has_owned_joint) else None
            record.finalize(
                outcome,
                {
                    "holding_state": observation.state.value,
                    "holding_sequence": observation.sequence,
                    "owned_joint": owned_joint,
                },
            )
            return True
        except Exception as error:
            LOGGER.warning(
                "Isaac gripper release diagnostic finalization failed generation=%s; gripper operation is unchanged: %s",
                record.data["action_graph_command_generation"],
                error,
            )
            return False
        finally:
            self._clear_release_tracking()

    def _clear_release_tracking(self):
        self._active_release_record = None
        self._last_release_joint_owned = None
        self._last_release_holding_state = None

    def _observe_capture_physics_step(self, dt):
        record = getattr(self.grasp_manager, "capture_diagnostic_record", None)
        target_path = getattr(
            self.grasp_manager, "capture_diagnostic_target_path", None
        )
        if record is None or target_path is None:
            return
        self._capture_physics_steps += 1
        try:
            pair = self.physx_observer.observe_pair(
                self.grasp_manager.tcp_path, target_path
            )
        except Exception as error:
            pair = {
                "status": "unavailable",
                "reason": f"{type(error).__name__}: {error}",
            }
        stage = self._stage_pair(self.grasp_manager, target_path)
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
                "holding_state": self.grasp_manager.holding_observation().state.value,
            },
            physics_step=self._physics_step_sequence,
            dt_s=float(dt),
        )
        record.data["coverage"]["physics_steps_observed"] = self._capture_physics_steps
        if self._capture_physics_steps >= 4:
            self.grasp_manager.finalize_capture_diagnostic()
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
            rotated_delta = GripperRuntimeSession._rotate(inv0, delta)
            qrel = GripperRuntimeSession._multiply(inv0, q1)
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
        rotated = GripperRuntimeSession._multiply(
            GripperRuntimeSession._multiply(q, (0.0, *v)),
            (q[0], -q[1], -q[2], -q[3]),
        )
        return tuple(rotated[1:])

    def subscribe(self):
        self._unsubscribe()
        self.physics_sub = self._physics_interface.subscribe_physics_on_step_events(
            self.on_physics_step, True, 0
        )
        return self.physics_sub

    def _unsubscribe(self):
        subscription = self.physics_sub
        if subscription is None:
            return
        unsubscribe = getattr(subscription, "unsubscribe", None)
        if callable(unsubscribe):
            unsubscribe()
        else:
            unsubscribe = getattr(
                self._physics_interface,
                "unsubscribe_physics_step_events",
                None,
            )
            if callable(unsubscribe):
                unsubscribe(subscription)
        self.physics_sub = None

    def reconcile_stale_owned_joint(self):
        reconcile = getattr(self.grasp_manager, "reconcile_stale_owned_joint", None)
        return True if not callable(reconcile) else bool(reconcile())

    def reset_for_simulation_stop(self):
        self._finalize_release_record("simulation_stop_before_release_confirmation")
        reset = getattr(self.grasp_manager, "reset_for_simulation_stop", None)
        if not callable(reset):
            return False
        return bool(reset())

    def shutdown(self):
        self._unsubscribe()
        self._finalize_release_record(
            "runtime_session_shutdown_before_release_confirmation"
        )
        shutdown = getattr(self.grasp_manager, "shutdown", None)
        result = True if not callable(shutdown) else bool(shutdown())
        finalize = getattr(self.grasp_manager, "finalize_capture_diagnostic", None)
        if callable(finalize):
            finalize("runtime_session_shutdown_before_step_window_complete")
        close_writer = getattr(self.capture_diagnostic_writer, "close", None)
        if callable(close_writer) and close_writer() is False:
            LOGGER.warning(
                "Capture diagnostic writer did not drain before shutdown "
                "dropped=%s write_failures=%s",
                getattr(self.capture_diagnostic_writer, "dropped_count", "unavailable"),
                getattr(
                    self.capture_diagnostic_writer, "write_failure_count", "unavailable"
                ),
            )
        return result

    def close_diagnostic_writer(self):
        """Drain current session evidence without tearing down the Kit runtime."""
        writer = self.capture_diagnostic_writer
        self.capture_diagnostic_writer = None
        configure = getattr(self.grasp_manager, "set_diagnostic_observers", None)
        if callable(configure):
            try:
                configure(None, None)
            except Exception:
                pass
        observer = self.physx_observer
        self.physx_observer = None
        close_observer = getattr(observer, "close", None)
        if callable(close_observer):
            try:
                close_observer()
            except Exception as error:
                LOGGER.warning("PhysX observer close failed: %s", error)
        close = getattr(writer, "close", None)
        if callable(close):
            try:
                return close(timeout=0.25)
            except Exception as error:
                LOGGER.warning("Capture diagnostic writer close failed: %s", error)
        return True


def create_gripper_runtime_session(
    physics_interface,
    capture_config=None,
    capture_reference_config=None,
    previous_session=None,
    initial_status_sequence=0,
    registry=None,
    diagnostic_output_directory=None,
):
    """Construct and subscribe the stable Script Editor runtime session."""
    if previous_session is not None:
        previous_session.shutdown()
    session = GripperRuntimeSession(
        physics_interface,
        capture_config=capture_config,
        capture_reference_config=capture_reference_config,
        registry=registry,
        initial_status_sequence=initial_status_sequence,
        diagnostic_output_directory=diagnostic_output_directory,
    )
    # Keep the startup baseline explicit at the factory boundary as well as in
    # the session constructor.  This remains before the physics subscription,
    # so the first callback can only publish generation >= 1.
    session._feedback_generation = 0
    session.gripper.feedback_generation = 0
    session.subscribe()
    return session
