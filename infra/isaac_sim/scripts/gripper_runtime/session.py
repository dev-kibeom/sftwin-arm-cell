"""Play-time lifecycle for the selected Tool actuator and Isaac adapter."""

import logging
import time
from pathlib import Path

from graph_builder.graph_contract import GRAPH_PATH
from isaac_runtime.run_records import diagnostic_component_directory, run_records_root
from isaac_runtime.environment_setup import resolve_diagnostics_selection

from gripper_runtime.action_graph_command_adapter import SimGripperAdapter
from gripper_runtime.capture_diagnostics import (
    BoundedCaptureDiagnosticWriter,
    IsaacGripperDiagnosticRecord,
    IsaacPhysXObserver,
)
from gripper_runtime.grasp_attachment import GraspManager, HoldingState
from gripper_runtime.grasp_stage import EligibleObjectRegistry
from gripper_runtime.tool_plugin import ToolIntent, select_tool_runtime

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
        actuator_cls=None,
        adapter_cls=SimGripperAdapter,
        grasp_manager_cls=GraspManager,
        capture_config=None,
        capture_reference_config=None,
        registry=None,
        initial_status_sequence=0,
        diagnostic_writer=None,
        physx_observer=None,
        diagnostic_output_directory=None,
        tool=None,
        configuration=None,
    ):
        self.grasp_config = capture_config
        self.registry = registry or EligibleObjectRegistry()
        if tool is None and actuator_cls is None:
            from isaac_runtime.runtime_configuration import load_runtime_configuration
            from isaac_runtime.robot_binding import RobotBinding
            configuration = configuration if configuration is not None else load_runtime_configuration()
            tool = select_tool_runtime(
                configuration['tool'], RobotBinding.from_runtime(configuration)
            )
        self.tool = tool
        self.gripper = tool.actuator if tool is not None else actuator_cls(verbose=True)
        self.sim_adapter = adapter_cls(
            gripper=self.gripper,
            graph_path=GRAPH_PATH,
            move_duration=tool.move_duration if tool is not None else 0.40,
            **({"tool": tool} if tool is not None else {}),
            verbose=True,
        )
        prime_generation = getattr(self.sim_adapter, "prime_command_generation", None)
        if callable(prime_generation):
            prime_generation()
        manager_factory = (tool.grasp_factory if tool is not None else None) or grasp_manager_cls
        self.grasp_manager = manager_factory(
            gripper=self.gripper,
            registry=self.registry,
            config=self.grasp_config,
            capture_reference_config=capture_reference_config,
            **({"tool": tool} if tool is not None else {}),
            verbose=True,
        )
        selected_diagnostics = resolve_diagnostics_selection()
        diagnostics_enabled = True  # Required lightweight Capture/Release evidence.
        detailed_enabled = "motion" in selected_diagnostics
        self.capture_diagnostics_enabled = diagnostics_enabled
        self.capture_diagnostic_writer = diagnostic_writer
        self.physx_observer = physx_observer
        output_directory = diagnostic_output_directory or (
            diagnostic_component_directory(Path(__file__).resolve().parents[4], "gripper")
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
        self._active_release_record = None
        self._last_release_joint_owned = None
        self._last_release_holding_state = None
        self.reconcile_stale_owned_joint()

    def on_physics_step(self, dt):
        self._physics_step_sequence += 1
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
        intent = getattr(self.sim_adapter, "last_intent", None)
        if command_applied and intent is ToolIntent.STOP:
            self.grasp_manager.cancel_tool_transaction()
        command_width = getattr(self.sim_adapter, "last_command_width_mm", None)
        maximum_width = getattr(self.sim_adapter, "MAX_WIDTH_MM", None)
        if maximum_width is None:
            maximum_width = getattr(self.gripper, "MAX_WIDTH_MM", None)
        is_open_command = command_applied and intent is ToolIntent.RELEASE
        is_open_command = is_open_command or (
            command_applied
            and command_width is not None
            and maximum_width is not None
            and float(command_width) >= maximum_width - 1e-6
        )
        if is_open_command and command_width is not None and self.capture_diagnostics_enabled:
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
        close_rejected = False
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
                            "close_failure_reason",
                            None,
                        )
                        or "close_transaction_arm_failed"
                    )
                    self.grasp_manager.reject_close_transaction(reason)
                    close_rejected = True
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
        if not close_rejected:
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
        observe_diagnostics = getattr(self.grasp_manager, 'observe_capture_diagnostic_step', None)
        if callable(observe_diagnostics):
            observe_diagnostics(dt, self._physics_step_sequence)
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
        attached = runtime_ready and bool(self._safe_attachment_owned())
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
            width_mm=self.tool.width_mm() if self.tool is not None else self.gripper.get_width(),
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
            observe = getattr(self.grasp_manager, 'attachment_owned', None)
            if callable(observe):
                return bool(observe())
            return False
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
            owned_joint = self._safe_attachment_owned()
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
    configuration=None,
):
    """Construct and subscribe the stable Script Editor runtime session."""
    if previous_session is not None:
        previous_session.shutdown()
    session = GripperRuntimeSession(
        physics_interface,
        configuration=configuration,
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
