"""Play-time lifecycle for the scripted Robotiq actuator and Isaac adapter."""

import logging
import time

from graph_builder.graph_contract import GRAPH_PATH
from gripper_runtime.action_graph_command_adapter import SimGripperAdapter
from gripper_runtime.grasp_attachment import (
    EligibleObjectRegistry,
    GraspManager,
    HoldingState,
)
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
        self.reconcile_stale_owned_joint()

    def on_physics_step(self, dt):
        command_applied = self.sim_adapter.update()
        if command_applied:
            note_command = getattr(
                self.grasp_manager, "note_gripper_command_applied", None
            )
            if callable(note_command):
                note_command()
        command_is_closing = getattr(self.sim_adapter, "last_command_is_closing", None)
        if command_applied and command_is_closing is True:
            arm_close = getattr(self.grasp_manager, "arm_close_transaction", None)
            if callable(arm_close):
                result = arm_close()
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
        self._feedback_generation += 1
        self.gripper.feedback_generation = self._feedback_generation
        self.grasp_manager.update(dt)
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
        )

    def subscribe(self):
        self._unsubscribe()
        self.physics_sub = self._physics_interface.subscribe_physics_step_events(
            self.on_physics_step
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

    def shutdown(self):
        self._unsubscribe()
        shutdown = getattr(self.grasp_manager, "shutdown", None)
        if not callable(shutdown):
            return True
        return bool(shutdown())


def create_gripper_runtime_session(
    physics_interface,
    capture_config=None,
    capture_reference_config=None,
    previous_session=None,
    initial_status_sequence=0,
    registry=None,
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
    )
    # Keep the startup baseline explicit at the factory boundary as well as in
    # the session constructor.  This remains before the physics subscription,
    # so the first callback can only publish generation >= 1.
    session._feedback_generation = 0
    session.gripper.feedback_generation = 0
    session.subscribe()
    return session
