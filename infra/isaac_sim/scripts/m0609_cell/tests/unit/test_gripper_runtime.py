from pathlib import Path
from types import SimpleNamespace

import pytest
from gripper_runtime.action_graph_command_adapter import SimGripperAdapter
from gripper_runtime.grasp_attachment import HoldingState
from gripper_runtime.grasp_policy import CaptureConfig
from gripper_runtime.robotiq_actuator import GripperState, Robotiq2F85Actuator
from gripper_runtime.session import GripperRuntimeSession, initial_status_sequence
from pnp_validation.run_validation import (
    ValidationStartPending,
    ValidationStartRejected,
    capture_config_from_profile,
    prepare_validation_start_state,
    request_validation_start_state,
    validation_start_state_ready,
)


def explicit_capture_config():
    return CaptureConfig(
        capture_volume_dimensions_m=(0.10, 0.10, 0.10),
    )


def test_runtime_reload_sequence_floor_exceeds_existing_motion_consumer_counter():
    assert initial_status_sequence(24614, monotonic_ns=9876543210000) == 9876543210000
    assert initial_status_sequence(9876543210000, monotonic_ns=24614) == 9876543210000


@pytest.mark.parametrize("state", ["held", "unknown"])
def test_validation_start_rejects_non_released_gripper(state):
    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value=state), fresh=lambda: True
        ),
        gripper=SimpleNamespace(open=lambda: None),
    )

    with pytest.raises(RuntimeError, match="fresh RELEASED"):
        prepare_validation_start_state(manager)


def test_validation_start_allows_fresh_released_and_detached_gripper():
    feedback = {"generation": 1}

    def open_gripper():
        feedback["generation"] += 1

    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(
            open=open_gripper,
            is_moving=False,
            get_position=lambda: 0.0,
            get_width=lambda: 85.0,
            feedback_generation=feedback["generation"],
        ),
    )

    manager.gripper.open = lambda: (
        feedback.__setitem__("generation", feedback["generation"] + 1),
        setattr(manager.gripper, "feedback_generation", feedback["generation"]),
    )

    assert prepare_validation_start_state(manager) is True


def test_validation_start_rejects_released_but_physically_closed_gripper():
    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(
            open=lambda: None,
            is_moving=False,
            get_position=lambda: 0.8,
            get_width=lambda: 0.0,
            feedback_generation=1,
        ),
    )

    manager.gripper.open = lambda: setattr(manager.gripper, "feedback_generation", 2)

    with pytest.raises(RuntimeError, match="physical OPEN"):
        prepare_validation_start_state(manager)


def test_validation_start_requires_fresh_physical_open_feedback():
    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(
            open=lambda: None,
            is_moving=False,
            get_position=lambda: 0.0,
            get_width=lambda: 85.0,
            feedback_generation=1,
            feedback_generation_before_open=1,
        ),
    )

    with pytest.raises(RuntimeError, match="fresh physical joint-state"):
        prepare_validation_start_state(manager)


def test_validation_start_pending_until_new_feedback_generation_arrives():
    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(
            is_moving=True,
            get_position=lambda: 0.5,
            get_width=lambda: 42.5,
            feedback_generation=4,
        ),
    )

    with pytest.raises(ValidationStartPending):
        validation_start_state_ready(manager, baseline_generation=4)


def test_validation_start_terminal_failures_do_not_use_pending_exception():
    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="unknown"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(
            is_moving=False,
            get_position=lambda: 0.0,
            get_width=lambda: 85.0,
            feedback_generation=5,
        ),
    )

    with pytest.raises(ValidationStartRejected):
        validation_start_state_ready(manager, baseline_generation=4)


@pytest.mark.parametrize(
    "gripper",
    [
        SimpleNamespace(
            is_moving=False,
            feedback_generation=5,
        ),
        SimpleNamespace(
            get_position=lambda: 0.0,
            feedback_generation=5,
        ),
        SimpleNamespace(
            get_position=lambda: 0.0,
            is_moving=False,
            feedback_generation="not-a-generation",
        ),
    ],
)
def test_validation_start_missing_or_invalid_physical_feedback_is_terminal(gripper):
    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"), fresh=lambda: True
        ),
        gripper=gripper,
    )

    with pytest.raises(ValidationStartRejected):
        validation_start_state_ready(manager, baseline_generation=4)


def test_validation_start_freshness_api_failure_is_terminal():
    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"),
            fresh=lambda: (_ for _ in ()).throw(RuntimeError("freshness failed")),
        ),
        gripper=SimpleNamespace(
            is_moving=False,
            get_position=lambda: 0.0,
            feedback_generation=5,
        ),
    )

    with pytest.raises(ValidationStartRejected):
        validation_start_state_ready(manager, baseline_generation=4)


def test_open_command_failure_is_terminal_validation_start_rejection():
    manager = SimpleNamespace(
        attach_joint_path=None,
        gripper=SimpleNamespace(
            feedback_generation=1,
            open=lambda: (_ for _ in ()).throw(RuntimeError("command failed")),
        ),
    )

    with pytest.raises(ValidationStartRejected):
        request_validation_start_state(manager)


def test_validation_start_unknown_observation_fails_closed_even_when_open():
    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="unknown"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(
            open=lambda: None,
            is_moving=False,
            get_position=lambda: 0.0,
            get_width=lambda: 85.0,
            feedback_generation=1,
        ),
    )
    manager.gripper.open = lambda: setattr(manager.gripper, "feedback_generation", 2)

    with pytest.raises(RuntimeError, match="fresh RELEASED"):
        prepare_validation_start_state(manager)


def test_validation_start_accepts_fresh_physical_open_feedback():
    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(
            open=lambda: None,
            is_moving=False,
            get_position=lambda: 0.0,
            get_width=lambda: 85.0,
            feedback_generation=1,
        ),
    )

    manager.gripper.open = lambda: setattr(manager.gripper, "feedback_generation", 2)

    assert prepare_validation_start_state(manager) is True


def test_validation_start_uses_parent_joint_as_authority_not_derived_width():
    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(
            open=lambda: None,
            is_moving=False,
            get_position=lambda: 0.0,
            get_width=lambda: 0.0,
            feedback_generation=1,
        ),
    )
    manager.gripper.open = lambda: setattr(manager.gripper, "feedback_generation", 2)

    assert prepare_validation_start_state(manager) is True


def test_validation_start_rejects_existing_owned_joint():
    manager = SimpleNamespace(
        attach_joint_path="/World/SFTwin/ValidationAttachment",
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(open=lambda: None),
    )

    with pytest.raises(RuntimeError, match="owned grasp joint"):
        prepare_validation_start_state(manager)


def test_validation_start_open_failure_fails_closed():
    def fail_open():
        raise RuntimeError("open failed")

    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(open=fail_open),
    )

    with pytest.raises(RuntimeError, match="open"):
        prepare_validation_start_state(manager)


def test_validation_start_rejects_attached_observation():
    manager = SimpleNamespace(
        attach_joint_path=None,
        attached=True,
        holding_observation=lambda: SimpleNamespace(
            attached=True,
            state=SimpleNamespace(value="released"),
            fresh=lambda: True,
        ),
        gripper=SimpleNamespace(open=lambda: None),
    )

    with pytest.raises(RuntimeError, match="attached"):
        prepare_validation_start_state(manager)


def test_validation_start_is_safe_to_repeat_after_valid_initialization():
    calls = []
    feedback = {"generation": 1}

    def open_gripper():
        calls.append("open")
        feedback["generation"] += 1
        manager.gripper.feedback_generation = feedback["generation"]

    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(
            open=open_gripper,
            is_moving=False,
            get_position=lambda: 0.0,
            get_width=lambda: 85.0,
            feedback_generation=feedback["generation"],
        ),
    )

    assert prepare_validation_start_state(manager) is True
    assert prepare_validation_start_state(manager) is True
    assert calls == ["open", "open"]


def test_actuator_pure_command_conversions_and_interpolation_boundaries():
    assert Robotiq2F85Actuator._clamp01(-2.0) == 0.0
    assert Robotiq2F85Actuator._clamp01(2.0) == 1.0
    assert Robotiq2F85Actuator._smoothstep01(0.0) == 0.0
    assert Robotiq2F85Actuator._smoothstep01(1.0) == 1.0
    assert Robotiq2F85Actuator.position_to_width(0.0) == 85.0
    assert Robotiq2F85Actuator.position_to_width(1.0) == 0.0
    assert Robotiq2F85Actuator.width_to_position(85.0) == 0.0
    assert Robotiq2F85Actuator.width_to_position(0.0) == 1.0


def test_actuator_cancel_motion_clears_trajectory_without_physics_access():
    actuator = Robotiq2F85Actuator.__new__(Robotiq2F85Actuator)
    actuator.state = GripperState.CLOSING
    actuator._motion_start_q = 0.2
    actuator._motion_target_q = 0.8
    actuator._motion_duration = 0.4
    actuator._motion_elapsed = 0.1

    actuator.cancel_motion()

    assert actuator.state is GripperState.IDLE
    assert actuator._motion_start_q is None
    assert actuator._motion_target_q is None
    assert actuator._motion_duration == 0.0
    assert actuator._motion_elapsed == 0.0


class _Attribute:
    def __init__(self, value, valid=True):
        self.value = value
        self.valid = valid

    def is_valid(self):
        return self.valid


class _Controller:
    def __init__(self, attribute):
        self.attribute_value = attribute
        self.paths = []

    def attribute(self, path):
        self.paths.append(path)
        return self.attribute_value

    def set(self, attribute, value):
        attribute.value = value

    @staticmethod
    def get(attribute):
        return attribute.value


class _GenerationController(_Controller):
    def __init__(
        self,
        attribute,
        generation_attribute,
        capture_reference_attribute=None,
        close_generation_attribute=None,
        close_id_attribute=None,
    ):
        super().__init__(attribute)
        self.generation_attribute = generation_attribute
        self.capture_reference_attribute = capture_reference_attribute
        self.close_generation_attribute = close_generation_attribute or _Attribute(0)
        self.close_id_attribute = close_id_attribute or _Attribute("")

    def attribute(self, path):
        self.paths.append(path)
        if "GripperCloseGeneration" in path:
            return self.close_generation_attribute
        if path.endswith("SubscribeGripperClose.outputs:data"):
            return self.close_id_attribute
        if "GripperCommandGeneration" in path:
            return self.generation_attribute
        if path.endswith("PublishGripperStatus.inputs:data"):
            return _Attribute("")
        if path.endswith("SubscribeGripperCaptureReference.outputs:data"):
            return self.capture_reference_attribute or _Attribute([])
        return self.attribute_value


class _Gripper:
    def __init__(self):
        self.commands = []

    def set_width(self, width_mm, duration=None):
        self.commands.append((width_mm, duration))

    def close(self, duration=None):
        self.commands.append(("close", duration))


class _WidthGripper(_Gripper):
    def __init__(self, width_mm):
        super().__init__()
        self.width_mm = width_mm

    def get_width(self):
        return self.width_mm

    def set_width(self, width_mm, duration=None):
        super().set_width(width_mm, duration)
        self.width_mm = width_mm


def test_action_graph_command_adapter_keeps_attribute_path_clamping_and_deduplication():
    attribute = _Attribute(100.0)
    controller = _GenerationController(attribute, _Attribute(1))
    gripper = _Gripper()
    adapter = SimGripperAdapter(
        gripper,
        graph_path="/Graph",
        subscriber_node="Subscriber",
        move_duration=0.4,
        controller=controller,
    )

    assert controller.paths == [
        "/Graph/Subscriber.outputs:data",
        "/Graph/GripperCommandGeneration.outputs:count",
        "/Graph/GripperCloseGeneration.outputs:count",
        "/Graph/PublishGripperStatus.inputs:data",
        "/Graph/SubscribeGripperCaptureReference.outputs:data",
    ]
    assert adapter.update() is True
    assert gripper.commands == [(85.0, 0.4)]
    assert adapter.update() is False


def test_action_graph_command_adapter_does_not_replay_retained_startup_value():
    attribute = _Attribute(0.0)
    generation = _Attribute(0)
    controller = _GenerationController(attribute, generation)
    gripper = _Gripper()
    adapter = SimGripperAdapter(gripper, graph_path="/Graph", controller=controller)

    adapter.prime_command_generation()

    assert adapter.update() is False
    assert gripper.commands == []
    attribute.value = 70.0
    generation.value = 1
    assert adapter.update() is True
    assert gripper.commands == [(70.0, None)]


def test_action_graph_command_adapter_publishes_backend_status_payload():
    attribute = _Attribute(0.0)
    controller = _GenerationController(attribute, _Attribute(1))
    status_attribute = _Attribute("")
    controller.attribute = lambda path: (
        status_attribute
        if "PublishGripperStatus" in path
        else (
            controller.generation_attribute
            if path.endswith(".outputs:count")
            else attribute
        )
    )
    gripper = _Gripper()
    adapter = SimGripperAdapter(
        gripper, graph_path="/Graph", move_duration=0.4, controller=controller
    )

    adapter.publish_status(
        runtime_ready=True,
        grasp_confirmed=True,
        attached=True,
        released=False,
        width_mm=12.5,
        sequence=7,
    )

    assert status_attribute.value == (
        "ready=1;grasp=1;attached=1;released=0;width_mm=12.500;seq=7;capture_reference_id="
    )
    attribute.value = -1.0
    assert adapter.update() is True
    assert gripper.commands[-1] == (0.0, 0.4)
    attribute.value = float("nan")
    assert adapter.update() is False


def test_action_graph_command_adapter_identifies_close_direction_at_boundary():
    attribute = _Attribute(70.0)
    controller = _GenerationController(attribute, _Attribute(1))
    gripper = _WidthGripper(71.0)
    adapter = SimGripperAdapter(gripper, controller=controller)

    assert adapter.update() is True
    assert adapter.last_command_is_closing is True

    attribute.value = 85.0
    controller.generation_attribute.value = 2
    assert adapter.update() is True
    assert adapter.last_command_is_closing is False


def test_logical_close_uses_actuator_close_without_width_position_target():
    attribute = _Attribute(70.0)
    controller = _GenerationController(
        attribute, _Attribute(0), close_generation_attribute=_Attribute(0)
    )
    gripper = _Gripper()
    adapter = SimGripperAdapter(
        gripper, graph_path="/Graph", move_duration=0.4, controller=controller
    )
    adapter.prime_command_generation()
    controller.close_id_attribute.value = "pick-123"
    controller.close_generation_attribute.value = 1

    assert adapter.update() is False
    assert gripper.commands == []
    adapter.mark_capture_reference_applied("pick-123")
    controller.close_generation_attribute.value = 2
    assert adapter.update() is True
    assert gripper.commands == [("close", 0.4)]
    assert adapter.last_command_width_mm is None
    assert adapter.last_command_is_closing is True


@pytest.mark.parametrize("order", [0, 1, 2, 4])
def test_capture_reference_preserves_selected_tcp_and_expected_contact_width(order):
    values = [
        0.1,
        0.2,
        0.3,
        0.0,
        0.0,
        0.7071067811865476,
        0.7071067811865476,
        80.0,
        float(order),
    ]
    import json

    packet = {
        "motion_goal_uuid": "pick-123",
        "frame_id": "base_link",
        "position": values[:3],
        "orientation_xyzw": values[3:7],
        "expected_contact_width_mm": values[7],
        "orientation_symmetry_order": values[8],
    }
    controller = _GenerationController(
        _Attribute(-1.0), _Attribute(1), _Attribute(json.dumps(packet))
    )
    adapter = SimGripperAdapter(_Gripper(), graph_path="/Graph", controller=controller)

    assert adapter.read_capture_reference() == {
        "motion_goal_uuid": "pick-123",
        "frame_id": "base_link",
        "values": tuple(values),
    }
    assert adapter.read_capture_reference() is None
    controller.capture_reference_attribute.value = json.dumps(
        {"motion_goal_uuid": "pick-123", "invalidate": True}
    )
    assert adapter.read_capture_reference() == {
        "motion_goal_uuid": "pick-123",
        "invalidate": True,
    }
    assert adapter.read_capture_reference() is None


def test_capture_reference_refreshes_changed_candidate_with_same_motion_goal():
    import json

    first = {
        "motion_goal_uuid": "pick-123",
        "frame_id": "base_link",
        "position": [0.1, 0.2, 0.3],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        "expected_contact_width_mm": 80.0,
        "orientation_symmetry_order": 2,
    }
    controller = _GenerationController(
        _Attribute(-1.0), _Attribute(1), _Attribute(json.dumps(first))
    )
    adapter = SimGripperAdapter(_Gripper(), graph_path="/Graph", controller=controller)

    assert adapter.read_capture_reference()["values"][7] == 80.0

    second = dict(first)
    second["position"] = [0.4, 0.5, 0.6]
    second["orientation_xyzw"] = [0.0, 0.0, 1.0, 0.0]
    second["expected_contact_width_mm"] = 70.0
    controller.capture_reference_attribute.value = json.dumps(second)

    refreshed = adapter.read_capture_reference()
    assert refreshed["motion_goal_uuid"] == "pick-123"
    assert refreshed["values"] == (0.4, 0.5, 0.6, 0.0, 0.0, 1.0, 0.0, 70.0, 2.0)
    assert adapter.read_capture_reference() is None


def test_close_rejects_reference_from_previous_pick():
    controller = _GenerationController(
        _Attribute(-1.0),
        _Attribute(0),
        close_generation_attribute=_Attribute(0),
        close_id_attribute=_Attribute("pick-old"),
    )
    gripper = _Gripper()
    adapter = SimGripperAdapter(gripper, controller=controller)
    adapter.prime_command_generation()
    adapter.mark_capture_reference_applied("pick-new")
    controller.close_generation_attribute.value = 1

    assert adapter.update() is False
    assert gripper.commands == []


def test_repeated_identical_width_with_new_ingress_generation_is_applied_once():
    attribute = _Attribute(70.0)
    generation = _Attribute(1)
    controller = _GenerationController(attribute, generation)
    gripper = _WidthGripper(85.0)
    adapter = SimGripperAdapter(gripper, controller=controller)

    assert adapter.update() is True
    assert gripper.commands == [(70.0, None)]
    assert adapter.last_command_generation == 1
    assert adapter.last_command_is_closing is True

    assert adapter.update() is False
    assert gripper.commands == [(70.0, None)]

    gripper.width_mm = 85.0
    generation.value = 2
    assert adapter.update() is True
    assert gripper.commands == [(70.0, None), (70.0, None)]
    assert adapter.last_command_generation == 2
    assert adapter.last_command_is_closing is True


def test_same_width_new_generation_uses_actual_width_for_direction():
    attribute = _Attribute(70.0)
    generation = _Attribute(1)
    controller = _GenerationController(attribute, generation)
    gripper = _WidthGripper(70.0)
    adapter = SimGripperAdapter(gripper, controller=controller)

    assert adapter.update() is True
    assert adapter.last_command_is_closing is False

    generation.value = 2
    gripper.width_mm = 85.0
    assert adapter.update() is True
    assert adapter.last_command_is_closing is True


class _Physics:
    def __init__(self):
        self.callback = None
        self.subscriptions = []
        self.step_subscription = None

    def subscribe_physics_on_step_events(self, callback, pre_step, order):
        self.callback = callback
        self.step_subscription = (pre_step, order)
        subscription = _Subscription(callback)
        self.subscriptions.append(subscription)
        return subscription


class _Subscription:
    def __init__(self, callback):
        self.callback = callback
        self.unsubscribe_calls = 0

    def unsubscribe(self):
        self.unsubscribe_calls += 1


class _RuntimeGripper:
    def __init__(self, verbose):
        self.events = []

    def update(self, dt):
        self.events.append(("gripper", dt))

    def get_width(self):
        return 0.0


class _RuntimeAdapter:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.events = []

    def update(self):
        self.events.append("adapter")

    def publish_status(self, **kwargs):
        self.events.append(("status", kwargs))


class _ImmediateCloseGripper(_RuntimeGripper):
    def __init__(self, verbose):
        super().__init__(verbose)
        self.state = SimpleNamespace(value="holding")


class _ImmediateCloseAdapter(_RuntimeAdapter):
    last_command_is_closing = True

    def update(self):
        self.events.append("adapter")
        return True


class _ImmediateOpenAdapter(_RuntimeAdapter):
    last_command_is_closing = False
    last_command_generation = 17
    last_command_width_mm = 85.0

    def update(self):
        self.events.append("adapter")
        return True


class _RuntimeAttachment:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.events = []
        self.is_grasped = False
        self.attach_joint_path = None
        self.ready = kwargs.get("config") is not None
        self.reconcile_calls = 0
        self.shutdown_calls = 0
        self.holding_sequence = 1
        self.capture_diagnostic_writer = None
        self.physx_observer = None

    def set_diagnostic_observers(self, writer=None, physx_observer=None):
        self.capture_diagnostic_writer = writer
        self.physx_observer = physx_observer

    def update(self, dt):
        self.events.append(("attachment", dt))

    def holding_observation(self):
        return SimpleNamespace(
            state=HoldingState.RELEASED, sequence=self.holding_sequence
        )

    def note_gripper_command_applied(self):
        self.holding_sequence += 1

    def note_open_release_command_applied(self, generation, width_mm):
        self.events.append(("release_intent", generation, width_mm))

    def reconcile_stale_owned_joint(self):
        self.reconcile_calls += 1
        return True

    def shutdown(self):
        self.shutdown_calls += 1
        self.ready = False
        return True


class _ArmingRuntimeAttachment(_RuntimeAttachment):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.arm_calls = 0

    def arm_close_transaction(self, execution_id=None, command_generation=None):
        self.arm_calls += 1
        return True


def test_session_preserves_runtime_configuration_and_physics_tick_order(
    monkeypatch, caplog
):
    caplog.set_level("INFO", logger="gripper_runtime.session")
    monkeypatch.delenv("SFTWIN_DIAGNOSTICS", raising=False)

    def unexpected_observer_creation():
        raise AssertionError("default session must not create PhysX observer")

    monkeypatch.setattr(
        "gripper_runtime.session.IsaacPhysXObserver", unexpected_observer_creation
    )
    writers = []
    monkeypatch.setattr(
        "gripper_runtime.session.BoundedCaptureDiagnosticWriter",
        lambda *args, **kwargs: writers.append((args, kwargs))
        or SimpleNamespace(close=lambda **_kw: True),
    )
    physics = _Physics()
    session = GripperRuntimeSession(
        physics,
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        capture_config=explicit_capture_config(),
    )

    assert session.gripper.feedback_generation == 0
    assert session.capture_diagnostic_writer is not None
    assert len(writers) == 1
    assert session.physx_observer is None
    assert (
        session.sim_adapter.kwargs["graph_path"]
        == "/World/SF_Twin_Cell/ROS2/ActionGraph"
    )
    assert session.sim_adapter.kwargs["move_duration"] == 0.40
    assert "registry" in session.grasp_manager.kwargs
    assert "target_path" not in session.grasp_manager.kwargs
    first_subscription = session.subscribe()
    assert first_subscription is physics.subscriptions[0]
    assert session.grasp_manager.reconcile_calls == 1
    assert physics.callback == session.on_physics_step
    assert physics.step_subscription == (True, 0)

    session.on_physics_step(0.25)
    assert session.gripper.feedback_generation == 1
    assert session.sim_adapter.events == [
        "adapter",
        (
            "status",
            {
                "runtime_ready": True,
                "grasp_confirmed": False,
                "attached": False,
                "released": True,
                "width_mm": 0.0,
                "sequence": 1,
                "capture_reference_id": "",
            },
        ),
    ]
    assert session.gripper.events == [("gripper", 0.25)]
    assert session.grasp_manager.events == [("attachment", 0.25)]
    assert session.shutdown() is True


def test_session_diagnostics_opt_in_initializes_physx_observer(monkeypatch, caplog):
    prepared = []
    writer_paths = []

    class _Writer:
        def close(self):
            pass

    class _Observer:
        def prepare(self, paths):
            prepared.extend(paths)

    monkeypatch.setenv("SFTWIN_DIAGNOSTICS", "motion")
    monkeypatch.setattr("gripper_runtime.session.LOGGER.propagate", True)
    caplog.set_level("DEBUG", logger="gripper_runtime.session")
    monkeypatch.setattr(
        "gripper_runtime.session.BoundedCaptureDiagnosticWriter",
        lambda path: writer_paths.append(Path(path)) or _Writer(),
    )
    monkeypatch.setattr("gripper_runtime.session.IsaacPhysXObserver", _Observer)
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
    )
    assert session.capture_diagnostics_enabled is True
    assert isinstance(session.physx_observer, _Observer)
    assert prepared == [""]
    assert writer_paths == [
        Path(__file__).resolve().parents[6]
        / ".local_artifacts"
        / "capture_diagnostic_records"
    ]
    diagnostic_init = [
        record
        for record in caplog.records
        if record.name == "gripper_runtime.session"
        and "Gripper lightweight capture diagnostics" in record.message
    ]
    assert len(diagnostic_init) == 1
    assert diagnostic_init[0].levelname == "DEBUG"
    assert session.shutdown() is True


def test_gripper_diagnostics_selection_does_not_require_debug_logging(
    monkeypatch, caplog
):
    caplog.set_level("INFO", logger="gripper_runtime.session")
    monkeypatch.setenv("SFTWIN_DIAGNOSTICS", "motion")
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
    )
    assert session.capture_diagnostics_enabled is True
    assert session.capture_diagnostic_writer is not None
    assert session.physx_observer is not None


def test_debug_logging_alone_keeps_baseline_capture_without_physx(monkeypatch, caplog):
    caplog.set_level("DEBUG", logger="gripper_runtime.session")
    monkeypatch.delenv("SFTWIN_DIAGNOSTICS", raising=False)
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
    )
    assert session.capture_diagnostics_enabled is True
    assert session.capture_diagnostic_writer is not None
    assert session.physx_observer is None


def test_non_motion_selection_keeps_baseline_capture_without_physx(monkeypatch, caplog):
    caplog.set_level("DEBUG", logger="gripper_runtime.session")
    monkeypatch.setenv("SFTWIN_DIAGNOSTICS", "vision,safety")
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
    )
    assert session.capture_diagnostics_enabled is True
    assert session.capture_diagnostic_writer is not None
    assert session.physx_observer is None


def test_capture_writer_construction_failure_does_not_change_gripper_runtime(
    monkeypatch,
):
    monkeypatch.delenv("SFTWIN_DIAGNOSTICS", raising=False)
    monkeypatch.setattr(
        "gripper_runtime.session.BoundedCaptureDiagnosticWriter",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("read only")),
    )
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
    )
    assert session.capture_diagnostics_enabled is True
    assert session.capture_diagnostic_writer is None
    session.on_physics_step(0.25)
    assert session.gripper.feedback_generation == 1
    assert session.shutdown() is True


def test_stop_can_close_session_writer_without_shutting_down_gripper():
    class _Writer:
        closed = False

        def close(self, timeout=1.0):
            self.closed = True
            return True

    writer = _Writer()
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        diagnostic_writer=writer,
    )
    subscription = session.subscribe()
    assert session.close_diagnostic_writer() is True
    assert writer.closed is True
    assert session.capture_diagnostic_writer is None
    assert session.gripper.feedback_generation == 0
    assert session.physics_sub is subscription


def test_stop_detaches_and_closes_detailed_observer():
    class _Observer:
        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True

    observer = _Observer()
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        physx_observer=observer,
    )
    assert session.close_diagnostic_writer() is True
    assert observer.closed is True
    assert session.physx_observer is None
    assert session.grasp_manager.physx_observer is None
    assert session.grasp_manager.capture_diagnostic_writer is None


def test_stop_play_reconnects_fresh_session_writer_and_physx_observer(
    tmp_path, monkeypatch
):
    monkeypatch.setenv("SFTWIN_DIAGNOSTICS", "motion")
    writers = []
    observers = []

    class _Writer:
        def __init__(self, path):
            self.path = Path(path)
            self.closed = False

        def close(self, timeout=1.0):
            self.closed = True
            return True

    class _Observer:
        def prepare(self, _paths):
            pass

    def writer_factory(path):
        writer = _Writer(path)
        writers.append(writer)
        return writer

    def observer_factory():
        observer = _Observer()
        observers.append(observer)
        return observer

    monkeypatch.setattr(
        "gripper_runtime.session.BoundedCaptureDiagnosticWriter", writer_factory
    )
    monkeypatch.setattr("gripper_runtime.session.IsaacPhysXObserver", observer_factory)
    first = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        diagnostic_output_directory=tmp_path / "session-one" / "gripper",
    )
    first.close_diagnostic_writer()
    second = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        diagnostic_output_directory=tmp_path / "session-two" / "gripper",
    )

    assert writers[0].closed
    assert writers[0] is not writers[1]
    assert writers[0].path != writers[1].path
    assert second.capture_diagnostic_writer is writers[1]
    assert second.grasp_manager.capture_diagnostic_writer is writers[1]
    assert second.physx_observer is observers[1]
    assert second.grasp_manager.physx_observer is observers[1]
    assert first.physx_observer is None
    second.close_diagnostic_writer()


def test_injected_diagnostic_observers_remain_available_without_debug(
    monkeypatch, caplog
):
    prepared = []

    class _Writer:
        def close(self):
            pass

    class _Observer:
        def prepare(self, paths):
            prepared.extend(paths)

    caplog.set_level("INFO", logger="gripper_runtime.session")
    monkeypatch.delenv("SFTWIN_DIAGNOSTICS", raising=False)
    writer = _Writer()
    observer = _Observer()
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        diagnostic_writer=writer,
        physx_observer=observer,
    )
    assert session.capture_diagnostics_enabled is True
    assert session.capture_diagnostic_writer is writer
    assert session.physx_observer is observer
    assert prepared == [""]


def test_status_publish_contract_is_emitted_by_adapter_each_physics_step():
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        capture_config=explicit_capture_config(),
    )

    session.on_physics_step(0.25)
    session.on_physics_step(0.25)

    status_events = [
        event for event in session.sim_adapter.events if event[0] == "status"
    ]
    assert len(status_events) == 2
    first_status, second_status = (event[1] for event in status_events)
    assert first_status["runtime_ready"] is True
    assert first_status["grasp_confirmed"] is False
    assert first_status["attached"] is False
    assert first_status["released"] is True
    assert second_status == first_status


def test_physx_relative_pose_change_is_separate_from_stage_pose():
    before = {
        "status": "observed",
        "source": "isaacsim.core.prims.SingleRigidPrim_physx",
        "tcp": {
            "status": "observed",
            "pose_world": {"position_m": (0, 0, 0), "orientation_wxyz": (1, 0, 0, 0)},
            "linear_velocity_m_s": (0, 0, 0),
            "angular_velocity_rad_s": (0, 0, 0),
        },
        "rawpart": {
            "status": "observed",
            "pose_world": {"position_m": (1, 0, 0), "orientation_wxyz": (1, 0, 0, 0)},
            "linear_velocity_m_s": (0, 0, 0),
            "angular_velocity_rad_s": (0, 0, 0),
        },
    }
    after = {
        **before,
        "rawpart": {
            **before["rawpart"],
            "pose_world": {
                "position_m": (1, 0, 0),
                "orientation_wxyz": (0, 0, 0, 1),
            },
            "angular_velocity_rad_s": (0, 0, 3.14),
        },
    }

    relative_before = GripperRuntimeSession._relative_physx(before)
    relative_after = GripperRuntimeSession._relative_physx(after)

    change = GripperRuntimeSession._relative_change(relative_before, relative_after)
    assert relative_after["status"] == "observed"
    assert change["orientation_delta_deg"] == pytest.approx(180.0)
    assert change["angular_velocity_difference_delta_world_rad_s"] == (0, 0, 3.14)
    assert after["source"] == "isaacsim.core.prims.SingleRigidPrim_physx"
    assert "usd_stage_world" not in after["source"]


def test_session_arms_close_transaction_when_command_reaches_holding_immediately():
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_ImmediateCloseGripper,
        adapter_cls=_ImmediateCloseAdapter,
        grasp_manager_cls=_ArmingRuntimeAttachment,
        capture_config=explicit_capture_config(),
    )

    session.on_physics_step(0.25)

    assert session.grasp_manager.arm_calls == 1


@pytest.mark.parametrize("capture_configured", [False, True])
@pytest.mark.parametrize("initial_position", [0.0, 0.0005, 0.5])
@pytest.mark.parametrize("attached", [False, True])
@pytest.mark.parametrize("remove_ok", [False, True])
def test_open_session_publishes_fresh_outcome_only_after_actuator_completion(
    initial_position,
    attached,
    remove_ok,
    capture_configured,
):
    from gripper_runtime.grasp_attachment import GraspManager
    from gripper_runtime.grasp_stage import StageCaptureQuery
    from test_grasp_attachment import FakeJointOwner

    joints = FakeJointOwner()
    if attached:
        joints.create("/World/Cube")
    joints.remove_result = remove_ok
    source = StageCaptureQuery(None, "/World/TCP", None, None)
    source.require_motion_reference = True

    def actuator(**_kwargs):
        # Exercise the real interpolation/completion code with a controlled DOF.
        result = Robotiq2F85Actuator.__new__(Robotiq2F85Actuator)
        position = [initial_position]
        result.get_position = lambda: position[0]
        result._apply_position = lambda q: position.__setitem__(0, q)
        result.verbose = False
        result.state = GripperState.HOLDING
        result._cancel_motion()
        return result

    class OpenAdapter(_ImmediateOpenAdapter):
        applied = False

        def update(self):
            if self.applied:
                return False
            self.applied = True
            self.kwargs["gripper"].open(duration=0.4)
            return True

    records = []
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=actuator,
        adapter_cls=OpenAdapter,
        grasp_manager_cls=lambda **kwargs: GraspManager(
            tcp_path="/World/TCP",
            candidate_source=source,
            joint_owner=joints,
            **kwargs,
        ),
        capture_config=explicit_capture_config() if capture_configured else None,
        diagnostic_writer=SimpleNamespace(
            submit=lambda record: records.append(record) or True
        ),
        physx_observer=SimpleNamespace(),
    )
    session.grasp_manager.update(0.0)
    baseline = session.grasp_manager.holding_observation()
    session.on_physics_step(0.1)
    first_status = session.sim_adapter.events[-1][1]
    if initial_position != 0.0:
        assert session.gripper.state is GripperState.OPENING
        assert first_status["sequence"] == baseline.sequence
        assert joints.has_owned_joint() is attached
        assert records == []
    for _ in range(3):
        session.on_physics_step(0.1)
    status = session.sim_adapter.events[-1][1]
    assert session.gripper.state is GripperState.HOLDING
    assert session.gripper.get_width() == pytest.approx(85.0)
    assert status["runtime_ready"] is True
    assert status["sequence"] > baseline.sequence
    if attached and not remove_ok:
        assert not status["released"]
        assert session.grasp_manager.holding_observation().state is HoldingState.UNKNOWN
        events = session._active_release_record.data["events"]
        detach_result = next(
            e["payload"]["result"] for e in events if e["kind"] == "detach_attempt"
        )
        assert detach_result == "joint_remove_failed"
    else:
        assert status["released"]
        assert not joints.has_owned_joint()
        assert records[0]["events"][-1]["payload"]["result"] == "released"
    assert source.reference_diagnostics() is None


def test_validation_start_uses_zero_session_baseline_and_next_tick_feedback():
    manager = SimpleNamespace(
        attach_joint_path=None,
        holding_observation=lambda: SimpleNamespace(
            state=SimpleNamespace(value="released"), fresh=lambda: True
        ),
        gripper=SimpleNamespace(
            feedback_generation=0,
            is_moving=False,
            get_position=lambda: 0.0,
            open=lambda: None,
        ),
    )

    baseline = request_validation_start_state(manager)
    assert baseline == 0

    manager.gripper.feedback_generation = 1
    assert validation_start_state_ready(manager, baseline_generation=0) is True


def test_validation_profile_capture_config_is_passed_to_runtime_session():
    config = capture_config_from_profile(
        {
            "capture": {
                "volume_dimensions_m": [0.12, 0.12, 0.12],
            }
        }
    )
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        capture_config=config,
    )

    assert session.grasp_config is config
    assert session.grasp_manager.kwargs["config"] is config
    assert session.grasp_manager.kwargs["config"].capture_volume_dimensions_m == (
        0.12,
        0.12,
        0.12,
    )


def test_repeated_subscription_is_idempotent_and_unsubscribes_previous_callback():
    physics = _Physics()
    session = GripperRuntimeSession(
        physics,
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        capture_config=explicit_capture_config(),
    )

    first = session.subscribe()
    second = session.subscribe()

    assert first is not second
    assert first.unsubscribe_calls == 1
    assert second.unsubscribe_calls == 0
    assert session.physics_sub is second
    assert len(physics.subscriptions) == 2


def test_shutdown_unsubscribes_and_invalidates_runtime_owner():
    physics = _Physics()
    session = GripperRuntimeSession(
        physics,
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        capture_config=explicit_capture_config(),
    )
    subscription = session.subscribe()

    assert session.shutdown() is True
    assert subscription.unsubscribe_calls == 1
    assert session.physics_sub is None
    assert session.grasp_manager.shutdown_calls == 1


def test_session_passes_missing_capture_configuration_to_manager():
    physics = _Physics()
    session = GripperRuntimeSession(
        physics,
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
    )

    assert session.grasp_config is None


def test_production_capture_configuration_uses_rawpart_geometry_and_recipe_width():
    from gripper_runtime.operational_config import production_capture_configuration

    config, reference, registry = production_capture_configuration(
        {
            "vision": {"detector_profiles": {"RawPart": {"material_id": "RawPart"}}},
            "material_definitions": {
                "RawPart": {
                    "material_id": "RawPart",
                    "shape": "box",
                    "nominal_dimensions_m": [0.07, 0.07, 0.08],
                }
            },
            "motion": {
                "observation_to_object_translation": [0.0, 0.0, -0.04],
                "object_to_grasp_tcp_translation": [0.0, 0.0, 0.0],
                "insertion_axis_tcp": [0.0, 0.0, 1.0],
            },
            "gripper_capture": {
                "volume_dimensions_m": [0.12, 0.12, 0.12],
            },
        },
        {
            "grasp": {
                "min_expected_contact_width_mm": 60.0,
                "max_expected_contact_width_mm": 85.0,
            }
        },
    )

    assert reference.fixture_dimensions_m == (0.07, 0.07, 0.08)
    assert reference.observation_to_grasp_tcp_m == pytest.approx(-0.04)
    assert config.capture_volume_dimensions_m == (0.12, 0.12, 0.12)
    assert registry.paths() == ()
    assert registry._subtrees == {"/World/SF_Twin_Cell/MaterialSupply/Deliveries"}


def test_production_profile_uses_requested_capture_limits():
    import yaml
    from gripper_runtime.operational_config import (
        load_production_mission_recipe,
        production_capture_configuration,
    )

    repository_root = Path(__file__).resolve().parents[6]
    profile_path = (
        repository_root
        / "ros2_ws/src/arm_cell/arm_cell_bringup/config/operational_profile.yaml"
    )
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    recipe = load_production_mission_recipe(profile, profile_path)
    capture, _reference, _registry = production_capture_configuration(profile, recipe)

    assert capture.capture_volume_dimensions_m == (0.12, 0.12, 0.12)


def test_eligible_registry_expands_delivery_subtree_to_rawpart_prims_only():
    from gripper_runtime.grasp_stage import EligibleObjectRegistry

    class _Attribute:
        def __init__(self, value):
            self.value = value

        def __bool__(self):
            return self.value is not None

        def Get(self):
            return self.value

    class _Prim:
        def __init__(self, path, material=None):
            self.path = path
            self.material = material

        def GetPath(self):
            return self.path

        def GetAttribute(self, _name):
            return _Attribute(self.material)

        def IsValid(self):
            return True

    class _Stage:
        prims = (
            _Prim("/World/SF_Twin_Cell/MaterialSupply/Deliveries"),
            _Prim("/World/SF_Twin_Cell/MaterialSupply/Deliveries/delivery-a"),
            _Prim(
                "/World/SF_Twin_Cell/MaterialSupply/Deliveries/delivery-a/RawPart_0000",
                "RawPart",
            ),
            _Prim(
                "/World/SF_Twin_Cell/MaterialSupply/Deliveries/delivery-a/fixture",
                "Fixture",
            ),
        )

        def GetPrimAtPath(self, path):
            return next((prim for prim in self.prims if prim.path == path), _Prim(path))

        def Traverse(self):
            return self.prims

    registry = EligibleObjectRegistry()
    registry.register_subtree("/World/SF_Twin_Cell/MaterialSupply/Deliveries")
    assert registry.candidate_paths(_Stage()) == (
        "/World/SF_Twin_Cell/MaterialSupply/Deliveries/delivery-a/RawPart_0000",
    )


def test_production_mission_recipe_is_selected_by_profile_target(tmp_path):
    import json

    from gripper_runtime.operational_config import load_production_mission_recipe

    profile_path = tmp_path / "operational_profile.yaml"
    recipe_directory = tmp_path / "recipes"
    recipe_directory.mkdir()
    expected = {
        "recipe_id": "rawpart-v1",
        "target_id": "RawPart",
        "grasp": {
            "min_expected_contact_width_mm": 60.0,
            "max_expected_contact_width_mm": 85.0,
            "unobserved_yaw_policy": "forbidden",
        },
    }
    (recipe_directory / "RawPart.json").write_text(json.dumps(expected))

    actual = load_production_mission_recipe(
        {
            "orchestration": {
                "mission_target_id": "RawPart",
                "recipe_directory": "recipes",
            }
        },
        profile_path,
    )

    assert actual == expected


def test_session_publishes_manager_unavailability():
    physics = _Physics()
    session = GripperRuntimeSession(
        physics,
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
    )

    session.grasp_manager.ready = False
    session.on_physics_step(0.25)

    status = session.sim_adapter.events[-1][1]
    assert status["runtime_ready"] is False
    assert status["grasp_confirmed"] is False
    assert status["attached"] is False
    assert status["released"] is False
