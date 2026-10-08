from pathlib import Path
from types import SimpleNamespace

import pytest

from gripper_runtime.action_graph_command_adapter import SimGripperAdapter
from gripper_runtime.grasp_attachment import HoldingState
from gripper_runtime.robotiq_actuator import Robotiq2F85Actuator
from gripper_runtime.session import GripperRuntimeSession, initial_status_sequence
from gripper_runtime.grasp_policy import CaptureConfig
from pnp_validation.run_validation import (
    capture_config_from_profile,
    prepare_validation_start_state,
    request_validation_start_state,
    validation_start_state_ready,
    ValidationStartPending,
    ValidationStartRejected,
)


def explicit_capture_config():
    return CaptureConfig(
        capture_volume_dimensions_m=(0.10, 0.10, 0.10),
        position_tolerance_m=0.015,
        orientation_tolerance_deg=12.0,
        expected_contact_width_mm=12.0,
        contact_width_tolerance_mm=2.0,
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
    def __init__(self, attribute, generation_attribute):
        super().__init__(attribute)
        self.generation_attribute = generation_attribute

    def attribute(self, path):
        self.paths.append(path)
        if path.endswith(".outputs:count"):
            return self.generation_attribute
        if path.endswith("PublishGripperStatus.inputs:data"):
            return _Attribute("")
        return self.attribute_value


class _Gripper:
    def __init__(self):
        self.commands = []

    def set_width(self, width_mm, duration=None):
        self.commands.append((width_mm, duration))


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
        "/Graph/PublishGripperStatus.inputs:data",
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
        "ready=1;grasp=1;attached=1;released=0;width_mm=12.500;seq=7"
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

    def subscribe_physics_step_events(self, callback):
        self.callback = callback
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

    def update(self, dt):
        self.events.append(("attachment", dt))

    def holding_observation(self):
        return SimpleNamespace(
            state=HoldingState.RELEASED, sequence=self.holding_sequence
        )

    def note_gripper_command_applied(self):
        self.holding_sequence += 1

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

    def arm_close_transaction(self):
        self.arm_calls += 1
        return True


def test_session_preserves_runtime_configuration_and_physics_tick_order():
    physics = _Physics()
    session = GripperRuntimeSession(
        physics,
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        capture_config=explicit_capture_config(),
    )

    assert session.gripper.feedback_generation == 0
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
            },
        ),
    ]
    assert session.gripper.events == [("gripper", 0.25)]
    assert session.grasp_manager.events == [("attachment", 0.25)]


def test_status_publish_diagnostic_logs_only_when_status_signature_changes(capsys):
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        capture_config=explicit_capture_config(),
    )

    session.on_physics_step(0.25)
    first = capsys.readouterr().out
    session.on_physics_step(0.25)
    second = capsys.readouterr().out

    assert first.count("event=STATUS_PUBLISH") == 1
    assert "event=STATUS_PUBLISH" not in second


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


def test_applied_open_command_publishes_a_fresh_released_sequence():
    session = GripperRuntimeSession(
        _Physics(),
        actuator_cls=_RuntimeGripper,
        adapter_cls=_ImmediateOpenAdapter,
        grasp_manager_cls=_RuntimeAttachment,
        capture_config=explicit_capture_config(),
    )
    baseline = session.grasp_manager.holding_observation().sequence

    session.on_physics_step(0.25)

    status = next(
        event[1]
        for event in session.sim_adapter.events
        if isinstance(event, tuple) and event[0] == "status"
    )
    assert status["released"] is True
    assert status["sequence"] > baseline


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
                "position_tolerance_m": 0.015,
                "orientation_tolerance_deg": 10.0,
                "expected_contact_width_mm": 70.0,
                "contact_width_tolerance_mm": 2.0,
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
    assert session.grasp_manager.kwargs["config"].orientation_tolerance_deg == 10.0


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


def test_session_without_capture_configuration_keeps_adapter_unready():
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
            "vision": {
                "detector_profiles": {"RawPart": {"dimensions_m": [0.07, 0.07, 0.08]}}
            },
            "motion": {
                "observation_to_object_translation": [0.0, 0.0, -0.04],
                "object_to_grasp_tcp_translation": [0.0, 0.0, 0.0],
                "insertion_axis_tcp": [0.0, 0.0, 1.0],
            },
            "gripper_capture": {
                "volume_dimensions_m": [0.12, 0.12, 0.12],
                "position_tolerance_m": 0.015,
                "orientation_tolerance_deg": 12.0,
                "contact_width_tolerance_mm": 2.0,
            },
        },
        {"grasp": {"width_mm": 70.0}},
    )

    assert reference.fixture_dimensions_m == (0.07, 0.07, 0.08)
    assert reference.observation_to_grasp_tcp_m == pytest.approx(-0.04)
    assert config.expected_contact_width_mm == 70.0
    assert registry.paths() == ("/World/SF_Twin_Cell/AMR/Mockup/RawPart",)


def test_production_mission_recipe_is_selected_by_profile_target(tmp_path):
    import json

    from gripper_runtime.operational_config import load_production_mission_recipe

    profile_path = tmp_path / "operational_profile.yaml"
    recipe_directory = tmp_path / "recipes"
    recipe_directory.mkdir()
    expected = {
        "recipe_id": "rawpart-v1",
        "target_id": "RawPart",
        "grasp": {"width_mm": 70.0},
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


def test_session_publishes_unready_runtime_for_missing_configuration():
    physics = _Physics()
    session = GripperRuntimeSession(
        physics,
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
    )

    session.on_physics_step(0.25)

    status = session.sim_adapter.events[-1][1]
    assert status["runtime_ready"] is False
    assert status["grasp_confirmed"] is False
    assert status["attached"] is False
    assert status["released"] is False
