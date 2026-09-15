from gripper_runtime.action_graph_command_adapter import SimGripperAdapter
from gripper_runtime.robotiq_actuator import Robotiq2F85Actuator
from gripper_runtime.session import GripperRuntimeSession


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

    @staticmethod
    def get(attribute):
        return attribute.value


class _Gripper:
    def __init__(self):
        self.commands = []

    def set_width(self, width_mm, duration=None):
        self.commands.append((width_mm, duration))


def test_action_graph_command_adapter_keeps_attribute_path_clamping_and_deduplication():
    attribute = _Attribute(100.0)
    controller = _Controller(attribute)
    gripper = _Gripper()
    adapter = SimGripperAdapter(
        gripper,
        graph_path="/Graph",
        subscriber_node="Subscriber",
        move_duration=0.4,
        controller=controller,
    )

    assert controller.paths == ["/Graph/Subscriber.outputs:data"]
    assert adapter.update() is True
    assert gripper.commands == [(85.0, 0.4)]
    assert adapter.update() is False
    attribute.value = -1.0
    assert adapter.update() is True
    assert gripper.commands[-1] == (0.0, 0.4)
    attribute.value = float("nan")
    assert adapter.update() is False


class _Physics:
    def __init__(self):
        self.callback = None

    def subscribe_physics_step_events(self, callback):
        self.callback = callback
        return "subscription"


class _RuntimeGripper:
    def __init__(self, verbose):
        self.events = []

    def update(self, dt):
        self.events.append(("gripper", dt))


class _RuntimeAdapter:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.events = []

    def update(self):
        self.events.append("adapter")


class _RuntimeAttachment:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.events = []

    def update(self, dt):
        self.events.append(("attachment", dt))


def test_session_preserves_runtime_configuration_and_physics_tick_order():
    physics = _Physics()
    session = GripperRuntimeSession(
        physics,
        actuator_cls=_RuntimeGripper,
        adapter_cls=_RuntimeAdapter,
        grasp_manager_cls=_RuntimeAttachment,
    )

    assert (
        session.sim_adapter.kwargs["graph_path"]
        == "/World/SF_Twin_Cell/ROS2/ActionGraph"
    )
    assert session.sim_adapter.kwargs["move_duration"] == 0.40
    assert session.grasp_manager.kwargs["target_path"] is None
    assert session.subscribe() == "subscription"
    assert physics.callback == session.on_physics_step

    session.on_physics_step(0.25)
    assert session.sim_adapter.events == ["adapter"]
    assert session.gripper.events == [("gripper", 0.25)]
    assert session.grasp_manager.events == [("attachment", 0.25)]
