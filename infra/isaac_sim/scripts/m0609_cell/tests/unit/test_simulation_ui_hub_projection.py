from arm_cell_simulation_ui_hub.projection import (
    FeedSnapshot,
    Freshness,
    HubProjection,
    camera_overlay_state,
)
from arm_cell_simulation_ui_hub.scenario_ports import OwnerScenarioPorts, ScenarioIntent


def test_feed_distinguishes_unavailable_current_and_stale_without_rewriting_value():
    projection = HubProjection()
    assert projection.freshness("safety", now=10) is Freshness.UNAVAILABLE
    owner_value = object()
    projection.observe(
        "safety", owner_value, received_at=10, source_stamp=2, max_age_seconds=1
    )
    assert projection.freshness("safety", now=10.5) is Freshness.CURRENT
    assert projection.freshness("safety", now=12) is Freshness.STALE
    assert projection.read("safety", now=12).value is owner_value
    assert FeedSnapshot().freshness(10) is Freshness.UNAVAILABLE


def test_retained_readiness_uses_source_age_as_well_as_local_receipt_age():
    projection = HubProjection()
    owner_value = object()
    projection.observe(
        "readiness",
        owner_value,
        received_at=10,
        source_stamp=8,
        max_age_seconds=1,
        source_age_seconds=2,
    )
    assert projection.freshness("readiness", now=10.1) is Freshness.STALE
    assert projection.read("readiness", now=10.1).value is owner_value


def test_camera_marker_requires_fresh_target_compatible_calibration_and_tf():
    args = dict(
        image_stamp=9.8,
        image_frame="camera_color_optical_frame",
        image_size=(640, 480),
        camera_stamp=9.8,
        camera_frame="camera_color_optical_frame",
        camera_size=(640, 480),
        target_valid=True,
        has_selected_target_pose=True,
        target_stamp=9.7,
        now=10.0,
        max_age_seconds=1.0,
        transform_available=True,
        project=lambda: (320, 240),
    )
    assert camera_overlay_state(**args) == ("current", (320, 240))
    assert camera_overlay_state(**{**args, "target_stamp": 8.0}) == ("stale", None)
    assert camera_overlay_state(**{**args, "camera_size": (320, 240)}) == (
        "unavailable",
        None,
    )
    assert camera_overlay_state(**{**args, "transform_available": False}) == (
        "unavailable",
        None,
    )
    assert camera_overlay_state(**{**args, "target_valid": False}) == (
        "unavailable",
        None,
    )
    assert camera_overlay_state(**{**args, "camera_stamp": 8.0}) == ("stale", None)
    assert camera_overlay_state(**{**args, "camera_stamp": None}) == (
        "unavailable",
        None,
    )


def test_scenario_intents_are_owner_local_and_fail_closed_when_port_missing():
    delivered = []
    ports = OwnerScenarioPorts(
        rgbd_source=lambda intent: delivered.append(intent) or True,
        motion_backend=None,
    )
    assert ports.set_active("rgbd_source", "target_stale", True)
    assert delivered == [ScenarioIntent("rgbd_source", "target_stale", True)]
    assert not ports.set_active("motion_backend", "holding_unknown", True)
    try:
        ports.set_active("rgbd_source", "holding_unknown", True)
    except ValueError:
        pass
    else:
        raise AssertionError("cross-owner scenario was accepted")


def test_rgbd_source_adapter_applies_and_clears_fault_before_ros_consumers():
    from graph_builder.ros_action_graph import apply_rgbd_source_scenario
    from graph_builder.graph_contract import CAMERA_FRAME_ID, GRAPH_PATH

    class Attribute:
        def __init__(self):
            self.value = None

        def is_valid(self):
            return True

        def set(self, value):
            self.value = value

    class Controller:
        attrs = {}

        @classmethod
        def attribute(cls, name):
            return cls.attrs.setdefault(name, Attribute())

    assert apply_rgbd_source_scenario("target_invalid", True, controller=Controller)
    assert (
        Controller.attribute(f"{GRAPH_PATH}/CameraInfoHelper.inputs:frameId").value
        == "scenario_invalid_optical_frame"
    )
    assert (
        Controller.attribute(f"{GRAPH_PATH}/CameraHelperColor.inputs:enabled").value
        is True
    )

    class StaleStimulus:
        def __init__(self, topic):
            self.topic = topic
            self.started = False
            self.closed = False

        def start(self):
            self.started = True
            return True

        def close(self):
            self.closed = True

    created = []

    def factory(topic):
        adapter = StaleStimulus(topic)
        created.append(adapter)
        return adapter

    assert apply_rgbd_source_scenario(
        "target_stale", True, controller=Controller, stale_adapter_factory=factory
    )
    assert created[0].started and created[0].topic == "/camera/color/image_raw"
    assert (
        Controller.attribute(f"{GRAPH_PATH}/CameraHelperColor.inputs:enabled").value
        is False
    )
    assert apply_rgbd_source_scenario(
        "target_stale", False, controller=Controller, stale_adapter_factory=factory
    )
    assert created[0].closed
    assert (
        Controller.attribute(f"{GRAPH_PATH}/CameraHelperColor.inputs:enabled").value
        is True
    )
    assert apply_rgbd_source_scenario("target_invalid", False, controller=Controller)
    assert (
        Controller.attribute(f"{GRAPH_PATH}/CameraInfoHelper.inputs:frameId").value
        == CAMERA_FRAME_ID
    )
    assert apply_rgbd_source_scenario("target_unavailable", True, controller=Controller)
    assert (
        Controller.attribute(f"{GRAPH_PATH}/CameraHelperColor.inputs:enabled").value
        is False
    )
    assert apply_rgbd_source_scenario(
        "target_unavailable", False, controller=Controller
    )
    assert (
        Controller.attribute(f"{GRAPH_PATH}/CameraHelperColor.inputs:enabled").value
        is True
    )


def test_rgbd_unavailable_stale_and_invalid_are_distinct_at_sensor_boundary():
    from types import SimpleNamespace

    from graph_builder.rgbd_source_stimulus import make_stale_sample

    original = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=20, nanosec=250_000_000),
            frame_id="camera_color_optical_frame",
        ),
        data=b"real-source-pixels",
    )
    stale = make_stale_sample(original, offset_seconds=2.0)
    assert stale is not original
    assert stale.header.stamp.sec == 18
    assert stale.header.stamp.nanosec == 250_000_000
    assert stale.header.frame_id == original.header.frame_id
    assert stale.data == original.data
    assert stale.header.stamp.sec < original.header.stamp.sec

    # Unavailable has no sample to publish; invalid keeps a sample but carries
    # the incompatible source frame authored by the source graph adapter.
    assert original.header.stamp.sec == 20
    assert original.header.frame_id == "camera_color_optical_frame"


def test_holding_scenario_is_applied_at_backend_observation_and_clear_does_not_fake_state():
    from gripper_runtime.grasp_attachment import (
        GraspManager,
        HoldingObservation,
        HoldingState,
    )
    from gripper_runtime.holding_scenario import set_holding_unknown

    manager = object.__new__(GraspManager)
    manager._holding = HoldingObservation(HoldingState.HELD, 10.0, 7)
    manager.ready = True
    manager._holding_sequence = 7
    manager._last_holding_semantics = (True, True, True, False)
    set_holding_unknown(True)
    injected = manager.holding_observation()
    assert injected.state is HoldingState.UNKNOWN
    assert injected.sequence == 8
    set_holding_unknown(False)
    restored = manager.holding_observation()
    assert restored is manager._holding
    assert restored.state is HoldingState.HELD
    assert restored.sequence == 9
