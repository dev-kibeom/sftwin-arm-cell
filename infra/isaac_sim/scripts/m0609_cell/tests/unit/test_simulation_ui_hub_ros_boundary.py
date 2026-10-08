"""ROS-boundary regression using deterministic fake messages and clients."""

import json
import socket
import sys
import threading
import types
import uuid
import time

import numpy as np
import pytest


class _Future:
    def __init__(self, result):
        self._result = result
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        return self._result


class _Client:
    def __init__(self, response):
        self.response = response
        self.request = None
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.request = request
        return _Future(self.response)


class _Node:
    def __init__(self, *_args):
        self.subscriptions = []
        self.clients = {}
        self.publishers = []
        self.logs = []
        self.parameter_overrides = {}

    def declare_parameter(self, name, default):
        return types.SimpleNamespace(
            name=name, value=self.parameter_overrides.get(name, default)
        )

    def create_subscription(self, kind, topic, callback, qos):
        item = (kind, topic, callback, qos)
        self.subscriptions.append(item)
        return item

    def create_client(self, kind, topic):
        client = _Client(
            types.SimpleNamespace(
                accepted=True,
                applied=True,
                success=True,
                message="fault activated",
            )
        )
        self.clients[topic] = (kind, client)
        return client

    def destroy_subscription(self, _item):
        pass

    def destroy_client(self, _client):
        pass

    def destroy_node(self):
        pass

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(nanoseconds=10_000_000_000)
        )

    def get_logger(self):
        return types.SimpleNamespace(
            warning=lambda message: self.logs.append(("warning", message)),
            info=lambda message: self.logs.append(("info", message)),
            debug=lambda message: self.logs.append(("debug", message)),
        )


def _install_ros_types(monkeypatch):
    def module(name, **attrs):
        value = types.ModuleType(name)
        for key, attr in attrs.items():
            setattr(value, key, attr)
        monkeypatch.setitem(sys.modules, name, value)
        return value

    interfaces = module("arm_cell_interfaces")
    interfaces.__path__ = []
    msg = module(
        "arm_cell_interfaces.msg",
        AMRDockingState=type("AMRDockingState", (), {}),
        MaterialHandoffState=type("MaterialHandoffState", (), {}),
        MaterialReadiness=type("MaterialReadiness", (), {}),
        MotionStatus=type("MotionStatus", (), {}),
        PackMLState=type("PackMLState", (), {}),
        SafetyState=type("SafetyState", (), {}),
        VisionDiagnostic=type("VisionDiagnostic", (), {}),
    )

    class RequestMaterialSupply:
        class Request:
            def __init__(self):
                self.request_id = types.SimpleNamespace(uuid=[])

    class ResetSafety:
        class Request:
            def __init__(self):
                self.request_id = types.SimpleNamespace(uuid=[])
                self.operator_acknowledged = False

    interfaces.msg = msg
    interfaces.srv = module(
        "arm_cell_interfaces.srv",
        RequestMaterialSupply=RequestMaterialSupply,
        ResetSafety=ResetSafety,
    )
    interfaces.action = module(
        "arm_cell_interfaces.action", ExecuteCycle=type("ExecuteCycle", (), {})
    )
    interfaces.action.__path__ = []

    class SetBool:
        class Request:
            def __init__(self):
                self.data = False

    std_srvs = module("std_srvs")
    std_srvs.__path__ = []
    std_srvs.srv = module("std_srvs.srv", SetBool=SetBool)
    module(
        "arm_cell_interfaces.action._execute_cycle",
        ExecuteCycle_FeedbackMessage=type("ExecuteCycle_FeedbackMessage", (), {}),
    )
    module("sensor_msgs")
    sensor = sys.modules["sensor_msgs"]
    sensor.__path__ = []
    sensor.msg = module(
        "sensor_msgs.msg",
        CameraInfo=type("CameraInfo", (), {}),
        Image=type("Image", (), {}),
    )
    rclpy = module("rclpy")
    rclpy.__path__ = []
    rclpy.ok = lambda: True
    module("rclpy.node", Node=_Node)

    class _Executor:
        def __init__(self):
            self.nodes = []
            self.spin_calls = 0
            self.stopped = False

        def add_node(self, node):
            self.nodes.append(node)

        def remove_node(self, node):
            self.nodes.remove(node)

        def spin_once(self, timeout_sec):
            assert timeout_sec == 0.0
            self.spin_calls += 1

        def shutdown(self):
            self.stopped = True

    module("rclpy.executors", SingleThreadedExecutor=_Executor)
    module(
        "rclpy.qos",
        qos_profile_sensor_data=object(),
        QoSProfile=lambda **kwargs: kwargs,
        ReliabilityPolicy=types.SimpleNamespace(
            RELIABLE="reliable", BEST_EFFORT="best_effort"
        ),
        DurabilityPolicy=types.SimpleNamespace(
            TRANSIENT_LOCAL="transient_local", VOLATILE="volatile"
        ),
        HistoryPolicy=types.SimpleNamespace(KEEP_LAST="keep_last"),
    )
    module("rclpy.time", Time=type("Time", (), {"__init__": lambda self, **kw: None}))
    module(
        "tf2_ros",
        Buffer=type("Buffer", (), {"__init__": lambda self, **kw: None}),
        TransformListener=type(
            "TransformListener", (), {"__init__": lambda self, *a: None}
        ),
    )
    module("action_msgs")
    sys.modules["action_msgs"].__path__ = []
    module(
        "action_msgs.msg",
        GoalStatusArray=type("GoalStatusArray", (), {}),
        GoalStatus=type(
            "GoalStatus",
            (),
            {
                "STATUS_ACCEPTED": 1,
                "STATUS_EXECUTING": 2,
                "STATUS_CANCELING": 3,
                "STATUS_SUCCEEDED": 4,
                "STATUS_CANCELED": 5,
                "STATUS_ABORTED": 6,
            },
        ),
    )


def test_python311_humble_runtime_returns_ros_sidecar(monkeypatch):
    import arm_cell_simulation_ui_hub.runtime as runtime_module

    class Sidecar:
        def __init__(self, **kwargs):
            self.options = kwargs

    sidecar_module = types.ModuleType("arm_cell_simulation_ui_hub.sidecar")
    sidecar_module.IsaacRosSidecar = Sidecar
    monkeypatch.setitem(sys.modules, sidecar_module.__name__, sidecar_module)
    monkeypatch.setattr(runtime_module.sys, "version_info", (3, 11, 0))
    monkeypatch.setenv("ROS_DISTRO", "humble")

    runtime = runtime_module.HubRuntime(default_observation_freshness_seconds=2.0)

    assert isinstance(runtime, Sidecar)
    assert runtime.options["default_observation_freshness_seconds"] == 2.0
    assert runtime.options["default_annotation_source_tolerance_seconds"] == 0.15


def test_hub_creates_only_approved_clients_and_sends_operator_identities(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    node.parameter_overrides["annotation_source_tolerance_seconds"] = 0.02
    runtime = HubRuntime(node=node)
    assert runtime.feed_max_age_seconds > 0
    safety_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/safety/state"
    )
    safety_callback(types.SimpleNamespace())
    assert (
        runtime.projection.freshness("safety", now=time.monotonic()).value == "current"
    )
    assert {topic for _kind, topic, _callback, _qos in node.subscriptions} == {
        "/safety/state",
        "/motion/state",
        "/amr/docking_report",
        "/vda/material_handoff_state",
        "/integration/material_readiness",
        "/packml/state",
        "/camera/color/image_raw",
        "/camera/aligned_depth_to_color/image_raw",
        "/vision/diagnostics",
        "/camera/color/camera_info",
        "/orchestration/execute_cycle/_action/feedback",
        "/orchestration/execute_cycle/_action/status",
    }
    video_qos = next(
        qos
        for _kind, topic, _callback, qos in node.subscriptions
        if topic == "/camera/color/image_raw"
    )
    from rclpy.qos import DurabilityPolicy, HistoryPolicy

    from sensor_msgs.msg import Image

    video_subscription_type = next(
        kind
        for kind, topic, _callback, _qos in node.subscriptions
        if topic == "/camera/color/image_raw"
    )
    assert video_subscription_type is Image
    assert video_qos["depth"] == 1
    assert video_qos["history"] == HistoryPolicy.KEEP_LAST
    assert video_qos["reliability"] == "best_effort"
    assert video_qos["durability"] == DurabilityPolicy.VOLATILE
    assert set(node.clients) == {"/integration/request_material", "/safety/reset"}
    assert node.publishers == []

    material_id, material_future = runtime.request_material()
    material = node.clients["/integration/request_material"][1]
    assert str(uuid.UUID(bytes=bytes(material.request.request_id.uuid))) == material_id
    assert not hasattr(material.request, "delivery_id")
    assert runtime.request_state[material_id] == "pending"
    material_future.callback(material_future)
    assert runtime.request_state[material_id] == "accepted"

    reset_id, reset_future = runtime.acknowledge_and_reset()
    reset = node.clients["/safety/reset"][1]
    assert str(uuid.UUID(bytes=bytes(reset.request.request_id.uuid))) == reset_id
    assert reset.request.operator_acknowledged
    reset_future.callback(reset_future)
    assert runtime.request_state[reset_id] == "applied_pending_owner_state"


def test_hub_vda_fault_requests_are_async_and_update_intent_only_on_success(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)

    request_id, future = runtime.set_external_fault("e_stop", True)

    client = node.clients["/external_state_mock/fault/e_stop"][1]
    assert client.request.data is True
    assert future is not None
    assert runtime.request_state[request_id] == "pending"
    assert runtime.fault_active["e_stop"] is False

    future.callback(future)

    assert runtime.request_state[request_id] == "applied: fault activated"
    assert runtime.fault_active["e_stop"] is True
    runtime.close()


@pytest.mark.parametrize(
    ("ready", "response", "future_error", "expected_state"),
    [
        (False, None, None, "unavailable"),
        (
            True,
            types.SimpleNamespace(success=False, message="rejected"),
            None,
            "rejected: rejected",
        ),
        (True, None, RuntimeError("transport failed"), "failed: transport failed"),
    ],
)
def test_hub_vda_fault_reports_unavailable_rejected_and_failed_requests(
    monkeypatch, ready, response, future_error, expected_state
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    original_create_client = node.create_client

    def create_client(kind, topic):
        client = original_create_client(kind, topic)
        if topic.endswith("/fault/invalid_state"):
            client.ready = ready
            if response is not None:
                client.response = response
            if future_error is not None:
                client.call_async = lambda _request: (_ for _ in ()).throw(future_error)
        return client

    node.create_client = create_client
    runtime = HubRuntime(node=node)
    request_id, future = runtime.set_external_fault("invalid_state", True)
    if not ready:
        assert runtime.request_state[request_id] == expected_state
        assert future is None
    elif future_error is not None:
        assert runtime.request_state[request_id] == expected_state
        assert future is None
    else:
        assert runtime.request_state[request_id] == "pending"
        future.callback(future)
        assert runtime.request_state[request_id] == expected_state
        assert runtime.fault_active["invalid_state"] is False
    runtime.close()


def test_hub_projects_latest_owner_readiness_with_compatible_subscription(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    subscription = next(
        item
        for item in node.subscriptions
        if item[1] == "/integration/material_readiness"
    )
    readiness_qos = subscription[3]
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

    assert readiness_qos["reliability"] == ReliabilityPolicy.RELIABLE
    assert readiness_qos["durability"] == DurabilityPolicy.TRANSIENT_LOCAL

    delivery_one = uuid.uuid4()
    delivery_two = uuid.uuid4()

    def state(delivery_id, ready, source_seconds=10):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=source_seconds, nanosec=0)
            ),
            delivery_id=types.SimpleNamespace(uuid=list(delivery_id.bytes)),
            material_ready=ready,
            valid=True,
        )

    subscription[2](state(delivery_one, True, source_seconds=8))
    stale_text = runtime.observation_text("readiness", now=time.monotonic())
    assert stale_text == "readiness: stale"

    subscription[2](state(delivery_two, False))
    text = runtime.observation_text("readiness", now=time.monotonic())
    assert "ready=False" in text
    assert str(delivery_two) in text
    assert str(delivery_one) not in text

    image = types.SimpleNamespace(
        height=1,
        width=1,
        encoding="rgb8",
        step=3,
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=4, nanosec=5), frame_id="camera_optical"
        ),
        data=bytes((3, 2, 1)),
    )
    video_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/camera/color/image_raw"
    )
    video_callback(image)
    pixels, width, height = runtime.camera_frame_rgba()
    assert (width, height) == (1, 1)
    assert pixels.dtype == np.uint8
    assert pixels.shape == (1, 1, 4)
    assert np.array_equal(pixels[0, 0], [3, 2, 1, 255])
    assert runtime.video_mode() == "LIVE"
    assert runtime.camera_identity() == (4.000000005, "camera_optical")
    runtime.close()


def test_rgb_depth_switch_uses_frozen_sources_during_pnp_then_returns_to_live(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    callbacks = {topic: callback for _kind, topic, callback, _qos in node.subscriptions}
    goal_id = bytes(range(16))
    callbacks["/orchestration/execute_cycle/_action/feedback"](
        types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(goal_id)),
            feedback=types.SimpleNamespace(phase=2),
        )
    )

    def header(stamp):
        seconds = int(stamp)
        nanoseconds = int((stamp - seconds) * 1e9)
        return types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=seconds, nanosec=nanoseconds),
            frame_id="camera_color_optical_frame",
        )

    def rgb(stamp, value):
        return types.SimpleNamespace(
            header=header(stamp),
            height=2,
            width=2,
            encoding="rgb8",
            step=6,
            data=np.full((2, 2, 3), value, dtype=np.uint8).tobytes(),
        )

    def depth(stamp, value):
        return types.SimpleNamespace(
            header=header(stamp),
            height=2,
            width=2,
            encoding="32FC1",
            step=8,
            is_bigendian=False,
            data=np.full((2, 2), value, dtype=np.float32).tobytes(),
        )

    callbacks["/camera/color/image_raw"](rgb(5.0, 10))
    callbacks["/camera/aligned_depth_to_color/image_raw"](depth(5.0, 0.5))
    callbacks["/vision/diagnostics"](
        types.SimpleNamespace(
            state=0,
            result_code=0,
            valid=True,
            source_rgb_header=header(5.0),
            target_id="RawPart",
            centroid_pixel_valid=False,
            centroid_pixel_x=0,
            centroid_pixel_y=0,
            object_region_valid=False,
            object_region_x=0,
            object_region_y=0,
            object_region_width=0,
            object_region_height=0,
            support_region_valid=False,
            support_region=[],
            yaw_available=False,
        )
    )

    assert runtime._detection_snapshot is not None
    runtime.set_display_source("depth")
    frozen_depth = runtime.camera_frame_rgba()[0].copy()
    callbacks["/camera/color/image_raw"](rgb(5.1, 20))
    callbacks["/camera/aligned_depth_to_color/image_raw"](depth(5.1, 1.5))
    assert np.array_equal(runtime.camera_frame_rgba()[0], frozen_depth)
    assert runtime.camera_identity() == (5.0, "camera_color_optical_frame")

    from action_msgs.msg import GoalStatus

    callbacks["/orchestration/execute_cycle/_action/status"](
        types.SimpleNamespace(
            status_list=[
                types.SimpleNamespace(
                    goal_info=types.SimpleNamespace(
                        goal_id=types.SimpleNamespace(uuid=list(goal_id))
                    ),
                    status=GoalStatus.STATUS_SUCCEEDED,
                )
            ]
        )
    )
    assert runtime._detection_snapshot is None
    assert runtime.display_source() == "depth"
    assert runtime.camera_identity()[0] == pytest.approx(5.1)
    assert runtime.camera_identity()[1] == "camera_color_optical_frame"
    assert not np.array_equal(runtime.camera_frame_rgba()[0], frozen_depth)
    runtime.close()


def test_unavailable_depth_does_not_prevent_rgb_pnp_snapshot(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    callbacks = {topic: callback for _kind, topic, callback, _qos in node.subscriptions}
    stamp = types.SimpleNamespace(sec=6, nanosec=0)
    header = types.SimpleNamespace(stamp=stamp, frame_id="camera_color_optical_frame")
    callbacks["/camera/color/image_raw"](
        types.SimpleNamespace(
            header=header,
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=bytes((1, 2, 3)),
        )
    )
    callbacks["/orchestration/execute_cycle/_action/feedback"](
        types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(bytes(range(16)))),
            feedback=types.SimpleNamespace(phase=2),
        )
    )
    callbacks["/vision/diagnostics"](
        types.SimpleNamespace(
            state=0,
            result_code=0,
            valid=True,
            source_rgb_header=header,
            target_id="RawPart",
            centroid_pixel_valid=False,
            centroid_pixel_x=0,
            centroid_pixel_y=0,
            object_region_valid=False,
            object_region_x=0,
            object_region_y=0,
            object_region_width=0,
            object_region_height=0,
            support_region_valid=False,
            support_region=[],
            yaw_available=False,
        )
    )
    assert runtime._detection_snapshot is not None
    assert runtime._detection_snapshot.depth_frame is None
    assert runtime.camera_frame_rgba() is not None
    assert runtime.video_mode() == "DETECTION SNAPSHOT · PnP ACTIVE"
    runtime.close()


def test_depth_display_is_independent_and_is_transformed_only_when_selected(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    depth_subscription = next(
        item
        for item in node.subscriptions
        if item[1] == "/camera/aligned_depth_to_color/image_raw"
    )
    rgb_subscription = next(
        item for item in node.subscriptions if item[1] == "/camera/color/image_raw"
    )
    assert depth_subscription[3] == rgb_subscription[3]
    assert depth_subscription[3]["depth"] == 1

    conversion_count = [0]
    depth_source = runtime._display_sources["depth"]
    original_convert = depth_source.convert

    def counted_convert(*args, **kwargs):
        conversion_count[0] += 1
        return original_convert(*args, **kwargs)

    monkeypatch.setattr(depth_source, "convert", counted_convert)
    depth_subscription[2](
        types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=7, nanosec=0),
                frame_id="camera_color_optical_frame",
            ),
            height=1,
            width=1,
            encoding="32FC1",
            step=4,
            is_bigendian=False,
            data=np.array([[0.5]], dtype=np.float32).tobytes(),
        )
    )
    assert conversion_count == [0]

    runtime.set_display_source("depth")
    frame = runtime.display_frame()
    assert frame is not None
    assert frame.source_id == "depth"
    assert conversion_count == [1]
    assert runtime.camera_identity() == (7.0, "camera_color_optical_frame")
    runtime.display_frame()
    assert conversion_count == [1]
    runtime.close()


def test_depth_annotation_requires_camera_geometry_and_operator_time_compatibility(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    callbacks = {topic: callback for _kind, topic, callback, _qos in node.subscriptions}

    def image_header(seconds, frame_id="camera_color_optical_frame"):
        return types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=seconds, nanosec=0), frame_id=frame_id
        )

    callbacks["/camera/color/image_raw"](
        types.SimpleNamespace(
            header=image_header(8),
            height=2,
            width=2,
            encoding="rgb8",
            step=6,
            data=np.zeros((2, 2, 3), dtype=np.uint8).tobytes(),
        )
    )
    callbacks["/vision/diagnostics"](
        types.SimpleNamespace(
            state=0,
            result_code=0,
            valid=True,
            source_rgb_header=image_header(8),
            target_id="RawPart",
            centroid_pixel_valid=False,
            centroid_pixel_x=0,
            centroid_pixel_y=0,
            object_region_valid=False,
            object_region_x=0,
            object_region_y=0,
            object_region_width=0,
            object_region_height=0,
            support_region_valid=False,
            support_region=[],
            yaw_available=False,
        )
    )

    def depth(seconds, *, frame_id="camera_color_optical_frame", width=2):
        return types.SimpleNamespace(
            header=image_header(seconds, frame_id),
            height=2,
            width=width,
            encoding="32FC1",
            step=width * 4,
            is_bigendian=False,
            data=np.ones((2, width), dtype=np.float32).tobytes(),
        )

    depth_callback = callbacks["/camera/aligned_depth_to_color/image_raw"]
    depth_callback(depth(8.1))
    runtime.set_display_source("depth")
    assert runtime.camera_frame_rgba() is not None  # No RGB-D sync gate for display.
    assert runtime.vision_display()[1] is not None

    depth_callback(depth(8.2))  # Beyond the annotation presentation window.
    assert runtime.camera_frame_rgba() is not None
    assert runtime.vision_display()[1] is None

    depth_callback(depth(8.05, frame_id="other_optical"))
    assert runtime.vision_display()[1] is None

    depth_callback(depth(8.05, width=1))
    assert runtime.vision_display()[1] is None

    runtime.set_display_source("rgb")
    assert runtime.camera_frame_rgba() is not None
    assert runtime.vision_display()[1] is not None
    runtime.close()


def test_unsupported_depth_is_unavailable_without_affecting_rgb(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    callbacks = {topic: callback for _kind, topic, callback, _qos in node.subscriptions}
    callbacks["/camera/color/image_raw"](
        types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=9, nanosec=0), frame_id="camera_optical"
            ),
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=bytes((1, 2, 3)),
        )
    )
    callbacks["/camera/aligned_depth_to_color/image_raw"](
        types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=9, nanosec=0), frame_id="camera_optical"
            ),
            height=1,
            width=1,
            encoding="16UC1",
            step=2,
            is_bigendian=False,
            data=b"\xe8\x03",
        )
    )
    runtime.set_display_source("depth")
    assert runtime.camera_frame_rgba() is None
    assert "UNAVAILABLE" in runtime.display_status()
    runtime.set_display_source("rgb")
    assert runtime.camera_frame_rgba() is not None
    assert runtime.video_mode() == "LIVE"
    runtime.close()


def test_stale_depth_is_unavailable_without_staling_rgb_or_vision(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    runtime.feed_max_age_seconds = 0.5
    callbacks = {topic: callback for _kind, topic, callback, _qos in node.subscriptions}
    rgb_header = types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=10, nanosec=0),
        frame_id="camera_color_optical_frame",
    )
    callbacks["/camera/color/image_raw"](
        types.SimpleNamespace(
            header=rgb_header,
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=bytes((11, 22, 33)),
        )
    )
    callbacks["/camera/aligned_depth_to_color/image_raw"](
        types.SimpleNamespace(
            header=rgb_header,
            height=1,
            width=1,
            encoding="32FC1",
            step=4,
            is_bigendian=False,
            data=np.array([[0.5]], dtype=np.float32).tobytes(),
        )
    )
    runtime._depth_received_at = time.monotonic() - 1.0

    runtime.set_display_source("depth")
    assert runtime.camera_frame_rgba() is None
    assert "STALE" in runtime.display_status()
    runtime.set_display_source("rgb")
    pixels, _, _ = runtime.camera_frame_rgba()
    assert pixels[0, 0].tolist() == [11, 22, 33, 255]
    assert runtime.video_mode() == "LIVE"
    runtime.close()


def test_operator_annotation_tolerance_above_200ms_uses_default(monkeypatch):
    """An oversized setting is rejected and leaves the default in effect."""
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    node.parameter_overrides["annotation_source_tolerance_seconds"] = 0.25
    runtime = HubRuntime(node=node)

    assert runtime._annotation_source_tolerance_seconds == 0.15

    runtime.close()


def test_terminal_source_is_authoritative_over_detecting_source(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    feedback_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/orchestration/execute_cycle/_action/feedback"
    )
    video_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/camera/color/image_raw"
    )
    diagnostic_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/vision/diagnostics"
    )
    goal_id = bytes(range(16))
    feedback_callback(
        types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(goal_id)),
            feedback=types.SimpleNamespace(phase=2),
        )
    )
    original_source = types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=3, nanosec=0), frame_id="camera_optical"
    )
    diagnostic_callback(
        types.SimpleNamespace(
            state=1,
            result_code=6,
            valid=False,
            source_rgb_header=original_source,
            target_id="RawPart",
        )
    )
    video_callback(
        types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=3, nanosec=83_333_000),
                frame_id="camera_optical",
            ),
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=np.zeros((1, 1, 3), np.uint8).tobytes(),
        )
    )
    diagnostic_callback(
        types.SimpleNamespace(
            state=0,
            result_code=0,
            valid=True,
            source_rgb_header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=3, nanosec=83_333_000),
                frame_id="camera_optical",
            ),
            target_id="RawPart",
            centroid_pixel_valid=False,
            centroid_pixel_x=0.0,
            centroid_pixel_y=0.0,
            object_region_valid=False,
            object_region_x=0,
            object_region_y=0,
            object_region_width=0,
            object_region_height=0,
            support_region_valid=False,
            support_region=[],
            yaw_available=False,
        )
    )

    assert runtime._annotation_source_tolerance_seconds == 0.15
    assert runtime._detection_snapshot is not None
    assert runtime._detection_snapshot.display_frame_identity == (
        3.083333,
        "camera_optical",
    )
    runtime.close()


def test_vision_overlay_requires_fresh_bounded_source_rgb_match():
    from arm_cell_simulation_ui_hub.projection import vision_overlay_match
    from arm_cell_simulation_ui_hub.runtime import (
        DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS,
    )

    tolerance = DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS

    assert (
        vision_overlay_match(
            image_stamp=1.0,
            image_frame="camera_optical",
            diagnostic_stamp=1.083333,
            diagnostic_frame="camera_optical",
            diagnostic_current=True,
            max_delta_seconds=tolerance,
        )
        == "current"
    )
    assert (
        vision_overlay_match(
            image_stamp=1.0,
            image_frame="camera_optical",
            diagnostic_stamp=1.0,
            diagnostic_frame="other_camera",
            diagnostic_current=True,
            max_delta_seconds=tolerance,
        )
        == "unavailable"
    )
    assert (
        vision_overlay_match(
            image_stamp=1.0,
            image_frame="camera_optical",
            diagnostic_stamp=1.150001,
            diagnostic_frame="camera_optical",
            diagnostic_current=True,
            max_delta_seconds=tolerance,
        )
        == "stale"
    )
    assert (
        vision_overlay_match(
            image_stamp=1.0,
            image_frame="camera_optical",
            diagnostic_stamp=1.0,
            diagnostic_frame="camera_optical",
            diagnostic_current=False,
            max_delta_seconds=tolerance,
        )
        == "stale"
    )


def test_compatible_operator_rgb_lookup_selects_nearest_bounded_frame_and_rejects_incompatible_frames(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import (
        select_compatible_operator_rgb_frame,
    )

    vision_source_identity = (44.816669004, "camera_color_optical_frame")
    nearest_frame = object()
    outside_tolerance_frame = object()
    history = [
        ((44.900002337, "camera_color_optical_frame"), nearest_frame, 10.0),
        ((44.983335679, "camera_color_optical_frame"), outside_tolerance_frame, 10.1),
    ]

    selected = select_compatible_operator_rgb_frame(
        history, vision_source_identity, max_delta_seconds=0.15
    )

    assert selected[1] is nearest_frame
    assert (
        select_compatible_operator_rgb_frame(
            [history[1]], vision_source_identity, max_delta_seconds=0.15
        )
        is None
    )
    assert (
        select_compatible_operator_rgb_frame(
            [((44.900002337, "other_camera"), nearest_frame, 10.0)],
            vision_source_identity,
            max_delta_seconds=0.15,
        )
        is None
    )

    import arm_cell_simulation_ui_hub.runtime as runtime_module

    runtime = runtime_module.HubRuntime(node=_Node())
    runtime._source_frames.extend(history)
    runtime._prune_source_frames(10.3)
    assert (
        select_compatible_operator_rgb_frame(
            runtime._source_frames,
            vision_source_identity,
            max_delta_seconds=0.15,
        )[1]
        is nearest_frame
    )
    runtime._prune_source_frames(10.51)
    assert not runtime._source_frames
    runtime._source_frames.extend(
        (
            (50.0 + index * 0.01, "camera_color_optical_frame"),
            object(),
            12.0,
        )
        for index in range(65)
    )
    assert (
        len(runtime._source_frames)
        == runtime_module.ANNOTATION_SOURCE_FRAME_HISTORY_SIZE
    )
    assert all(entry[1] is not nearest_frame for entry in runtime._source_frames)
    runtime._prune_source_frames(14.1)
    assert not runtime._source_frames
    runtime.close()


@pytest.mark.parametrize(
    "terminal_status", ["STATUS_SUCCEEDED", "STATUS_CANCELED", "STATUS_ABORTED"]
)
@pytest.mark.parametrize("include_detecting", [False, True])
def test_vision_latches_nearest_compatible_rgb_during_execute_cycle(
    monkeypatch, terminal_status, include_detecting
):
    import arm_cell_simulation_ui_hub.runtime as runtime_module

    _install_ros_types(monkeypatch)
    now = [100.0]
    monkeypatch.setattr(runtime_module.time, "monotonic", lambda: now[0])
    node = _Node()
    runtime = runtime_module.HubRuntime(node=node)
    runtime.feed_max_age_seconds = 1.0
    video_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/camera/color/image_raw"
    )
    diagnostic_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/vision/diagnostics"
    )
    feedback_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/orchestration/execute_cycle/_action/feedback"
    )
    status_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/orchestration/execute_cycle/_action/status"
    )
    from action_msgs.msg import GoalStatus

    terminal_status = getattr(GoalStatus, terminal_status)

    def raw_frame(stamp, color, nanosec=0):
        rgb = np.full((3, 3, 3), color, np.uint8)
        return types.SimpleNamespace(
            height=3,
            width=3,
            encoding="rgb8",
            step=9,
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=stamp, nanosec=nanosec),
                frame_id="camera_optical",
            ),
            data=rgb.tobytes(),
        )

    goal_id = bytes(range(16))
    feedback_callback(
        types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(goal_id)),
            feedback=types.SimpleNamespace(phase=2),
        )
    )
    source = types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=44, nanosec=816_669_004),
        frame_id="camera_optical",
    )
    if include_detecting:
        diagnostic_callback(
            types.SimpleNamespace(
                state=1,
                result_code=6,
                valid=False,
                source_rgb_header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id=""
                ),
                target_id="RawPart",
            )
        )
        diagnostic_callback(
            types.SimpleNamespace(
                state=1,
                result_code=6,
                valid=False,
                source_rgb_header=source,
                target_id="RawPart",
            )
        )
    # Operator video may use a nearby same-camera frame. This is 83.333 ms
    # after the Vision source and must remain within the bounded display window.
    video_callback(raw_frame(44, (240, 20, 1), 900_002_337))
    for stamp in range(45, 65):
        video_callback(raw_frame(stamp, (240, 20, 1)))
    assert runtime.camera_identity()[0] == 64.0
    expected_snapshot_pixels = np.full((3, 3, 4), (240, 20, 1, 255), np.uint8)
    diagnostic_callback(
        types.SimpleNamespace(
            state=0,
            result_code=0,
            valid=True,
            source_rgb_header=source,
            target_id="RawPart",
            centroid_pixel_valid=True,
            centroid_pixel_x=1.0,
            centroid_pixel_y=1.0,
            object_region_valid=True,
            object_region_x=0,
            object_region_y=0,
            object_region_width=3,
            object_region_height=3,
            support_region_valid=True,
            support_region=[
                types.SimpleNamespace(x=0.0, y=0.0),
                types.SimpleNamespace(x=2.0, y=0.0),
                types.SimpleNamespace(x=2.0, y=2.0),
            ],
            yaw_available=True,
        )
    )
    snapshot_pixels, _, _ = runtime.camera_frame_rgba()
    assert np.array_equal(snapshot_pixels, expected_snapshot_pixels)
    assert not snapshot_pixels.flags.writeable
    assert runtime.camera_identity() == (44.900002337, "camera_optical")
    assert runtime.video_mode() == "DETECTION SNAPSHOT · PnP ACTIVE"
    _state, metadata = runtime.vision_display()
    assert metadata is not None
    assert metadata["target_id"] == "RawPart"
    assert metadata["object_region"] == (0, 0, 3, 3)
    assert metadata["centroid"] == (1.0, 1.0)
    assert metadata["support_region"] == [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]

    video_callback(raw_frame(66, (240, 20, 1)))
    displayed_pixels, _, _ = runtime.camera_frame_rgba()
    assert np.array_equal(displayed_pixels, expected_snapshot_pixels)
    assert runtime.camera_identity() == (44.900002337, "camera_optical")

    status_callback(
        types.SimpleNamespace(
            status_list=[
                types.SimpleNamespace(
                    goal_info=types.SimpleNamespace(
                        goal_id=types.SimpleNamespace(uuid=list(goal_id))
                    ),
                    status=terminal_status,
                )
            ]
        )
    )
    assert runtime.camera_identity()[0] == 66.0
    assert runtime.video_mode() == "LIVE"
    state, metadata = runtime.vision_display()
    assert state == "TARGET DETECTED"
    assert metadata is not None
    assert metadata["target_id"] == "RawPart"
    runtime.close()


def test_stale_terminal_diagnostic_does_not_latch_late_operator_frame(monkeypatch):
    import arm_cell_simulation_ui_hub.runtime as runtime_module

    _install_ros_types(monkeypatch)
    now = [100.0]
    monkeypatch.setattr(runtime_module.time, "monotonic", lambda: now[0])
    node = _Node()
    runtime = runtime_module.HubRuntime(node=node)
    runtime.feed_max_age_seconds = 1.0
    video_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/camera/color/image_raw"
    )
    diagnostic_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/vision/diagnostics"
    )
    feedback_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/orchestration/execute_cycle/_action/feedback"
    )

    rgb = np.full((1, 1, 3), (1, 20, 230), np.uint8)
    source = types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=44, nanosec=816_669_004),
        frame_id="camera_color_optical_frame",
    )
    goal_id = bytes(range(16))
    feedback_callback(
        types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(goal_id)),
            feedback=types.SimpleNamespace(phase=2),
        )
    )
    diagnostic_callback(
        types.SimpleNamespace(state=1, source_rgb_header=source, target_id="RawPart")
    )
    diagnostic_callback(
        types.SimpleNamespace(
            state=0,
            result_code=0,
            valid=True,
            source_rgb_header=source,
            target_id="RawPart",
            centroid_pixel_valid=False,
            centroid_pixel_x=0.0,
            centroid_pixel_y=0.0,
            object_region_valid=False,
            object_region_x=0,
            object_region_y=0,
            object_region_width=0,
            object_region_height=0,
            support_region_valid=False,
            support_region=[],
            yaw_available=False,
        )
    )
    assert runtime._detection_snapshot is None

    now[0] += 1.1
    video_callback(
        types.SimpleNamespace(
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            header=source,
            data=rgb.tobytes(),
        )
    )

    assert runtime._detection_snapshot is None
    assert runtime._pending_detection_diagnostic is None
    assert runtime.vision_display()[1] is None
    assert any("vision_diagnostic_expired" in message for _level, message in node.logs)
    runtime.close()


@pytest.mark.parametrize("include_detecting", [False, True])
def test_successful_terminal_annotation_is_presented_without_execute_cycle(
    monkeypatch, include_detecting
):
    import arm_cell_simulation_ui_hub.runtime as runtime_module

    _install_ros_types(monkeypatch)
    now = [100.0]
    monkeypatch.setattr(runtime_module.time, "monotonic", lambda: now[0])
    node = _Node()
    runtime = runtime_module.HubRuntime(node=node)
    runtime.feed_max_age_seconds = 1.0
    callbacks = {topic: callback for _kind, topic, callback, _qos in node.subscriptions}
    source = types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=5, nanosec=0),
        frame_id="camera_optical",
    )
    if include_detecting:
        callbacks["/vision/diagnostics"](
            types.SimpleNamespace(
                state=1,
                result_code=6,
                valid=False,
                source_rgb_header=source,
                target_id="RawPart",
            )
        )
    callbacks["/camera/color/image_raw"](
        types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=5, nanosec=50_000_000),
                frame_id="camera_optical",
            ),
            height=2,
            width=2,
            encoding="rgb8",
            step=6,
            data=np.zeros((2, 2, 3), np.uint8).tobytes(),
        )
    )
    callbacks["/vision/diagnostics"](
        types.SimpleNamespace(
            state=0,
            result_code=0,
            valid=True,
            source_rgb_header=source,
            target_id="RawPart",
            centroid_pixel_valid=True,
            centroid_pixel_x=1.0,
            centroid_pixel_y=1.0,
            object_region_valid=False,
            support_region_valid=False,
            yaw_available=False,
        )
    )

    frame = runtime.camera_frame_rgba()
    state, annotation = runtime.vision_display()
    assert frame is not None
    assert runtime.camera_identity() == (5.05, "camera_optical")
    assert runtime._detection_snapshot is None
    assert state == "TARGET DETECTED"
    assert annotation == {
        "target_id": "RawPart",
        "centroid": (1.0, 1.0),
        "object_region": None,
        "support_region": [],
        "yaw_available": False,
    }
    now[0] += 1.1
    assert runtime.vision_display()[1] is None
    assert runtime.camera_frame_rgba() is None
    runtime.close()


@pytest.mark.parametrize(
    ("operator_stamp", "operator_frame", "include_detecting", "expected_match"),
    [
        (2.05, "camera_optical", False, True),
        (2.083333, "camera_optical", True, True),
        (2.166667001, "camera_optical", True, False),
        (2.05, "another_camera", True, False),
    ],
)
def test_vision_annotation_requires_bounded_same_camera_operator_frame(
    monkeypatch, operator_stamp, operator_frame, include_detecting, expected_match
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    feedback_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/orchestration/execute_cycle/_action/feedback"
    )
    video_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/camera/color/image_raw"
    )
    diagnostic_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/vision/diagnostics"
    )
    goal_id = bytes(range(16))
    feedback_callback(
        types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(goal_id)),
            feedback=types.SimpleNamespace(phase=2),
        )
    )
    source = types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=2, nanosec=0), frame_id="camera_optical"
    )
    if include_detecting:
        diagnostic_callback(
            types.SimpleNamespace(
                state=1,
                result_code=6,
                valid=False,
                source_rgb_header=source,
                target_id="RawPart",
            )
        )
    video_callback(
        types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(
                    sec=int(operator_stamp),
                    nanosec=round((operator_stamp % 1) * 1e9),
                ),
                frame_id=operator_frame,
            ),
            height=2,
            width=2,
            encoding="rgb8",
            step=6,
            data=np.zeros((2, 2, 3), np.uint8).tobytes(),
        )
    )
    diagnostic = types.SimpleNamespace(
        state=0,
        result_code=0,
        valid=True,
        source_rgb_header=source,
        target_id="RawPart",
        centroid_pixel_valid=True,
        centroid_pixel_x=321.0,
        centroid_pixel_y=123.0,
        object_region_valid=True,
        object_region_x=300,
        object_region_y=100,
        object_region_width=40,
        object_region_height=50,
        support_region_valid=True,
        support_region=[types.SimpleNamespace(x=1.0, y=2.0)],
        yaw_available=False,
    )
    diagnostic_callback(diagnostic)
    assert (runtime._detection_snapshot is not None) is expected_match
    assert (runtime.vision_display()[1] is not None) is expected_match
    assert source.stamp.sec == 2
    assert source.stamp.nanosec == 0
    assert diagnostic.valid is True
    if expected_match:
        assert runtime.camera_identity() == (operator_stamp, operator_frame)
    else:
        assert runtime.camera_identity() == (operator_stamp, operator_frame)
    runtime.close()


@pytest.mark.parametrize(("valid", "result_code"), [(False, 0), (True, 6)])
def test_invalid_or_non_success_terminal_does_not_create_annotation(
    monkeypatch, valid, result_code
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    callbacks = {topic: callback for _kind, topic, callback, _qos in node.subscriptions}
    callbacks["/orchestration/execute_cycle/_action/feedback"](
        types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(bytes(range(16)))),
            feedback=types.SimpleNamespace(phase=2),
        )
    )
    callbacks["/camera/color/image_raw"](
        types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=2, nanosec=50_000_000),
                frame_id="camera_optical",
            ),
            height=2,
            width=2,
            encoding="rgb8",
            step=6,
            data=np.zeros((2, 2, 3), np.uint8).tobytes(),
        )
    )
    callbacks["/vision/diagnostics"](
        types.SimpleNamespace(
            state=0,
            result_code=result_code,
            valid=valid,
            source_rgb_header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=2, nanosec=0),
                frame_id="camera_optical",
            ),
            target_id="RawPart",
        )
    )
    assert runtime._detection_snapshot is None
    assert runtime.vision_display()[1] is None
    runtime.close()


def test_raw_rgb_frame_validation_preserves_unavailable_recovery(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    video_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/camera/color/image_raw"
    )
    header = types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=1, nanosec=0), frame_id="camera_optical"
    )
    bad_frame = types.SimpleNamespace(
        header=header,
        height=2,
        width=2,
        encoding="rgb8",
        step=6,
        data=b"short",
    )
    video_callback(bad_frame)
    video_callback(bad_frame)
    assert runtime.camera_frame_rgba() is None
    assert runtime.video_mode() == "UNAVAILABLE"
    video_callback(
        types.SimpleNamespace(
            header=header,
            height=2,
            width=2,
            encoding="rgb8",
            step=6,
            data=np.zeros((2, 2, 3), np.uint8).tobytes(),
        )
    )
    assert runtime.camera_frame_rgba() is not None
    assert runtime.video_mode() == "LIVE"
    runtime.close()


def test_hub_pumps_ros_callbacks_and_forwards_private_scenario_intents(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime
    from arm_cell_simulation_ui_hub.scenario_ports import (
        OwnerScenarioPorts,
        ScenarioIntent,
    )

    received = []
    runtime = HubRuntime(
        node=_Node(),
        scenario_ports=OwnerScenarioPorts(
            rgbd_source=lambda intent: received.append(intent) or True,
            motion_backend=lambda intent: received.append(intent) or True,
        ),
    )
    runtime.spin_once()
    assert runtime._executor.spin_calls == 1
    assert runtime.set_scenario_active("rgbd_source", "target_invalid", True)
    assert runtime.set_scenario_active("motion_backend", "holding_unknown", False)
    assert received == [
        ScenarioIntent("rgbd_source", "target_invalid", True),
        ScenarioIntent("motion_backend", "holding_unknown", False),
    ]
    executor = runtime._executor
    runtime.close()
    assert executor.stopped


def test_hub_marks_missing_owner_services_unavailable_without_faking_outcomes(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    node.clients["/integration/request_material"][1].ready = False
    material_id, material_future = runtime.request_material()
    assert material_future is None
    assert runtime.request_state[material_id] == "unavailable"
    node.clients["/safety/reset"][1].ready = False
    reset_id, reset_future = runtime.acknowledge_and_reset()
    assert reset_future is None
    assert runtime.request_state[reset_id] == "unavailable"
    runtime.close()


def test_isaac_extension_startup_builds_observation_surface_and_cleans_up(monkeypatch):
    _install_ros_types(monkeypatch)

    class _Container:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

    class _Frame:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def set_build_fn(self, callback):
            self.build = callback

    class _Window:
        def __init__(self, *_args, **_kwargs):
            self.frame = _Frame()
            self.visible = False
            self.destroyed = False
            self.width = _kwargs.get("width", 900)
            self.height = _kwargs.get("height", 640)
            self.width_changed_fn = None

        def set_width_changed_fn(self, callback):
            self.width_changed_fn = callback

        def destroy(self):
            self.destroyed = True

    class _Workspace:
        show_functions = {}

        @classmethod
        def set_show_window_fn(cls, name, callback):
            cls.show_functions[name] = callback

        @classmethod
        def show_window(cls, name, visible):
            cls.show_functions[name](visible)

    class _WindowMenu:
        def menu_startup(self, create_window, window_name, _description, _group):
            self.create_window = create_window
            self.window_name = window_name
            self.window = None
            _Workspace.set_show_window_fn(window_name, self._show)

        def _show(self, visible):
            if visible:
                if self.window is None or self.window.destroyed:
                    self.window = self.create_window()
                self.window.visible = True
            elif self.window is not None:
                self.window.destroy()
                self.window = None

        def menu_shutdown(self):
            _Workspace.set_show_window_fn(self.window_name, None)
            if self.window is not None:
                self.window.destroy()
                self.window = None

    class _Widget:
        def __init__(self, text="", **kwargs):
            self.text = text
            self.clicked_fn = kwargs.get("clicked_fn")
            self.enabled = True

    class _ImageProvider:
        def __init__(self):
            self.data = None
            self.size = None

        def set_bytes_data(self, data, size):
            assert isinstance(data, memoryview)
            self.data = np.frombuffer(data, dtype=np.uint8).copy()
            self.size = list(size)

    ui = types.ModuleType("omni.ui")
    ui.Workspace = _Workspace
    ui.Window = _Window
    ui.VStack = lambda **_kwargs: _Container()
    ui.HStack = lambda **_kwargs: _Container()
    ui.ScrollingFrame = lambda **_kwargs: _Container()
    ui.CollapsableFrame = lambda *_args, **_kwargs: _Container()
    ui.ScrollBarPolicy = types.SimpleNamespace(
        SCROLLBAR_AS_NEEDED="as_needed", SCROLLBAR_ALWAYS_OFF="always_off"
    )
    ui.Label = lambda text, **_kwargs: _Widget(text)
    ui.Button = lambda text, **kwargs: _Widget(text, **kwargs)
    ui.ImageWithProvider = lambda *_args, **_kwargs: _Widget()
    ui.ByteImageProvider = _ImageProvider
    omni = types.ModuleType("omni")
    omni.__path__ = []
    monkeypatch.setitem(sys.modules, "omni", omni)
    monkeypatch.setitem(sys.modules, "omni.ui", ui)
    menu_utils = types.ModuleType("omni.kit.menu.utils")
    menu_utils.MenuHelperExtensionFull = _WindowMenu
    monkeypatch.setitem(sys.modules, "omni.kit.menu.utils", menu_utils)

    class _UpdateStream:
        def create_subscription_to_pop(self, *_args, **_kwargs):
            return object()

    kit = types.ModuleType("omni.kit")
    kit.__path__ = []
    app = types.ModuleType("omni.kit.app")
    app.get_app = lambda: types.SimpleNamespace(
        get_update_event_stream=lambda: _UpdateStream()
    )
    monkeypatch.setitem(sys.modules, "omni.kit", kit)
    monkeypatch.setitem(sys.modules, "omni.kit.app", app)

    import arm_cell_simulation_ui_hub.runtime as runtime_module

    class _Runtime:
        request_state = {}

        def __init__(self, **_kwargs):
            self.status = "current"
            self.frame = (np.zeros((720, 1280, 4), dtype=np.uint8), 1280, 720)
            self.request_state = {}
            self.fault_active = {}
            self.fault_request_count = 0

        def spin_once(self):
            pass

        def camera_frame_rgba(self):
            return self.frame

        def selected_target_pixel(self, *, now=None):
            return self.status, (640.0, 360.0) if self.status == "current" else None

        def vision_display(self):
            return "IDLE", None

        def camera_identity(self):
            return (1.0, "camera_optical")

        def video_mode(self):
            return "LIVE"

        def set_display_source(self, source_id):
            self.selected_display_source = source_id

        def observation_text(self, name, *, now=None):
            return f"{name}: current"

        def request_material(self):
            self.request_state["material-request"] = "pending"
            return "request", None

        def acknowledge_and_reset(self):
            self.request_state["reset"] = "pending"
            return "reset", None

        def set_scenario_active(self, _owner, _scenario, _active):
            return True

        def set_external_fault(self, scenario, active):
            self.fault_request_count += 1
            request_id = f"fault-request-{self.fault_request_count}"
            self.request_state[request_id] = "pending"
            self.last_fault_request = (scenario, active)
            return request_id, None

        def close(self):
            pass

    monkeypatch.setattr(runtime_module, "HubRuntime", _Runtime)
    from arm_cell_simulation_ui_hub.extension import SimulationUIHubExtension

    extension = SimulationUIHubExtension()
    extension.on_startup("arm_cell.simulation_ui_hub")
    first_window = extension._window
    assert first_window.visible is True
    assert extension._window_menu.window_name == extension.WINDOW_TITLE
    _Workspace.show_window(extension.WINDOW_TITLE, False)
    assert first_window.destroyed is True
    _Workspace.show_window(extension.WINDOW_TITLE, True)
    assert extension._window is not first_window
    assert extension._window.visible is True
    extension._build()
    assert len(extension._observation_labels) == 7
    assert len(extension._fault_buttons) == 22
    assert set(extension._display_source_buttons) == {"rgb", "depth"}
    assert (
        "external_mock",
        "communication_degradation",
    ) in extension._fault_status_labels
    extension._display_source_buttons["depth"].clicked_fn()
    assert extension.runtime.selected_display_source == "depth"

    e_stop_inject = extension._fault_buttons[("external_mock", "e_stop", True)]
    e_stop_clear = extension._fault_buttons[("external_mock", "e_stop", False)]
    e_stop_inject.clicked_fn()
    fault_id = extension._fault_request_ids[("external_mock", "e_stop")]
    assert extension.runtime.fault_active.get("e_stop", False) is False
    assert e_stop_inject.enabled is False and e_stop_clear.enabled is False
    extension.runtime.request_state[fault_id] = "rejected: unavailable"
    extension._last_ui_update = 0.0
    extension._on_update(None)
    assert e_stop_inject.enabled is True and e_stop_clear.enabled is False

    e_stop_inject.clicked_fn()
    fault_id = extension._fault_request_ids[("external_mock", "e_stop")]
    extension.runtime.request_state[fault_id] = "applied: fault activated"
    extension.runtime.fault_active["e_stop"] = True
    extension._refresh_fault_statuses()
    assert e_stop_inject.enabled is False and e_stop_clear.enabled is True
    assert (
        "does not confirm the system fault state"
        in extension._fault_status_labels[("external_mock", "e_stop")].text
    )

    e_stop_clear.clicked_fn()
    fault_id = extension._fault_request_ids[("external_mock", "e_stop")]
    extension.runtime.request_state[fault_id] = "applied: fault cleared"
    extension.runtime.fault_active["e_stop"] = False
    extension._refresh_fault_statuses()
    assert e_stop_inject.enabled is True and e_stop_clear.enabled is False

    vision_inject = extension._fault_buttons[
        ("rgbd_source", "target_unavailable", True)
    ]
    vision_clear = extension._fault_buttons[
        ("rgbd_source", "target_unavailable", False)
    ]
    extension.runtime.set_scenario_active = lambda *_args: False
    vision_inject.clicked_fn()
    assert extension._scenario_active.get(("rgbd_source", "target_unavailable")) is None
    assert vision_inject.enabled is True and vision_clear.enabled is False

    extension._submit_operator_request("safety_reset")
    extension.runtime.request_state["reset"] = "applied_pending_owner_state"
    extension._last_ui_update = 0.0
    extension._on_update(None)
    assert (
        "not confirmed"
        in extension._operator_request_status_labels["safety_reset"].text
    )

    extension._on_update(None)
    uploaded = extension._image_provider.data.reshape(720, 1280, 4)
    assert extension._image_provider.size == [1280, 720]
    assert np.array_equal(uploaded, extension._latest_image)
    assert uploaded[360, 660, :3].tolist() == [0, 0, 0]
    assert extension.runtime.frame[0].sum() == 0

    extension.runtime.status = "stale"
    extension._on_update(None)
    uploaded = extension._image_provider.data.reshape(720, 1280, 4)
    assert uploaded[360, 640, :3].tolist() == [0, 0, 0]
    extension.runtime.frame = None
    extension._on_update(None)
    assert extension._image_provider.size == [1, 1]
    assert extension._latest_image.sum() == 0
    extension.on_shutdown()
    assert extension.runtime is None
    assert extension._window is None


def test_sidecar_round_trip_clears_camera_frame_when_owner_feed_goes_stale(monkeypatch):
    import arm_cell_simulation_ui_hub.runtime as runtime_module
    import arm_cell_simulation_ui_hub.sidecar as sidecar_module

    feed_is_stale = {"value": False}

    class Runtime:
        def __init__(self, **_kwargs):
            self.request_state = {}
            self.fault_active = {"communication_loss": False}
            self.selected_display_source = "rgb"

        def spin_once(self):
            pass

        def observation_text(self, name, *, now=None):
            state = "stale" if feed_is_stale["value"] else "current"
            return f"{name}: {state}"

        def selected_target_pixel(self, *, now=None):
            if feed_is_stale["value"]:
                return "stale", None
            return "current", (12.0, 18.0)

        def vision_display(self, *, now=None):
            return "IDLE", None

        def camera_identity(self):
            return (1.0, "camera_optical")

        def display_frame_identity(self):
            return self.camera_identity()

        def set_display_source(self, source_id):
            self.selected_display_source = source_id

        def display_source(self):
            return self.selected_display_source

        def display_status(self, *, now=None):
            return f"{self.selected_display_source.upper()} · LIVE"

        def video_mode(self):
            return "LIVE"

        def camera_frame_rgba(self):
            if feed_is_stale["value"]:
                return None
            return np.array([[[1, 2, 3, 255]]], dtype=np.uint8), 1, 1

        def close(self):
            pass

        def set_external_fault(self, scenario, active, *, request_id):
            self.request_state[request_id] = "applied: fault activated"
            self.fault_active[scenario] = active
            return request_id, None

    monkeypatch.setattr(runtime_module, "HubRuntime", Runtime)
    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: False
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)

    client_socket, server_socket = socket.socketpair()
    server = threading.Thread(
        target=sidecar_module._serve,
        args=(server_socket.detach(),),
        daemon=True,
    )
    server.start()
    reader = client_socket.makefile("rb")
    writer = client_socket.makefile("wb")
    proxy = sidecar_module.IsaacRosSidecar.__new__(sidecar_module.IsaacRosSidecar)
    proxy._closed = False
    proxy._rpc_broken = False
    proxy._fault_rpc_in_flight = False
    proxy._observations = {
        name: f"{name}: unavailable" for name in sidecar_module._OBSERVATIONS
    }
    proxy.request_state = {}
    proxy.fault_active = {"communication_loss": False}
    proxy._target_pixel = ("unavailable", None)
    proxy._camera_frame = None
    proxy._camera_identity = None
    proxy._vision_state = "unavailable"
    proxy._vision_overlay = None
    proxy._video_mode = "LIVE"
    proxy._sidecar_available = None
    proxy._sidecar_camera_frame_available = None
    proxy._last_poll_at = 0.0
    proxy._reader = reader
    proxy._writer = writer
    proxy._socket = client_socket
    try:
        assert json.loads(reader.readline()) == {"ready": True}
        proxy.spin_once()
        assert proxy.camera_frame_rgba()[1:] == (1, 1)
        assert np.array_equal(proxy.camera_frame_rgba()[0][0, 0], [1, 2, 3, 255])
        assert proxy.selected_target_pixel() == ("current", (12.0, 18.0))

        fault_id, _future = proxy.set_external_fault("communication_loss", True)
        assert proxy.request_state[fault_id] == "pending"
        deadline = time.monotonic() + 1.0
        while proxy._fault_rpc_in_flight and time.monotonic() < deadline:
            time.sleep(0.001)
        assert fault_id in proxy.request_state
        assert proxy.request_state[fault_id] == "applied: fault activated"
        proxy._last_poll_at = 0.0
        proxy.spin_once()
        assert proxy.fault_active["communication_loss"] is True

        feed_is_stale["value"] = True
        proxy.set_display_source("depth")
        proxy._last_poll_at = 0.0
        proxy.spin_once()
        assert proxy.display_source() == "depth"
        assert proxy.display_status(now=time.monotonic()) == "DEPTH · LIVE"
        assert proxy.selected_target_pixel() == ("stale", None)
        assert proxy.camera_frame_rgba() is None
    finally:
        try:
            writer.write(b'{"op":"shutdown"}\n')
            writer.flush()
            reader.readline()
        except (BrokenPipeError, OSError):
            pass
        reader.close()
        writer.close()
        client_socket.close()
        server.join(timeout=2.0)


def test_sidecar_fault_rpc_does_not_block_polling_or_allow_duplicate_requests():
    from arm_cell_simulation_ui_hub.sidecar import IsaacRosSidecar

    request_entered = threading.Event()
    release_request = threading.Event()
    rpc_calls = []
    proxy = IsaacRosSidecar.__new__(IsaacRosSidecar)
    proxy._closed = False
    proxy._rpc_broken = False
    proxy._fault_rpc_in_flight = False
    proxy._last_poll_at = 0.0
    proxy.request_state = {}
    proxy.fault_active = {"e_stop": False}
    proxy._observations = {"safety": "safety: current"}
    proxy._camera_frame = None
    proxy._camera_identity = None
    proxy._target_pixel = ("current", None)
    proxy._vision_state = "IDLE"
    proxy._vision_overlay = None
    proxy._video_mode = "LIVE"
    proxy._sidecar_available = True
    proxy._sidecar_camera_frame_available = None
    proxy._display_source = "rgb"
    proxy._display_status = "RGB · LIVE"

    def rpc(message, timeout=None):
        rpc_calls.append((message, timeout))
        if message["op"] == "set_external_fault":
            request_entered.set()
            assert release_request.wait(timeout=1.0)
            return {"request_id": message["request_id"], "state": "pending"}
        return {
            "observations": {"safety": "safety: current"},
            "requests": {},
            "fault_active": {"e_stop": False},
            "target_pixel": {},
            "camera_frame_current": False,
            "vision_state": "IDLE",
            "video_mode": "LIVE",
        }

    proxy._rpc = rpc
    request_id, _future = proxy.set_external_fault("e_stop", True)
    assert request_entered.wait(timeout=1.0)
    assert proxy.request_state[request_id] == "pending"
    duplicate_id, _future = proxy.set_external_fault("e_stop", False)
    assert proxy.request_state[duplicate_id] == "failed: sidecar busy"
    proxy.spin_once()
    assert [call[0]["op"] for call in rpc_calls] == ["set_external_fault"]

    release_request.set()
    deadline = time.monotonic() + 1.0
    while proxy._fault_rpc_in_flight and time.monotonic() < deadline:
        time.sleep(0.001)
    assert proxy.request_state[request_id] == "pending"
    proxy._last_poll_at = 0.0
    proxy.spin_once()
    assert rpc_calls[-1][0]["op"] == "poll"
    assert proxy.fault_active["e_stop"] is False


def test_sidecar_fault_rpc_failure_is_reported_without_changing_applied_intent():
    from arm_cell_simulation_ui_hub.sidecar import IsaacRosSidecar

    proxy = IsaacRosSidecar.__new__(IsaacRosSidecar)
    proxy._closed = False
    proxy._rpc_broken = False
    proxy._fault_rpc_in_flight = False
    proxy.request_state = {}
    proxy.fault_active = {"e_stop": True}
    proxy._socket = types.SimpleNamespace(close=lambda: None)
    proxy._rpc = lambda _message, timeout=None: (_ for _ in ()).throw(
        ConnectionError("sidecar disconnected")
    )

    request_id, _future = proxy.set_external_fault("e_stop", False)
    deadline = time.monotonic() + 1.0
    while proxy._fault_rpc_in_flight and time.monotonic() < deadline:
        time.sleep(0.001)

    assert proxy.request_state[request_id].startswith("failed: sidecar communication")
    assert proxy.fault_active["e_stop"] is True
    assert proxy._rpc_broken is True


def test_sidecar_fault_rpc_response_timeout_is_bounded(monkeypatch):
    import arm_cell_simulation_ui_hub.sidecar as sidecar_module

    monkeypatch.setattr(sidecar_module, "_FAULT_RPC_TIMEOUT_SECONDS", 0.02)
    client_socket, server_socket = socket.socketpair()
    proxy = sidecar_module.IsaacRosSidecar.__new__(sidecar_module.IsaacRosSidecar)
    proxy._closed = False
    proxy._rpc_broken = False
    proxy._fault_rpc_in_flight = False
    proxy.request_state = {}
    proxy.fault_active = {"e_stop": False}
    proxy._socket = client_socket
    proxy._reader = client_socket.makefile("rb")
    proxy._writer = client_socket.makefile("wb")

    try:
        request_id, _future = proxy.set_external_fault("e_stop", True)
        assert json.loads(server_socket.recv(4096))["op"] == "set_external_fault"
        deadline = time.monotonic() + 1.0
        while proxy._fault_rpc_in_flight and time.monotonic() < deadline:
            time.sleep(0.001)

        assert proxy.request_state[request_id].startswith(
            "failed: sidecar communication failed"
        )
        assert proxy.fault_active["e_stop"] is False
        assert proxy._rpc_broken is True
    finally:
        proxy._reader.close()
        proxy._writer.close()
        server_socket.close()


def test_sidecar_rpc_failure_clears_camera_and_selected_target():
    from arm_cell_simulation_ui_hub.sidecar import IsaacRosSidecar, _OBSERVATIONS

    proxy = IsaacRosSidecar.__new__(IsaacRosSidecar)
    proxy._closed = False
    proxy._rpc_broken = False
    proxy._fault_rpc_in_flight = False
    proxy._last_poll_at = 0.0
    proxy._sidecar_available = None
    proxy._vision_state = "IDLE"
    proxy._vision_overlay = None
    proxy._video_mode = "LIVE"
    proxy._observations = {name: f"{name}: current" for name in _OBSERVATIONS}
    proxy._camera_frame = (np.zeros((1, 1, 4), dtype=np.uint8), 1, 1)
    proxy._target_pixel = ("current", (12.0, 18.0))
    proxy._rpc = lambda _message: (_ for _ in ()).throw(RuntimeError("disconnected"))

    proxy.spin_once()

    assert proxy.camera_frame_rgba() is None
    assert proxy.selected_target_pixel() == ("unavailable", None)
    assert proxy.vision_display() == ("unavailable", None)
    assert all(value.endswith("unavailable") for value in proxy._observations.values())
