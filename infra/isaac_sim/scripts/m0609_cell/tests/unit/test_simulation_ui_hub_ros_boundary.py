"""ROS-boundary regression using deterministic fake messages and clients."""

import json
import socket
import sys
import threading
import types
import uuid
import time
from pathlib import Path

import numpy as np
import pytest
import yaml


def test_simulation_profile_supplies_handoff_trigger_defaults():
    repository = Path(__file__).resolve().parents[6]
    profile_path = (
        repository
        / "ros2_ws/src/arm_cell_bringup/config/operational_profile.yaml"
    )
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))

    assert profile["simulation"]["external_handoff_material_type"] == "RawPart"
    assert profile["simulation"]["external_handoff_default_quantity"] == 1


class _Future:
    def __init__(self, result):
        self._result = result
        self.callback = None
        self._done = False

    def add_done_callback(self, callback):
        def mark_done(future):
            self._done = True
            callback(future)

        self.callback = mark_done

    def result(self):
        return self._result

    def done(self):
        return self._done


class _Client:
    def __init__(self, response):
        self.response = response
        self.request = None
        self.calls = []
        self.ready = True

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.request = request
        self.calls.append(request)
        self.future = _Future(self.response)
        return self.future


class _Node:
    def __init__(self, *_args):
        self.subscriptions = []
        self.destroyed_subscriptions = []
        self.clients = {}
        self.publishers = []
        self.logs = []
        self.parameter_overrides = {}
        self.action_status_publishers = 1

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
                diagnostic_detail="Safety reset policy applied; observe SafetyState",
                success=True,
                message="fault activated",
            )
        )
        self.clients[topic] = (kind, client)
        return client

    def create_publisher(self, kind, topic, qos):
        publisher = types.SimpleNamespace(messages=[])
        publisher.publish = publisher.messages.append
        self.publishers.append((kind, topic, qos, publisher))
        return publisher

    def destroy_subscription(self, item):
        self.destroyed_subscriptions.append(item)

    def destroy_client(self, _client):
        pass

    def destroy_node(self):
        pass

    def get_publishers_info_by_topic(self, topic):
        if topic != "/orchestration/execute_cycle/_action/status":
            return []
        return [object() for _ in range(self.action_status_publishers)]

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(
                nanoseconds=10_000_000_000,
                to_msg=lambda: types.SimpleNamespace(sec=10, nanosec=0),
            )
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

    class HandoffTrigger:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)
            self.delivery_id = types.SimpleNamespace(uuid=[])

    msg = module(
        "arm_cell_interfaces.msg",
        AMRDockingState=type("AMRDockingState", (), {}),
        HandoffTrigger=HandoffTrigger,
        HandoffTriggerStatus=type("HandoffTriggerStatus", (), {}),
        MissionExitReason=type(
            "MissionExitReason",
            (),
            {
                "MISSION_EXIT_NONE": 0,
                "MISSION_EXIT_DEPLETED": 1,
                "MISSION_EXIT_VISION_ERROR": 2,
                "MISSION_EXIT_MOTION_ERROR": 3,
                "MISSION_EXIT_SAFETY_PREEMPTED": 4,
                "MISSION_EXIT_CANCELED": 5,
                "MISSION_EXIT_CONFIGURATION_ERROR": 6,
                "MISSION_EXIT_QUANTITY_REACHED": 7,
                "MISSION_EXIT_QUANTITY_SHORTFALL": 8,
            },
        ),
        MaterialHandoffState=type("MaterialHandoffState", (), {}),
        MaterialReadiness=type("MaterialReadiness", (), {}),
        MotionStatus=type("MotionStatus", (), {}),
        PackMLState=type("PackMLState", (), {}),
        SafetyState=type("SafetyState", (), {}),
        SceneStability=type("SceneStability", (), {}),
        DetectTargetResultCode=type(
            "DetectTargetResultCode",
            (),
            {"DETECT_RESULT_SUCCESS": 0, "DETECT_RESULT_BUSY": 8},
        ),
        TransferCompletion=type("TransferCompletion", (), {}),
        VisionCalibration=type(
            "VisionCalibration",
            (),
            {"STATE_CALIBRATING": 0, "STATE_READY": 1},
        ),
        VisionDiagnostic=type("VisionDiagnostic", (), {}),
        VisionDetectorSnapshot=type("VisionDetectorSnapshot", (), {}),
    )

    class ResetSafety:
        class Request:
            def __init__(self):
                self.request_id = types.SimpleNamespace(uuid=[])
                self.operator_acknowledged = False

    class DetectTarget:
        class Request:
            def __init__(self):
                self.target_id = ""
                self.timeout = types.SimpleNamespace(sec=0, nanosec=0)

    class GetResultService:
        class Request:
            def __init__(self):
                self.goal_id = None

    ExecuteCycle = type(
        "ExecuteCycle",
        (),
        {"Impl": types.SimpleNamespace(GetResultService=GetResultService)},
    )

    interfaces.msg = msg
    interfaces.srv = module(
        "arm_cell_interfaces.srv",
        DetectTarget=DetectTarget,
        ResetSafety=ResetSafety,
    )
    interfaces.action = module("arm_cell_interfaces.action", ExecuteCycle=ExecuteCycle)
    interfaces.action.__path__ = []

    class SetBool:
        class Request:
            def __init__(self):
                self.data = False

    class Trigger:
        class Request:
            pass

    std_srvs = module("std_srvs")
    std_srvs.__path__ = []
    std_srvs.srv = module("std_srvs.srv", SetBool=SetBool, Trigger=Trigger)
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


def _make_retry_detection_eligible(runtime):
    now = time.monotonic()
    runtime._execute_cycle_status_received_at = now
    runtime.projection.observe(
        "readiness",
        types.SimpleNamespace(material_ready=False, valid=True),
        received_at=now,
        max_age_seconds=runtime.feed_max_age_seconds,
    )
    runtime.projection.observe(
        "vision_diagnostic",
        types.SimpleNamespace(state=0),
        received_at=now,
        max_age_seconds=runtime.feed_max_age_seconds,
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
    assert runtime.display_source() == "none"
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
        "/integration/accepted_handoff",
        "/integration/handoff_trigger_status",
        "/material_supply/transfer_completion",
        "/vision/scene_stability",
        "/vision/calibration_status",
        "/integration/material_readiness",
        "/packml/state",
        "/vision/diagnostics",
        "/vision/detector_snapshot",
        "/camera/color/camera_info",
        "/orchestration/execute_cycle/_action/feedback",
        "/orchestration/execute_cycle/_action/status",
    }
    _kind, _topic, snapshot_callback, snapshot_qos = next(
        item for item in node.subscriptions if item[1] == "/vision/detector_snapshot"
    )
    assert snapshot_qos["depth"] == 1
    assert snapshot_qos["history"] == sys.modules["rclpy.qos"].HistoryPolicy.KEEP_LAST
    assert (
        snapshot_qos["reliability"]
        == sys.modules["rclpy.qos"].ReliabilityPolicy.RELIABLE
    )
    assert (
        snapshot_qos["durability"]
        == sys.modules["rclpy.qos"].DurabilityPolicy.TRANSIENT_LOCAL
    )
    snapshot = types.SimpleNamespace(
        snapshot_id="vision-run-1-4",
        revision=4,
        event_kind=3,
        vision_run_id="vision-run-1",
        calibration_id="cal-1",
        acquisition_id="acq-2",
        image_png=types.SimpleNamespace(format="png", data=[1, 2, 3]),
        metadata_json='{"result_code":"NO_TARGET","failure_cause":"no candidate"}',
    )
    snapshot_callback(snapshot)
    assert runtime.vision_detector_snapshot() == {
        "snapshot_id": "vision-run-1-4",
        "revision": 4,
        "event_kind": 3,
        "vision_run_id": "vision-run-1",
        "calibration_id": "cal-1",
        "acquisition_id": "acq-2",
        "image_format": "png",
        "image_data": bytes([1, 2, 3]),
        "metadata_json": snapshot.metadata_json,
        "diagnostic_record_json": "",
    }
    snapshot.snapshot_id = "vision-run-1-5"
    snapshot.revision = 5
    snapshot.image_png.data = []
    snapshot.metadata_json = ""
    snapshot_callback(snapshot)
    assert runtime.vision_detector_snapshot()["snapshot_id"] == "vision-run-1-5"
    assert runtime.vision_detector_snapshot()["image_data"] == b""
    assert runtime.vision_detector_snapshot()["metadata_json"] == ""
    runtime.set_display_source("depth")
    depth_subscription = next(
        item
        for item in node.subscriptions
        if item[1] == "/camera/aligned_depth_to_color/image_raw"
    )
    runtime.set_display_source("tray")
    assert depth_subscription in node.destroyed_subscriptions
    tray_subscription = next(
        item
        for item in node.subscriptions
        if item[1] == "/vision/tray_aligned_depth/image_raw"
    )
    runtime.set_display_source("none")
    assert tray_subscription in node.destroyed_subscriptions
    assert runtime.camera_frame_rgba() is None
    calibration_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/vision/calibration_status"
    )
    calibration_callback(
        types.SimpleNamespace(
            state=1,
            STATE_READY=1,
            accepted_frames=5,
            required_frames=5,
            median_depth_residual_m=0.0015,
        )
    )
    assert runtime.observation_text("vision_calibration", now=time.monotonic()) == (
        "Tray calibration: READY · frames=5/5 · depth residual=1.5 mm"
    )
    runtime.set_display_source("rgb")
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
    assert set(node.clients) == {
        "/safety/reset",
        "/vision/detect_target",
        "/orchestration/calibrate_vision",
        "/orchestration/execute_cycle/_action/get_result",
    }
    assert len(node.publishers) == 1
    _kind, trigger_topic, _qos, trigger_publisher = node.publishers[0]
    assert trigger_topic == "/external/handoff_trigger"

    trigger_id = runtime.publish_handoff_trigger("RawPart", 3, random_seed=42)
    trigger = trigger_publisher.messages[-1]
    assert str(uuid.UUID(bytes=bytes(trigger.delivery_id.uuid))) == trigger_id
    assert trigger.material_type == "RawPart"
    assert trigger.quantity == 3
    assert trigger.has_random_seed
    assert trigger.random_seed == 42
    assert trigger.valid
    assert trigger.header.stamp.sec == 10

    reset_id, reset_future = runtime.acknowledge_and_reset()
    reset = node.clients["/safety/reset"][1]
    assert str(uuid.UUID(bytes=bytes(reset.request.request_id.uuid))) == reset_id
    assert reset.request.operator_acknowledged
    reset_future.callback(reset_future)
    assert runtime.request_state[reset_id] == "applied_pending_owner_state"
    assert (
        runtime.request_detail[reset_id]
        == "Safety reset policy applied; observe SafetyState"
    )
    rejected = node.clients["/safety/reset"][1]
    rejected.response.applied = False
    rejected.response.diagnostic_detail = "reset policy rejected: motion is active"
    rejected_id, rejected_future = runtime.acknowledge_and_reset()
    rejected_future.callback(rejected_future)
    assert runtime.request_state[rejected_id] == "rejected"
    assert (
        runtime.request_detail[rejected_id] == "reset policy rejected: motion is active"
    )
    runtime.close()


def test_retry_detection_sends_one_vision_request_and_matches_its_snapshot(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    _make_retry_detection_eligible(runtime)
    client = node.clients["/vision/detect_target"][1]
    request_id, future = runtime.request_retry_detection("RawPart")

    assert future is not None
    assert client.request.target_id == "RawPart"
    assert client.request.timeout.sec == 30
    assert client.request.timeout.nanosec == 0
    assert len(client.calls) == 1

    repeated_id, repeated_future = runtime.request_retry_detection("RawPart")
    assert repeated_id == request_id
    assert repeated_future is None
    assert len(client.calls) == 1

    snapshot_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/vision/detector_snapshot"
    )
    snapshot_callback(
        types.SimpleNamespace(
            snapshot_id="unrelated-snapshot",
            revision=2,
            event_kind=2,
            vision_run_id="vision-run-1",
            calibration_id="cal-1",
            acquisition_id="other-acquisition",
            image_png=types.SimpleNamespace(format="png", data=[1]),
            metadata_json="{}",
            diagnostic_record_json="{}",
        )
    )
    snapshot_callback(
        types.SimpleNamespace(
            snapshot_id="retry-snapshot",
            revision=3,
            event_kind=2,
            vision_run_id="vision-run-1",
            calibration_id="cal-1",
            acquisition_id="acq-retry",
            image_png=types.SimpleNamespace(format="png", data=[2, 3]),
            metadata_json='{"result_code":"SUCCESS"}',
            diagnostic_record_json='{"acquisition_id":"acq-retry"}',
        )
    )
    future._result = types.SimpleNamespace(
        result_code=types.SimpleNamespace(value=0),
        acquisition_id="acq-retry",
        diagnostic_detail="stable observation accepted",
    )
    future.callback(future)

    result = runtime.retry_detection_result()
    assert result["request_id"] == request_id
    assert result["state"] == "succeeded"
    assert result["acquisition_id"] == "acq-retry"
    assert result["diagnostic_detail"] == "stable observation accepted"
    assert result["snapshot"]["snapshot_id"] == "retry-snapshot"
    assert result["snapshot"]["acquisition_id"] == "acq-retry"
    assert result["snapshot"]["diagnostic_record_json"] == (
        '{"acquisition_id":"acq-retry"}'
    )
    assert runtime._active_execute_goal_id is None
    assert client.calls[0].target_id == "RawPart"
    assert (
        len(node.clients["/orchestration/execute_cycle/_action/get_result"][1].calls)
        == 0
    )
    assert node.publishers[0][3].messages == []
    runtime.close()


def test_vision_calibration_calls_orchestration_once_and_reports_response(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    client = node.clients["/orchestration/calibrate_vision"][1]

    request_id, future = runtime.request_vision_calibration()
    assert future is not None
    assert isinstance(client.request, sys.modules["std_srvs.srv"].Trigger.Request)
    assert len(client.calls) == 1
    assert runtime.vision_calibration_result()["state"] == "pending"

    repeated_id, repeated_future = runtime.request_vision_calibration()
    assert repeated_id == request_id
    assert repeated_future is None
    assert len(client.calls) == 1

    future._result = types.SimpleNamespace(
        success=True, message="/calibrations/vision-20261014.yaml"
    )
    future.callback(future)
    assert runtime.vision_calibration_result() == {
        "request_id": request_id,
        "state": "succeeded",
        "message": "/calibrations/vision-20261014.yaml",
    }

    runtime.close()


def test_vision_calibration_reports_service_rejection_and_unavailable_service(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    client = node.clients["/orchestration/calibrate_vision"][1]
    client.response = types.SimpleNamespace(success=False, message="Mission is active")
    request_id, future = runtime.request_vision_calibration()
    future.callback(future)
    assert runtime.vision_calibration_result() == {
        "request_id": request_id,
        "state": "failed",
        "message": "Mission is active",
    }

    client.ready = False
    unavailable_id, unavailable_future = runtime.request_vision_calibration()
    assert unavailable_future is None
    assert runtime.vision_calibration_result() == {
        "request_id": unavailable_id,
        "state": "unavailable",
        "message": "Calibration service is unavailable",
    }
    runtime.close()


def test_retry_detection_eligibility_accepts_initial_idle_action_server(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    runtime = HubRuntime(node=_Node())
    runtime.projection.observe(
        "readiness",
        types.SimpleNamespace(material_ready=False, valid=True),
        received_at=time.monotonic(),
        max_age_seconds=runtime.feed_max_age_seconds,
    )
    runtime.projection.observe(
        "vision_diagnostic",
        types.SimpleNamespace(state=0),
        received_at=time.monotonic(),
        max_age_seconds=runtime.feed_max_age_seconds,
    )
    eligible, reason = runtime.retry_detection_eligibility()

    assert eligible
    assert reason == "Available"
    assert runtime.node.clients["/vision/detect_target"][1].calls == []
    runtime.close()


def test_retry_detection_eligibility_accepts_long_idle_after_terminal(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    runtime = HubRuntime(node=_Node())
    _make_retry_detection_eligible(runtime)
    status_callback = next(
        callback
        for _kind, topic, callback, _qos in runtime.node.subscriptions
        if topic == "/orchestration/execute_cycle/_action/status"
    )
    goal_id = bytes(range(16))
    goal_status = types.SimpleNamespace(
        goal_info=types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(goal_id)),
            stamp=types.SimpleNamespace(sec=1, nanosec=0),
        ),
        status=2,
    )
    status_callback(types.SimpleNamespace(status_list=[goal_status]))
    active, reason = runtime.retry_detection_eligibility()
    assert not active
    assert reason == "Mission is active"

    goal_status.status = 4
    status_callback(types.SimpleNamespace(status_list=[goal_status]))
    runtime._execute_cycle_status_received_at = 1.0
    runtime.projection.observe(
        "readiness",
        types.SimpleNamespace(material_ready=False, valid=True),
        received_at=10_000.0,
        max_age_seconds=runtime.feed_max_age_seconds,
    )
    runtime.projection.observe(
        "vision_diagnostic",
        types.SimpleNamespace(state=0),
        received_at=10_000.0,
        max_age_seconds=runtime.feed_max_age_seconds,
    )

    eligible, reason = runtime.retry_detection_eligibility(now=10_000.0)

    assert eligible
    assert reason == "Available"
    assert runtime._execute_cycle_latest_terminal_goal_id == goal_id.hex()
    runtime.close()


def test_retry_detection_eligibility_fails_closed_without_action_server_observation(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    node.action_status_publishers = 0
    runtime = HubRuntime(node=node)
    _make_retry_detection_eligible(runtime)
    runtime._execute_cycle_status_received_at = None

    eligible, reason = runtime.retry_detection_eligibility()

    assert not eligible
    assert "Mission" in reason or "observation" in reason
    runtime.close()


@pytest.mark.parametrize(
    ("blocked_owner", "reason_fragment"),
    (("readiness", "readiness"), ("vision", "Vision"), ("mission", "Mission")),
)
def test_retry_detection_does_not_dispatch_when_owner_state_blocks_it(
    monkeypatch, blocked_owner, reason_fragment
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    runtime = HubRuntime(node=_Node())
    _make_retry_detection_eligible(runtime)
    if blocked_owner == "readiness":
        runtime._receive_material_readiness(
            types.SimpleNamespace(
                material_ready=True,
                valid=True,
                delivery_id=types.SimpleNamespace(uuid=list(bytes(range(16)))),
            )
        )
    elif blocked_owner == "vision":
        runtime.projection.observe(
            "vision_diagnostic",
            types.SimpleNamespace(state=1),
            received_at=time.monotonic(),
            max_age_seconds=runtime.feed_max_age_seconds,
        )
    else:
        runtime._active_execute_goal_id = "active-mission"

    eligible, reason = runtime.retry_detection_eligibility()
    _request_id, future = runtime.request_retry_detection("RawPart")

    assert not eligible
    assert reason_fragment in reason
    assert future is None
    assert runtime.retry_detection_result()["state"] == "unavailable"
    assert runtime.node.clients["/vision/detect_target"][1].calls == []
    runtime.close()


def test_retry_detection_keeps_observed_ready_delivery_pending_until_admission(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    runtime = HubRuntime(node=_Node())
    _make_retry_detection_eligible(runtime)
    readiness_callback = next(
        callback
        for _kind, topic, callback, _qos in runtime.node.subscriptions
        if topic == "/integration/material_readiness"
    )
    delivery_bytes = bytes(range(16))
    readiness_callback(
        types.SimpleNamespace(
            delivery_id=types.SimpleNamespace(uuid=list(delivery_bytes)),
            material_ready=True,
            valid=True,
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=10, nanosec=0)
            ),
        )
    )
    readiness_callback(
        types.SimpleNamespace(
            delivery_id=types.SimpleNamespace(
                uuid=list(bytes(reversed(delivery_bytes)))
            ),
            material_ready=False,
            valid=True,
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=10, nanosec=0)
            ),
        )
    )

    eligible, reason = runtime.retry_detection_eligibility()
    assert not eligible
    assert "pending Mission admission" in reason
    assert len(runtime.node.clients["/vision/detect_target"][1].calls) == 0

    runtime._receive_execute_cycle_feedback(
        types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(bytes(reversed(range(16))))),
            feedback=types.SimpleNamespace(
                delivery_id=types.SimpleNamespace(uuid=list(delivery_bytes))
            ),
        )
    )
    runtime._active_execute_goal_id = None
    readiness_callback(
        types.SimpleNamespace(
            delivery_id=types.SimpleNamespace(uuid=list(delivery_bytes)),
            material_ready=True,
            valid=True,
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=10, nanosec=0)
            ),
        )
    )
    eligible, _reason = runtime.retry_detection_eligibility()
    assert eligible
    runtime.close()


def test_retry_detection_does_not_match_terminal_snapshot_from_previous_vision_run(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    _make_retry_detection_eligible(runtime)
    _request_id, future = runtime.request_retry_detection("RawPart")
    snapshot_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/vision/detector_snapshot"
    )

    snapshot_callback(
        types.SimpleNamespace(
            snapshot_id="old-run-terminal",
            revision=1,
            event_kind=3,
            vision_run_id="vision-run-old",
            calibration_id="cal-old",
            acquisition_id="vision-run-old-acq-1",
            image_png=types.SimpleNamespace(format="png", data=[1]),
            metadata_json="{}",
        )
    )
    future._result = types.SimpleNamespace(
        result_code=types.SimpleNamespace(value=0),
        acquisition_id="vision-run-new-acq-1",
        diagnostic_detail="current run result",
    )
    future.callback(future)

    result = runtime.retry_detection_result()
    assert result["state"] == "succeeded"
    assert result["snapshot"] is None

    snapshot_callback(
        types.SimpleNamespace(
            snapshot_id="new-run-terminal",
            revision=1,
            event_kind=3,
            vision_run_id="vision-run-new",
            calibration_id="cal-new",
            acquisition_id="vision-run-new-acq-1",
            image_png=types.SimpleNamespace(format="png", data=[2]),
            metadata_json="{}",
        )
    )
    result = runtime.retry_detection_result()
    assert result["snapshot"]["snapshot_id"] == "new-run-terminal"
    runtime.close()


def test_retry_detection_busy_response_has_no_acquisition_or_snapshot(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    _make_retry_detection_eligible(runtime)
    client = node.clients["/vision/detect_target"][1]
    request_id, future = runtime.request_retry_detection("RawPart")
    future._result = types.SimpleNamespace(
        result_code=types.SimpleNamespace(value=8),
        acquisition_id="",
        diagnostic_detail="request contention: another DetectTarget acquisition is active",
    )
    future.callback(future)

    result = runtime.retry_detection_result()
    assert result["request_id"] == request_id
    assert result["state"] == "busy"
    assert result["acquisition_id"] == ""
    assert result["snapshot"] is None
    assert result["diagnostic_detail"].startswith("request contention:")
    assert len(client.calls) == 1
    assert runtime._active_execute_goal_id is None
    assert (
        len(node.clients["/orchestration/execute_cycle/_action/get_result"][1].calls)
        == 0
    )
    assert node.publishers[0][3].messages == []
    runtime.close()


def test_retry_detection_service_failure_has_no_acquisition_or_snapshot(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    runtime = HubRuntime(node=_Node())
    _make_retry_detection_eligible(runtime)
    request_id, future = runtime.request_retry_detection("RawPart")

    def fail_response():
        raise RuntimeError("service transport failed")

    future.result = fail_response
    future.callback(future)
    result = runtime.retry_detection_result()

    assert result["request_id"] == request_id
    assert result["state"] == "failed"
    assert result["acquisition_id"] == ""
    assert result["snapshot"] is None
    assert result["diagnostic_detail"] == "service transport failed"
    assert len(runtime.node.clients["/vision/detect_target"][1].calls) == 1
    runtime.close()


def test_retry_detection_times_out_at_the_composed_client_bound_and_allows_retry(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    import arm_cell_simulation_ui_hub.runtime as runtime_module

    node = _Node()
    runtime = runtime_module.HubRuntime(node=node)
    client = node.clients["/vision/detect_target"][1]
    now = [100.0]
    monkeypatch.setattr(runtime_module.time, "monotonic", lambda: now[0])
    _make_retry_detection_eligible(runtime)

    first_id, first_future = runtime.request_retry_detection("RawPart")
    assert first_future is not None

    now[0] = 129.999
    runtime.spin_once()
    assert runtime.retry_detection_inflight
    assert runtime.retry_detection_result()["state"] == "pending"

    now[0] = 160.099
    runtime.spin_once()
    assert runtime.retry_detection_inflight

    now[0] = 160.1
    runtime.spin_once()
    result = runtime.retry_detection_result()
    assert not runtime.retry_detection_inflight
    assert result["request_id"] == first_id
    assert result["state"] == "failed"
    assert result["acquisition_id"] == ""
    assert result["snapshot"] is None
    assert "30" in result["diagnostic_detail"]

    _make_retry_detection_eligible(runtime)
    second_id, second_future = runtime.request_retry_detection("RawPart")
    assert second_future is not None
    assert second_id != first_id
    assert len(client.calls) == 2

    first_future._result = types.SimpleNamespace(
        result_code=types.SimpleNamespace(value=0),
        acquisition_id="acq-timed-out",
        diagnostic_detail="late response after timeout",
    )
    first_future.callback(first_future)
    current_result = runtime.retry_detection_result()
    assert current_result["request_id"] == second_id
    assert current_result["state"] == "pending"
    assert current_result["acquisition_id"] == ""
    assert current_result["snapshot"] is None
    assert runtime.retry_detection_inflight
    runtime.close()


def test_competing_vision_detection_does_not_extend_retry_detection_deadline(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    import arm_cell_simulation_ui_hub.runtime as runtime_module

    node = _Node()
    runtime = runtime_module.HubRuntime(node=node)
    now = [100.0]
    monkeypatch.setattr(runtime_module.time, "monotonic", lambda: now[0])
    _make_retry_detection_eligible(runtime)
    request_id, future = runtime.request_retry_detection("RawPart")
    assert future is not None

    now[0] = 105.0
    diagnostic_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/vision/diagnostics"
    )
    diagnostic_callback(types.SimpleNamespace(state=1))

    # VisionDiagnostic is global: another caller's DETECTING must not move
    # this request's independent finite response deadline.
    assert runtime._retry_detection_deadline == 160.1

    now[0] = 160.0
    runtime.spin_once()
    assert runtime.retry_detection_inflight

    future._result = types.SimpleNamespace(
        result_code=types.SimpleNamespace(value=0),
        acquisition_id="acq-terminal-edge",
        diagnostic_detail="terminal at Vision's 30 second boundary",
    )
    future._done = True
    now[0] = 160.1
    runtime.spin_once()
    result = runtime.retry_detection_result()
    assert result["request_id"] == request_id
    assert result["state"] == "succeeded"
    assert result["acquisition_id"] == "acq-terminal-edge"
    runtime.close()


def test_late_retry_detection_future_cannot_overwrite_a_new_request(monkeypatch):
    _install_ros_types(monkeypatch)
    import arm_cell_simulation_ui_hub.runtime as runtime_module

    node = _Node()
    runtime = runtime_module.HubRuntime(node=node)
    now = [0.0]
    monkeypatch.setattr(runtime_module.time, "monotonic", lambda: now[0])
    _make_retry_detection_eligible(runtime)
    client = node.clients["/vision/detect_target"][1]

    first_id, first_future = runtime.request_retry_detection("RawPart")
    now[0] = 60.1
    runtime.spin_once()
    _make_retry_detection_eligible(runtime)
    second_id, second_future = runtime.request_retry_detection("RawPart")
    assert second_future is not None

    second_future._result = types.SimpleNamespace(
        result_code=types.SimpleNamespace(value=0),
        acquisition_id="acq-new",
        diagnostic_detail="current response",
    )
    second_future.callback(second_future)
    snapshot_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/vision/detector_snapshot"
    )
    snapshot_callback(
        types.SimpleNamespace(
            snapshot_id="new-request-snapshot",
            revision=1,
            event_kind=2,
            vision_run_id="vision-run-new",
            calibration_id="cal-new",
            acquisition_id="acq-new",
            image_png=types.SimpleNamespace(format="png", data=[4]),
            metadata_json="{}",
        )
    )

    first_future._result = types.SimpleNamespace(
        result_code=types.SimpleNamespace(value=0),
        acquisition_id="acq-old",
        diagnostic_detail="late old response",
    )
    first_future.callback(first_future)
    result = runtime.retry_detection_result()
    assert result["request_id"] == second_id
    assert result["request_id"] != first_id
    assert result["state"] == "succeeded"
    assert result["acquisition_id"] == "acq-new"
    assert result["diagnostic_detail"] == "current response"
    assert result["snapshot"]["snapshot_id"] == "new-request-snapshot"
    assert not runtime.retry_detection_inflight
    assert runtime._active_execute_goal_id is None
    assert (
        len(node.clients["/orchestration/execute_cycle/_action/get_result"][1].calls)
        == 0
    )
    assert node.publishers[0][3].messages == []
    assert len(client.calls) == 2
    runtime.close()


def test_retry_detection_future_callback_after_close_does_not_mutate_runtime(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    _make_retry_detection_eligible(runtime)
    _request_id, future = runtime.request_retry_detection("RawPart")
    runtime.close()
    closed_result = runtime.retry_detection_result()

    future._result = types.SimpleNamespace(
        result_code=types.SimpleNamespace(value=0),
        acquisition_id="acq-after-close",
        diagnostic_detail="response after close",
    )
    future.callback(future)

    assert runtime.retry_detection_result() == closed_result
    assert not runtime.retry_detection_inflight


def test_hub_preserves_safety_reset_diagnostic_detail(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    client = node.clients["/safety/reset"][1]
    client.response.applied = False
    client.response.diagnostic_detail = "Safety reset rejected: motion_not_inactive"

    request_id, future = runtime.acknowledge_and_reset()
    future.callback(future)

    assert runtime.request_state[request_id] == "rejected"
    assert (
        runtime.request_detail[request_id]
        == "Safety reset rejected: motion_not_inactive"
    )
    runtime.close()


def test_execute_cycle_terminal_status_and_canonical_result_are_observed_separately(
    monkeypatch,
):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    goal_id = bytes(range(16))
    runtime._receive_execute_cycle_feedback(
        types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(goal_id)),
            feedback=types.SimpleNamespace(
                phase=0,
                batch_processed_count=0,
                quantity=1,
                retry_count=0,
                target_valid=False,
                interruption_count=0,
                first_interruption_reason=types.SimpleNamespace(value=0),
                first_interruption_causes=types.SimpleNamespace(value=0),
                first_interruption_stop_mode=types.SimpleNamespace(value=0),
            ),
        )
    )
    status = types.SimpleNamespace(
        goal_info=types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(goal_id))
        ),
        status=6,
    )
    runtime._receive_execute_cycle_status(types.SimpleNamespace(status_list=[status]))

    client = node.clients["/orchestration/execute_cycle/_action/get_result"][1]
    assert bytes(client.request.goal_id.uuid) == goal_id
    client.future._result = types.SimpleNamespace(
        status=6,
        result=types.SimpleNamespace(
            exit_reason=types.SimpleNamespace(value=8),
            diagnostic_detail="valid target depleted before requested quantity",
        ),
    )
    client.future.callback(client.future)

    text = runtime.observation_text("execute_cycle", now=time.monotonic())
    assert "terminal=ABORTED" in text
    assert "exit_reason=MISSION_EXIT_QUANTITY_SHORTFALL" in text
    assert "valid target depleted before requested quantity" in text
    runtime.close()


def test_execute_cycle_configuration_result_is_observed_without_feedback(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    goal_id = bytes(range(16))
    status = types.SimpleNamespace(
        goal_info=types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(goal_id)),
            stamp=types.SimpleNamespace(sec=10, nanosec=0),
        ),
        status=6,
    )
    runtime._receive_execute_cycle_status(types.SimpleNamespace(status_list=[status]))

    client = node.clients["/orchestration/execute_cycle/_action/get_result"][1]
    assert bytes(client.request.goal_id.uuid) == goal_id
    client.future._result = types.SimpleNamespace(
        status=6,
        result=types.SimpleNamespace(
            exit_reason=types.SimpleNamespace(value=6),
            diagnostic_detail="mission recipe is missing grasp width",
        ),
    )
    client.future.callback(client.future)

    text = runtime.observation_text("execute_cycle", now=time.monotonic())
    assert "terminal=ABORTED" in text
    assert "exit_reason=MISSION_EXIT_CONFIGURATION_ERROR" in text
    assert "mission recipe is missing grasp width" in text
    for index in range(1, 10):
        runtime._remember_execute_cycle_goal(f"recent-{index}")
    assert len(runtime._execute_cycle_recent_goal_ids) == 8
    assert goal_id.hex() not in runtime._execute_cycle_status_by_goal
    runtime.close()


def test_execute_cycle_result_query_retries_after_service_becomes_ready(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    goal_id = bytes(range(16))
    goal = types.SimpleNamespace(uuid=list(goal_id))
    client = node.clients["/orchestration/execute_cycle/_action/get_result"][1]
    client.ready = False
    runtime._observe_execute_cycle_terminal(
        types.SimpleNamespace(goal_info=types.SimpleNamespace(goal_id=goal), status=6)
    )

    assert goal_id.hex() in runtime._execute_cycle_pending_results
    assert client.request is None

    client.ready = True
    pending_goal, _ = runtime._execute_cycle_pending_results[goal_id.hex()]
    runtime._execute_cycle_pending_results[goal_id.hex()] = (pending_goal, 0.0)
    runtime._poll_execute_cycle_results()

    assert bytes(client.request.goal_id.uuid) == goal_id
    assert goal_id.hex() not in runtime._execute_cycle_pending_results
    runtime.close()


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
            material_type="RawPart",
            quantity=3,
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


def test_hub_projects_accepted_trigger_supply_counts_and_quantity_feedback(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    runtime.set_display_source("rgb")
    delivery_id = uuid.uuid4()
    message_fields = {
        "header": types.SimpleNamespace(stamp=types.SimpleNamespace(sec=10, nanosec=0)),
        "delivery_id": types.SimpleNamespace(uuid=list(delivery_id.bytes)),
        "material_type": "RawPart",
        "quantity": 4,
        "has_random_seed": True,
        "random_seed": 91,
        "requested_quantity": 4,
        "supplied_quantity": 3,
        "completed": False,
        "valid": True,
        "failure_reason": 4,
    }
    callbacks = {topic: callback for _kind, topic, callback, _qos in node.subscriptions}
    accepted_message = types.SimpleNamespace(**message_fields)
    completion_message = types.SimpleNamespace(**message_fields)
    callbacks["/integration/accepted_handoff"](accepted_message)
    callbacks["/material_supply/transfer_completion"](completion_message)
    now = time.monotonic()
    accepted = runtime.observation_text("accepted_handoff", now=now)
    supply = runtime.observation_text("transfer_completion", now=now)
    assert str(delivery_id) in accepted
    assert "type=RawPart quantity=4 seed=91" in accepted
    assert "supplied=3" in supply
    assert "stabilized=" not in supply
    assert "completed=False valid=True failure=canceled" in supply
    callbacks["/integration/handoff_trigger_status"](
        types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=10, nanosec=0)
            ),
            delivery_id=types.SimpleNamespace(uuid=list(delivery_id.bytes)),
            state=0,
            rejection_reason=0,
            TRIGGER_ACCEPTED=0,
            TRIGGER_REJECTED=1,
            REJECTION_NONE=0,
            REJECTION_DUPLICATE_DELIVERY_ID=6,
            diagnostic_detail="",
        )
    )
    now = time.monotonic()
    decision = runtime.observation_text("trigger_status", now=now)
    assert str(delivery_id) in decision
    assert "state=accepted rejection=none" in decision
    callbacks["/integration/handoff_trigger_status"](
        types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=10, nanosec=0)
            ),
            delivery_id=types.SimpleNamespace(uuid=list(delivery_id.bytes)),
            state=1,
            rejection_reason=6,
            TRIGGER_ACCEPTED=0,
            TRIGGER_REJECTED=1,
            REJECTION_NONE=0,
            REJECTION_DUPLICATE_DELIVERY_ID=6,
            diagnostic_detail="duplicate_delivery_id",
        )
    )
    now = time.monotonic()
    decision = runtime.observation_text("trigger_status", now=now)
    assert "state=rejected rejection=duplicate_delivery_id" in decision
    completion_message.header.stamp.sec = 8
    callbacks["/material_supply/transfer_completion"](completion_message)
    assert runtime.observation_text("transfer_completion", now=now) == (
        "transfer_completion: stale"
    )

    scene = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=10, nanosec=0)),
        target_id="RawPart",
        scene_generation=7,
        status=2,
    )
    callbacks["/vision/scene_stability"](scene)
    assert runtime.observation_text("scene_stability", now=time.monotonic()) == (
        "vision scene: current · target=RawPart status=STABLE generation=7"
    )

    feedback = types.SimpleNamespace(
        phase="PICK",
        batch_processed_count=2,
        quantity=4,
        retry_count=0,
        target_valid=True,
        interruption_count=0,
        first_interruption_reason=types.SimpleNamespace(value=0),
        first_interruption_causes=types.SimpleNamespace(value=0),
        first_interruption_stop_mode=types.SimpleNamespace(value=0),
    )
    runtime._observe(
        "execute_cycle",
        types.SimpleNamespace(
            goal_id=types.SimpleNamespace(uuid=list(bytes(range(16)))),
            feedback=feedback,
        ),
    )
    now = time.monotonic()
    mission = runtime.observation_text("execute_cycle", now=now)
    assert "processed=2/4" in mission

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


def test_selected_sources_release_and_reopen_around_frozen_pnp_result(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    owner_callbacks = {
        topic: callback for _kind, topic, callback, _qos in node.subscriptions
    }
    goal_id = bytes(range(16))
    owner_callbacks["/orchestration/execute_cycle/_action/feedback"](
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

    runtime.set_display_source("rgb")
    rgb_subscription = next(
        item for item in node.subscriptions if item[1] == "/camera/color/image_raw"
    )
    rgb_subscription[2](
        types.SimpleNamespace(
            header=header(5.0),
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=bytes((10, 20, 30)),
        )
    )
    owner_callbacks["/vision/diagnostics"](
        types.SimpleNamespace(
            state=0,
            result_code=0,
            valid=True,
            source_rgb_header=header(5.0),
            target_id="RawPart",
            support_region_valid=False,
            support_region=[],
        )
    )
    assert runtime._detection_snapshot is not None

    runtime.set_display_source("depth")
    assert rgb_subscription in node.destroyed_subscriptions
    depth_subscription = next(
        item
        for item in node.subscriptions
        if item[1] == "/camera/aligned_depth_to_color/image_raw"
    )
    assert runtime.camera_frame_rgba() is None

    from action_msgs.msg import GoalStatus

    owner_callbacks["/orchestration/execute_cycle/_action/status"](
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
    depth_subscription[2](
        types.SimpleNamespace(
            header=header(5.1),
            height=1,
            width=1,
            encoding="32FC1",
            step=4,
            is_bigendian=False,
            data=np.array([[0.5]], dtype=np.float32).tobytes(),
        )
    )
    assert runtime.camera_frame_rgba() is not None

    runtime.set_display_source("rgb")
    assert depth_subscription in node.destroyed_subscriptions
    rgb_subscription = next(
        item for item in node.subscriptions if item[1] == "/camera/color/image_raw"
    )
    rgb_subscription[2](
        types.SimpleNamespace(
            header=header(5.2),
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=bytes((1, 2, 3)),
        )
    )
    assert runtime.camera_identity() == (5.2, "camera_color_optical_frame")
    runtime.close()


def test_unavailable_depth_does_not_prevent_rgb_pnp_snapshot(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    runtime.set_display_source("rgb")
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
            support_region_valid=False,
            support_region=[],
        )
    )
    assert runtime._detection_snapshot is not None
    assert runtime._detection_snapshot.depth_frame is None
    assert runtime.camera_frame_rgba() is not None
    assert runtime.video_mode() == "DETECTION SNAPSHOT · PnP ACTIVE"
    runtime.close()


def test_depth_display_is_subscribed_and_transformed_only_when_selected(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    assert not any(
        item[1] == "/camera/aligned_depth_to_color/image_raw"
        for item in node.subscriptions
    )
    conversion_count = [0]
    depth_source = runtime._display_sources["depth"]
    original_convert = depth_source.convert

    def counted_convert(*args, **kwargs):
        conversion_count[0] += 1
        return original_convert(*args, **kwargs)

    monkeypatch.setattr(depth_source, "convert", counted_convert)
    runtime.set_display_source("depth")
    depth_subscription = next(
        item
        for item in node.subscriptions
        if item[1] == "/camera/aligned_depth_to_color/image_raw"
    )
    assert depth_subscription[3]["depth"] == 1
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
    frame = runtime.display_frame()
    assert frame is not None
    assert frame.source_id == "depth"
    assert conversion_count == [1]
    runtime.display_frame()
    assert conversion_count == [1]
    runtime.set_display_source("none")
    assert depth_subscription in node.destroyed_subscriptions
    assert runtime.camera_frame_rgba() is None
    runtime.close()


def test_depth_mode_skips_unselected_rgb_annotation_overlay(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    runtime.set_display_source("rgb")
    rgb_subscription = next(
        item for item in node.subscriptions if item[1] == "/camera/color/image_raw"
    )
    header = types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=8, nanosec=0),
        frame_id="camera_color_optical_frame",
    )
    rgb_subscription[2](
        types.SimpleNamespace(
            header=header,
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=bytes((1, 2, 3)),
        )
    )
    diagnostic = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/vision/diagnostics"
    )
    diagnostic(
        types.SimpleNamespace(
            state=0,
            result_code=0,
            valid=True,
            source_rgb_header=header,
            target_id="RawPart",
            support_region_valid=False,
            support_region=[],
        )
    )
    assert runtime.vision_display()[1] is not None

    runtime.set_display_source("depth")
    assert rgb_subscription in node.destroyed_subscriptions
    depth_subscription = next(
        item
        for item in node.subscriptions
        if item[1] == "/camera/aligned_depth_to_color/image_raw"
    )
    depth_subscription[2](
        types.SimpleNamespace(
            header=header,
            height=1,
            width=1,
            encoding="32FC1",
            step=4,
            is_bigendian=False,
            data=np.array([[0.5]], dtype=np.float32).tobytes(),
        )
    )
    assert runtime.camera_frame_rgba() is not None
    assert runtime.vision_display()[1] is None
    runtime.close()


def test_unsupported_depth_is_unavailable_without_affecting_rgb(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    runtime.set_display_source("rgb")
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
    runtime.set_display_source("depth")
    depth_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/camera/aligned_depth_to_color/image_raw"
    )
    depth_callback(
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
    assert runtime.camera_frame_rgba() is None
    assert "UNAVAILABLE" in runtime.display_status()
    runtime.set_display_source("rgb")
    rgb_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/camera/color/image_raw"
    )
    rgb_callback(
        types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=10, nanosec=0),
                frame_id="camera_optical",
            ),
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=bytes((1, 2, 3)),
        )
    )
    assert runtime.camera_frame_rgba() is not None
    assert runtime.video_mode() == "LIVE"
    runtime.close()


def test_stale_depth_is_unavailable_without_staling_rgb_or_vision(monkeypatch):
    _install_ros_types(monkeypatch)
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    node = _Node()
    runtime = HubRuntime(node=node)
    runtime.set_display_source("rgb")
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
    runtime.set_display_source("depth")
    depth_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/camera/aligned_depth_to_color/image_raw"
    )
    depth_callback(
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

    assert runtime.camera_frame_rgba() is None
    assert "STALE" in runtime.display_status()
    runtime.set_display_source("rgb")
    rgb_callback = next(
        callback
        for _kind, topic, callback, _qos in node.subscriptions
        if topic == "/camera/color/image_raw"
    )
    rgb_callback(
        types.SimpleNamespace(
            header=rgb_header,
            height=1,
            width=1,
            encoding="rgb8",
            step=3,
            data=bytes((11, 22, 33)),
        )
    )
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
    runtime.set_display_source("rgb")
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
            support_region_valid=False,
            support_region=[],
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


def test_compatible_operator_rgb_lookup_selects_nearest_bounded_frame_and_rejects(
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
    runtime.set_display_source("rgb")
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
            support_region_valid=True,
            support_region=[
                types.SimpleNamespace(x=0.0, y=0.0),
                types.SimpleNamespace(x=2.0, y=0.0),
                types.SimpleNamespace(x=2.0, y=2.0),
            ],
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
    assert "object_region" not in metadata
    assert "centroid" not in metadata
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
    runtime.set_display_source("rgb")
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
            support_region_valid=False,
            support_region=[],
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
    runtime.set_display_source("rgb")
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
            support_region_valid=False,
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
        "support_region": [],
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
    runtime.set_display_source("rgb")
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
        support_region_valid=True,
        support_region=[types.SimpleNamespace(x=1.0, y=2.0)],
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
    runtime.set_display_source("rgb")
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
    runtime.set_display_source("rgb")
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
    trigger_id = runtime.publish_handoff_trigger("RawPart", 2)
    assert trigger_id
    assert len(node.publishers[0][3].messages) == 1
    assert node.publishers[0][3].messages[0].has_random_seed is False
    node.clients["/safety/reset"][1].ready = False
    reset_id, reset_future = runtime.acknowledge_and_reset()
    assert reset_future is None
    assert runtime.request_state[reset_id] == "unavailable"
    runtime.close()


def test_isaac_extension_builds_focused_surface_and_cleans_up(monkeypatch):
    _install_ros_types(monkeypatch)
    collapsible_sections = []
    observation_calls = []

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
            self.model = types.SimpleNamespace(value=None)
            self.model.set_value = lambda value: setattr(self.model, "value", value)
            self.model.get_value_as_string = lambda: str(self.model.value or "")
            self.model.get_value_as_int = lambda: int(self.model.value or 0)
            self.model.get_value_as_bool = lambda: bool(self.model.value)
            self.model.get_item_value_model = lambda: self.model

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
    ui.Pixel = lambda value: value

    def _collapsible_frame(title, **_kwargs):
        collapsible_sections.append(title)
        return _Container()

    ui.CollapsableFrame = _collapsible_frame
    ui.ScrollBarPolicy = types.SimpleNamespace(
        SCROLLBAR_AS_NEEDED="as_needed", SCROLLBAR_ALWAYS_OFF="always_off"
    )
    ui.Label = lambda text, **_kwargs: _Widget(text)
    ui.Button = lambda text, **kwargs: _Widget(text, **kwargs)
    ui.StringField = lambda **kwargs: _Widget(**kwargs)
    ui.IntField = lambda **kwargs: _Widget(**kwargs)
    ui.ComboBox = lambda *_args, **kwargs: _Widget(**kwargs)
    ui.ImageWithProvider = lambda *_args, **_kwargs: _Widget()
    ui.ByteImageProvider = _ImageProvider
    omni = types.ModuleType("omni")
    omni.__path__ = []
    monkeypatch.setitem(sys.modules, "omni", omni)
    monkeypatch.setitem(sys.modules, "omni.ui", ui)
    menu_utils = types.ModuleType("omni.kit.menu.utils")
    menu_utils.MenuHelperExtensionFull = _WindowMenu
    monkeypatch.setitem(sys.modules, "omni.kit.menu.utils", menu_utils)

    subscriptions = []

    class _Subscription:
        def __init__(self):
            self.unsubscribe_calls = 0

        def unsubscribe(self):
            self.unsubscribe_calls += 1

        @property
        def active(self):
            return self.unsubscribe_calls == 0

    class _UpdateStream:
        def create_subscription_to_pop(self, *_args, **_kwargs):
            subscription = _Subscription()
            subscriptions.append(subscription)
            return subscription

    timeline_subscriptions = []

    class _TimelineStream:
        def create_subscription_to_pop(self, *_args, **_kwargs):
            subscription = _Subscription()
            timeline_subscriptions.append(subscription)
            return subscription

    kit = types.ModuleType("omni.kit")
    kit.__path__ = []
    app = types.ModuleType("omni.kit.app")

    def _get_update_event_stream():
        stream = _UpdateStream()
        return stream

    app.get_app = lambda: types.SimpleNamespace(
        get_update_event_stream=_get_update_event_stream
    )
    omni.kit = kit
    kit.app = app
    monkeypatch.setitem(sys.modules, "omni.kit", kit)
    monkeypatch.setitem(sys.modules, "omni.kit.app", app)
    timeline_module = types.ModuleType("omni.timeline")
    timeline_module.TimelineEventType = types.SimpleNamespace(STOP=4)
    timeline_module.get_timeline_interface = lambda: types.SimpleNamespace(
        get_timeline_event_stream=lambda: _TimelineStream(), is_playing=lambda: False
    )
    monkeypatch.setitem(sys.modules, "omni.timeline", timeline_module)
    monkeypatch.setattr(omni, "timeline", timeline_module, raising=False)

    import arm_cell_simulation_ui_hub.runtime as runtime_module

    runtimes = []

    class _Runtime:
        request_state = {}

        def __init__(self, **_kwargs):
            self.closed = False
            runtimes.append(self)
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
            observation_calls.append(name)
            return f"{name}: current"

        def publish_handoff_trigger(self, material_type, quantity, *, random_seed=None):
            self.last_trigger = (material_type, quantity, random_seed)
            return "trigger-identity"

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

        def set_vision_mislocalize(self, active):
            request_id = "vision-request"
            self.request_state[request_id] = "pending"
            self.last_vision_request = active
            return request_id, None

        def close(self):
            self.closed = True

    monkeypatch.setattr(runtime_module, "HubRuntime", _Runtime)
    from arm_cell_simulation_ui_hub.extension import SimulationUIHubExtension

    extension = SimulationUIHubExtension()
    extension.on_startup("arm_cell.simulation_ui_hub")
    first_window = extension._window
    assert first_window.visible is True
    assert extension._window_menu.window_name == extension.WINDOW_TITLE
    assert len(timeline_subscriptions) == 1
    _Workspace.show_window(extension.WINDOW_TITLE, False)
    assert first_window.destroyed is True
    _Workspace.show_window(extension.WINDOW_TITLE, True)
    assert extension._window is not first_window
    assert extension._window.visible is True
    collapsible_sections.clear()
    extension._retry_detection_enabled = True
    extension._build()
    assert collapsible_sections[0] == "External Handoff Trigger"
    assert "System Observations" not in collapsible_sections
    assert "Vision Detector Snapshot" in collapsible_sections
    assert "Cuboid Demonstration" in collapsible_sections
    assert extension._vision_calibration_button.text == "Calibrate Vision"
    assert "Live Camera (optional)" in collapsible_sections
    assert "Timeline State" not in collapsible_sections
    assert "Developer Tools · Vision only" in collapsible_sections
    assert not hasattr(extension, "_observation_labels")
    assert len(extension._fault_buttons) == 4
    assert {
        scenario for _owner, scenario, active in extension._fault_buttons if active
    } == {"e_stop", "safety_signal_loss", "mislocalize_next"}
    assert set(extension._display_source_buttons) == {"none", "rgb", "depth", "tray"}
    assert not hasattr(extension, "_simulation_buttons")
    extension._handoff_quantity_field.model.set_value(2)
    extension._publish_handoff_trigger()
    assert extension.runtime.last_trigger == ("RawPart", 2, None)
    assert not hasattr(extension, "_handoff_seed_field")
    assert (
        "trigger stimulus published" in extension._handoff_trigger_status.text.lower()
    )
    assert not observation_calls
    assert set(scenario for _owner, scenario in extension._fault_status_labels) == {
        "e_stop",
        "safety_signal_loss",
        "mislocalize_next",
    }
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
        ("external_mock", "mislocalize_next", True)
    ]
    vision_inject.clicked_fn()
    assert extension.runtime.last_vision_request is True
    assert vision_inject.enabled is False

    extension._submit_operator_request("safety_reset")
    extension.runtime.request_state["reset"] = "applied_pending_owner_state"
    extension.runtime.request_detail = {
        "reset": "Safety reset policy applied; observe SafetyState"
    }
    extension._last_ui_update = 0.0
    extension._on_update(None)
    assert (
        "not confirmed"
        in extension._operator_request_status_labels["safety_reset"].text
    )
    assert (
        "Safety reset policy applied"
        in extension._operator_request_status_labels["safety_reset"].text
    )
    extension.runtime.request_state["reset"] = "rejected"
    extension.runtime.request_detail["reset"] = (
        "Safety reset rejected: motion_not_inactive"
    )
    extension._refresh_operator_request_statuses()
    assert (
        "motion_not_inactive"
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
    assert subscriptions[0].unsubscribe_calls == 1
    assert timeline_subscriptions[0].unsubscribe_calls == 1
    assert extension._timer is None

    extension.on_startup("arm_cell.simulation_ui_hub")
    assert len(runtimes) == 2
    assert sum(not runtime.closed for runtime in runtimes) == 1
    assert sum(subscription.active for subscription in subscriptions) == 1
    extension.on_shutdown()
    assert sum(not runtime.closed for runtime in runtimes) == 0
    assert sum(subscription.active for subscription in subscriptions) == 0
    assert sum(subscription.active for subscription in timeline_subscriptions) == 0


def test_sidecar_round_trip_clears_camera_frame_when_owner_feed_goes_stale(monkeypatch):
    import arm_cell_simulation_ui_hub.runtime as runtime_module
    import arm_cell_simulation_ui_hub.sidecar as sidecar_module

    feed_is_stale = {"value": False}
    tf_buffer_clear_calls = []

    class Runtime:
        def __init__(self, **_kwargs):
            self.request_state = {}
            self.fault_active = {"communication_loss": False}
            self.selected_display_source = "rgb"

        def spin_once(self):
            pass

        def clear_tf_buffer(self):
            tf_buffer_clear_calls.append(True)

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

        proxy.clear_tf_buffer()
        proxy._last_poll_at = 0.0
        proxy.spin_once()
        assert tf_buffer_clear_calls == [True]

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


def test_sidecar_keeps_terminal_detector_snapshot_until_a_new_snapshot_arrives():
    import base64

    from arm_cell_simulation_ui_hub.sidecar import IsaacRosSidecar

    proxy = IsaacRosSidecar.__new__(IsaacRosSidecar)
    proxy._closed = False
    proxy._rpc_broken = False
    proxy._fault_rpc_in_flight = False
    proxy._last_poll_at = 0.0
    proxy._observations = {}
    proxy.request_state = {}
    proxy.fault_active = {}
    proxy._target_pixel = ("unavailable", None)
    proxy._vision_state = "unavailable"
    proxy._vision_overlay = None
    proxy._vision_detector_snapshot = None
    proxy._video_mode = "UNAVAILABLE"
    proxy._sidecar_available = None
    proxy._sidecar_camera_frame_available = None
    proxy._camera_frame = None
    proxy._camera_identity = None
    proxy._display_source = "rgb"
    proxy._display_status = "RGB · LIVE"
    payload = {
        "snapshot_id": "run:9",
        "event_kind": 3,
        "metadata_json": '{"terminal_result":{"result_code":9}}',
        "image_data_b64": base64.b64encode(b"vision-png").decode("ascii"),
    }
    response = {
        "vision_detector_snapshot_changed": True,
        "vision_detector_snapshot": payload,
        "camera_frame_current": False,
    }
    proxy._rpc = lambda _request: response

    proxy.spin_once()
    terminal = proxy.vision_detector_snapshot()
    assert terminal["snapshot_id"] == "run:9"
    assert terminal["image_data"] == b"vision-png"
    assert terminal["metadata_json"] == payload["metadata_json"]

    proxy._last_poll_at = 0.0
    response = {"camera_frame_current": False}
    proxy.spin_once()
    assert proxy.vision_detector_snapshot() == terminal


def test_vision_detector_view_decodes_owner_png_and_handles_missing_image_or_metadata():
    import cv2
    from arm_cell_simulation_ui_hub.extension import SimulationUIHubExtension

    class Provider:
        def __init__(self):
            self.data = None
            self.dimensions = None

        def set_bytes_data(self, data, dimensions):
            self.data = bytes(data)
            self.dimensions = dimensions

    class Label:
        text = ""

    pixels = np.full((2, 3, 3), 120, dtype=np.uint8)
    encoded_ok, encoded = cv2.imencode(".png", pixels)
    assert encoded_ok
    extension = SimulationUIHubExtension.__new__(SimulationUIHubExtension)
    extension.runtime = types.SimpleNamespace(
        vision_detector_snapshot=lambda: extension.snapshot
    )
    extension.snapshot = {
        "snapshot_id": "run:1",
        "revision": 1,
        "event_kind": 2,
        "image_data": encoded.tobytes(),
        "metadata_json": (
            '{"event_kind":"DETECT_TARGET_SUCCESS","image_available":true,'
            '"source_image_rendered":true,"processing_stage":"target_selection",'
            '"terminal_result":{"diagnostic_detail":"selected"},'
            '"observation":{"result":{"rawpart":{"candidate_count":2}}},'
            '"stability":{"qualifying_observations":2}}'
        ),
    }
    extension._detector_snapshot_id = None
    extension._detector_image_decode_available = None
    extension._detector_metadata_available = None
    extension._detector_image_provider = Provider()
    extension._detector_image_widget = types.SimpleNamespace(width=0, height=0)
    extension._detector_event_label = Label()
    extension._detector_diagnostics_label = Label()
    extension._ui = types.SimpleNamespace(Pixel=lambda value: value)

    extension._refresh_detector_snapshot()
    assert extension._detector_image_provider.dimensions == [3, 2]
    assert "DETECT_TARGET_SUCCESS" in extension._detector_event_label.text
    assert "source image available" in extension._detector_event_label.text
    assert "terminal_result" in extension._detector_diagnostics_label.text
    assert "rawpart" in extension._detector_diagnostics_label.text
    assert "stability" in extension._detector_diagnostics_label.text

    extension.snapshot = {
        "snapshot_id": "run:2",
        "revision": 2,
        "event_kind": 3,
        "image_data": b"",
        "metadata_json": "{invalid",
    }
    extension._refresh_detector_snapshot()
    assert extension._detector_image_provider.dimensions == [1, 1]
    assert "source image unavailable" in extension._detector_event_label.text
    assert "PNG unavailable" in extension._detector_event_label.text
    assert "invalid JSON" in extension._detector_diagnostics_label.text


def test_sidecar_publishes_handoff_trigger_through_ros_process():
    from arm_cell_simulation_ui_hub.sidecar import IsaacRosSidecar

    proxy = IsaacRosSidecar.__new__(IsaacRosSidecar)
    proxy._fault_rpc_in_flight = False
    proxy._rpc_broken = False
    calls = []
    proxy._rpc = lambda message: calls.append(message) or {
        "delivery_id": "delivery-123"
    }

    delivery_id = proxy.publish_handoff_trigger("RawPart", 2, random_seed=7)

    assert delivery_id == "delivery-123"
    assert calls == [
        {
            "op": "publish_handoff_trigger",
            "material_type": "RawPart",
            "quantity": 2,
            "random_seed": 7,
        }
    ]


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


def test_sidecar_tf_buffer_clear_is_forwarded_with_next_poll():
    from arm_cell_simulation_ui_hub.sidecar import IsaacRosSidecar

    proxy = IsaacRosSidecar.__new__(IsaacRosSidecar)
    proxy._closed = False
    proxy._rpc_broken = False
    proxy._fault_rpc_in_flight = False
    proxy._last_poll_at = 0.0
    proxy._camera_frame = None
    proxy._display_source = "rgb"
    proxy._sidecar_available = None
    proxy._observations = {}
    proxy.request_state = {}
    proxy.request_detail = {}
    proxy.fault_active = {}
    requests = []
    proxy._rpc = lambda request: requests.append(request) or {}

    proxy.clear_tf_buffer()
    proxy.spin_once()

    assert requests[0]["op"] == "poll"
    assert requests[0]["clear_tf_buffer"] is True
    assert proxy._tf_buffer_clear_pending is False


def test_timeline_stop_clears_isaac_faults_without_ros_mock_requests():
    from types import SimpleNamespace

    from arm_cell_simulation_ui_hub.extension import (
        SimulationUIHubExtension,
    )

    class _Runtime:
        def __init__(self):
            self.cleared = []
            self.safety_reset_calls = 0
            self.tf_buffer_clear_calls = 0

        def clear_tf_buffer(self):
            self.tf_buffer_clear_calls += 1

        def set_external_fault(self, scenario, active):
            self.cleared.append(("external_mock", scenario, active))
            return f"request-{scenario}", None

        def set_scenario_active(self, owner, scenario, active):
            self.cleared.append((owner, scenario, active))
            return True

    extension = SimulationUIHubExtension.__new__(SimulationUIHubExtension)
    extension.runtime = _Runtime()
    extension._fault_request_ids = {}
    extension._fault_local_status = {}
    extension._fault_status_labels = {}
    extension._scenario_active = {}
    extension._on_timeline_event(SimpleNamespace(type="STOP"))

    expected = {
        ("external_mock", "e_stop", False),
        ("external_mock", "safety_signal_loss", False),
    }
    assert set(extension.runtime.cleared) == expected
    assert extension.runtime.cleared == [
        ("external_mock", "e_stop", False),
        ("external_mock", "safety_signal_loss", False),
    ]
    assert set(extension._fault_request_ids) == {
        ("external_mock", "e_stop"),
        ("external_mock", "safety_signal_loss"),
    }
    assert extension.runtime.safety_reset_calls == 0
    assert extension.runtime.tf_buffer_clear_calls == 1


def test_hub_runtime_clears_its_tf_buffer():
    from arm_cell_simulation_ui_hub.runtime import HubRuntime

    class _Buffer:
        def __init__(self):
            self.clear_calls = 0

        def clear(self):
            self.clear_calls += 1

    runtime = HubRuntime.__new__(HubRuntime)
    runtime._tf_buffer = _Buffer()

    runtime.clear_tf_buffer()

    assert runtime._tf_buffer.clear_calls == 1


def test_retry_detection_rpc_reaches_existing_runtime_request_method(monkeypatch):
    import arm_cell_simulation_ui_hub.runtime as runtime_module
    import arm_cell_simulation_ui_hub.sidecar as sidecar_module

    runtimes = []

    class Runtime:
        def __init__(self, **_kwargs):
            runtimes.append(self)
            self.request_state = {}
            self.request_detail = {}
            self.fault_active = {}
            self.snapshot = None
            self.result = {
                "request_id": "request-1",
                "target_id": None,
                "state": "pending",
                "result_code": None,
                "acquisition_id": "",
                "diagnostic_detail": "",
                "snapshot": None,
            }
            self.calibration_result = {
                "request_id": "calibration-request-1",
                "state": "pending",
                "message": "",
            }

        def spin_once(self):
            pass

        def request_retry_detection(self, target_id):
            self.result["target_id"] = target_id

        def retry_detection_result(self):
            return dict(self.result)

        def retry_detection_eligibility(self):
            return True, "Available"

        def request_vision_calibration(self):
            return "calibration-request-1", None

        def vision_calibration_result(self):
            return dict(self.calibration_result)

        def observation_text(self, name, *, now=None):
            return f"{name}: current"

        def vision_detector_snapshot(self):
            return self.snapshot

        def selected_target_pixel(self, *, now=None):
            return "unavailable", None

        def vision_display(self, *, now=None):
            return "IDLE", None

        def video_mode(self):
            return "UNAVAILABLE"

        def display_source(self):
            return "rgb"

        def display_status(self, *, now=None):
            return "RGB · UNAVAILABLE"

        def camera_frame_rgba(self):
            return None

        def camera_identity(self):
            return None

        def close(self):
            pass

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
    proxy._retry_detection_result = {
        "request_id": None,
        "target_id": None,
        "state": "unavailable",
        "result_code": None,
        "acquisition_id": "",
        "diagnostic_detail": "",
        "snapshot": None,
    }
    proxy._vision_calibration_result = {
        "request_id": None,
        "state": "ready",
        "message": "",
    }
    proxy._reader = reader
    proxy._writer = writer
    proxy._socket = client_socket
    proxy._observations = {}
    proxy.request_state = {}
    proxy.request_detail = {}
    proxy.fault_active = {}
    proxy._target_pixel = ("unavailable", None)
    proxy._camera_frame = None
    proxy._camera_identity = None
    proxy._video_mode = "UNAVAILABLE"
    proxy._display_source = "rgb"
    proxy._sidecar_available = None
    proxy._sidecar_camera_frame_available = None
    proxy._last_poll_at = 0.0
    try:
        assert json.loads(reader.readline()) == {"ready": True}
        request_id, _future = proxy.request_retry_detection("RawPart")
        assert request_id == "request-1"
        assert proxy.retry_detection_result()["target_id"] == "RawPart"
        calibration_id, _future = proxy.request_vision_calibration()
        assert calibration_id == "calibration-request-1"
        assert proxy.vision_calibration_result()["state"] == "pending"
        runtimes[0].calibration_result.update(
            {"state": "succeeded", "message": "/calibration/new.yaml"}
        )
        runtimes[0].result.update(
            {
                "state": "succeeded",
                "result_code": 0,
                "acquisition_id": "acq-1",
                "diagnostic_detail": "detected",
                "snapshot": {
                    "snapshot_id": "snapshot-1",
                    "acquisition_id": "acq-1",
                    "event_kind": 2,
                    "image_data": b"png-bytes",
                },
            }
        )
        proxy.spin_once()
        assert proxy.vision_calibration_result() == {
            "request_id": calibration_id,
            "state": "succeeded",
            "message": "/calibration/new.yaml",
        }
        retry = proxy.retry_detection_result()
        assert retry["request_id"] == request_id
        assert retry["acquisition_id"] == "acq-1"
        # The detector Snapshot topic owns UI delivery. The recurring status
        # poll must not encode and resend the same image bytes.
        assert retry["snapshot"] is None
        writer.write(b'{"op":"shutdown"}\n')
        writer.flush()
        assert json.loads(reader.readline()) == {"ok": True}
    finally:
        reader.close()
        writer.close()
        client_socket.close()
        server.join(timeout=1.0)


def test_extension_retry_detection_requires_dev_setting_running_timeline_and_eligibility(
    monkeypatch,
):
    from arm_cell_simulation_ui_hub.extension import SimulationUIHubExtension

    timeline_interface = types.SimpleNamespace(is_playing=lambda: True)
    omni = types.ModuleType("omni")
    omni.__path__ = []
    timeline = types.ModuleType("omni.timeline")
    timeline.get_timeline_interface = lambda: timeline_interface
    omni.timeline = timeline
    monkeypatch.setitem(sys.modules, "omni", omni)
    monkeypatch.setitem(sys.modules, "omni.timeline", timeline)

    class Runtime:
        def __init__(self):
            self.eligibility = (False, "Mission is active")
            self.requests = []

        def retry_detection_eligibility(self):
            return self.eligibility

        def request_retry_detection(self, target_id):
            self.requests.append(target_id)

        def retry_detection_result(self):
            return {"state": "unavailable"}

    extension = SimulationUIHubExtension.__new__(SimulationUIHubExtension)
    extension.runtime = Runtime()
    extension._retry_detection_enabled = False
    extension._retry_detection_target_id = "RawPart"
    extension._retry_detection_button = types.SimpleNamespace(enabled=False)
    extension._retry_detection_status = types.SimpleNamespace(text="")

    extension._request_retry_detection()
    assert extension.runtime.requests == []

    extension._retry_detection_enabled = True
    extension._refresh_retry_detection()
    assert not extension._retry_detection_button.enabled
    assert extension._retry_detection_status.text == "Mission is active"
    extension._request_retry_detection()
    assert extension.runtime.requests == []

    extension.runtime.eligibility = (True, "Available")
    extension._refresh_retry_detection()
    assert extension._retry_detection_button.enabled
    extension._request_retry_detection()
    assert extension.runtime.requests == ["RawPart"]


def test_extension_vision_calibration_button_disables_while_pending_and_shows_result():
    from arm_cell_simulation_ui_hub.extension import SimulationUIHubExtension

    class Runtime:
        result = {"request_id": "request-1", "state": "pending", "message": ""}

        def vision_calibration_result(self):
            return dict(self.result)

        def request_vision_calibration(self):
            return "request-1", None

    extension = SimulationUIHubExtension.__new__(SimulationUIHubExtension)
    extension.runtime = Runtime()
    extension._vision_calibration_button = types.SimpleNamespace(enabled=True)
    extension._vision_calibration_status = types.SimpleNamespace(text="")

    extension._refresh_vision_calibration()
    assert extension._vision_calibration_button.enabled is False
    assert extension._vision_calibration_status.text == (
        "Vision calibration request pending."
    )

    extension.runtime.result.update(
        {"state": "succeeded", "message": "/calibration/new.yaml"}
    )
    extension._refresh_vision_calibration()
    assert extension._vision_calibration_button.enabled is True
    assert "/calibration/new.yaml" in extension._vision_calibration_status.text


def test_sidecar_loss_terminates_pending_retry_detection_projection():
    from arm_cell_simulation_ui_hub.sidecar import IsaacRosSidecar

    sidecar = IsaacRosSidecar.__new__(IsaacRosSidecar)
    sidecar._observations = {}
    sidecar._camera_frame = None
    sidecar._target_pixel = ("current", (1.0, 2.0))
    sidecar._vision_state = "IDLE"
    sidecar._vision_overlay = None
    sidecar._video_mode = "LIVE"
    sidecar._display_source = "rgb"
    sidecar._display_status = "RGB · LIVE"
    sidecar._sidecar_available = None
    sidecar._retry_detection_result = {
        "request_id": "request-1",
        "state": "pending",
        "diagnostic_detail": "",
        "snapshot": None,
    }

    sidecar._mark_sidecar_unavailable()

    assert sidecar.retry_detection_result()["state"] == "failed"
    assert sidecar.retry_detection_result()["snapshot"] is None


def test_demonstration_control_calls_local_adapter_without_handoff(monkeypatch):
    from arm_cell_simulation_ui_hub.extension import SimulationUIHubExtension
    from scene_builder import cuboid_demonstration

    timeline_module = types.ModuleType("omni.timeline")
    timeline_module.get_timeline_interface = lambda: types.SimpleNamespace(
        is_playing=lambda: True
    )
    omni_module = types.ModuleType("omni")
    omni_module.__path__ = []
    omni_module.timeline = timeline_module
    monkeypatch.setitem(sys.modules, "omni", omni_module)
    monkeypatch.setitem(sys.modules, "omni.timeline", timeline_module)

    loaded = []
    monkeypatch.setattr(
        cuboid_demonstration,
        "load_demonstration",
        lambda scenario: loaded.append(scenario) or 3,
    )

    class _Runtime:
        def publish_handoff_trigger(self, *_args, **_kwargs):
            raise AssertionError("Demonstration must not publish a handoff trigger")

    extension = SimulationUIHubExtension.__new__(SimulationUIHubExtension)
    extension.runtime = _Runtime()
    extension._demonstration_scenarios = (
        ("side_by_side", "Side by Side", "two contact parts"),
    )
    extension._demonstration_combo = types.SimpleNamespace(
        enabled=True,
        model=types.SimpleNamespace(
            get_item_value_model=lambda: types.SimpleNamespace(
                get_value_as_int=lambda: 0
            )
        ),
    )
    extension._demonstration_status = types.SimpleNamespace(text="")
    extension._demonstration_loaded_this_session = False
    extension._demonstration_button = types.SimpleNamespace(enabled=True)

    extension._load_demonstration()

    assert loaded == ["side_by_side"]
    assert "Loaded Side by Side: 3 RawParts" in extension._demonstration_status.text
    assert extension._demonstration_button.enabled is False
    assert extension._demonstration_combo.enabled is False
