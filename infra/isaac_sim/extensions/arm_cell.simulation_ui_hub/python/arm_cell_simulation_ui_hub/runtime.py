"""ROS 2 owner connections used by the Isaac Simulation UI Hub."""

from __future__ import annotations

import os
import sys
import time
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Any

from .display_sources import (
    AlignedDepthHeatmapDisplaySource,
    DisplayFrame,
    RGBDisplaySource,
    TrayAlignedDepthDisplaySource,
    image_identity,
)
from .projection import HubProjection
from .scenario_ports import Owner, OwnerScenarioPorts


DEFAULT_OBSERVATION_FRESHNESS_SECONDS = 1.0
DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS = 0.15
MAX_ANNOTATION_SOURCE_TOLERANCE_SECONDS = 0.2
ANNOTATION_SOURCE_FRAME_HISTORY_SECONDS = 0.4
ANNOTATION_SOURCE_FRAME_HISTORY_SIZE = 24
CAMERA_SUBSCRIPTION_QUEUE_DEPTH = 1
RETRY_DETECTION_TIMEOUT_SECONDS = 30.0
RETRY_DETECTION_ADMISSION_RESPONSE_TIMEOUT_SECONDS = 30.0
RETRY_DETECTION_RESPONSE_MARGIN_SECONDS = 0.1
VDA_MOCK_FAULT_SCENARIOS = (
    "material_handoff_failure",
    "e_stop",
    "premature_undock",
    "packml_abort",
    "invalid_state",
    "communication_loss",
    "communication_degradation",
    "safety_signal_loss",
)


def select_compatible_operator_rgb_frame(
    frames: Any,
    vision_source_identity: tuple[float, str],
    *,
    max_delta_seconds: float,
) -> Any | None:
    """Select the nearest same-camera operator frame within the temporal window."""
    source_stamp, source_frame = vision_source_identity
    compatible = [
        entry
        for entry in frames
        if entry[0][1] == source_frame
        and abs(entry[0][0] - source_stamp) <= max_delta_seconds
    ]
    if not compatible:
        return None
    return min(compatible, key=lambda entry: abs(entry[0][0] - source_stamp))


@dataclass(frozen=True)
class DetectionAnnotation:
    display_frame_identity: tuple[float, str]
    target_id: str
    support_region: tuple[tuple[float, float], ...]
    received_at: float
    display_frame_size: tuple[int, int]

    def metadata(self) -> dict[str, Any]:
        return {
            "target_id": self.target_id,
            "support_region": list(self.support_region),
        }


@dataclass(frozen=True)
class DetectionSnapshot:
    rgb_frame: DisplayFrame
    depth_frame: DisplayFrame | None
    annotation: DetectionAnnotation
    execute_cycle_goal_id: str

    @property
    def frame_rgba(self) -> Any:
        return self.rgb_frame.rgba

    @property
    def display_frame_identity(self) -> tuple[float, str]:
        return self.annotation.display_frame_identity

    def metadata(self) -> dict[str, Any]:
        return self.annotation.metadata()


class HubRuntime:
    """Thin ROS client/subscriber boundary; never publishes canonical state."""

    def __new__(
        cls,
        *,
        node: Any | None = None,
        scenario_ports: OwnerScenarioPorts | None = None,
        default_observation_freshness_seconds: float = (
            DEFAULT_OBSERVATION_FRESHNESS_SECONDS
        ),
        default_annotation_source_tolerance_seconds: float = (
            DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS
        ),
    ):
        if (
            cls is HubRuntime
            and node is None
            and sys.version_info >= (3, 11)
            and os.environ.get("ROS_DISTRO") == "humble"
        ):
            from .sidecar import IsaacRosSidecar

            return IsaacRosSidecar(
                default_observation_freshness_seconds=(
                    default_observation_freshness_seconds
                ),
                default_annotation_source_tolerance_seconds=(
                    default_annotation_source_tolerance_seconds
                ),
                scenario_ports=scenario_ports,
            )
        return super().__new__(cls)

    def __init__(
        self,
        *,
        node: Any | None = None,
        scenario_ports: OwnerScenarioPorts | None = None,
        default_observation_freshness_seconds: float = (
            DEFAULT_OBSERVATION_FRESHNESS_SECONDS
        ),
        default_annotation_source_tolerance_seconds: float = (
            DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS
        ),
    ):
        if node is None:
            import rclpy
            from rclpy.node import Node
            from rclpy.parameter import Parameter

            if not rclpy.ok():
                rclpy.init(args=None)
            node = Node(
                "simulation_ui_hub",
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
        self.node = node
        self.projection = HubProjection()
        self.feed_max_age_seconds = None
        self._annotation_source_tolerance_seconds = (
            DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS
        )
        try:
            configured = float(
                node.declare_parameter(
                    "observation_freshness_seconds",
                    float(default_observation_freshness_seconds),
                ).value
            )
            if configured > 0:
                self.feed_max_age_seconds = configured
        except (AttributeError, TypeError, ValueError):
            self.feed_max_age_seconds = None
        try:
            tolerance = float(
                node.declare_parameter(
                    "annotation_source_tolerance_seconds",
                    float(default_annotation_source_tolerance_seconds),
                ).value
            )
            if 0 < tolerance <= MAX_ANNOTATION_SOURCE_TOLERANCE_SECONDS:
                self._annotation_source_tolerance_seconds = tolerance
        except (AttributeError, TypeError, ValueError):
            pass
        self.scenario_ports = (
            scenario_ports or OwnerScenarioPorts.from_process_registry()
        )
        self.request_state: dict[str, str] = {}
        self.request_detail: dict[str, str] = {}
        self._retry_detection_inflight = False
        self._retry_detection_deadline: float | None = None
        self._retry_detection_dispatched_at: float | None = None
        self._retry_detection_future: Any | None = None
        self._retry_detection_result: dict[str, Any] = {
            "request_id": None,
            "target_id": None,
            "state": "unavailable",
            "result_code": None,
            "acquisition_id": "",
            "diagnostic_detail": "Retry Detection has not been requested",
            "snapshot": None,
        }
        self.fault_active = {scenario: False for scenario in VDA_MOCK_FAULT_SCENARIOS}
        self._subscriptions: list[Any] = []
        self._clients: dict[str, Any] = {}
        self._tf_buffer: Any | None = None
        self._tf_listener: Any | None = None
        self._camera_identity: tuple[float, str] | None = None
        self._camera_received_at: float | None = None
        self._camera_size: tuple[int, int] | None = None
        self._source_frames: deque[tuple[tuple[float, str], Any, float]] = deque(
            maxlen=ANNOTATION_SOURCE_FRAME_HISTORY_SIZE
        )
        self._display_sources = {
            "rgb": RGBDisplaySource(),
            "depth": AlignedDepthHeatmapDisplaySource(),
            "tray": TrayAlignedDepthDisplaySource(),
        }
        self._selected_display_source = "none"
        self._display_image_type: Any | None = None
        self._display_camera_qos: Any | None = None
        self._display_subscription: Any | None = None
        self._latest_rgb_message: Any | None = None
        self._latest_depth_message: Any | None = None
        self._latest_tray_depth_message: Any | None = None
        self._depth_received_at: float | None = None
        self._tray_depth_received_at: float | None = None
        self._display_frames: dict[str, DisplayFrame] = {}
        self._display_errors: dict[str, str] = {}
        self._last_success_event: tuple[float, str, str] | None = None
        self._pending_detection_diagnostic: Any | None = None
        self._pending_detection_received_at: float | None = None
        self._unconsumed_readiness_ids: set[str] = set()
        self._admitted_readiness_ids: set[str] = set()
        self._admitted_readiness_order: list[str] = []
        self._active_execute_goal_id: str | None = None
        self._execute_cycle_status_received_at: float | None = None
        self._execute_cycle_status_by_goal: dict[str, int] = {}
        self._execute_cycle_result_by_goal: dict[str, Any] = {}
        self._execute_cycle_result_requests: set[str] = set()
        self._execute_cycle_pending_results: dict[str, tuple[Any, float]] = {}
        self._execute_cycle_recent_goal_ids: list[str] = []
        self._execute_cycle_latest_terminal_goal_id: str | None = None
        self._operator_annotation: DetectionAnnotation | None = None
        self._detection_snapshot: DetectionSnapshot | None = None
        self._vision_detector_snapshot: dict[str, Any] | None = None
        self._raw_frame_available: bool | None = None
        self._executor: Any | None = None
        self._closed = False
        self._connect_ros()

    def spin_once(self) -> None:
        """Pump ROS callbacks from the Isaac Kit update loop."""
        if self._closed:
            return
        self._expire_retry_detection_request(time.monotonic())
        if self._executor is None:
            from rclpy.executors import SingleThreadedExecutor

            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self.node)
        self._executor.spin_once(timeout_sec=0.0)
        self._poll_execute_cycle_results()
        self._prune_source_frames(time.monotonic())

    def _connect_ros(self) -> None:
        from arm_cell_interfaces.msg import (
            AMRDockingState,
            HandoffTrigger,
            HandoffTriggerStatus,
            MaterialHandoffState,
            MaterialReadiness,
            MotionStatus,
            PackMLState,
            SafetyState,
            SceneStability,
            TransferCompletion,
            VisionDiagnostic,
            VisionCalibration,
            VisionDetectorSnapshot,
        )
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )

        self._subscribe(SafetyState, "/safety/state", "safety", 10)
        self._subscribe(MotionStatus, "/motion/state", "motion", 10)
        self._subscribe(AMRDockingState, "/amr/docking_report", "amr", 10)
        self._subscribe(
            MaterialHandoffState, "/vda/material_handoff_state", "handoff", 10
        )
        self._subscribe(
            HandoffTrigger, "/integration/accepted_handoff", "accepted_handoff", 10
        )
        self._subscribe(
            HandoffTriggerStatus,
            "/integration/handoff_trigger_status",
            "trigger_status",
            10,
        )
        self._subscribe(
            TransferCompletion,
            "/material_supply/transfer_completion",
            "transfer_completion",
            10,
        )
        self._subscribe(
            SceneStability, "/vision/scene_stability", "scene_stability", 10
        )
        calibration_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscribe(
            VisionCalibration,
            "/vision/calibration_status",
            "vision_calibration",
            calibration_qos,
        )
        readiness_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscriptions.append(
            self.node.create_subscription(
                MaterialReadiness,
                "/integration/material_readiness",
                self._receive_material_readiness,
                readiness_qos,
            )
        )
        self._subscribe(PackMLState, "/packml/state", "packml", 10)
        camera_qos = QoSProfile(
            depth=CAMERA_SUBSCRIPTION_QUEUE_DEPTH,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._display_image_type = Image
        self._display_camera_qos = camera_qos
        diagnostic_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscriptions.append(
            self.node.create_subscription(
                VisionDiagnostic,
                "/vision/diagnostics",
                self._receive_vision_diagnostic,
                diagnostic_qos,
            )
        )
        detector_snapshot_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscriptions.append(
            self.node.create_subscription(
                VisionDetectorSnapshot,
                "/vision/detector_snapshot",
                self._receive_vision_detector_snapshot,
                detector_snapshot_qos,
            )
        )
        self._subscribe(
            CameraInfo,
            "/camera/color/camera_info",
            "camera_info",
            qos_profile_sensor_data,
        )
        from arm_cell_interfaces.action._execute_cycle import (
            ExecuteCycle_FeedbackMessage,
        )
        from arm_cell_interfaces.action import ExecuteCycle
        from action_msgs.msg import GoalStatusArray

        self._subscriptions.append(
            self.node.create_subscription(
                ExecuteCycle_FeedbackMessage,
                "/orchestration/execute_cycle/_action/feedback",
                self._receive_execute_cycle_feedback,
                10,
            )
        )
        self._clients["execute_cycle_result"] = self.node.create_client(
            ExecuteCycle.Impl.GetResultService,
            "/orchestration/execute_cycle/_action/get_result",
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscriptions.append(
            self.node.create_subscription(
                GoalStatusArray,
                "/orchestration/execute_cycle/_action/status",
                self._receive_execute_cycle_status,
                status_qos,
            )
        )

        from arm_cell_interfaces.srv import ResetSafety

        self._trigger_publisher = self.node.create_publisher(
            HandoffTrigger,
            "/external/handoff_trigger",
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self._clients["safety_reset"] = self.node.create_client(
            ResetSafety, "/safety/reset"
        )
        from arm_cell_interfaces.srv import DetectTarget
        from std_srvs.srv import Trigger

        self._clients["retry_detection"] = self.node.create_client(
            DetectTarget, "/vision/detect_target"
        )
        self._clients["vision_calibration"] = self.node.create_client(
            Trigger, "/orchestration/calibrate_vision"
        )
        self._vision_calibration_result: dict[str, Any] = {
            "request_id": None,
            "state": "ready",
            "message": "",
        }

        # TF is consumed read-only to project the canonical selected target.
        try:
            from tf2_ros import Buffer, TransformListener

            self._tf_buffer = Buffer(node=self.node)
            self._tf_listener = TransformListener(self._tf_buffer, self.node)
        except (ImportError, AttributeError):
            self._tf_buffer = None

    def clear_tf_buffer(self) -> None:
        """Discard transforms from the simulation epoch that just ended."""
        if self._tf_buffer is not None:
            self._tf_buffer.clear()

    def _live_camera_frame_rgba(self, now: float) -> tuple[Any, int, int] | None:
        frame = self.display_frame(now=now)
        if frame is None:
            return None
        return frame.rgba, frame.width, frame.height

    def set_display_source(self, source_id: str) -> None:
        if source_id != "none" and source_id not in self._display_sources:
            raise ValueError(f"unsupported Hub display source: {source_id}")
        if source_id == self._selected_display_source:
            return
        self._destroy_display_subscription()
        self._clear_display_source_data(self._selected_display_source)
        self._selected_display_source = source_id
        if source_id == "none":
            return
        topics = {
            "rgb": ("/camera/color/image_raw", self._receive_raw_rgb),
            "depth": (
                "/camera/aligned_depth_to_color/image_raw",
                self._receive_aligned_depth,
            ),
            "tray": (
                "/vision/tray_aligned_depth/image_raw",
                self._receive_tray_aligned_depth,
            ),
        }
        topic, callback = topics[source_id]
        self._display_subscription = self.node.create_subscription(
            self._display_image_type, topic, callback, self._display_camera_qos
        )

    def _destroy_display_subscription(self) -> None:
        if self._display_subscription is None:
            return
        self.node.destroy_subscription(self._display_subscription)
        self._display_subscription = None

    def _clear_display_source_data(self, source_id: str) -> None:
        if source_id == "rgb":
            self._latest_rgb_message = None
            self._camera_identity = None
            self._camera_received_at = None
            self._camera_size = None
            self._source_frames.clear()
            self._operator_annotation = None
        elif source_id == "depth":
            self._latest_depth_message = None
            self._depth_received_at = None
        elif source_id == "tray":
            self._latest_tray_depth_message = None
            self._tray_depth_received_at = None
        self._display_frames.pop(source_id, None)
        self._display_errors.pop(source_id, None)

    def display_source(self) -> str:
        return self._selected_display_source

    def display_frame(self, *, now: float | None = None) -> DisplayFrame | None:
        """Return the selected live frame or its immutable PnP capture."""
        now = time.monotonic() if now is None else now
        if self._selected_display_source == "none":
            return None
        if (
            self._detection_snapshot is not None
            and self._selected_display_source != "tray"
        ):
            return (
                self._detection_snapshot.rgb_frame
                if self._selected_display_source == "rgb"
                else self._detection_snapshot.depth_frame
            )
        if self.feed_max_age_seconds is None:
            return None
        if self._selected_display_source == "rgb":
            message = self._latest_rgb_message
            received_at = self._camera_received_at
        elif self._selected_display_source == "depth":
            message = self._latest_depth_message
            received_at = self._depth_received_at
        else:
            message = self._latest_tray_depth_message
            received_at = self._tray_depth_received_at
        if (
            message is None
            or received_at is None
            or now - received_at > self.feed_max_age_seconds
        ):
            return None
        identity = image_identity(message)
        cached = self._display_frames.get(self._selected_display_source)
        if cached is not None and cached.identity == identity:
            return cached
        try:
            frame = self._display_sources[self._selected_display_source].convert(
                message, received_at=received_at
            )
        except (TypeError, ValueError, BufferError) as error:
            self._display_errors[self._selected_display_source] = str(error)
            self._display_frames.pop(self._selected_display_source, None)
            return None
        self._display_errors.pop(self._selected_display_source, None)
        self._display_frames[self._selected_display_source] = frame
        return frame

    def display_status(self, *, now: float | None = None) -> str:
        if self._selected_display_source == "none":
            return "DIAGNOSTIC VIDEO OFF"
        frame = self.display_frame(now=now)
        if frame is not None:
            if (
                self._detection_snapshot is not None
                and self._selected_display_source != "tray"
            ):
                return f"{self._selected_display_source.upper()} · PnP SNAPSHOT"
            return f"{self._selected_display_source.upper()} · LIVE"
        error = self._display_errors.get(self._selected_display_source)
        if error:
            return f"{self._selected_display_source.upper()} UNAVAILABLE · {error}"
        received_at = (
            self._camera_received_at
            if self._selected_display_source == "rgb"
            else (
                self._depth_received_at
                if self._selected_display_source == "depth"
                else self._tray_depth_received_at
            )
        )
        if received_at is not None and self.feed_max_age_seconds is not None:
            return f"{self._selected_display_source.upper()} STALE"
        return f"{self._selected_display_source.upper()} UNAVAILABLE"

    def camera_frame_rgba(
        self, *, now: float | None = None
    ) -> tuple[Any, int, int] | None:
        """Compatibility accessor for the selected display frame pixels."""
        return self._live_camera_frame_rgba(time.monotonic() if now is None else now)

    def camera_identity(self, *, now: float | None = None) -> tuple[float, str] | None:
        """Compatibility alias for the selected display-frame identity."""
        return self.display_frame_identity(now=now)

    def display_frame_identity(
        self, *, now: float | None = None
    ) -> tuple[float, str] | None:
        """Return the capture identity of the currently selected display frame."""
        frame = self.display_frame(now=now)
        return frame.identity if frame is not None else None

    def video_mode(self) -> str:
        if self._detection_snapshot is not None:
            if self.display_frame() is None:
                return (
                    f"{self._selected_display_source.upper()} UNAVAILABLE · PnP ACTIVE"
                )
            return "DETECTION SNAPSHOT · PnP ACTIVE"
        if (
            self._pending_detection_diagnostic is not None
            and self._active_execute_goal_id is not None
        ):
            return "ANNOTATION PENDING · PnP ACTIVE"
        if self._live_camera_frame_rgba(time.monotonic()) is None:
            return "UNAVAILABLE"
        return "LIVE"

    def _receive_raw_rgb(self, message: Any) -> None:
        identity = image_identity(message)
        received_at = time.monotonic()
        self._camera_received_at = received_at
        self._camera_identity = identity
        try:
            RGBDisplaySource.validate(message)
        except (TypeError, ValueError) as error:
            if self._raw_frame_available is not False:
                self._log_video(
                    "warning", f"Raw RGB operator frame unavailable: {error}"
                )
                self._log_video("debug", f"raw RGB frame detail: {error}")
            self._raw_frame_available = False
            self._latest_rgb_message = None
            self._camera_identity = None
            self._camera_size = None
            self._display_errors["rgb"] = str(error)
            self._display_frames.pop("rgb", None)
            self._try_create_operator_annotation()
            return
        if self._raw_frame_available is False:
            self._log_video("info", "raw RGB operator frame recovered")
        self._raw_frame_available = True
        self._display_errors.pop("rgb", None)
        self._latest_rgb_message = message
        self._camera_size = (int(message.width), int(message.height))
        self._display_frames.pop("rgb", None)
        self._source_frames.append((identity, message, received_at))
        self._prune_source_frames(received_at)
        self._try_create_operator_annotation()

    def _receive_aligned_depth(self, message: Any) -> None:
        self._latest_depth_message = message
        self._depth_received_at = time.monotonic()
        self._display_frames.pop("depth", None)
        self._display_errors.pop("depth", None)

    def _receive_tray_aligned_depth(self, message: Any) -> None:
        self._latest_tray_depth_message = message
        self._tray_depth_received_at = time.monotonic()
        self._display_frames.pop("tray", None)
        self._display_errors.pop("tray", None)

    @staticmethod
    def _raw_rgb_to_rgba(message: Any) -> Any:
        return RGBDisplaySource().convert(message, received_at=time.monotonic()).rgba

    def _prune_source_frames(self, now: float) -> None:
        while (
            self._source_frames
            and now - self._source_frames[0][2]
            > ANNOTATION_SOURCE_FRAME_HISTORY_SECONDS
        ):
            self._source_frames.popleft()

    def _clear_pending_detection(self) -> None:
        self._pending_detection_diagnostic = None
        self._pending_detection_received_at = None

    def _current_operator_annotation(self, now: float) -> DetectionAnnotation | None:
        annotation = self._operator_annotation
        if (
            annotation is None
            or self.feed_max_age_seconds is None
            or now - annotation.received_at > self.feed_max_age_seconds
        ):
            return None
        return annotation

    def _annotation_binding_context(self, diagnostic: Any) -> tuple[str, str, str, str]:
        source = diagnostic.source_rgb_header
        source_stamp = float(source.stamp.sec) + float(source.stamp.nanosec) / 1e9
        source_frame = str(source.frame_id)
        nearest = min(
            self._source_frames,
            key=lambda entry: abs(entry[0][0] - source_stamp),
            default=None,
        )
        if nearest is None:
            return f"{source_stamp:.9f}", source_frame, "none", "none"
        identity = nearest[0]
        from .projection import vision_overlay_match

        match = vision_overlay_match(
            image_stamp=identity[0],
            image_frame=identity[1],
            diagnostic_stamp=source_stamp,
            diagnostic_frame=source_frame,
            diagnostic_current=True,
            max_delta_seconds=self._annotation_source_tolerance_seconds,
        )
        delta_ms = abs(identity[0] - source_stamp) * 1000.0
        return (
            f"{source_stamp:.9f}",
            source_frame,
            f"{identity[0]:.9f}/{identity[1]}",
            f"{match}:{delta_ms:.3f}ms",
        )

    def _log_annotation_binding_failure(
        self, reason: str, diagnostic: Any | None = None
    ) -> None:
        diagnostic = diagnostic or self._pending_detection_diagnostic
        if diagnostic is None:
            return
        source_stamp, source_frame, nearest, match = self._annotation_binding_context(
            diagnostic
        )
        self._log_video(
            "warning",
            "Hub operator annotation unavailable "
            f"reason={reason} source_frame={source_frame}",
        )

    def _log_video(self, level: str, message: str) -> None:
        try:
            logger = self.node.get_logger()
            getattr(logger, level)(message)
        except (AttributeError, RuntimeError):
            pass

    @staticmethod
    def _goal_id(message: Any) -> str:
        return bytes(message.goal_id.uuid).hex()

    def _receive_execute_cycle_feedback(self, message: Any) -> None:
        self._observe("execute_cycle", message)
        delivery_id = getattr(message.feedback, "delivery_id", None)
        if delivery_id is not None:
            delivery_key = bytes(delivery_id.uuid).hex()
            self._unconsumed_readiness_ids.discard(delivery_key)
            if any(bytes(delivery_id.uuid)):
                if delivery_key not in self._admitted_readiness_ids:
                    self._admitted_readiness_order.append(delivery_key)
                self._admitted_readiness_ids.add(delivery_key)
                while len(self._admitted_readiness_order) > 16:
                    expired = self._admitted_readiness_order.pop(0)
                    self._admitted_readiness_ids.discard(expired)
        goal_id = self._goal_id(message)
        self._remember_execute_cycle_goal(goal_id)
        if self._execute_cycle_latest_terminal_goal_id != goal_id:
            self._execute_cycle_latest_terminal_goal_id = None
        self._active_execute_goal_id = goal_id
        self._try_create_operator_annotation()

    def _receive_execute_cycle_status(self, message: Any) -> None:
        self._execute_cycle_status_received_at = time.monotonic()
        from action_msgs.msg import GoalStatus

        active_statuses = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        terminal_statuses = {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }
        status_list = list(message.status_list)
        if self._active_execute_goal_id is not None:
            matching_terminal = next(
                (
                    item
                    for item in status_list
                    if bytes(item.goal_info.goal_id.uuid).hex()
                    == self._active_execute_goal_id
                    and int(item.status) in terminal_statuses
                ),
                None,
            )
            if matching_terminal is not None:
                self._observe_execute_cycle_terminal(matching_terminal)
            return

        active_items = [
            item for item in status_list if int(item.status) in active_statuses
        ]
        if active_items:
            item = max(active_items, key=self._execute_cycle_status_stamp)
            self._active_execute_goal_id = bytes(item.goal_info.goal_id.uuid).hex()
            if (
                self._execute_cycle_latest_terminal_goal_id
                != self._active_execute_goal_id
            ):
                self._execute_cycle_latest_terminal_goal_id = None
            self._try_create_operator_annotation()
            return

        # Configuration validation can terminate before the first feedback.
        # In that case the existing status and result path identifies the goal.
        terminal_items = [
            item for item in status_list if int(item.status) in terminal_statuses
        ]
        if terminal_items:
            self._observe_execute_cycle_terminal(
                max(terminal_items, key=self._execute_cycle_status_stamp)
            )

    @staticmethod
    def _execute_cycle_status_stamp(item: Any) -> tuple[int, int]:
        stamp = getattr(item.goal_info, "stamp", None)
        return (
            int(getattr(stamp, "sec", 0)),
            int(getattr(stamp, "nanosec", 0)),
        )

    def _observe_execute_cycle_terminal(self, item: Any) -> None:
        goal_id = bytes(item.goal_info.goal_id.uuid).hex()
        self._remember_execute_cycle_goal(goal_id)
        self._execute_cycle_latest_terminal_goal_id = goal_id
        self._execute_cycle_status_by_goal[goal_id] = int(item.status)
        self._request_execute_cycle_result(item.goal_info.goal_id, goal_id)
        if (
            self._detection_snapshot is not None
            and self._detection_snapshot.execute_cycle_goal_id == goal_id
        ):
            self._detection_snapshot = None
        self._active_execute_goal_id = None

    def _remember_execute_cycle_goal(self, goal_id: str) -> None:
        if goal_id in self._execute_cycle_recent_goal_ids:
            self._execute_cycle_recent_goal_ids.remove(goal_id)
        self._execute_cycle_recent_goal_ids.append(goal_id)
        while len(self._execute_cycle_recent_goal_ids) > 8:
            expired = self._execute_cycle_recent_goal_ids.pop(0)
            self._execute_cycle_status_by_goal.pop(expired, None)
            self._execute_cycle_result_by_goal.pop(expired, None)
            self._execute_cycle_result_requests.discard(expired)
            self._execute_cycle_pending_results.pop(expired, None)
            if self._execute_cycle_latest_terminal_goal_id == expired:
                self._execute_cycle_latest_terminal_goal_id = None

    def _request_execute_cycle_result(self, ros_goal_id: Any, goal_id: str) -> None:
        client = self._clients.get("execute_cycle_result")
        if client is None or goal_id in self._execute_cycle_result_requests:
            return
        if hasattr(client, "service_is_ready") and not client.service_is_ready():
            self._execute_cycle_pending_results.setdefault(
                goal_id, (ros_goal_id, time.monotonic())
            )
            return
        self._execute_cycle_pending_results.pop(goal_id, None)
        from arm_cell_interfaces.action import ExecuteCycle

        request = ExecuteCycle.Impl.GetResultService.Request()
        request.goal_id = ros_goal_id
        self._execute_cycle_result_requests.add(goal_id)
        future = client.call_async(request)

        def receive_result(done_future: Any) -> None:
            try:
                response = done_future.result()
                if goal_id in self._execute_cycle_recent_goal_ids:
                    self._execute_cycle_result_by_goal[goal_id] = response
            except Exception:
                # Keep the terminal goal available for a rate-limited read retry.
                self._execute_cycle_result_requests.discard(goal_id)
                if goal_id in self._execute_cycle_recent_goal_ids:
                    self._execute_cycle_pending_results[goal_id] = (
                        ros_goal_id,
                        time.monotonic(),
                    )

        future.add_done_callback(receive_result)

    def _poll_execute_cycle_results(self) -> None:
        """Retry read-only result queries at most once per second per goal."""
        now = time.monotonic()
        for goal_id, (ros_goal_id, last_attempt) in list(
            self._execute_cycle_pending_results.items()
        ):
            if now - last_attempt < 1.0:
                continue
            self._execute_cycle_pending_results[goal_id] = (ros_goal_id, now)
            self._request_execute_cycle_result(ros_goal_id, goal_id)

    def _receive_vision_diagnostic(self, message: Any) -> None:
        self._observe("vision_diagnostic", message)
        received_at = time.monotonic()
        if int(message.state) == 1:
            # This diagnostic is server-wide and cannot establish acceptance
            # of this Hub request or alter its finite client response bound.
            self._clear_pending_detection()
            self._operator_annotation = None
            return
        if int(message.state) != 0:
            return
        if not bool(message.valid) or int(message.result_code) != 0:
            self._clear_pending_detection()
            self._operator_annotation = None
            return
        source = message.source_rgb_header
        event_identity = (
            float(source.stamp.sec) + float(source.stamp.nanosec) / 1e9,
            str(source.frame_id),
            str(message.target_id),
        )
        if event_identity == self._last_success_event:
            return
        self._last_success_event = event_identity
        self._operator_annotation = None
        self._pending_detection_diagnostic = message
        self._pending_detection_received_at = received_at
        self._try_create_operator_annotation()

    def _receive_vision_detector_snapshot(self, message: Any) -> None:
        """Copy Vision's immutable event without decoding in the ROS callback."""
        if self._closed:
            return
        self._vision_detector_snapshot = {
            "snapshot_id": str(message.snapshot_id),
            "revision": int(message.revision),
            "event_kind": int(message.event_kind),
            "vision_run_id": str(message.vision_run_id),
            "calibration_id": str(message.calibration_id),
            "acquisition_id": str(message.acquisition_id),
            "image_format": str(message.image_png.format),
            "image_data": bytes(message.image_png.data),
            "metadata_json": str(message.metadata_json),
            "diagnostic_record_json": str(
                getattr(message, "diagnostic_record_json", "")
            ),
        }
        self._match_retry_detection_snapshot()

    def vision_detector_snapshot(self) -> dict[str, Any] | None:
        return self._vision_detector_snapshot

    def retry_detection_result(self) -> dict[str, Any]:
        """Return the last Vision-only development request and matching Snapshot."""
        return dict(self._retry_detection_result)

    @property
    def retry_detection_inflight(self) -> bool:
        return self._retry_detection_inflight

    def retry_detection_eligibility(
        self, *, now: float | None = None
    ) -> tuple[bool, str]:
        """Check current owner observations immediately before a dev request."""
        now = time.monotonic() if now is None else now
        if self._closed:
            return False, "Hub runtime is closed"
        if self._retry_detection_inflight:
            return False, "Retry Detection request is already in flight"
        if self._active_execute_goal_id is not None:
            return False, "Mission is active"
        try:
            status_publishers = self.node.get_publishers_info_by_topic(
                "/orchestration/execute_cycle/_action/status"
            )
        except Exception:
            status_publishers = ()
        if not status_publishers:
            return False, "Mission status observation is unavailable"
        if self._unconsumed_readiness_ids:
            return False, "Material readiness is pending Mission admission"

        readiness = self.projection.read("readiness", now=now)
        if readiness.value is None:
            return False, "Material readiness observation is unavailable"
        # MATERIAL_READY is a delivery fact whose validity does not expire
        # with its source timestamp. Track valid completion consumption by
        # Delivery ID below; false/invalid samples do not establish readiness.

        vision = self.projection.read("vision_diagnostic", now=now)
        if vision.freshness(now).value != "current":
            return False, "Vision processing observation is unavailable or stale"
        if int(getattr(vision.value, "state", -1)) != 0:
            return False, "Vision is not IDLE"
        return True, "Available"

    def request_retry_detection(self, target_id: str) -> tuple[str, Any | None]:
        """Send one bounded DetectTarget request without invoking Mission or Motion."""
        if self._closed:
            return str(self._retry_detection_result["request_id"] or ""), None
        if not target_id:
            raise ValueError("target_id must be configured")
        if self._retry_detection_inflight:
            return str(self._retry_detection_result["request_id"]), None

        eligible, reason = self.retry_detection_eligibility()

        request_id = str(uuid.uuid4())
        self._retry_detection_result = {
            "request_id": request_id,
            "target_id": target_id,
            "state": "pending" if eligible else "unavailable",
            "result_code": None,
            "acquisition_id": "",
            "diagnostic_detail": "" if eligible else reason,
            "snapshot": None,
        }
        if not eligible:
            return request_id, None
        client = self._clients["retry_detection"]
        if hasattr(client, "service_is_ready") and not client.service_is_ready():
            self._retry_detection_result["state"] = "unavailable"
            self._retry_detection_result["diagnostic_detail"] = (
                "Vision DetectTarget service is unavailable"
            )
            return request_id, None

        from arm_cell_interfaces.srv import DetectTarget

        request = DetectTarget.Request()
        request.target_id = target_id
        request.timeout.sec = int(RETRY_DETECTION_TIMEOUT_SECONDS)
        request.timeout.nanosec = 0
        self._retry_detection_inflight = True
        self._retry_detection_dispatched_at = time.monotonic()
        self._retry_detection_deadline = (
            self._retry_detection_dispatched_at
            + RETRY_DETECTION_ADMISSION_RESPONSE_TIMEOUT_SECONDS
            + RETRY_DETECTION_TIMEOUT_SECONDS
            + RETRY_DETECTION_RESPONSE_MARGIN_SECONDS
        )
        try:
            future = client.call_async(request)
        except (RuntimeError, TypeError, ValueError) as error:
            self._retry_detection_inflight = False
            self._retry_detection_deadline = None
            self._retry_detection_dispatched_at = None
            self._retry_detection_result["state"] = "failed"
            self._retry_detection_result["diagnostic_detail"] = str(error)
            return request_id, None

        future.add_done_callback(
            lambda done, request_id=request_id: self._complete_retry_detection(
                request_id, done
            )
        )
        self._retry_detection_future = future
        return request_id, future

    def vision_calibration_result(self) -> dict[str, Any]:
        return dict(self._vision_calibration_result)

    def request_vision_calibration(self) -> tuple[str, Any | None]:
        """Request one Orchestration-owned Vision Calibration attempt."""
        result = self._vision_calibration_result
        if self._closed:
            return str(result["request_id"] or ""), None
        if result["state"] == "pending":
            return str(result["request_id"]), None

        request_id = str(uuid.uuid4())
        result.update({"request_id": request_id, "state": "pending", "message": ""})
        client = self._clients["vision_calibration"]
        if hasattr(client, "service_is_ready") and not client.service_is_ready():
            result.update(
                {
                    "state": "unavailable",
                    "message": "Calibration service is unavailable",
                }
            )
            return request_id, None

        from std_srvs.srv import Trigger

        try:
            future = client.call_async(Trigger.Request())
        except Exception as error:
            result.update({"state": "failed", "message": str(error)})
            return request_id, None

        def complete(done: Any) -> None:
            if self._closed or result.get("request_id") != request_id:
                return
            try:
                response = done.result()
            except Exception as error:
                result.update({"state": "failed", "message": str(error)})
                return
            result.update(
                {
                    "state": "succeeded" if response.success else "failed",
                    "message": str(response.message),
                }
            )

        future.add_done_callback(complete)
        return request_id, future

    def _expire_retry_detection_request(self, now: float) -> None:
        deadline = self._retry_detection_deadline
        if not self._retry_detection_inflight or deadline is None or now < deadline:
            return
        future = self._retry_detection_future
        if future is not None and future.done():
            self._complete_retry_detection(
                str(self._retry_detection_result["request_id"]), future
            )
            return
        self._retry_detection_inflight = False
        self._retry_detection_deadline = None
        self._retry_detection_dispatched_at = None
        self._retry_detection_future = None
        self._retry_detection_result.update(
            {
                "state": "failed",
                "diagnostic_detail": (
                    "Hub client response watchdog expired before a Vision terminal "
                    "response was received within the bounded acceptance and "
                    "Acquisition response window for a "
                    f"{int(RETRY_DETECTION_TIMEOUT_SECONDS)} second Acquisition"
                ),
                "snapshot": None,
            }
        )

    def _complete_retry_detection(self, request_id: str, future: Any) -> None:
        if self._closed or not self._retry_detection_inflight:
            return
        if self._retry_detection_result.get("request_id") != request_id:
            return
        self._retry_detection_inflight = False
        self._retry_detection_deadline = None
        self._retry_detection_dispatched_at = None
        self._retry_detection_future = None
        try:
            response = future.result()
        except Exception as error:
            self._retry_detection_result["state"] = "failed"
            self._retry_detection_result["diagnostic_detail"] = str(error)
            return

        result_code = int(response.result_code.value)
        from arm_cell_interfaces.msg import DetectTargetResultCode

        is_busy = result_code == DetectTargetResultCode.DETECT_RESULT_BUSY
        acquisition_id = "" if is_busy else str(response.acquisition_id)
        self._retry_detection_result.update(
            {
                "state": (
                    "busy"
                    if is_busy
                    else (
                        "succeeded"
                        if result_code == DetectTargetResultCode.DETECT_RESULT_SUCCESS
                        else "failed"
                    )
                ),
                "result_code": result_code,
                "acquisition_id": acquisition_id,
                "diagnostic_detail": str(response.diagnostic_detail),
                "snapshot": None,
            }
        )
        self._match_retry_detection_snapshot()

    def _match_retry_detection_snapshot(self) -> None:
        if self._closed:
            return
        acquisition_id = self._retry_detection_result.get("acquisition_id")
        snapshot = self._vision_detector_snapshot
        if not acquisition_id or snapshot is None:
            return
        if snapshot["acquisition_id"] != acquisition_id:
            return
        if snapshot["event_kind"] not in (2, 3):
            return
        self._retry_detection_result["snapshot"] = dict(snapshot)

    def _try_create_operator_annotation(self) -> None:
        diagnostic = self._pending_detection_diagnostic
        received_at = self._pending_detection_received_at
        if diagnostic is None or received_at is None:
            return
        if self._detection_snapshot is not None:
            self._clear_pending_detection()
            return
        now = time.monotonic()
        self._prune_source_frames(now)
        if (
            self.feed_max_age_seconds is None
            or now - received_at > self.feed_max_age_seconds
        ):
            self._log_annotation_binding_failure("vision_diagnostic_expired")
            self._clear_pending_detection()
            return
        source = diagnostic.source_rgb_header
        vision_source_identity = (
            float(source.stamp.sec) + float(source.stamp.nanosec) / 1e9,
            str(source.frame_id),
        )
        selected = select_compatible_operator_rgb_frame(
            self._source_frames,
            vision_source_identity,
            max_delta_seconds=self._annotation_source_tolerance_seconds,
        )
        if selected is None:
            return
        identity, rgb_message = selected[0], selected[1]
        annotation = DetectionAnnotation(
            display_frame_identity=identity,
            target_id=str(diagnostic.target_id),
            support_region=(
                tuple(
                    (float(point.x), float(point.y))
                    for point in diagnostic.support_region
                )
                if diagnostic.support_region_valid
                else ()
            ),
            received_at=received_at,
            display_frame_size=(int(rgb_message.width), int(rgb_message.height)),
        )
        self._operator_annotation = annotation
        goal_id = self._active_execute_goal_id
        if goal_id is not None:
            rgb_frame = self._display_frames.get("rgb")
            if rgb_frame is None or rgb_frame.identity != identity:
                rgb_frame = self._display_sources["rgb"].convert(
                    rgb_message, received_at=selected[2]
                )
            rgb_frame = self._immutable_display_frame(rgb_frame)
            depth_frame = self._fresh_depth_snapshot(time.monotonic())
            self._detection_snapshot = DetectionSnapshot(
                rgb_frame=rgb_frame,
                depth_frame=depth_frame,
                annotation=annotation,
                execute_cycle_goal_id=goal_id,
            )
        source_stamp, source_frame, nearest, match_result = (
            self._annotation_binding_context(diagnostic)
        )
        self._log_video(
            "debug",
            "Hub operator annotation bound "
            f"source_frame={source_frame} matching=current",
        )
        self._clear_pending_detection()

    @staticmethod
    def _immutable_display_frame(frame: DisplayFrame) -> DisplayFrame:
        frame.rgba.setflags(write=False)
        return frame

    def _fresh_depth_snapshot(self, now: float) -> DisplayFrame | None:
        message = self._latest_depth_message
        received_at = self._depth_received_at
        if (
            message is None
            or received_at is None
            or self.feed_max_age_seconds is None
            or now - received_at > self.feed_max_age_seconds
        ):
            return None
        identity = image_identity(message)
        cached = self._display_frames.get("depth")
        if cached is not None and cached.identity == identity:
            return self._immutable_display_frame(cached)
        try:
            frame = self._display_sources["depth"].convert(
                message, received_at=received_at
            )
        except (TypeError, ValueError, BufferError) as error:
            self._display_errors["depth"] = str(error)
            return None
        return self._immutable_display_frame(frame)

    def _depth_annotation_compatible(
        self, annotation: DetectionAnnotation, frame: DisplayFrame | None
    ) -> bool:
        if frame is None:
            return False
        return (
            frame.identity[1] == annotation.display_frame_identity[1]
            and (frame.width, frame.height) == annotation.display_frame_size
            and abs(frame.identity[0] - annotation.display_frame_identity[0])
            <= self._annotation_source_tolerance_seconds
        )

    def vision_display(
        self, *, now: float | None = None
    ) -> tuple[str, dict[str, Any] | None]:
        """Return a fresh annotation or the latest processing state."""
        now = time.monotonic() if now is None else now
        if self._selected_display_source == "tray":
            return "TRAY DEPTH", None
        if self._detection_snapshot is not None:
            annotation = self._detection_snapshot.annotation
            if self._selected_display_source == "depth" and not (
                self._depth_annotation_compatible(
                    annotation, self.display_frame(now=now)
                )
            ):
                return "TARGET LOCKED", None
            return "TARGET LOCKED", self._detection_snapshot.metadata()
        annotation = self._current_operator_annotation(now)
        if annotation is not None:
            if self._selected_display_source == "depth" and not (
                self._depth_annotation_compatible(
                    annotation, self.display_frame(now=now)
                )
            ):
                return "TARGET DETECTED", None
            return "TARGET DETECTED", annotation.metadata()
        if self._pending_detection_diagnostic is not None:
            return "ANNOTATION PENDING", None
        snapshot = self.projection.read("vision_diagnostic", now=now)
        diagnostic = snapshot.value
        freshness = snapshot.freshness(now).value
        if diagnostic is None or freshness != "current":
            return freshness, None
        state = {
            0: "IDLE",
            1: "DETECTING",
        }.get(int(diagnostic.state), "unavailable")
        return state, None

    def selected_target_pixel(
        self, *, now: float | None = None
    ) -> tuple[str, tuple[float, float] | None]:
        """Project only the active ExecuteCycle feedback target into RGB pixels."""
        from .projection import camera_overlay_state

        now = time.monotonic() if now is None else now
        info = self.projection.read("camera_info", now=now)
        feedback = self.projection.read("execute_cycle", now=now)
        if self._camera_identity is None or self._camera_size is None:
            return "unavailable", None
        if (
            self._camera_received_at is None
            or self.feed_max_age_seconds is None
            or now - self._camera_received_at > self.feed_max_age_seconds
        ):
            return "stale", None
        freshness = (feedback.freshness(now).value,)
        if info.value is None:
            return "unavailable", None
        info_freshness = info.freshness(now).value
        if info_freshness != "current":
            return info_freshness, None
        if "unavailable" in freshness:
            return "unavailable", None
        if "stale" in freshness:
            return "stale", None
        info_msg = info.value
        wrapped = feedback.value
        target = getattr(
            getattr(wrapped, "feedback", None), "selected_target_pose", None
        )
        fields = getattr(wrapped, "feedback", None)
        if info_msg is None or fields is None or target is None:
            return "unavailable", None
        stamp = getattr(fields, "target_observation_stamp", None)
        target_stamp = float(stamp.sec) + float(stamp.nanosec) / 1e9 if stamp else None
        image_stamp, image_frame = self._camera_identity
        info_stamp = info.source_stamp
        ros_now = float(self.node.get_clock().now().nanoseconds) / 1e9
        age_limits = (self.feed_max_age_seconds, feedback.max_age_seconds)
        if any(value is None or value <= 0 for value in age_limits):
            return "unavailable", None
        max_age = min(value for value in age_limits if value is not None)
        info_size = (int(info_msg.width), int(info_msg.height))

        def project() -> tuple[float, float] | None:
            if self._tf_buffer is None:
                return None
            try:
                from rclpy.time import Time

                pose = target.pose
                source_frame = target.header.frame_id
                target_frame = info_msg.header.frame_id
                if source_frame != target_frame:
                    transform = self._tf_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        Time(seconds=target_stamp),
                    )
                    pose = _transform_pose(pose, transform.transform)
                x, y, z = pose.position.x, pose.position.y, pose.position.z
                if z <= 0:
                    return None
                k = info_msg.k
                if any(float(value) != 0.0 for value in getattr(info_msg, "d", ())):
                    return None
                pixel_x = (k[0] * x / z) + k[2]
                pixel_y = (k[4] * y / z) + k[5]
                return float(pixel_x), float(pixel_y)
            except Exception:
                return None

        state, pixel = camera_overlay_state(
            image_stamp=image_stamp,
            image_frame=image_frame,
            image_size=self._camera_size,
            camera_stamp=info_stamp,
            camera_frame=info_msg.header.frame_id,
            camera_size=info_size,
            target_valid=bool(fields.target_valid),
            has_selected_target_pose=bool(fields.has_selected_target_pose),
            target_stamp=target_stamp,
            now=ros_now,
            max_age_seconds=max_age,
            transform_available=self._tf_buffer is not None,
            project=project,
        )
        return state, pixel

    def observation_text(self, name: str, *, now: float | None = None) -> str:
        """Format owner values without translating them into Hub authority."""
        now = time.monotonic() if now is None else now
        snapshot = self.projection.read(name, now=now)
        freshness = snapshot.freshness(now).value
        message = snapshot.value
        if (
            name == "execute_cycle"
            and self._execute_cycle_latest_terminal_goal_id
            and (
                message is None
                or freshness != "current"
                or self._goal_id(message) != self._execute_cycle_latest_terminal_goal_id
            )
        ):
            terminal_text = self._execute_cycle_terminal_text(
                self._execute_cycle_latest_terminal_goal_id
            )
            if terminal_text:
                return f"mission: progress=unavailable · {terminal_text}"
        if message is None or freshness != "current":
            return f"{name}: {freshness}"
        if name == "safety":
            return (
                f"safety: current · state={message.safety_state} "
                f"capability={message.motion_capability.value} "
                f"stop_mode={message.selected_stop_mode.value} "
                f"envelope_valid={message.motion_envelope_valid} "
                f"velocity_scale={message.max_velocity_scale:g} "
                f"acceleration_scale={message.max_acceleration_scale:g} "
                f"valid={message.valid} causes={message.active_causes.value} "
                f"latched={message.latched_causes.value}"
            )
        if name == "motion":
            return (
                f"motion: current · execution={message.execution_state} "
                f"holding={message.holding_state} active={message.execution_active} "
                f"velocity_scale={message.applied_velocity_scale:g} "
                f"acceleration_scale={message.applied_acceleration_scale:g}"
            )
        if name == "amr":
            return (
                f"amr: current · docking={message.docking_state} valid={message.valid}"
            )
        if name == "handoff":
            return f"material handoff: current · phase={message.phase} valid={message.valid}"
        if name == "accepted_handoff":
            delivery_id = uuid.UUID(bytes=bytes(message.delivery_id.uuid))
            seed = message.random_seed if message.has_random_seed else "omitted"
            return (
                f"accepted handoff: current · delivery_id={delivery_id} "
                f"type={message.material_type} quantity={message.quantity} seed={seed}"
            )
        if name == "trigger_status":
            delivery_id = uuid.UUID(bytes=bytes(message.delivery_id.uuid))
            state_names = {
                getattr(message, "TRIGGER_ACCEPTED", 0): "accepted",
                getattr(message, "TRIGGER_REJECTED", 1): "rejected",
            }
            rejection_names = {
                getattr(message, "REJECTION_NONE", 0): "none",
                getattr(message, "REJECTION_INVALID_TRIGGER", 1): "invalid_trigger",
                getattr(
                    message, "REJECTION_INVALID_DELIVERY_ID", 2
                ): "invalid_delivery_id",
                getattr(message, "REJECTION_STALE_SOURCE_TIMESTAMP", 3): (
                    "stale_source_timestamp"
                ),
                getattr(message, "REJECTION_INVALID_QUANTITY", 4): "invalid_quantity",
                getattr(message, "REJECTION_UNSUPPORTED_MATERIAL_TYPE", 5): (
                    "unsupported_material_type"
                ),
                getattr(message, "REJECTION_DUPLICATE_DELIVERY_ID", 6): (
                    "duplicate_delivery_id"
                ),
            }
            detail = (
                f" detail={message.diagnostic_detail}"
                if message.diagnostic_detail
                else ""
            )
            return (
                f"handoff trigger decision: current · delivery_id={delivery_id} "
                f"state={state_names.get(message.state, message.state)} "
                f"rejection={rejection_names.get(message.rejection_reason, message.rejection_reason)}"
                f"{detail}"
            )
        if name == "transfer_completion":
            delivery_id = uuid.UUID(bytes=bytes(message.delivery_id.uuid))
            failure_names = {
                getattr(message, "TRANSFER_FAILURE_NONE", 0): "none",
                getattr(message, "TRANSFER_FAILURE_SUPPLY", 1): "supply",
                getattr(message, "TRANSFER_FAILURE_CANCELED", 4): "canceled",
            }
            return (
                f"material supply: current · delivery_id={delivery_id} "
                f"type={message.material_type} requested={message.requested_quantity} "
                f"supplied={message.supplied_quantity} "
                f"completed={message.completed} valid={message.valid} "
                f"failure={failure_names.get(message.failure_reason, message.failure_reason)}"
            )
        if name == "scene_stability":
            status_names = {
                getattr(message, "SCENE_STABILITY_UNOBSERVABLE", 0): ("UNOBSERVABLE"),
                getattr(message, "SCENE_STABILITY_UNSTABLE", 1): "UNSTABLE",
                getattr(message, "SCENE_STABILITY_STABLE", 2): "STABLE",
            }
            return (
                f"vision scene: current · target={message.target_id} "
                f"status={status_names.get(message.status, f'UNKNOWN({message.status})')} "
                f"generation={message.scene_generation}"
            )
        if name == "vision_calibration":
            state = (
                "READY"
                if int(message.state) == int(message.STATE_READY)
                else "CALIBRATING"
            )
            residual_mm = float(message.median_depth_residual_m) * 1000.0
            return (
                f"Tray calibration: {state} · frames={message.accepted_frames}/"
                f"{message.required_frames} · depth residual={residual_mm:.1f} mm"
            )
        if name == "readiness":
            delivery_id = uuid.UUID(bytes=bytes(message.delivery_id.uuid))
            return (
                f"material readiness: current · ready={message.material_ready} "
                f"valid={message.valid} type={message.material_type} "
                f"quantity={message.quantity} delivery_id={delivery_id}"
            )
        if name == "packml":
            return f"external process: current · state={message.state} valid={message.valid}"
        if name == "execute_cycle":
            feedback = message.feedback
            text = (
                f"mission: current · phase={feedback.phase} "
                f"processed={feedback.batch_processed_count}/{feedback.quantity} "
                f"retry={feedback.retry_count} target_valid={feedback.target_valid} "
                f"interruptions={feedback.interruption_count} "
                f"first_exit={feedback.first_interruption_reason.value} "
                f"first_causes={feedback.first_interruption_causes.value} "
                f"first_stop={feedback.first_interruption_stop_mode.value}"
            )
            goal_id = self._goal_id(message)
            terminal_text = self._execute_cycle_terminal_text(goal_id)
            return text + (f" · {terminal_text}" if terminal_text else "")
        return f"{name}: current"

    def _execute_cycle_terminal_text(self, goal_id: str) -> str:
        status = self._execute_cycle_status_by_goal.get(goal_id)
        if status is None:
            return ""
        from action_msgs.msg import GoalStatus

        status_names = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        text = f"terminal={status_names.get(status, status)}"
        response = self._execute_cycle_result_by_goal.get(goal_id)
        if response is None:
            return text
        from arm_cell_interfaces.msg import MissionExitReason

        reason_names = {
            int(getattr(MissionExitReason, name)): name
            for name in dir(MissionExitReason)
            if name.startswith("MISSION_EXIT_")
            and isinstance(getattr(MissionExitReason, name), int)
        }
        result = response.result
        reason = int(result.exit_reason.value)
        return (
            f"{text} · exit_reason={reason_names.get(reason, f'UNKNOWN({reason})')}"
            f" diagnostic_detail={result.diagnostic_detail}"
        )

    def _subscribe(self, message_type: Any, topic: str, key: str, qos: Any) -> None:
        def receive(message: Any) -> None:
            self._observe(key, message)

        self._subscriptions.append(
            self.node.create_subscription(message_type, topic, receive, qos)
        )

    def _receive_material_readiness(self, message: Any) -> None:
        self._observe("readiness", message)
        if bool(getattr(message, "material_ready", False)) and bool(
            getattr(message, "valid", False)
        ):
            delivery_id = bytes(message.delivery_id.uuid)
            if (
                any(delivery_id)
                and delivery_id.hex() not in self._admitted_readiness_ids
            ):
                self._unconsumed_readiness_ids.add(delivery_id.hex())

    def _observe(self, key: str, message: Any) -> None:
        now = time.monotonic()
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        source_stamp = (
            float(stamp.sec) + float(stamp.nanosec) / 1e9 if stamp is not None else None
        )
        source_age = None
        source_timestamped = {
            "readiness",
            "accepted_handoff",
            "trigger_status",
            "transfer_completion",
            "scene_stability",
        }
        if key in source_timestamped and source_stamp is not None:
            source_now = self.node.get_clock().now().nanoseconds / 1e9
            source_age = source_now - source_stamp
        self.projection.observe(
            key,
            message,
            received_at=now,
            source_stamp=source_stamp,
            max_age_seconds=self.feed_max_age_seconds,
            source_age_seconds=source_age,
        )

    def publish_handoff_trigger(
        self,
        material_type: str,
        quantity: int,
        *,
        random_seed: int | None = None,
    ) -> str:
        """Emit the terminal-compatible external trigger; Integration owns acceptance."""
        if not material_type:
            raise ValueError("material_type must be configured")
        if not 0 < quantity < 2**32:
            raise ValueError("quantity must fit a positive uint32")
        if random_seed is not None and not 0 <= random_seed < 2**64:
            raise ValueError("random_seed must fit uint64")
        from arm_cell_interfaces.msg import HandoffTrigger

        delivery_id = uuid.uuid4()
        trigger = HandoffTrigger()
        trigger.header.stamp = self.node.get_clock().now().to_msg()
        trigger.delivery_id.uuid = list(delivery_id.bytes)
        trigger.material_type = material_type
        trigger.quantity = quantity
        trigger.has_random_seed = random_seed is not None
        trigger.random_seed = 0 if random_seed is None else random_seed
        trigger.valid = True
        self._trigger_publisher.publish(trigger)
        return str(delivery_id)

    def acknowledge_and_reset(self) -> tuple[str, Any]:
        from arm_cell_interfaces.srv import ResetSafety

        request_id = str(uuid.uuid4())
        request = ResetSafety.Request()
        request.request_id.uuid = list(uuid.UUID(request_id).bytes)
        request.operator_acknowledged = True
        client = self._clients["safety_reset"]
        if hasattr(client, "service_is_ready") and not client.service_is_ready():
            self.request_state[request_id] = "unavailable"
            return request_id, None
        self.request_state[request_id] = "pending"
        future = client.call_async(request)

        def complete(result: Any) -> None:
            try:
                response = result.result()
                self.request_detail[request_id] = response.diagnostic_detail
                self.request_state[request_id] = (
                    "applied_pending_owner_state" if response.applied else "rejected"
                )
            except Exception:
                self.request_state[request_id] = "unavailable"

        future.add_done_callback(complete)
        return request_id, future

    def request_gripper_open(
        self, object_supported: bool, release_area_clear: bool
    ) -> tuple[str, Any]:
        from arm_cell_interfaces.srv import RequestGripperOpen

        request_id = str(uuid.uuid4())
        client = self._clients.get("gripper_open")
        if client is None:
            client = self.node.create_client(
                RequestGripperOpen, "/motion/operator/gripper_open"
            )
            self._clients["gripper_open"] = client
        if hasattr(client, "service_is_ready") and not client.service_is_ready():
            self.request_state[request_id] = "unavailable"
            return request_id, None
        request = RequestGripperOpen.Request()
        request.object_supported_or_restrained = object_supported
        request.release_area_clear = release_area_clear
        self.request_state[request_id] = "pending"
        future = client.call_async(request)

        def complete(result: Any) -> None:
            try:
                response = result.result()
                self.request_state[request_id] = (
                    "accepted" if response.accepted else "rejected"
                )
                self.request_detail[request_id] = response.diagnostic_detail
                self.request_detail[f"{request_id}:execution"] = (
                    f"command_issued={response.command_issued}; "
                    f"released_confirmed={response.released_confirmed}"
                )
            except Exception:
                self.request_state[request_id] = "unavailable"

        future.add_done_callback(complete)
        return request_id, future

    def set_external_fault(
        self, scenario: str, active: bool, *, request_id: str | None = None
    ) -> tuple[str, Any | None]:
        """Request one existing VDA mock SetBool fault without waiting for it."""
        if scenario not in VDA_MOCK_FAULT_SCENARIOS:
            raise ValueError(f"unsupported VDA mock fault: {scenario}")

        from std_srvs.srv import SetBool

        request_id = request_id or str(uuid.uuid4())
        client_key = f"vda_fault:{scenario}"
        client = self._clients.get(client_key)
        if client is None:
            client = self.node.create_client(
                SetBool, f"/external_state_mock/fault/{scenario}"
            )
            self._clients[client_key] = client
        if hasattr(client, "service_is_ready") and not client.service_is_ready():
            self.request_state[request_id] = "unavailable"
            return request_id, None

        request = SetBool.Request()
        request.data = active
        self.request_state[request_id] = "pending"
        try:
            future = client.call_async(request)
        except Exception as error:
            self.request_state[request_id] = f"failed: {error}"
            return request_id, None

        def complete(result: Any) -> None:
            try:
                response = result.result()
                if not response.success:
                    self.request_state[request_id] = (
                        f"rejected: {response.message or 'request rejected'}"
                    )
                    return
                self.fault_active[scenario] = active
                message = response.message or "mock accepted the request"
                self.request_state[request_id] = f"applied: {message}"
            except Exception as error:
                self.request_state[request_id] = f"failed: {error}"

        future.add_done_callback(complete)
        return request_id, future

    def set_vision_mislocalize(self, active: bool) -> tuple[str, Any | None]:
        from std_srvs.srv import SetBool

        request_id = str(uuid.uuid4())
        client = self._clients.get("vision_mislocalize")
        if client is None:
            client = self.node.create_client(
                SetBool, "/vision/test_scenario/mislocalize_next"
            )
            self._clients["vision_mislocalize"] = client
        if hasattr(client, "service_is_ready") and not client.service_is_ready():
            self.request_state[request_id] = "unavailable"
            return request_id, None
        request = SetBool.Request()
        request.data = active
        self.request_state[request_id] = "pending"
        future = client.call_async(request)

        def complete(result: Any) -> None:
            try:
                response = result.result()
                self.request_detail[request_id] = response.message
                self.request_state[request_id] = (
                    "accepted" if response.success else "rejected"
                )
            except Exception:
                self.request_state[request_id] = "unavailable"

        future.add_done_callback(complete)
        return request_id, future

    def set_scenario_active(self, owner: Owner, scenario: str, active: bool) -> bool:
        """Pass intent to an injected owner-local private adapter only."""
        return self.scenario_ports.set_active(owner, scenario, active)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._destroy_display_subscription()
        self._clear_display_source_data(self._selected_display_source)
        self._retry_detection_inflight = False
        self._retry_detection_deadline = None
        self._retry_detection_dispatched_at = None
        if self._executor is not None:
            self._executor.remove_node(self.node)
            self._executor.shutdown()
            self._executor = None
        for subscription in self._subscriptions:
            self.node.destroy_subscription(subscription)
        for client in self._clients.values():
            self.node.destroy_client(client)
        destroy_publisher = getattr(self.node, "destroy_publisher", None)
        if destroy_publisher is not None:
            destroy_publisher(self._trigger_publisher)
        self.node.destroy_node()


def main() -> None:
    import rclpy

    runtime = HubRuntime()
    try:
        rclpy.spin(runtime.node)
    finally:
        runtime.close()
        if rclpy.ok():
            rclpy.shutdown()


def _transform_pose(pose: Any, transform: Any) -> Any:
    """Return a copied pose transformed by the existing TF at its observation time."""
    import copy
    import math

    out = copy.deepcopy(pose)
    q = transform.rotation
    p = transform.translation
    # q * v * q^-1, using the normalized TF quaternion.
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm == 0:
        raise ValueError("invalid transform quaternion")
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    vx, vy, vz = pose.position.x, pose.position.y, pose.position.z
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    out.position.x = vx + w * tx + (y * tz - z * ty) + p.x
    out.position.y = vy + w * ty + (z * tx - x * tz) + p.y
    out.position.z = vz + w * tz + (x * ty - y * tx) + p.z
    return out


if __name__ == "__main__":
    main()
