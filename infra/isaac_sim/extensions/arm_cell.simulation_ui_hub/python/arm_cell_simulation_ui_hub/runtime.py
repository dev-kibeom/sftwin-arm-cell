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
VDA_MOCK_FAULT_SCENARIOS = (
    "material_handoff_failure",
    "e_stop",
    "premature_undock",
    "packml_abort",
    "invalid_state",
    "communication_loss",
    "communication_degradation",
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
    centroid: tuple[float, float] | None
    object_region: tuple[int, int, int, int] | None
    support_region: tuple[tuple[float, float], ...]
    yaw_available: bool
    received_at: float
    display_frame_size: tuple[int, int]

    def metadata(self) -> dict[str, Any]:
        return {
            "target_id": self.target_id,
            "centroid": self.centroid,
            "object_region": self.object_region,
            "support_region": list(self.support_region),
            "yaw_available": self.yaw_available,
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

            if not rclpy.ok():
                rclpy.init(args=None)
            node = Node("simulation_ui_hub")
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
        }
        self._selected_display_source = "rgb"
        self._latest_rgb_message: Any | None = None
        self._latest_depth_message: Any | None = None
        self._depth_received_at: float | None = None
        self._display_frames: dict[str, DisplayFrame] = {}
        self._display_errors: dict[str, str] = {}
        self._last_success_event: tuple[float, str, str] | None = None
        self._pending_detection_diagnostic: Any | None = None
        self._pending_detection_received_at: float | None = None
        self._active_execute_goal_id: str | None = None
        self._operator_annotation: DetectionAnnotation | None = None
        self._detection_snapshot: DetectionSnapshot | None = None
        self._raw_frame_available: bool | None = None
        self._executor: Any | None = None
        self._closed = False
        self._connect_ros()

    def spin_once(self) -> None:
        """Pump ROS callbacks from the Isaac Kit update loop."""
        if self._executor is None:
            from rclpy.executors import SingleThreadedExecutor

            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self.node)
        self._executor.spin_once(timeout_sec=0.0)
        self._prune_source_frames(time.monotonic())

    def _connect_ros(self) -> None:
        from arm_cell_interfaces.msg import (
            AMRDockingState,
            MaterialHandoffState,
            MaterialReadiness,
            MotionStatus,
            PackMLState,
            SafetyState,
            VisionDiagnostic,
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
        readiness_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscribe(
            MaterialReadiness,
            "/integration/material_readiness",
            "readiness",
            readiness_qos,
        )
        self._subscribe(PackMLState, "/packml/state", "packml", 10)
        camera_qos = QoSProfile(
            depth=CAMERA_SUBSCRIPTION_QUEUE_DEPTH,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions.append(
            self.node.create_subscription(
                Image,
                "/camera/color/image_raw",
                self._receive_raw_rgb,
                camera_qos,
            )
        )
        self._subscriptions.append(
            self.node.create_subscription(
                Image,
                "/camera/aligned_depth_to_color/image_raw",
                self._receive_aligned_depth,
                camera_qos,
            )
        )
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
        self._subscribe(
            CameraInfo,
            "/camera/color/camera_info",
            "camera_info",
            qos_profile_sensor_data,
        )
        from arm_cell_interfaces.action._execute_cycle import (
            ExecuteCycle_FeedbackMessage,
        )
        from action_msgs.msg import GoalStatusArray

        self._subscriptions.append(
            self.node.create_subscription(
                ExecuteCycle_FeedbackMessage,
                "/orchestration/execute_cycle/_action/feedback",
                self._receive_execute_cycle_feedback,
                10,
            )
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

        from arm_cell_interfaces.srv import RequestMaterialSupply, ResetSafety

        self._clients["material"] = self.node.create_client(
            RequestMaterialSupply, "/integration/request_material"
        )
        self._clients["safety_reset"] = self.node.create_client(
            ResetSafety, "/safety/reset"
        )

        # TF is consumed read-only to project the canonical selected target.
        try:
            from tf2_ros import Buffer, TransformListener

            self._tf_buffer = Buffer(node=self.node)
            self._tf_listener = TransformListener(self._tf_buffer, self.node)
        except (ImportError, AttributeError):
            self._tf_buffer = None

    def _live_camera_frame_rgba(self, now: float) -> tuple[Any, int, int] | None:
        frame = self.display_frame(now=now)
        if frame is None:
            return None
        return frame.rgba, frame.width, frame.height

    def set_display_source(self, source_id: str) -> None:
        if source_id not in self._display_sources:
            raise ValueError(f"unsupported Hub display source: {source_id}")
        self._selected_display_source = source_id

    def display_source(self) -> str:
        return self._selected_display_source

    def display_frame(self, *, now: float | None = None) -> DisplayFrame | None:
        """Return the selected live frame or its immutable PnP capture."""
        now = time.monotonic() if now is None else now
        if self._detection_snapshot is not None:
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
        else:
            message = self._latest_depth_message
            received_at = self._depth_received_at
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
        frame = self.display_frame(now=now)
        if frame is not None:
            if self._detection_snapshot is not None:
                return f"{self._selected_display_source.upper()} · PnP SNAPSHOT"
            return f"{self._selected_display_source.upper()} · LIVE"
        error = self._display_errors.get(self._selected_display_source)
        if error:
            return f"{self._selected_display_source.upper()} UNAVAILABLE · {error}"
        received_at = (
            self._camera_received_at
            if self._selected_display_source == "rgb"
            else self._depth_received_at
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
                self._log_video("warning", "raw RGB operator frame unavailable")
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
            f"run_id={self._active_execute_goal_id} target_id={diagnostic.target_id} "
            f"reason={reason} source_stamp={source_stamp} source_frame={source_frame} "
            f"nearest_operator_frame={nearest} matching={match}",
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
        goal_id = self._goal_id(message)
        self._active_execute_goal_id = goal_id
        self._try_create_operator_annotation()

    def _receive_execute_cycle_status(self, message: Any) -> None:
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
        for item in message.status_list:
            goal_id = bytes(item.goal_info.goal_id.uuid).hex()
            status = int(item.status)
            if (
                goal_id != self._active_execute_goal_id
                or status not in terminal_statuses
            ):
                continue
            if (
                self._detection_snapshot is not None
                and self._detection_snapshot.execute_cycle_goal_id == goal_id
            ):
                self._detection_snapshot = None
            self._active_execute_goal_id = None
            return
        if self._active_execute_goal_id is None:
            active = next(
                (
                    bytes(item.goal_info.goal_id.uuid).hex()
                    for item in message.status_list
                    if int(item.status) in active_statuses
                ),
                None,
            )
            if active is not None:
                self._active_execute_goal_id = active
                self._try_create_operator_annotation()

    def _receive_vision_diagnostic(self, message: Any) -> None:
        self._observe("vision_diagnostic", message)
        received_at = time.monotonic()
        if int(message.state) == 1:
            # DETECTING is presentation state only. The terminal observation
            # owns the source identity used to select an operator frame.
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
            centroid=(
                (float(diagnostic.centroid_pixel_x), float(diagnostic.centroid_pixel_y))
                if diagnostic.centroid_pixel_valid
                else None
            ),
            object_region=(
                (
                    int(diagnostic.object_region_x),
                    int(diagnostic.object_region_y),
                    int(diagnostic.object_region_width),
                    int(diagnostic.object_region_height),
                )
                if diagnostic.object_region_valid
                else None
            ),
            support_region=(
                tuple(
                    (float(point.x), float(point.y))
                    for point in diagnostic.support_region
                )
                if diagnostic.support_region_valid
                else ()
            ),
            yaw_available=bool(diagnostic.yaw_available),
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
            f"execute_cycle_run_id={goal_id or 'none'} "
            f"target_id={diagnostic.target_id} "
            f"source_stamp={source_stamp} source_frame={source_frame} "
            f"operator_frame={identity[0]:.9f}/{identity[1]} "
            f"nearest_operator_frame={nearest} matching=current "
            f"nearest_match={match_result}",
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
        if name == "readiness":
            delivery_id = uuid.UUID(bytes=bytes(message.delivery_id.uuid))
            return (
                f"material readiness: current · ready={message.material_ready} "
                f"valid={message.valid} delivery_id={delivery_id}"
            )
        if name == "packml":
            return f"external process: current · state={message.state} valid={message.valid}"
        if name == "execute_cycle":
            feedback = message.feedback
            return (
                f"mission: current · phase={feedback.phase} "
                f"processed={feedback.batch_processed_count} "
                f"retry={feedback.retry_count} target_valid={feedback.target_valid} "
                f"interruptions={feedback.interruption_count} "
                f"first_exit={feedback.first_interruption_reason.value} "
                f"first_causes={feedback.first_interruption_causes.value} "
                f"first_stop={feedback.first_interruption_stop_mode.value}"
            )
        return f"{name}: current"

    def _subscribe(self, message_type: Any, topic: str, key: str, qos: Any) -> None:
        def receive(message: Any) -> None:
            self._observe(key, message)

        self._subscriptions.append(
            self.node.create_subscription(message_type, topic, receive, qos)
        )

    def _observe(self, key: str, message: Any) -> None:
        now = time.monotonic()
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        source_stamp = (
            float(stamp.sec) + float(stamp.nanosec) / 1e9 if stamp is not None else None
        )
        source_age = None
        if key == "readiness" and source_stamp is not None:
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

    def request_material(self) -> tuple[str, Any]:
        from arm_cell_interfaces.srv import RequestMaterialSupply

        request_id = str(uuid.uuid4())
        request = RequestMaterialSupply.Request()
        request.request_id.uuid = list(uuid.UUID(request_id).bytes)
        client = self._clients["material"]
        if hasattr(client, "service_is_ready") and not client.service_is_ready():
            self.request_state[request_id] = "unavailable"
            return request_id, None
        self.request_state[request_id] = "pending"
        future = client.call_async(request)

        def complete(result: Any) -> None:
            try:
                response = result.result()
                self.request_state[request_id] = (
                    "accepted" if response.accepted else "rejected"
                )
                # delivery_id is intentionally not cached: canonical
                # readiness/handoff observations remain the outcome source.
            except Exception:
                self.request_state[request_id] = "unavailable"

        future.add_done_callback(complete)
        return request_id, future

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
                self.request_state[request_id] = (
                    "applied_pending_owner_state" if response.applied else "rejected"
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

    def set_scenario_active(self, owner: Owner, scenario: str, active: bool) -> bool:
        """Pass intent to an injected owner-local private adapter only."""
        return self.scenario_ports.set_active(owner, scenario, active)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._executor is not None:
            self._executor.remove_node(self.node)
            self._executor.shutdown()
            self._executor = None
        for subscription in self._subscriptions:
            self.node.destroy_subscription(subscription)
        for client in self._clients.values():
            self.node.destroy_client(client)
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
