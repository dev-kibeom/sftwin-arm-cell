"""Isaac Kit extension entry point for the Hub skeleton."""

from __future__ import annotations

import logging
from typing import Any

import numpy as np


_CAMERA_DISPLAY_WIDTH = 480
_CAMERA_DISPLAY_HEIGHT = 270
_WIDE_LAYOUT_MIN_WIDTH = 780
_CAMERA_ASPECT_RATIO = _CAMERA_DISPLAY_WIDTH / _CAMERA_DISPLAY_HEIGHT
_TARGET_MARKER_RADIUS = 20
_TARGET_MARKER_COLOR = np.array((32, 255, 96, 255), dtype=np.uint8)
_LOGGER = logging.getLogger(__name__)
_FAULT_GROUPS = (
    (
        "Final Demo Fault Injection",
        "external_mock",
        (
            (
                "e_stop",
                "Emergency Stop",
                "Inject or clear the E-Stop input; clearing does not reset the Safety latch.",
            ),
            (
                "safety_signal_loss",
                "Safety Signal Loss",
                "Suppress Safety-HW signals for five seconds, then request fresh publication.",
            ),
            (
                "mislocalize_next",
                "Vision Mislocalization",
                "Offset the next successful detection upward by 100 mm.",
            ),
        ),
    ),
)


try:
    import omni.ext

    _ExtensionBase = omni.ext.IExt
except ImportError:  # Importable by structural smoke outside Isaac Kit.

    class _ExtensionBase:
        pass


class SimulationUIHubExtension(_ExtensionBase):
    """Lazy Isaac extension so deterministic checks work outside Kit."""

    WINDOW_TITLE = "ARM Cell Simulation UI Hub"

    def on_startup(self, ext_id: str) -> None:
        import omni.ui as ui
        import os
        import sys
        from pathlib import Path

        from .runtime import HubRuntime
        from .scenario_ports import bind_owner_scenario_port
        from omni.kit.menu.utils import MenuHelperExtensionFull

        scripts_path = str(Path(__file__).resolve().parents[4] / "scripts")
        cell_scripts_path = str(Path(scripts_path) / "m0609_cell")
        for path in (scripts_path, cell_scripts_path):
            if path not in sys.path:
                sys.path.insert(0, path)
        from graph_builder.ros_action_graph import apply_rgbd_source_scenario
        from gripper_runtime.holding_scenario import set_holding_unknown

        bind_owner_scenario_port(
            "rgbd_source",
            lambda intent: apply_rgbd_source_scenario(intent.scenario, intent.active),
        )
        bind_owner_scenario_port(
            "motion_backend",
            lambda intent: (
                set_holding_unknown(intent.active)
                if intent.scenario == "holding_unknown"
                else False
            ),
        )

        from .runtime import (
            DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS,
            DEFAULT_OBSERVATION_FRESHNESS_SECONDS,
        )

        handoff_material_type = "RawPart"
        handoff_default_quantity = 1
        retry_detection_target_id = "RawPart"
        project_root = os.environ.get("SFTWIN_PROJECT_ROOT")
        if project_root:
            import yaml

            profile_path = (
                Path(project_root).expanduser().resolve()
                / "ros2_ws/src/arm_cell_bringup/config/operational_profile.yaml"
            )
            profile = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
            simulation_profile = profile.get("simulation", {})
            handoff_material_type = str(
                simulation_profile.get("external_handoff_material_type", "RawPart")
            )
            handoff_default_quantity = int(
                simulation_profile.get("external_handoff_default_quantity", 1)
            )
            retry_detection_target_id = str(
                profile.get("orchestration", {}).get("mission_target_id", "RawPart")
            )

        try:
            import carb

            freshness_default = carb.settings.get_settings().get(
                "/exts/arm_cell.simulation_ui_hub/observation_freshness_seconds"
            )
            annotation_tolerance = carb.settings.get_settings().get(
                "/exts/arm_cell.simulation_ui_hub/annotation_source_tolerance_seconds"
            )
            self._retry_detection_enabled = bool(
                carb.settings.get_settings().get(
                    "/exts/arm_cell.simulation_ui_hub/developer_tools/enable_retry_detection"
                )
            )
            if freshness_default is None:
                freshness_default = DEFAULT_OBSERVATION_FRESHNESS_SECONDS
        except (ImportError, AttributeError):
            freshness_default = DEFAULT_OBSERVATION_FRESHNESS_SECONDS
            annotation_tolerance = DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS
            self._retry_detection_enabled = False
        self.runtime = HubRuntime(
            default_observation_freshness_seconds=float(freshness_default),
            default_annotation_source_tolerance_seconds=float(
                annotation_tolerance
                if annotation_tolerance is not None
                else DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS
            ),
        )
        self._handoff_default_material_type = handoff_material_type
        self._handoff_default_quantity = handoff_default_quantity
        self._retry_detection_target_id = retry_detection_target_id
        self._ui = ui
        self._window = None
        self._window_menu = MenuHelperExtensionFull()
        self._window_menu.menu_startup(
            self._create_window,
            self.WINDOW_TITLE,
            self.WINDOW_TITLE,
            "Window",
        )
        ui.Workspace.show_window(self.WINDOW_TITLE, True)
        self._latest_image: Any | None = None
        self._image_provider = ui.ByteImageProvider()
        self._detector_image_provider = ui.ByteImageProvider()
        self._detector_snapshot_id: str | None = None
        self._detector_image_decode_available: bool | None = None
        self._detector_metadata_available: bool | None = None
        self._detector_image: Any | None = None
        self._detector_event_text = (
            "Waiting for Vision Snapshot · source image unavailable"
        )
        self._detector_diagnostics_text = "Vision metadata unavailable"
        self._detector_image_widget: Any | None = None
        self._detector_event_label: Any | None = None
        self._detector_diagnostics_label: Any | None = None
        self._vision_calibration_button: Any | None = None
        self._vision_calibration_status: Any | None = None
        self._last_image_source: Any | None = None
        self._last_image_identity: tuple[Any, ...] | None = None
        self._last_ui_update = 0.0
        self._annotation_render_available: bool | None = None
        self._video_mode_label: Any | None = None
        self._display_status_label: Any | None = None
        self._calibration_status_label: Any | None = None
        self._display_source_buttons: dict[str, Any] = {}
        self._target_label: Any | None = None
        self._operator_request_status_labels: dict[str, Any] = {}
        self._handoff_material_type_field: Any | None = None
        self._handoff_quantity_field: Any | None = None
        self._demonstration_combo: Any | None = None
        self._demonstration_button: Any | None = None
        self._demonstration_status: Any | None = None
        self._demonstration_loaded_this_session = False
        self._last_demonstration_state_refresh = 0.0
        self._handoff_trigger_status: Any | None = None
        self._retry_detection_button: Any | None = None
        self._retry_detection_status: Any | None = None
        self._operator_request_ids: dict[str, str] = {}
        self._gripper_supported_confirmed = False
        self._gripper_area_clear_confirmed = False
        self._fault_buttons: dict[tuple[str, str, bool], Any] = {}
        self._fault_status_labels: dict[tuple[str, str], Any] = {}
        self._fault_request_ids: dict[tuple[str, str], str] = {}
        self._fault_local_status: dict[tuple[str, str], str] = {}
        self._scenario_active: dict[tuple[str, str], bool] = {}
        self._timer = None
        self._timeline_subscription = None
        try:
            import omni.kit.app

            self._timer = (
                omni.kit.app.get_app()
                .get_update_event_stream()
                .create_subscription_to_pop(
                    self._on_update, name="arm_cell_simulation_ui_hub"
                )
            )
        except (ImportError, AttributeError):
            self._timer = None
        try:
            import omni.timeline

            self._timeline_subscription = (
                omni.timeline.get_timeline_interface()
                .get_timeline_event_stream()
                .create_subscription_to_pop(
                    self._on_timeline_event,
                    name="arm_cell_simulation_ui_hub_timeline",
                )
            )
        except (ImportError, AttributeError):
            self._timeline_subscription = None

    def _create_window(self):
        """Create the UI window when startup or the Window menu requests it."""
        self._window = self._ui.Window(self.WINDOW_TITLE, width=900, height=640)
        self._window.frame.set_build_fn(self._build)
        self._layout_mode = self._layout_mode_for_width(self._window.width)
        self._window.set_width_changed_fn(self._on_window_width_changed)
        return self._window

    @staticmethod
    def _layout_mode_for_width(width: float) -> str:
        return "wide" if width >= _WIDE_LAYOUT_MIN_WIDTH else "narrow"

    @staticmethod
    def _camera_size_for_width(width: float) -> tuple[float, float]:
        available_width = max(72, int(width) - 36)
        image_width = min(_CAMERA_DISPLAY_WIDTH, available_width)
        return image_width, image_width / _CAMERA_ASPECT_RATIO

    def _on_window_width_changed(self, width: float) -> None:
        image = getattr(self, "_camera_image", None)
        if image is not None:
            image_width, image_height = self._camera_size_for_width(width)
            image.width = self._ui.Pixel(image_width)
            image.height = self._ui.Pixel(image_height)

        layout_mode = self._layout_mode_for_width(width)
        if layout_mode != getattr(self, "_layout_mode", None):
            self._layout_mode = layout_mode
            if self._window is not None:
                self._window.frame.rebuild()

    def _on_update(self, _event: Any) -> None:
        self.runtime.spin_once()
        self._refresh_demonstration_state()
        frame = self.runtime.camera_frame_rgba()
        now = __import__("time").monotonic()
        target_status, target_pixel = self.runtime.selected_target_pixel(now=now)
        vision_state, vision_overlay = self.runtime.vision_display()
        identity_reader = getattr(
            self.runtime, "display_frame_identity", self.runtime.camera_identity
        )
        camera_identity = identity_reader()
        display_source_reader = getattr(self.runtime, "display_source", None)
        display_source = display_source_reader() if display_source_reader else "rgb"
        video_mode = self.runtime.video_mode()
        display_status = (
            self.runtime.display_status(now=now)
            if hasattr(self.runtime, "display_status")
            else video_mode
        )
        if frame is not None:
            pixels, width, height = frame
            identity = (
                id(pixels),
                width,
                height,
                camera_identity,
                display_status,
                display_source,
                vision_state,
                repr(vision_overlay),
                video_mode,
            )
            if identity != self._last_image_identity:
                display_pixels = pixels.copy() if vision_overlay is not None else pixels
                if vision_overlay is not None and display_source != "tray":
                    try:
                        self._annotate_vision(
                            display_pixels,
                            vision_overlay,
                            display_source=display_source,
                        )
                        if self._annotation_render_available is False:
                            _LOGGER.info("Hub Vision annotation rendering recovered")
                        self._annotation_render_available = True
                    except Exception as error:
                        if self._annotation_render_available is not False:
                            _LOGGER.warning(
                                "Hub Vision annotation unavailable: %s", error
                            )
                            _LOGGER.debug("Hub Vision annotation detail: %s", error)
                        self._annotation_render_available = False
                self._latest_image = display_pixels
                self._image_provider.set_bytes_data(
                    memoryview(display_pixels.reshape(-1)), [width, height]
                )
                self._last_image_source = pixels
                self._last_image_identity = identity
        elif self._last_image_identity is not None:
            blank = np.zeros((1, 1, 4), dtype=np.uint8)
            self._latest_image = blank
            self._image_provider.set_bytes_data(memoryview(blank.reshape(-1)), [1, 1])
            self._last_image_source = None
            self._last_image_identity = None
        if now - self._last_ui_update < 0.1:
            return
        self._last_ui_update = now
        self._refresh_detector_snapshot()
        self._refresh_vision_calibration()
        if self._video_mode_label is not None:
            self._video_mode_label.text = video_mode
        if self._display_status_label is not None:
            self._display_status_label.text = display_status
        if self._target_label is not None:
            vision_detail = ""
            if vision_overlay is not None:
                vision_detail = (
                    f" · target={vision_overlay['target_id']} · frame match=current"
                )
            elif self._annotation_render_available is False:
                vision_detail = " · annotation unavailable"
            elif vision_state in {"TARGET DETECTED", "TARGET LOCKED"}:
                vision_detail = " · annotation suppressed or incompatible"
            elif vision_state in {"stale", "unavailable"}:
                vision_detail = " · annotation suppressed"
            self._target_label.text = (
                f"Vision: {vision_state} · Selected target feedback: {target_status} "
                f"{target_pixel if target_pixel else ''}{vision_detail}"
            )
        self._refresh_operator_request_statuses()
        self._refresh_retry_detection()
        self._refresh_fault_statuses()

    @staticmethod
    def _annotate_target(pixels: Any, target_pixel: tuple[float, float]) -> None:
        height, width = pixels.shape[:2]
        x, y = (int(round(value)) for value in target_pixel)
        radius = _TARGET_MARKER_RADIUS
        left, right = max(0, x - radius), min(width, x + radius + 1)
        top, bottom = max(0, y - radius), min(height, y + radius + 1)
        yy, xx = np.ogrid[top:bottom, left:right]
        distance = (xx - x) ** 2 + (yy - y) ** 2
        ring = (distance <= radius**2) & (distance >= (radius - 5) ** 2)
        pixels[top:bottom, left:right][ring] = _TARGET_MARKER_COLOR

    @staticmethod
    def _annotate_vision(
        pixels: Any,
        metadata: dict[str, Any],
        *,
        display_source: str = "rgb",
    ) -> None:
        import cv2

        color = (32, 255, 96, 255)
        high_contrast = display_source == "depth"
        outline = (12, 12, 12, 255)
        region = metadata.get("object_region")
        if region is not None:
            x, y, width, height = (int(value) for value in region)
            if high_contrast:
                cv2.rectangle(
                    pixels, (x, y), (x + width - 1, y + height - 1), outline, 5
                )
            cv2.rectangle(pixels, (x, y), (x + width - 1, y + height - 1), color, 2)
        support = metadata.get("support_region") or []
        if len(support) >= 3:
            points = np.asarray(support, dtype=np.int32).reshape((-1, 1, 2))
            if high_contrast:
                cv2.polylines(pixels, [points], True, outline, 5)
            cv2.polylines(pixels, [points], True, (255, 192, 32, 255), 2)
        centroid = metadata.get("centroid")
        if centroid is not None:
            point = tuple(int(round(value)) for value in centroid)
            if high_contrast:
                cv2.drawMarker(pixels, point, outline, cv2.MARKER_CROSS, 22, 5)
            cv2.drawMarker(pixels, point, color, cv2.MARKER_CROSS, 18, 2)

    def _build(self) -> None:
        ui = self._ui
        self._operator_request_status_labels.clear()
        self._fault_buttons.clear()
        self._fault_status_labels.clear()
        with (
            self._window.frame,
            ui.ScrollingFrame(
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            ),
        ):
            self._root = ui.VStack(spacing=6)
            with self._root:
                self._build_contents(ui)

    def _build_contents(self, ui: Any) -> None:
        ui.Label("ARM Cell · Simulation Hub")
        with ui.CollapsableFrame("External Handoff Trigger", collapsed=False):
            with ui.VStack(spacing=4):
                ui.Label(
                    "The Hub reports local publication only; owner acceptance and supply state are not shown here.",
                    word_wrap=True,
                )
                ui.Label("Material type")
                self._handoff_material_type_field = ui.StringField()
                self._handoff_material_type_field.model.set_value(
                    self._handoff_default_material_type
                )
                ui.Label("Quantity")
                self._handoff_quantity_field = ui.IntField()
                self._handoff_quantity_field.model.set_value(
                    self._handoff_default_quantity
                )
                ui.Button(
                    "Random Drop",
                    clicked_fn=self._publish_handoff_trigger,
                )
                self._handoff_trigger_status = ui.Label(
                    "No trigger stimulus published.", word_wrap=True
                )

        with ui.CollapsableFrame("Cuboid Demonstration", collapsed=False):
            with ui.VStack(spacing=4):
                ui.Label(
                    "Load one preset during each Play session. Stop, then Play to choose another.",
                    word_wrap=True,
                )
                from scene_builder.cuboid_demonstrations import display_name, scenarios

                self._demonstration_scenarios = scenarios()
                self._demonstration_combo = ui.ComboBox(
                    0,
                    *(
                        f"{display_name(scenario_id, name)} — {description}"
                        for scenario_id, name, description in self._demonstration_scenarios
                    ),
                )
                self._demonstration_button = ui.Button(
                    "Load Demonstration", clicked_fn=self._load_demonstration
                )
                self._demonstration_status = ui.Label(
                    "Waiting for Timeline Play.", word_wrap=True
                )

        image_width, image_height = self._camera_size_for_width(self._window.width)
        self._build_detector_snapshot(ui, image_width, image_height)
        with ui.CollapsableFrame("Live Camera (optional)", collapsed=True):
            self._build_camera_status(ui, image_width, image_height)

        if self._retry_detection_enabled:
            with (
                ui.CollapsableFrame("Developer Tools · Vision only", collapsed=False),
                ui.VStack(spacing=4),
            ):
                ui.Label(
                    "Runs one Vision observation request. It does not resume or alter a Mission.",
                    word_wrap=True,
                )
                self._retry_detection_button = ui.Button(
                    "Retry Detection", clicked_fn=self._request_retry_detection
                )
                self._retry_detection_status = ui.Label(
                    "Waiting for current Mission, readiness, and Vision observations.",
                    word_wrap=True,
                )
                self._refresh_retry_detection()

        for title, owner, scenarios in _FAULT_GROUPS:
            self._build_fault_group(ui, title, owner, scenarios)

        ui.Label("Safety Recovery")
        ui.Label(
            "Acknowledgement and reset are a separate request. Confirm recovery from Safety observations.",
            word_wrap=True,
        )
        ui.Button(
            "Acknowledge & Request Safety Reset",
            clicked_fn=lambda: self._submit_operator_request("safety_reset"),
        )
        self._operator_request_status_labels["safety_reset"] = ui.Label(
            "No Safety reset requested.", word_wrap=True
        )
        ui.Label("Gripper Open")
        ui.Label(
            "Confirm the object is supported or restrained and the release area is clear.",
            word_wrap=True,
        )
        self._gripper_supported_button = ui.Button(
            "Confirm object supported or restrained: No",
            clicked_fn=self._toggle_gripper_supported,
        )
        self._gripper_area_clear_button = ui.Button(
            "Confirm release area clear: No", clicked_fn=self._toggle_gripper_area_clear
        )
        ui.Button("Request Gripper Open", clicked_fn=self._request_gripper_open)
        self._operator_request_status_labels["gripper_open"] = ui.Label(
            "No Gripper Open request submitted.", word_wrap=True
        )
        self._refresh_operator_request_statuses()
        self._refresh_fault_statuses()

    def _build_fault_group(self, ui: Any, title: str, owner: str, scenarios) -> None:
        with (
            ui.CollapsableFrame(title, collapsed=False),
            ui.VStack(spacing=5),
        ):
            for scenario, label, description in scenarios:
                key = (owner, scenario)
                ui.Label(label)
                ui.Label(description, word_wrap=True)
                with ui.VStack(spacing=3):
                    self._fault_buttons[(owner, scenario, True)] = ui.Button(
                        {
                            "e_stop": "Inject",
                            "safety_signal_loss": "Inject 5s",
                            "mislocalize_next": "Inject Next Detection Error",
                        }.get(scenario, "Inject Fault"),
                        clicked_fn=lambda owner=owner, scenario=scenario: (
                            self._set_scenario(owner, scenario, True)
                        ),
                    )
                    if scenario == "e_stop":
                        self._fault_buttons[(owner, scenario, False)] = ui.Button(
                            "Clear",
                            clicked_fn=lambda owner=owner, scenario=scenario: (
                                self._set_scenario(owner, scenario, False)
                            ),
                        )
                self._fault_status_labels[key] = ui.Label(
                    self._fault_local_status.get(key, "No fault request submitted."),
                    word_wrap=True,
                )

    def _build_camera_status(
        self, ui: Any, image_width: float, image_height: float
    ) -> None:
        """Build video and source status in the selected responsive direction."""
        with (
            ui.HStack(spacing=12)
            if self._layout_mode == "wide"
            else ui.VStack(spacing=6)
        ):
            self._camera_image = ui.ImageWithProvider(
                self._image_provider, width=image_width, height=image_height
            )
            with ui.VStack():
                ui.Label("Display source")
                with ui.VStack(spacing=4):
                    self._display_source_buttons["none"] = ui.Button(
                        "Off",
                        clicked_fn=lambda: self._select_display_source("none"),
                    )
                    self._display_source_buttons["rgb"] = ui.Button(
                        "RGB", clicked_fn=lambda: self._select_display_source("rgb")
                    )
                    self._display_source_buttons["depth"] = ui.Button(
                        "Depth Heatmap",
                        clicked_fn=lambda: self._select_display_source("depth"),
                    )
                    self._display_source_buttons["tray"] = ui.Button(
                        "Tray Aligned Depth",
                        clicked_fn=lambda: self._select_display_source("tray"),
                    )
                self._display_status_label = ui.Label("RGB · LIVE", word_wrap=True)
                self._calibration_status_label = ui.Label(
                    "Tray calibration: waiting for camera frames", word_wrap=True
                )
                self._video_mode_label = ui.Label("LIVE", word_wrap=True)
                self._target_label = ui.Label(
                    "Selected target: unavailable until canonical feedback is current",
                    word_wrap=True,
                )
                with (
                    ui.CollapsableFrame("Camera details", collapsed=True),
                    ui.VStack(spacing=2),
                ):
                    ui.Label("RGB: /camera/color/image_raw", word_wrap=True)
                    ui.Label(
                        "Depth: /camera/aligned_depth_to_color/image_raw",
                        word_wrap=True,
                    )
                    ui.Label(
                        "Depth: teal 0.6 m → yellow 1.6 m · magenta invalid",
                        word_wrap=True,
                    )
                    ui.Label(
                        "Tray depth: top-down support-frame Z · -0.02 m → 0.18 m",
                        word_wrap=True,
                    )

    def _build_detector_snapshot(
        self, ui: Any, image_width: float, image_height: float
    ) -> None:
        if self._detector_image is not None:
            image_height_px, image_width_px = self._detector_image.shape[:2]
            image_width = min(_CAMERA_DISPLAY_WIDTH, image_width_px)
            image_height = image_width * image_height_px / image_width_px
        with ui.CollapsableFrame("Vision Detector Snapshot", collapsed=False):
            with ui.VStack(spacing=4):
                self._detector_image_widget = ui.ImageWithProvider(
                    self._detector_image_provider,
                    width=image_width,
                    height=image_height,
                )
                self._detector_event_label = ui.Label(
                    self._detector_event_text, word_wrap=True
                )
                with ui.CollapsableFrame("Snapshot diagnostics", collapsed=True):
                    with ui.ScrollingFrame(
                        height=ui.Pixel(120),
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    ):
                        self._detector_diagnostics_label = ui.Label(
                            self._detector_diagnostics_text,
                            word_wrap=True,
                        )
                self._vision_calibration_button = ui.Button(
                    "Calibrate Vision",
                    clicked_fn=self._request_vision_calibration,
                )
                self._vision_calibration_status = ui.Label(
                    "Vision calibration has not been requested.", word_wrap=True
                )
        self._refresh_detector_snapshot()
        self._refresh_vision_calibration()

    def _request_vision_calibration(self) -> None:
        try:
            self.runtime.request_vision_calibration()
        except (OSError, RuntimeError, ValueError, KeyError, AttributeError) as error:
            if self._vision_calibration_status is not None:
                self._vision_calibration_status.text = (
                    f"Vision calibration request failed: {error}"
                )
        self._refresh_vision_calibration()

    def _refresh_vision_calibration(self) -> None:
        button = getattr(self, "_vision_calibration_button", None)
        label = getattr(self, "_vision_calibration_status", None)
        if button is None and label is None:
            return
        result_reader = getattr(self.runtime, "vision_calibration_result", None)
        result = result_reader() if result_reader is not None else {}
        state = str(result.get("state", "unavailable"))
        if button is not None:
            button.enabled = state != "pending"
        if label is not None:
            if state == "ready":
                label.text = "Vision calibration has not been requested."
            elif state == "pending":
                label.text = "Vision calibration request pending."
            else:
                message = str(result.get("message", ""))
                label.text = f"Vision calibration {state}." + (
                    f" {message}" if message else ""
                )

    def _refresh_detector_snapshot(self) -> None:
        retry_reader = getattr(self.runtime, "retry_detection_result", None)
        retry_result = retry_reader() if retry_reader is not None else {}
        snapshot = retry_result.get("snapshot")
        if snapshot is None:
            snapshot_reader = getattr(self.runtime, "vision_detector_snapshot", None)
            snapshot = snapshot_reader() if snapshot_reader is not None else None
        if (
            snapshot is None
            or snapshot.get("snapshot_id") == self._detector_snapshot_id
        ):
            return
        snapshot_id = str(snapshot.get("snapshot_id", ""))
        self._detector_snapshot_id = snapshot_id
        metadata: dict[str, Any] = {}
        metadata_error = None
        try:
            import json

            metadata = json.loads(snapshot.get("metadata_json", ""))
            if not isinstance(metadata, dict):
                raise ValueError("Snapshot metadata root is not an object")
        except (TypeError, ValueError) as error:
            metadata_error = str(error)
        pixels = None
        image_error = None
        try:
            import cv2

            encoded = np.frombuffer(snapshot.get("image_data", b""), dtype=np.uint8)
            decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR) if encoded.size else None
            if decoded is not None:
                pixels = cv2.cvtColor(decoded, cv2.COLOR_BGR2RGBA)
        except Exception as error:
            image_error = error
        image_available_from_png = pixels is not None
        previous_image_state = self._detector_image_decode_available
        if previous_image_state != image_available_from_png:
            if image_available_from_png:
                if previous_image_state is False:
                    _LOGGER.info("Hub Vision Snapshot image decoding recovered")
            else:
                _LOGGER.warning(
                    "Hub Vision Snapshot image unavailable: %s",
                    image_error or "Snapshot contains no decodable image",
                )
                if image_error is not None:
                    _LOGGER.debug(
                        "Hub Vision Snapshot image decode detail: %s", image_error
                    )
            self._detector_image_decode_available = image_available_from_png
        if pixels is not None:
            height, width = pixels.shape[:2]
            self._detector_image = pixels
            self._detector_image_provider.set_bytes_data(
                memoryview(pixels.reshape(-1)), [width, height]
            )
            if self._detector_image_widget is not None:
                view_width = min(_CAMERA_DISPLAY_WIDTH, width)
                self._detector_image_widget.width = self._ui.Pixel(view_width)
                self._detector_image_widget.height = self._ui.Pixel(
                    view_width * height / width
                )
        else:
            self._detector_image = None
            blank = np.zeros((1, 1, 4), dtype=np.uint8)
            self._detector_image_provider.set_bytes_data(
                memoryview(blank.reshape(-1)), [1, 1]
            )
        event = metadata.get("event_kind", snapshot.get("event_kind", "unknown"))
        image_available = bool(metadata.get("image_available", pixels is not None))
        source_available = bool(metadata.get("source_image_rendered", image_available))
        status = (
            "source image available" if source_available else "source image unavailable"
        )
        event_text = (
            f"{event} · revision {snapshot.get('revision', 'unknown')} · {status}"
        )
        if snapshot.get("transfer_error"):
            event_text += " · snapshot image transfer unavailable"
        if not image_available or pixels is None:
            event_text += " · PNG unavailable"
        diagnostic_lines = []
        diagnostic_lines.extend(
            (
                f"snapshot_id: {snapshot_id}",
                f"acquisition_id: {snapshot.get('acquisition_id', 'unknown')}",
            )
        )
        for key in ("annotation", "calibration_summary"):
            if key in metadata and metadata[key] is not None:
                diagnostic_lines.append(f"{key}: {metadata[key]}")
        diagnostic_record_error = None
        diagnostic_record: dict[str, Any] = {}
        try:
            diagnostic_record = json.loads(snapshot.get("diagnostic_record_json", ""))
            if not isinstance(diagnostic_record, dict):
                raise ValueError("Vision Diagnostic Record root is not an object")
        except (TypeError, ValueError) as error:
            diagnostic_record_error = str(error)
        schema_version = metadata.get("schema_version", 0)
        current_schema = isinstance(schema_version, int) and schema_version >= 11
        if diagnostic_record:
            for key in (
                "strategy",
                "terminal_result",
                "single_observation",
                "acquisition_stability",
                "applied_profile",
                "support_calibration",
                "cuboid_height_patches",
                "cuboid_stages",
                "replay_evidence",
            ):
                if key in diagnostic_record and diagnostic_record[key] is not None:
                    diagnostic_lines.append(f"{key}: {diagnostic_record[key]}")
        else:
            # Older Snapshot schemas carried these optional values in metadata.
            for key in (
                "processing_stage",
                "terminal_result",
                "observation",
                "stability",
            ):
                if key in metadata and metadata[key] is not None:
                    diagnostic_lines.append(f"{key}: {metadata[key]}")
            if (
                diagnostic_record_error
                and snapshot.get("acquisition_id")
                and int(snapshot.get("event_kind", -1)) in (2, 3)
                and current_schema
            ):
                diagnostic_lines.append("Vision Diagnostic Record unavailable")
        if metadata_error:
            diagnostic_lines = ["Vision metadata unavailable (invalid JSON)"]
        elif not diagnostic_lines:
            diagnostic_lines.append("No diagnostic fields provided by Vision")
        metadata_available = metadata_error is None
        previous_metadata_state = self._detector_metadata_available
        if previous_metadata_state != metadata_available:
            if metadata_available:
                if previous_metadata_state is False:
                    _LOGGER.info("Hub Vision Snapshot metadata parsing recovered")
            else:
                _LOGGER.warning(
                    "Hub Vision Snapshot metadata unavailable: %s", metadata_error
                )
                _LOGGER.debug(
                    "Hub Vision Snapshot metadata parse detail: %s", metadata_error
                )
            self._detector_metadata_available = metadata_available
        if self._detector_event_label is not None:
            self._detector_event_text = event_text
            self._detector_event_label.text = self._detector_event_text
        else:
            self._detector_event_text = event_text
        if self._detector_diagnostics_label is not None:
            self._detector_diagnostics_text = "\n".join(diagnostic_lines)
            self._detector_diagnostics_label.text = self._detector_diagnostics_text
        else:
            self._detector_diagnostics_text = "\n".join(diagnostic_lines)

    def _select_display_source(self, source_id: str) -> None:
        self.runtime.set_display_source(source_id)

    def _retry_detection_block_reason(self) -> str:
        try:
            import omni.timeline

            if not omni.timeline.get_timeline_interface().is_playing():
                return "Timeline must be running"
        except (ImportError, AttributeError, RuntimeError):
            return "Timeline state is unavailable"
        eligible, reason = self.runtime.retry_detection_eligibility()
        return "Available" if eligible else reason

    def _refresh_retry_detection(self) -> None:
        button = getattr(self, "_retry_detection_button", None)
        label = getattr(self, "_retry_detection_status", None)
        if button is None and label is None:
            return
        reason = self._retry_detection_block_reason()
        result_reader = getattr(self.runtime, "retry_detection_result", None)
        result = result_reader() if result_reader is not None else {}
        if result.get("state") == "pending":
            reason = "Requesting Vision detection"
        if button is not None:
            button.enabled = reason == "Available"
        if label is not None:
            state = str(result.get("state", "unavailable"))
            if state != "pending" and result.get("request_id"):
                snapshot = result.get("snapshot") or {}
                label.text = (
                    f"{state.upper()} · request={result['request_id']} · "
                    f"result={result.get('result_code', 'none')} · "
                    f"acquisition={result.get('acquisition_id') or 'none'} · "
                    f"snapshot={snapshot.get('snapshot_id', 'none')} · "
                    f"{result.get('diagnostic_detail', '')}"
                    + (f" · unavailable: {reason}" if reason != "Available" else "")
                )
            else:
                label.text = reason

    def _request_retry_detection(self) -> None:
        if not self._retry_detection_enabled:
            return
        reason = self._retry_detection_block_reason()
        if reason != "Available":
            if self._retry_detection_status is not None:
                self._retry_detection_status.text = reason
            return
        try:
            self.runtime.request_retry_detection(self._retry_detection_target_id)
        except (OSError, RuntimeError, ValueError, KeyError, AttributeError) as error:
            if self._retry_detection_status is not None:
                self._retry_detection_status.text = (
                    f"Retry Detection request failed: {error}"
                )
        self._refresh_retry_detection()

    def _submit_operator_request(self, action: str) -> None:
        method_name = {"safety_reset": "acknowledge_and_reset"}[action]
        try:
            request_id, _future = getattr(self.runtime, method_name)()
            self._operator_request_ids[action] = request_id
            self._refresh_operator_request_statuses()
        except (OSError, RuntimeError, ValueError, KeyError) as error:
            self._operator_request_ids.pop(action, None)
            label = self._operator_request_status_labels.get(action)
            if label is not None:
                label.text = f"Request failed: {error}"

    def _publish_handoff_trigger(self) -> None:
        label = self._handoff_trigger_status
        try:
            material_type = (
                self._handoff_material_type_field.model.get_value_as_string().strip()
            )
            quantity = self._handoff_quantity_field.model.get_value_as_int()
            delivery_id = self.runtime.publish_handoff_trigger(material_type, quantity)
            label.text = (
                f"Trigger stimulus published · delivery_id={delivery_id} · "
                f"type={material_type} · quantity={quantity}. "
                "Owner acceptance, supply, and readiness are not displayed in this Hub."
            )
        except (OSError, RuntimeError, ValueError, KeyError, AttributeError) as error:
            if label is not None:
                label.text = f"Trigger stimulus failed: {error}"

    def _refresh_demonstration_state(self, *, force: bool = False) -> None:
        if getattr(self, "_demonstration_button", None) is None:
            return
        now = __import__("time").monotonic()
        if not force and now - self._last_demonstration_state_refresh < 0.5:
            return
        self._last_demonstration_state_refresh = now
        try:
            import omni.timeline

            playing = omni.timeline.get_timeline_interface().is_playing()
            from scene_builder.cuboid_demonstration import (
                demonstration_loaded,
                load_unavailable_reason,
            )

            self._demonstration_loaded_this_session |= demonstration_loaded()
            unavailable = load_unavailable_reason()
            self._demonstration_button.enabled = playing and unavailable is None
            if self._demonstration_combo is not None:
                self._demonstration_combo.enabled = (
                    not self._demonstration_loaded_this_session
                )
            if (
                self._demonstration_status is not None
                and not self._demonstration_loaded_this_session
            ):
                if not playing:
                    self._demonstration_status.text = "Waiting for Timeline Play."
                elif unavailable:
                    self._demonstration_status.text = f"Load unavailable: {unavailable}"
                else:
                    self._demonstration_status.text = (
                        "Ready to load once in this Play session."
                    )
        except (ImportError, AttributeError, RuntimeError):
            self._demonstration_button.enabled = False

    def _load_demonstration(self) -> None:
        label = self._demonstration_status
        try:
            import omni.timeline

            if not omni.timeline.get_timeline_interface().is_playing():
                raise RuntimeError("Load Demonstration requires Timeline Play")
            selected = (
                self._demonstration_combo.model.get_item_value_model().get_value_as_int()
            )
            scenario_id, name, _description = self._demonstration_scenarios[selected]
            from scene_builder.cuboid_demonstration import load_demonstration

            count = load_demonstration(scenario_id)
            self._demonstration_loaded_this_session = True
            if label is not None:
                label.text = f"Loaded {name}: {count} RawParts. Use Retry Detection and Snapshot to observe."
        except (
            RuntimeError,
            ValueError,
            IndexError,
            AttributeError,
            ImportError,
        ) as error:
            if label is not None:
                label.text = f"Load failed: {error}"
        self._refresh_demonstration_state(force=True)

    def _on_timeline_event(self, event: Any) -> None:
        try:
            import omni.timeline

            stop_type = omni.timeline.TimelineEventType.STOP
            is_stop = int(getattr(event, "type", -1)) == int(stop_type)
        except (ImportError, AttributeError, TypeError, ValueError):
            is_stop = getattr(event, "type", None) == "STOP"
        if not is_stop:
            return
        self._demonstration_loaded_this_session = False
        self._refresh_demonstration_state(force=True)
        self.runtime.clear_tf_buffer()
        for scenario in ("e_stop", "safety_signal_loss"):
            self._set_scenario("external_mock", scenario, False)
        disarm = getattr(self.runtime, "set_vision_mislocalize", None)
        if callable(disarm):
            disarm(False)

    def _refresh_operator_request_statuses(self) -> None:
        request_state = getattr(self.runtime, "request_state", {})
        for action, label in self._operator_request_status_labels.items():
            request_id = self._operator_request_ids.get(action)
            if request_id is None:
                continue
            state = request_state.get(request_id, "pending")
            if state == "pending":
                label.text = "Request pending. Check the canonical observations below."
            elif action == "safety_reset" and state == "applied_pending_owner_state":
                detail = getattr(self.runtime, "request_detail", {}).get(request_id, "")
                label.text = (
                    "Safety accepted the reset request; recovery is not confirmed. "
                    "Check Safety state and readiness below."
                    + (f" Safety response: {detail}" if detail else "")
                )
            elif action == "safety_reset" and state == "rejected":
                detail = getattr(self.runtime, "request_detail", {}).get(request_id, "")
                label.text = (
                    "Safety rejected the reset request. Canonical Safety state "
                    "remains the outcome source."
                    + (f" Safety response: {detail}" if detail else "")
                )
            elif action == "gripper_open":
                details = getattr(self.runtime, "request_detail", {})
                label.text = (
                    f"Motion owner request {state}; command result: "
                    f"{details.get(request_id + ':execution', 'not confirmed')}. "
                    "Check fresh Motion holding state for RELEASED."
                )
            else:
                label.text = (
                    f"Request {state}. Canonical state remains the outcome source."
                )

    def _request_gripper_open(self) -> None:
        try:
            request_id, _future = self.runtime.request_gripper_open(
                self._gripper_supported_confirmed, self._gripper_area_clear_confirmed
            )
            self._operator_request_ids["gripper_open"] = request_id
        except (OSError, RuntimeError, ValueError, KeyError) as error:
            self._operator_request_status_labels["gripper_open"].text = (
                f"Request failed: {error}"
            )
        self._refresh_operator_request_statuses()

    def _toggle_gripper_supported(self) -> None:
        self._gripper_supported_confirmed = not self._gripper_supported_confirmed
        self._gripper_supported_button.text = (
            "Confirm object supported or restrained: "
            + ("Yes" if self._gripper_supported_confirmed else "No")
        )

    def _toggle_gripper_area_clear(self) -> None:
        self._gripper_area_clear_confirmed = not self._gripper_area_clear_confirmed
        self._gripper_area_clear_button.text = "Confirm release area clear: " + (
            "Yes" if self._gripper_area_clear_confirmed else "No"
        )

    def _set_scenario(self, owner: str, scenario: str, active: bool) -> None:
        key = (owner, scenario)
        if scenario == "mislocalize_next":
            try:
                request_id, _future = self.runtime.set_vision_mislocalize(active)
                self._fault_request_ids[key] = request_id
            except (OSError, RuntimeError, ValueError, KeyError) as error:
                self._fault_local_status[key] = f"Request failed: {error}"
            self._refresh_fault_statuses()
            return
        if owner == "external_mock":
            try:
                request_id, _future = self.runtime.set_external_fault(scenario, active)
                self._fault_request_ids[key] = request_id
                self._refresh_fault_statuses()
            except (OSError, RuntimeError, ValueError, KeyError) as error:
                self._fault_request_ids.pop(key, None)
                self._fault_local_status[key] = f"Request failed: {error}"
                self._refresh_fault_statuses()
            return

        try:
            applied = self.runtime.set_scenario_active(owner, scenario, active)
            if applied:
                self._scenario_active[key] = active
                self._fault_local_status[key] = (
                    "Owner adapter accepted the intent; confirm the result in "
                    "canonical observations."
                )
            else:
                self._fault_local_status[key] = (
                    "Owner adapter unavailable; applied intent is unchanged."
                )
        except (ValueError, RuntimeError) as error:
            self._fault_local_status[key] = f"Request failed: {error}"
        self._refresh_fault_statuses()

    def _refresh_fault_statuses(self) -> None:
        request_state = getattr(self.runtime, "request_state", {})
        fault_active = getattr(self.runtime, "fault_active", {})
        for key, label in self._fault_status_labels.items():
            owner, scenario = key
            active = (
                bool(fault_active.get(scenario, False))
                if owner == "external_mock"
                else self._scenario_active.get(key, False)
            )
            pending = False
            if owner == "external_mock":
                request_id = self._fault_request_ids.get(key)
                state = request_state.get(request_id, "") if request_id else ""
                pending = state == "pending"
                if not request_id:
                    label.text = self._fault_local_status.get(
                        key,
                        "No fault request submitted. Confirm effects in observations.",
                    )
                elif not state:
                    label.text = (
                        "No fault request submitted. Confirm effects in observations."
                    )
                elif pending:
                    label.text = (
                        "Fault request pending. Confirm effects in observations."
                    )
                elif state.startswith("applied:"):
                    operation = "injection" if active else "clear"
                    label.text = (
                        f"VDA mock applied the {operation} request ({state[8:]}). "
                        "This does not confirm the system fault state; check observations."
                    )
                else:
                    label.text = (
                        f"Fault request {state}. Applied intent is unchanged; "
                        "check canonical observations."
                    )
            else:
                label.text = self._fault_local_status.get(
                    key, "No fault request submitted. Confirm effects in observations."
                )

            inject = self._fault_buttons.get((owner, scenario, True))
            clear = self._fault_buttons.get((owner, scenario, False))
            if inject is not None:
                inject.enabled = not active and not pending
            if clear is not None:
                clear.enabled = active and not pending

    def on_shutdown(self) -> None:
        timeline_subscription = getattr(self, "_timeline_subscription", None)
        unsubscribe = getattr(timeline_subscription, "unsubscribe", None)
        if callable(unsubscribe):
            try:
                unsubscribe()
            except Exception:
                _LOGGER.exception(
                    "failed to unsubscribe Simulation UI Hub Timeline callback"
                )
        self._timeline_subscription = None
        update_subscription = getattr(self, "_timer", None)
        unsubscribe = getattr(update_subscription, "unsubscribe", None)
        if callable(unsubscribe):
            try:
                unsubscribe()
            except Exception:
                _LOGGER.exception(
                    "failed to unsubscribe Simulation UI Hub update callback"
                )
        self._timer = None
        if getattr(self, "runtime", None) is not None:
            for scenario in ("e_stop", "safety_signal_loss"):
                try:
                    self.runtime.set_external_fault(scenario, False)
                except Exception:
                    _LOGGER.warning("could not cancel %s during Hub shutdown", scenario)
            try:
                self.runtime.set_vision_mislocalize(False)
            except Exception:
                _LOGGER.warning("could not disarm Vision injection during Hub shutdown")
            for (owner, scenario), active in tuple(
                getattr(self, "_scenario_active", {}).items()
            ):
                if active:
                    try:
                        self.runtime.set_scenario_active(owner, scenario, False)
                    except Exception:
                        pass
            self.runtime.close()
            self.runtime = None
        window_menu = getattr(self, "_window_menu", None)
        if window_menu is not None:
            window_menu.menu_shutdown()
            self._window_menu = None
        self._window = None


def extension_class() -> type[SimulationUIHubExtension]:
    return SimulationUIHubExtension
