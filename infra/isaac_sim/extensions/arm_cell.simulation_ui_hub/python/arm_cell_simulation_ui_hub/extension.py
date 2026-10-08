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
        "Vision Faults",
        "rgbd_source",
        (
            (
                "target_unavailable",
                "Camera Unavailable",
                "Stops color image publication so Vision has no current camera frame.",
            ),
            (
                "target_stale",
                "Stale Observation",
                "Replays a real image with an older source timestamp.",
            ),
            (
                "target_invalid",
                "Invalid Observation",
                "Changes the camera optical frame metadata to an incompatible value.",
            ),
        ),
    ),
    (
        "Gripper Faults",
        "motion_backend",
        (
            (
                "holding_unknown",
                "Holding State Unknown",
                "Injects UNKNOWN through the normal gripper holding observation path.",
            ),
        ),
    ),
    (
        "Safety & External Faults",
        "external_mock",
        (
            (
                "e_stop",
                "E-Stop",
                "Publishes an emergency stop condition; clearing it does not reset the Safety latch.",
            ),
            (
                "premature_undock",
                "Premature Undock",
                "Reports the AMR as undocked while driving; clearing does not redock it.",
            ),
            (
                "packml_abort",
                "PackML Abort",
                "Publishes PackML ABORTED while injected; clearing restores fresh mock state.",
            ),
            (
                "communication_loss",
                "Communication Loss",
                "Suppresses required external-state publishers until cleared.",
            ),
            (
                "communication_degradation",
                "Communication Degradation",
                "Adds bounded delay; normal motion limits return only after Safety rechecks fresh state.",
            ),
            (
                "invalid_state",
                "Invalid External State",
                "Publishes external state as UNKNOWN and invalid.",
            ),
            (
                "material_handoff_failure",
                "Material Handoff Failure",
                "Fails the active handoff; clearing does not undo a terminal failed handoff.",
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

        try:
            import carb

            freshness_default = carb.settings.get_settings().get(
                "/exts/arm_cell.simulation_ui_hub/observation_freshness_seconds"
            )
            annotation_tolerance = carb.settings.get_settings().get(
                "/exts/arm_cell.simulation_ui_hub/annotation_source_tolerance_seconds"
            )
            if freshness_default is None:
                freshness_default = DEFAULT_OBSERVATION_FRESHNESS_SECONDS
        except (ImportError, AttributeError):
            freshness_default = DEFAULT_OBSERVATION_FRESHNESS_SECONDS
            annotation_tolerance = DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS
        self.runtime = HubRuntime(
            default_observation_freshness_seconds=float(freshness_default),
            default_annotation_source_tolerance_seconds=float(
                annotation_tolerance
                if annotation_tolerance is not None
                else DEFAULT_ANNOTATION_SOURCE_TOLERANCE_SECONDS
            ),
        )
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
        self._last_image_source: Any | None = None
        self._last_image_identity: tuple[Any, ...] | None = None
        self._last_ui_update = 0.0
        self._annotation_render_available: bool | None = None
        self._video_mode_label: Any | None = None
        self._display_status_label: Any | None = None
        self._display_source_buttons: dict[str, Any] = {}
        self._observation_labels: list[tuple[str, Any]] = []
        self._target_label: Any | None = None
        self._operator_request_status_labels: dict[str, Any] = {}
        self._operator_request_ids: dict[str, str] = {}
        self._fault_buttons: dict[tuple[str, str, bool], Any] = {}
        self._fault_status_labels: dict[tuple[str, str], Any] = {}
        self._fault_request_ids: dict[tuple[str, str], str] = {}
        self._fault_local_status: dict[tuple[str, str], str] = {}
        self._scenario_active: dict[tuple[str, str], bool] = {}
        self._timer = None
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
                if vision_overlay is not None:
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
                            _LOGGER.warning("Hub Vision annotation unavailable")
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
        for name, label in self._observation_labels:
            label.text = self.runtime.observation_text(name, now=now)
        if self._video_mode_label is not None:
            self._video_mode_label.text = video_mode
        if self._display_status_label is not None:
            self._display_status_label.text = display_status
        if self._target_label is not None:
            vision_detail = ""
            if vision_overlay is not None:
                yaw_status = (
                    "available" if vision_overlay["yaw_available"] else "unavailable"
                )
                vision_detail = (
                    f" · target={vision_overlay['target_id']} · frame match=current "
                    f"· yaw={yaw_status}"
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
        self._observation_labels.clear()
        self._operator_request_status_labels.clear()
        self._fault_buttons.clear()
        self._fault_status_labels.clear()
        with self._window.frame, ui.ScrollingFrame(
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
        ):
            self._root = ui.VStack(spacing=6)
            with self._root:
                self._build_contents(ui)

    def _build_contents(self, ui: Any) -> None:
        ui.Label("ARM Cell · Simulation Hub")
        image_width, image_height = self._camera_size_for_width(self._window.width)
        self._build_camera_status(ui, image_width, image_height)

        with ui.CollapsableFrame("System Observations", collapsed=False):
            with ui.VStack(spacing=3):
                self._build_observations(ui)

        ui.Label("Material Supply")
        ui.Button(
            "Request Material",
            clicked_fn=lambda: self._submit_operator_request("material"),
        )
        self._operator_request_status_labels["material"] = ui.Label(
            "No material request submitted.", word_wrap=True
        )

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
        self._refresh_operator_request_statuses()
        self._refresh_fault_statuses()

    def _build_observations(self, ui: Any) -> None:
        now = __import__("time").monotonic()
        for name in (
            "safety",
            "motion",
            "amr",
            "handoff",
            "readiness",
            "packml",
            "execute_cycle",
        ):
            self._observation_labels.append(
                (
                    name,
                    ui.Label(
                        self.runtime.observation_text(name, now=now), word_wrap=True
                    ),
                )
            )

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
                        "Inject Fault",
                        clicked_fn=lambda owner=owner, scenario=scenario: self._set_scenario(
                            owner, scenario, True
                        ),
                    )
                    self._fault_buttons[(owner, scenario, False)] = ui.Button(
                        "Clear Fault",
                        clicked_fn=lambda owner=owner, scenario=scenario: self._set_scenario(
                            owner, scenario, False
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
                    self._display_source_buttons["rgb"] = ui.Button(
                        "RGB", clicked_fn=lambda: self._select_display_source("rgb")
                    )
                    self._display_source_buttons["depth"] = ui.Button(
                        "Depth Heatmap",
                        clicked_fn=lambda: self._select_display_source("depth"),
                    )
                self._display_status_label = ui.Label("RGB · LIVE", word_wrap=True)
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

    def _select_display_source(self, source_id: str) -> None:
        self.runtime.set_display_source(source_id)

    def _submit_operator_request(self, action: str) -> None:
        method_name = {
            "material": "request_material",
            "safety_reset": "acknowledge_and_reset",
        }[action]
        try:
            request_id, _future = getattr(self.runtime, method_name)()
            self._operator_request_ids[action] = request_id
            self._refresh_operator_request_statuses()
        except (OSError, RuntimeError, ValueError, KeyError) as error:
            self._operator_request_ids.pop(action, None)
            label = self._operator_request_status_labels.get(action)
            if label is not None:
                label.text = f"Request failed: {error}"

    def _refresh_operator_request_statuses(self) -> None:
        request_state = getattr(self.runtime, "request_state", {})
        for action, label in self._operator_request_status_labels.items():
            request_id = self._operator_request_ids.get(action)
            if request_id is None:
                continue
            state = request_state.get(request_id, "pending")
            if state == "pending":
                label.text = "Request pending. Check the canonical observations below."
            elif action == "material" and state == "accepted":
                label.text = (
                    "Integration accepted the request; confirm handoff and readiness "
                    "from the canonical observations."
                )
            elif action == "safety_reset" and state == "applied_pending_owner_state":
                label.text = (
                    "Safety accepted the reset request; recovery is not confirmed. "
                    "Check Safety state and readiness below."
                )
            else:
                label.text = (
                    f"Request {state}. Canonical state remains the outcome source."
                )

    def _set_scenario(self, owner: str, scenario: str, active: bool) -> None:
        key = (owner, scenario)
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
        if getattr(self, "runtime", None) is not None:
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
        self._timer = None


def extension_class() -> type[SimulationUIHubExtension]:
    return SimulationUIHubExtension
