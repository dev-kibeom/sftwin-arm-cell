import logging
import math
import json

from graph_builder.graph_contract import GRAPH_PATH

LOGGER = logging.getLogger(__name__)


class SimGripperAdapter:
    """
    SF-Twin simulation-side logical gripper adapter.

    Boundary:
        ROS2 /gripper/command
            ↓
        ActionGraph ROS2Subscriber
            ↓
        outputs:data  [width in mm]
            ↓
        SimGripperAdapter
            ↓
        Robotiq2F85Actuator

    This class deliberately has no rclpy dependency.

    ROS communication belongs to Isaac's ROS2 bridge.
    The actuator remains ROS-agnostic.
    """

    DEFAULT_GRAPH_PATH = GRAPH_PATH
    DEFAULT_NODE_NAME = "SubscribeGripperCommand"
    DEFAULT_STATUS_NODE_NAME = "PublishGripperStatus"

    MIN_WIDTH_MM = 0.0
    MAX_WIDTH_MM = 85.0

    def __init__(
        self,
        gripper,
        graph_path=None,
        subscriber_node=None,
        move_duration=None,
        verbose=False,
        controller=None,
    ):
        self.gripper = gripper
        self.graph_path = graph_path or self.DEFAULT_GRAPH_PATH
        self.subscriber_node = subscriber_node or self.DEFAULT_NODE_NAME
        self.move_duration = move_duration
        self.verbose = verbose
        if controller is None:
            import omni.graph.core as og

            controller = og.Controller
        self._controller = controller

        self._data_attribute = None
        self._generation_attribute = None
        self._close_generation_attribute = None
        self._status_attribute = None
        self._capture_reference_attribute = None
        self._close_id_attribute = None
        self._applied_capture_reference_id = None
        self._last_capture_reference_packet = None
        self._last_command_width_mm = None
        self._last_command_generation = None
        self._last_close_generation = None
        self._last_command_is_closing = None

        self._resolve_attribute()
        self._resolve_generation_attribute()
        self._resolve_close_generation_attribute()
        self._resolve_status_attribute()
        self._resolve_capture_reference_attribute()

        if self.verbose:
            LOGGER.debug("Simulation gripper command adapter ready")

    # ------------------------------------------------------------------
    # OmniGraph access
    # ------------------------------------------------------------------

    def _attribute_path(self):
        return f"{self.graph_path}/{self.subscriber_node}.outputs:data"

    def _resolve_attribute(self):
        """
        Generic ROS2Subscriber creates message outputs dynamically after
        messagePackage/messageName configuration.

        Resolve lazily so Script Editor execution order is less fragile.
        """
        path = self._attribute_path()

        attr = self._controller.attribute(path)

        if attr is None or not attr.is_valid():
            self._data_attribute = None
            return False

        self._data_attribute = attr
        return True

    def _status_attribute_path(self):
        return f"{self.graph_path}/{self.DEFAULT_STATUS_NODE_NAME}.inputs:data"

    def _resolve_status_attribute(self):
        attr = self._controller.attribute(self._status_attribute_path())
        if attr is None or not attr.is_valid():
            self._status_attribute = None
            return False
        self._status_attribute = attr
        return True

    def _resolve_capture_reference_attribute(self):
        path = f"{self.graph_path}/SubscribeGripperCaptureReference.outputs:data"
        attr = self._controller.attribute(path)
        if attr is None or not attr.is_valid():
            self._capture_reference_attribute = None
            return False
        self._capture_reference_attribute = attr
        return True

    def read_capture_reference(self):
        if self._capture_reference_attribute is None:
            if not self._resolve_capture_reference_attribute():
                return None
        try:
            raw = self._controller.get(self._capture_reference_attribute)
            packet = json.loads(raw)
        except Exception:
            self._capture_reference_attribute = None
            return None
        if not isinstance(packet, dict) or not isinstance(
            packet.get("motion_goal_uuid"), str
        ):
            self._applied_capture_reference_id = None
            return None
        if packet.get("invalidate") is True:
            signature = (packet["motion_goal_uuid"], True)
            if signature == self._last_capture_reference_packet:
                return None
            self._last_capture_reference_packet = signature
            if packet["motion_goal_uuid"] == self._applied_capture_reference_id:
                self._applied_capture_reference_id = None
            return packet
        try:
            values = tuple(
                float(value)
                for value in (
                    *packet["position"],
                    *packet["orientation_xyzw"],
                    packet["expected_contact_width_mm"],
                    packet["orientation_symmetry_order"],
                )
            )
        except (KeyError, TypeError, ValueError):
            return None
        if len(values) != 9 or not all(math.isfinite(value) for value in values):
            return None
        if values[7] <= 0.0 or values[8] not in (0.0, 1.0, 2.0, 4.0):
            self._applied_capture_reference_id = None
            return None
        signature = (
            packet["motion_goal_uuid"],
            False,
            packet.get("frame_id"),
            values,
        )
        if signature == self._last_capture_reference_packet:
            return None
        self._last_capture_reference_packet = signature
        return {
            "motion_goal_uuid": packet["motion_goal_uuid"],
            "frame_id": packet.get("frame_id"),
            "values": values,
        }

    def _read_close_execution_id(self):
        if self._close_id_attribute is None:
            attr = self._controller.attribute(
                f"{self.graph_path}/SubscribeGripperClose.outputs:data"
            )
            if attr is None or not attr.is_valid():
                return None
            self._close_id_attribute = attr
        try:
            return str(self._controller.get(self._close_id_attribute) or "")
        except Exception:
            self._close_id_attribute = None
            return None

    def read_close_execution_id(self):
        return self._read_close_execution_id()

    def mark_capture_reference_applied(self, execution_id):
        self._applied_capture_reference_id = execution_id

    def _resolve_generation_attribute(self):
        path = f"{self.graph_path}/GripperCommandGeneration.outputs:count"
        attr = self._controller.attribute(path)
        if attr is None or not attr.is_valid():
            self._generation_attribute = None
            return False
        self._generation_attribute = attr
        return True

    def _read_width_mm(self):
        if self._data_attribute is None:
            if not self._resolve_attribute():
                return None

        try:
            value = self._controller.get(self._data_attribute)
        except Exception:
            # Graph may have been rebuilt since this adapter was created.
            self._data_attribute = None
            return None

        if value is None:
            return None

        try:
            value = float(value)
        except (TypeError, ValueError):
            return None

        if not math.isfinite(value):
            return None

        return value

    def _read_command_generation(self):
        if self._generation_attribute is None:
            if not self._resolve_generation_attribute():
                return None
        try:
            value = self._controller.get(self._generation_attribute)
        except Exception:
            self._generation_attribute = None
            return None
        try:
            generation = int(value)
        except (TypeError, ValueError):
            return None
        return generation if generation >= 0 else None

    def _resolve_close_generation_attribute(self):
        path = f"{self.graph_path}/GripperCloseGeneration.outputs:count"
        attr = self._controller.attribute(path)
        if attr is None or not attr.is_valid():
            self._close_generation_attribute = None
            return False
        self._close_generation_attribute = attr
        return True

    def _read_close_generation(self):
        if self._close_generation_attribute is None:
            if not self._resolve_close_generation_attribute():
                return None
        try:
            value = self._controller.get(self._close_generation_attribute)
            generation = int(value)
        except Exception:
            self._close_generation_attribute = None
            return None
        return generation if generation >= 0 else None

    def prime_command_generation(self):
        """Treat the retained graph value as a baseline, not a new command."""
        generation = self._read_command_generation()
        if generation is None:
            return False
        self._last_command_generation = generation
        self._last_close_generation = self._read_close_generation()
        return True

    # ------------------------------------------------------------------
    # Command adaptation
    # ------------------------------------------------------------------

    @classmethod
    def _clamp_width(cls, width_mm):
        return max(
            cls.MIN_WIDTH_MM,
            min(cls.MAX_WIDTH_MM, float(width_mm)),
        )

    def update(self):
        """
        Read the latest logical width command from ActionGraph.

        Returns:
            True  if a new command was applied.
            False otherwise.
        """
        close_generation = self._read_close_generation()
        if (
            close_generation is not None
            and self._last_close_generation is not None
            and close_generation != self._last_close_generation
        ):
            close_execution_id = self._read_close_execution_id()
            if close_execution_id is None or (
                close_execution_id != (self._applied_capture_reference_id or "")
            ):
                self._last_close_generation = close_generation
                self._last_command_is_closing = False
                return False
            self.gripper.close(duration=self.move_duration)
            self._last_close_generation = close_generation
            self._last_command_width_mm = None
            self._last_command_is_closing = True
            return True

        requested_width = self._read_width_mm()

        if requested_width is None:
            return False

        generation = self._read_command_generation()

        # The Counter is driven by ROS2Subscriber.outputs:execOut, which fires
        # only when a new message is received.  The retained data value is not
        # a command identity.
        if generation is None:
            return False
        if generation == self._last_command_generation:
            return False

        width_mm = self._clamp_width(requested_width)
        if width_mm != requested_width:
            LOGGER.warning(
                "Gripper command clamped because requested width is outside the supported range generation=%d",
                generation,
            )

        get_width = getattr(self.gripper, "get_width", None)
        command_is_closing = None
        if callable(get_width):
            try:
                current_width = float(get_width())
                if math.isfinite(current_width):
                    # Equal-width commands have no direction in the public
                    # width-only command and must not be treated as close.
                    command_is_closing = width_mm < current_width - 1e-6
            except (TypeError, ValueError):
                command_is_closing = None

        self.gripper.set_width(
            width_mm,
            duration=self.move_duration,
        )

        self._last_command_width_mm = width_mm
        self._last_command_generation = generation
        self._last_command_is_closing = command_is_closing

        return True

    def publish_status(
        self,
        *,
        runtime_ready,
        grasp_confirmed,
        attached,
        released,
        width_mm,
        sequence,
        capture_reference_id="",
    ):
        """Publish simulator-owned grasp/attachment state through the ActionGraph."""
        if self._status_attribute is None and not self._resolve_status_attribute():
            return False
        payload = (
            f"ready={int(bool(runtime_ready))};"
            f"grasp={int(bool(grasp_confirmed))};"
            f"attached={int(bool(attached))};"
            f"released={int(bool(released))};"
            f"width_mm={float(width_mm):.3f};"
            f"seq={int(sequence)}"
            f";capture_reference_id={capture_reference_id or ''}"
        )
        self._controller.set(self._status_attribute, payload)
        return True

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    @property
    def last_command_width_mm(self):
        return self._last_command_width_mm

    @property
    def last_command_generation(self):
        return self._last_command_generation

    @property
    def last_command_is_closing(self):
        return self._last_command_is_closing

    def diagnostics(self):
        return {
            "graph_path": self.graph_path,
            "subscriber_node": self.subscriber_node,
            "attribute_path": self._attribute_path(),
            "attribute_ready": self._data_attribute is not None,
            "last_command_width_mm": self._last_command_width_mm,
            "last_command_generation": self._last_command_generation,
        }
