import logging
import math

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
        self._status_attribute = None
        self._last_command_width_mm = None
        self._last_command_generation = None
        self._last_command_is_closing = None

        self._resolve_attribute()
        self._resolve_generation_attribute()
        self._resolve_status_attribute()

        if self.verbose:
            LOGGER.debug(
                "simulation gripper adapter ready graph=%s node=%s input=%s",
                self.graph_path,
                self.subscriber_node,
                self._attribute_path(),
            )

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

    def prime_command_generation(self):
        """Treat the retained graph value as a baseline, not a new command."""
        generation = self._read_command_generation()
        if generation is None:
            return False
        self._last_command_generation = generation
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
        requested_width = self._read_width_mm()

        if requested_width is None:
            return False

        width_mm = self._clamp_width(requested_width)
        generation = self._read_command_generation()

        # The Counter is driven by ROS2Subscriber.outputs:execOut, which fires
        # only when a new message is received.  The retained data value is not
        # a command identity.
        if generation is None:
            return False
        if generation == self._last_command_generation:
            return False

        if width_mm != requested_width:
            LOGGER.warning(
                "gripper command width clamped generation=%d requested_width_mm=%.3f "
                "applied_width_mm=%.3f",
                generation,
                requested_width,
                width_mm,
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

        if self.verbose:
            LOGGER.debug(
                "gripper command applied generation=%d width_mm=%.3f closing=%s",
                generation,
                width_mm,
                command_is_closing is True,
            )

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
