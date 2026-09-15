import math

from graph_builder.graph_contract import GRAPH_PATH


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
        self._last_command_width_mm = None

        self._resolve_attribute()

        if self.verbose:
            print(
                "[SimGripperAdapter] READY\n"
                f"  graph : {self.graph_path}\n"
                f"  node  : {self.subscriber_node}\n"
                f"  input : {self._attribute_path()}"
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

        # Generic subscriber output retains the latest received value.
        # Apply only when the command value changes.
        if (
            self._last_command_width_mm is not None
            and abs(width_mm - self._last_command_width_mm) < 1e-6
        ):
            return False

        if self.verbose and width_mm != requested_width:
            print(
                "[SimGripperAdapter] width clamped: "
                f"{requested_width:.3f} -> {width_mm:.3f} mm"
            )

        self.gripper.set_width(
            width_mm,
            duration=self.move_duration,
        )

        self._last_command_width_mm = width_mm

        if self.verbose:
            print(f"[SimGripperAdapter] command width={width_mm:.1f} mm")

        return True

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    @property
    def last_command_width_mm(self):
        return self._last_command_width_mm

    def diagnostics(self):
        return {
            "graph_path": self.graph_path,
            "subscriber_node": self.subscriber_node,
            "attribute_path": self._attribute_path(),
            "attribute_ready": self._data_attribute is not None,
            "last_command_width_mm": self._last_command_width_mm,
        }
