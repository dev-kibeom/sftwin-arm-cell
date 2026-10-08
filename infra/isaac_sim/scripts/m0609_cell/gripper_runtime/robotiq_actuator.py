import logging
from enum import Enum

LOGGER = logging.getLogger(__name__)


class GripperState(Enum):
    IDLE = "idle"
    OPENING = "opening"
    CLOSING = "closing"
    HOLDING = "holding"


class Robotiq2F85Actuator:
    """
    SF-Twin logical actuator for the Robotiq 2F-85.

    Design:
      - ROS / MoveIt keeps the original URDF mimic model.
      - Isaac Sim imports with parse_mimic=False.
      - This class maps one logical command q in [0, 1] to the six Robotiq DOFs.
      - Grasp attachment/detachment is intentionally handled by a separate GraspManager.

    q convention:
      0.0 = fully open
      1.0 = fully closed
    """

    JOINT_SIGNS = {
        "gripper_robotiq_85_left_knuckle_joint": +1.0,
        "gripper_robotiq_85_right_knuckle_joint": -1.0,
        "gripper_robotiq_85_left_inner_knuckle_joint": +1.0,
        "gripper_robotiq_85_right_inner_knuckle_joint": -1.0,
        "gripper_robotiq_85_left_finger_tip_joint": -1.0,
        "gripper_robotiq_85_right_finger_tip_joint": +1.0,
    }

    MAX_ANGLE_DEG = 45.8366
    MAX_WIDTH_MM = 85.0
    DEFAULT_MOVE_DURATION = 0.40
    POSITION_EPSILON = 1e-4

    def __init__(
        self,
        robot_root_path=None,
        articulation_root_path=None,
        articulation_base_path=None,
        stage=None,
        verbose=False,
    ):
        if stage is None:
            import omni.usd

            stage = omni.usd.get_context().get_stage()
        self.stage = stage
        self.verbose = verbose

        self.joints = {}
        self.dof_indices = {}

        self._discover_joints()

        inferred_robot_root = self._infer_robot_root_path()
        self.robot_root_path = robot_root_path or inferred_robot_root

        self.articulation_root_path = (
            articulation_root_path or self._discover_articulation_root_path()
        )

        self.articulation_base_path = (
            articulation_base_path or f"{self.robot_root_path}/base_link"
        )

        self._validate_articulation_root()
        self._connect_runtime_articulation()

        # Motion-state layer.  Low-level articulation discovery above is
        # unchanged from the validated actuator.
        self.state = GripperState.IDLE
        self._commanded_q = self.get_position()
        self._motion_start_q = None
        self._motion_target_q = None
        self._motion_duration = 0.0
        self._motion_elapsed = 0.0

    def _discover_joints(self):
        for prim in self.stage.Traverse():
            name = prim.GetName()
            if name in self.JOINT_SIGNS:
                self.joints[name] = prim

        missing = set(self.JOINT_SIGNS) - set(self.joints)
        if missing:
            raise RuntimeError(
                "Robotiq joints not found:\n  " + "\n  ".join(sorted(missing))
            )

        if self.verbose:
            LOGGER.debug(
                "Robotiq USD joints discovered paths=%s",
                {name: str(self.joints[name].GetPath()) for name in self.JOINT_SIGNS},
            )

    def _infer_robot_root_path(self):
        roots = set()

        for prim in self.joints.values():
            joint_path = prim.GetPath()
            joints_parent = joint_path.GetParentPath()

            if joints_parent.name != "joints":
                raise RuntimeError(
                    "Unexpected imported joint layout. Expected "
                    f".../joints/<joint>, got: {joint_path}"
                )

            roots.add(str(joints_parent.GetParentPath()))

        if len(roots) != 1:
            raise RuntimeError(
                "Robotiq joints do not belong to one robot root:\n  "
                + "\n  ".join(sorted(roots))
            )

        root = next(iter(roots))

        if self.verbose:
            LOGGER.debug("Robotiq robot root inferred path=%s", root)

        return root

    def _discover_articulation_root_path(self):
        candidates = []
        root_prefix = self.robot_root_path.rstrip("/") + "/"

        for prim in self.stage.Traverse():
            path = str(prim.GetPath())

            if not (path == self.robot_root_path or path.startswith(root_prefix)):
                continue

            from pxr import UsdPhysics

            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                candidates.append(path)

        if len(candidates) != 1:
            raise RuntimeError(
                "Expected exactly one ArticulationRootAPI under "
                f"{self.robot_root_path}, found {len(candidates)}:\n  "
                + "\n  ".join(candidates)
            )

        if self.verbose:
            LOGGER.debug("Robotiq articulation root discovered path=%s", candidates[0])

        return candidates[0]

    def _validate_articulation_root(self):
        prim = self.stage.GetPrimAtPath(self.articulation_root_path)

        if not prim.IsValid():
            raise RuntimeError(
                f"Articulation root prim not found: {self.articulation_root_path}"
            )

        from pxr import UsdPhysics

        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            raise RuntimeError(
                f"ArticulationRootAPI not found on: {self.articulation_root_path}"
            )

    def _connect_runtime_articulation(self):
        from omni.isaac.dynamic_control import _dynamic_control

        self._dynamic_control = _dynamic_control
        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        if not self.dc.is_simulating():
            raise RuntimeError(
                "Physics is not running. Press Play, allow at least one physics "
                "frame, then construct Robotiq2F85Actuator."
            )

        self.articulation = self.dc.get_articulation(self.articulation_base_path)

        if self.articulation == _dynamic_control.INVALID_HANDLE:
            raise RuntimeError(
                "PhysX articulation was not found from the runtime lookup path:\n"
                f"  {self.articulation_base_path}\n"
                "USD ArticulationRootAPI path:\n"
                f"  {self.articulation_root_path}"
            )

        dof_count = self.dc.get_articulation_dof_count(self.articulation)

        for index in range(dof_count):
            dof = self.dc.get_articulation_dof(self.articulation, index)
            name = self.dc.get_dof_name(dof)

            if name in self.JOINT_SIGNS:
                self.dof_indices[name] = index

        missing = set(self.JOINT_SIGNS) - set(self.dof_indices)
        if missing:
            raise RuntimeError(
                "Robotiq USD joints exist, but these DOFs are absent from the "
                "runtime articulation:\n  " + "\n  ".join(sorted(missing))
            )

        if self.verbose:
            runtime_path = self.dc.get_articulation_path(self.articulation)
            LOGGER.debug(
                "Robotiq actuator ready robot_root=%s lookup_path=%s "
                "runtime_path=%s dof_count=%d",
                self.robot_root_path,
                self.articulation_base_path,
                runtime_path,
                dof_count,
            )

    @staticmethod
    def _clamp01(value):
        return max(0.0, min(1.0, float(value)))

    @staticmethod
    def _smoothstep01(t):
        """C1-continuous interpolation: zero slope at start/end."""
        t = max(0.0, min(1.0, float(t)))
        return t * t * (3.0 - 2.0 * t)

    # -------------------------------------------------------------------------
    # Low-level instantaneous pose application
    # -------------------------------------------------------------------------

    def _apply_position(self, q):
        """
        Apply q immediately to all six Robotiq DOFs.

        This is the already-validated atomic write path.  It is kept private so
        the motion state machine can call it every physics tick without
        cancelling its own trajectory.
        """
        import numpy as np

        q = self._clamp01(q)
        logical_angle = q * np.deg2rad(self.MAX_ANGLE_DEG)

        states = self.dc.get_articulation_dof_states(
            self.articulation,
            self._dynamic_control.STATE_ALL,
        )

        for name, sign in self.JOINT_SIGNS.items():
            index = self.dof_indices[name]
            states["pos"][index] = logical_angle * sign
            states["vel"][index] = 0.0

        self.dc.wake_up_articulation(self.articulation)

        ok = self.dc.set_articulation_dof_states(
            self.articulation,
            states,
            self._dynamic_control.STATE_ALL,
        )
        if not ok:
            raise RuntimeError("Failed to write articulation DOF states.")

        # Align the imported drives with the written state so the following
        # physics step does not pull the linkage toward stale targets.
        targets = self.dc.get_articulation_dof_position_targets(self.articulation)

        for name, sign in self.JOINT_SIGNS.items():
            index = self.dof_indices[name]
            targets[index] = logical_angle * sign

        ok = self.dc.set_articulation_dof_position_targets(
            self.articulation,
            targets,
        )
        if not ok:
            raise RuntimeError("Failed to update articulation position targets.")

        self._commanded_q = q

    def set_position(self, q):
        """
        Immediately place the gripper at q.

        q:
            0.0 = fully open
            1.0 = fully closed

        This intentionally bypasses interpolation.  Use move_to()/open()/close()
        for normal time-based motion.
        """
        self._cancel_motion()
        self._apply_position(q)
        self.state = GripperState.HOLDING

        if self.verbose:
            LOGGER.debug(
                "Robotiq position set q=%.3f width_mm=%.3f",
                self._commanded_q,
                self.position_to_width(self._commanded_q),
            )

    def get_position(self):
        """Read logical q from the left knuckle DOF."""
        import numpy as np

        name = "gripper_robotiq_85_left_knuckle_joint"
        index = self.dof_indices[name]

        states = self.dc.get_articulation_dof_states(
            self.articulation,
            self._dynamic_control.STATE_POS,
        )

        angle = float(states["pos"][index])
        max_angle = np.deg2rad(self.MAX_ANGLE_DEG)

        if max_angle <= 0.0:
            return 0.0

        return self._clamp01(angle / max_angle)

    # -------------------------------------------------------------------------
    # Tick-based motion state machine
    # -------------------------------------------------------------------------

    def _cancel_motion(self):
        self._motion_start_q = None
        self._motion_target_q = None
        self._motion_duration = 0.0
        self._motion_elapsed = 0.0

    def move_to(self, q, duration=None):
        """
        Start a non-blocking interpolated move.

        Call update(dt) once per physics tick until is_moving becomes False.

        Args:
            q: target logical position in [0, 1].
            duration: motion duration in seconds.  If omitted,
                      DEFAULT_MOVE_DURATION is used.
        """
        q = self._clamp01(q)
        duration = (
            self.DEFAULT_MOVE_DURATION
            if duration is None
            else max(0.0, float(duration))
        )

        start_q = self.get_position()

        if duration <= 0.0 or abs(q - start_q) <= self.POSITION_EPSILON:
            self._cancel_motion()
            self._apply_position(q)
            self.state = GripperState.HOLDING
            return

        self._motion_start_q = start_q
        self._motion_target_q = q
        self._motion_duration = duration
        self._motion_elapsed = 0.0

        if q > start_q:
            self.state = GripperState.CLOSING
        else:
            self.state = GripperState.OPENING

        if self.verbose:
            LOGGER.debug(
                "Robotiq motion started start_q=%.3f target_q=%.3f "
                "start_width_mm=%.3f target_width_mm=%.3f duration_s=%.3f state=%s",
                start_q,
                q,
                self.position_to_width(start_q),
                self.position_to_width(q),
                duration,
                self.state.value,
            )

    def update(self, dt):
        """
        Advance the active motion by one physics tick.

        Returns:
            True while a motion remains active, otherwise False.
        """
        if not self.is_moving:
            return False

        dt = max(0.0, float(dt))
        if dt <= 0.0:
            return True

        self._motion_elapsed += dt
        u = min(1.0, self._motion_elapsed / self._motion_duration)
        s = self._smoothstep01(u)

        q = self._motion_start_q + (self._motion_target_q - self._motion_start_q) * s
        self._apply_position(q)

        if u >= 1.0:
            target_q = self._motion_target_q
            self._cancel_motion()
            self._apply_position(target_q)
            self.state = GripperState.HOLDING

            if self.verbose:
                LOGGER.debug(
                    "Robotiq motion completed target_q=%.3f target_width_mm=%.3f",
                    target_q,
                    self.position_to_width(target_q),
                )

            return False

        return True

    def stop(self):
        """
        Stop an active interpolated motion at its current position.

        GraspManager will later use this when virtual resistance/contact is
        detected during CLOSING.
        """
        q = self.get_position()
        self._cancel_motion()
        self._apply_position(q)
        self.state = GripperState.HOLDING

        if self.verbose:
            LOGGER.debug(
                "Robotiq motion stopped q=%.3f observed_width_mm=%.3f",
                q,
                self.position_to_width(q),
            )

        return q

    @property
    def is_moving(self):
        return self.state in (
            GripperState.OPENING,
            GripperState.CLOSING,
        )

    @property
    def target_position(self):
        return self._motion_target_q

    @property
    def commanded_position(self):
        return self._commanded_q

    # -------------------------------------------------------------------------
    # Width abstraction / convenience commands
    # -------------------------------------------------------------------------

    @classmethod
    def position_to_width(cls, q):
        """
        Logical linear width mapping.

        This is an SF-Twin command abstraction, not an exact fingertip
        kinematic solution.
        """
        q = cls._clamp01(q)
        return cls.MAX_WIDTH_MM * (1.0 - q)

    @classmethod
    def width_to_position(cls, width_mm):
        width_mm = max(0.0, min(cls.MAX_WIDTH_MM, float(width_mm)))
        return 1.0 - (width_mm / cls.MAX_WIDTH_MM)

    def set_width(self, width_mm, duration=None):
        """Move smoothly to a logical opening width."""
        self.move_to(self.width_to_position(width_mm), duration=duration)

    def get_width(self):
        return self.position_to_width(self.get_position())

    def open(self, duration=None):
        """Start a smooth opening motion."""
        self.move_to(0.0, duration=duration)

    def close(self, duration=None):
        """Start a smooth closing motion."""
        self.move_to(1.0, duration=duration)
