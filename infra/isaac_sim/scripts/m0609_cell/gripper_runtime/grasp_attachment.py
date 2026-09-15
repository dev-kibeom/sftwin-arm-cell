from enum import Enum
import math

from gripper_runtime.grasp_policy import GraspConfig


def _pxr_modules():
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    return Gf, Sdf, Usd, UsdGeom, UsdPhysics


class GraspState(Enum):
    EMPTY = "empty"
    SEEKING = "seeking"
    CONTACT = "contact"
    GRASPED = "grasped"


class GraspManager:
    """
    SF-Twin scripted grasp manager.

    Responsibilities
    ----------------
    - Observe sf_grasp_tcp and a registered workpiece every physics tick.
    - While the gripper is CLOSING, evaluate:
        1) TCP <-> grasp-point position error
        2) orientation error
        3) logical finger-width/contact condition
    - Generate a virtual resistance/contact event.
    - Stop the logical gripper and attach the workpiece with a fixed joint.
    - Release by removing that fixed joint.

    Non-responsibilities
    --------------------
    - Robotiq linkage animation / q interpolation -> Robotiq2F85Actuator
    - Vision grasp-pose estimation               -> Vision component
    - BT SUCCESS/FAILURE policy                  -> Behavior Tree layer

    Assumption for v4
    -----------------
    The workpiece prim origin is its desired grasp point.  A later version can
    accept a separate grasp-frame prim produced by Vision.
    """

    DEFAULT_TCP_NAME = "sf_grasp_tcp"
    ATTACH_JOINT_NAME = "sf_scripted_grasp_joint"

    def __init__(
        self,
        gripper,
        target_path=None,
        tcp_path=None,
        target_width_mm=None,
        config=None,
        stage=None,
        verbose=False,
    ):
        if stage is None:
            import omni.usd

            self.stage = omni.usd.get_context().get_stage()
        else:
            self.stage = stage
        self.gripper = gripper
        self.verbose = verbose

        self.config = (config or GraspConfig()).validate()

        self.tcp_path = tcp_path or self._discover_tcp_path()
        self.target_path = None
        self.target_width_mm = None

        self.state = GraspState.EMPTY
        self.object_detected = False
        self.resistance = False

        self.last_position_error_m = 0
        self.last_orientation_error_deg = 0
        self.last_width_error_mm = 0

        self._attach_joint_path = None

        if target_path is not None:
            self.set_target(target_path, target_width_mm=target_width_mm)

    # ------------------------------------------------------------------
    # Target / TCP discovery
    # ------------------------------------------------------------------

    def _discover_tcp_path(self):
        candidates = []

        robot_root = getattr(self.gripper, "robot_root_path", None)
        root_prefix = robot_root.rstrip("/") + "/" if robot_root else None

        for prim in self.stage.Traverse():
            if prim.GetName() != self.DEFAULT_TCP_NAME:
                continue

            path = str(prim.GetPath())

            if root_prefix is None or path.startswith(root_prefix):
                candidates.append(path)

        if len(candidates) != 1:
            raise RuntimeError(
                f"Expected exactly one '{self.DEFAULT_TCP_NAME}'"
                + (f" under {robot_root}" if robot_root else "")
                + f", found {len(candidates)}:\n  "
                + "\n  ".join(candidates)
            )

        if self.verbose:
            print(f"[GraspManager] TCP: {candidates[0]}")

        return candidates[0]

    def set_target(self, target_path, target_width_mm=None):
        prim = self.stage.GetPrimAtPath(target_path)
        if not prim.IsValid():
            raise RuntimeError(f"Target prim not found: {target_path}")

        if self.state == GraspState.GRASPED:
            raise RuntimeError(
                "Cannot replace target while a workpiece is grasped. "
                "Call release() first."
            )

        self.target_path = str(target_path)
        self.target_width_mm = (
            None if target_width_mm is None else float(target_width_mm)
        )

        self.state = GraspState.SEEKING
        self.object_detected = False
        self.resistance = False

        if self.verbose:
            print(
                f"[GraspManager] target={self.target_path}, "
                f"width={self.target_width_mm}"
            )

    def clear_target(self):
        if self.state == GraspState.GRASPED:
            raise RuntimeError("Release the grasped object before clearing target.")

        self.target_path = None
        self.target_width_mm = None
        self.state = GraspState.EMPTY
        self.object_detected = False
        self.resistance = False

    # ------------------------------------------------------------------
    # Pose / contact evaluation
    # ------------------------------------------------------------------

    def _world_transform(self, prim_path):
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise RuntimeError(f"Prim not found: {prim_path}")

        _, _, usd, usd_geom, _ = _pxr_modules()
        return usd_geom.Xformable(prim).ComputeLocalToWorldTransform(
            usd.TimeCode.Default()
        )

    @staticmethod
    def _translation(matrix):
        gf, _, _, _, _ = _pxr_modules()
        t = matrix.ExtractTranslation()
        return gf.Vec3d(t[0], t[1], t[2])

    @staticmethod
    def _rotation(matrix):
        return matrix.ExtractRotation()

    def _position_error(self, tcp_world, target_world):
        a = self._translation(tcp_world)
        b = self._translation(target_world)
        d = a - b
        return math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])

    def _orientation_error_deg(self, tcp_world, target_world):
        """
        Full-frame angular difference between TCP and target.

        v1 assumes target prim orientation is the desired grasp orientation.
        For symmetric cylindrical workpieces this can later be replaced by an
        axis-aware metric.
        """
        tcp_q = self._rotation(tcp_world).GetQuat()
        target_q = self._rotation(target_world).GetQuat()

        rel = tcp_q.GetInverse() * target_q
        real = max(-1.0, min(1.0, abs(float(rel.GetReal()))))
        angle_rad = 2.0 * math.acos(real)
        return math.degrees(angle_rad)

    def _pose_is_valid(self):
        tcp_world = self._world_transform(self.tcp_path)
        target_world = self._world_transform(self.target_path)

        self.last_position_error_m = self._position_error(tcp_world, target_world)
        self.last_orientation_error_deg = self._orientation_error_deg(
            tcp_world, target_world
        )

        return (
            self.last_position_error_m <= self.config.position_tolerance_m
            and self.last_orientation_error_deg <= self.config.orientation_tolerance_deg
        )

    def _width_contact_is_valid(self):
        """
        Virtual resistance model.

        If target_width_mm is supplied, contact occurs once the commanded
        opening reaches approximately that width.

        If no width is supplied, pose validity alone is used as the v3 contact
        trigger.  For meaningful resistance simulation, provide target width.
        """
        if self.target_width_mm is None:
            self.last_width_error_mm = None
            return True

        current_width = self.gripper.get_width()
        self.last_width_error_mm = current_width - self.target_width_mm

        # Contact is valid only inside a bounded band around the expected
        # workpiece width.  This prevents a late pose match from producing a
        # false grasp after the fingers have already closed far past the part.
        return abs(self.last_width_error_mm) <= self.config.contact_width_tolerance_mm

    # ------------------------------------------------------------------
    # Physics-tick update
    # ------------------------------------------------------------------

    def update(self, dt):
        """
        Observe one physics tick.

        Recommended order in the shared physics callback:

            gripper.update(dt)
            grasp_manager.update(dt)

        so contact is evaluated after the gripper has advanced to q(t).
        """
        if self.target_path is None:
            return self.state

        if self.state == GraspState.GRASPED:
            self.object_detected = True
            self.resistance = True
            return self.state

        # Import locally to avoid coupling this module to the actuator module
        # filename/package layout.
        gripper_state = getattr(self.gripper.state, "value", "")

        if gripper_state != "closing":
            self.object_detected = False
            self.resistance = False

            if self.state != GraspState.EMPTY:
                self.state = GraspState.SEEKING

            return self.state

        pose_ok = self._pose_is_valid()
        width_ok = self._width_contact_is_valid()

        self.object_detected = bool(pose_ok)

        if pose_ok and width_ok:
            self.state = GraspState.CONTACT
            self.resistance = True

            # Freeze at the current interpolated opening before attachment.
            self.gripper.stop()
            self._attach_target()

            self.state = GraspState.GRASPED
            self.object_detected = True

            if self.verbose:
                print(
                    "[GraspManager] GRASPED "
                    f"pos_err={self.last_position_error_m * 1000.0:.2f} mm, "
                    f"rot_err={self.last_orientation_error_deg:.2f} deg, "
                    f"width={self.gripper.get_width():.2f} mm"
                )
        else:
            self.state = GraspState.SEEKING
            self.resistance = False

        return self.state

    # ------------------------------------------------------------------
    # Scripted attachment
    # ------------------------------------------------------------------

    @staticmethod
    def _world_frame_relative_to_body(body_world, frame_world):
        """
        Express a world-space frame in a body's local coordinates.

        UsdPhysics localPos/localRot are defined as the joint frame relative
        to each body's frame.  Computing translation and rotation explicitly
        avoids relying on Matrix4d multiplication-order assumptions.
        """
        body_inv = body_world.GetInverse()

        frame_world_pos = frame_world.ExtractTranslation()
        local_pos_d = body_inv.Transform(frame_world_pos)

        body_world_q = body_world.ExtractRotationQuat()
        frame_world_q = frame_world.ExtractRotationQuat()

        # q_local rotates from the body's frame into the joint frame.
        local_q_d = body_world_q.GetInverse() * frame_world_q
        local_q_d.Normalize()

        qi = local_q_d.GetImaginary()

        gf, _, _, _, _ = _pxr_modules()
        local_pos = gf.Vec3f(
            float(local_pos_d[0]),
            float(local_pos_d[1]),
            float(local_pos_d[2]),
        )
        local_rot = gf.Quatf(
            float(local_q_d.GetReal()),
            gf.Vec3f(
                float(qi[0]),
                float(qi[1]),
                float(qi[2]),
            ),
        )
        return local_pos, local_rot

    def _attach_target(self):
        if self._attach_joint_path is not None:
            return

        target_prim = self.stage.GetPrimAtPath(self.target_path)
        tcp_prim = self.stage.GetPrimAtPath(self.tcp_path)

        if not target_prim.IsValid() or not tcp_prim.IsValid():
            raise RuntimeError("TCP or target prim became invalid before attach.")

        # Capture current transforms at the exact accepted-contact instant.
        tcp_world = self._world_transform(self.tcp_path)
        target_world = self._world_transform(self.target_path)

        # Use the workpiece's CURRENT frame as the joint world frame.
        # Therefore:
        #
        #   body0 / TCP      -> current workpiece frame expressed in TCP frame
        #   body1 / target   -> identity, because joint frame == target frame
        #
        # Both sides consequently describe the exact same world-space frame,
        # so creating the FixedJoint should not require a positional or
        # rotational snap.
        local_pos0, local_rot0 = self._world_frame_relative_to_body(
            tcp_world,
            target_world,
        )

        gf, sdf, _, _, usd_physics = _pxr_modules()
        local_pos1 = gf.Vec3f(0.0, 0.0, 0.0)
        local_rot1 = gf.Quatf(
            1.0,
            gf.Vec3f(0.0, 0.0, 0.0),
        )

        joint_path = sdf.Path(self.target_path).AppendChild(self.ATTACH_JOINT_NAME)

        if self.stage.GetPrimAtPath(joint_path).IsValid():
            self.stage.RemovePrim(joint_path)

        joint = usd_physics.FixedJoint.Define(self.stage, joint_path)
        joint.CreateBody0Rel().SetTargets([sdf.Path(self.tcp_path)])
        joint.CreateBody1Rel().SetTargets([sdf.Path(self.target_path)])

        joint.CreateLocalPos0Attr().Set(local_pos0)
        joint.CreateLocalRot0Attr().Set(local_rot0)
        joint.CreateLocalPos1Attr().Set(local_pos1)
        joint.CreateLocalRot1Attr().Set(local_rot1)

        self._attach_joint_path = str(joint_path)

        if self.verbose:
            print(
                "[GraspManager] attached: "
                f"{self._attach_joint_path} "
                f"(preserved offset="
                f"{local_pos0[0] * 1000.0:.2f}, "
                f"{local_pos0[1] * 1000.0:.2f}, "
                f"{local_pos0[2] * 1000.0:.2f} mm)"
            )

    def release(self, open_gripper=True, duration=None):
        """
        Remove scripted attachment and optionally command the gripper open.
        """
        if self._attach_joint_path:
            prim = self.stage.GetPrimAtPath(self._attach_joint_path)
            if prim.IsValid():
                self.stage.RemovePrim(self._attach_joint_path)

        self._attach_joint_path = None
        self.resistance = False
        self.object_detected = False

        if self.target_path is None:
            self.state = GraspState.EMPTY
        else:
            self.state = GraspState.SEEKING

        if open_gripper:
            self.gripper.open(duration=duration)

        if self.verbose:
            print("[GraspManager] released")

    @property
    def is_grasped(self):
        return self.state == GraspState.GRASPED

    @property
    def attach_joint_path(self):
        return self._attach_joint_path

    def _refresh_diagnostics(self):
        """
        Refresh observable pose/width values regardless of gripper state.

        This prevents diagnostics() after release/open from reporting values
        cached from a previous closing cycle.
        """
        if self.target_path is None:
            self.last_position_error_m = None
            self.last_orientation_error_deg = None
            self.last_width_error_mm = None
            return

        target_prim = self.stage.GetPrimAtPath(self.target_path)
        tcp_prim = self.stage.GetPrimAtPath(self.tcp_path)

        if not target_prim.IsValid() or not tcp_prim.IsValid():
            self.last_position_error_m = None
            self.last_orientation_error_deg = None
            self.last_width_error_mm = None
            return

        tcp_world = self._world_transform(self.tcp_path)
        target_world = self._world_transform(self.target_path)

        self.last_position_error_m = self._position_error(
            tcp_world,
            target_world,
        )
        self.last_orientation_error_deg = self._orientation_error_deg(
            tcp_world,
            target_world,
        )

        if self.target_width_mm is None:
            self.last_width_error_mm = None
        else:
            self.last_width_error_mm = self.gripper.get_width() - self.target_width_mm

    def diagnostics(self):
        self._refresh_diagnostics()

        return {
            "state": self.state.value,
            "target_path": self.target_path,
            "tcp_path": self.tcp_path,
            "object_detected": self.object_detected,
            "resistance": self.resistance,
            "position_error_m": self.last_position_error_m,
            "orientation_error_deg": self.last_orientation_error_deg,
            "width_error_mm": self.last_width_error_mm,
            "gripper_width_mm": self.gripper.get_width(),
            "attach_joint_path": self._attach_joint_path,
            "config": {
                "position_tolerance_m": self.config.position_tolerance_m,
                "orientation_tolerance_deg": (self.config.orientation_tolerance_deg),
                "contact_width_tolerance_mm": (self.config.contact_width_tolerance_mm),
            },
        }
