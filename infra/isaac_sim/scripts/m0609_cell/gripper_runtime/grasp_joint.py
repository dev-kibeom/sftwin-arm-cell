from gripper_runtime.grasp_stage import _pxr_modules, _stage_pose_dict


def _world_frame_relative_to_body(body_world, frame_world):
    body_inv = body_world.GetInverse()
    frame_world_pos = frame_world.ExtractTranslation()
    local_pos_d = body_inv.Transform(frame_world_pos)
    body_world_q = body_world.ExtractRotationQuat()
    frame_world_q = frame_world.ExtractRotationQuat()
    local_q_d = body_world_q.GetInverse() * frame_world_q
    local_q_d.Normalize()

    gf, _, _, _, _ = _pxr_modules()
    qi = local_q_d.GetImaginary()
    return (
        gf.Vec3f(
            float(local_pos_d[0]),
            float(local_pos_d[1]),
            float(local_pos_d[2]),
        ),
        gf.Quatf(
            float(local_q_d.GetReal()),
            gf.Vec3f(float(qi[0]), float(qi[1]), float(qi[2])),
        ),
    )


class UsdFixedJointOwner:
    """Own and mutate only the adapter-created fixed joint."""

    JOINT_NAME = "sf_scripted_grasp_joint"

    def __init__(self, stage, tcp_path):
        self.stage = stage
        self.tcp_path = str(tcp_path)
        self._owned_joint_path = None
        self._target_path = None
        self.last_create_diagnostics = None

    def has_owned_joint(self):
        return self._owned_joint_path is not None

    def owned_joint_valid(self):
        if self._owned_joint_path is None or self._target_path is None:
            return False
        return self._signature_valid(self._owned_joint_path, self._target_path)

    def _signature_valid(self, joint_path, target_path):
        joint = self.stage.GetPrimAtPath(joint_path)
        target = self.stage.GetPrimAtPath(target_path)
        tcp = self.stage.GetPrimAtPath(self.tcp_path)
        if not joint.IsValid() or not target.IsValid() or not tcp.IsValid():
            return False
        if str(joint.GetTypeName()) != "PhysicsFixedJoint":
            return False
        body0 = [
            str(path) for path in joint.GetRelationship("physics:body0").GetTargets()
        ]
        body1 = [
            str(path) for path in joint.GetRelationship("physics:body1").GetTargets()
        ]
        return body0 == [self.tcp_path] and body1 == [str(target_path)]

    def reconcile_stale(self):
        """Remove only valid canonical joints left by an earlier runtime."""
        joint_paths = set()
        traverse = getattr(self.stage, "Traverse", None)
        if callable(traverse):
            for prim in traverse():
                if prim.GetName() == self.JOINT_NAME:
                    joint_paths.add(str(prim.GetPath()))
        if self._target_path is not None:
            joint_paths.add(self._joint_path(self._target_path))

        for joint_path in sorted(joint_paths):
            target_path = joint_path[: -len(self.JOINT_NAME) - 1]
            if not self._signature_valid(joint_path, target_path):
                return False
            self.stage.RemovePrim(joint_path)
            if self.stage.GetPrimAtPath(joint_path).IsValid():
                return False
        self._owned_joint_path = None
        self._target_path = None
        return True

    def create(self, target_path):
        target_path = str(target_path)
        self.last_create_diagnostics = {
            "joint_path": self._joint_path(target_path),
            "body0_path": self.tcp_path,
            "body1_path": target_path,
            "creation_result": "pending",
            "ownership_validation": "pending",
        }
        target_prim = self.stage.GetPrimAtPath(target_path)
        tcp_prim = self.stage.GetPrimAtPath(self.tcp_path)
        if not target_prim.IsValid() or not tcp_prim.IsValid():
            error = RuntimeError("TCP or candidate prim became invalid before attach")
            self.last_create_diagnostics.update(
                creation_result="failed",
                error_type=type(error).__name__,
                error=str(error),
            )
            raise error

        tcp_world = self._world_transform(self.tcp_path)
        target_world = self._world_transform(target_path)
        self.last_create_diagnostics["stage_pose_before"] = {
            "tcp": _stage_pose_dict(tcp_world),
            "rawpart": _stage_pose_dict(target_world),
        }
        local_pos0, local_rot0 = _world_frame_relative_to_body(
            tcp_world,
            target_world,
        )

        gf, sdf, _, _, usd_physics = _pxr_modules()
        joint_path = sdf.Path(target_path).AppendChild(self.JOINT_NAME)
        existing = self.stage.GetPrimAtPath(joint_path)
        if existing.IsValid():
            self.last_create_diagnostics.update(
                creation_result="failed",
                error_type="RuntimeError",
                error="target joint path already exists",
            )
            return False

        joint = usd_physics.FixedJoint.Define(self.stage, joint_path)
        joint.CreateBody0Rel().SetTargets([sdf.Path(self.tcp_path)])
        joint.CreateBody1Rel().SetTargets([sdf.Path(target_path)])
        joint.CreateLocalPos0Attr().Set(local_pos0)
        joint.CreateLocalRot0Attr().Set(local_rot0)
        joint.CreateLocalPos1Attr().Set(gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr().Set(gf.Quatf(1.0, gf.Vec3f(0.0, 0.0, 0.0)))
        self._owned_joint_path = str(joint_path)
        self._target_path = target_path
        self.last_create_diagnostics["creation_result"] = "created"
        for key, reader, converter in (
            ("local_pos0", joint.GetLocalPos0Attr, self._vector_value),
            ("local_rot0", joint.GetLocalRot0Attr, self._quaternion_value),
            ("local_pos1", joint.GetLocalPos1Attr, self._vector_value),
            ("local_rot1", joint.GetLocalRot1Attr, self._quaternion_value),
        ):
            try:
                self.last_create_diagnostics[key] = converter(reader().Get())
            except Exception as error:
                self.last_create_diagnostics[key] = {
                    "status": "unavailable",
                    "reason": f"USD authored value read failed: {type(error).__name__}: {error}",
                }
        tcp_after = self._world_transform(self.tcp_path)
        target_after = self._world_transform(target_path)
        self.last_create_diagnostics["stage_pose_after"] = {
            "tcp": _stage_pose_dict(tcp_after),
            "rawpart": _stage_pose_dict(target_after),
        }
        return True

    def set_ownership_validation(self, valid):
        if self.last_create_diagnostics is not None:
            self.last_create_diagnostics["ownership_validation"] = bool(valid)

    @staticmethod
    def _vector_value(value):
        return tuple(float(value[i]) for i in range(3))

    @staticmethod
    def _quaternion_value(value):
        imaginary = value.GetImaginary()
        return (
            float(imaginary[0]),
            float(imaginary[1]),
            float(imaginary[2]),
            float(value.GetReal()),
        )

    @classmethod
    def _joint_path(cls, target_path):
        return f"{str(target_path).rstrip('/')}/{cls.JOINT_NAME}"

    def remove_owned(self):
        joint_path = self._owned_joint_path
        target_path = self._target_path
        if joint_path is None or target_path is None:
            return False

        try:
            joint = self.stage.GetPrimAtPath(joint_path)
            target = self.stage.GetPrimAtPath(target_path)
            if not joint.IsValid() or not target.IsValid():
                return False
            if not self._signature_valid(joint_path, target_path):
                return False
            self.stage.RemovePrim(joint_path)
            removed_joint = self.stage.GetPrimAtPath(joint_path)
            remaining_target = self.stage.GetPrimAtPath(target_path)
        except Exception:
            return False
        if removed_joint.IsValid() or not remaining_target.IsValid():
            return False

        self._owned_joint_path = None
        self._target_path = None
        return True

    def target_exists(self):
        return (
            self._target_path is not None
            and self.stage.GetPrimAtPath(self._target_path).IsValid()
        )

    @property
    def owned_joint_path(self):
        return self._owned_joint_path

    @staticmethod
    def _path(value):
        _, sdf, _, _, _ = _pxr_modules()
        return sdf.Path(value)

    def _world_transform(self, prim_path):
        _, _, usd, usd_geom, _ = _pxr_modules()
        prim = self.stage.GetPrimAtPath(prim_path)
        return usd_geom.Xformable(prim).ComputeLocalToWorldTransform(
            usd.TimeCode.Default()
        )
