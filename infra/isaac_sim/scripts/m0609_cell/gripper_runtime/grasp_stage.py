import math
from dataclasses import dataclass

from gripper_runtime.grasp_policy import CaptureCandidate


def _pxr_modules():
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    return Gf, Sdf, Usd, UsdGeom, UsdPhysics


@dataclass(frozen=True)
class CaptureReferenceConfig:
    """Fixture geometry needed to derive the semantic grasp reference."""

    fixture_dimensions_m: tuple
    observation_to_grasp_tcp_m: float
    insertion_axis_tcp: tuple

    def validate(self):
        if len(self.fixture_dimensions_m) != 3:
            raise ValueError("fixture_dimensions_m must have three values")
        if any(
            not math.isfinite(float(value)) or float(value) <= 0.0
            for value in self.fixture_dimensions_m
        ):
            raise ValueError("fixture dimensions must be finite and positive")
        if not math.isfinite(float(self.observation_to_grasp_tcp_m)):
            raise ValueError("observation-to-TCP offset must be finite")
        if len(self.insertion_axis_tcp) != 3:
            raise ValueError("insertion_axis_tcp must have three values")
        if not all(math.isfinite(float(value)) for value in self.insertion_axis_tcp):
            raise ValueError("insertion_axis_tcp must be finite")
        if (
            math.sqrt(sum(float(value) ** 2 for value in self.insertion_axis_tcp))
            <= 1e-12
        ):
            raise ValueError("insertion_axis_tcp must be non-zero")
        return self


@dataclass(frozen=True)
class CanonicalCaptureReference:
    position: tuple
    top_normal: tuple


class EligibleObjectRegistry:
    """Fixture-owned registry of candidate prim identities."""

    def __init__(self):
        self._paths = set()
        self._subtrees = set()

    def register(self, prim_path):
        self._paths.add(str(prim_path))

    def unregister(self, prim_path):
        self._paths.discard(str(prim_path))

    def register_subtree(self, prim_path):
        self._subtrees.add(str(prim_path).rstrip("/"))

    def paths(self):
        return tuple(sorted(self._paths))

    def candidate_paths(self, stage):
        """Resolve registered exact paths and typed descendants at query time."""
        paths = set(self._paths)
        for root_path in self._subtrees:
            root = stage.GetPrimAtPath(root_path)
            if not root.IsValid():
                return None
            for prim in stage.Traverse():
                path = str(prim.GetPath())
                if not path.startswith(root_path + "/"):
                    continue
                material = prim.GetAttribute("sf_twin:material_type")
                if material and material.Get() == "RawPart":
                    paths.add(path)
        return tuple(sorted(paths))


def _stage_pose_dict(transform):
    position = transform.ExtractTranslation()
    rotation = transform.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    return {
        "position_m": tuple(float(position[i]) for i in range(3)),
        "orientation_xyzw": tuple(float(imaginary[i]) for i in range(3))
        + (float(rotation.GetReal()),),
        "source": "usd_stage_world",
    }


class StageCaptureQuery:
    """Convert current USD stage observations into pure-policy candidates."""

    def __init__(self, stage, tcp_path, registry, gripper, reference_config=None):
        self.stage = stage
        self.tcp_path = str(tcp_path)
        self.registry = registry
        self.gripper = gripper
        self.reference_config = None
        self._last_reference_diagnostics = None
        self.expected_tcp_reference = None
        self._expected_tcp_reference_error = None
        self.actual_orientation_tolerance_deg = 5.0
        self.actual_contact_width_tolerance_mm = 2.0
        if reference_config is not None:
            self.configure_reference(reference_config)

    def configure_reference(self, reference_config):
        self.reference_config = reference_config.validate()
        return True

    def configure_expected_tcp_reference(
        self,
        frame_id,
        position,
        orientation_xyzw,
        symmetry_order,
        expected_contact_width_mm=None,
    ):
        self.expected_tcp_reference = None
        self._expected_tcp_reference_error = None
        if not isinstance(frame_id, str) or not frame_id.strip():
            self._expected_tcp_reference_error = {
                "frame_id": frame_id,
                "rejection_reason": "reference_frame_missing",
            }
            return False
        if len(position) != 3 or len(orientation_xyzw) != 4:
            self._expected_tcp_reference_error = {
                "frame_id": frame_id,
                "rejection_reason": "capture_reference_invalid",
            }
            return False
        values = tuple(float(value) for value in (*position, *orientation_xyzw))
        if not all(math.isfinite(value) for value in values):
            self._expected_tcp_reference_error = {
                "frame_id": frame_id,
                "rejection_reason": "capture_reference_invalid",
            }
            return False
        norm = math.sqrt(sum(value * value for value in values[3:]))
        # Order 0/1 is Fixed Vision provenance; Motion has already resolved a
        # concrete TCP orientation, including any allowed unobserved-yaw fallback.
        if abs(norm - 1.0) > 1e-6 or symmetry_order not in (0, 1, 2, 4):
            self._expected_tcp_reference_error = {
                "frame_id": frame_id,
                "rejection_reason": "capture_reference_invalid",
            }
            return False
        width = float(expected_contact_width_mm or 0.0)
        if not math.isfinite(width) or width <= 0.0:
            self._expected_tcp_reference_error = {
                "frame_id": frame_id,
                "rejection_reason": "capture_reference_invalid",
            }
            return False
        self.expected_tcp_reference = (
            frame_id,
            values[:3],
            values[3:],
            int(symmetry_order),
            width,
        )
        return True

    def clear_expected_tcp_reference(self):
        self.expected_tcp_reference = None
        self._expected_tcp_reference_error = None

    def invalidate_expected_tcp_reference(self, reason, frame_id=None):
        self.expected_tcp_reference = None
        self._expected_tcp_reference_error = {
            "frame_id": frame_id,
            "rejection_reason": str(reason),
        }

    def reference_diagnostics(self):
        return self._last_reference_diagnostics

    @classmethod
    def _safe_pose_dict(cls, transform):
        try:
            return cls._pose_dict(transform)
        except (
            Exception
        ) as error:  # noqa: BLE001 - optional pose diagnostics cannot gate Capture
            return {
                "status": "unavailable",
                "reason": f"Stage pose diagnostic unavailable: {type(error).__name__}",
            }

    def query(self):
        if self._expected_tcp_reference_error is not None:
            self._last_reference_diagnostics = dict(self._expected_tcp_reference_error)
        tcp_prim = self.stage.GetPrimAtPath(self.tcp_path)
        if not tcp_prim.IsValid():
            return False, []

        tcp_world = self._world_transform(self.tcp_path)
        tcp_stage_pose = self._safe_pose_dict(tcp_world)
        world_reference = None
        if self.expected_tcp_reference is not None:
            frame_id = self.expected_tcp_reference[0]
            try:
                frame_path = self._resolve_reference_frame_path(frame_id)
                if frame_path is None:
                    self._last_reference_diagnostics = {
                        "frame_id": frame_id,
                        "rejection_reason": "reference_frame_missing",
                    }
                else:
                    frame_world = self._world_transform(frame_path)
                    world_reference = self._transform_pose_to_world(
                        frame_world,
                        self.expected_tcp_reference[1],
                        self.expected_tcp_reference[2],
                    )
            except (TypeError, ValueError, ArithmeticError, RuntimeError) as error:
                self._last_reference_diagnostics = {
                    "frame_id": frame_id,
                    "rejection_reason": "reference_frame_transform_invalid",
                    "error_type": type(error).__name__,
                }
            except (
                Exception
            ) as error:  # noqa: BLE001 - optional reference diagnostics cannot gate Capture
                self._last_reference_diagnostics = {
                    "frame_id": frame_id,
                    "rejection_reason": "reference_frame_transform_failed",
                    "error_type": type(error).__name__,
                }
        candidates = []
        candidate_paths = getattr(self.registry, "candidate_paths", None)
        paths = (
            candidate_paths(self.stage) if candidate_paths else self.registry.paths()
        )
        if paths is None:
            return False, []
        for path in paths:
            target_prim = self.stage.GetPrimAtPath(path)
            if not target_prim.IsValid():
                return False, []
            shape_attribute = target_prim.GetAttribute("sf_twin:shape")
            shape = shape_attribute.Get() if shape_attribute else None

            target_world = self._world_transform(path)
            expected_orientation_error = None
            rawpart_error = None
            rawpart_rejection_reason = None
            orientation_error = None
            if self.expected_tcp_reference is not None and world_reference is not None:
                reference = None
                (
                    _frame_id,
                    expected_position,
                    expected_orientation,
                    symmetry_order,
                    expected_width_mm,
                ) = self.expected_tcp_reference
                expected_position, expected_orientation = world_reference
                target_position = target_world.ExtractTranslation()
                tcp_position = tcp_world.ExtractTranslation()
                delta = tuple(
                    float(tcp_position[index]) - expected_position[index]
                    for index in range(3)
                )
                position_error = math.sqrt(sum(value * value for value in delta))
                try:
                    expected_orientation_error = (
                        self._expected_tcp_orientation_error_deg(
                            tcp_world, expected_orientation, symmetry_order
                        )
                    )
                    rawpart_error, rawpart_rejection_reason = (
                        self._rawpart_orientation_error_deg(
                            tcp_world,
                            target_world,
                            expected_width_mm,
                            symmetry_order,
                            self.actual_orientation_tolerance_deg,
                            self.actual_contact_width_tolerance_mm,
                            dimensions_m=self._authored_dimensions(target_prim),
                            return_reason=True,
                        )
                    )
                    orientation_error = (
                        max(expected_orientation_error, rawpart_error)
                        if rawpart_error is not None
                        else None
                    )
                except (
                    Exception
                ) as error:  # noqa: BLE001 - optional orientation diagnostics cannot gate Capture
                    self._last_reference_diagnostics = {
                        "rejection_reason": "reference_diagnostic_unavailable",
                        "error_type": type(error).__name__,
                    }
                reference_position = expected_position
            elif (
                self.reference_config is not None
                and self.expected_tcp_reference is None
            ):
                target_position = target_world.ExtractTranslation()
                try:
                    reference = self._canonical_capture_reference(
                        target_world, self.reference_config
                    )
                    tcp_position = tcp_world.ExtractTranslation()
                    delta = tuple(
                        float(tcp_position[index]) - reference.position[index]
                        for index in range(3)
                    )
                    position_error = math.sqrt(
                        sum(float(value) ** 2 for value in delta)
                    )
                    orientation_error = self._orientation_error_deg(
                        tcp_world,
                        target_world,
                        self.reference_config.insertion_axis_tcp,
                    )
                    reference_position = reference.position
                except (
                    Exception
                ) as error:  # noqa: BLE001 - optional reference diagnostics cannot gate Capture
                    reference = None
                    position_error = None
                    orientation_error = None
                    reference_position = None
                    self._last_reference_diagnostics = {
                        "rejection_reason": "reference_diagnostic_unavailable",
                        "error_type": type(error).__name__,
                    }
            else:
                reference = None
                target_position = target_world.ExtractTranslation()
                position_error = None
                orientation_error = None
                reference_position = None
            local_position = tcp_world.GetInverse().Transform(target_position)
            tcp_rotation = tcp_world.ExtractRotationQuat()
            target_rotation = target_world.ExtractRotationQuat()
            try:
                local_rotation = self._relative_quaternion(
                    tcp_rotation, target_rotation
                )
            except Exception:  # noqa: BLE001 - relative orientation is diagnostic only
                local_rotation = None
            reference_data = (
                {
                    "frame_id": self.expected_tcp_reference[0],
                    "applied_frame": frame_path,
                    "reference_position_stage_world_m": world_reference[0],
                    "reference_stage_world_pose": {
                        "position_m": world_reference[0],
                        "orientation_xyzw": world_reference[1],
                        "source": "usd_stage_world",
                    },
                    "reference_local_pose": {
                        "position_m": self.expected_tcp_reference[1],
                        "orientation_xyzw": self.expected_tcp_reference[2],
                    },
                    "expected_contact_width_mm": self.expected_tcp_reference[4],
                    "orientation_symmetry_order": self.expected_tcp_reference[3],
                }
                if self.expected_tcp_reference is not None
                and world_reference is not None
                else (
                    {
                        "frame_id": self.expected_tcp_reference[0],
                        "applied_frame": {
                            "status": "unavailable",
                            "reason": (self._last_reference_diagnostics or {}).get(
                                "rejection_reason", "reference frame unavailable"
                            ),
                        },
                        "reference_local_pose": {
                            "position_m": self.expected_tcp_reference[1],
                            "orientation_xyzw": self.expected_tcp_reference[2],
                        },
                        "expected_contact_width_mm": self.expected_tcp_reference[4],
                        "orientation_symmetry_order": self.expected_tcp_reference[3],
                    }
                    if self.expected_tcp_reference is not None
                    else {
                        "applied_frame": "canonical_capture_reference",
                        "reference_position_stage_world_m": (
                            tuple(float(reference.position[i]) for i in range(3))
                            if reference is not None
                            else None
                        ),
                        "reference_top_normal_stage_world": (
                            tuple(float(reference.top_normal[i]) for i in range(3))
                            if reference is not None
                            else None
                        ),
                    }
                )
            )
            try:
                measured_opening = float(self.gripper.get_width())
            except Exception:  # noqa: BLE001 - measured opening is diagnostic only
                measured_opening = None
            candidates.append(
                CaptureCandidate(
                    prim_path=path,
                    eligible=True,
                    tcp_local_position_m=(
                        float(local_position[0]),
                        float(local_position[1]),
                        float(local_position[2]),
                    ),
                    position_error_m=position_error,
                    orientation_error_deg=orientation_error,
                    measured_opening_mm=measured_opening,
                    shape=shape,
                    expected_orientation_error_deg=(
                        expected_orientation_error
                        if self.expected_tcp_reference is not None
                        else None
                    ),
                    rawpart_orientation_error_deg=(
                        rawpart_error
                        if self.expected_tcp_reference is not None
                        else None
                    ),
                    rawpart_rejection_reason=(
                        rawpart_rejection_reason
                        if self.expected_tcp_reference is not None
                        else None
                    ),
                    tcp_stage_world_pose=tcp_stage_pose,
                    stage_world_pose=self._safe_pose_dict(target_world),
                    relative_pose={
                        "position_m": tuple(float(local_position[i]) for i in range(3)),
                        "orientation_xyzw": (
                            tuple(float(local_rotation[i]) for i in range(1, 4))
                            + (float(local_rotation[0]),)
                            if local_rotation is not None
                            else None
                        ),
                    },
                    capture_reference=reference_data,
                )
            )
            self._last_reference_diagnostics = {
                "raw_candidate_position": tuple(
                    float(value) for value in target_position
                ),
                "canonical_capture_position": reference_position,
                "expected_tcp_reference": self.expected_tcp_reference is not None,
                "frame_id": (
                    self.expected_tcp_reference[0]
                    if self.expected_tcp_reference is not None
                    else None
                ),
                "position_error_m": (
                    position_error if self.expected_tcp_reference is not None else None
                ),
                "expected_orientation_error_deg": (
                    expected_orientation_error
                    if self.expected_tcp_reference is not None
                    else None
                ),
                "rawpart_orientation_error_deg": (
                    rawpart_error if self.expected_tcp_reference is not None else None
                ),
                "rawpart_rejection_reason": (
                    rawpart_rejection_reason
                    if self.expected_tcp_reference is not None
                    else None
                ),
                "rejection_reason": (
                    rawpart_rejection_reason
                    if self.expected_tcp_reference is not None
                    else None
                ),
            }
        return True, candidates

    def _resolve_reference_frame_path(self, frame_id):
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise ValueError("empty reference frame id")
        parts = frame_id.split("/")
        if frame_id.startswith("/") or any(part in ("", ".", "..") for part in parts):
            raise ValueError("invalid reference frame id")
        root = getattr(self.gripper, "robot_root_path", None)
        if not isinstance(root, str) or not root.startswith("/"):
            raise ValueError("invalid robot root path")
        path = f"{root.rstrip('/')}/{frame_id}"
        prim = self.stage.GetPrimAtPath(path)
        return path if prim.IsValid() else None

    @staticmethod
    def _transform_pose_to_world(frame_world, position, orientation_xyzw):
        gf, _, _, _, _ = _pxr_modules()
        axes = [
            StageCaptureQuery._transform_direction(frame_world, axis)
            for axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        ]
        lengths = [
            math.sqrt(sum(component * component for component in axis)) for axis in axes
        ]
        if any(
            not math.isfinite(length) or abs(length - 1.0) > 1e-3 for length in lengths
        ):
            raise ValueError("reference frame transform is not rigid")
        if any(
            abs(sum(a * b for a, b in zip(axes[i], axes[j]))) > 1e-3
            for i in range(3)
            for j in range(i + 1, 3)
        ):
            raise ValueError("reference frame transform is not orthogonal")
        cross = (
            axes[0][1] * axes[1][2] - axes[0][2] * axes[1][1],
            axes[0][2] * axes[1][0] - axes[0][0] * axes[1][2],
            axes[0][0] * axes[1][1] - axes[0][1] * axes[1][0],
        )
        if sum(cross[i] * axes[2][i] for i in range(3)) < 0.999:
            raise ValueError("reference frame transform is not right handed")
        frame_position = frame_world.ExtractTranslation()
        if not all(math.isfinite(float(frame_position[i])) for i in range(3)):
            raise ValueError("reference frame translation is not finite")
        frame_q = frame_world.ExtractRotationQuat()
        imag = frame_q.GetImaginary()
        local_q = gf.Quatd(
            float(orientation_xyzw[3]), gf.Vec3d(*map(float, orientation_xyzw[:3]))
        )
        world_q = frame_q * local_q
        world_q.Normalize()
        world_imag = world_q.GetImaginary()
        world_position = frame_world.Transform(gf.Vec3d(*map(float, position)))
        result_position = tuple(float(world_position[i]) for i in range(3))
        result_orientation = tuple(float(world_imag[i]) for i in range(3)) + (
            float(world_q.GetReal()),
        )
        if not all(
            math.isfinite(value) for value in (*result_position, *result_orientation)
        ):
            raise ValueError("transformed reference pose is not finite")
        if not all(math.isfinite(float(imag[i])) for i in range(3)):
            raise ValueError("reference rotation is not finite")
        return result_position, result_orientation

    @staticmethod
    def _expected_tcp_orientation_error_deg(tcp_world, expected_xyzw, symmetry_order):
        gf, _, _, _, _ = _pxr_modules()
        expected_q = gf.Quatd(
            float(expected_xyzw[3]),
            gf.Vec3d(
                float(expected_xyzw[0]),
                float(expected_xyzw[1]),
                float(expected_xyzw[2]),
            ),
        )
        expected_rotation = gf.Matrix3d(expected_q)
        actual_rotation = tcp_world.ExtractRotationMatrix()
        expected_approach = gf.Vec3d(0.0, 0.0, 1.0) * expected_rotation
        actual_approach = gf.Vec3d(0.0, 0.0, 1.0) * actual_rotation
        expected_x = gf.Vec3d(1.0, 0.0, 0.0) * expected_rotation
        expected_y = gf.Vec3d(0.0, 1.0, 0.0) * expected_rotation
        actual_x = gf.Vec3d(1.0, 0.0, 0.0) * actual_rotation
        approach_error = StageCaptureQuery._axis_error_deg(
            actual_approach, expected_approach
        )
        closing_error = StageCaptureQuery._axis_error_deg(
            actual_x, expected_x, sign_invariant=True
        )
        if symmetry_order == 4:
            closing_error = min(
                closing_error,
                StageCaptureQuery._axis_error_deg(
                    actual_x, expected_y, sign_invariant=True
                ),
            )
        return max(approach_error, closing_error)

    @staticmethod
    def _rawpart_orientation_error_deg(
        tcp_world,
        target_world,
        expected_contact_width_mm,
        orientation_symmetry_order,
        orientation_tolerance_deg,
        contact_width_tolerance_mm,
        *,
        dimensions_m=None,
        return_reason=False,
    ):
        """Check box axes and opposing-face width from the Stage transform."""

        def result(error, reason=None):
            return (error, reason) if return_reason else error

        if orientation_symmetry_order not in (0, 1, 2, 4):
            return result(None, "rawpart_orientation_invalid")
        axes = [
            StageCaptureQuery._transform_direction(target_world, unit)
            for unit in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        ]
        transform_lengths = [
            math.sqrt(sum(value * value for value in axis)) for axis in axes
        ]
        if any(
            not math.isfinite(length) or length <= 1e-9 for length in transform_lengths
        ):
            return result(None, "rawpart_orientation_invalid")
        directions = [
            tuple(value / length for value in axis)
            for axis, length in zip(axes, transform_lengths)
        ]
        # Require a rigid, non-sheared object transform for an unambiguous face.
        for first in range(3):
            for second in range(first + 1, 3):
                if (
                    abs(
                        sum(
                            a * b for a, b in zip(directions[first], directions[second])
                        )
                    )
                    > 1e-3
                ):
                    return result(None, "rawpart_orientation_invalid")
        # Motion derives this symmetry from its observed face frame. The
        # simulator's local X/Y axes need not describe that same face, so do
        # not classify or compare symmetry from these box-local edge lengths.
        dimensions = (
            [float(value) for value in dimensions_m]
            if dimensions_m is not None
            else transform_lengths
        )
        if any(not math.isfinite(length) or length <= 1e-9 for length in dimensions):
            return result(None, "rawpart_orientation_invalid")
        expected_m = float(expected_contact_width_mm) / 1000.0
        matching_edges = [
            index
            for index in range(3)
            if abs(dimensions[index] - expected_m) * 1000.0
            <= contact_width_tolerance_mm
        ]
        # Motion's selected Closing Axis identifies the opposing contact-face
        # pair. A world-up/top-face classification would reject valid tilted
        # side-face grasps, so shape validity is based on the box frame itself.
        if not matching_edges:
            return result(None, "contact_width_tolerance_exceeded")
        tcp_closing = StageCaptureQuery._transform_direction(tcp_world, (1.0, 0.0, 0.0))
        closing_error = min(
            StageCaptureQuery._axis_error_deg(
                tcp_closing, directions[index], sign_invariant=True
            )
            for index in matching_edges
        )
        return result(closing_error)

    @staticmethod
    def _canonical_capture_reference(target_world, config):
        top_normal = StageCaptureQuery._normalized_direction(
            StageCaptureQuery._transform_direction(target_world, (0.0, 0.0, 1.0))
        )
        height = float(config.fixture_dimensions_m[2])
        offset = float(config.observation_to_grasp_tcp_m)
        center = target_world.ExtractTranslation()
        position = tuple(
            float(center[index]) + top_normal[index] * (height / 2.0 + offset)
            for index in range(3)
        )
        return CanonicalCaptureReference(position, top_normal)

    def _world_transform(self, prim_path):
        _, _, usd, usd_geom, _ = _pxr_modules()
        prim = self.stage.GetPrimAtPath(prim_path)
        return usd_geom.Xformable(prim).ComputeLocalToWorldTransform(
            usd.TimeCode.Default()
        )

    @staticmethod
    def _authored_dimensions(prim):
        get_attribute = getattr(prim, "GetAttribute", None)
        if not callable(get_attribute):
            return None
        attribute = get_attribute("sf_twin:dimensions_m")
        if attribute is None or not attribute.IsValid():
            return None
        values = attribute.Get()
        if values is None or len(values) != 3:
            return None
        dimensions = tuple(float(value) for value in values)
        if not all(math.isfinite(value) and value > 0.0 for value in dimensions):
            return None
        return dimensions

    @staticmethod
    def _pose_dict(transform):
        return _stage_pose_dict(transform)

    @staticmethod
    def _relative_quaternion(parent, child):
        pw = float(parent.GetReal())
        px, py, pz = (float(value) for value in parent.GetImaginary())
        cw = float(child.GetReal())
        cx, cy, cz = (float(value) for value in child.GetImaginary())
        return (
            pw * cw + px * cx + py * cy + pz * cz,
            pw * cx - px * cw - py * cz + pz * cy,
            pw * cy + px * cz - py * cw - pz * cx,
            pw * cz - px * cy + py * cx - pz * cw,
        )

    @staticmethod
    def _orientation_error_deg(
        tcp_world,
        target_world,
        insertion_axis_tcp=(0.0, 0.0, 1.0),
        *,
        expected_contact_width_mm=None,
        orientation_symmetry_order=2,
        orientation_tolerance_deg=5.0,
        contact_width_tolerance_mm=2.0,
    ):
        """Compare TCP pose with the selected grasp and contact geometry."""
        if expected_contact_width_mm is not None:
            return StageCaptureQuery._rawpart_orientation_error_deg(
                tcp_world,
                target_world,
                expected_contact_width_mm,
                orientation_symmetry_order,
                orientation_tolerance_deg,
                contact_width_tolerance_mm,
            )
        # Preserve the configured legacy reference path for profiles without
        # a Motion-selected Capture Reference.
        approach_error = StageCaptureQuery._axis_error_deg(
            StageCaptureQuery._transform_direction(tcp_world, insertion_axis_tcp),
            tuple(
                -value
                for value in StageCaptureQuery._transform_direction(
                    target_world, (0.0, 0.0, 1.0)
                )
            ),
        )
        closing_axis = StageCaptureQuery._transform_direction(
            tcp_world, (1.0, 0.0, 0.0)
        )
        closing_error = min(
            StageCaptureQuery._axis_error_deg(
                closing_axis,
                StageCaptureQuery._transform_direction(target_world, (1.0, 0.0, 0.0)),
                sign_invariant=True,
            ),
            StageCaptureQuery._axis_error_deg(
                closing_axis,
                StageCaptureQuery._transform_direction(target_world, (0.0, 1.0, 0.0)),
                sign_invariant=True,
            ),
        )
        return max(approach_error, closing_error)

    @staticmethod
    def _normalized_direction(direction):
        norm = math.sqrt(sum(float(value) ** 2 for value in direction))
        if norm <= 1e-12:
            raise ValueError("transform direction must be non-zero")
        return tuple(float(value) / norm for value in direction)

    @staticmethod
    def _transform_direction(transform, axis):
        try:
            from pxr import Gf

            axis_value = Gf.Vec3d(*axis)
        except ImportError:
            axis_value = axis
        direction = transform.TransformDir(axis_value)
        return tuple(float(direction[index]) for index in range(3))

    @staticmethod
    def _axis_error_deg(first, second, *, sign_invariant=False):
        first_norm = math.sqrt(sum(float(value) ** 2 for value in first))
        second_norm = math.sqrt(sum(float(value) ** 2 for value in second))
        if first_norm <= 1e-12 or second_norm <= 1e-12:
            return 180.0
        dot = sum(float(a) * float(b) for a, b in zip(first, second))
        dot = dot / (first_norm * second_norm)
        if sign_invariant:
            dot = abs(dot)
        dot = max(-1.0, min(1.0, dot))
        return math.degrees(math.acos(dot))
