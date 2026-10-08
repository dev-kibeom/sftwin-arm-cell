import logging
import math
import time
from dataclasses import dataclass
from enum import Enum

from gripper_runtime.grasp_policy import (
    CaptureCandidate,
    CaptureDecision,
    CapturePolicy,
)
from gripper_runtime.holding_scenario import observe_with_scenario

LOGGER = logging.getLogger(__name__)


def _pxr_modules():
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    return Gf, Sdf, Usd, UsdGeom, UsdPhysics


class GraspState(Enum):
    EMPTY = "empty"
    SEEKING = "seeking"
    CONTACT = "contact"
    GRASPED = "grasped"


class HoldingState(Enum):
    HELD = "held"
    RELEASED = "released"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class HoldingObservation:
    state: HoldingState
    observed_at: float
    sequence: int

    def fresh(self, now=None, max_age=0.5):
        now = time.monotonic() if now is None else now
        return (
            self.state is not HoldingState.UNKNOWN
            and self.sequence > 0
            and now >= self.observed_at
            and now - self.observed_at <= max_age
        )


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

    def register(self, prim_path):
        self._paths.add(str(prim_path))

    def unregister(self, prim_path):
        self._paths.discard(str(prim_path))

    def paths(self):
        return tuple(sorted(self._paths))


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


class StageCaptureQuery:
    """Convert current USD stage observations into pure-policy candidates."""

    def __init__(self, stage, tcp_path, registry, gripper, reference_config=None):
        self.stage = stage
        self.tcp_path = str(tcp_path)
        self.registry = registry
        self.gripper = gripper
        self.reference_config = None
        self._last_reference_diagnostics = None
        if reference_config is not None:
            self.configure_reference(reference_config)

    def configure_reference(self, reference_config):
        self.reference_config = reference_config.validate()
        return True

    def reference_diagnostics(self):
        return self._last_reference_diagnostics

    def query(self):
        if self.reference_config is None:
            self._last_reference_diagnostics = None
            return False, []
        tcp_prim = self.stage.GetPrimAtPath(self.tcp_path)
        if not tcp_prim.IsValid():
            return False, []

        tcp_world = self._world_transform(self.tcp_path)
        candidates = []
        for path in self.registry.paths():
            target_prim = self.stage.GetPrimAtPath(path)
            if not target_prim.IsValid():
                return False, []

            target_world = self._world_transform(path)
            reference = self._canonical_capture_reference(
                target_world, self.reference_config
            )
            target_position = target_world.ExtractTranslation()
            tcp_position = tcp_world.ExtractTranslation()
            delta = tuple(
                float(tcp_position[index]) - reference.position[index]
                for index in range(3)
            )
            position_error = math.sqrt(sum(float(value) ** 2 for value in delta))
            orientation_error = self._orientation_error_deg(
                tcp_world,
                target_world,
                self.reference_config.insertion_axis_tcp,
            )
            local_position = tcp_world.GetInverse().Transform(target_position)
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
                    measured_opening_mm=float(self.gripper.get_width()),
                )
            )
            self._last_reference_diagnostics = {
                "raw_candidate_position": tuple(
                    float(value) for value in target_position
                ),
                "canonical_capture_position": reference.position,
                "canonical_top_normal": reference.top_normal,
            }
        return True, candidates

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
    def _orientation_error_deg(
        tcp_world, target_world, insertion_axis_tcp=(0.0, 0.0, 1.0)
    ):
        """Return error for the cube's top-grasp axes, not raw frame rotation.

        The TCP +Z axis approaches the cube top face and its +X axis is the
        Robotiq closing direction.  The cube's local +Z is its top normal;
        either top-face edge (+X or +Y) is valid for the symmetric cube.
        """
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


class UsdFixedJointOwner:
    """Own and mutate only the adapter-created fixed joint."""

    JOINT_NAME = "sf_scripted_grasp_joint"

    def __init__(self, stage, tcp_path):
        self.stage = stage
        self.tcp_path = str(tcp_path)
        self._owned_joint_path = None
        self._target_path = None

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
        target_prim = self.stage.GetPrimAtPath(target_path)
        tcp_prim = self.stage.GetPrimAtPath(self.tcp_path)
        if not target_prim.IsValid() or not tcp_prim.IsValid():
            raise RuntimeError("TCP or candidate prim became invalid before attach")

        tcp_world = self._world_transform(self.tcp_path)
        target_world = self._world_transform(target_path)
        local_pos0, local_rot0 = _world_frame_relative_to_body(
            tcp_world,
            target_world,
        )

        gf, sdf, _, _, usd_physics = _pxr_modules()
        joint_path = sdf.Path(target_path).AppendChild(self.JOINT_NAME)
        existing = self.stage.GetPrimAtPath(joint_path)
        if existing.IsValid():
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
        return True

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


class GraspManager:
    """Isaac-side capture-local attachment and runtime holding adapter."""

    DEFAULT_TCP_NAME = "sf_grasp_tcp"

    def __init__(
        self,
        gripper,
        tcp_path=None,
        registry=None,
        config=None,
        stage=None,
        candidate_source=None,
        capture_reference_config=None,
        joint_owner=None,
        pre_capture_stability=None,
        verbose=False,
    ):
        if stage is None and (candidate_source is None or joint_owner is None):
            import omni.usd

            stage = omni.usd.get_context().get_stage()
        self.stage = stage
        self.gripper = gripper
        self.verbose = verbose
        self.config = None
        self.ready = False
        if config is not None:
            try:
                self.config = config.validate()
                self.ready = True
            except (TypeError, ValueError):
                self.config = None
        self.registry = registry or EligibleObjectRegistry()

        if tcp_path is None:
            tcp_path = self._discover_tcp_path()
        self.tcp_path = str(tcp_path)

        self.candidate_source = candidate_source or StageCaptureQuery(
            self.stage,
            self.tcp_path,
            self.registry,
            self.gripper,
            reference_config=None,
        )
        if capture_reference_config is not None:
            self.configure_capture_reference(capture_reference_config)
        self.joint_owner = joint_owner or UsdFixedJointOwner(
            self.stage,
            self.tcp_path,
        )
        self.pre_capture_stability = pre_capture_stability

        self.state = GraspState.EMPTY
        self.object_detected = False
        self.resistance = False
        self.last_position_error_m = None
        self.last_orientation_error_deg = None
        self.last_width_error_mm = None
        self._capture_transition_failed = False
        self._capture_transaction_armed = False
        self._holding = HoldingObservation(
            HoldingState.UNKNOWN,
            time.monotonic(),
            1,
        )
        self._holding_sequence = 1
        self._last_holding_semantics = self._semantic_signature(HoldingState.UNKNOWN)

    def configure_capture_reference(self, reference_config):
        configure = getattr(self.candidate_source, "configure_reference", None)
        if not callable(configure):
            return False
        return bool(configure(reference_config))

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
                f"Expected exactly one '{self.DEFAULT_TCP_NAME}', "
                f"found {len(candidates)}"
            )
        return candidates[0]

    def _record(self, state):
        previous_state = self._holding.state
        self._holding = HoldingObservation(
            state,
            time.monotonic(),
            self._holding_sequence,
        )
        if state is not previous_state:
            LOGGER.info(
                "gripper holding state changed from=%s to=%s",
                previous_state.value,
                state.value,
            )
            LOGGER.debug(
                "gripper holding transition diagnostics width_mm=%.3f",
                self.gripper.get_width(),
            )

    def note_gripper_command_applied(self):
        """Start a fresh status observation after a newly applied command."""
        self._holding_sequence += 1
        self._holding = HoldingObservation(
            self._holding.state,
            time.monotonic(),
            self._holding_sequence,
        )
        return self._holding_sequence

    def _semantic_signature(self, state):
        # Keep this tuple aligned with GripperRuntimeSession's published flags;
        # actuator width is live telemetry, not holding identity.
        ready = bool(self.ready)
        held = ready and state is HoldingState.HELD
        released = ready and state is HoldingState.RELEASED
        return (ready, held, held, released)

    def restore_holding_sequence(self, sequence):
        """Continue the monotonic status sequence after a runtime replacement."""
        sequence = int(sequence)
        if sequence < 0:
            raise ValueError("holding sequence must be non-negative")
        if sequence > self._holding_sequence:
            self._holding_sequence = sequence
            self._holding = HoldingObservation(
                self._holding.state,
                time.monotonic(),
                sequence,
            )
        return self._holding_sequence

    def holding_observation(self):
        base_observation = self._holding
        observation = observe_with_scenario(
            base_observation,
            unknown_factory=lambda observed_at, sequence: HoldingObservation(
                HoldingState.UNKNOWN, observed_at, sequence
            ),
        )
        semantics = self._semantic_signature(observation.state)
        if semantics != self._last_holding_semantics:
            self._holding_sequence += 1
            self._last_holding_semantics = semantics
        if base_observation.sequence != self._holding_sequence:
            self._holding = HoldingObservation(
                base_observation.state,
                base_observation.observed_at,
                self._holding_sequence,
            )
        if observation is base_observation:
            return self._holding
        return HoldingObservation(
            observation.state,
            observation.observed_at,
            self._holding_sequence,
        )

    def arm_close_transaction(self):
        """Arm capture for a newly accepted close command."""
        if not self.ready or self.joint_owner.has_owned_joint():
            return False
        self._capture_transition_failed = False
        self._capture_transaction_armed = True
        return True

    def reconcile_stale_owned_joint(self):
        reconcile = getattr(self.joint_owner, "reconcile_stale", None)
        if not callable(reconcile):
            return True
        try:
            result = bool(reconcile())
        except Exception:
            result = False
        if not result:
            self.ready = False
            self._mark_unknown()
            LOGGER.warning(
                "gripper runtime unready reason=stale_attachment_reconcile_failed"
            )
        return result

    def shutdown(self):
        self._capture_transaction_armed = False
        cleanup_ok = True
        if self.joint_owner.has_owned_joint():
            cleanup_ok = (
                self.joint_owner.owned_joint_valid()
                and self.joint_owner.remove_owned()
                and not self.joint_owner.has_owned_joint()
            )
            if cleanup_ok:
                cleanup_ok = self.reconcile_stale_owned_joint()
        else:
            cleanup_ok = self.reconcile_stale_owned_joint()
        self.ready = False
        self._mark_unknown()
        return cleanup_ok

    def _mark_unknown(self):
        self.state = GraspState.SEEKING
        self.object_detected = False
        self.resistance = False
        self._record(HoldingState.UNKNOWN)

    def _log_capture_success(self, actuator_state, candidate):
        target_position = getattr(self.gripper, "target_position", None)
        position_to_width = getattr(type(self.gripper), "position_to_width", None)
        commanded_width = (
            position_to_width(target_position)
            if target_position is not None and callable(position_to_width)
            else None
        )
        actual_width = self.gripper.get_width()
        width_error = (
            abs(actual_width - self.config.expected_contact_width_mm)
            if self.config is not None
            else None
        )
        reference_diagnostics = getattr(
            self.candidate_source, "reference_diagnostics", lambda: None
        )()
        LOGGER.debug(
            "gripper capture succeeded actuator_state=%s candidate=%s "
            "position_error_m=%.6f position_tolerance_m=%.6f "
            "orientation_error_deg=%.3f orientation_tolerance_deg=%.3f "
            "commanded_width_mm=%s observed_width_mm=%.3f width_error_mm=%s "
            "width_tolerance_mm=%.3f sequence=%d reference=%s",
            actuator_state,
            candidate.prim_path,
            candidate.position_error_m,
            self.config.position_tolerance_m,
            candidate.orientation_error_deg,
            self.config.orientation_tolerance_deg,
            commanded_width,
            actual_width,
            width_error,
            self.config.contact_width_tolerance_mm,
            self._holding_sequence,
            reference_diagnostics,
        )

    def _attempt_capture(self, actuator_state, runtime_available, candidates):
        result = CapturePolicy.evaluate(
            close_active=True,
            candidates=candidates,
            config=self.config,
            runtime_available=runtime_available,
        )
        if result.decision is CaptureDecision.CAPTURE_ELIGIBLE:
            self.state = GraspState.CONTACT
            self.resistance = True
            self.gripper.stop()
            try:
                created = self.joint_owner.create(result.candidate.prim_path)
            except Exception as error:
                LOGGER.warning(
                    "gripper capture failed reason=attachment_create_failed "
                    "candidate=%s error_type=%s error=%s",
                    result.candidate.prim_path,
                    type(error).__name__,
                    error,
                )
                self._capture_transition_failed = True
                self._capture_transaction_armed = False
                self._mark_unknown()
                return self.state
            if not created or not self.joint_owner.owned_joint_valid():
                LOGGER.warning(
                    "gripper capture failed reason=attachment_ownership_validation "
                    "candidate=%s created=%s",
                    result.candidate.prim_path,
                    created,
                )
                self._capture_transition_failed = True
                self._capture_transaction_armed = False
                self._mark_unknown()
                return self.state
            if not self._release_for_capture(result.candidate.prim_path):
                LOGGER.warning(
                    "gripper capture failed reason=contact_policy_transition "
                    "candidate=%s",
                    result.candidate.prim_path,
                )
                cleanup_ok = self._detach_owned_joint()
                if not cleanup_ok:
                    self.ready = False
                self._capture_transition_failed = True
                self._capture_transaction_armed = False
                self._mark_unknown()
                return self.state
            self._capture_transaction_armed = False
            self.state = GraspState.GRASPED
            self.object_detected = True
            self._record(HoldingState.HELD)
            self._log_capture_success(actuator_state, result.candidate)
            return self.state

        if result.decision in (
            CaptureDecision.AMBIGUOUS,
            CaptureDecision.INVALID_CONFIG,
            CaptureDecision.UNKNOWN_RUNTIME,
        ):
            LOGGER.warning(
                "gripper capture rejected decision=%s actuator_state=%s "
                "candidate_count=%d sequence=%d",
                result.decision.value,
                actuator_state,
                len(candidates),
                self._holding_sequence,
            )
            self._capture_transition_failed = True
            self._capture_transaction_armed = False
            self._mark_unknown()
            return self.state

        self.state = GraspState.SEEKING
        self.object_detected = False
        self.resistance = False
        if actuator_state == "closing":
            # A close transaction can legitimately be ineligible while the
            # fingers are still moving toward the expected contact width.
            # Keep the prior semantic holding observation until either HELD
            # or a terminal close-completion result is available.
            return self.state
        self._record(HoldingState.UNKNOWN)
        return self.state

    def _detach_owned_joint(self):
        if not self.joint_owner.has_owned_joint():
            return True
        if not self.joint_owner.owned_joint_valid():
            return False
        removed = self.joint_owner.remove_owned()
        return removed and not self.joint_owner.has_owned_joint()

    def _release_for_capture(self, target_path):
        transition = self.pre_capture_stability
        if transition is None:
            return True
        try:
            return bool(transition.release_for_capture(str(target_path)))
        except Exception:
            return False

    def _record_released_if_runtime_fresh(self):
        runtime_available, _ = self.candidate_source.query()
        if not runtime_available or self.joint_owner.has_owned_joint():
            self._mark_unknown()
            return False
        self.state = GraspState.SEEKING
        self.resistance = False
        self.object_detected = False
        self._record(HoldingState.RELEASED)
        return True

    def update(self, dt):
        del dt

        if not self.ready:
            self._capture_transaction_armed = False
            self._mark_unknown()
            return self.state

        gripper_state = getattr(self.gripper.state, "value", "")
        if self._capture_transition_failed and gripper_state != "opening":
            self._capture_transaction_armed = False
            return self.state
        if gripper_state == "opening":
            self._capture_transaction_armed = False
            self._capture_transition_failed = False
            runtime_available, _ = self.candidate_source.query()
            if not runtime_available:
                self._mark_unknown()
                return self.state
            if not self._detach_owned_joint():
                self._mark_unknown()
                return self.state
            self._record_released_if_runtime_fresh()
            return self.state

        if self.joint_owner.has_owned_joint():
            self._capture_transaction_armed = False
            if not self.joint_owner.owned_joint_valid():
                self._mark_unknown()
            else:
                self.state = GraspState.GRASPED
                self.object_detected = True
                self.resistance = True
                self._record(HoldingState.HELD)
            return self.state

        if gripper_state == "closing" and self._capture_transaction_armed:
            runtime_available, candidates = self.candidate_source.query()
            return self._attempt_capture(gripper_state, runtime_available, candidates)

        if gripper_state == "closing":
            self.state = GraspState.SEEKING
            self.object_detected = False
            self.resistance = False
            self._record(HoldingState.UNKNOWN)
            return self.state

        if gripper_state == "holding" and self._capture_transaction_armed:
            runtime_available, candidates = self.candidate_source.query()
            result_state = self._attempt_capture(
                gripper_state, runtime_available, candidates
            )
            self._capture_transaction_armed = False
            if not self.joint_owner.has_owned_joint():
                self._capture_transition_failed = True
                self._capture_transaction_armed = False
            return result_state

        if gripper_state != "closing":
            self.state = GraspState.SEEKING
            self.object_detected = False
            self.resistance = False
            runtime_available, _ = self.candidate_source.query()
            if runtime_available:
                self._record(HoldingState.RELEASED)
            else:
                self._record(HoldingState.UNKNOWN)
            return self.state

        return self.state

    def release(self, open_gripper=True, duration=None):
        self._capture_transaction_armed = False
        if not self.ready:
            self._mark_unknown()
            return
        if open_gripper:
            self.gripper.open(duration=duration)
        runtime_available, _ = self.candidate_source.query()
        if not runtime_available:
            self._mark_unknown()
            return
        if not self._detach_owned_joint():
            self._mark_unknown()
            return
        self._record_released_if_runtime_fresh()

    @property
    def is_grasped(self):
        return self._holding.state is HoldingState.HELD

    @property
    def attach_joint_path(self):
        return getattr(self.joint_owner, "owned_joint_path", None)

    def diagnostics(self):
        holding = self.holding_observation()
        return {
            "state": self.state.value,
            "tcp_path": self.tcp_path,
            "object_detected": self.object_detected,
            "resistance": self.resistance,
            "position_error_m": self.last_position_error_m,
            "orientation_error_deg": self.last_orientation_error_deg,
            "width_error_mm": self.last_width_error_mm,
            "gripper_width_mm": self.gripper.get_width(),
            "attach_joint_path": self.attach_joint_path,
            "holding_state": holding.state.value,
            "holding_sequence": holding.sequence,
        }
