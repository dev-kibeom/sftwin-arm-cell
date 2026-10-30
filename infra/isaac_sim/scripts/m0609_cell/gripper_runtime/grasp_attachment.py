import logging
import math
import time
from dataclasses import dataclass
from enum import Enum

from gripper_runtime.capture_diagnostics import CaptureDiagnosticRecord
from gripper_runtime.grasp_joint import UsdFixedJointOwner
from gripper_runtime.grasp_qualification import (
    GraspQualification,
    GripperCharacteristics,
)
from gripper_runtime.grasp_policy import (
    CaptureAssessment,
    CaptureCandidate,
    CaptureDecision,
    CapturePolicy,
    CaptureResult,
)
from gripper_runtime.grasp_stage import (
    CanonicalCaptureReference,
    CaptureReferenceConfig,
    EligibleObjectRegistry,
    StageCaptureQuery,
)
from gripper_runtime.holding_scenario import observe_with_scenario

LOGGER = logging.getLogger(__name__)

__all__ = [
    "CanonicalCaptureReference",
    "CaptureCandidate",
    "CaptureDecision",
    "CapturePolicy",
    "CaptureReferenceConfig",
    "EligibleObjectRegistry",
    "GraspManager",
    "GraspState",
    "HoldingObservation",
    "HoldingState",
    "StageCaptureQuery",
    "UsdFixedJointOwner",
]


class GraspState(Enum):
    EMPTY = "empty"
    SEEKING = "seeking"
    ATTACHING = "attaching"
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
        # Capture configuration constrains CLOSE, not physical OPEN/release.
        # Lifecycle/attachment cleanup failures still invalidate the runtime.
        self.ready = True
        if config is not None:
            try:
                self.config = config.validate()
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
        self._close_transaction_failure_reason = None
        self._last_release_command_generation = None
        self._pending_release_command = None
        self._release_detach_event = None
        self.capture_reference_id = None
        self._holding = HoldingObservation(
            HoldingState.UNKNOWN,
            time.monotonic(),
            1,
        )
        self._holding_sequence = 1
        self._last_holding_semantics = self._semantic_signature(HoldingState.UNKNOWN)
        self._diagnostic_log_incidents = {}
        self.capture_diagnostic_writer = None
        self.physx_observer = None
        self.capture_diagnostic_record = None
        self.capture_diagnostic_target_path = None
        self._close_command_generation = None

    def set_diagnostic_observers(self, writer=None, physx_observer=None):
        self.capture_diagnostic_writer = writer
        self.physx_observer = physx_observer

    def _report_diagnostic_log_incident(self, operation, error=None):
        previous = self._diagnostic_log_incidents.get(operation)
        if error is None:
            if previous is not None:
                LOGGER.debug("Capture diagnostic %s recovered", operation)
                self._diagnostic_log_incidents.pop(operation, None)
            return
        cause = (type(error).__name__, str(error))
        if previous != cause:
            LOGGER.warning(
                "Capture diagnostic %s failed; capture result is unchanged: %s",
                operation,
                error,
            )
            self._diagnostic_log_incidents[operation] = cause

    def _diagnostic_append(self, kind, payload, **kwargs):
        record = self.capture_diagnostic_record
        if record is None:
            return
        try:
            record.append(kind, payload, **kwargs)
            self._report_diagnostic_log_incident("observation")
        except Exception as error:
            self._report_diagnostic_log_incident("observation", error)

    def _ensure_capture_diagnostic_record(self, motion_goal_uuid):
        writer = self.capture_diagnostic_writer
        if (
            writer is None
            or not isinstance(motion_goal_uuid, str)
            or not motion_goal_uuid
        ):
            return None
        record = self.capture_diagnostic_record
        if (
            record is not None
            and not record._finalized
            and record.motion_goal_uuid != motion_goal_uuid
        ):
            self._diagnostic_outcome("capture_reference_replaced")
            record = self.capture_diagnostic_record
        if (
            record is None
            or record._finalized
            or record.motion_goal_uuid != motion_goal_uuid
        ):
            self.capture_diagnostic_record = CaptureDiagnosticRecord(
                writer, motion_goal_uuid, motion_goal_uuid
            )
        return self.capture_diagnostic_record

    def _record_capture_decision(self, candidates, result):
        writer = self.capture_diagnostic_writer
        if writer is None:
            return
        try:
            goal_uuid = self.capture_reference_id
            record = self.capture_diagnostic_record
            if (
                record is None
                or record._finalized
                or record.motion_goal_uuid != goal_uuid
            ):
                record = self._ensure_capture_diagnostic_record(goal_uuid)
            if record is None:
                return
            record = self.capture_diagnostic_record
            attempt = record.data["coverage"].get("capture_attempts", 0) + 1
            record.data["coverage"]["capture_attempts"] = attempt
            config = self.config
            limits = {
                "capture_volume_dimensions_m": getattr(
                    config, "capture_volume_dimensions_m", None
                ),
            }
            if attempt <= 8:
                stage_candidates = [
                    {
                        "prim_path": candidate.prim_path,
                        "eligible": candidate.eligible,
                        "shape": candidate.shape,
                        "position_error_m": candidate.position_error_m,
                        "orientation_error_deg": candidate.orientation_error_deg,
                        "expected_orientation_error_deg": candidate.expected_orientation_error_deg,
                        "rawpart_orientation_error_deg": candidate.rawpart_orientation_error_deg,
                        "rawpart_rejection_reason": candidate.rawpart_rejection_reason,
                        "measured_opening_mm": candidate.measured_opening_mm,
                        "expected_contact_width_mm": (
                            candidate.capture_reference.get("expected_contact_width_mm")
                            if candidate.capture_reference
                            else None
                        ),
                        "tcp_stage_world_pose": candidate.tcp_stage_world_pose
                        or {
                            "status": "unavailable",
                            "reason": "StageCaptureQuery did not provide pose",
                        },
                        "rawpart_stage_world_pose": candidate.stage_world_pose
                        or {
                            "status": "unavailable",
                            "reason": "StageCaptureQuery did not provide pose",
                        },
                        "relative_pose": candidate.relative_pose
                        or {
                            "status": "unavailable",
                            "reason": "candidate relative pose unavailable",
                        },
                        "capture_reference": candidate.capture_reference
                        or {
                            "status": "unavailable",
                            "reason": "no Capture Reference applied",
                        },
                    }
                    for candidate in candidates[:64]
                ]
                candidate_coverage = {
                    "candidate_count": len(candidates),
                    "candidates_omitted": max(
                        0, len(candidates) - len(stage_candidates)
                    ),
                    "candidates": stage_candidates,
                    **limits,
                }
            else:
                record.data["coverage"]["capture_attempts_omitted"] = attempt - 8
                candidate_coverage = {
                    "candidate_count": len(candidates),
                    "status": "unavailable",
                    "reason": "bounded detail limit of eight capture attempts per Motion Goal reached",
                    **limits,
                }
            query_diagnostics = getattr(
                self.candidate_source, "reference_diagnostics", lambda: None
            )()
            reference_frame = next(
                (
                    item.capture_reference.get("frame_id")
                    for item in candidates
                    if item.capture_reference and item.capture_reference.get("frame_id")
                ),
                None,
            )
            if reference_frame is None and isinstance(query_diagnostics, dict):
                reference_frame = query_diagnostics.get("frame_id")
            record.append(
                "observation",
                {
                    "attempt": attempt,
                    "source": "usd_stage_and_capture_decision_inputs",
                    "capture_reference": {
                        "motion_goal_uuid": goal_uuid,
                        "reference_id": goal_uuid,
                        "frame_id": reference_frame,
                        "applied_frame": next(
                            (
                                item.capture_reference.get("applied_frame")
                                for item in candidates
                                if item.capture_reference
                                and item.capture_reference.get("applied_frame")
                            ),
                            None,
                        )
                        or {
                            "status": "unavailable",
                            "reason": (
                                query_diagnostics.get("rejection_reason")
                                if isinstance(query_diagnostics, dict)
                                else "no candidate Capture Reference frame was applied"
                            ),
                        },
                    },
                    "query_diagnostics": query_diagnostics,
                    **candidate_coverage,
                },
            )
            assessments = list(result.assessments[:64])
            record.append(
                "decision",
                {
                    "attempt": attempt,
                    "policy": "CapturePolicy.evaluate",
                    "decision": result.decision.value,
                    "selected_candidate": (
                        result.candidate.prim_path
                        if result.candidate is not None
                        else None
                    ),
                    "assessments": [assessment.__dict__ for assessment in assessments],
                    "assessments_omitted": max(
                        0, len(result.assessments) - len(assessments)
                    ),
                    "failure_reason": (
                        None
                        if result.decision is CaptureDecision.CAPTURE_ELIGIBLE
                        else self._capture_rejection_reason(result)
                    ),
                },
            )
            self._report_diagnostic_log_incident("decision recording")
        except Exception as error:
            # Diagnostic serialization must never affect the Capture decision.
            self._report_diagnostic_log_incident("decision recording", error)

    def _diagnostic_physx_pair(self, target_path):
        if self.physx_observer is None:
            return {
                "source": "physx_rigid_body",
                "status": "unavailable",
                "reason": "PhysX observer was not configured in this runtime session",
                "tcp": {
                    "status": "unavailable",
                    "reason": "PhysX observer was not configured",
                },
                "rawpart": {
                    "status": "unavailable",
                    "reason": "PhysX observer was not configured",
                },
            }
        try:
            return self.physx_observer.observe_pair(self.tcp_path, target_path)
        except Exception as error:
            return {
                "source": "physx_rigid_body",
                "status": "unavailable",
                "reason": f"{type(error).__name__}: {error}",
            }

    def _diagnostic_outcome(
        self,
        reason,
        finalize=True,
        *,
        capture_decision=None,
        capture_result=None,
        transition_reason=None,
    ):
        record = self.capture_diagnostic_record
        if record is None:
            return
        try:
            observation = self._holding
            record.set_outcome(
                observation.state.value,
                reason,
                observation.sequence,
                observation.observed_at,
                capture_decision=(
                    capture_decision.value
                    if isinstance(capture_decision, CaptureDecision)
                    else capture_decision
                ),
                capture_result=capture_result,
                transition_reason=transition_reason,
            )
            if finalize:
                self.finalize_capture_diagnostic(reason)
            self._report_diagnostic_log_incident("outcome")
        except Exception as error:
            self._report_diagnostic_log_incident("outcome", error)

    def finalize_capture_diagnostic(self, reason=None):
        record = self.capture_diagnostic_record
        if record is None:
            return False
        try:
            if self.capture_diagnostic_target_path is not None:
                expected = int(record.data["coverage"].get("physics_step_limit", 4))
                observed = int(record.data["coverage"].get("physics_steps_observed", 0))
                if observed < expected:
                    record.data["coverage"]["physics_steps_unavailable"] = {
                        "count": expected - observed,
                        "reason": reason
                        or "capture ended before bounded observation window completed",
                    }
            observation = self._holding
            record.set_outcome(
                observation.state.value,
                reason or (record._outcome or {}).get("reason", "capture_complete"),
                observation.sequence,
                observation.observed_at,
            )
            queued = record.finalize()
            self.capture_diagnostic_record = None
            self.capture_diagnostic_target_path = None
            self._report_diagnostic_log_incident("finalization")
            return queued
        except Exception as error:
            self._report_diagnostic_log_incident("finalization", error)
            self.capture_diagnostic_record = None
            self.capture_diagnostic_target_path = None
            return False

    def configure_capture_reference(self, reference_config):
        configure = getattr(self.candidate_source, "configure_reference", None)
        if not callable(configure):
            return False
        return bool(configure(reference_config))

    def configure_selected_grasp_reference(self, values):
        reference_id = None
        frame_id = None
        if isinstance(values, dict):
            reference_id = values.get("motion_goal_uuid")
            frame_id = values.get("frame_id")
            if values.get("invalidate") is True:
                if reference_id == self.capture_reference_id:
                    self._diagnostic_outcome("capture_reference_invalidated")
                    clear = getattr(
                        self.candidate_source, "clear_expected_tcp_reference", None
                    )
                    if callable(clear):
                        clear()
                    self.capture_reference_id = None
                return False
            values = values.get("values")
        if (
            self.capture_reference_id
            and reference_id
            and reference_id != self.capture_reference_id
        ):
            self._diagnostic_outcome("capture_reference_replaced")
        if values is None or len(values) != 9:
            invalidate = getattr(
                self.candidate_source, "invalidate_expected_tcp_reference", None
            )
            if callable(invalidate):
                invalidate("capture_reference_unavailable", frame_id)
            self.capture_reference_id = reference_id
            return True
        configure = getattr(
            self.candidate_source, "configure_expected_tcp_reference", None
        )
        if not callable(configure):
            return False
        self.capture_reference_id = reference_id
        try:
            position = values[0:3]
            orientation = values[3:7]
            width_mm = float(values[7])
            symmetry_order = int(values[8])
            configure(frame_id, position, orientation, symmetry_order, width_mm)
        except (TypeError, ValueError, ArithmeticError):
            invalidate = getattr(
                self.candidate_source, "invalidate_expected_tcp_reference", None
            )
            if callable(invalidate):
                invalidate("capture_reference_unavailable", frame_id)
        return True

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

    def _record(self, state, reason=None):
        previous_state = self._holding.state
        self._holding = HoldingObservation(
            state,
            time.monotonic(),
            self._holding_sequence,
        )
        if state is not previous_state:
            self._diagnostic_append(
                "observation",
                {
                    "phase": "holding_transition",
                    "from": previous_state.value,
                    "to": state.value,
                    "sequence": self._holding_sequence,
                    **({"reason": str(reason)} if reason else {}),
                },
            )
            LOGGER.info(
                "gripper holding state changed from=%s to=%s reason=%s",
                previous_state.value,
                state.value,
                reason or "unspecified",
            )
            LOGGER.debug("Gripper holding transition observed")
            return True
        return False

    def note_gripper_command_applied(self):
        """Start a fresh status observation after a newly applied command."""
        self._pending_release_command = None
        self._holding_sequence += 1
        self._holding = HoldingObservation(
            self._holding.state,
            time.monotonic(),
            self._holding_sequence,
        )
        return self._holding_sequence

    def note_open_release_command_applied(self, generation, width_mm):
        """Remember one newly applied open command as a release intent."""
        try:
            generation = int(generation)
            width_mm = float(width_mm)
        except (TypeError, ValueError):
            return False
        if generation == self._last_release_command_generation:
            return False
        self._last_release_command_generation = generation
        self._pending_release_command = (generation, width_mm)
        return True

    def consume_release_detach_event(self):
        event = self._release_detach_event
        self._release_detach_event = None
        return event

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

    def arm_close_transaction(self, execution_id=None, command_generation=None):
        """Arm capture for a newly accepted close command."""
        self._close_transaction_failure_reason = None
        self._ensure_capture_diagnostic_record(
            self.capture_reference_id or execution_id
        )
        if not self.ready or self.joint_owner.has_owned_joint():
            self._close_transaction_failure_reason = (
                "runtime_unavailable" if not self.ready else "attachment_already_owned"
            )
            self._diagnostic_append(
                "decision",
                {
                    "phase": "close_transaction_arm",
                    "result": "rejected",
                    "close_requested": True,
                    "reason": self._close_transaction_failure_reason,
                    "command_generation": command_generation,
                },
            )
            return False
        if self.config is None:
            self._close_transaction_failure_reason = "capture_configuration_unavailable"
            self._capture_transition_failed = True
            self._diagnostic_append(
                "decision",
                {
                    "phase": "close_transaction_arm",
                    "result": "rejected",
                    "close_requested": True,
                    "reason": self._close_transaction_failure_reason,
                    "command_generation": command_generation,
                },
            )
            return False
        self._capture_transition_failed = False
        self._capture_transaction_armed = True
        self._close_command_generation = command_generation
        self._diagnostic_append(
            "observation",
            {
                "phase": "close_transaction_arm",
                "result": "accepted",
                "close_requested": True,
                "command_generation": command_generation,
                "execution_id": execution_id,
                "capture_reference_id": self.capture_reference_id,
                "capture_reference_matches_execution": (
                    None
                    if self.capture_reference_id is None or execution_id is None
                    else self.capture_reference_id == execution_id
                ),
            },
        )
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
        self._mark_unknown("runtime_shutdown")
        return cleanup_ok

    def reset_for_simulation_stop(self):
        """Drop only Isaac-owned grasp state before the Stage clears material."""
        self._capture_transaction_armed = False
        self._close_command_generation = None
        self._pending_release_command = None
        cleanup_ok = True
        if self.joint_owner.has_owned_joint():
            cleanup_ok = (
                self.joint_owner.owned_joint_valid()
                and self.joint_owner.remove_owned()
                and not self.joint_owner.has_owned_joint()
            )
        if cleanup_ok:
            cleanup_ok = self.reconcile_stale_owned_joint()
        if not cleanup_ok:
            self.ready = False
            self._mark_unknown()
            return False

        clear_reference = getattr(
            self.candidate_source, "clear_expected_tcp_reference", None
        )
        if callable(clear_reference):
            clear_reference()
        self.capture_reference_id = None
        cancel_motion = getattr(self.gripper, "cancel_motion", None)
        if callable(cancel_motion):
            cancel_motion()
        self._mark_unknown()
        return True

    def _mark_unknown(self, reason="state_unavailable"):
        self.state = GraspState.SEEKING
        self.object_detected = False
        self.resistance = False
        return self._record(HoldingState.UNKNOWN, reason)

    def _log_capture_success(self, candidate):
        del candidate
        LOGGER.info("Gripper captured an object and is holding it")

    def _attempt_capture(
        self, actuator_state, runtime_available, candidates, decision=None
    ):
        result = decision or CapturePolicy.evaluate(
            close_active=actuator_state == "holding",
            candidates=candidates,
            config=self.config,
            runtime_available=runtime_available,
            owned_joint_path=getattr(self.joint_owner, "owned_joint_path", None),
        )
        self._record_capture_decision(candidates, result)
        if result.decision is CaptureDecision.CAPTURE_ELIGIBLE:
            self.state = GraspState.ATTACHING
            self.resistance = True
            target_path = result.candidate.prim_path
            physx_before = self._diagnostic_physx_pair(target_path)
            self._diagnostic_append(
                "observation",
                {
                    "phase": "attach_immediately_before_joint_creation",
                    "source": "physx_rigid_body",
                    "pair": physx_before,
                },
            )
            try:
                created = self.joint_owner.create(target_path)
            except Exception as error:
                joint_diagnostics = getattr(
                    self.joint_owner, "last_create_diagnostics", None
                )
                joint_diagnostics = (
                    dict(joint_diagnostics)
                    if isinstance(joint_diagnostics, dict)
                    else {}
                )
                joint_diagnostics.update(
                    ownership_validation="not_performed",
                    call_result="raised",
                    error_type=type(error).__name__,
                    error=str(error),
                )
                if joint_diagnostics.get("creation_result") in (None, "pending"):
                    joint_diagnostics["creation_result"] = "failed"
                joint_diagnostics.setdefault(
                    "joint_path", UsdFixedJointOwner._joint_path(target_path)
                )
                joint_diagnostics.setdefault("body0_path", self.tcp_path)
                joint_diagnostics.setdefault("body1_path", target_path)
                joint_diagnostics.setdefault(
                    "stage_pose_before",
                    {
                        "status": "unavailable",
                        "reason": "joint creation raised before pre-creation Stage observation",
                    },
                )
                for key in ("local_pos0", "local_rot0", "local_pos1", "local_rot1"):
                    joint_diagnostics.setdefault(
                        key,
                        {
                            "status": "unavailable",
                            "reason": "joint creation raised before the authored value could be read",
                        },
                    )
                joint_diagnostics.setdefault(
                    "stage_pose_after",
                    {
                        "status": "unavailable",
                        "reason": "joint creation raised before post-creation Stage observation",
                    },
                )
                self._diagnostic_append(
                    "observation",
                    {
                        "phase": "fixed_joint_creation",
                        "details": joint_diagnostics,
                    },
                )
                LOGGER.warning(
                    "Gripper could not attach the selected object: attachment_create_failed"
                )
                self._capture_transition_failed = True
                self._capture_transaction_armed = False
                transitioned = self._mark_unknown("attachment_create_failed")
                self._diagnostic_outcome(
                    "attachment_create_failed",
                    capture_decision=result.decision,
                    capture_result="failed",
                    transition_reason=(
                        "attachment_create_failed" if transitioned else None
                    ),
                )
                return self.state
            try:
                ownership_valid = bool(created and self.joint_owner.owned_joint_valid())
            except Exception as error:
                ownership_valid = False
                self._diagnostic_append(
                    "observation",
                    {
                        "phase": "fixed_joint_ownership_validation",
                        "result": "failed",
                        "error_type": type(error).__name__,
                        "error": str(error),
                    },
                )
            set_validation = getattr(self.joint_owner, "set_ownership_validation", None)
            if callable(set_validation):
                try:
                    set_validation(ownership_valid)
                except Exception as error:
                    ownership_valid = False
                    self._diagnostic_append(
                        "observation",
                        {
                            "phase": "fixed_joint_ownership_validation",
                            "result": "failed",
                            "error_type": type(error).__name__,
                            "error": str(error),
                        },
                    )
            joint_diagnostics = getattr(
                self.joint_owner, "last_create_diagnostics", None
            )
            if not isinstance(joint_diagnostics, dict):
                joint_diagnostics = {
                    "joint_path": UsdFixedJointOwner._joint_path(target_path),
                    "body0_path": self.tcp_path,
                    "body1_path": target_path,
                    "status": "unavailable",
                    "reason": "joint owner did not expose creation details",
                }
            if not created:
                for key in ("local_pos0", "local_rot0", "local_pos1", "local_rot1"):
                    joint_diagnostics.setdefault(
                        key,
                        {
                            "status": "unavailable",
                            "reason": "joint was not created; no authored USD joint frame exists",
                        },
                    )
                joint_diagnostics.setdefault(
                    "stage_pose_after",
                    {
                        "status": "unavailable",
                        "reason": "joint was not created",
                    },
                )
            self._diagnostic_append(
                "observation",
                {
                    "phase": "fixed_joint_creation",
                    "details": joint_diagnostics,
                },
            )
            if created:
                self._diagnostic_append(
                    "observation",
                    {
                        "phase": "attach_immediately_after_joint_creation",
                        "source": "physx_rigid_body",
                        "pair": self._diagnostic_physx_pair(target_path),
                    },
                )
            if not created or not ownership_valid:
                failure_reason = (
                    "attachment_create_failed"
                    if not created
                    else "attachment_ownership_validation_failed"
                )
                LOGGER.warning(
                    "Gripper could not attach the selected object: %s",
                    failure_reason,
                )
                self._capture_transition_failed = True
                self._capture_transaction_armed = False
                transitioned = self._mark_unknown(failure_reason)
                self._diagnostic_outcome(
                    failure_reason,
                    capture_decision=result.decision,
                    capture_result="failed",
                    transition_reason=failure_reason if transitioned else None,
                )
                return self.state
            if not self._release_for_capture(result.candidate.prim_path):
                LOGGER.warning("Gripper could not complete the capture transition")
                cleanup_ok = self._detach_owned_joint()
                if not cleanup_ok:
                    self.ready = False
                self._capture_transition_failed = True
                self._capture_transaction_armed = False
                transitioned = self._mark_unknown("pre_capture_transition_failed")
                self._diagnostic_outcome(
                    "pre_capture_transition_failed",
                    capture_decision=result.decision,
                    capture_result="failed",
                    transition_reason=(
                        "pre_capture_transition_failed" if transitioned else None
                    ),
                )
                return self.state
            self._capture_transaction_armed = False
            self.state = GraspState.GRASPED
            self.object_detected = True
            transitioned = self._record(HoldingState.HELD, "attachment_created")
            self._log_capture_success(result.candidate)
            clear_reference = getattr(
                self.candidate_source, "clear_expected_tcp_reference", None
            )
            if callable(clear_reference):
                clear_reference()
            self.capture_reference_id = None
            self.capture_diagnostic_target_path = target_path
            self._diagnostic_outcome(
                "capture_held",
                finalize=False,
                capture_decision=result.decision,
                capture_result="captured",
                transition_reason=("attachment_created" if transitioned else None),
            )
            return self.state

        self.state = GraspState.SEEKING
        self.object_detected = False
        self.resistance = False
        if actuator_state == "closing":
            # Closing is an active search interval, not a terminal rejection.
            return self.state
        reason = self._capture_rejection_reason(result)
        LOGGER.warning(
            "Gripper could not capture an object: %s",
            reason,
        )
        self._capture_transition_failed = True
        self._capture_transaction_armed = False
        transitioned = self._mark_unknown(reason)
        self._diagnostic_outcome(
            reason,
            capture_decision=result.decision,
            capture_result="rejected",
            transition_reason=reason if transitioned else None,
        )
        return self.state

    def _evaluate_capture_transaction(self, actuator_state):
        """Search during CLOSE and make one final Capture assessment at HOLDING."""
        if (
            not self.ready
            or not self._capture_transaction_armed
            or self._pending_release_command is not None
            or self._capture_transition_failed
            or actuator_state not in ("closing", "holding")
        ):
            return False

        try:
            runtime_available, candidates = self.candidate_source.query()
        except Exception as error:
            runtime_available, candidates = False, []
            self._diagnostic_append(
                "observation",
                {
                    "phase": "capture_candidate_query",
                    "result": "failed",
                    "error_type": type(error).__name__,
                    "error": str(error),
                },
            )
        try:
            selection = CapturePolicy.evaluate(
                close_active=True,
                candidates=candidates,
                config=self.config,
                runtime_available=runtime_available,
                owned_joint_path=getattr(self.joint_owner, "owned_joint_path", None),
            )
        except Exception as error:
            selection = CaptureResult(
                CaptureDecision.UNKNOWN_RUNTIME,
                assessments=(CaptureAssessment("", False, "unknown_runtime"),),
            )
            self._diagnostic_append(
                "observation",
                {
                    "phase": "capture_policy_evaluation",
                    "result": "failed",
                    "error_type": type(error).__name__,
                    "error": str(error),
                },
            )
        try:
            qualification = (
                GraspQualification.evaluate(
                    material_shape=selection.candidate.shape,
                    gripper_characteristics=GripperCharacteristics(
                        getattr(self.gripper, "MODEL_ID", None)
                    ),
                )
                if selection.decision is CaptureDecision.CAPTURE_ELIGIBLE
                else None
            )
        except Exception as error:
            qualification = None
            self._diagnostic_append(
                "observation",
                {
                    "phase": "grasp_qualification",
                    "result": "failed",
                    "error_type": type(error).__name__,
                    "error": str(error),
                },
            )
        self._diagnostic_append(
            "decision",
            {
                "phase": "grasp_qualification",
                "result": (
                    "accepted"
                    if qualification is not None and qualification.eligible
                    else "rejected" if qualification is not None else "not_evaluated"
                ),
                "reason": None if qualification is None else qualification.reason,
                "material_shape": (
                    selection.candidate.shape
                    if selection.decision is CaptureDecision.CAPTURE_ELIGIBLE
                    else None
                ),
                "gripper_model_id": getattr(self.gripper, "MODEL_ID", None),
            },
        )
        if selection.decision is not CaptureDecision.CAPTURE_ELIGIBLE:
            reason = self._capture_rejection_reason(selection)
        elif qualification is None or not qualification.eligible:
            reason = (
                qualification.reason
                if qualification is not None
                else "qualification_unavailable"
            )
        else:
            if actuator_state == "closing":
                try:
                    self.gripper.stop()
                except Exception as error:
                    reason = "gripper_stop_failed"
                    self._diagnostic_append(
                        "observation",
                        {
                            "phase": "close_interruption",
                            "result": "failed",
                            "actuator_state": actuator_state,
                            "error_type": type(error).__name__,
                            "error": str(error),
                        },
                    )
                    self._record_capture_decision(candidates, selection)
                    LOGGER.warning("Gripper could not capture an object: %s", reason)
                    self._capture_transaction_armed = False
                    self._capture_transition_failed = True
                    transitioned = self._mark_unknown(reason)
                    self._diagnostic_outcome(
                        reason,
                        capture_decision=selection.decision,
                        capture_result="failed",
                        transition_reason=reason if transitioned else None,
                    )
                    return True
                self._diagnostic_append(
                    "observation",
                    {
                        "phase": "close_interruption",
                        "result": "stopped_for_capture",
                        "actuator_state": "closing",
                        "rawpart_path": selection.candidate.prim_path,
                    },
                )
            self._attempt_capture(
                actuator_state, runtime_available, candidates, decision=selection
            )
            return True

        self._record_capture_decision(candidates, selection)
        if (
            selection.decision is CaptureDecision.NO_CAPTURE
            and actuator_state == "closing"
        ):
            # The object may enter the Capture Volume on a later physics tick.
            # Keep the transaction armed and do not publish a terminal result.
            self.state = GraspState.SEEKING
            self.object_detected = False
            self.resistance = False
            self._diagnostic_append(
                "observation",
                {
                    "phase": "capture_search",
                    "result": "no_candidate_yet",
                    "actuator_state": actuator_state,
                    "close_completed": False,
                },
            )
            return False

        LOGGER.warning("Gripper could not capture an object: %s", reason)
        self._capture_transaction_armed = False
        self._capture_transition_failed = True
        transitioned = self._mark_unknown(reason)
        self._diagnostic_outcome(
            reason,
            capture_decision=selection.decision,
            capture_result="rejected",
            transition_reason=reason if transitioned else None,
        )
        return True

    @staticmethod
    def _capture_rejection_reason(result):
        if result.decision is CaptureDecision.AMBIGUOUS:
            return "ambiguous"
        if result.decision is CaptureDecision.INVALID_CONFIG:
            return "invalid_config"
        if result.decision is CaptureDecision.UNKNOWN_RUNTIME:
            return "unknown_runtime"
        for assessment in result.assessments:
            if assessment.rejection_reason:
                return assessment.rejection_reason
        return "no_capture"

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

    def _record_released_if_open(self):
        """Confirm physical OPEN and joint absence without a Capture query."""
        pending = self._pending_release_command
        target_width_mm = pending[1] if pending is not None else 85.0
        measured_width_mm = None
        result = "runtime_error"
        attempted = False
        gripper_state = None
        error_type = None
        try:
            gripper_state = self.gripper.state.value
            result = "open_incomplete"
            if gripper_state != "holding":
                return False
            result = "actuator_width_unavailable"
            measured_width_mm = float(self.gripper.get_width())
            if not math.isfinite(measured_width_mm):
                return False
            result = "open_target_not_reached"
            if measured_width_mm < target_width_mm - 0.1:
                return False
            result = "joint_state_unavailable"
            owned = self.joint_owner.has_owned_joint()
            if owned is not True and owned is not False:
                return False
            if owned:
                if pending is None or not self.joint_owner.owned_joint_valid():
                    return False
                attempted = True
                result = "joint_remove_failed"
                if not self.joint_owner.remove_owned():
                    return False
                result = "joint_state_unavailable"
                remaining = self.joint_owner.has_owned_joint()
                if remaining is not False:
                    if remaining is True:
                        result = "joint_remove_failed"
                    return False
            result = "released"
            return True
        except Exception as error:
            error_type = type(error).__name__
            return False
        finally:
            # A pending OPEN is not a terminal UNKNOWN observation. Motion waits
            # for a new sequence and treats a new UNKNOWN as a terminal failure.
            pending = self._pending_release_command
            if pending is None or gripper_state != "opening":
                if pending is not None:
                    generation, _ = pending
                    self.note_gripper_command_applied()
                    self._release_detach_event = {
                        "generation": generation,
                        "attempted": attempted,
                        "actuator_state": gripper_state,
                        "target_width_mm": target_width_mm,
                        "measured_width_mm": measured_width_mm,
                        "result": (
                            ("detached" if attempted else "already_absent")
                            if result == "released"
                            else result
                        ),
                        **({"error_type": error_type} if error_type else {}),
                    }
                    self._capture_transition_failed = result != "released"
                if result == "released":
                    self.state = GraspState.SEEKING
                    self.resistance = False
                    self.object_detected = False
                    self._record(HoldingState.RELEASED, "release_confirmed")
                else:
                    self._mark_unknown("release_not_confirmed_" + result)

    def update(self, dt):
        del dt
        if not self.ready:
            self._capture_transaction_armed = False
            self._mark_unknown("runtime_unavailable")
            return self.state

        gripper_state = getattr(self.gripper.state, "value", "")
        if self._pending_release_command is not None:
            self._capture_transaction_armed = False
            self._record_released_if_open()
            return self.state

        if self._capture_transition_failed:
            self._capture_transaction_armed = False
            return self.state

        try:
            if self.joint_owner.has_owned_joint():
                self._capture_transaction_armed = False
                if not self.joint_owner.owned_joint_valid():
                    self._mark_unknown("attachment_ownership_invalid")
                else:
                    self.state = GraspState.GRASPED
                    self.object_detected = True
                    self.resistance = True
                    self._record(HoldingState.HELD, "attachment_owned")
                return self.state
        except Exception:
            self._mark_unknown("attachment_state_unavailable")
            return self.state

        if (
            self._holding.state is HoldingState.RELEASED
            and not self._capture_transaction_armed
        ):
            # RELEASED is a completed observation. Rechecking an idle gripper
            # against the open target on every physics tick can downgrade it
            # when measured width jitters just below the tolerance. First
            # verify that no owned attachment remains or has become invalid.
            return self.state

        if self._capture_transaction_armed and gripper_state in ("closing", "holding"):
            try:
                measured_width = self.gripper.get_width()
            except Exception:
                measured_width = None
            self._diagnostic_append(
                "observation",
                {
                    "phase": (
                        "close_completion"
                        if gripper_state == "holding"
                        else "close_progress"
                    ),
                    "close_requested": True,
                    "close_completed": gripper_state == "holding",
                    "command_generation": self._close_command_generation,
                    "actuator_state": gripper_state,
                    "measured_width_mm": measured_width,
                },
            )
            self._evaluate_capture_transaction(gripper_state)
            return self.state

        if self._capture_transaction_armed:
            self._capture_transaction_armed = False
            self._capture_transition_failed = True
            reason = "close_failed"
            self._diagnostic_append(
                "decision",
                {
                    "phase": "close_completion",
                    "close_requested": True,
                    "close_completed": False,
                    "actuator_state": gripper_state or "unknown",
                    "reason": reason,
                },
            )
            transitioned = self._mark_unknown(reason)
            self._diagnostic_outcome(
                reason,
                capture_result="not_attempted",
                transition_reason=reason if transitioned else None,
            )
            return self.state

        if gripper_state != "closing":
            self._record_released_if_open()
            return self.state

        return self.state

    def release(self, open_gripper=True, duration=None):
        self._capture_transaction_armed = False
        if not self.ready:
            self._mark_unknown()
            return
        self._pending_release_command = (None, 85.0)
        if open_gripper:
            try:
                self.gripper.open(duration=duration)
            except Exception:
                self._pending_release_command = None
                self._release_detach_event = {
                    "generation": None,
                    "attempted": False,
                    "result": "runtime_error",
                }
                self._capture_transition_failed = True
                self._mark_unknown()
                return
        self.update(0.0)

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
