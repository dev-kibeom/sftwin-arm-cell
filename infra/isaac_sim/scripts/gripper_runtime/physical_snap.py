"""Contact qualification strategy; GraspManager retains all attachment ownership."""
import math

from gripper_runtime.contact_reaction import finger_reactions
from gripper_runtime.position_stability import PositionStability
from gripper_runtime.tool_plugin import ParallelGripperCapture
from gripper_runtime.grasp_policy import CaptureResult, CaptureDecision


class PhysicalSnapCapture(ParallelGripperCapture):
    requires_reference = True

    def __init__(self, *, settings, observer, fingertip_paths):
        super().__init__(max_width_mm=85.)
        self.settings = settings
        self.observer = observer
        self.fingertip_paths = fingertip_paths
        self.stability = PositionStability()
        self.dt = 0.
        self.target = None

    def reset(self):
        self.stability.reset()
        self.target = None
        self.dt = 0.
        self.observer.clear()

    def shutdown(self):
        self.reset()
        self.observer.close()

    def advance(self, dt):
        self.dt = dt
        try:
            self.observer.advance()
        except Exception:
            self.observer.snapshot = None
            self.stability.reset()

    def evaluate_transaction(self, manager, actuator_state):
        candidate = None
        reactions = None
        try:
            available, candidates = manager.candidate_source.query()
            selected = manager.candidate_source.selected_object_path
            if (not available or not selected or len(candidates) != 1
                    or not manager.capture_reference_id
                    or manager.capture_reference_id != manager._capture_execution_id):
                raise ValueError('capture_reference_or_association_unavailable')
            candidate = candidates[0]
            if not candidate.eligible or candidate.prim_path != selected:
                raise ValueError('capture_target_mismatch')
            expected = candidate.capture_reference['expected_contact_width_mm']
            observation = manager.gripper.observation()
            position = observation['actual_position_rad']
            target = observation['target_position_rad']
            epsilon = observation['position_tolerance_rad']
            reactions = finger_reactions(self.observer.snapshot, self.dt,
                target_path=selected, fingertip_paths=self.fingertip_paths)
            valid = (observation['command_direction'] == 'closing'
                and all(math.isfinite(v) for v in (position, target, expected))
                and target - position > epsilon
                and observation['lower_limit_rad'] + epsilon < position < observation['upper_limit_rad'] - epsilon
                and self.width_matches_reference(observation['actual_width_mm'], expected,
                                                 self.settings['width_tolerance_mm'])
                and reactions is not None
                and min(reactions) >= self.settings['min_contact_force_per_finger_n'])
            if not valid:
                raise ValueError('physical_contact_not_qualified')
            identity = (manager.capture_reference_id, selected)
            if self.target != identity:
                self.stability.reset()
                self.target = identity
            duration = self.stability.update(position, self.dt,
                                             self.settings['held_position_tolerance_rad'])
            qualified = duration >= self.settings['stopped_dwell_s']
            reason = 'qualified' if qualified else 'stability_pending'
        except (AttributeError, KeyError, TypeError, ValueError, RuntimeError, ArithmeticError) as error:
            self.stability.reset()
            self.target = None
            qualified = False
            reason = str(error)
        manager._diagnostic_append('decision', dict(phase='physical_snap_qualification',
            qualified=qualified, reason=reason, physics_dt_s=self.dt, finger_reactions_n=reactions,
            stationary_duration_s=self.stability.duration,
            target_path=candidate.prim_path if candidate else None))
        if not qualified:
            manager._mark_unknown('physical_snap_qualification_pending')
            return False
        # Qualification is not HELD. Only the existing owner create/validate path
        # may publish HELD; no logical capture/pose qualification is repeated.
        manager._attempt_capture(actuator_state, True, [candidate],
            decision=CaptureResult(CaptureDecision.CAPTURE_ELIGIBLE, candidate))
        return True
