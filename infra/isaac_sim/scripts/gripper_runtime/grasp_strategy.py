"""Tool capability plus task requirement selects grasp semantics, independently of Robot."""
import math

from gripper_runtime.grasp_policy import CapturePolicy, GraspConfig
from gripper_runtime.grasp_qualification import GraspQualification, GripperCharacteristics


class BoxExteriorGraspStrategy:
    """Existing parallel-gripper capture and release predicates, unchanged."""
    name = 'box_exterior'
    select = staticmethod(CapturePolicy.evaluate)

    def __init__(self, supported_shapes=('box',), max_width_mm=None):
        self.supported_shapes = tuple(supported_shapes)
        self.max_width_mm = max_width_mm

    @staticmethod
    def candidate_source(stage, tcp_path, registry, actuator):
        from gripper_runtime.grasp_stage import StageCaptureQuery
        return StageCaptureQuery(stage, tcp_path, registry, actuator, reference_config=None)

    def qualify(self, candidate):
        return GraspQualification.evaluate(
            material_shape=candidate.shape,
            gripper_characteristics=GripperCharacteristics('parallel_gripper', self.supported_shapes),
        )

    def ready_during_grasp(self, candidate, source):
        try:
            measured = float(candidate.measured_opening_mm)
            expected = float(candidate.capture_reference['expected_contact_width_mm'])
            tolerance = float(getattr(
                source, 'actual_contact_width_tolerance_mm',
                GraspConfig().contact_width_tolerance_mm,
            ))
            return (math.isfinite(measured) and measured >= 0.0
                    and math.isfinite(expected) and expected > 0.0
                    and math.isfinite(tolerance) and tolerance >= 0.0
                    and measured <= expected + tolerance)
        except (TypeError, ValueError, KeyError, OverflowError):
            return False

    @staticmethod
    def physical_holding_detected(observation, settings):
        """Parallel Tool obstruction inference, without uncalibrated aperture gating.

        This is device-level possession evidence, not target identification.
        The physical actuator's width is compatibility telemetry; Snap's
        geometric qualification retains its separate width requirement.
        """
        pos, target = observation['actual_position_rad'], observation['target_position_rad']
        epsilon = observation['position_tolerance_rad']
        return (observation['command_direction'] == 'closing'
                and observation['object_detection'] == 'stopped_closing'
                and observation['stopped_duration_s'] >= settings['stopped_dwell_s']
                and all(math.isfinite(v) for v in (pos, target, epsilon))
                and target - pos > epsilon
                and observation['lower_limit_rad'] + epsilon < pos
                < observation['upper_limit_rad'] - epsilon)

    @staticmethod
    def width_matches_reference(measured, expected, tolerance):
        return (all(math.isfinite(v) for v in (measured, expected, tolerance))
                and measured >= 0.0 and expected > 0.0 and tolerance >= 0.0
                and abs(measured - expected) <= tolerance)

    @staticmethod
    def on_capture_ready(actuator):
        actuator.stop()

    def release_complete(self, actuator, target):
        width = float(actuator.get_width())
        target = self.max_width_mm if target is None else target
        return target is not None and math.isfinite(width) and width >= target - 0.1



class InternalRadialGraspStrategy:
    """Expansion captures an explicitly specified bore; contraction releases it."""
    name = 'internal_radial'

    def __init__(self, geometry, requirement, tolerance):
        from internal_grasp_geometry import qualify_capture
        self._qualify = qualify_capture
        self.tolerance = float(tolerance)
        if not math.isfinite(self.tolerance) or self.tolerance < 0:
            raise ValueError('Invalid radial position tolerance')
        try:
            self.solution = geometry.solve_bore(*(float(requirement[key]) for key in
                ('insertion_depth_m', 'thickness_m', 'inner_radius_m')))
        except (KeyError, TypeError, AttributeError) as exc:
            raise ValueError('Internal radial requires explicit bore geometry and selected Tool geometry') from exc

    def primary_position(self, intent):
        intent = getattr(intent, 'value', intent)
        if intent == 'GRASP':
            return self.solution['grasp_position_m']
        if intent == 'RELEASE':
            return self.solution['release_position_m']
        raise ValueError('No position target for Tool intent')

    def qualify_capture(self, relative_position, relative_rotation, measured_primary):
        self._qualify(relative_position, relative_rotation, measured_primary,
                      self.solution, self.tolerance)


def select_grasp_strategy(settings, requirement, *, geometry=None):
    """Fail closed unless both the task and Tool declare the same strategy."""
    name = requirement.get('strategy')
    if name not in settings.get('grasp_strategies', ()):
        raise ValueError(f'Tool does not support required grasp strategy: {name}')
    if name == 'box_exterior':
        return BoxExteriorGraspStrategy(settings.get('supported_shapes', ()), settings.get('max_width_mm'))
    if name == 'internal_radial':
        return InternalRadialGraspStrategy(geometry, requirement, settings['position_tolerance_m'])
    raise ValueError(f'Unsupported grasp strategy: {name}')
