"""Tool-owned actuation and capture predicates; attachment remains in GraspManager."""
from dataclasses import dataclass
from enum import Enum
import math

from gripper_runtime.grasp_policy import CapturePolicy, GraspConfig
from gripper_runtime.grasp_qualification import GraspQualification, GripperCharacteristics


class ToolIntent(Enum):
    GRASP = 'GRASP'
    RELEASE = 'RELEASE'
    STOP = 'STOP'


@dataclass
class ToolRuntime:
    """Injected Tool implementation.

    The actuator reports logical opening/closing/holding phases and advances in
    update(dt); these describe actuation, never material ownership. Capture owns
    candidate observation/selection, qualification, interruption before attach,
    and release completion. Width is optional telemetry guarded by capability.
    STOP cancels actuation without requesting material release.
    """
    actuator: object
    capabilities: frozenset
    execute: object
    capture: object
    max_width_mm: float | None = None
    move_duration: float = 0.40
    grasp_factory: object = None
    control: object = None

    def command(self, intent, duration=None, parameters=None):
        intent = ToolIntent(intent)
        duration = self.move_duration if duration is None else duration
        if parameters:
            if self.control is None:
                raise ValueError('Selected Tool has no parameter control capability')
            self.control(intent, parameters)
        else:
            self.execute(intent, duration)

    def observation(self):
        observe = getattr(self.actuator, 'observation', None)
        return observe() if callable(observe) else None

    def cancel_motion(self):
        cancel = getattr(self.actuator, 'cancel_motion', None)
        if callable(cancel):
            cancel()
        else:
            self.command(ToolIntent.STOP)

    def width_mm(self):
        if 'width' not in self.capabilities:
            return None
        return self.actuator.get_width()


class ParallelGripperCapture:
    """Existing parallel-gripper capture and release predicates, unchanged."""
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


def _parallel_command(actuator, intent, duration):
    if intent is ToolIntent.GRASP:
        actuator.close(duration=duration)
    elif intent is ToolIntent.RELEASE:
        actuator.open(duration=duration)
    else:
        actuator.stop()


def parallel_tool_runtime(actuator, profile):
    settings = profile['runtime']
    maximum = float(settings['max_width_mm'])
    return ToolRuntime(
        actuator, frozenset(settings['capabilities']),
        lambda intent, duration: _parallel_command(actuator, intent, duration),
        ParallelGripperCapture(settings['supported_shapes'], maximum),
        maximum, float(settings['move_duration_s']),
    )


def _robotiq_runtime(profile, binding):
    from gripper_runtime.robotiq_actuator import Robotiq2F85Actuator
    settings = profile['runtime']
    combination = (settings.get('actuator', 'logical'), settings.get('grasp', 'fixed_attach'))
    if combination not in {('logical', 'fixed_attach'), ('physics', 'fixed_attach'), ('physics', 'physical')}:
        raise ValueError(f'Unsupported Robotiq actuator/grasp combination: {combination}')
    if combination[0] == 'physics':
        from gripper_runtime.physics_actuator import Robotiq2F85PhysicsActuator
        actuator = Robotiq2F85PhysicsActuator(settings=settings['physics'],
                    joint_model=settings['joint_model'], binding=binding)
        tool = parallel_tool_runtime(actuator, profile)
        tool.capabilities = tool.capabilities | frozenset({'position', 'speed_setting', 'force_setting', 'actuator_observation'})
        def control(intent, parameters):
            if intent is ToolIntent.STOP or set(parameters) - {'target_position', 'speed_fraction', 'force_setting'}:
                raise ValueError('Unsupported Tool command parameters')
            values = dict(parameters)
            values.setdefault('target_position', 0.0 if intent is ToolIntent.RELEASE else 1.0)
            actuator.configure_command(**values)
        tool.control = control
        if combination[1] == 'physical':
            from gripper_runtime.physical_grasp import PhysicalGraspManager
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            paths = [str(prim.GetPath()) for prim in stage.Traverse()
                     if prim.GetName() == 'sf_grasp_tcp' and str(prim.GetPath()).startswith(binding.robot_path + '/')]
            if len(paths) != 1:
                raise ValueError('Physical Grasp requires one Tool TCP in selected Assembly')
            tool.grasp_factory = lambda **services: PhysicalGraspManager(
                stage=stage, tcp_path=paths[0],
                holding_settings=settings['physics']['holding'], **services)
        return tool
    actuator = Robotiq2F85Actuator(
        robot_root_path=binding.robot_path,
        articulation_root_path=binding.articulation_root_path,
        articulation_base_path=binding.base_link_path,
        verbose=True,
    )
    return parallel_tool_runtime(actuator, profile)


def select_tool_runtime(profile, binding, *, implementations=None):
    """One explicit assembly boundary; test implementations never enter production."""
    implementations = {'robotiq_2f85': _robotiq_runtime} if implementations is None else implementations
    name = profile['runtime']['implementation']
    try:
        build = implementations[name]
    except KeyError:
        raise ValueError(f'Unsupported Tool runtime implementation: {name}') from None
    return build(profile, binding)
