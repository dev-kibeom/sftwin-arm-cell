"""Tool-owned actuation and capture predicates; attachment remains in GraspManager."""
from dataclasses import dataclass
from enum import Enum

from gripper_runtime.grasp_strategy import BoxExteriorGraspStrategy, select_grasp_strategy


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


# Compatibility name for existing Tool clients.
ParallelGripperCapture = BoxExteriorGraspStrategy


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
        select_grasp_strategy({**settings, 'grasp_strategies': settings.get('grasp_strategies', ['box_exterior'])},
                              {'strategy': 'box_exterior'}),
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
                    joint_model=settings['joint_model'], force_transmission=settings['force_transmission'], binding=binding)
        tool = parallel_tool_runtime(actuator, profile)
        tool.capabilities = tool.capabilities | frozenset({'position', 'speed_setting', 'grip_force_n', 'actuator_observation'})
        def control(intent, parameters):
            if intent is ToolIntent.STOP or set(parameters) - {'target_position', 'speed_fraction', 'grip_force_n'}:
                raise ValueError('Unsupported Tool command parameters')
            values = dict(parameters)
            values.setdefault('target_position', 0.0 if intent is ToolIntent.RELEASE else 1.0)
            actuator.configure_command(**values)
        tool.control = control
        if combination[1] == 'fixed_attach':
            from gripper_runtime.physical_snap import PhysicalSnapCapture
            from gripper_runtime.physical_contacts import PhysicalContactObserver
            tool.capture = PhysicalSnapCapture(settings=settings['physics']['holding'],
                observer=PhysicalContactObserver(), fingertip_paths=tuple(
                    binding.robot_path.rstrip('/') + '/gripper_robotiq_85_' + side + '_finger_tip_link'
                    for side in ('left', 'right')))
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
    tool = parallel_tool_runtime(actuator, profile)
    # Logical Snap retains the common request without claiming motor/contact
    # force enforcement. Force remains policy input even when motion is logical.
    tool.capabilities = tool.capabilities | frozenset({'grip_force_n'})
    def control(intent, parameters):
        from gripper_runtime.force_transmission import validate_grip_force_n
        if intent is ToolIntent.STOP or set(parameters) != {'grip_force_n'}:
            raise ValueError('Unsupported logical Tool command parameters')
        force = validate_grip_force_n(parameters['grip_force_n'])
        actuator.requested_grip_force_n = force
        _parallel_command(actuator, intent, tool.move_duration)
    tool.control = control
    return tool


def select_tool_runtime(profile, binding, *, implementations=None, configuration=None):
    """One explicit assembly boundary; test implementations never enter production."""
    if implementations is None:
        from gripper_runtime.radial_physical import radial_physical_runtime
        implementations = {'robotiq_2f85': _robotiq_runtime, 'hand_e_c10_physical': radial_physical_runtime}
    name = profile['runtime']['implementation']
    try:
        build = implementations[name]
    except KeyError:
        raise ValueError(f'Unsupported Tool runtime implementation: {name}') from None
    if name == 'hand_e_c10_physical' and configuration is not None:
        return build(profile, binding, configuration=configuration)
    return build(profile, binding)
