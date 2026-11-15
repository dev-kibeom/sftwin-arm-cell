"""Drive commands are not state writes or proof of material ownership."""
from types import SimpleNamespace
import math

import pytest


def settings():
    return dict(primary_joint='gripper_robotiq_85_left_knuckle_joint',
                speed_fraction=0.5, grip_force_n=40.,
                drive_stiffness=20., drive_damping=2.,
                position_tolerance_rad=0.01, stopped_velocity_rad_s=0.01,
                blocked_dwell_s=0.15)


class Drive:
    """Only supported DC target/property/read methods; no state-write methods."""
    def __init__(self):
        self.pos = 0.
        self.vel = 0.
        self.target = None
        self.props = SimpleNamespace(max_velocity=0.5, max_effort=50., stiffness=0., damping=0.)

    def get_dof_state(self, handle, flags):
        return SimpleNamespace(pos=self.pos, vel=self.vel)

    def get_dof_properties(self, handle):
        return self.props

    def set_dof_properties(self, handle, props):
        self.props = props
        return True

    def set_dof_position_target(self, handle, target):
        self.target = target
        return True

    def set_dof_velocity_target(self, handle, target):
        return True

    def wake_up_articulation(self, articulation):
        return True


def actuator():
    from gripper_runtime.physics_actuator import Robotiq2F85PhysicsActuator
    drive = Drive()
    result = Robotiq2F85PhysicsActuator(
        settings=settings(), force_transmission=dict(finger_lever_x_m=.0371575, finger_lever_z_m=.04342168), joint_model={
            'gripper_robotiq_85_left_knuckle_joint': dict(lower=0., upper=0.8, velocity=0.5, effort=50.)},
        runtime=(drive, SimpleNamespace(STATE_ALL=7, DRIVE_FORCE=0), 1, 2))
    return result, drive


def test_physics_commands_targets_and_reads_actual_state():
    gripper, drive = actuator()
    gripper.close()
    assert drive.target == pytest.approx(0.8)
    assert drive.props.max_effort == pytest.approx(40. * .04342168)
    assert 0. < drive.props.max_velocity <= 0.5
    for _ in range(20):
        gripper.update(0.01)
    assert gripper.get_position() == 0.  # No fake motion from elapsed time.
    assert gripper.object_detection == 'stopped_closing'
    drive.pos = 0.4
    drive.vel = 0.12
    assert gripper.get_position() == pytest.approx(0.5)
    assert gripper.observation()['actual_velocity_rad_s'] == pytest.approx(0.12)
    gripper.stop()
    assert drive.target == pytest.approx(0.4)
    assert not gripper.is_moving


def test_physics_target_completion_and_invalid_parameters():
    gripper, drive = actuator()
    gripper.close()
    drive.pos = 0.8
    gripper.update(0.01)
    assert gripper.object_detection == 'at_requested_position'
    assert not gripper.is_moving
    for parameter in ({'speed_fraction': math.nan}, {'grip_force_n': 471.}, {'target_position': -1.}):
        with pytest.raises(ValueError):
            gripper.configure_command(**parameter)


def test_physical_grasp_never_attaches_or_confirms_held_from_stall():
    from gripper_runtime.physical_grasp import PhysicalGraspManager
    from gripper_runtime.grasp_attachment import HoldingState
    gripper, drive = actuator()
    from gripper_runtime.tool_plugin import ParallelGripperCapture
    tool = SimpleNamespace(actuator=gripper, cancel_motion=gripper.cancel_motion,
                           capture=ParallelGripperCapture(max_width_mm=85.))
    manager = PhysicalGraspManager(gripper, tool=tool)
    manager.arm_close_transaction('goal', 1)
    gripper.close()
    for _ in range(30):
        gripper.update(0.01)
        manager.update(0.01)
    assert manager.holding_observation().state is HoldingState.UNKNOWN
    assert manager.attach_joint_path is None
    manager.note_open_release_command_applied(2, 85.)
    gripper.open()
    drive.pos = 0.
    manager.update(0.01)
    assert manager.holding_observation().state is HoldingState.RELEASED
    manager.reset_for_simulation_stop()
    assert manager.holding_observation().state is HoldingState.UNKNOWN


def test_unsupported_actuator_grasp_combination_is_configuration_error():
    from gripper_runtime.tool_plugin import select_tool_runtime
    profile = {'runtime': {'implementation': 'robotiq_2f85', 'actuator': 'logical', 'grasp': 'physical'}}
    with pytest.raises(ValueError, match='combination'):
        select_tool_runtime(profile, SimpleNamespace())


def test_physics_stop_holds_actual_position_without_changing_observation():
    gripper, drive = actuator()
    drive.pos, drive.vel = 0.3, 0.02
    gripper.close()
    gripper.stop()
    assert drive.target == pytest.approx(0.3)
    assert drive.pos == pytest.approx(0.3)
    assert drive.vel == pytest.approx(0.02)
    gripper.cancel_motion()
    assert gripper.target_position is None
    assert gripper.object_detection == 'unknown'


def test_optional_tool_parameters_do_not_become_required_for_logical_tools():
    from gripper_runtime.tool_plugin import ToolRuntime, ToolIntent
    applied = []
    tool = ToolRuntime(None, frozenset(), lambda intent, duration: applied.append(intent), None)
    with pytest.raises(ValueError, match='capability'):
        tool.command(ToolIntent.GRASP, parameters={'grip_force_n': 100.})
    assert not applied
    tool.command(ToolIntent.GRASP)
    assert applied == [ToolIntent.GRASP]


def test_obstruction_status_is_reobserved_when_physical_object_moves_away():
    gripper, drive = actuator()
    gripper.close()
    for _ in range(5):
        gripper.update(0.05)
    assert gripper.object_detection == 'stopped_closing'
    drive.pos, drive.vel = 0.6, 0.1
    gripper.update(0.01)
    assert gripper.object_detection == 'in_motion'
    drive.pos, drive.vel = 0.8, 0.
    gripper.update(0.01)
    assert gripper.object_detection == 'at_requested_position'

    drive.pos = 0.6
    gripper.update(0.01)
    assert gripper.object_detection == 'in_motion'  # New obstruction needs a new dwell.


def test_optional_physical_diagnostic_failure_does_not_change_grasp_state():
    from gripper_runtime.physical_grasp import PhysicalGraspManager
    from gripper_runtime.grasp_attachment import HoldingState
    gripper, _ = actuator()
    tool = SimpleNamespace(cancel_motion=gripper.cancel_motion)
    manager = PhysicalGraspManager(gripper, tool=tool)
    def unavailable(record):
        raise OSError('diagnostics unavailable')
    manager.set_diagnostic_observers(SimpleNamespace(submit=unavailable))
    manager.update(0.1)
    assert manager.ready
    assert manager.holding_observation().state is HoldingState.UNKNOWN


def test_total_force_virtual_work_and_live_linkage_position():
    from gripper_runtime.force_transmission import RobotiqForceTransmission
    model = RobotiqForceTransmission(dict(finger_lever_x_m=0.03, finger_lever_z_m=0.04))
    # At q=0 each contact moves 0.04 m/rad; 100 N total gives 50 N per finger.
    assert model.drive_torque_nm(100., 0.) == pytest.approx(4.)
    assert model.drive_torque_nm(100., math.pi / 2) == pytest.approx(3.)
    gripper, drive = actuator()
    gripper.configure_command(grip_force_n=100., target_position=1.)
    first = drive.props.max_effort
    drive.pos = 0.4
    gripper.update(.01)
    assert drive.props.max_effort != pytest.approx(first)
    assert gripper.observation()['grip_force_n'] == 100.


@pytest.mark.parametrize('force', [0., 39., 471., math.inf, math.nan])
def test_invalid_force_rejects_before_changing_target(force):
    gripper, drive = actuator()
    with pytest.raises(ValueError):
        gripper.configure_command(grip_force_n=force, target_position=1.)
    assert drive.target is None


def test_force_beyond_urdf_motor_effort_rejects_without_clamping():
    gripper, drive = actuator()
    gripper.model['effort'] = 1.
    with pytest.raises(ValueError, match='primary joint effort'):
        gripper.configure_command(grip_force_n=100., target_position=1.)
    assert drive.target is None
    assert gripper.grip_force_n == 40.


def test_logical_tool_retains_force_request_without_physics_claim(monkeypatch):
    from gripper_runtime import robotiq_actuator
    from gripper_runtime.tool_plugin import select_tool_runtime, ToolIntent
    commands = []
    logical = SimpleNamespace(close=lambda duration: commands.append('close'))
    monkeypatch.setattr(robotiq_actuator, 'Robotiq2F85Actuator', lambda **kwargs: logical)
    profile = {'runtime': {'implementation': 'robotiq_2f85', 'max_width_mm': 85.,
                          'capabilities': ['width'], 'supported_shapes': ['box'], 'move_duration_s': .4}}
    binding = SimpleNamespace(robot_path='/robot', articulation_root_path='/robot', base_link_path='/robot/base')
    tool = select_tool_runtime(profile, binding)
    with pytest.raises(ValueError):
        tool.command(ToolIntent.GRASP, parameters={'grip_force_n': 471.})
    assert commands == []
    tool.command(ToolIntent.GRASP, parameters={'grip_force_n': 100.})
    assert commands == ['close']
    assert logical.requested_grip_force_n == 100.


def test_future_linkage_effort_limit_rejects_before_motion():
    gripper, drive = actuator()
    # 100N needs 4.34Nm at q=0 but ~5.7Nm later in the closing stroke.
    gripper.model['effort'] = 5.
    with pytest.raises(ValueError, match='primary joint effort'):
        gripper.configure_command(grip_force_n=100., target_position=1.)
    assert drive.target is None
