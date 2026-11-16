"""Editable run settings; Bringup owns validation for GUI and other producers."""
import math

# path, display label with unit, minimum, maximum, advanced, modes
FIELDS = [
    ('move_duration_s', '논리 구동 시간 (s)', 0.001, 60., False, ('logical',)),
    ('physics.speed_fraction', '구동 속도 비율', 0.001, 1., False, ('physics',)),
    ('physics.grip_force_n', '초기 기본 파지력 합계 (N)', 40., 470., False, ('physics',)),
    ('physics.drive_stiffness', 'Drive 강성 (N·m/rad)', 0.000001, 1e6, True, ('physics',)),
    ('physics.drive_damping', 'Drive 감쇠 (N·m·s/rad)', 0.000001, 1e6, True, ('physics',)),
    ('physics.position_tolerance_rad', '위치 허용오차 (rad)', 0.000001, 1., True, ('physics',)),
    ('physics.stopped_velocity_rad_s', '정지 속도 한계 (rad/s)', 0.000001, 10., True, ('physics',)),
    ('physics.blocked_dwell_s', '구동 정지 판정 시간 (s)', 0.000001, 60., True, ('physics',)),
    ('physics.holding.min_contact_force_per_finger_n', '손가락별 최소 접촉력 (N)', 0.000001, 1000., True, ('fixed_attach',)),
    ('physics.holding.width_tolerance_mm', '파지 폭 허용오차 (mm)', 0.000001, 85., True, ('physics',)),
    ('physics.holding.stopped_dwell_s', '파지 정지 판정 시간 (s)', 0.000001, 60., True, ('physics',)),
    ('physics.holding.held_position_tolerance_rad', '파지 위치 허용오차 (rad)', 0.000001, 1., True, ('physics',)),
    ('physics.release.open_tolerance_mm', '해제 열림 허용오차 (mm)', 0.000001, 84.999, True, ('physical',)),
    ('physics.release.position_stability_tolerance_rad', '해제 안정 허용오차 (rad)', 0.000001, 1., True, ('physical',)),
    ('physics.release.stable_duration_s', '해제 안정 시간 (s)', 0.000001, 60., True, ('physical',)),
    ('physics.contact_material.static_friction', '정지 마찰계수', 0., 10., True, ('physics',)),
    ('physics.contact_material.dynamic_friction', '동적 마찰계수', 0., 10., True, ('physics',)),
    ('physics.contact_material.restitution', '반발계수', 0., 1., True, ('physics',)),
]


def available_fields(runtime):
    actuator, grasp = runtime['actuator'], runtime['grasp']
    for field in FIELDS:
        modes = field[5]
        if actuator in modes or (actuator == 'physics' and grasp in modes):
            try:
                parameter_value(runtime, field[0])
            except KeyError:
                continue
            yield field


def parameter_value(runtime, path):
    value = runtime
    for key in path.split('.'):
        value = value[key]
    return value


def apply_tool_parameters(runtime, overrides):
    fields = {field[0]: field for field in available_fields(runtime)}
    for path, value in overrides.items():
        if path not in fields:
            raise ValueError(f'Unsupported parameter for selected Grasp Mode: {path}')
        field = fields[path]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not field[2] <= value <= field[3]:
            raise ValueError(f'{field[1]} must be in [{field[2]}, {field[3]}]')
        target = runtime
        keys = path.split('.')
        for key in keys[:-1]:
            target = target[key]
        target[keys[-1]] = value
    if runtime['actuator'] == 'physics':
        material = runtime['physics']['contact_material']
        if material['dynamic_friction'] > material['static_friction']:
            raise ValueError('동적 마찰계수는 정지 마찰계수 이하여야 합니다.')
