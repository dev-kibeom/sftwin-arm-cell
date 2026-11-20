"""Bringup validation for temporary launcher settings on operational profiles."""
from dataclasses import dataclass


@dataclass(frozen=True)
class Field:
    label: str
    minimum: int | float
    maximum: int | float
    integer: bool = False

    def accepts(self, value):
        number = type(value) is int if self.integer else type(value) in (int, float)
        return number and self.minimum <= value <= self.maximum

    def validate(self, path, value):
        if not self.accepts(value):
            raise ValueError(
                f'Invalid experiment setting {path}: expected '
                f'{"an integer" if self.integer else "a number"} '
                f'in [{self.minimum}, {self.maximum}]'
            )
        return int(value) if self.integer else float(value)


FIELDS = {
    'simulation': {
        'external_handoff_default_quantity': Field('기본 자재 공급 수량', 1, 2**31 - 1, integer=True),
    },
    'motion': {
        'planning_time_s': Field('계획 시간 (s)', 0.05, 300.0),
        'planning_velocity_scaling_factor': Field('계획 속도 배율', 0.0001, 1.0),
        'planning_acceleration_scaling_factor': Field('계획 가속도 배율', 0.0001, 1.0),
    },
    'vision': {
        'sync_slop_ms': Field('캘리브레이션 RGB-D 동기화 허용 시간 (ms)', 0.0, 9_223_372_036_854.0),
        'reusable_pair_max_age_ms': Field('기존 Depth 관측 재사용 한계 (ms)', 0.0, 200.0),
    },
}


def available_fields(section, component, *, configured_supply=False, owned_motion_keys=()):
    if configured_supply and section != 'motion':
        return {}
    return {
        key: field for key, field in FIELDS[section].items()
        if key in component and (section != 'motion' or key not in owned_motion_keys)
    }


def apply_component_parameters(config, overrides, *, configured_supply=False, owned_motion_keys=()):
    if not isinstance(overrides, dict):
        raise ValueError('Component experiment settings require a mapping')
    for section, edits in overrides.items():
        if section not in FIELDS or section not in config or not isinstance(edits, dict):
            raise ValueError(f'Unsupported experiment section: {section}')
        fields = available_fields(section, config[section], configured_supply=configured_supply,
                                  owned_motion_keys=owned_motion_keys)
        for key, value in edits.items():
            if key not in fields:
                raise ValueError(f'Unsupported experiment setting: {section}.{key}')
            config[section][key] = fields[key].validate(f'{section}.{key}', value)
