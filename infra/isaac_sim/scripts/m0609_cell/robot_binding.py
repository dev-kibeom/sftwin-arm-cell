"""Isaac robot bindings and USD drive authoring from the exported owner data."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RobotBinding:
    robot_path: str
    articulation_root_path: str
    base_link_path: str
    joint_names: tuple
    home_positions: tuple
    stiffness_scales: dict

    @classmethod
    def from_runtime(cls, configuration):
        robot_path = configuration['cell']['isaac']['robot_prim_path']
        assembly = configuration['model_configuration']['isaac']
        return cls(
            robot_path,
            f"{robot_path}/{assembly['articulation_root']}",
            f"{robot_path}/{assembly['articulation_base']}",
            tuple(configuration['motion']['arm_joint_names']),
            tuple(configuration['home_joint_positions']),
            configuration['robot'].get('isaac', {}).get('joint_drive_stiffness_scales', {}),
        )


def load_robot_binding():
    from runtime_configuration import load_runtime_configuration
    return RobotBinding.from_runtime(load_runtime_configuration())


def _robot_joints(stage, binding, names):
    # Include the slash so a sibling such as RobotExtra cannot match Robot.
    prefix = binding.robot_path + '/'
    joints = {}
    for prim in stage.Traverse():
        if (prim.IsValid() and str(prim.GetPath()).startswith(prefix)
                and prim.GetName() in names):
            name = prim.GetName()
            if name in joints:
                raise RuntimeError(f'ambiguous joint in robot subtree: {name}')
            joints[name] = prim
    missing = [name for name in names if name not in joints]
    if missing:
        raise RuntimeError(f'joints not found in robot stage: {missing}')
    return joints


def author_home_targets(stage, binding, usd_physics):
    """Convert canonical radians to USD angular degrees, also on Stop/Reset."""
    joints = _robot_joints(stage, binding, binding.joint_names)
    for name, position_rad in zip(binding.joint_names, binding.home_positions):
        drive = usd_physics.DriveAPI.Get(joints[name], 'angular')
        if not drive:
            raise RuntimeError(f'angular drive not found for HOME joint: {name}')
        target = drive.GetTargetPositionAttr()
        if not target.IsValid():
            raise RuntimeError(f'angular drive target unavailable for HOME joint: {name}')
        target.Set(math.degrees(position_rad))
        joints[name].SetCustomDataByKey('sf_twin:initial_home_position_rad', position_rad)


def apply_joint_drive_profile(stage, binding, usd_physics):
    """Scale imported drive stiffness once after Scene construction, before Play."""
    joints = _robot_joints(stage, binding, binding.stiffness_scales)
    for name, scale in binding.stiffness_scales.items():
        drive = usd_physics.DriveAPI.Get(joints[name], 'angular')
        if not drive:
            raise RuntimeError(f'angular drive not found for {name}')
        stiffness = drive.GetStiffnessAttr()
        if not stiffness.IsValid():
            raise RuntimeError(f'angular drive stiffness unavailable for {name}')
        base_stiffness = float(stiffness.Get())
        if not math.isfinite(base_stiffness) or base_stiffness <= 0.0:
            raise RuntimeError(f'{name} imported angular stiffness must be positive')
        stiffness.Set(base_stiffness * scale)
