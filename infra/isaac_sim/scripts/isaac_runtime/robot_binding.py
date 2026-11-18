"""Isaac robot bindings and USD drive authoring from the exported owner data."""
from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class RobotBinding:
    robot_path: str
    articulation_root_path: str
    base_link_path: str
    joint_names: tuple
    home_positions: tuple
    stiffness_scales: dict
    max_forces: dict = field(default_factory=dict)
    damping: dict = field(default_factory=dict)

    damping_scales: dict = field(default_factory=dict)

    @classmethod
    def from_runtime(cls, configuration):
        robot_path = configuration['cell']['isaac']['robot_prim_path']
        assembly = configuration['model_configuration']['isaac']
        robot = configuration['robot'].get('isaac', {})
        stiffness = dict(robot.get('joint_drive_stiffness_scales', {}))
        damping_scales = {}
        profile_name = assembly.get('robot_drive_profile')
        if profile_name:
            profile = robot['drive_profiles'][profile_name]
            for key in ('stiffness_scale', 'damping_scale'):
                value = profile[key]
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                    raise ValueError(f'Robot drive profile {key} must be finite and positive')
            stiffness = {name:stiffness.get(name, 1.)*profile['stiffness_scale']
                         for name in configuration['motion']['arm_joint_names']}
            damping_scales = {name:profile['damping_scale'] for name in configuration['motion']['arm_joint_names']}
        return cls(
            robot_path,
            f"{robot_path}/{assembly['articulation_root']}",
            f"{robot_path}/{assembly['articulation_base']}",
            tuple(configuration['motion']['arm_joint_names']),
            tuple(configuration['home_joint_positions']),
            stiffness,
            configuration['robot'].get('isaac', {}).get('joint_drive_max_forces', {}),
            robot.get('joint_drive_damping', {}),
            damping_scales,
        )


def load_robot_binding():
    from .runtime_configuration import load_runtime_configuration
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

    for name, scale in binding.damping_scales.items():
        joint = _robot_joints(stage, binding, [name])[name]
        drive = usd_physics.DriveAPI.Get(joint, 'angular')
        if not drive or not drive.GetDampingAttr().IsValid():
            raise RuntimeError(f'angular drive damping unavailable for {name}')
        damping = drive.GetDampingAttr()
        damping.Set(float(damping.Get()) * scale)

    for name, value in binding.max_forces.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise RuntimeError(f'{name} max force must be finite and positive')
        if name not in binding.joint_names:
            raise RuntimeError(f'unknown arm joint: {name}')
        joint = _robot_joints(stage, binding, [name])[name]
        drive = usd_physics.DriveAPI.Get(joint, 'angular')
        if not drive or not drive.GetMaxForceAttr().IsValid():
            raise RuntimeError(f'angular drive max force unavailable for {name}')
        drive.GetMaxForceAttr().Set(value)

    for name, value in binding.damping.items():
        if (name not in binding.joint_names or isinstance(value, bool)
                or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0):
            raise RuntimeError(f'{name} damping requires an arm joint and finite nonnegative value')
        joint = _robot_joints(stage, binding, [name])[name]
        drive = usd_physics.DriveAPI.Get(joint, 'angular')
        if not drive or not drive.GetDampingAttr().IsValid():
            raise RuntimeError(f'angular drive damping unavailable for {name}')
        drive.GetDampingAttr().Set(value)


def author_initial_pose(stage, binding, urdf, extra_joint_positions=None):
    """Initialize a stopped Scene at Robot HOME, with consistent body and joint states.

    This is setup, not a motion command. Play-time movements use the ROS backend.
    """
    import numpy as np
    import omni.timeline
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    from isaac_runtime.urdf_kinematics import link_transforms
    if not omni.timeline.get_timeline_interface().is_stopped():
        raise RuntimeError('Initial robot pose may only be authored while stopped')
    positions=dict(zip(binding.joint_names,binding.home_positions))
    positions.update(extra_joint_positions or {})
    transforms=link_transforms(urdf,positions)
    root_world=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(binding.robot_path))).T
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(binding.robot_path+'/'):
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() in transforms:
            parent_world=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent())).T
            local=np.linalg.inv(parent_world)@root_world@transforms[prim.GetName()]
            UsdGeom.Xformable(prim).MakeMatrixXform().Set(Gf.Matrix4d(*local.T.reshape(-1).tolist()))
            body=UsdPhysics.RigidBodyAPI(prim)
            body.CreateVelocityAttr(Gf.Vec3f(0.));body.CreateAngularVelocityAttr(Gf.Vec3f(0.))
        if prim.GetName() in positions:
            kind='angular' if prim.IsA(UsdPhysics.RevoluteJoint) else 'linear'
            state=PhysxSchema.JointStateAPI.Apply(prim,kind)
            value=positions[prim.GetName()]
            state.CreatePositionAttr(math.degrees(value) if kind=='angular' else value)
            state.CreateVelocityAttr(0.)
