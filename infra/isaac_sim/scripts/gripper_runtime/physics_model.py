"""Tool-only pre-Play USD assembly overlay; source assets remain unchanged."""
import math


def prepare(*, configuration, stage):
    from pxr import Sdf
    session = stage.GetSessionLayer()
    # Replace the Tool overlay on explicit scene reassembly, including return to logical.
    session.subLayerPaths = [path for path in session.subLayerPaths if 'SFTwinToolPhysics' not in path]
    settings = configuration['tool']['runtime']
    if settings.get('actuator', 'logical') == 'logical':
        return
    if settings.get('actuator') != 'physics':
        raise ValueError('Unsupported Tool actuator implementation')
    from pxr import Usd, UsdPhysics, PhysxSchema, UsdShade
    from isaac_runtime.robot_binding import RobotBinding
    binding = RobotBinding.from_runtime(configuration)
    root = binding.robot_path.rstrip('/')
    model = settings['joint_model']
    joints = {prim.GetName(): prim for prim in stage.Traverse()
              if str(prim.GetPath()).startswith(root + '/') and prim.GetName() in model}
    if set(joints) != set(model):
        raise ValueError('Selected Tool joints are absent from the installed Assembly')
    physics = settings['physics']
    primary = physics['primary_joint']
    # Session-layer opinions prevent mode settings leaking into generated USDs.
    overlay = Sdf.Layer.CreateAnonymous('SFTwinToolPhysics.usda')
    session.subLayerPaths.insert(0, overlay.identifier)
    with Usd.EditContext(stage, overlay):
        for name, item in model.items():
            prim = joints[name]
            joint = UsdPhysics.RevoluteJoint(prim)
            if not joint or abs(joint.GetLowerLimitAttr().Get() - math.degrees(item['lower'])) > 0.01 or abs(joint.GetUpperLimitAttr().Get() - math.degrees(item['upper'])) > 0.01:
                raise ValueError(f'Tool USD/URDF joint limits disagree: {name}; rebuild Assembly')
            max_velocity = math.degrees(item['velocity'] * physics['speed_fraction'])
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(max_velocity)
            drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
            if name == primary:
                drive.CreateTypeAttr('force')
                drive.CreateStiffnessAttr(math.radians(physics['drive_stiffness']))
                drive.CreateDampingAttr(math.radians(physics['drive_damping']))
                low, high = physics['drive_torque_range_nm']
                drive.CreateMaxForceAttr(low + physics['force_setting'] * (high - low))
                drive.CreateTargetPositionAttr(0.0)
                drive.CreateTargetVelocityAttr(0.0)
            else:
                # Disable redundant drives; coupling transmits contact resistance.
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(0.0)
                drive.CreateMaxForceAttr(0.0)
                relation = item['mimic']
                mimic = PhysxSchema.PhysxMimicJointAPI.Apply(prim, UsdPhysics.Tokens.rotX)
                mimic.CreateReferenceJointRel().SetTargets([joints[relation['joint']].GetPath()])
                mimic.CreateGearingAttr(-relation['multiplier'])
                mimic.CreateOffsetAttr(-math.degrees(relation['offset']))
        material = UsdShade.Material.Define(stage, root + '/ToolContactMaterial')
        surface = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        contact = physics['contact_material']
        surface.CreateStaticFrictionAttr(contact['static_friction'])
        surface.CreateDynamicFrictionAttr(contact['dynamic_friction'])
        surface.CreateRestitutionAttr(contact['restitution'])
        link_names = configuration['tool']['contact_links']['payload_touch_links']
        for prim in stage.Traverse():
            if prim.GetName() in link_names and str(prim.GetPath()).startswith(root + '/'):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants, 'physics')
