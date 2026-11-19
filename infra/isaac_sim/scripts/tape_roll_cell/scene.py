"""Cell scene: epoxy floor, robot pedestal, indexed infeed and vertical outfeed."""
from tape_roll_cell.material import TapeRoll


def build_scene(*, configuration, stage=None, world_cls=None, add_reference_to_stage=None, **unused):
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, UsdShade, UsdLux
    world = None
    robot = None
    if stage is None:
        import os
        from pathlib import Path
        from isaac_runtime.model_build_configuration import ModelBuildConfiguration
        from isaacsim.core.prims import SingleArticulation
        if world_cls is None:
            from isaacsim.core.api import World
            world_cls = World
        world = world_cls.instance() or world_cls(stage_units_in_meters=1., physics_dt=1/60, rendering_dt=1/60)
        stage = world.stage
        if world.get_physics_context() is not None:
            world.set_simulation_dt(physics_dt=1/60, rendering_dt=1/60)
            world.get_physics_context().set_gravity(-9.81)
        else:
            # Kit initializes World asynchronously during common startup.
            # Author gravity without dereferencing the pending context.
            physics = UsdPhysics.Scene.Define(stage, '/physicsScene')
            physics.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1))
            physics.CreateGravityMagnitudeAttr(9.81)
            PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim()).CreateTimeStepsPerSecondAttr(60)
        model = ModelBuildConfiguration.from_runtime(configuration, Path(os.environ['SFTWIN_PROJECT_ROOT']))
        if add_reference_to_stage is None:
            from isaacsim.core.utils.stage import add_reference_to_stage
        robot_path = configuration['cell']['isaac']['robot_prim_path']
        if stage.GetPrimAtPath(robot_path):
            stage.RemovePrim(robot_path)
        add_reference_to_stage(str(model.usd), robot_path)
        import xml.etree.ElementTree as ET
        from isaac_runtime.robot_binding import RobotBinding, author_initial_pose
        tool=configuration['tool']['runtime']
        contracted={name:model['upper'] for name,model in tool['joint_model'].items()}
        author_initial_pose(stage,RobotBinding.from_runtime(configuration),ET.parse(model.generated_urdf).getroot(),contracted)
        old = world.scene.get_object('selected_robot')
        if old is not None:
            world.scene.remove_object('selected_robot', registry_only=True)
        robot = world.scene.add(SingleArticulation(prim_path=robot_path, name='selected_robot'))
        ground = world.scene.get_object('default_ground_plane')
        if ground is not None:
            world.scene.remove_object('default_ground_plane')
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000.)
    if stage.GetPrimAtPath('/World/defaultGroundPlane'):
        stage.RemovePrim('/World/defaultGroundPlane')
    config = configuration['cell']['tape_roll']
    roll = TapeRoll(**config['material'])
    root = '/World/TapeRollCell'
    if stage.GetPrimAtPath(root):
        stage.RemovePrim(root)
    UsdGeom.Xform.Define(stage, root)
    from tape_roll_cell.teaching_program import author_equipment_frames
    author_equipment_frames(stage, configuration['cell']['teaching_program'], configuration)
    material = UsdShade.Material.Define(stage, root+'/ContactMaterial')
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(roll.static_friction)
    physics_material.CreateDynamicFrictionAttr(roll.dynamic_friction)
    physics_material.CreateRestitutionAttr(0.)

    def collision(prim):
        UsdPhysics.CollisionAPI.Apply(prim)
        contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        contact.CreateContactOffsetAttr(.0002)
        contact.CreateRestOffsetAttr(0.)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, materialPurpose='physics')

    from tape_roll_cell.presentation import author_presentation
    view = configuration['cell'].get('presentation', {}).get('overview',
        dict(eye=[1.4, -1.25, 1.1], target=[.28, .18, .36]))
    looks = author_presentation(stage, config, roll, root, view, configuration['cell']['environment'])

    roll_paths = []  # Supply creates full cartridges on the physical infeed belt.
    for item in config['static_boxes']:
        cube = UsdGeom.Cube.Define(stage, item['id'])
        cube.CreateSizeAttr(1.)
        pose=item.get('local_pose',item['pose'])
        cube.AddTranslateOp().Set(Gf.Vec3d(*pose['position_m']))
        q = pose['quaternion_xyzw']
        cube.AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
        cube.AddScaleOp().Set(Gf.Vec3f(*item['geometry']['dimensions_m']))
        look = 'floor' if item['id'].endswith('/EpoxyFloor') else 'steel'
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(looks[look])
        collision(cube.GetPrim())
    from tape_roll_cell.supply import ConfiguredTapeSupply
    supply = ConfiguredTapeSupply(len(config['slots']),
        supply_id=configuration['cell']['configured_supply']['supply_id'],
        material_id=configuration['cell']['configured_supply']['material_id'])
    supply.unload()
    return dict(stage=stage, world=world, robot=robot, configuration=configuration,
                roll_path=None, roll_paths=roll_paths, root_path=root, roll=roll, supply=supply)
