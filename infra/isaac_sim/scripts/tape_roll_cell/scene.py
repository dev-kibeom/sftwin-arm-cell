"""Cell scene: epoxy floor, robot pedestal, indexed infeed and vertical outfeed."""
from tape_roll_cell.material import TapeRoll


def build_scene(*, configuration, stage=None, world_cls=None, add_reference_to_stage=None, **unused):
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, UsdShade
    world = None
    robot = None
    if stage is None:
        from isaac_runtime.robot_scene import build_robot_world
        world, robot = build_robot_world(configuration, world_cls, add_reference_to_stage)
        stage = world.stage
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
