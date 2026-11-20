"""Create the selected articulated robot and a metric Isaac World."""


def build_robot_world(configuration, world_cls=None, add_reference_to_stage=None):
    from pxr import Gf, PhysxSchema, UsdLux, UsdPhysics
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
    contracted={name:model['upper'] for name,model in tool.get('joint_model', {}).items()}
    author_initial_pose(stage,RobotBinding.from_runtime(configuration),ET.parse(model.generated_urdf).getroot(),contracted)
    old = world.scene.get_object('selected_robot')
    if old is not None:
        world.scene.remove_object('selected_robot', registry_only=True)
    robot = world.scene.add(SingleArticulation(prim_path=robot_path, name='selected_robot'))
    ground = world.scene.get_object('default_ground_plane')
    if ground is not None:
        world.scene.remove_object('default_ground_plane')
    UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000.)
    return world, robot
