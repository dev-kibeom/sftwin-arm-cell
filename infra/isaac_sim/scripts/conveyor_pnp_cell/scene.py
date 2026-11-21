"""One native PhysX belt, a travelling side stop and one discharge slot."""
from uuid import uuid4
import time

from .supply import material_instance_id

ROOT = '/World/ConveyorPnP'


def box(stage, path, center, size, color, *, collision=True):
    from pxr import Gf, UsdGeom, UsdPhysics
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()


def build_scene(*, configuration, world_cls=None, add_reference_to_stage=None, **unused):
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics
    from isaac_runtime.robot_scene import build_robot_world
    from tape_roll_cell.conveyor_belts import ConveyorBelts
    stage = omni.usd.get_context().get_stage()
    if stage and stage.GetPrimAtPath(ROOT):
        # Rebuilding an active stream would destroy its physical reconciliation evidence.
        raise RuntimeError('Open a fresh stage before rebuilding the Conveyor cell')
    mount_path = configuration['cell']['isaac']['robot_prim_path'].rsplit('/', 1)[0]
    if mount_path != '/World':
        mount = UsdGeom.Xform.Define(stage, mount_path)
        from tape_roll_cell.teaching_program import base_matrix
        transform = base_matrix(configuration['cell']['world_to_base'])
        mount.MakeMatrixXform().Set(Gf.Matrix4d(*transform.T.flatten().tolist()))
    world, robot = build_robot_world(configuration, world_cls, add_reference_to_stage)
    stage = world.stage
    UsdGeom.Xform.Define(stage, ROOT)
    layout = configuration['cell']['conveyor']
    for item in layout['boxes']:
        box(stage, item['id'], item['pose']['position_m'], item['geometry']['dimensions_m'],
            (.17, .19, .22) if item['id'].endswith('InfeedBelt') else (.35, .39, .43))
    stopper = box(stage, ROOT+'/Stopper', layout['stopper_retracted_m'],
                  layout['stopper_size_m'], (.87, .5, .12))
    UsdPhysics.RigidBodyAPI.Apply(stopper).CreateKinematicEnabledAttr(True)
    for name in ('infeed', 'pick'):
        region = layout['regions'][name]
        marker = box(stage, ROOT+'/'+name+'Sensor', region['center'], region['size'],
                     (0., .8, .6), collision=False)
        UsdGeom.Gprim(marker).CreateDisplayOpacityAttr([.15])
    belts = ConveyorBelts(stage, layout['boxes'], graph_path=ROOT+'/ConveyorGraph')
    return dict(world=world, robot=robot, stage=stage, equipment=Equipment(stage, layout, belts,
                configuration['cell']['isaac']['robot_prim_path']))


class Equipment:
    def __init__(self, stage, layout, belts, robot_path):
        from pxr import PhysxSchema
        from omni.isaac.dynamic_control import _dynamic_control
        from omni.physx import get_physx_scene_query_interface
        self.stage, self.layout, self.belts = stage, layout, belts
        self.robot_path = robot_path
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.invalid = _dynamic_control.INVALID_HANDLE
        self.query = get_physx_scene_query_interface()
        self.surface = PhysxSchema.PhysxSurfaceVelocityAPI(stage.GetPrimAtPath(ROOT+'/InfeedBelt'))
        self.stop_position = stage.GetPrimAtPath(ROOT+'/Stopper').GetAttribute('xformOp:translate')
        self.material_type_id = ''
        self.stream_id = uuid4().hex
        self.cycle = 0
        self.spawned_ns = None
        self.last_discharge = None
        self.path = None
        self.speed = None
        self.fixtures = {item['id'] for item in layout['boxes']} | {ROOT+'/Stopper'}

    def spawn(self, material, cycle):
        from pxr import Gf, Sdf, UsdGeom
        from isaac_runtime.cuboid import author_cuboid_body
        if self.path is not None:
            return False
        path = f'{ROOT}/Part_{cycle}'
        if self.stage.GetPrimAtPath(path):
            return False
        part = UsdGeom.Xform.Define(self.stage, path)
        part.AddTranslateOp().Set(Gf.Vec3d(*self.layout['spawn_m']))
        prim = part.GetPrim()
        author_cuboid_body(self.stage, prim, self.layout['part_size_m'], self.layout['mass_kg'],
                          (.8, .45, .14))
        prim.CreateAttribute('arm_cell:material_type_id', Sdf.ValueTypeNames.String).Set(material)
        prim.CreateAttribute('arm_cell:material_instance_id', Sdf.ValueTypeNames.String).Set(
            material_instance_id(self.stream_id, cycle))
        self.path, self.cycle, self.material_type_id = path, cycle, material
        self.spawned_ns = time.monotonic_ns()
        return True

    def material_path(self, material_type_id, instance_id):
        if self.path and instance_id == material_instance_id(self.stream_id, self.cycle):
            prim = self.stage.GetPrimAtPath(self.path)
            if (prim and prim.GetAttribute('arm_cell:material_type_id').Get() == material_type_id
                    and prim.GetAttribute('arm_cell:material_instance_id').Get() == instance_id):
                return self.path
        return None

    def body(self, path):
        handle = self.dc.get_rigid_body(path)
        if handle == self.invalid:
            raise ValueError('Rigid body unavailable: '+path)
        return handle

    def inside(self, name, point):
        region = self.layout['regions'][name]
        return all(abs(p-c) <= size/2 for p, c, size in zip(point, region['center'], region['size']))

    def overlaps(self, name, matches):
        import carb
        region = self.layout['regions'][name]
        found = False

        def hit(shape):
            nonlocal found
            found |= matches(str(shape.rigid_body))
            return not found

        self.query.overlap_box(carb.Float3(*(d/2 for d in region['size'])),
                               carb.Float3(*region['center']), carb.Float4(0, 0, 0, 1), hit, False)
        return found

    def observe(self, permitted):
        from .equipment_io import Sensors
        import math
        if self.path:
            if not self.material_path(self.material_type_id, material_instance_id(self.stream_id, self.cycle)):
                raise ValueError('Tracked material identity unavailable')
            handle = self.body(self.path)
            pose = self.dc.get_rigid_body_pose(handle).p
            point = (pose.x, pose.y, pose.z)
            if not all(map(math.isfinite, point)):
                raise ValueError('Non-finite material pose')
        else:
            point = None

        def robot(path):
            return path.startswith(self.robot_path+'/')

        def unexpected(path):
            return path != self.path and path not in self.fixtures and not robot(path)

        if any(self.overlaps(name, unexpected) for name in ('belt', 'tray')):
            raise ValueError('Unidentified body on conveyor or tray')
        regions = {name: point is not None and self.inside(name, point)
                   for name in ('infeed', 'pick', 'tray', 'belt')}
        velocity = self.surface.GetSurfaceVelocityAttr().Get()
        belt_speed = velocity.GetLength() if self.surface.GetSurfaceVelocityEnabledAttr().Get() else 0.
        tolerance = self.layout['stopped_tolerance_m_s']
        stop_y = self.dc.get_rigid_body_pose(self.body(ROOT+'/Stopper')).p.y
        extended_y = self.layout['stopper_retracted_m'][1] - self.layout['stopper_travel_m']
        return Sensors(infeed=regions['infeed'], pick=regions['pick'], occupied=regions['tray'],
                       stopped=belt_speed <= tolerance,
                       extended=abs(stop_y-extended_y) < .001, path_clear=self.path is None or regions['tray'],
                       drive_available=bool(self.surface), on_belt=regions['belt'],
                       pick_zone_clear=not self.overlaps('robot_pick', robot),
                       discharge_zone_clear=not self.overlaps('robot_discharge', robot), permitted=permitted)

    def drive(self, speed):
        if self.speed != speed:
            self.belts.set_speeds(infeed=speed)
            self.speed = speed

    def step(self, command, sensors, dt, enabled):
        run = enabled and command.run and sensors.drive_available
        self.drive(self.layout['speed_m_s'] if run else 0.)
        if not enabled:
            return
        base = self.layout['stopper_retracted_m']
        target = base[1] - (self.layout['stopper_travel_m'] if command.extend else 0.)
        pos = self.stop_position.Get()
        stroke = self.layout['stopper_speed_m_s'] * dt
        pos[1] += max(-stroke, min(stroke, target-pos[1]))
        self.stop_position.Set(pos)
        if command.discharge and sensors.occupied:
            if not self.stage.RemovePrim(self.path):
                raise RuntimeError('Discharge could not remove the observed part')
            self.last_discharge = (self.cycle, self.spawned_ns, time.monotonic_ns())
            self.path = None
