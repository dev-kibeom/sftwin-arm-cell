"""Isolated teaching-pose sequence using the existing model joint-drive runner."""
import hashlib
import math
from pathlib import Path

import numpy as np

from internal_grasp_geometry import link_transforms, slab_radius


def run_internal_grasp(config, geometry, stage, world, robot, prims, arm_indices,
                       finger_indices, step, command, report, cycles, task_execution_library=None):
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    from isaacsim.core.prims import SingleRigidPrim
    from isaacsim.core.utils.types import ArticulationAction
    from gripper_runtime.grasp_joint import UsdFixedJointOwner
    from gripper_runtime.tool_plugin import ToolIntent, radial_tool_runtime

    experiment = config['internal_grasp_experiment']
    fixture = geometry.fixture(experiment['fixture']['insertion_depth_m'], experiment['fixture']['thickness_m'])
    report['fixture'] = fixture
    report['teaching_poses_rad'] = {key: experiment[key] for key in ('approach', 'insert', 'transfer', 'retreat')}
    report['experiment_source_sha256'] = {name: hashlib.sha256((Path(__file__).parent / name).read_bytes()).hexdigest()
                                           for name in ('internal_grasp_experiment.py', 'internal_grasp_geometry.py', 'grasp_cycle_fixture.py',
                                                        'gripper_runtime/grasp_strategy.py', 'gripper_runtime/tool_plugin.py')}
    report['arm_drive_stiffness_scale'] = experiment['arm_drive_stiffness_scale']
    arm_names = config['robot']['motion']['arm_joint_names']
    settings = config['tool']['runtime']
    tolerance = settings['position_tolerance_m']
    primary_index = list(settings['joint_model']).index(settings['primary_joint'])
    tcp_prim = prims['sf_grasp_tcp']
    ring_path = '/World/InternalGraspRing'
    attachment = UsdFixedJointOwner(stage, str(tcp_prim.GetPath()))
    ring = None

    def transform(prim):
        matrix = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T
        if not np.all(np.isfinite(matrix)):
            raise RuntimeError('Nonfinite live transform')
        return matrix

    def relative_ring():
        return np.linalg.inv(transform(tcp_prim)) @ transform(stage.GetPrimAtPath(ring_path))

    def record(name, **extra):
        entry = dict(phase=name, simulation_time_s=world.current_time,
                     finger_positions_m=robot.get_joint_positions(joint_indices=np.array(finger_indices)).tolist(),
                     tcp_world=transform(tcp_prim).tolist(), ring_world=transform(stage.GetPrimAtPath(ring_path)).tolist(),
                     attached=attachment.owned_joint_valid(), **extra)
        report['cycles'][-1]['phases'].append(entry)
        print(f'C10 internal grasp: {name} attached={entry["attached"]}', flush=True)

    def move_arm(destination, inspect=None):
        start = np.array(robot.get_joint_positions(joint_indices=np.array(arm_indices)))
        target = np.array(destination)
        frames = math.ceil(experiment['move_duration_s'] * 120)
        # Smooth endpoint velocity, bounded by selected URDF velocity limits.
        for name, delta in zip(arm_names, np.abs(target-start)):
            limit = float(geometry.urdf.find(f"joint[@name='{name}']/limit").get('velocity'))
            if 1.5*delta/experiment['move_duration_s'] > limit:
                raise RuntimeError(f'Teaching interpolation exceeds velocity: {name}')
        for frame in range(1, frames+1):
            t = frame/frames
            robot.apply_action(ArticulationAction(joint_positions=start+(target-start)*(3*t*t-2*t*t*t), joint_indices=np.array(arm_indices)))
            step()
            if inspect is not None:
                inspect()
        for _ in range(120):
            step()
            if inspect is not None:
                inspect()
        measured = robot.get_joint_positions(joint_indices=np.array(arm_indices))
        if np.max(np.abs(measured-target)) > .002:
            raise RuntimeError(f'Teaching pose not reached: {measured-target}')

    def move_fingers(destination):
        start = float(robot.get_joint_positions(joint_indices=np.array(finger_indices))[primary_index])
        frames = math.ceil(settings['sweep_duration_s']*120)
        for frame in range(1, frames+1):
            command(start+(destination-start)*frame/frames)
            step()
        for _ in range(120):
            values = step()
        for name, value in zip(settings['joint_model'], values):
            mimic = settings['joint_model'][name].get('mimic', {})
            expected = destination*mimic.get('multiplier', 1.)+mimic.get('offset', 0.)
            if abs(value-expected) > tolerance:
                raise RuntimeError('Measured finger target not reached')
        return float(values[primary_index])

    requirement = {key: fixture[key] for key in ('insertion_depth_m', 'thickness_m', 'inner_radius_m')}
    requirement['strategy'] = 'internal_radial'
    tool = radial_tool_runtime(move_fingers, config['tool'], geometry, requirement)
    report['grasp_strategy'] = dict(name=tool.capture.name, solution=tool.capture.solution)

    def make_ring(pose):
        prim = UsdGeom.Xform.Define(stage, ring_path)
        prim.AddTransformOp().Set(Gf.Matrix4d(pose.T.tolist()))
        mesh = UsdGeom.Mesh.Define(stage, ring_path+'/Visual')
        # Annular mesh, never a solid cylinder or convex collision approximation.
        points = []
        segments = 96
        for z in (-fixture['thickness_m']/2, fixture['thickness_m']/2):
            for radius in (fixture['inner_radius_m'], fixture['outer_radius_m']):
                points.extend([(radius*math.cos(i*2*math.pi/segments), radius*math.sin(i*2*math.pi/segments), z)
                               for i in range(segments)])
        faces = []
        for i in range(segments):
            j = (i+1)%segments
            faces.extend([[i,j,segments+j,segments+i],
                          [2*segments+i,3*segments+i,3*segments+j,2*segments+j],
                          [i,2*segments+i,2*segments+j,j],
                          [segments+i,segments+j,3*segments+j,3*segments+i]])
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4]*len(faces))
        mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([(0.15, 0.55, 0.85)])
        UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim())
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim.GetPrim()).CreateDisableGravityAttr(True)
        body = SingleRigidPrim(ring_path, name='internal_grasp_ring', mass=.05, reset_xform_properties=False)
        body.initialize()
        return body

    try:
        cycle = 0
        while cycles == 0 or cycle < cycles:
            # Reuse the rigid body across repetitions: deleting a body invalidates
            # Isaac tensor views shared with the robot articulation.
            # The fixture is positioned while stationary. In native mode the
            # common BT performs contraction and approach after setup.
            if task_execution_library is None:
                tool.command(ToolIntent.RELEASE)
                move_arm(experiment['approach'])
            target_tcp = link_transforms(geometry.urdf, dict(zip(arm_names, experiment['insert'])))['sf_grasp_tcp']
            # Respect the live base transform (including any importer root transform).
            base_world = transform(prims['base_link'])
            target_tcp = base_world @ target_tcp
            ring_pose = target_tcp.copy()
            ring_pose[:3, 3] -= target_tcp[:3, 2]*fixture['insertion_depth_m']
            if ring is None:
                ring = make_ring(ring_pose)
            else:
                quaternion = Gf.Matrix4d(ring_pose.T.tolist()).ExtractRotationQuat()
                ring.set_world_pose(position=ring_pose[:3,3], orientation=np.array([quaternion.GetReal(), *quaternion.GetImaginary()]))
                ring.set_linear_velocity(np.zeros(3))
                ring.set_angular_velocity(np.zeros(3))
                UsdGeom.Imageable(stage.GetPrimAtPath(ring_path)).MakeVisible()
            report['cycles'].append({'phases': [], 'max_insertion_radial_error_m': 0., 'minimum_insertion_clearance_m': float('inf')})
            current = report['cycles'][-1]
            if task_execution_library is None:
                record('CONTRACT')

            def inspect_insertion():
                relative = relative_ring()
                # Qualify the entire swept distal envelope in the ring frame.
                # FK uses measured joint positions, not command endpoints.
                measured = dict(zip(robot.dof_names, robot.get_joint_positions()))
                fk = link_transforms(geometry.urdf, measured)
                live_tcp = transform(tcp_prim)
                if not np.allclose(live_tcp, base_world @ fk['sf_grasp_tcp'], atol=.0003):
                    raise RuntimeError('Selected live TCP differs from measured URDF FK')
                triangles = geometry.triangles(measured[settings['primary_joint']])
                tcp_to_ring = np.linalg.inv(relative)
                triangles = triangles @ tcp_to_ring[:3, :3].T + tcp_to_ring[:3, 3]
                if triangles[:,:,2].max() < -fixture['thickness_m']/2:
                    return
                radius = slab_radius(triangles, -fixture['thickness_m']/2, fixture['thickness_m']/2)
                clearance = fixture['inner_radius_m']-radius
                current['minimum_insertion_clearance_m'] = min(current['minimum_insertion_clearance_m'], clearance)
                current['max_insertion_radial_error_m'] = max(current['max_insertion_radial_error_m'], float(np.linalg.norm(relative[:2,3])))
                if clearance < .001:
                    raise RuntimeError(f'Insertion collision envelope has insufficient clearance: {clearance}')

            capture_relative = None
            initial_ring_position = None
            released = None

            def inspect_attachment():
                if not attachment.owned_joint_valid():
                    raise RuntimeError('Logical Attach disappeared during transfer')
                actual = relative_ring()
                error = float(np.linalg.norm(actual[:3,3]-capture_relative[:3,3]))
                if error > .001 or np.max(np.abs(actual[:3,:3]-capture_relative[:3,:3])) > .01:
                    raise RuntimeError(f'Ring did not follow selected TCP: {error}')
                current['max_attachment_position_error_m'] = max(current.get('max_attachment_position_error_m',0.), error)

            def fixture_callback(event, name):
                nonlocal capture_relative, initial_ring_position, released
                if event == 0:  # Existing fixture backend for MoveTaught.
                    if name == 'approach':
                        move_arm(experiment[name])
                    elif name == 'insert':
                        move_arm(experiment[name], inspect_insertion)
                        record('INSERT', ring_in_tcp=relative_ring().tolist())
                    elif name == 'transfer':
                        move_arm(experiment[name], inspect_attachment)
                        displacement = float(np.linalg.norm(transform(stage.GetPrimAtPath(ring_path))[:3,3]-initial_ring_position))
                        if displacement < .05:
                            raise RuntimeError('Attached ring did not transfer')
                        record('TRANSFER', ring_displacement_m=displacement)
                    elif name == 'retreat':
                        move_arm(experiment[name], inspect_insertion)
                        final = transform(stage.GetPrimAtPath(ring_path))
                        if attachment.has_owned_joint() or np.linalg.norm(final[:3,3]-released[:3,3]) > .001:
                            raise RuntimeError('Released ring moved with retreating TCP')
                        if relative_ring()[2,3] < fixture['thickness_m']/2 + .005:
                            raise RuntimeError('TCP did not retreat clear of ring')
                        record('RETREAT', released_ring_drift_m=float(np.linalg.norm(final[:3,3]-released[:3,3])))
                    else:
                        raise ValueError(f'Unknown fixture motion: {name}')
                elif event == 1:  # Semantic GRASP: measured expansion then attach.
                    tool.command(ToolIntent.GRASP)
                    measured = float(robot.get_joint_positions(joint_indices=np.array(finger_indices))[primary_index])
                    relative = relative_ring()
                    tool.capture.qualify_capture(relative[:3,3], relative[:3,:3], measured)
                    if not attachment.create(ring_path) or not attachment.owned_joint_valid():
                        raise RuntimeError('Logical Attach ownership failed')
                    capture_relative = relative.copy()
                    initial_ring_position = transform(stage.GetPrimAtPath(ring_path))[:3,3].copy()
                    record('EXPAND/GRASP', ring_in_tcp=relative.tolist())
                elif event == 2:  # Semantic RELEASE, including initial contraction.
                    tool.command(ToolIntent.RELEASE)
                    if attachment.has_owned_joint():
                        if not attachment.remove_owned() or attachment.has_owned_joint():
                            raise RuntimeError('Logical Detach failed')
                        ring.set_linear_velocity(np.zeros(3))
                        ring.set_angular_velocity(np.zeros(3))
                        released = transform(stage.GetPrimAtPath(ring_path)).copy()
                        record('CONTRACT/RELEASE')
                    elif not current['phases']:
                        record('CONTRACT')
                elif event == 3:  # Fresh fixture Holding observation.
                    if attachment.owned_joint_valid():
                        inspect_attachment()
                        return 1
                    measured = float(robot.get_joint_positions(joint_indices=np.array(finger_indices))[primary_index])
                    return 2 if abs(measured-fixture['release_position_m']) <= tolerance else 0
                elif event == 4:
                    return 0 if attachment.owned_joint_valid() else 9
                elif event == 5:
                    return 9 if attachment.has_owned_joint() else 0
                elif event == 6:
                    tool.command(ToolIntent.STOP)
                else:
                    raise ValueError(f'Unknown fixture event: {event}')
                return 0

            if task_execution_library is not None:
                from grasp_cycle_fixture import run_task_grasp_cycle
                current['task_execution'] = run_task_grasp_cycle(
                    task_execution_library, tool.capture.name, tool.capture.solution, fixture_callback)
            else:
                # Compatibility for the original PR #103 isolated experiment.
                for event, name in ((0, 'insert'), (1, ''), (0, 'transfer'), (2, ''), (0, 'retreat')):
                    fixture_callback(event, name)
            UsdGeom.Imageable(stage.GetPrimAtPath(ring_path)).MakeInvisible()
            cycle += 1
            if cycles == 0:
                report['cycles'] = report['cycles'][-10:]
    finally:
        if attachment.has_owned_joint():
            attachment.remove_owned()
        if stage.GetPrimAtPath(ring_path).IsValid():
            stage.RemovePrim(ring_path)
