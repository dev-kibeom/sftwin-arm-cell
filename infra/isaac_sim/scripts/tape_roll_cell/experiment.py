"""Live physical fixture backend for the existing native GraspCycle BT."""
import math
import hashlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from gripper_runtime.grasp_attachment import HoldingState
from gripper_runtime.tool_plugin import select_tool_runtime, ToolIntent
from gripper_runtime.radial_physical import radial_physical_runtime
from internal_grasp_geometry import link_transforms
from scene_entrypoint import run_scene_build


def run(*, configuration, geometry, stage, world, robot, prims, arm_indices,
        finger_indices, step, command, report, cycles, task_execution_library):
    from pxr import UsdGeom, UsdPhysics, PhysxSchema
    from isaacsim.core.prims import SingleRigidPrim
    from isaacsim.core.utils.types import ArticulationAction
    from gripper_runtime.physical_contacts import PhysicalContactObserver
    from grasp_cycle_fixture import run_task_grasp_cycle
    if not task_execution_library:
        raise ValueError('Tape roll requires the common native Task Execution fixture library')
    if cycles != 1:
        raise ValueError('Tape-roll validation requires --cycles 1; multiple-roll processing is out of scope')
    report['experiment_source_sha256'] = {str(p.relative_to(Path(__file__).parents[1])): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in list(Path(__file__).parent.glob('*.py')) + [Path(__file__).parents[1]/'gripper_runtime/radial_physical.py']}
    task_execution_library=str(Path(task_execution_library).resolve(strict=True))
    gravity_direction, gravity_magnitude=world.get_physics_context().get_gravity()
    report['gravity_vector_m_s2']=(np.array(gravity_direction)*gravity_magnitude).tolist()
    if not np.allclose(report['gravity_vector_m_s2'],[0,0,-9.81],atol=.01):
        raise RuntimeError('Physical validation requires normal Earth gravity')
    config = configuration['cell']['tape_roll']
    settings = configuration['tool']['runtime']
    roll_config = config['material']
    requirement = dict(strategy='internal_radial', insertion_depth_m=config['insertion_depth_m'],
        thickness_m=roll_config['width_m'], inner_radius_m=roll_config['inner_radius_m'])
    # Fail before creating a scene if the selected Tool cannot enter this bore.
    geometry.solve_bore(requirement['insertion_depth_m'],requirement['thickness_m'],requirement['inner_radius_m'])
    context = run_scene_build(configuration=configuration, stage=stage)
    roll_path = context['roll_path']
    tcp = prims['sf_grasp_tcp']
    arm_names = configuration['robot']['motion']['arm_joint_names']
    fingers = np.array(finger_indices)
    arms = np.array(arm_indices)
    primary_index = list(settings['joint_model']).index(settings['primary_joint'])

    def transform(prim):
        return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T

    def measured():
        return robot.get_joint_positions(joint_indices=fingers).tolist()

    binding = SimpleNamespace(geometry=geometry, requirement=requirement,
        write_primary=command, measure_fingers=measured, physical_settings=config['physical'])
    tool = select_tool_runtime(configuration['tool'],binding,
        implementations={'hand_e_c10_physical':radial_physical_runtime})
    actuator = tool.actuator
    # Contact reports are requested without disabling any collision pair.
    robot_bodies = set()
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith('/World/Robot/') and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            robot_bodies.add(str(prim.GetPath()))
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
    finger_bodies = []
    for name in settings['finger_tip_links']:
        prim = prims[name]
        while prim and not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim = prim.GetParent()
        if not prim:
            raise RuntimeError('No rigid body for selected finger')
        finger_bodies.append(str(prim.GetPath()))
    if len(set(finger_bodies)) != 3:
        raise RuntimeError('Three independent physical finger bodies required')
    world.reset()
    # Initialize the isolated fixture at its taught approach before any physics
    # step. HOME-to-cell travel is outside this single-pick experiment.
    initial_arm=np.array(config['teaching']['approach'])
    robot.set_joint_positions(initial_arm,joint_indices=arms)
    robot.apply_action(ArticulationAction(joint_positions=initial_arm,joint_indices=arms))
    roll = SingleRigidPrim(prim_path=roll_path,name='tape_roll')
    roll.initialize()
    observer = PhysicalContactObserver()
    current = dict(phases=[], samples=[], relation_committed=False)
    report.update(scope='horizontal physical tape roll, native Task BT fixture; no Mission/vision',
                  execution_mode='common_task_bt_physical_fixture', material=roll_config,
                  teaching=config['teaching'], finger_bodies=finger_bodies,
                  cycles=[current], gravity_m_s2=9.81,
                  collision_policy='all existing pairs enabled; unexpected robot/fixture contact fails')
    relation = None
    sampling_elapsed = 0.
    require_held = False

    def record(phase):
        print(f'Tape roll phase: {phase}',flush=True)
        current['phases'].append(dict(phase=phase, simulation_time_s=world.current_time,
            roll_world=transform(stage.GetPrimAtPath(roll_path)).tolist(),
            tcp_world=transform(tcp).tolist(), tool=actuator.observation()))

    def advance():
        nonlocal sampling_elapsed
        try:
            step()
        except Exception as error:
            observer.advance()
            current['failure_contacts']=observer.snapshot
            current['physics_failure']=str(error)
            raise
        observer.advance()
        forces = [0.,0.,0.] if observer.snapshot is not None else None
        for contact in observer.snapshot or ():
            if contact['lost']:
                continue
            actors = (contact['actor0'],contact['actor1'])
            if roll_path in actors:
                other = actors[1] if actors[0] == roll_path else actors[0]
                if other in finger_bodies:
                    forces[finger_bodies.index(other)] += sum(
                        float(np.linalg.norm(point['impulse']))*120 for point in contact['points'])
                elif other in robot_bodies:
                    raise RuntimeError(f'Non-finger Robot/roll contact: {other}')
            if (any(a in robot_bodies for a in actors)
                    and any(a.startswith(context['root_path']+'/') and a != roll_path for a in actors)):
                raise RuntimeError(f'Robot/fixture collision: {actors}')
        actuator.update(1/120, forces)
        sampling_elapsed += 1/120
        if sampling_elapsed >= .1:
            sampling_elapsed = 0.
            current['samples'].append(dict(simulation_time_s=world.current_time,
                roll_position_m=roll.get_world_pose()[0].tolist(), tool=actuator.observation()))
        if require_held and actuator.holding_observation().state is not HoldingState.HELD:
            raise RuntimeError('Physical contact Holding lost during extract/transfer')
        position = roll.get_world_pose()[0]
        if not np.all(np.isfinite(position)) or position[2] < .1:
            raise RuntimeError('Roll fell or produced invalid state')

    def dwell(seconds):
        for _ in range(math.ceil(seconds*120)):
            advance()

    def move(name):
        start = np.array(robot.get_joint_positions(joint_indices=arms))
        destination = np.array(config['teaching'][name])
        duration = config['move_duration_s']
        for joint,delta in zip(arm_names,np.abs(destination-start)):
            velocity = float(geometry.urdf.find(f"joint[@name='{joint}']/limit").get('velocity'))
            if 1.5*delta/duration > velocity:
                raise ValueError('Teaching segment exceeds selected joint velocity')
        frames = math.ceil(duration*120)
        for frame in range(1,frames+1):
            t=frame/frames
            robot.apply_action(ArticulationAction(joint_positions=start+(destination-start)*(3*t*t-2*t*t*t),joint_indices=arms))
            advance()
        dwell(.3)
        actual=np.array(robot.get_joint_positions(joint_indices=arms))
        if np.max(np.abs(actual-destination)) > .003:
            raise RuntimeError(f'Taught motion failed to settle: {name}')
        record(name.upper())

    def await_state(intent, expected):
        tool.command(intent)
        for _ in range(480):
            advance()
            if actuator.holding_observation().state is expected:
                return
        record('TOOL_TIMEOUT')
        raise RuntimeError(f'Physical {intent.value} did not yield {expected.value}: {actuator.observation()}')

    def callback(event,name):
        nonlocal relation, require_held
        if event == 0:
            if name == 'transfer':
                require_held=True
                move('extract')
                move('transfer_clearance')
                move('transfer')
                displacement=np.linalg.norm(roll.get_world_pose()[0]-np.array(config['supply_center_m']))
                current['transferred_distance_m']=float(displacement)
                if displacement < .1:
                    raise RuntimeError('Physical roll did not transfer')
                expected=np.array(config['receiver_center_m'])
                if np.linalg.norm(roll.get_world_pose()[0]-expected) > .01:
                    raise RuntimeError('Roll missed receiver before release')
            else:
                move(name)
            if name == 'retreat':
                dwell(1.)
                error=float(np.linalg.norm(roll.get_world_pose()[0]-np.array(config['receiver_center_m'])))
                current['receiver_position_error_m']=error
                current['receiver_speed_m_s']=float(np.linalg.norm(roll.get_linear_velocity()))
                current['receiver_angular_speed_rad_s']=float(np.linalg.norm(roll.get_angular_velocity()))
                axis=transform(stage.GetPrimAtPath(roll_path))[:3,2]
                current['receiver_axis_error_rad']=float(math.acos(np.clip(abs(axis[0]),-1,1)))
                if (error > .01 or current['receiver_speed_m_s'] > .02
                        or current['receiver_angular_speed_rad_s'] > .2
                        or current['receiver_axis_error_rad'] > .05):
                    raise RuntimeError('Released roll is not resting in receiver')
        elif event == 1:
            await_state(ToolIntent.GRASP,HoldingState.HELD)
            record('EXPAND/GRASP')
        elif event == 2:
            require_held=False
            await_state(ToolIntent.RELEASE,HoldingState.RELEASED)
            record('CONTRACT/RELEASE')
        elif event == 3:
            # The synchronous fixture still observes a fresh physics step.
            advance()
            return {HoldingState.HELD:1,HoldingState.RELEASED:2}.get(actuator.holding_observation().state,0)
        elif event == 4:
            if actuator.holding_observation().state is not HoldingState.HELD:
                return 9
            relation=np.linalg.inv(transform(tcp))@transform(stage.GetPrimAtPath(roll_path))
            current['relation_committed']=True
            current['grasp_relation']=relation.tolist()
        elif event == 5:
            if actuator.holding_observation().state is not HoldingState.RELEASED:
                return 9
            relation=None
            current['relation_cleared_after_release']=True
        elif event == 6:
            require_held=False
            tool.command(ToolIntent.STOP)
            record('STOP')
        else:
            raise ValueError('Unknown common Task fixture event')
        return 0

    try:
        # Set initial fingers once through the physical drive, before approach.
        await_state(ToolIntent.RELEASE,HoldingState.RELEASED)
        tool.command(ToolIntent.GRASP)
        dwell(.2)
        tool.command(ToolIntent.STOP)
        stop_target=actuator.target
        dwell(.3)
        current['stop_hold_error_m']=float(max(abs(q-stop_target) for q in measured()))
        if current['stop_hold_error_m'] > config['physical']['tolerance_m']:
            raise RuntimeError('STOP did not hold measured prismatic position')
        if actuator.holding_observation().state is not HoldingState.UNKNOWN:
            raise RuntimeError('STOP synthesized Holding/Release evidence')
        record('STOP_CHECK')
        await_state(ToolIntent.RELEASE,HoldingState.RELEASED)
        dwell(.5)
        current['task_execution']=run_task_grasp_cycle(task_execution_library,tool.capture.name,
            tool.capture.solution,callback)
        if relation is not None:
            raise RuntimeError('Grasp Relation remains after confirmed release')
        current['final_roll_pose']=transform(stage.GetPrimAtPath(roll_path)).tolist()
    finally:
        require_held=False
        tool.command(ToolIntent.STOP)
        record('FINAL')
        observer.close()
