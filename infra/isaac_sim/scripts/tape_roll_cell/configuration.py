"""Cell configuration compiler: geometry and taught paths for configured-source ROS tasks."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import xacro
from arm_cell_bringup.runtime import read, resolve, project_root
from arm_cell_bringup.model_validation import validate_assembly_configuration, joint_pose, planning_limits
from tape_roll_cell.material import TapeRoll
from tape_roll_cell.supply import TapeSupply
from tape_roll_cell.teaching import TeachingSolver, TeachingUnavailable, vertical_place_teaching
from tape_roll_cell.fixtures import static_fixtures
from gripper_runtime.radial_geometry import ToolGeometry


def _timed_path(paths, duration, velocity_limits, acceleration_limits):
    """Preserve the taught path; extend each segment until physical limits fit."""
    output=[]
    velocity=np.asarray(velocity_limits)*.3
    acceleration=np.asarray(acceleration_limits)*.2
    for path in paths:
        knots=np.array(path)
        segment_duration=duration
        for attempt in range(12):
            frames=max(1,int(np.ceil(segment_duration/.02)))
            samples=[]
            for frame in range(frames+1):
                t=frame/frames
                coordinate=(3*t*t-2*t*t*t)*(len(knots)-1)
                index=min(int(coordinate),len(knots)-2)
                samples.append(knots[index]+(knots[index+1]-knots[index])*(coordinate-index))
            speeds=np.diff(samples,axis=0)/.02
            accelerations=np.diff(np.vstack([np.zeros(len(velocity)),speeds,np.zeros(len(velocity))]),axis=0)/.02
            velocity_ratio=float(np.max(np.abs(speeds)/velocity))
            acceleration_ratio=float(np.max(np.abs(accelerations)/acceleration))
            if max(velocity_ratio,acceleration_ratio)<=.95:break
            # Velocity scales with 1/T and acceleration with 1/T². Recheck the
            # actual discrete samples after each extension, including corners.
            segment_duration=frames*.02*max(1.05,velocity_ratio*1.05,
                                            np.sqrt(acceleration_ratio)*1.05)
        else:raise TeachingUnavailable('Cell trajectory cannot satisfy selected Robot timing limits')
        # Retain one stationary sample between segments so the endpoint
        # acceleration limits also hold when two directions are concatenated.
        output.extend(np.asarray(samples).tolist())
    return dict(joint_path=output,duration_s=(len(output)-1)*.02,velocity_scale=.3,acceleration_scale=.2)


def compile_cell(*, configuration):
    config=configuration
    layout=config['cell']['tape_roll']
    layout['place'].update(layout['place'].get('tool_overrides',{}).get(config['tool']['tool_id'],{}))
    if isinstance(layout['material'],str):
        layout['material']=read(resolve(layout['material'],project_root()))
    roll=TapeRoll(**layout['material'])
    count=layout['cartridge']['slot_count']
    TapeSupply(count)
    spacing=layout['cartridge']['slot_spacing_m']
    axis=np.array(layout['cartridge']['axis'],dtype=float)
    if axis.shape!=(3,) or not np.allclose(axis,[1,0,0]) or not np.isfinite(spacing) or spacing<roll.width_m+.012:
        raise ValueError('Horizontal cartridge requires separated axial +X slots')
    duration=layout['move_duration_s']
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError('Cell movement duration must be positive')
    reference=layout['teaching'][config['tool']['tool_id']]
    model=config['model_configuration']
    urdf=ET.fromstring(xacro.process_file(model['urdf'],mappings={'mesh_root':model['mesh_root']}).toxml())
    _,joints,arm=validate_assembly_configuration(config)
    overrides=planning_limits(config['robot']['joint_limits'],arm,joints)
    velocities=[min(float(joints[name].find('limit').get('velocity')),
                    overrides[name]['max_velocity'] if overrides.get(name,{}).get('has_velocity_limits') else float('inf')) for name in arm]
    accelerations=[min(config['motion']['planning_max_acceleration_rad_s2'],
                       overrides[name]['max_acceleration'] if overrides.get(name,{}).get('has_acceleration_limits') else float('inf')) for name in arm]
    solver=TeachingSolver(urdf,config['robot']['motion']['arm_joint_names'])
    geometry=ToolGeometry(urdf,config['tool']['runtime'])
    solution=geometry.solve_bore(layout['insertion_depth_m'],roll.width_m,roll.inner_radius_m)
    registry={};motions=dict(home=dict(target='home',velocity_scale=.3,acceleration_scale=.2))
    layout['slots']=[]
    place = layout['place']
    if place['transport_height_m'] <= max(layout['supply_center_m'][2]+roll.outer_radius_m,
                                         place['surface_z_m']+roll.width_m)+.08:
        raise ValueError('Transport height must clear stored and placed rolls')
    if config['cell']['world_to_base']['z'] != config['cell']['environment']['robot_support_top_z_m']:
        raise ValueError('Robot support top and base frame must agree')
    scene_slots = [dict(index=i,slot_id=f'slot_{i}',
        supply_center_m=list(layout['supply_center_m']),
        receiver_center_m=[place['first_center_xy_m'][0],place['first_center_xy_m'][1],place['surface_z_m']+roll.width_m/2])
        for i in range(count)]
    config['cell']['motion_preparation'] = dict(ready=False,detail='')
    try:
        for index in range(count):
            supply_center = list(layout['supply_center_m'])
            receiver_center = [place['first_center_xy_m'][0],place['first_center_xy_m'][1],place['surface_z_m']+roll.width_m/2]
            previous = layout['slots'][-1]['teaching'] if index else None
            if index:
                taught = layout['slots'][0]['teaching']
                paths = dict(layout['slots'][0]['trajectories'])
                # Retrace the already solved empty-tool corridor after Retreat.
                # Every indexed roll arrives at the same mechanical datum.
                if index == 1:
                    return_approach = solver.path(taught['carry_lift'],taught['approach'])
                paths['approach'] = return_approach
                for key,segment in [('return_place','transfer_clearance'),('return_turn','turn'),('return_clearance','turn_clearance'),('return_forward','carry_forward')]:
                    paths[key] = list(reversed(paths[segment]))
            else:
                taught,paths = vertical_place_teaching(solver,reference,supply_center,receiver_center,layout,previous)
            for name,knots in paths.items():
                for pose in knots:joint_pose(pose,name,arm,joints,overrides)
            slot=dict(index=index,slot_id=f'slot_{index}',supply_center_m=supply_center,
                      receiver_center_m=receiver_center,teaching=taught,trajectories=paths)
            layout['slots'].append(slot)
            groups=dict(approach=['approach'] if not index else ['return_place','return_turn','return_clearance','return_forward','approach'],
                        insert=['insert'],transfer=['lift','extract','carry_lift','carry_forward','turn_clearance','turn','transfer_clearance','transfer'],retreat=['retreat'])
            refs={}
            for name,segments in groups.items():
                key=f'tape_slot_{index}_{name}'
                motions[key]=_timed_path([paths[s] for s in segments],duration,velocities,accelerations)
                if not index and name=='approach':motions[key]['plan_to_start']=True
                refs[name]=key
            registry[slot['slot_id']]=dict(supply_id=config['cell']['configured_supply']['supply_id'],material_id=config['cell']['configured_supply']['material_id'],
                grasp=dict(strategy='internal_radial',insertion_depth_m=layout['insertion_depth_m'],thickness_m=roll.width_m,
                           inner_radius_m=roll.inner_radius_m,release_position_m=solution['release_position_m'],grasp_position_m=solution['grasp_position_m']),
                approach_motion=refs['approach'],insertion_motion=refs['insert'],transfer_motion=refs['transfer'],retreat_motion=refs['retreat'])
    except TeachingUnavailable as error:
        # Scene construction does not require a robot trajectory. Keep every
        # material and fixture, but publish no partial executable operation.
        layout['slots'] = scene_slots
        registry = {}
        motions = {}
        config['cell']['motion_preparation']['detail'] = f'slot_{index}: {error}'
    else:
        config['cell']['motion_preparation']['ready'] = True
    config['motion']['configured_grasp_cycles_json']=json.dumps(registry,allow_nan=False)
    config['motion']['taught_motions_json']=json.dumps(motions,allow_nan=False)
    layout['conveyor_presentation'] = dict(config['cell']['conveyor'])
    boxes = static_fixtures(layout,roll,config['cell']['environment'],config['cell']['conveyor'])
    layout['static_boxes'] = boxes
    static=dict(schema_version=1,frame_id='world',objects=boxes)
    encoded=json.dumps(static,sort_keys=True)
    directory=project_root()/'.run_records/configuration';directory.mkdir(parents=True,exist_ok=True)
    path=directory/('tape-static-'+hashlib.sha256(encoded.encode()).hexdigest()+'.json')
    path.write_text(encoded+'\n')
    config['cell']['static_scene']=str(path)
