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
from tape_roll_cell.teaching import TeachingSolver, translated_teaching, trajectories
from gripper_runtime.radial_geometry import ToolGeometry


def _timed_path(paths, duration):
    """Uniform-time samples, one smooth scalar per Cartesian segment."""
    output=[]
    frames=max(1,int(np.ceil(duration/.02)))
    for path in paths:
        knots=np.array(path)
        for frame in range(frames+1):
            if output and frame==0:continue
            t=frame/frames
            coordinate=(3*t*t-2*t*t*t)*(len(knots)-1)
            index=min(int(coordinate),len(knots)-2)
            output.append((knots[index]+(knots[index+1]-knots[index])*(coordinate-index)).tolist())
    return dict(joint_path=output,duration_s=duration*len(paths),velocity_scale=.3,acceleration_scale=.2)


def compile_cell(*, configuration):
    config=configuration
    layout=config['cell']['tape_roll']
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
    solver=TeachingSolver(urdf,config['robot']['motion']['arm_joint_names'])
    geometry=ToolGeometry(urdf,config['tool']['runtime'])
    solution=geometry.solve_bore(layout['insertion_depth_m'],roll.width_m,roll.inner_radius_m)
    registry={};motions=dict(home=dict(target='home',velocity_scale=.3,acceleration_scale=.2))
    layout['slots']=[]
    for index in range(count):
        offset=index*spacing*axis
        receiver_offset=(count-1-index)*spacing*axis
        taught=translated_teaching(solver,reference,offset,receiver_offset,layout['extract_lift_m'])
        paths=trajectories(solver,taught,layout['slots'][-1]['teaching'] if index else None)
        if not index:
            # Production starts at Robot HOME. This entry path is collision checked
            # by Motion against the current planning scene before any command.
            target=solver.fk(taught['approach']).copy();target[2,3]=.72
            entry=solver.solve(target,taught['approach'])
            paths['approach']=solver.path(entry,taught['approach'])
        for name,knots in paths.items():
            for pose in knots:joint_pose(pose,name,arm,joints,overrides)
        slot=dict(index=index,slot_id=f'slot_{index}',supply_center_m=(np.array(layout['supply_center_m'])+offset).tolist(),
                  receiver_center_m=(np.array(layout['receiver_center_m'])+receiver_offset).tolist(),teaching=taught,trajectories=paths)
        layout['slots'].append(slot)
        groups=dict(approach=['approach'] if not index else ['return_clearance','supply_clearance','approach'],
                    insert=['insert'],transfer=['lift','extract','transfer_clearance','transfer'],retreat=['retreat'])
        refs={}
        for name,segments in groups.items():
            key=f'tape_slot_{index}_{name}'
            motions[key]=_timed_path([paths[s] for s in segments],duration)
            if not index and name=='approach':motions[key]['plan_to_start']=True
            refs[name]=key
        registry[slot['slot_id']]=dict(supply_id=config['cell']['configured_supply']['supply_id'],material_id=config['cell']['configured_supply']['material_id'],
            grasp=dict(strategy='internal_radial',insertion_depth_m=layout['insertion_depth_m'],thickness_m=roll.width_m,
                       inner_radius_m=roll.inner_radius_m,release_position_m=solution['release_position_m'],grasp_position_m=solution['grasp_position_m']),
            approach_motion=refs['approach'],insertion_motion=refs['insert'],transfer_motion=refs['transfer'],retreat_motion=refs['retreat'])
    config['motion']['configured_grasp_cycles_json']=json.dumps(registry,allow_nan=False)
    config['motion']['taught_motions_json']=json.dumps(motions,allow_nan=False)
    boxes=[]
    for slot in layout['slots']:
        for label,key in [('Cartridge','supply_center_m'),('Receiver','receiver_center_m')]:
            x,y,z=slot[key];r=roll.outer_radius_m;w=roll.width_m
            prefix=f"/World/TapeRollCell/{label}{slot['index']:03d}"
            boxes.append(dict(id=prefix+'Floor',geometry=dict(type='box',dimensions_m=[w+.008,2*r+.024,.01]),pose=dict(position_m=[x,y,z-r-.005],quaternion_xyzw=[0.,0.,0.,1.])))
            for side,sign in [('left',-1),('right',1)]:
                boxes.append(dict(id=prefix+side.title()+'Guide',geometry=dict(type='box',dimensions_m=[w+.008,.01,.04]),pose=dict(position_m=[x,y+sign*(r+.009),z-r+.015],quaternion_xyzw=[0.,0.,0.,1.])))
    static=dict(schema_version=1,frame_id='world',objects=boxes)
    encoded=json.dumps(static,sort_keys=True)
    directory=project_root()/'.run_records/configuration';directory.mkdir(parents=True,exist_ok=True)
    path=directory/('tape-static-'+hashlib.sha256(encoded.encode()).hexdigest()+'.json')
    path.write_text(encoded+'\n')
    config['cell']['static_scene']=str(path)
