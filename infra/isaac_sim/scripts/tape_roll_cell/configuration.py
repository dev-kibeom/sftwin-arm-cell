"""Compile Cell geometry and compact frame teaching for configured-source tasks."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import xacro
from arm_cell_bringup.runtime import read, resolve, project_root
from arm_cell_bringup.model_validation import validate_assembly_configuration
from tape_roll_cell.material import TapeRoll
from tape_roll_cell.supply import TapeSupply
from tape_roll_cell.teaching_program import default_program, validate_program, resolve_program, apply_layout
from gripper_runtime.radial_geometry import ToolGeometry


def compile_cell(*, configuration):
    config=configuration;cell=config['cell'];layout=cell['tape_roll']
    layout['place'].update(layout['place'].get('tool_overrides',{}).get(config['tool']['tool_id'],{}))
    if isinstance(layout['material'],str):
        layout['material']=read(resolve(layout['material'],project_root()))
    roll=TapeRoll(**layout['material']);count=layout['cartridge']['slot_count'];TapeSupply(count)
    spacing=layout['cartridge']['slot_spacing_m'];axis=np.array(layout['cartridge']['axis'],dtype=float)
    if axis.shape!=(3,) or not np.allclose(axis,[1,0,0]) or not np.isfinite(spacing) or spacing<roll.width_m+.012:
        raise ValueError('Horizontal cartridge requires separated axial +X slots')
    if cell['world_to_base']['z'] != cell['environment']['robot_support_top_z_m']:
        raise ValueError('Robot support top and base frame must agree')
    model=config['model_configuration']
    urdf=ET.fromstring(xacro.process_file(model['urdf'],mappings={'mesh_root':model['mesh_root']}).toxml())
    validate_assembly_configuration(config)
    geometry=ToolGeometry(urdf,config['tool']['runtime'])
    solution=geometry.solve_bore(layout['insertion_depth_m'],roll.width_m,roll.inner_radius_m)
    layout['slots']=[dict(index=i,slot_id=f'slot_{i}') for i in range(count)]
    layout['conveyor_presentation']=dict(cell['conveyor'])
    teaching_file=Path(cell.get('teaching_file',project_root()/'config'/'teaching'/f"{model['model_id']}.json"))
    cell['teaching_file']=str(teaching_file)
    program=validate_program(json.loads(teaching_file.read_text())) if teaching_file.exists() else default_program(config)
    apply_layout(config,program)
    motions=resolve_program(program,cell['world_to_base'])
    motions['home']=dict(target='home',velocity_scale=.1,acceleration_scale=.1)
    registry={slot['slot_id']:dict(supply_id=cell['configured_supply']['supply_id'],material_id=cell['configured_supply']['material_id'],
        grasp=dict(strategy='internal_radial',insertion_depth_m=layout['insertion_depth_m'],thickness_m=roll.width_m,
                   inner_radius_m=roll.inner_radius_m,release_position_m=solution['release_position_m'],grasp_position_m=solution['grasp_position_m']),
        approach_motion='tape_approach',insertion_motion='tape_insert',transfer_motion='tape_transfer',retreat_motion='tape_retreat') for slot in layout['slots']}
    cell['motion_preparation']=dict(ready=True,detail='Teaching loaded; MoveJ/MoveL follow taught movements without Scene collision checks.')
    config['motion']['configured_grasp_cycles_json']=json.dumps(registry,allow_nan=False)
    config['motion']['taught_motions_json']=json.dumps(motions,allow_nan=False)
    static=dict(schema_version=1,frame_id='world',objects=layout['static_boxes'])
    encoded=json.dumps(static,sort_keys=True)
    directory=project_root()/'.run_records/configuration';directory.mkdir(parents=True,exist_ok=True)
    path=directory/('tape-static-'+hashlib.sha256(encoded.encode()).hexdigest()+'.json')
    path.write_text(encoded+'\n');cell['static_scene']=str(path)
