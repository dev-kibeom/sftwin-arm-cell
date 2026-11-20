"""Compile one fixed Pick teaching into the Task server's exact-key catalog."""
import json
from pathlib import Path


def fixed_pick_slots(teaching, *, first, count):
    if type(first) is not int or type(count) is not int:
        raise ValueError('Conveyor slot IDs and count must be integers')
    if not (1 <= first <= 0xffffffff and 0 <= count <= 65535 and first + count <= 2**32):
        raise ValueError('Invalid Conveyor deployment slot range')
    return {f'slot_{cycle}': teaching for cycle in range(first, first + count)} if teaching else {}


def compile_cell(*, configuration):
    from arm_cell_bringup.runtime import read, resolve, project_root
    cell = configuration['cell']
    layout = cell['conveyor']
    profile = read(resolve(layout['plc_profile'], project_root()))['plc_task_adapter']['ros__parameters']
    layout['connection'] = {key: profile[key] for key in ('host', 'port', 'unit_id', 'poll_seconds', 'timeout_seconds')}
    layout['boxes'] = json.loads(Path(cell['static_scene']).read_text())['objects']
    teaching = layout['teaching']
    cycle = teaching['cycle']
    if cycle and any(cycle[key] != value for key, value in cell['configured_supply'].items()):
        raise ValueError('Fixed teaching must match the selected Conveyor supply and material')
    if cycle:
        motions = json.loads(configuration['motion'].get('taught_motions_json', '{}'))
        for key in ('approach_motion', 'insertion_motion', 'transfer_motion', 'retreat_motion'):
            if cycle.get(key) not in motions:
                raise ValueError(f'Conveyor {key} requires an installed taught motion')
    catalog = fixed_pick_slots(cycle, first=teaching['first'], count=teaching['count'])
    configuration['motion']['configured_grasp_cycles_json'] = json.dumps(catalog)
    configuration['orchestration']['plc_admission_enabled'] = True
