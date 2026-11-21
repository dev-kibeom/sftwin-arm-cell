"""Install the Conveyor's fixed Pick/Place teaching in the configured grasp catalog."""
import json
from pathlib import Path


def compile_cell(*, configuration):
    from arm_cell_bringup.runtime import read, resolve, project_root
    cell = configuration['cell']
    layout = cell['conveyor']
    profile = read(resolve(layout['plc_profile'], project_root()))['plc_task_adapter']['ros__parameters']
    layout['connection'] = {key: profile[key] for key in ('host', 'port', 'unit_id', 'poll_seconds', 'timeout_seconds')}
    layout['receipt_file'] = profile['receipt_file']
    layout['boxes'] = json.loads(Path(cell['static_scene']).read_text())['objects']
    teaching = layout['teaching']
    if 'motions' in teaching:
        motions = {name: dict(cartesian_waypoints=points, velocity_scale=.20, acceleration_scale=.20)
                   for name, points in teaching['motions'].items()}
        motions['home'] = dict(target='home', velocity_scale=.20, acceleration_scale=.20)
        configuration['motion']['taught_motions_json'] = json.dumps(motions, allow_nan=False)
    cycle = teaching['cycle']
    if cycle and any(cycle[key] != value for key, value in cell['configured_supply'].items()):
        raise ValueError('Fixed teaching must match the selected Conveyor supply and material')
    if cycle:
        motions = json.loads(configuration['motion'].get('taught_motions_json', '{}'))
        for key in ('approach_motion', 'insertion_motion', 'transfer_motion', 'retreat_motion'):
            if cycle.get(key) not in motions:
                raise ValueError(f'Conveyor {key} requires an installed taught motion')
        configuration['motion']['taught_cycle_motion_names'] = [
            cycle[key] for key in ('approach_motion', 'insertion_motion', 'transfer_motion', 'retreat_motion')]
    types = layout['material_type_ids']
    if (not isinstance(types, list) or not types or any(not isinstance(t, str) or not t.strip() for t in types)
            or len(set(types)) != len(types)):
        raise ValueError('Conveyor requires distinct material type IDs')
    pick, place = layout['pick_slot_id'], layout['place_slot_id']
    for slot in (pick, place):
        if len(slot.split('/')) != 3 or not all(part.strip() for part in slot.split('/')):
            raise ValueError('Expected cell/equipment/slot address')
    if pick == place or pick.split('/')[0] != place.split('/')[0]:
        raise ValueError('Pick and Place must be distinct slots in the same cell')
    catalog = {'fixed_pick': dict(cycle, pick_slot_id=pick, place_slot_id=place,
                                 material_type_ids=types)} if cycle else {}
    configuration['motion']['configured_grasp_cycles_json'] = json.dumps(catalog)
    configuration['orchestration']['plc_admission_enabled'] = True
