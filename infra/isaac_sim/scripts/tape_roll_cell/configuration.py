"""Export Cell-owned scene/teaching for explicitly supported physical fixture execution."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import xacro
from arm_cell_bringup.runtime import read, resolve
from arm_cell_bringup.model_experiment import load_model_experiment
from arm_cell_bringup.model_validation import joint_pose, validate_assembly_configuration, planning_limits
from tape_roll_cell.material import TapeRoll
from tape_roll_cell.supply import TapeSupply
from tape_roll_cell.teaching import TeachingSolver, translated_teaching, trajectories


def compose(profile, experiment_path=None):
    profile = Path(profile).resolve()
    source = read(profile)
    slot_count = None
    drive_reference = source.get('tool_drive')
    drive_base = profile.parent
    if source.get('kind') == 'arm-cell-physical-fixture':
        if source.get('schema_version') != 1 or source.get('execution') != 'native_task_bt_physical_fixture':
            raise ValueError('Unsupported Cell execution scope')
        model_profile = resolve(source['model_experiment'], profile.parent)
        cell_path = resolve(source['cell'], profile.parent)
        scope = 'cell_physical_fixture'
    else:
        model_profile = profile
        experiment = read(experiment_path or Path(__file__).with_name('experiment.yaml'))
        cell_path = resolve(experiment['cell'], Path(experiment_path or __file__).parent)
        slot_count = experiment['slot_count']
        drive_reference = experiment['tool_drive']
        drive_base = Path(experiment_path or __file__).parent
        scope = 'model_only'
    config = load_model_experiment(model_profile)
    if drive_reference is not None:
        drive = read(resolve(drive_reference, drive_base))
        if set(drive) != {'drive_stiffness', 'drive_damping', 'max_velocity_fraction'} or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not np.isfinite(v) or v <= 0
                for v in drive.values()):
            raise ValueError('Tool physical drive requires finite positive parameters')
        if drive['max_velocity_fraction'] > 1:
            raise ValueError('Tool fixture velocity fraction cannot exceed the URDF limit')
        config['tool']['runtime'].update(drive)
    cell = read(cell_path)
    if not cell['execution_support'].get('native_task_bt_physical_fixture'):
        raise ValueError('Cell does not support native physical fixture execution')
    if config['model_configuration']['model_id'] not in cell['execution_support']['models']:
        raise ValueError('Selected assembly is unsupported by Tape Cell teaching')
    layout = cell['tape_roll']
    layout['material'] = read(resolve(layout['material'], cell_path.parent))
    roll = TapeRoll(**layout['material'])
    cartridge = layout['cartridge']
    if slot_count is not None:
        cartridge['slot_count'] = slot_count
    TapeSupply(cartridge['slot_count'])
    spacing = cartridge['slot_spacing_m']
    axis = np.array(cartridge['axis'], dtype=float)
    if axis.shape != (3,) or not np.allclose(axis, [1, 0, 0]):
        raise ValueError('This horizontal cartridge requires axial +X slots')
    if (isinstance(spacing, bool) or not isinstance(spacing, (int, float))
            or not np.isfinite(spacing) or spacing < roll.width_m + .012):
        raise ValueError('Cartridge slots must leave clearance between roll cradles')
    for field in ('move_duration_s', 'release_clearance_m', 'extract_lift_m', 'insertion_depth_m'):
        value = layout[field]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
            raise ValueError(f'Cell {field} must be finite and positive')
    for field in ('supply_center_m', 'receiver_center_m'):
        center = layout[field]
        if (not isinstance(center, list) or len(center) != 3 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not np.isfinite(v) for v in center)):
            raise ValueError(f'Cell {field} requires three finite coordinates')
    tool_id = config['tool']['tool_id']
    reference = layout.pop('teaching')[tool_id]
    model = config['model_configuration']
    urdf = ET.fromstring(xacro.process_file(model['urdf'], mappings={'mesh_root':model['mesh_root']}).toxml())
    _, joints, arm = validate_assembly_configuration(config)
    overrides = planning_limits(config['robot']['joint_limits'], arm, joints)
    layout['slots'] = []
    solver = TeachingSolver(urdf, config['robot']['motion']['arm_joint_names'])
    for index in range(cartridge['slot_count']):
        offset = index * spacing * axis
        receiver_offset = (cartridge['slot_count']-1-index) * spacing * axis
        teaching = translated_teaching(solver, reference, offset, receiver_offset, layout['extract_lift_m'])
        paths = trajectories(solver, teaching, layout['slots'][-1]['teaching'] if index else None)
        for name, knots in paths.items():
            for pose in knots:
                joint_pose(pose, name, arm, joints, overrides)
        layout['slots'].append(dict(index=index,
            supply_center_m=(np.array(layout['supply_center_m'])+offset).tolist(),
            receiver_center_m=(np.array(layout['receiver_center_m'])+receiver_offset).tolist(),
            teaching=teaching, trajectories=paths))
    layout['teaching'] = layout['slots'][0]['teaching']
    config['execution_scope'] = scope
    config['experiment_drive_stiffness_scale'] = 100.0
    config['cell'] = cell
    config['tool']['runtime']['implementation'] = 'hand_e_c10_physical'
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--experiment')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    Path(args.output).write_text(json.dumps(compose(args.profile,args.experiment),indent=2,allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
