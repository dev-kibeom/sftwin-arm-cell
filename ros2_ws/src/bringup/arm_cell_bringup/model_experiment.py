"""Compose model-only experiments without Mission, Cell or grasp runtime inputs."""
import math
from pathlib import Path

from .runtime import read, resolve
from .model_validation import validate_assembly_configuration, finite_number, names


def load_model_experiment(path):
    path = Path(path)
    source = read(path)
    if source.get('kind') != 'arm-cell-model-experiment' or source.get('schema_version') != 1:
        raise ValueError('Unsupported model experiment')
    config = {'execution_scope': 'model_only'}
    for key in ('robot', 'tool', 'model_configuration', 'motion_configuration'):
        owner = resolve(source[key], path.parent)
        config[key] = read(owner)
        keys = {'robot': ('home', 'joint_limits'), 'tool': (),
                'model_configuration': ('urdf', 'mesh_root', 'semantic'),
                'motion_configuration': tuple(config[key])}[key]
        for field in keys:
            config[key][field] = str(resolve(config[key][field], owner.parent))
    robot, tool, model = (config[key] for key in ('robot', 'tool', 'model_configuration'))
    if any(model[f'{key}_id'] != config[key][f'{key}_id'] for key in ('robot', 'tool')):
        raise ValueError('Model experiment owner identity mismatch')
    config['home_joint_positions'] = read(robot['home'])['home_joint_positions']
    links, joints, arm = validate_assembly_configuration(config)
    runtime = tool['runtime']
    if runtime['supported_combinations'] or runtime['capabilities'] != ['joint_position']:
        raise ValueError('Model experiment cannot advertise grasp support')
    primary = runtime['primary_joint']
    if primary not in joints or primary in arm or joints[primary].find('mimic') is not None:
        raise ValueError('Experiment requires an independent Tool primary joint')
    selected = {primary: joints[primary]}
    selected.update({name: joint for name, joint in joints.items()
                     if joint.find('mimic') is not None and joint.find('mimic').get('joint') == primary})
    if set(selected) != set(model['isaac']['prismatic_joints']):
        raise ValueError('Experiment Tool joints must match Assembly prismatic bindings')
    linkage = {}
    for name, joint in selected.items():
        if joint.get('type') != 'prismatic' or name in arm:
            raise ValueError('Experiment requires prismatic Tool joints')
        limit = joint.find('limit')
        values = {key: float(limit.get(key)) for key in ('lower', 'upper', 'velocity', 'effort')}
        if not all(math.isfinite(v) for v in values.values()) or not (
                values['lower'] < values['upper'] and values['velocity'] > 0 and values['effort'] > 0):
            raise ValueError('Invalid experiment joint limits')
        mimic = joint.find('mimic')
        if mimic is not None:
            values['mimic'] = dict(joint=primary, multiplier=float(mimic.get('multiplier', '1')),
                                   offset=float(mimic.get('offset', '0')))
        linkage[name] = values
    for key in ('drive_stiffness', 'drive_damping', 'sweep_duration_s', 'position_tolerance_m'):
        if not finite_number(runtime[key]) or runtime[key] <= 0:
            raise ValueError(f'Experiment {key} must be finite and positive')
    runtime['joint_model'] = linkage
    tips = runtime.get('finger_tip_links', [joint.find('child').get('link') for joint in selected.values()])
    names(tips, 'Tool finger_tip_links', links)
    if len(tips) != len(selected):
        raise ValueError('Tool requires one observed tip per moving finger')
    by_child = {joint.find('child').get('link'): joint for joint in joints.values()}
    for tip, finger in zip(tips, selected.values()):
        cursor, visited = tip, set()
        while cursor != finger.find('child').get('link'):
            joint = by_child.get(cursor)
            if cursor in visited or joint is None or joint.get('type') != 'fixed':
                raise ValueError('Tool tip must be rigidly attached to its moving finger')
            visited.add(cursor)
            cursor = joint.find('parent').get('link')
    runtime['finger_tip_links'] = tips
    # Validate dependent endpoints before a simulator is launched.
    for position in (linkage[primary]['lower'], linkage[primary]['upper']):
        joint_targets(linkage, position)
    return config


def joint_targets(linkage, position):
    """SI metre targets; preserve the URDF mimic multiplier and offset."""
    if not finite_number(position):
        raise ValueError('Joint position must be finite metres')
    targets = {}
    for name, joint in linkage.items():
        mimic = joint.get('mimic', {})
        target = position * mimic.get('multiplier', 1.) + mimic.get('offset', 0.)
        if not math.isfinite(target) or not joint['lower'] <= target <= joint['upper']:
            raise ValueError(f'Joint position exceeds limits: {name}')
        targets[name] = target
    return targets


def main():
    import argparse
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    configuration = load_model_experiment(args.profile)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(configuration, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
