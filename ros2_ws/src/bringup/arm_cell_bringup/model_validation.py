"""Check selected owner/model consistency at source configuration composition.

This does not run on exported snapshots or inspect a live Isaac stage. Xacro is
already the model producer used by ROS launch; no second model format is parsed.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def names(value, label, allowed=None):
    if (not isinstance(value, list) or
            any(not isinstance(name, str) or not name for name in value) or
            len(value) != len(set(value))):
        raise ValueError(f'{label} requires unique names')
    if allowed is not None and not set(value) <= set(allowed):
        raise ValueError(f'{label} contains unknown names')
    return value


def joint_pose(value, label, joint_names, joints, overrides):
    if (not isinstance(value, list) or len(value) != len(joint_names) or
            not all(finite_number(item) for item in value)):
        raise ValueError(f'{label} requires one finite position per arm joint')
    for name, position in zip(joint_names, value):
        joint = joints[name]
        if joint.get('type') != 'continuous':
            limit = joint.find('limit')
            lower, upper = float(limit.get('lower')), float(limit.get('upper'))
            if not lower <= position <= upper:
                raise ValueError(f'{label} exceeds URDF position limits for {name}')
        override = overrides.get(name, {})
        if override.get('has_position_limits', False) and not (
                override['min_position'] <= position <= override['max_position']):
            raise ValueError(f'{label} exceeds Robot joint_limits for {name}')


def planning_limits(path, joint_names, joints):
    limits = yaml.safe_load(Path(path).read_text())['joint_limits']
    if not isinstance(limits, dict) or not set(limits) <= set(joint_names):
        raise ValueError('Robot joint_limits contains unknown arm joints')
    for name, override in limits.items():
        physical = joints[name].find('limit')
        for kind in ('velocity', 'acceleration'):
            if override.get(f'has_{kind}_limits', False):
                maximum = override.get(f'max_{kind}')
                if not finite_number(maximum) or maximum <= 0:
                    raise ValueError(f'Robot joint_limits.{name} requires positive {kind}')
                if kind == 'velocity' and maximum > float(physical.get('velocity')):
                    raise ValueError(f'Robot joint_limits.{name} exceeds URDF velocity')
        if override.get('has_position_limits', False):
            lower, upper = override.get('min_position'), override.get('max_position')
            if not finite_number(lower) or not finite_number(upper) or lower > upper:
                raise ValueError(f'Robot joint_limits.{name} has invalid positions')
            if joints[name].get('type') != 'continuous' and (
                    lower < float(physical.get('lower')) or upper > float(physical.get('upper'))):
                raise ValueError(f'Robot joint_limits.{name} exceeds URDF positions')
    return limits


def prim_path(value, label, absolute):
    if (not isinstance(value, str) or not value or
            value.startswith('/') != absolute or
            any(not part.isidentifier() for part in (value[1:] if absolute else value).split('/'))):
        raise ValueError(f'{label} requires a {"stage-absolute" if absolute else "model-relative"} prim path')
    return value


def validate_model_configuration(config):
    """Validate the selected assembly and its Robot/Tool/Cell references.

    Existing runtime parameter projections are deliberately left untouched.
    Isaac prim bindings are checked as references, not as proof of USD topology.
    """
    import xacro

    robot, tool, cell = (config[key] for key in ('robot', 'tool', 'cell'))
    model = config['model_configuration']
    urdf = ET.fromstring(xacro.process_file(model['urdf'], mappings={
        'mesh_root': model['mesh_root'],
    }).toxml())
    semantic = ET.parse(model['semantic']).getroot()
    if not model.get('model_id') or any(
            root.get('name') != model['model_id'] for root in (urdf, semantic)):
        raise ValueError('Assembly model identity must match URDF and SRDF')
    links = {link.get('name') for link in urdf.findall('link')}
    if not {'base_link', 'sf_grasp_tcp'} <= links:
        raise ValueError('Assembly requires canonical base_link and sf_grasp_tcp links')
    joints = {joint.get('name'): joint for joint in urdf.findall('joint')}
    group = next((group for group in semantic.findall('group')
                  if group.get('name') == model.get('planning_group')), None)
    if group is None:
        raise ValueError('Assembly planning group is absent from SRDF')
    if len(group) != 1 or group[0].tag != 'chain':
        raise ValueError('Assembly planning group must contain a single canonical chain')
    chain = group.find('chain')
    if (chain is None or chain.get('base_link') != 'base_link' or
            chain.get('tip_link') != 'sf_grasp_tcp'):
        raise ValueError('Assembly planning group requires the canonical base-to-TCP chain')
    by_child = {joint.find('child').get('link'): joint for joint in joints.values()}
    chain_names = []
    cursor = 'sf_grasp_tcp'
    visited = set()
    while cursor != 'base_link':
        if cursor in visited or cursor not in by_child:
            raise ValueError('Assembly planning group chain does not reach base_link')
        visited.add(cursor)
        joint = by_child[cursor]
        if joint.get('type') != 'fixed' and joint.find('mimic') is None:
            if joint.get('type') not in ('revolute', 'continuous', 'prismatic'):
                raise ValueError('Assembly planning group requires scalar joints')
            chain_names.append(joint.get('name'))
        cursor = joint.find('parent').get('link')
    arm = names(robot['motion']['arm_joint_names'], 'Robot arm_joint_names')
    if not arm or arm != chain_names[::-1]:
        raise ValueError('Robot arm_joint_names must match ordered planning group joints')
    for name in arm:
        joint = joints[name]
        limit = joint.find('limit')
        if limit is None or not finite_number(float(limit.get('velocity', 'nan'))) or float(limit.get('velocity')) <= 0:
            raise ValueError(f'Robot joint_limits requires a positive URDF velocity for {name}')
        if joint.get('type') != 'continuous':
            bounds = [float(limit.get(key, 'nan')) for key in ('lower', 'upper')]
            if not all(finite_number(value) for value in bounds) or bounds[0] > bounds[1]:
                raise ValueError(f'Robot joint_limits has invalid URDF bounds for {name}')
    scales = robot.get('isaac', {}).get('joint_drive_stiffness_scales', {})
    if (not isinstance(scales, dict) or not set(scales) <= set(arm)
            or any(not finite_number(value) or value <= 0 for value in scales.values())):
        raise ValueError('Robot Isaac joint drive stiffness scales require arm joint names and finite positive values')
    overrides = planning_limits(robot['joint_limits'], arm, joints)
    joint_pose(config['home_joint_positions'], 'Robot HOME', arm, joints, overrides)
    for key in ('retract_joint_positions', 'place_tray_return_joint_positions'):
        joint_pose(robot['motion'][key], f'Robot {key}', arm, joints, overrides)
    for key in ('kinematics', 'planning_pipeline'):
        planning = yaml.safe_load(Path(config['motion_configuration'][key]).read_text())
        if model['planning_group'] not in planning:
            raise ValueError(f'Assembly planning group is absent from {key}')
    for key in ('wrap_equivalent_joints', 'place_ik_seed_joints'):
        names(robot['joint_policy'][key], f'Robot joint_policy.{key}', arm)
    names(tool['grasp_pose_collision_links'], 'Tool grasp_pose_collision_links', links)
    for key in ('payload_touch_links', 'pick_retract'):
        names(tool['contact_links'][key], f'Tool contact_links.{key}', links)
    names(tool['validation_motion']['validation_grasp_contact_links'],
          'Tool validation_grasp_contact_links', links)
    scene = tool['validation_planning_scene']
    if scene['attachment_link'] not in links:
        raise ValueError('Tool attachment_link is absent from assembly')
    names(scene['touch_links'], 'Tool validation touch_links', links)
    geometry = yaml.safe_load(Path(cell['geometry']).read_text())
    static_scene = yaml.safe_load(Path(cell['static_scene']).read_text())
    contact = cell['contact_objects']['pick_retract']
    if any(contact not in {obj['id'] for obj in source['objects']}
           for source in (geometry, static_scene)):
        raise ValueError('Cell pick_retract contact object must exist in geometry and static_scene')
    for label, reference in (
            ('calibration_file', cell['calibration_file']),
            ('validation static_scene_file', cell['validation_planning_scene']['static_scene_file'])):
        if not Path(reference).is_file():
            raise ValueError(f'Cell {label} does not exist: {reference}')
    for location in cell['work_locations'].values():
        if location.get('frame') != 'base_link':
            raise ValueError('Cell work locations must use canonical base_link')
    prim_path(cell['isaac']['robot_prim_path'], 'Cell robot_prim_path', True)
    prim_path(model['isaac']['articulation_root'], 'Assembly articulation_root', False)
    base = prim_path(model['isaac']['articulation_base'], 'Assembly articulation_base', False)
    if str(base) not in links:
        raise ValueError('Assembly articulation_base must name a URDF link')
