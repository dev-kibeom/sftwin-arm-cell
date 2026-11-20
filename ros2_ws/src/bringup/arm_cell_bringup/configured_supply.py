"""Compose configured-source Missions using selected Cell and Tool owners."""
import copy
import importlib
import json
import sys
from .component_parameters import apply_component_parameters
from .runtime import read, resolve, selected_profiles, write_recipes, configure_grip_force, project_root
from .model_validation import validate_assembly_configuration, finite_number, names


def configured_tool_model(config):
    links, joints, arm = validate_assembly_configuration(config)
    for name in ('robot','tool'):
        if config['model_configuration'][name+'_id'] != config[name][name+'_id']:
            raise ValueError('Assembly owner identity mismatch')
    runtime = config['tool']['runtime']
    pair = [runtime['actuator'], runtime['grasp']]
    if pair not in runtime['supported_combinations']:
        raise ValueError(f'Unsupported Tool actuator/grasp combination: {pair}')
    runtime['mode_id'] = next(mode['id'] for mode in runtime['launcher_modes'] if mode['combination'] == pair)
    if config['tool']['tool_type'] == 'radial_gripper':
        _validate_radial_model(config, links, joints, arm)
    elif config['tool']['tool_type'] == 'suction_gripper':
        names(config['tool']['grasp_pose_collision_links'], 'Suction contact links', links)
        names([runtime['cup_link'], runtime['carrier_link']], 'Suction D6 body links', links)
        for key in ('max_grip_distance_m', 'coaxial_force_limit_n', 'shear_force_limit_n', 'retry_interval_s'):
            if not finite_number(runtime[key]) or runtime[key] <= 0:
                raise ValueError(f'Invalid suction Tool {key}')
        if runtime['grasp_strategies'] != ['surface_suction']:
            raise ValueError('Suction Tool requires its cup link and surface_suction strategy')
    else:
        raise ValueError('Unsupported configured Tool model')
    return config


def _validate_radial_model(config, links, joints, arm):
    runtime = config['tool']['runtime']
    primary = runtime['primary_joint']
    if primary not in joints or primary in arm or joints[primary].find('mimic') is not None:
        raise ValueError('Radial primary must be independent and outside the arm')
    selected = {name: joint for name, joint in joints.items() if name == primary or
                (joint.find('mimic') is not None and joint.find('mimic').get('joint') == primary)}
    if set(selected) != set(config['model_configuration']['isaac']['prismatic_joints']):
        raise ValueError('Selected Tool joint model and Assembly binding disagree')
    model = {}
    for name, joint in selected.items():
        if name in arm or joint.get('type') != 'prismatic':
            raise ValueError('Radial Tool requires independent prismatic fingers outside the arm')
        limit = joint.find('limit')
        value = {key:float(limit.get(key)) for key in ('lower','upper','velocity','effort')}
        if not all(finite_number(v) for v in value.values()) or not (value['lower'] < value['upper'] and value['velocity'] > 0 and value['effort'] > 0):
            raise ValueError('Invalid radial Tool limits')
        mimic = joint.find('mimic')
        if mimic is not None:
            value['mimic'] = dict(joint=primary,multiplier=float(mimic.get('multiplier','1')),offset=float(mimic.get('offset','0')))
        model[name] = value
    if joints[primary].find('mimic') is not None:
        raise ValueError('Radial primary must be independent')
    runtime['joint_model'] = model
    names(runtime['finger_tip_links'], 'Radial finger tips', links)
    if len(runtime['finger_tip_links']) != len(model):
        raise ValueError('One tip per independent physical finger is required')
    by_child = {joint.find('child').get('link'):joint for joint in joints.values()}
    for tip,finger in zip(runtime['finger_tip_links'], selected.values()):
        cursor,visited=tip,set()
        while cursor != finger.find('child').get('link'):
            joint=by_child.get(cursor)
            if cursor in visited or joint is None or joint.get('type') != 'fixed':
                raise ValueError('Tool tip must be rigidly attached to its moving finger')
            visited.add(cursor);cursor=joint.find('parent').get('link')
    for key in ('drive_stiffness','drive_damping','position_tolerance_m','max_velocity_fraction'):
        if not finite_number(runtime[key]) or runtime[key] <= 0:
            raise ValueError(f'Invalid radial Tool {key}')
    if runtime['max_velocity_fraction'] > 1:
        raise ValueError('Radial speed cannot exceed model limits')


def load_configured_profile(source, path, *, tool_actuator=None, tool_grasp=None, tool_parameters=None,
                            component_parameters=None, **unused):
    from .tool_parameters import apply_tool_parameters
    config = {key:read(resolve(reference,path.parent)) for key,reference in source['components'].items()}
    config.update(use_sim_time=source.get('use_sim_time',True),readiness=source['readiness'],execution_scope='operational')
    config.update(selected_profiles(source['profiles'],path.parent))
    runtime = config['tool']['runtime']
    if tool_actuator is not None:runtime['actuator']=tool_actuator
    if tool_grasp is not None:runtime['grasp']=tool_grasp
    apply_tool_parameters(runtime,tool_parameters or {})
    model_path=resolve(source['model_configuration'],path.parent)
    model=read(model_path)
    for key in ('urdf','semantic','mesh_root'):
        model[key]=str(resolve(model[key],model_path.parent))
    config['model_configuration']=model
    motion_path=resolve(source['motion_configuration'],path.parent)
    config['motion_configuration']={key:str(resolve(value,motion_path.parent)) for key,value in read(motion_path).items()}
    config['home_joint_positions']=read(config['robot']['home'])['home_joint_positions']
    configured_tool_model(config)
    mission_path=resolve(source['mission'],path.parent)
    mission=read(mission_path)
    if mission['schema_version'] != 3 or mission['source']['type'] != 'configured_slots':
        raise ValueError('Configured supply requires a configured_slots mission source')
    supply=config['cell']['configured_supply']
    if supply['supply_id'] != mission['source']['supply_id'] or supply['material_id'] != mission['target_id']:
        raise ValueError('Mission and Cell supply identity disagree')
    config['_mission_recipe']=copy.deepcopy(mission)
    config['_mission_recipes']={mission['target_id']:copy.deepcopy(mission)}
    config['orchestration']=dict(material_source='configured_slots',mission_target_id=mission['target_id'],
        supply_id=supply['supply_id'],recipe_directory=write_recipes(config['_mission_recipes']))
    owned_motion = set(source.get('motion', {})) | config['robot']['motion'].keys() | config['tool']['motion'].keys()
    apply_component_parameters(config, component_parameters or {}, configured_supply=True,
                               owned_motion_keys=owned_motion)
    motion=config['motion']
    motion.update(source.get('motion',{}))
    motion.update(config['robot']['motion'])
    motion.update(config['tool']['motion'])
    motion.update(tool_width_control=False,tool_grasp_strategies=runtime['grasp_strategies'],model_id=model['model_id'],planning_group=model['planning_group'],
        payload_touch_links=config['tool']['contact_links']['payload_touch_links'],tool_grasp_collision_links=config['tool']['grasp_pose_collision_links'])
    motion['general_ik_timeout_s']=motion.get('general_ik_timeout_s',1.)
    motion['pick_ik_timeout_s']=motion.pop('general_ik_timeout_s')
    motion['place_total_planning_time_s']=motion.pop('qualification_total_planning_time_s',60.)
    motion['object_to_grasp_tcp_quaternion']=motion.pop('tool_axis_quaternion_xyzw')
    configure_grip_force(config,path)
    module,name=config['cell']['isaac']['configuration_compiler'].split(':')
    scripts=project_root()/'infra/isaac_sim/scripts'
    if str(scripts) not in sys.path:sys.path.insert(0,str(scripts))
    getattr(importlib.import_module(module),name)(configuration=config)
    return config
