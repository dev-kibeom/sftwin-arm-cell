"""Resolve owner configuration and compile mission inputs at the composition edge."""
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile

import yaml

from .material_catalog import load_material_catalog
from .model_validation import validate_model_configuration


def project_root():
    explicit = os.environ.get('SFTWIN_PROJECT_ROOT')
    if explicit:
        return Path(explicit).expanduser().resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent / 'ros2_ws/src').is_dir():
            return parent
    raise ValueError('SFTWIN_PROJECT_ROOT is required for project configuration')


def resolve(reference, base):
    if reference.startswith('package://'):
        package, relative = reference[10:].split('/', 1)
        # The source export CLI also works without installed ROS packages.
        try:
            if (Path(__file__).resolve().parents[1] / 'package.xml').is_file() and Path(__file__).resolve().parents[1].name == 'bringup':
                raise ImportError('source configuration uses source package owners')
            from ament_index_python.packages import get_package_share_directory
            root = Path(get_package_share_directory(package))
        except (ImportError, LookupError):
            source_root = Path(__file__).resolve().parents[2]
            candidates = list(source_root.glob('*/package.xml')) if (source_root / 'bringup/package.xml').is_file() else list((project_root() / 'ros2_ws/src').glob('*/package.xml'))
            import xml.etree.ElementTree as ET
            roots = [p.parent for p in candidates if ET.parse(p).findtext('name') == package]
            if len(roots) != 1:
                raise ValueError(f'Cannot resolve configuration package {package}')
            root = roots[0]
        path = root / relative
        # Motion installs the checkout's canonical assets alongside its config.
        if package == 'arm_cell_motion' and relative.startswith('assets/') and not path.exists():
            source_asset = root.resolve().parents[2] / relative
            if source_asset.exists():
                path = source_asset
        if '..' in Path(relative).parts or Path(relative).is_absolute():
            raise ValueError('Package configuration must stay inside its package')
        return path.resolve()
    if reference.startswith('project://'):
        for ancestor in (Path(base).resolve(), *Path(base).resolve().parents):
            if (ancestor / 'ros2_ws/src/bringup').is_dir():
                return (ancestor / reference[10:]).resolve()
        return (project_root() / reference[10:]).resolve()
    path = Path(reference).expanduser()
    return path if path.is_absolute() else (Path(base) / path).resolve()


def read(path):
    value = yaml.safe_load(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError(f'Configuration must be a mapping: {path}')
    return value


def merge_owned(destination, values, section):
    """Reject competing source owners before producing transport parameters."""
    overlap = destination.keys() & values.keys()
    if overlap:
        raise ValueError(f'Duplicate configuration owner: {section}.{sorted(overlap)[0]}')
    destination.update(values)


DEFAULT_PROFILES = {
    'robot': 'package://arm_cell_motion/config/robots/m0609.yaml',
    'tool': 'package://arm_cell_motion/config/tools/robotiq_2f85.yaml',
    'cell': 'package://arm_cell_bringup/config/cells/isaac_m0609.yaml',
}


def selected_profiles(references, base):
    """Load explicit owner selections; resolve references at each owner's location."""
    if set(references) != set(DEFAULT_PROFILES):
        raise ValueError('Profiles require exactly robot, tool and cell owners')
    owners = {}
    for name, reference in references.items():
        path = resolve(reference, base)
        owner = read(path)
        for key in ('home', 'joint_limits', 'geometry', 'static_scene', 'calibration_file', 'capture'):
            if key in owner:
                owner[key] = str(resolve(owner[key], path.parent))
        if 'work_locations' in owner:
            owner['work_locations'] = {key: read(resolve(value, path.parent))
                                       for key, value in owner['work_locations'].items()}
        if 'validation_planning_scene' in owner and 'static_scene_file' in owner['validation_planning_scene']:
            owner['validation_planning_scene']['static_scene_file'] = str(resolve(
                owner['validation_planning_scene']['static_scene_file'], path.parent))
        owners[name] = owner
    return owners


def compose_owner_profiles(config, path):
    owners = selected_profiles(config.pop('profiles', DEFAULT_PROFILES), path.parent)
    motion = config.setdefault('motion', {})
    for name, sections in config.pop('motion_profile_sections').items():
        for section in sections:
            merge_owned(motion, owners[name][section], 'motion')
    capture_key = config.pop('capture_section')
    if capture_key in config:
        raise ValueError(f'Duplicate configuration owner: {capture_key}')
    config[capture_key] = read(owners['tool']['capture'])
    model_path = resolve(config['model_configuration'], path.parent)
    model = read(model_path)
    for name in ('robot', 'tool'):
        if model[f'{name}_id'] != owners[name][f'{name}_id']:
            raise ValueError(f'Selected model is incompatible with {name} profile')
    for key in ('urdf', 'semantic', 'mesh_root'):
        model[key] = str(resolve(model[key], model_path.parent))
    if 'urdf' in model['isaac']:
        model['isaac']['urdf'] = str(resolve(model['isaac']['urdf'], model_path.parent))
    config['model_configuration'] = model
    config['robot'] = owners['robot']
    config['tool'] = owners['tool']
    config['cell'] = owners['cell']
    config['home_joint_positions'] = read(owners['robot']['home'])['home_joint_positions']
    reference = config['motion_configuration']
    owner_path = resolve(reference, path.parent)
    config['motion_configuration'] = {key: str(resolve(value, owner_path.parent))
                                      for key, value in read(owner_path).items()}
    validate_model_configuration(config)
    for key in ('planning_group', 'model_id'):
        merge_owned(motion, {key: model[key]}, 'motion')
    for key in ('wrap_equivalent_joints', 'place_ik_seed_joints'):
        merge_owned(motion, {key: owners['robot']['joint_policy'][key]}, 'motion')
    merge_owned(motion, {
        'pick_clearance_objects': owners['cell']['collision_objects']['pick_clearance'],
        'tool_grasp_collision_links': owners['tool']['grasp_pose_collision_links'],
        'tool_width_control': 'width' in owners['tool']['runtime']['capabilities'],
        'payload_touch_links': owners['tool']['contact_links']['payload_touch_links'],
        'pick_retract_contact_links': owners['tool']['contact_links']['pick_retract'],
        'pick_retract_contact_object': owners['cell']['contact_objects']['pick_retract'],
    }, 'motion')
    return owners


def compile_mission(path, *, pick_policy=None, place_task=None, material_override=None, tool_quaternion=None, work_locations=None):
    mission = read(path)
    if mission.get('schema_version') != 2:
        raise ValueError(f'Unsupported mission schema: {path}')
    material = copy.deepcopy(material_override) if material_override is not None else read(resolve(mission['material'], Path(path).parent))
    if material['material_id'] != mission['target_id']:
        raise ValueError('Mission material_id and target_id must match')
    grasp = {}
    pick = read(resolve(pick_policy or mission['pick_policy'], Path(path).parent))
    for key in ('yaw_rad', 'unobserved_yaw_policy'):
        grasp[key] = pick.pop(key)
    transform = copy.deepcopy(material['pick_offset'])
    tool = tool_quaternion or read(resolve(DEFAULT_PROFILES['tool'], Path(path).parent))['motion']['tool_axis_quaternion_xyzw']
    transform['quaternion_xyzw'] = quaternion_product(transform['quaternion_xyzw'], tool)
    place = read(resolve(place_task or mission['place_task'], Path(path).parent))
    if 'work_location' in place:
        locations = work_locations if work_locations is not None else selected_profiles(DEFAULT_PROFILES, Path(path).parent)['cell']['work_locations']
        place['desired_object_pose'] = copy.deepcopy(locations[place.pop('work_location')])
    return {'schema_version': 1, 'recipe_id': mission['mission_id'], 'object_to_grasp_tcp': transform,
            'target_id': mission['target_id'], 'grasp': grasp,
            'vision': mission.get('vision', {}),
            'pick': pick,
            'place': place}, material


def mission_directory(directory, output_root=None, *, overrides=None):
    """Produce immutable, content-addressed legacy transport documents.

    YAML owner files are authoritative; generated optional transforms preserve legacy callers.
    """
    recipes = {}
    for path in sorted(Path(directory).glob('*.yaml')):
        recipe, _ = compile_mission(path, **(overrides or {}).get(path.stem, {}))
        if path.stem != recipe['target_id']:
            raise ValueError('Mission filename must match target_id')
        recipes[path.stem] = recipe
    if not recipes:
        raise ValueError(f'No missions found: {directory}')
    return write_recipes(recipes, output_root)


def write_recipes(recipes, output_root=None):
    encoded = json.dumps(recipes, sort_keys=True).encode()
    digest = hashlib.sha256(encoded).hexdigest()
    root = Path(output_root) if output_root else Path(tempfile.gettempdir()) / f'arm-cell-missions-{os.getuid()}'
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    output = root / digest
    output.mkdir(mode=0o700, exist_ok=True)
    for name, recipe in recipes.items():
        # Unique temporary files prevent readers seeing partially written JSON.
        with tempfile.NamedTemporaryFile(mode='w', dir=output, delete=False) as stream:
            json.dump(recipe, stream)
            temporary = stream.name
        os.replace(temporary, output / f'{name}.json')
    return str(output)


def load_profile(path, *, material_id=None, pick_offset_z_m=None):
    path = Path(path).expanduser().resolve()
    config = read(path)
    if config.get('kind') == 'arm-cell-runtime-configuration':
        if config.get('schema_version') != 1:
            raise ValueError('Unsupported runtime configuration schema')
        configuration = config['configuration']
        digest = hashlib.sha256(json.dumps(configuration, sort_keys=True, allow_nan=False).encode()).hexdigest()
        if digest != config.get('configuration_sha256'):
            raise ValueError('Runtime configuration checksum mismatch')
        if any(value is not None for value in (material_id, pick_offset_z_m)):
            raise ValueError('Re-export the source profile to change material trial settings')
        configuration = copy.deepcopy(configuration)
        recipes = configuration.get('_mission_recipes', {configuration['_mission_recipe']['target_id']: configuration['_mission_recipe']})
        configuration['orchestration']['recipe_directory'] = write_recipes(recipes)
        return configuration
    if 'components' not in config:
        if 'profiles' in config:
            compose_owner_profiles(config, path)
        return validation_configuration(config, path, material_id, pick_offset_z_m)
    if any(value is not None for value in (pick_offset_z_m,)):
        raise ValueError('Grasp calibration trial settings require a validation profile')
    if config.get('schema_version') != 2:
        raise ValueError('Unsupported deployment profile schema')
    for section, reference in config.pop('components').items():
        if section in config:
            raise ValueError(f'Duplicate configuration owner: {section}')
        config[section] = read(resolve(reference, path.parent))
    owners = compose_owner_profiles(config, path)
    if 'calibration_file' in config['vision']:
        raise ValueError('Duplicate configuration owner: vision.calibration_file')
    config['vision']['calibration_file'] = owners['cell']['calibration_file']
    if material_id is not None:
        config['orchestration']['mission_target_id'] = material_id
    target = config['orchestration']['mission_target_id']
    detectors = config['vision']['detector_profiles']
    template = copy.deepcopy(next(iter(detectors.values())))
    detectors.setdefault(target, template)
    detectors[target]['fiducial'] = owners['cell']['tray_fiducial']
    for detector in detectors.values():
        for key, value in owners['tool']['vision'].items():
            if key in detector['cuboid']:
                raise ValueError(f'Duplicate configuration owner: vision.{key}')
            detector['cuboid'][key] = value
    merge_owned(config['motion'], config.get('task_execution', {}), 'motion')
    # Preserve the existing ROS parameter contract at the composition boundary.
    for owned, runtime in (
        ('general_ik_timeout_s', 'pick_ik_timeout_s'),
        ('qualification_total_planning_time_s', 'place_total_planning_time_s'),
        ('tool_axis_quaternion_xyzw', 'object_to_grasp_tcp_quaternion'),
    ):
        config['motion'][runtime] = config['motion'].pop(owned)
    orchestration = config['orchestration']
    directory = resolve(orchestration['recipe_directory'], path.parent)
    tool_convention = config['motion']['object_to_grasp_tcp_quaternion']
    recipe, material = compile_mission(directory / f"{orchestration['mission_target_id']}.yaml", tool_quaternion=tool_convention, work_locations=owners['cell']['work_locations'])
    catalog = load_material_catalog(resolve('package://arm_cell_bringup/config/materials', path.parent))
    # An explicit mission may override a catalog material for calibration trials.
    catalog[material['material_id']] = material
    catalog = {key: value for key, value in catalog.items() if (directory / f'{key}.yaml').is_file()}
    config['material_catalog'] = catalog
    config['material_definitions'] = {key: value['geometry'] for key, value in catalog.items()}
    template = copy.deepcopy(config['vision']['detector_profiles'][target])
    for identity in catalog:
        detector = copy.deepcopy(config['vision']['detector_profiles'].get(identity, template))
        detector.setdefault('fiducial', template['fiducial'])
        detector.update(target_id=identity, material_id=identity)
        config['vision']['detector_profiles'][identity] = detector
    config['vision']['material_ids'] = sorted(catalog)
    config['material_handoff']['material_types'] = sorted(catalog)
    config['orchestration']['material_types'] = sorted(catalog)

    offset = material['pick_offset']['translation_m']
    import math
    if len(offset) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in offset):
        raise ValueError('Material pick_offset.translation_m requires three finite metres')
    config['motion']['object_to_grasp_tcp_translation'] = offset
    q = material['pick_offset']['quaternion_xyzw']
    if len(q) != 4 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in q) or not math.isclose(sum(v*v for v in q), 1.0, abs_tol=1e-6):
        raise ValueError('Material pick_offset.quaternion_xyzw must be a unit quaternion')
    # Material-frame rotation followed by the Tool-owned axis convention.
    x, y, z, w = q
    a, b, c, d = config['motion']['object_to_grasp_tcp_quaternion']
    config['motion']['object_to_grasp_tcp_quaternion'] = [
        w*a + x*d + y*c - z*b, w*b - x*c + y*d + z*a,
        w*c + x*b - y*a + z*d, w*d - x*a - y*b - z*c]

    place = recipe['place']
    pose = place['desired_object_pose']
    config['motion']['place_waypoint_target_pose_xyzw'] = pose['position'] + pose['orientation_xyzw']
    config['motion']['place_waypoint_approach_distance_m'] = place['approach']['distance_m']
    config['motion']['place_waypoint_approach_direction_object'] = place['approach']['direction_object']
    orchestration['recipe_directory'] = mission_directory(directory, overrides={key: {'tool_quaternion': tool_convention, 'work_locations': owners['cell']['work_locations']} for key in catalog})
    return config


def export_runtime_configuration(profile_path, output_path, **trial_settings):
    """Export self-contained data for external runtimes; no Python API is shared."""
    profile_path = Path(profile_path).expanduser().resolve()
    configuration = load_profile(profile_path, **trial_settings)
    orchestration = configuration['orchestration']
    directory = orchestration['recipe_directory']
    if str(directory).startswith('package://'):
        directory = mission_directory(resolve(directory, profile_path.parent))
    recipe = configuration.get('_mission_recipe') or json.loads((Path(directory) / f"{orchestration['mission_target_id']}.json").read_text())
    configuration = copy.deepcopy(configuration)
    configuration['_mission_recipes'] = {p.stem: json.loads(p.read_text()) for p in Path(directory).glob('*.json')}
    # External runtimes consume the embedded recipe, never ROS-local temp paths.
    configuration['orchestration'].pop('recipe_directory', None)
    configuration['_mission_recipe'] = recipe
    document = {'kind': 'arm-cell-runtime-configuration', 'schema_version': 1,
                'source_profile': str(profile_path), 'configuration': configuration}
    canonical = json.dumps(configuration, sort_keys=True, allow_nan=False).encode()
    document['configuration_sha256'] = hashlib.sha256(canonical).hexdigest()
    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=destination.parent, delete=False) as stream:
        json.dump(document, stream, indent=2, allow_nan=False)
        temporary = stream.name
    os.replace(temporary, destination)
    return destination


def quaternion_product(first, second):
    x, y, z, w = first
    a, b, c, d = second
    return [w*a + x*d + y*c - z*b, w*b - x*c + y*d + z*a,
            w*c + x*b - y*a + z*d, w*d - x*a - y*b - z*c]


def validation_configuration(config, path, material_id=None, pick_offset_z_m=None):
    """Resolve the validation fixture from the same material owner as production."""
    if 'fixture' not in config:
        return config
    validation = config['validation']
    for owner in validation.pop('planning_scene_profiles'):
        merge_owned(validation['planning_scene'],
                    config[owner]['validation_planning_scene'], 'validation.planning_scene')
    catalog = load_material_catalog(resolve('package://arm_cell_bringup/config/materials', path.parent))
    identity = material_id or config['orchestration']['mission_target_id']
    if identity not in catalog:
        raise ValueError(f'Unknown validation material: {identity}')
    material = copy.deepcopy(catalog[identity])
    dimensions = material['geometry']['nominal_dimensions_m']
    if pick_offset_z_m is not None:
        import math
        if not math.isfinite(pick_offset_z_m) or abs(pick_offset_z_m) > dimensions[2]/2:
            raise ValueError('Trial pick offset must remain inside the material height')
        material['pick_offset']['translation_m'][2] = pick_offset_z_m
    config['orchestration']['mission_target_id'] = identity
    config['orchestration']['material_types'] = [identity]
    config['fixture']['dimensions_m'] = dimensions
    config['fixture']['grasp_recipe']['object_top_to_grasp_tcp_z_m'] = material['pick_offset']['translation_m'][2] - dimensions[2]/2
    config['motion']['object_to_grasp_tcp_translation'] = material['pick_offset']['translation_m']
    config['motion']['object_to_grasp_tcp_quaternion'] = quaternion_product(material['pick_offset']['quaternion_xyzw'], config['motion'].pop('tool_axis_quaternion_xyzw'))
    directory = resolve(config['orchestration']['recipe_directory'], path.parent)
    validation = config['validation']
    options = dict(material_override=material,
                   pick_policy=validation['pick_policy'], place_task=validation['place_task'],
                   tool_quaternion=config['tool']['motion']['tool_axis_quaternion_xyzw'],
                   work_locations=config['cell']['work_locations'])
    overrides = {p.stem: {'tool_quaternion': options['tool_quaternion'],
                           'work_locations': options['work_locations']}
                 for p in directory.glob('*.yaml')}
    overrides[identity] = options
    config['orchestration']['recipe_directory'] = mission_directory(directory, overrides=overrides)
    recipe, _ = compile_mission(directory / f'{identity}.yaml', **options)
    config['_mission_recipe'] = recipe
    config['material_catalog'] = {identity: material}
    config['material_definitions'] = {identity: material['geometry']}
    return config
