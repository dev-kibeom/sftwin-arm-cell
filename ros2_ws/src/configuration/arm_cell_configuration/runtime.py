"""Resolve owner configuration and compile mission inputs at the composition edge."""
import copy
import hashlib
import json
import os
from pathlib import Path
import tempfile

import yaml


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
        # Source operation is also supported in Isaac without a ROS environment.
        try:
            if (Path(__file__).resolve().parents[1] / 'package.xml').is_file() and Path(__file__).resolve().parents[1].name == 'configuration':
                raise ImportError('source configuration uses source package owners')
            from ament_index_python.packages import get_package_share_directory
            root = Path(get_package_share_directory(package))
        except (ImportError, LookupError):
            source_root = Path(__file__).resolve().parents[2]
            candidates = list(source_root.glob('*/package.xml')) if (source_root / 'configuration/package.xml').is_file() else list((project_root() / 'ros2_ws/src').glob('*/package.xml'))
            import xml.etree.ElementTree as ET
            roots = [p.parent for p in candidates if ET.parse(p).findtext('name') == package]
            if len(roots) != 1:
                raise ValueError(f'Cannot resolve configuration package {package}')
            root = roots[0]
        path = root / relative
        if '..' in Path(relative).parts or Path(relative).is_absolute():
            raise ValueError('Package configuration must stay inside its package')
        return path.resolve()
    if reference.startswith('project://'):
        for ancestor in (Path(base).resolve(), *Path(base).resolve().parents):
            if (ancestor / 'ros2_ws/src/configuration').is_dir():
                return (ancestor / reference[10:]).resolve()
        return (project_root() / reference[10:]).resolve()
    path = Path(reference).expanduser()
    return path if path.is_absolute() else (Path(base) / path).resolve()


def read(path):
    value = yaml.safe_load(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError(f'Configuration must be a mapping: {path}')
    return value


def compile_mission(path):
    mission = read(path)
    if mission.get('schema_version') != 2:
        raise ValueError(f'Unsupported mission schema: {path}')
    material = read(resolve(mission['material'], Path(path).parent))
    if material['material_id'] != mission['target_id']:
        raise ValueError('Mission material_id and target_id must match')
    grasp = copy.deepcopy(material['grasp'])
    pick = read(resolve(mission['pick_policy'], Path(path).parent))
    for key in ('yaw_rad', 'unobserved_yaw_policy'):
        grasp[key] = pick.pop(key)
    if 'width_mm' in grasp:
        grasp['min_expected_contact_width_mm'] = grasp.pop('width_mm')
        grasp['max_expected_contact_width_mm'] = grasp['min_expected_contact_width_mm']
    return {'schema_version': 1, 'recipe_id': mission['mission_id'],
            'target_id': mission['target_id'], 'grasp': grasp,
            'vision': mission.get('vision', {}),
            'pick': pick,
            'place': read(resolve(mission['place_task'], Path(path).parent))}, material


def mission_directory(directory, output_root=None):
    """Produce immutable, content-addressed legacy transport documents.

    The C++ runtime contract stays stable; YAML owner files are authoritative.
    """
    recipes = {}
    for path in sorted(Path(directory).glob('*.yaml')):
        recipe, _ = compile_mission(path)
        if path.stem != recipe['target_id']:
            raise ValueError('Mission filename must match target_id')
        recipes[path.stem] = recipe
    if not recipes:
        raise ValueError(f'No missions found: {directory}')
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


def load_profile(path):
    path = Path(path).expanduser().resolve()
    config = read(path)
    if 'components' not in config:
        return config  # Isolated historical validation fixture contract.
    if config.get('schema_version') != 2:
        raise ValueError('Unsupported deployment profile schema')
    for section, reference in config.pop('components').items():
        if section in config:
            raise ValueError(f'Duplicate configuration owner: {section}')
        config[section] = read(resolve(reference, path.parent))
    config['vision']['calibration_file'] = str(resolve(config['vision']['calibration_file'], path.parent))
    target = config['orchestration']['mission_target_id']
    config['vision']['detector_profiles'][target]['fiducial'] = config.pop('cell_geometry')['tray_fiducial']
    config['motion'].update(config.get('task_execution', {}))
    # Preserve the existing ROS parameter contract at the composition boundary.
    for owned, runtime in (
        ('general_ik_timeout_s', 'pick_ik_timeout_s'),
        ('qualification_total_planning_time_s', 'place_total_planning_time_s'),
        ('tool_axis_quaternion_xyzw', 'object_to_grasp_tcp_quaternion'),
    ):
        config['motion'][runtime] = config['motion'].pop(owned)
    orchestration = config['orchestration']
    directory = resolve(orchestration['recipe_directory'], path.parent)
    recipe, material = compile_mission(directory / f"{orchestration['mission_target_id']}.yaml")
    config['material_definitions'] = {material['material_id']: material['geometry']}
    offset = material['pick_offset']['translation_m']
    import math
    if len(offset) != 3 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in offset):
        raise ValueError('Material pick_offset.translation_m requires three finite metres')
    config['motion']['object_to_grasp_tcp_translation'] = offset
    q = material['pick_offset']['quaternion_xyzw']
    if len(q) != 4 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in q) or not math.isclose(sum(v*v for v in q), 1.0, abs_tol=1e-6):
        raise ValueError('Material pick_offset.quaternion_xyzw must be a unit quaternion')
    # Material-frame rotation followed by the Motion-owned tool axis convention.
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
    orchestration['recipe_directory'] = mission_directory(directory)
    return config


def selected_profile():
    return Path(os.environ.get('ARM_CELL_PROFILE', project_root() / 'ros2_ws/src/bringup/config/operational_profile.yaml'))
