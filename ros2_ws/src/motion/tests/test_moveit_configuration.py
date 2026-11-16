"""Planning configuration must preserve robot semantics and exclude execution."""

from pathlib import Path
import hashlib
import importlib.util
import json
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
import xacro
import yaml


def load_generator():
    source_package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "generate_static_scene", source_package / "scripts/generate_static_scene.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_model_groups_and_limits():
    share = Path(get_package_share_directory("arm_cell_motion"))
    description = (
        Path(get_package_share_directory("arm_cell_motion"))
        / "assets/robots/m0609_robotiq_2f85"
    )
    model = ET.fromstring(
        xacro.process_file(
            str(description / "urdf/m0609_robotiq_2f85.urdf.xacro")
        ).toxml()
    )
    semantic = ET.parse(share / "config/moveit/m0609_robotiq_2f85.srdf").getroot()
    links = {link.get("name") for link in model.findall("link")}
    for chain in semantic.findall("group/chain"):
        assert chain.get("base_link") in links
        assert chain.get("tip_link") in links
    for exclusion in semantic.findall("disable_collisions"):
        assert exclusion.get("link1") in links
        assert exclusion.get("link2") in links
    parameters = yaml.safe_load((share / "config/moveit/planning_only.yaml").read_text())
    assert parameters["allow_trajectory_execution"] is False
    assert (
        "move_group/MoveGroupExecuteTrajectoryAction"
        in parameters["disable_capabilities"]
    )
    pipeline = yaml.safe_load((share / "config/moveit/ompl_planning.yaml").read_text())
    assert "AddTimeOptimalParameterization" not in pipeline["request_adapters"]
    assert "FixStartStateCollision" not in pipeline["request_adapters"]
    assert (
        pipeline["request_adapters"]
        == "default_planner_request_adapters/ResolveConstraintFrames"
    )


def test_move_group_uses_simulation_time():
    source_package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "planning_only_launch", source_package / "launch/planning_only.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "use_rviz": "false",
            "joint_states": "/joint_states",
            "planning_profile": "",
            "log_level": "info",
        }
    )
    move_group = next(
        entity
        for entity in module.compose(context)
        if isinstance(entity, Node) and entity.node_package == "moveit_ros_move_group"
    )
    parameters = move_group._Node__parameters
    evaluated = evaluate_parameters(context, parameters)
    assert evaluated[0]["use_sim_time"] is True


@pytest.mark.parametrize('runtime_document', [False, True])
def test_validation_profile_selects_time_parameterization_and_acceleration_limits(
    tmp_path, runtime_document,
):
    source_package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "planning_only_launch", source_package / "launch/planning_only.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations["log_level"] = "info"
    context.launch_configurations["planning_profile"] = str(
        Path(__file__).resolve().parents[2]
        / "bringup/config/pnp_validation_profile.yaml"
    )
    if runtime_document:
        source = Path(context.launch_configurations['planning_profile'])
        exported = tmp_path / 'runtime.json'
        exported.write_text(json.dumps({
            'kind': 'arm-cell-runtime-configuration', 'schema_version': 1,
            'configuration': yaml.safe_load(source.read_text()),
        }))
        context.launch_configurations['planning_profile'] = str(exported)
    move_group = next(
        entity
        for entity in module.compose(context)
        if isinstance(entity, Node) and entity.node_package == "moveit_ros_move_group"
    )
    parameters = evaluate_parameters(context, move_group._Node__parameters)
    robot = yaml.safe_load((source_package / 'config/robots/m0609.yaml').read_text())
    for joint_name in robot['motion']['arm_joint_names']:
        assert (
            parameters[0][
                f"robot_description_planning.joint_limits.{joint_name}.has_acceleration_limits"
            ]
            is True
        )
        assert (
            parameters[0][
                f"robot_description_planning.joint_limits.{joint_name}.max_acceleration"
            ]
            > 0.0
        )
    assert "AddTimeOptimalParameterization" in parameters[0]["ompl.request_adapters"]


@pytest.mark.parametrize(
    'selection_path',
    sorted((Path(__file__).resolve().parents[1] / 'config/moveit').glob(
        '*_static_scene_selection.json')),
    ids=lambda path: path.stem,
)
def test_static_scene_artifact_matches_canonical_manifest(selection_path):
    share = Path(get_package_share_directory("arm_cell_motion"))
    repo_manifest = next(
        parent / "config/pnp_cnc_static_environment.json"
        for parent in [Path.cwd(), *Path(__file__).resolve().parents]
        if (parent / "config/pnp_cnc_static_environment.json").is_file()
    )
    manifest = json.loads(repo_manifest.read_text())
    selection = json.loads(
        (share / 'config/moveit' / selection_path.name).read_text()
    )
    artifact_name = selection_path.name.replace('_selection.json', '.json')
    artifact = json.loads((share / 'config/moveit' / artifact_name).read_text())
    generator = load_generator()
    by_id = generator.validate(manifest, selection)
    actual = {obj['id']: obj for obj in artifact['objects']}
    assert len(actual) == len(artifact['objects'])  # Duplicate IDs must not be hidden by the map.
    assert actual == {object_id: by_id[object_id] for object_id in selection['selected_ids']}
    assert (
        artifact["source"]["canonical_manifest_sha256"]
        == hashlib.sha256(generator.canonical_bytes(manifest).encode()).hexdigest()
    )
    assert (
        artifact["source"]["selection_sha256"]
        == hashlib.sha256(generator.canonical_bytes(selection).encode()).hexdigest()
    )


def load_isaac_manifest_resolver():
    repo_root = Path(__file__).resolve().parents[4]
    source = (
        repo_root
        / "infra/isaac_sim/scripts/pnp_cnc_cell/scene_builder/static_environment.py"
    )
    spec = importlib.util.spec_from_file_location("isaac_static_environment", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, repo_root


def test_isaac_builder_resolves_manifest_from_repository_source_path():
    resolver, repo_root = load_isaac_manifest_resolver()
    assert (
        resolver.canonical_manifest_path(
            script_path=repo_root
            / "infra/isaac_sim/scripts/pnp_cnc_cell/runtime_steps/setup_scene.py"
        )
        == repo_root / "config/pnp_cnc_static_environment.json"
    )


def test_isaac_builder_script_editor_path_requires_and_accepts_explicit_root(
    monkeypatch,
):
    resolver, repo_root = load_isaac_manifest_resolver()
    temporary_script = Path("/tmp/carb/script_1789055439.py")
    with pytest.raises(RuntimeError, match="SFTWIN_PROJECT_ROOT"):
        resolver.canonical_manifest_path(script_path=temporary_script)
    monkeypatch.setenv("SFTWIN_PROJECT_ROOT", str(repo_root))
    assert (
        resolver.canonical_manifest_path(script_path=temporary_script)
        == repo_root / "config/pnp_cnc_static_environment.json"
    )


def test_isaac_builder_rejects_invalid_explicit_root_and_missing_manifest(tmp_path):
    resolver, _ = load_isaac_manifest_resolver()
    with pytest.raises(RuntimeError, match="not a directory"):
        resolver.canonical_manifest_path(project_root=tmp_path / "missing")
    with pytest.raises(RuntimeError, match="does not contain"):
        resolver.canonical_manifest_path(project_root=tmp_path)


def test_isaac_builder_preserves_canonical_static_environment_semantics():
    resolver, repo_root = load_isaac_manifest_resolver()
    objects = resolver.load_canonical_static_geometry(project_root=repo_root)
    manifest = json.loads(resolver.canonical_manifest_path(project_root=repo_root).read_text())
    assert objects == {obj['id']: obj for obj in manifest['objects']}


def test_selected_robot_limits_survive_validation_time_parameterization(tmp_path):
    source = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        'selected_robot_limits_launch', source / 'launch/planning_only.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    limits = tmp_path / 'joint_limits.yaml'
    limits.write_text(yaml.safe_dump({'joint_limits': {'joint_1': {
        'has_velocity_limits': True, 'max_velocity': 0.25}}}))
    context = LaunchContext()
    context.launch_configurations.update(
        use_rviz='false', joint_states='/joint_states', log_level='info',
        joint_limits_file=str(limits), arm_joint_names='joint_1',
        planning_profile=str(source.parent / 'bringup/config/pnp_validation_profile.yaml'))
    move_group = module.compose(context)[0]
    (parameters, _) = evaluate_parameters(context, move_group._Node__parameters)
    # launch_ros flattens nested parameter dictionaries into dotted keys.
    selected_limits = yaml.safe_load(limits.read_text())['joint_limits']['joint_1']
    assert (parameters['robot_description_planning.joint_limits.joint_1.max_velocity']
            == selected_limits['max_velocity'])
    assert parameters[
        'robot_description_planning.joint_limits.joint_1.has_velocity_limits'] is True
    validation_path = source.parent / 'bringup/config/pnp_validation_profile.yaml'
    validation = yaml.safe_load(validation_path.read_text())
    assert (parameters['robot_description_planning.joint_limits.joint_1.max_acceleration']
            == validation['motion']['planning_max_acceleration_rad_s2'])


def test_standalone_moveit_uses_selected_logical_robot_profile(tmp_path):
    source = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        'selected_robot_profile_launch', source / 'launch/planning_only.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    robot = yaml.safe_load((source / 'config/robots/m0609.yaml').read_text())
    robot['motion']['arm_joint_names'] = ['joint_2']
    robot['joint_limits'] = 'limits.yaml'
    (tmp_path / 'limits.yaml').write_text(yaml.safe_dump({'joint_limits': {
        'joint_2': {'has_velocity_limits': True, 'max_velocity': 0.25}}}))
    selected = tmp_path / 'robot.yaml'
    selected.write_text(yaml.safe_dump(robot))
    context = LaunchContext()
    context.launch_configurations.update(
        robot_profile=str(selected), use_rviz='false',
        joint_states='/joint_states', log_level='info',
        planning_profile=str(source.parent / 'bringup/config/pnp_validation_profile.yaml'))
    (parameters, _) = evaluate_parameters(context, module.compose(context)[0]._Node__parameters)
    limits_document = yaml.safe_load((tmp_path / 'limits.yaml').read_text())
    selected_limits = limits_document['joint_limits']['joint_2']
    assert (parameters['robot_description_planning.joint_limits.joint_2.max_velocity']
            == selected_limits['max_velocity'])
    validation_path = source.parent / 'bringup/config/pnp_validation_profile.yaml'
    validation = yaml.safe_load(validation_path.read_text())
    assert (parameters['robot_description_planning.joint_limits.joint_2.max_acceleration']
            == validation['motion']['planning_max_acceleration_rad_s2'])
    assert 'robot_description_planning.joint_limits.joint_1.max_acceleration' not in parameters
    from launch.actions import DeclareLaunchArgument
    arguments = {item.name for item in module.generate_launch_description().entities
                 if isinstance(item, DeclareLaunchArgument)}
    assert 'robot_profile' in arguments
