"""Planning configuration must preserve robot semantics and exclude execution."""

from pathlib import Path
import ast
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
def test_validation_profile_selects_time_parameterization_and_acceleration_limits(tmp_path, runtime_document):
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
    for joint_name in (f"joint_{index}" for index in range(1, 7)):
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


def test_static_scene_artifact_matches_canonical_manifest():
    share = Path(get_package_share_directory("arm_cell_motion"))
    repo_manifest = next(
        parent / "config/m0609_static_environment.json"
        for parent in [Path.cwd(), *Path(__file__).resolve().parents]
        if (parent / "config/m0609_static_environment.json").is_file()
    )
    manifest = json.loads(repo_manifest.read_text())
    selection = json.loads(
        (share / "config/moveit/m0609_static_scene_selection.json").read_text()
    )
    artifact = json.loads((share / "config/moveit/m0609_static_scene.json").read_text())
    generator = load_generator()
    by_id = generator.validate(manifest, selection)
    assert [obj["id"] for obj in artifact["objects"]] == selection["selected_ids"]
    assert artifact["objects"] == [
        by_id[object_id] for object_id in selection["selected_ids"]
    ]
    assert (
        artifact["source"]["canonical_manifest_sha256"]
        == hashlib.sha256(generator.canonical_bytes(manifest).encode()).hexdigest()
    )
    assert (
        artifact["source"]["selection_sha256"]
        == hashlib.sha256(generator.canonical_bytes(selection).encode()).hexdigest()
    )
    assert len(artifact["objects"]) == len(selection["selected_ids"])
    assert not any(
        "dimensions" in json.dumps(value) or "pose" in json.dumps(value)
        for value in selection.values()
    )


def test_pnp_static_scene_contains_observed_isaac_collision_obstacles():
    share = Path(get_package_share_directory("arm_cell_motion"))
    artifact = json.loads((share / "config/moveit/m0609_pnp_static_scene.json").read_text())
    selection = json.loads(
        (share / "config/moveit/m0609_pnp_static_scene_selection.json").read_text()
    )
    repo_manifest = next(
        parent / "config/m0609_static_environment.json"
        for parent in [Path.cwd(), *Path(__file__).resolve().parents]
        if (parent / "config/m0609_static_environment.json").is_file()
    )
    manifest = json.loads(repo_manifest.read_text())
    by_id = load_generator().validate(manifest, selection)
    objects = {obj["id"]: obj for obj in artifact["objects"]}

    assert [obj["id"] for obj in artifact["objects"]] == selection["selected_ids"]
    assert artifact["objects"] == [
        by_id[object_id] for object_id in selection["selected_ids"]
    ]

    assert {
        "amr_tray",  # Required by the bounded PICK retract fingertip exception.
        "cnc_front_left",
        "cnc_front_top",
        "camera_rig_post_left",
        "camera_rig_post_right",
        "amr_tray_rail_left",
        "amr_tray_rail_right",
        "amr_tray_rail_rear",
        "amr_tray_marker_post",
    } <= objects.keys()
    # Geometry is owned by the canonical manifest. The full equality above
    # checks every selected object's dimensions and pose without a second copy
    # of installation coordinates in this test. Required IDs remain explicit
    # to catch accidental loss of collision coverage from the selection.
    assert not {
        "amr_raw_slot_support",
        "amr_raw_slot_marker_guard",
        "amr_finished_slot_support",
    }.intersection(objects)


def test_camera_drop_bracket_uses_canonical_static_geometry():
    resolver, repo_root = load_isaac_manifest_resolver()
    objects = resolver["load_canonical_static_geometry"](project_root=repo_root)
    manifest = json.loads(
        (repo_root / "config/m0609_static_environment.json").read_text()
    )
    bracket = next(
        obj for obj in manifest["objects"] if obj["id"] == "camera_drop_bracket"
    )
    assert objects["camera_drop_bracket"] == bracket


def load_isaac_manifest_resolver():
    repo_root = Path(__file__).resolve().parents[4]
    source = (
        repo_root
        / "infra/isaac_sim/scripts/m0609_cell/scene_builder/static_environment.py"
    )
    tree = ast.parse(source.read_text())
    selected = [
        node
        for node in tree.body
        if isinstance(node, (ast.Assign, ast.FunctionDef))
        and getattr(node, "name", None)
        in {"canonical_manifest_path", "load_canonical_static_geometry"}
        or isinstance(node, ast.Assign)
        and any(
            getattr(target, "id", None)
            in {"STATIC_MANIFEST_RELATIVE_PATH", "REQUIRED_STATIC_OBJECT_IDS"}
            for target in node.targets
        )
    ]
    namespace = {
        "Path": Path,
        "json": json,
        "os": __import__("os"),
        "__file__": str(source),
    }
    exec(
        compile(ast.Module(body=selected, type_ignores=[]), str(source), "exec"),
        namespace,
    )
    return namespace, repo_root


def test_isaac_builder_resolves_manifest_from_repository_source_path():
    resolver, repo_root = load_isaac_manifest_resolver()
    assert (
        resolver["canonical_manifest_path"](
            script_path=repo_root
            / "infra/isaac_sim/scripts/m0609_cell/runtime_steps/setup_scene.py"
        )
        == repo_root / "config/m0609_static_environment.json"
    )


def test_isaac_builder_script_editor_path_requires_and_accepts_explicit_root(
    monkeypatch,
):
    resolver, repo_root = load_isaac_manifest_resolver()
    temporary_script = Path("/tmp/carb/script_1789055439.py")
    with pytest.raises(RuntimeError, match="SFTWIN_PROJECT_ROOT"):
        resolver["canonical_manifest_path"](script_path=temporary_script)
    monkeypatch.setenv("SFTWIN_PROJECT_ROOT", str(repo_root))
    assert (
        resolver["canonical_manifest_path"](script_path=temporary_script)
        == repo_root / "config/m0609_static_environment.json"
    )


def test_isaac_builder_rejects_invalid_explicit_root_and_missing_manifest(tmp_path):
    resolver, _ = load_isaac_manifest_resolver()
    with pytest.raises(RuntimeError, match="not a directory"):
        resolver["canonical_manifest_path"](project_root=tmp_path / "missing")
    with pytest.raises(RuntimeError, match="does not contain"):
        resolver["canonical_manifest_path"](project_root=tmp_path)


def test_isaac_builder_preserves_canonical_static_environment_semantics():
    resolver, repo_root = load_isaac_manifest_resolver()
    objects = resolver["load_canonical_static_geometry"](project_root=repo_root)
    assert set(objects) == resolver["REQUIRED_STATIC_OBJECT_IDS"]
    assert len(objects) == 23


def test_selected_robot_limits_survive_validation_time_parameterization(tmp_path):
    source = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('selected_robot_limits_launch', source / 'launch/planning_only.launch.py')
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
    assert parameters['robot_description_planning.joint_limits.joint_1.max_velocity'] == 0.25
    assert parameters['robot_description_planning.joint_limits.joint_1.has_velocity_limits'] is True
    assert parameters['robot_description_planning.joint_limits.joint_1.max_acceleration'] == 5.0
