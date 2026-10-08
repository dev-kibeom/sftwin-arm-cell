"""Planning configuration must preserve robot semantics and exclude execution."""

from pathlib import Path
import ast
import hashlib
import importlib.util
import json
import re
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
    share = Path(get_package_share_directory("arm_cell_moveit_config"))
    description = Path(get_package_share_directory("arm_cell_description"))
    model = ET.fromstring(
        xacro.process_file(
            str(description / "urdf/m0609_robotiq_2f85.urdf.xacro")
        ).toxml()
    )
    semantic = ET.parse(share / "config/m0609_robotiq_2f85.srdf").getroot()
    links = {link.get("name") for link in model.findall("link")}
    for chain in semantic.findall("group/chain"):
        assert chain.get("base_link") in links
        assert chain.get("tip_link") in links
    for exclusion in semantic.findall("disable_collisions"):
        assert exclusion.get("link1") in links
        assert exclusion.get("link2") in links
    parameters = yaml.safe_load((share / "config/planning_only.yaml").read_text())
    assert parameters["allow_trajectory_execution"] is False
    assert (
        "move_group/MoveGroupExecuteTrajectoryAction"
        in parameters["disable_capabilities"]
    )
    pipeline = yaml.safe_load((share / "config/ompl_planning.yaml").read_text())
    assert "AddTimeOptimalParameterization" not in pipeline["request_adapters"]
    assert "FixStartStateCollision" not in pipeline["request_adapters"]


def test_move_group_uses_simulation_time():
    source_package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "planning_only_launch", source_package / "launch/planning_only.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(
        {"use_rviz": "false", "joint_states": "/joint_states", "planning_profile": ""}
    )
    move_group = next(
        entity
        for entity in module.compose(context)
        if isinstance(entity, Node) and entity.node_package == "moveit_ros_move_group"
    )
    parameters = move_group._Node__parameters
    evaluated = evaluate_parameters(context, parameters)
    assert evaluated[0]["use_sim_time"] is True


def test_validation_profile_selects_time_parameterization_and_acceleration_limits():
    source_package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "planning_only_launch", source_package / "launch/planning_only.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations["planning_profile"] = str(
        Path(__file__).resolve().parents[2]
        / "arm_cell_bringup/config/pnp_validation_profile.yaml"
    )
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
    share = Path(get_package_share_directory("arm_cell_moveit_config"))
    repo_manifest = next(
        parent / "config/arm_cell/m0609_static_environment.json"
        for parent in [Path.cwd(), *Path(__file__).resolve().parents]
        if (parent / "config/arm_cell/m0609_static_environment.json").is_file()
    )
    manifest = json.loads(repo_manifest.read_text())
    selection = json.loads(
        (share / "config/m0609_static_scene_selection.json").read_text()
    )
    artifact = json.loads((share / "config/m0609_static_scene.json").read_text())
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
    assert len(artifact["objects"]) == 23
    assert not any(
        "dimensions" in json.dumps(value) or "pose" in json.dumps(value)
        for value in selection.values()
    )


def test_pnp_static_scene_contains_observed_isaac_collision_obstacles():
    share = Path(get_package_share_directory("arm_cell_moveit_config"))
    artifact = json.loads((share / "config/m0609_pnp_static_scene.json").read_text())
    selection = json.loads(
        (share / "config/m0609_pnp_static_scene_selection.json").read_text()
    )
    repo_manifest = next(
        parent / "config/arm_cell/m0609_static_environment.json"
        for parent in [Path.cwd(), *Path(__file__).resolve().parents]
        if (parent / "config/arm_cell/m0609_static_environment.json").is_file()
    )
    manifest = json.loads(repo_manifest.read_text())
    by_id = load_generator().validate(manifest, selection)
    objects = {obj["id"]: obj for obj in artifact["objects"]}

    assert [obj["id"] for obj in artifact["objects"]] == selection["selected_ids"]
    assert artifact["objects"] == [
        by_id[object_id] for object_id in selection["selected_ids"]
    ]

    assert {
        "cnc_front_left",
        "cnc_front_top",
        "camera_rig_post_left",
        "amr_tray_rail_left",
        "amr_tray_rail_right",
        "amr_tray_rail_rear",
    } <= objects.keys()
    assert objects["cnc_front_left"]["geometry"] == {
        "dimensions_m": [0.08, 0.07, 0.66],
        "type": "box",
    }
    assert objects["cnc_front_left"]["pose"]["position_m"] == [0.30, 0.305, 0.92]
    assert objects["cnc_front_top"]["geometry"] == {
        "dimensions_m": [1.08, 0.07, 0.13],
        "type": "box",
    }
    assert objects["cnc_front_top"]["pose"]["position_m"] == [0.82, 0.305, 1.22]
    assert objects["camera_rig_post_left"]["geometry"] == {
        "dimensions_m": [0.055, 0.055, 1.9],
        "type": "box",
    }
    assert objects["camera_rig_post_left"]["pose"]["position_m"] == [
        -0.56,
        -0.35,
        0.95,
    ]
    expected_rails = {
        "amr_tray_rail_left": ([0.025, 0.46, 0.07], [-0.125, -0.85, 0.675]),
        "amr_tray_rail_right": ([0.025, 0.46, 0.07], [0.525, -0.85, 0.675]),
        "amr_tray_rail_rear": ([0.66, 0.025, 0.07], [0.2, -0.625, 0.675]),
    }
    for object_id, (dimensions, position) in expected_rails.items():
        assert objects[object_id]["geometry"] == {
            "dimensions_m": dimensions,
            "type": "box",
        }
        assert objects[object_id]["pose"]["position_m"] == position


def test_camera_drop_bracket_uses_isaac_builder_source_expression():
    repo_root = Path(__file__).resolve().parents[5]
    amr_builder = (
        repo_root / "infra/isaac_sim/scripts/m0609_cell/scene_builder/amr.py"
    ).read_text()
    composition = (
        repo_root / "infra/isaac_sim/scripts/m0609_cell/scene_builder/composition.py"
    ).read_text()
    amr = re.search(r"amr_x, amr_y = ([^,]+), ([^\n]+)", amr_builder)
    camera = re.search(r"context\.amr_y \+ ([^,\]]+)", composition)
    assert amr and camera

    expected_y = float(amr.group(2).strip()) + float(camera.group(1).strip())
    manifest = json.loads(
        (repo_root / "config/arm_cell/m0609_static_environment.json").read_text()
    )
    bracket = next(
        obj for obj in manifest["objects"] if obj["id"] == "camera_drop_bracket"
    )
    assert bracket["pose"]["position_m"][1] == expected_y


def load_isaac_manifest_resolver():
    repo_root = Path(__file__).resolve().parents[5]
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
        == repo_root / "config/arm_cell/m0609_static_environment.json"
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
        == repo_root / "config/arm_cell/m0609_static_environment.json"
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
    assert len(objects) == 24
