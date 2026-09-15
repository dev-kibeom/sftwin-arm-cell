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
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
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
    assert "Time" not in pipeline["request_adapters"]
    assert "FixStartStateCollision" not in pipeline["request_adapters"]


def test_move_group_uses_simulation_time():
    source_package = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(
        "planning_only_launch", source_package / "launch/planning_only.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    move_group = next(
        entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, Node) and entity.node_package == "moveit_ros_move_group"
    )
    parameters = move_group._Node__parameters
    context = LaunchContext()
    use_sim_time = [
        value
        for parameter_set in parameters
        for name, value in parameter_set.items()
        if perform_substitutions(context, name) == "use_sim_time"
    ]
    assert use_sim_time == [True]


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
    assert len(artifact["objects"]) == 12
    assert not any(
        "dimensions" in json.dumps(value) or "pose" in json.dumps(value)
        for value in selection.values()
    )


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
            / "infra/isaac_sim/scripts/m0609_cell/02_setup_scene.py"
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


def test_isaac_builder_preserves_canonical_twelve_object_semantics():
    resolver, repo_root = load_isaac_manifest_resolver()
    objects = resolver["load_canonical_static_geometry"](project_root=repo_root)
    assert set(objects) == resolver["REQUIRED_STATIC_OBJECT_IDS"]
    assert len(objects) == 12
