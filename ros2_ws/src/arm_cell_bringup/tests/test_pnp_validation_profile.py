"""Static checks for validation-only PnP composition and profile."""

import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch/arm_cell_pnp_validation.launch.py"
PROFILE_PATH = PACKAGE_ROOT / "config/pnp_validation_profile.yaml"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "arm_cell_pnp_validation_launch", LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    source_root = PACKAGE_ROOT.parents[0]
    package_roots = {
        "arm_cell_description": source_root / "arm_cell_description",
        "arm_cell_moveit_config": source_root / "arm_cell_moveit_config",
    }
    module.get_package_share_directory = lambda package: (
        str(PACKAGE_ROOT)
        if package == "arm_cell_bringup"
        else (
            str(package_roots[package])
            if package in package_roots
            else get_package_share_directory(package)
        )
    )
    return module


def _nodes():
    module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations["profile"] = str(PROFILE_PATH)
    return module.compose(context)


def test_validation_profile_selects_fixed_vision_and_its_recipe():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    nodes = [action for action in _nodes() if isinstance(action, Node)]
    vision = next(
        action for action in nodes if action.node_package == "arm_cell_vision"
    )

    assert vision.node_executable == "fixed_detect_target_node"
    assert all(node.node_executable != "detect_target_node" for node in nodes)
    mission = profile["orchestration"]
    recipe_path = (
        PACKAGE_ROOT
        / "config"
        / mission["recipe_directory"]
        / f"{mission['mission_target_id']}.json"
    )
    recipe = json.loads(recipe_path.read_text())
    assert recipe["target_id"] == mission["mission_target_id"]
    orchestration = next(
        node for node in nodes if node.node_executable == "orchestration_node"
    )
    (parameters,) = evaluate_parameters(
        LaunchContext(), orchestration._Node__parameters
    )
    assert Path(parameters["recipe_directory"]).resolve() == (
        recipe_path.parent.resolve()
    )
    for key in (
        "mission_target_id",
        "vision_request_timeout_ms",
        "safety_state_freshness_timeout_ms",
    ):
        assert parameters[key] == mission[key]
    assert profile["validation"]["registration_handoff_schema"] == (
        "wu14-pnp-registration/v2"
    )
    grasp_dimension_mm = profile["fixture"]["dimensions_m"][0] * 1000.0
    assert grasp_dimension_mm <= 85.0
    assert recipe["grasp"]["width_mm"] == grasp_dimension_mm


def test_validation_profile_owns_moveit_planning_scaling():
    motion_node = next(
        action
        for action in _nodes()
        if isinstance(action, Node) and action.node_executable == "task_execution_node"
    )
    (motion_parameters,) = evaluate_parameters(
        LaunchContext(), motion_node._Node__parameters
    )

    assert motion_node._Node__ros_arguments == [
        "--log-level",
        "info",
        "--log-level",
        "move_group_interface:=warn",
    ]

    assert motion_parameters["planning_velocity_scaling_factor"] == 1.0
    assert motion_parameters["planning_acceleration_scaling_factor"] == 1.0


def test_validation_place_target_is_above_the_cnc_workarea():
    planning = yaml.safe_load((PACKAGE_ROOT / "config/isaac_planning.yaml").read_text())

    recipe = json.loads((PACKAGE_ROOT / "config/recipes/cube_profile.json").read_text())
    assert recipe["place"]["desired_object_pose"]["frame"] == "base_link"
    assert recipe["place"]["desired_object_pose"]["position"] == pytest.approx(
        [0.50, 0.25, 0.37]
    )
    world_to_base = planning["world_to_base"]
    target_in_world = [
        recipe["place"]["desired_object_pose"]["position"][0] + world_to_base["x"],
        recipe["place"]["desired_object_pose"]["position"][1] + world_to_base["y"],
        recipe["place"]["desired_object_pose"]["position"][2] + world_to_base["z"],
    ]
    assert target_in_world == pytest.approx([0.65, 0.10, 1.05])
    assert recipe["place"]["desired_object_pose"]["orientation_xyzw"] == pytest.approx(
        [0.0, 0.0, 0.0, 1.0]
    )


def test_validation_profile_passes_motion_completion_tolerances_to_motion_node():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    motion = next(
        action
        for action in _nodes()
        if isinstance(action, Node) and action.node_executable == "task_execution_node"
    )
    (parameters,) = evaluate_parameters(LaunchContext(), motion._Node__parameters)

    assert parameters["cartesian_tcp_orientation_tolerance_rad"] == pytest.approx(
        profile["motion"]["cartesian_tcp_orientation_tolerance_rad"]
    )
    assert parameters["trajectory_goal_tolerance_rad"] == pytest.approx(
        profile["motion"]["trajectory_goal_tolerance_rad"]
    )
    assert parameters["holding_confirmation_timeout_ms"] == 1500
    assert (
        parameters["holding_confirmation_timeout_ms"]
        == profile["motion"]["holding_confirmation_timeout_ms"]
    )
    assert "place_approach_direction_object" not in parameters
    assert "place_approach_distance_m" not in parameters
    assert "retract_distance_m" not in parameters


def test_validation_profile_rejects_invalid_motion_completion_tolerances():
    module = _load_launch_module()
    for key in (
        "cartesian_tcp_orientation_tolerance_rad",
        "trajectory_goal_tolerance_rad",
    ):
        for value in (None, 0.0, -1.0, float("nan"), float("inf")):
            candidate = yaml.safe_load(PROFILE_PATH.read_text())
            if value is None:
                del candidate["motion"][key]
            else:
                candidate["motion"][key] = value
            with pytest.raises(RuntimeError, match="finite and positive"):
                module.validate_motion_completion_tolerances(candidate)


def test_validation_profile_keeps_recipe_and_geometry_contracts_aligned():
    profile = yaml.safe_load(PROFILE_PATH.read_text())

    # Motion resolves the detector-specific observation reference to object center before
    # applying the object-center-to-TCP transform. The WU-14 contact value
    # remains a validation-only geometry sanity reference.
    translation = profile["motion"]["object_to_grasp_tcp_translation"]
    observation_translation = profile["motion"]["observation_to_object_translation"]
    assert observation_translation[:2] == [0.0, 0.0]
    assert observation_translation[2] == pytest.approx(
        -profile["fixture"]["dimensions_m"][2] / 2.0
    )
    grasp_recipe = profile["fixture"]["grasp_recipe"]
    grasp_offset = observation_translation[2] + translation[2]
    assert grasp_offset == pytest.approx(grasp_recipe["object_top_to_grasp_tcp_z_m"])
    contact_band = grasp_recipe["measured_contact_band_m"]
    assert contact_band[0] <= grasp_offset <= contact_band[1]
    assert profile["motion"]["object_to_grasp_tcp_quaternion"] == pytest.approx(
        [1.0, 0.0, 0.0, 0.0]
    )
    assert profile["motion"]["insertion_axis_tcp"] == pytest.approx([0.0, 0.0, 1.0])
    recipe = json.loads((PACKAGE_ROOT / "config/recipes/cube_profile.json").read_text())
    assert recipe["pick"]["approach_distance_m"] == pytest.approx(0.05)
    assert recipe["place"]["approach"]["distance_m"] == pytest.approx(0.05)
    assert recipe["place"]["approach"]["direction_object"] == [
        0.0,
        0.0,
        1.0,
    ]
    assert profile["motion"]["validation_grasp_contact_policy_enabled"] is True
    assert profile["motion"]["validation_grasp_contact_links"] == [
        "gripper_robotiq_85_left_finger_tip_link",
        "gripper_robotiq_85_right_finger_tip_link",
    ]


def test_validation_profile_uses_canonical_home_joint_pose():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    home = yaml.safe_load(
        (PACKAGE_ROOT / "config/home_joint_positions.yaml").read_text()
    )

    assert "home_joint_positions" not in profile["motion"]
    assert len(home["home_joint_positions"]) == 6

    motion = next(
        node
        for node in _nodes()
        if isinstance(node, Node) and node.node_executable == "task_execution_node"
    )
    (parameters,) = evaluate_parameters(LaunchContext(), motion._Node__parameters)
    assert parameters["home_joint_positions"] == pytest.approx(
        home["home_joint_positions"]
    )


def test_sf_grasp_tcp_matches_nominal_contact_geometry():
    # The repository's canonical model is the 2F85 xacro.
    urdf = (
        PACKAGE_ROOT.parents[0]
        / "arm_cell_description/urdf/m0609_robotiq_2f85.urdf.xacro"
    )
    root = ET.parse(urdf).getroot()
    joint = root.find(".//joint[@name='gripper_to_sf_grasp_tcp']/origin")
    assert joint is not None
    xyz = [float(value) for value in joint.attrib["xyz"].split()]
    assert xyz[0] == pytest.approx(0.0, abs=1e-6)
    assert xyz[1] == pytest.approx(-0.0003749994, abs=2e-6)
    assert xyz[2] == pytest.approx(0.1357482403, abs=2e-6)
    assert joint.attrib["rpy"] == "0 0 0"


def test_production_operational_profile_has_no_fixed_adapter_selection():
    production = yaml.safe_load(
        (PACKAGE_ROOT / "config/operational_profile.yaml").read_text()
    )
    assert production["vision"].get("adapter") != "fixed"
    assert "fixed_vision" not in production
    assert "validation_grasp_contact_policy_enabled" not in production.get("motion", {})
    assert "validation_grasp_contact_links" not in production.get("motion", {})
