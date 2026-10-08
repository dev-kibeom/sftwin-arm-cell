"""Static checks for validation-only PnP composition and profile."""

import importlib.util
import json
import math
from pathlib import Path
import sys
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
ISAAC_SCRIPTS = PACKAGE_ROOT.parents[3] / "infra/isaac_sim/scripts/m0609_cell"
if str(ISAAC_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(ISAAC_SCRIPTS))

from pnp_validation.run_validation import validate_validation_profile  # noqa: E402


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


def test_validation_profile_explicitly_selects_fixed_adapter_and_values():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    node = next(
        action
        for action in _nodes()
        if isinstance(action, Node) and action.node_package == "arm_cell_vision"
    )

    assert node.node_executable == "fixed_detect_target_node"
    assert profile["orchestration"]["mission_target_id"] == "cube_profile"
    assert profile["orchestration"]["recipe_directory"] == "recipes"
    assert not any(
        key.startswith("mission_grasp") or key.startswith("mission_place")
        for key in profile["orchestration"]
    )
    assert profile["validation"]["runtime_runner_entrypoint"].endswith(
        "pnp_validation/entrypoints/start_validation.py"
    )
    assert profile["validation"]["registration_helper_entrypoint"].endswith(
        "register_pnp_fixture.py"
    )
    assert profile["validation"]["registration_handoff_schema"] == (
        "wu14-pnp-registration/v1"
    )
    assert profile["validation"]["registration_ttl_s"] == 30.0
    assert profile["validation"]["registration_ttl_max_s"] == 60.0
    assert profile["validation"]["holding_freshness_ms"] == 500.0
    assert profile["validation"]["open_readiness_timeout_s"] == 2.0
    assert profile["validation"]["open_readiness_timeout_s"] != (
        profile["validation"]["holding_freshness_ms"] / 1000.0
    )
    assert profile["motion"]["planning_max_acceleration_rad_s2"] > 0.0
    assert profile["motion"]["trajectory_sampler"] == "linear"
    assert profile["motion"]["trajectory_clock_liveness_timeout_s"] > 0.0
    assert profile["motion"]["motion_progress_stall_timeout_s"] > 0.0
    assert profile["motion"]["approach_entry_position_tolerance_m"] == 0.010
    assert profile["motion"]["place_approach_entry_position_tolerance_m"] == 0.015
    assert profile["motion"]["place_tracking_tolerance_rad"] == 0.020
    assert profile["motion"][
        "approach_entry_orientation_tolerance_rad"
    ] == pytest.approx(0.08726646259971647)
    assert profile["fixture"]["gravity_enabled"] is False
    assert profile["place"]["deletion_delay_s"] == 3.0
    assert profile["place"]["position_tolerance_m"] == pytest.approx(0.05)
    assert "orientation_tolerance_deg" in profile["capture"]
    assert "orientation_tolerance_rad" not in profile["capture"]
    assert profile["capture"]["position_tolerance_m"] > 0.0
    assert profile["capture"]["orientation_tolerance_deg"] > 0.0
    assert profile["capture"]["expected_contact_width_mm"] > 0.0
    assert profile["capture"]["contact_width_tolerance_mm"] > 0.0
    grasp_dimension_mm = profile["fixture"]["dimensions_m"][0] * 1000.0
    assert grasp_dimension_mm <= 85.0
    assert profile["capture"]["expected_contact_width_mm"] == grasp_dimension_mm
    recipe = json.loads((PACKAGE_ROOT / "config/recipes/cube_profile.json").read_text())
    assert (
        recipe["grasp"]["width_mm"] == profile["capture"]["expected_contact_width_mm"]
    )
    assert profile["orchestration"]["recipe_directory"] == "recipes"
    validate_validation_profile(profile)


def test_validation_profile_owns_moveit_planning_scaling():
    motion_node = next(
        action
        for action in _nodes()
        if isinstance(action, Node) and action.node_executable == "motion_node"
    )
    (motion_parameters,) = evaluate_parameters(
        LaunchContext(), motion_node._Node__parameters
    )

    assert motion_parameters["planning_velocity_scaling_factor"] == 1.0
    assert motion_parameters["planning_acceleration_scaling_factor"] == 1.0


def test_validation_place_target_is_above_the_cnc_workarea():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
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


def test_validation_profile_passes_approach_tolerances_to_motion_node():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    motion = next(
        action
        for action in _nodes()
        if isinstance(action, Node) and action.node_executable == "motion_node"
    )
    (parameters,) = evaluate_parameters(LaunchContext(), motion._Node__parameters)

    assert parameters["approach_entry_position_tolerance_m"] == pytest.approx(
        profile["motion"]["approach_entry_position_tolerance_m"]
    )
    assert parameters["place_approach_entry_position_tolerance_m"] == pytest.approx(
        profile["motion"]["place_approach_entry_position_tolerance_m"]
    )
    assert parameters["approach_entry_orientation_tolerance_rad"] == pytest.approx(
        profile["motion"]["approach_entry_orientation_tolerance_rad"]
    )
    assert parameters["place_tracking_tolerance_rad"] == pytest.approx(
        profile["motion"]["place_tracking_tolerance_rad"]
    )
    assert parameters["holding_confirmation_timeout_ms"] == 1500
    assert (
        parameters["holding_confirmation_timeout_ms"]
        == profile["motion"]["holding_confirmation_timeout_ms"]
    )
    assert "place_approach_direction_object" not in parameters
    assert "place_approach_distance_m" not in parameters
    assert "retract_distance_m" not in parameters


def test_validation_profile_rejects_invalid_approach_tolerances():
    module = _load_launch_module()
    for key in (
        "approach_entry_position_tolerance_m",
        "place_approach_entry_position_tolerance_m",
        "approach_entry_orientation_tolerance_rad",
        "place_tracking_tolerance_rad",
    ):
        for value in (None, 0.0, -1.0, float("nan"), float("inf")):
            candidate = yaml.safe_load(PROFILE_PATH.read_text())
            if value is None:
                del candidate["motion"][key]
            else:
                candidate["motion"][key] = value
            with pytest.raises(RuntimeError, match="finite and positive"):
                module.validate_approach_entry_tolerances(candidate)


def test_validation_profile_uses_geometry_derived_grasp_contract():
    profile = yaml.safe_load(PROFILE_PATH.read_text())

    # Motion resolves the detector-specific observation reference to object center before
    # applying the object-center-to-TCP transform. The WU-14 contact value
    # remains a validation-only geometry sanity reference.
    translation = profile["motion"]["object_to_grasp_tcp_translation"]
    observation_translation = profile["motion"]["observation_to_object_translation"]
    assert observation_translation == pytest.approx([0.0, 0.0, -0.04])
    assert translation == pytest.approx([0.0, 0.0, 0.0375])
    assert observation_translation[2] + translation[2] == pytest.approx(-0.0025)
    assert profile["motion"]["object_to_grasp_tcp_quaternion"] == pytest.approx(
        [1.0, 0.0, 0.0, 0.0]
    )
    assert profile["motion"]["insertion_axis_tcp"] == pytest.approx([0.0, 0.0, 1.0])
    assert profile["motion"]["pick_approach_distance_m"] == pytest.approx(0.05)
    recipe = json.loads((PACKAGE_ROOT / "config/recipes/cube_profile.json").read_text())
    assert recipe["place"]["approach"]["distance_m"] == pytest.approx(0.05)
    assert recipe["place"]["approach"]["direction_object"] == [
        0.0,
        0.0,
        1.0,
    ]
    assert profile["fixture"]["grasp_recipe"][
        "object_top_to_grasp_tcp_z_m"
    ] == pytest.approx(-0.0025)
    assert profile["fixture"]["grasp_recipe"][
        "measured_contact_band_m"
    ] == pytest.approx([-0.024, 0.019])
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
    assert home["home_joint_positions"] == pytest.approx(
        [0.0, 0.0, math.pi / 2.0, 0.0, math.pi / 2.0, 0.0]
    )

    motion = next(
        node
        for node in _nodes()
        if isinstance(node, Node) and node.node_executable == "motion_node"
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


def test_grasp_contract_preserves_top_down_insertion_axis():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    axis = profile["motion"]["insertion_axis_tcp"]
    assert axis == [0.0, 0.0, 1.0]
    assert math.isclose(sum(component * component for component in axis), 1.0)


def test_validation_profile_rejects_impossible_grasp_recipe():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    profile["fixture"]["dimensions_m"][0] = 0.14

    with pytest.raises(ValueError, match="exceeds Robotiq"):
        validate_validation_profile(profile)


def test_validation_launch_does_not_start_production_detector():
    assert all(
        not (
            isinstance(action, Node) and action.node_executable == "detect_target_node"
        )
        for action in _nodes()
    )


def test_production_operational_profile_has_no_fixed_adapter_selection():
    production = yaml.safe_load(
        (PACKAGE_ROOT / "config/operational_profile.yaml").read_text()
    )
    assert production["vision"].get("adapter") != "fixed"
    assert "fixed_vision" not in production
    assert "validation_grasp_contact_policy_enabled" not in production.get("motion", {})
    assert "validation_grasp_contact_links" not in production.get("motion", {})
