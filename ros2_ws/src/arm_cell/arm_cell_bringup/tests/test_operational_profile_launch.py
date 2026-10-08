"""Static checks for the supported ARM Cell operational profile."""

import importlib.util
import json
import math
from pathlib import Path
from xml.etree import ElementTree

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, Substitution
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch/arm_cell_operational.launch.py"
PROFILE_PATH = PACKAGE_ROOT / "config/operational_profile.yaml"
HOME_PATH = PACKAGE_ROOT / "config/home_joint_positions.yaml"


def _assert_installed_package_identity(package_name):
    share_path = Path(get_package_share_directory(package_name))
    package_xml = share_path / "package.xml"
    assert package_xml.is_file(), (
        f"operational profile package {package_name!r} must resolve to an installed "
        f"package.xml, got {share_path}"
    )
    declared_name = ElementTree.parse(package_xml).findtext("name")
    assert declared_name == package_name, (
        f"installed package identity mismatch: launch requested {package_name!r}, "
        f"package.xml declares {declared_name!r}"
    )


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "arm_cell_operational_launch", LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda package: (
        str(PACKAGE_ROOT)
        if package == "arm_cell_bringup"
        else get_package_share_directory(package)
    )
    return module


def _composed_nodes(log_level="info"):
    module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations["profile"] = str(PROFILE_PATH)
    context.launch_configurations["include_simulation"] = "true"
    context.launch_configurations["log_level"] = log_level
    context.launch_configurations["action_logger_level"] = "warn"
    return module.compose(context)


def test_profile_declares_reviewed_runtime_nodes_and_readiness_config():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    nodes = [action for action in _composed_nodes() if isinstance(action, Node)]

    project_nodes = {(node.node_package, node.node_executable) for node in nodes}
    assert project_nodes == {
        ("arm_cell_vda", "external_state_mock_node"),
        ("arm_cell_integration", "material_handoff_node"),
        ("arm_cell_safety", "safety_node"),
        ("arm_cell_vision", "detect_target_node"),
        ("arm_cell_motion_moveit2", "motion_node"),
        ("arm_cell_bt", "orchestration_node"),
    }
    assert all(node.node_package != "image_transport" for node in nodes)
    for package_name, _ in {
        (node.node_package, node.node_executable) for node in nodes
    }:
        _assert_installed_package_identity(package_name)
    assert profile["readiness"]["fail_closed_until_inputs_fresh"] is True
    assert profile["readiness"]["required_motion_backend"] is True


def test_simulation_composition_includes_camera_tf_owner_launch():
    includes = [
        action
        for action in _composed_nodes()
        if isinstance(action, IncludeLaunchDescription)
    ]
    included_sources = []
    for action in includes:
        source = action.launch_description_source
        source.get_launch_description(LaunchContext())
        included_sources.append(source.location)
    assert any(
        isinstance(path, str) and path.endswith("/launch/isaac_camera.launch.py")
        for path in included_sources
    ), included_sources


def test_debug_selects_owned_diagnostics_without_raising_process_default():
    module = _load_launch_module()
    expected_debug_loggers = {
        "arm_cell_integration": {"material_handoff_node"},
        "arm_cell_safety": {"safety_node"},
        "arm_cell_vision": {"detect_target_node"},
        "arm_cell_motion_moveit2": {
            "motion_node",
            "isaac_motion_backend",
            "motion_core",
            "motion_action_server",
            "task_executor",
        },
        "arm_cell_bt": {"orchestration_node"},
        "arm_cell_vda": set(),
    }
    for package_name, expected in expected_debug_loggers.items():
        info_arguments = module.node_ros_arguments(package_name, "info")
        assert info_arguments[:2] == ["--log-level", "info"]
        debug_arguments = module.node_ros_arguments(package_name, "debug")
        assert debug_arguments[:2] == ["--log-level", "info"]
        configured_loggers = {
            argument.removesuffix(":=debug")
            for argument in debug_arguments[3::2]
            if argument.endswith(":=debug")
        }
        assert configured_loggers == expected
        action_loggers = {
            "arm_cell_motion_moveit2": "isaac_motion_backend.rclcpp_action",
            "arm_cell_bt": "orchestration_node.rclcpp_action",
        }
        if package_name in action_loggers:
            logger = action_loggers[package_name]
            assert f"{logger}:=warn" in info_arguments
            assert f"{logger}:=warn" in debug_arguments
            user_debug_arguments = module.node_ros_arguments(
                package_name, "debug", "debug"
            )
            assert f"{logger}:=debug" in user_debug_arguments
        else:
            assert info_arguments == ["--log-level", "info"]
    for framework_package in (
        "rcl",
        "rclcpp",
        "rmw",
        "tf2",
        "robot_state_publisher",
        "move_group",
    ):
        assert module.node_ros_arguments(framework_package, "debug") == [
            "--log-level",
            "info",
        ]

    planning_source = Path(__file__).parents[1] / "launch/isaac_planning.launch.py"
    spec = importlib.util.spec_from_file_location(
        "isaac_planning_launch", planning_source
    )
    planning_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(planning_module)
    assert planning_module.node_ros_arguments() == ["--log-level", "info"]


def test_operational_profile_selects_static_collision_geometry_for_isaac_motion():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    assert profile["simulation"]["static_scene"] is True

    scene_path = (
        Path(__file__).resolve().parents[2]
        / "arm_cell_moveit_config"
        / "config"
        / "m0609_static_scene.json"
    )
    scene = json.loads(scene_path.read_text())
    objects = {item["id"]: item for item in scene["objects"]}
    assert {
        "amr_tray",
        "amr_tray_rail_left",
        "amr_tray_rail_right",
        "amr_tray_rail_front",
        "amr_tray_rail_rear",
        "amr_raw_slot_support",
    }.issubset(objects)
    assert objects["amr_tray"]["geometry"]["dimensions_m"] == [0.66, 0.46, 0.045]
    assert objects["amr_raw_slot_support"]["geometry"]["dimensions_m"] == [
        *profile["vision"]["detector_profiles"]["RawPart"][
            "support_region_dimensions_m"
        ],
        0.008,
    ]
    assert "required_vda_heartbeat" not in profile["readiness"]
    assert profile["vda"]["publish_period_ms"] == 100
    assert profile["vda"]["material_phase_period_ms"] == 100
    assert profile["vda"]["degraded_publish_period_ms"] == 300
    assert profile["material_handoff"]["transfer_duration_ms"] == 100
    assert profile["material_handoff"]["source_freshness_timeout_ms"] == 500
    assert profile["safety"]["freshness_timeout_ms"] == 500
    assert profile["safety"]["source_timestamp_max_age_ms"] == 2000
    assert profile["safety"]["degraded_freshness_threshold_ms"] == 180
    assert profile["safety"]["degraded_freshness_inputs"] == ["AMR", "PACKML"]
    assert profile["safety"]["degraded_velocity_scale"] == 0.5
    assert profile["safety"]["degraded_acceleration_scale"] == 0.5
    assert profile["vision"]["sync_slop_ms"] == 10.0


def test_operational_profile_uses_w06_place_destination():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    rawpart = profile["vision"]["detector_profiles"]["RawPart"]
    mission = profile["orchestration"]
    assert mission["mission_target_id"] == rawpart["target_id"]
    recipe = json.loads((PACKAGE_ROOT / "config/recipes/RawPart.json").read_text())
    assert recipe["target_id"] == mission["mission_target_id"]
    assert recipe["grasp"]["width_mm"] == rawpart["dimensions_m"][0] * 1000.0
    assert rawpart["dimensions_m"] == [0.07, 0.07, 0.08]
    assert profile["motion"]["observation_to_object_translation"] == [
        0.0,
        0.0,
        -0.04,
    ]
    assert profile["motion"]["object_to_grasp_tcp_translation"] == [0.0, 0.0, 0.0]
    pick_approach_distance = profile["motion"]["pick_approach_distance_m"]
    rawpart_top_surface_offset = rawpart["dimensions_m"][2] / 2.0
    assert pick_approach_distance == 0.15
    assert recipe["place"]["approach"]["direction_object"] == [0.0, 0.0, 1.0]
    assert recipe["place"]["approach"]["distance_m"] == 0.015
    assert pick_approach_distance - rawpart_top_surface_offset >= 0.10
    assert recipe["place"]["retract_distance_m"] == 0.05
    assert recipe["grasp"]["yaw_rad"] == 0.0
    assert recipe["place"]["desired_object_pose"]["frame"] == "base_link"
    assert recipe["place"]["desired_object_pose"]["position"] == [0.50, 0.25, 0.3675]
    assert recipe["place"]["position_tolerance_m"] == 0.01
    assert recipe["place"]["orientation_constraint"] == "free"
    assert "tool_orientation" not in recipe["place"]
    assert math.isclose(
        recipe["place"]["orientation_tolerance_rad"],
        math.radians(2.0),
        abs_tol=1e-9,
    )
    assert recipe["place"]["desired_object_pose"]["orientation_xyzw"] == [
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def test_place_approach_geometry_is_recipe_owned_and_not_injected_into_motion():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    motion = next(
        action
        for action in _composed_nodes()
        if isinstance(action, Node) and action.node_executable == "motion_node"
    )
    (parameters,) = evaluate_parameters(LaunchContext(), motion._Node__parameters)
    mission = profile["orchestration"]

    assert "place_approach_direction_object" not in parameters
    assert "place_approach_distance_m" not in parameters
    assert "retract_distance_m" not in parameters
    assert mission["recipe_directory"] == "recipes"
    assert not any(
        key.startswith("mission_grasp") or key.startswith("mission_place")
        for key in mission
    )


def test_profile_preserves_canonical_topics_and_backend_neutral_startup():
    nodes = _composed_nodes()
    by_executable = {
        action.node_executable: action for action in nodes if isinstance(action, Node)
    }

    profile = yaml.safe_load(PROFILE_PATH.read_text())
    module = _load_launch_module()
    assert module.profile_parameters(profile, "motion")["backend_mode"] == "isaac"
    assert "backend_available" not in profile["motion"]
    assert (
        module.profile_parameters(profile, "motion")["enable_test_backend_controls"]
        is False
    )
    assert module.profile_parameters(profile, "safety")["freshness_timeout_ms"] == 500
    assert (
        module.profile_parameters(profile, "safety")["source_timestamp_max_age_ms"]
        == 2000
    )
    assert module.profile_parameters(profile, "safety")[
        "degraded_freshness_inputs"
    ] == [
        "AMR",
        "PACKML",
    ]
    assert (
        module.profile_parameters(profile, "safety")["degraded_freshness_threshold_ms"]
        == 180
    )
    assert profile["vision"]["rgb_topic"] == "/camera/color/image_raw"
    assert (
        profile["vision"]["depth_topic"] == "/camera/aligned_depth_to_color/image_raw"
    )


def test_operational_nodes_have_launch_encodable_parameters():
    context = LaunchContext()
    for node in [action for action in _composed_nodes() if isinstance(action, Node)]:
        evaluate_parameters(context, node._Node__parameters)

    integration = next(
        node
        for node in _composed_nodes()
        if isinstance(node, Node) and node.node_executable == "material_handoff_node"
    )
    (parameters,) = evaluate_parameters(context, integration._Node__parameters)
    assert parameters["transfer_duration_ms"] == 100
    assert parameters["source_freshness_timeout_ms"] == 500
    motion = next(
        node
        for node in _composed_nodes()
        if isinstance(node, Node) and node.node_executable == "motion_node"
    )
    (motion_parameters,) = evaluate_parameters(context, motion._Node__parameters)
    assert motion_parameters["planning_time_s"] == 15.0
    assert motion_parameters["place_candidate_planning_time_s"] == 15.0
    assert motion_parameters["place_total_planning_time_s"] == 60.0
    assert motion_parameters["holding_confirmation_timeout_ms"] == 2000


def test_motion_backend_receives_canonical_arm_kinematics_parameters():
    motion = next(
        node
        for node in _composed_nodes()
        if isinstance(node, Node) and node.node_executable == "motion_node"
    )
    (parameters,) = evaluate_parameters(LaunchContext(), motion._Node__parameters)
    assert parameters["planning_time_s"] == 15.0
    assert parameters["planning_max_acceleration_rad_s2"] == 5.0
    assert parameters["planning_max_acceleration_rad_s2"] > 0.0
    assert parameters["place_candidate_planning_time_s"] == 15.0
    assert parameters["place_total_planning_time_s"] == 60.0
    assert parameters["robot_description_kinematics.arm.kinematics_solver"] == (
        "kdl_kinematics_plugin/KDLKinematicsPlugin"
    )
    assert (
        parameters[
            "robot_description_kinematics.arm.kinematics_solver_search_resolution"
        ]
        == 0.005
    )
    assert (
        parameters["robot_description_kinematics.arm.kinematics_solver_timeout"] == 0.1
    )


def test_operational_motion_receives_home_from_single_source_of_truth():
    home = yaml.safe_load(HOME_PATH.read_text())["home_joint_positions"]
    motion = next(
        node
        for node in _composed_nodes()
        if isinstance(node, Node) and node.node_executable == "motion_node"
    )
    (parameters,) = evaluate_parameters(LaunchContext(), motion._Node__parameters)

    assert list(parameters["home_joint_positions"]) == home
    assert (
        "home_joint_positions" not in yaml.safe_load(PROFILE_PATH.read_text())["motion"]
    )
