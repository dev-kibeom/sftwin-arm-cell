"""Static checks for the supported ARM Cell operational profile."""

import importlib.util
import json
import os
from pathlib import Path
from xml.etree import ElementTree

import yaml
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, Substitution
from launch.actions import IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
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
    source_packages = {
        "arm_cell_bringup": PACKAGE_ROOT,
        "arm_cell_description": PACKAGE_ROOT.parent / "arm_cell_description",
        "arm_cell_moveit_config": (PACKAGE_ROOT.parent / "arm_cell_moveit_config"),
    }
    module.get_package_share_directory = lambda package: str(
        source_packages[package]
        if package in source_packages
        else get_package_share_directory(package)
    )
    return module


def _composed_nodes(
    log_level="info", diagnostics=None, module=None, planning_scene_log_level="info"
):
    module = module or _load_launch_module()
    context = LaunchContext()
    context.launch_configurations["profile"] = str(PROFILE_PATH)
    context.launch_configurations["include_simulation"] = "true"
    context.launch_configurations["log_level"] = log_level
    context.launch_configurations["action_logger_level"] = "warn"
    context.launch_configurations["planning_scene_log_level"] = planning_scene_log_level
    if diagnostics is not None:
        context.launch_configurations["diagnostics"] = diagnostics
    return module.compose(context)


def _project_nodes(actions):
    return {
        (
            "motion_node"
            if action.node_package == "arm_cell_task_execution"
            else action.node_executable
        ): action
        for action in actions
        if isinstance(action, Node)
    }


def _node_parameters(node):
    return evaluate_parameters(LaunchContext(), node._Node__parameters)[0]


def _node_ros_arguments(node):
    return list(node._Node__ros_arguments)


def test_profile_declares_reviewed_runtime_nodes_and_readiness_config():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    nodes = [action for action in _composed_nodes() if isinstance(action, Node)]

    project_nodes = {(node.node_package, node.node_executable) for node in nodes}
    assert project_nodes == {
        ("arm_cell_vda", "external_state_mock_node"),
        ("arm_cell_gateway", "material_handoff_node"),
        ("arm_cell_safety", "safety_node"),
        ("arm_cell_vision", "detect_target_node"),
        ("arm_cell_task_execution", "task_execution_node"),
        ("arm_cell_orchestration", "orchestration_node"),
    }
    assert all(node.node_package != "image_transport" for node in nodes)
    assert all(node._ExecuteLocal__output_format == "{line}" for node in nodes)
    for package_name, _ in {
        (node.node_package, node.node_executable) for node in nodes
    }:
        _assert_installed_package_identity(package_name)
    assert profile["readiness"]["fail_closed_until_inputs_fresh"] is True
    assert profile["readiness"]["required_motion_backend"] is True


def test_pick_ik_profile_reaches_motion_without_changing_cartesian_solver():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    motion = profile["motion"]
    nodes = _project_nodes(_composed_nodes())
    parameters = _node_parameters(nodes["motion_node"])
    configured = {key: parameters[key] for key in motion}
    assert parameters["place_waypoint_trajectory_file"].endswith(
        motion["place_waypoint_trajectory_file"]
    )
    configured.pop("place_waypoint_trajectory_file")
    expected_motion = dict(motion)
    expected_motion.pop("place_waypoint_trajectory_file")
    assert {
        key: list(value) if isinstance(value, tuple) else value
        for key, value in configured.items()
    } == expected_motion
    kinematics = yaml.safe_load(
        (
            PACKAGE_ROOT.parent / "arm_cell_moveit_config/config/kinematics.yaml"
        ).read_text()
    )
    assert (
        kinematics["arm"]["kinematics_solver"]
        == "kdl_kinematics_plugin/KDLKinematicsPlugin"
    )
    assert not kinematics["arm"].get("position_only_ik", False)


def test_launch_uses_utc_and_preserves_ros_console_fields():
    actions = _composed_nodes()
    environment = [
        action for action in actions if isinstance(action, SetEnvironmentVariable)
    ]
    context = LaunchContext()
    output_format = next(
        action
        for action in environment
        if action.name[0].perform(context) == "RCUTILS_CONSOLE_OUTPUT_FORMAT"
    )
    assert output_format.value[0].perform(context) == "[{severity}] [{name}] {message}"
    timezone = next(
        action for action in environment if action.name[0].perform(context) == "TZ"
    )
    assert timezone.value[0].perform(context) == "UTC"


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


def test_debug_raises_process_default_for_console_and_file_log_parity():
    module = _load_launch_module()
    expected_debug_loggers = {
        "arm_cell_gateway": {
            "material_handoff_node",
        },
        "arm_cell_safety": {"safety_node"},
        "arm_cell_vision": {"detect_target_node"},
        "arm_cell_motion": {
            "motion_node",
            "isaac_motion_backend",
            "motion_core",
            "motion_action_server",
            "task_executor",
        },
        "arm_cell_orchestration": {"orchestration_node"},
        "arm_cell_vda": set(),
    }
    for package_name, expected in expected_debug_loggers.items():
        info_arguments = module.node_ros_arguments(package_name, "info")
        assert info_arguments[:2] == ["--log-level", "info"]
        debug_arguments = module.node_ros_arguments(package_name, "debug")
        assert debug_arguments[:2] == ["--log-level", "debug"]
        for noisy_logger in (
            "rcl",
            "rclcpp",
            "rcl_action",
            "rclcpp_action",
            "rmw",
            "rmw_fastrtps_cpp",
            "move_group",
            "moveit",
            "moveit_ros",
            "moveit_core",
            "pluginlib",
            "kdl_parser",
            "kinematics_plugin_loader",
            "robot_state_publisher",
            "moveit_robot_model",
            "moveit_kinematics_base",
            "moveit_constraint_samplers",
        ):
            assert f"{noisy_logger}:=warn" in debug_arguments
        configured_loggers = {
            argument.removesuffix(":=debug")
            for argument in debug_arguments
            if argument.endswith(":=debug")
        }
        assert configured_loggers == expected
        if package_name == "arm_cell_motion":
            for arguments in (info_arguments, debug_arguments):
                assert arguments.count("ompl:=fatal") == 1
                assert "ompl:=warn" not in arguments
            for logger_setting in (
                "moveit_kinematic_constraints.kinematic_constraints:=" "warn",
                "moveit_collision_detection_fcl.collision_common:=" "warn",
            ):
                assert logger_setting in info_arguments
                assert logger_setting in debug_arguments
            assert "move_group_interface:=warn" in info_arguments
            assert "move_group_interface:=warn" in debug_arguments
            assert "moveit_planning_scene.planning_scene:=info" in info_arguments
            assert "moveit_planning_scene.planning_scene:=info" in debug_arguments
        else:
            assert not any(
                argument.startswith("move_group_interface:=")
                for argument in info_arguments + debug_arguments
            )
        action_loggers = {
            "arm_cell_motion": (
                "isaac_motion_backend.rclcpp_action",
                "motion_node.rclcpp_action",
            ),
            "arm_cell_orchestration": ("orchestration_node.rclcpp_action",),
        }
        if package_name in action_loggers:
            for logger in action_loggers[package_name]:
                assert f"{logger}:=warn" in info_arguments
                assert f"{logger}:=warn" in debug_arguments
                user_debug_arguments = module.node_ros_arguments(
                    package_name, "debug", "debug"
                )
                assert f"{logger}:=debug" in user_debug_arguments
        elif package_name != "arm_cell_motion":
            assert info_arguments == ["--log-level", "info"]
    for framework_package in (
        "rcl",
        "rclcpp",
        "rmw",
        "tf2",
        "robot_state_publisher",
        "move_group",
    ):
        assert module.node_ros_arguments(framework_package, "debug")[:2] == [
            "--log-level",
            "debug",
        ]

    planning_source = Path(__file__).parents[1] / "launch/isaac_planning.launch.py"
    spec = importlib.util.spec_from_file_location(
        "isaac_planning_launch", planning_source
    )
    planning_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(planning_module)
    planning_arguments = planning_module.node_ros_arguments()
    assert planning_arguments[:2] == ["--log-level", "info"]
    assert (
        "moveit_kinematic_constraints.kinematic_constraints:=warn" in planning_arguments
    )
    assert "moveit_collision_detection_fcl.collision_common:=warn" in planning_arguments


def test_diagnostics_selection_never_changes_ros_logger_levels():
    selected = _project_nodes(
        _composed_nodes("info", "motion,vision,safety,orchestration")
    )
    none = _project_nodes(_composed_nodes("info", "none"))
    for executable in selected:
        assert _node_ros_arguments(selected[executable]) == _node_ros_arguments(
            none[executable]
        )


def test_moveit_raw_logs_are_filtered_at_their_source_logger(monkeypatch):
    package_roots = {
        "arm_cell_bringup": PACKAGE_ROOT,
        "arm_cell_moveit_config": PACKAGE_ROOT.parent / "arm_cell_moveit_config",
    }
    monkeypatch.setattr(
        "ament_index_python.packages.get_package_share_directory",
        lambda package: str(package_roots[package]),
    )
    module = _load_launch_module()
    expected = {
        "moveit_kinematic_constraints.kinematic_constraints:=" "warn",
        "moveit_collision_detection_fcl.collision_common:=" "warn",
        "ompl:=fatal",
    }
    for log_level in ("info", "debug"):
        arguments = module.node_ros_arguments("arm_cell_motion", log_level)
        assert expected.issubset(set(arguments))
        motion = _project_nodes(_composed_nodes(log_level=log_level, module=module))[
            "motion_node"
        ]
        assert "ompl:=fatal" in _node_ros_arguments(motion)


def test_planning_scene_debug_is_suppressed_selectively_and_can_be_restored():
    module = _load_launch_module()
    default_arguments = module.node_ros_arguments("arm_cell_motion", "debug")
    diagnostic_arguments = module.node_ros_arguments(
        "arm_cell_motion", "debug", "warn", "debug"
    )

    assert "moveit_planning_scene.planning_scene:=info" in default_arguments
    assert "moveit_planning_scene.planning_scene:=debug" in diagnostic_arguments
    for arguments in (default_arguments, diagnostic_arguments):
        assert "motion_node:=debug" in arguments
        assert "motion_core:=debug" in arguments
        assert "task_executor:=debug" in arguments
        assert "moveit_planning_scene:=warn" not in arguments
        assert "moveit:=warn" in arguments

    motion = _project_nodes(
        _composed_nodes(log_level="debug", planning_scene_log_level="debug")
    )["motion_node"]
    assert "moveit_planning_scene.planning_scene:=debug" in _node_ros_arguments(motion)


def test_log_level_does_not_change_optional_capture_parameters():
    info_nodes = _project_nodes(_composed_nodes("info", "none"))
    debug_nodes = _project_nodes(_composed_nodes("debug", "none"))
    selected_nodes = _project_nodes(_composed_nodes("info", "motion,vision"))

    for executable in info_nodes:
        assert _node_parameters(info_nodes[executable]) == _node_parameters(
            debug_nodes[executable]
        )
    assert (
        _node_parameters(selected_nodes["motion_node"])["diagnostics.capture_enabled"]
        is True
    )
    assert (
        _node_parameters(selected_nodes["detect_target_node"])[
            "diagnostics.capture_enabled"
        ]
        is True
    )
    selected_safety = _project_nodes(_composed_nodes("info", "safety"))["safety_node"]
    selected_orchestration = _project_nodes(_composed_nodes("info", "orchestration"))[
        "orchestration_node"
    ]
    assert _node_parameters(selected_safety)["diagnostics.capture_enabled"] is True
    assert (
        _node_parameters(selected_orchestration)["diagnostics.capture_enabled"] is True
    )
    assert (
        _node_parameters(info_nodes["motion_node"])["diagnostics.capture_enabled"]
        is False
    )
    assert (
        _node_parameters(info_nodes["detect_target_node"])[
            "diagnostics.capture_enabled"
        ]
        is False
    )
    assert (
        _node_parameters(info_nodes["safety_node"])["diagnostics.capture_enabled"]
        is False
    )
    assert (
        _node_parameters(info_nodes["orchestration_node"])[
            "diagnostics.capture_enabled"
        ]
        is False
    )
    assert "motion_node:=debug" in _node_ros_arguments(debug_nodes["motion_node"])


def test_diagnostics_parser_selects_supported_components():
    module = _load_launch_module()

    assert module.parse_diagnostics("none") == frozenset()
    assert module.parse_diagnostics("all") == {
        "motion",
        "vision",
        "safety",
        "orchestration",
    }
    assert module.parse_diagnostics("motion, vision,safety") == {
        "motion",
        "vision",
        "safety",
    }


def test_diagnostics_parser_rejects_unknown_empty_duplicate_and_mixed_all():
    import pytest

    module = _load_launch_module()
    for value in (
        "camera",
        "clock",
        "gripper",
        "vision,",
        ",vision",
        "vision,,motion",
        "vision,vision",
        "all,vision",
        "none,vision",
        "NONE",
    ):
        with pytest.raises(RuntimeError, match="diagnostics"):
            module.parse_diagnostics(value)


def test_diagnostics_parser_explains_that_none_and_all_must_stand_alone():
    import pytest

    module = _load_launch_module()
    for value in ("all,vision", "none,motion"):
        with pytest.raises(RuntimeError, match="must be used alone"):
            module.parse_diagnostics(value)


def test_installed_launch_resolves_checkout_artifacts_from_any_working_directory(
    tmp_path, monkeypatch
):
    module = _load_launch_module()
    checkout = tmp_path / "checkout"
    (checkout / "ros2_ws/src").mkdir(parents=True)
    (checkout / "infra/isaac_sim/scripts/m0609_cell").mkdir(parents=True)
    installed_launch = (
        checkout
        / "ros2_ws/install/arm_cell_bringup/share/arm_cell_bringup/launch"
        / "arm_cell_operational.launch.py"
    )
    installed_launch.parent.mkdir(parents=True)
    installed_launch.touch()
    outside = tmp_path / "outside"
    outside.mkdir()
    monkeypatch.chdir(outside)
    monkeypatch.delenv("SFTWIN_PROJECT_ROOT", raising=False)
    module.__file__ = str(installed_launch)

    assert module.project_artifacts_directory() == str(checkout / ".local_artifacts")


def test_info_vision_diagnostics_enables_capture_without_debug_logging():
    module = _load_launch_module()
    module.join_live_test_session = lambda _artifact_directory: None
    actions = _composed_nodes(log_level="info", diagnostics="vision", module=module)
    nodes = _project_nodes(actions)
    vision = nodes["detect_target_node"]

    assert _node_parameters(vision)["diagnostics.capture_enabled"] is True
    assert _node_parameters(vision)["diagnostics.snapshot_run_directory"] == ""
    assert _node_parameters(vision)["diagnostics.live_test_session_id"] == ""
    clock_directory = Path(
        _node_parameters(nodes["motion_node"])["diagnostics.output_directory"]
    )
    assert clock_directory.is_absolute()
    assert clock_directory.parts[-2:] == (".local_artifacts", "observability")
    assert module.node_ros_arguments("arm_cell_vision", "info") == [
        "--log-level",
        "info",
    ]


def test_omitted_diagnostics_defaults_to_all_only_for_debug_logging():
    info_actions = _composed_nodes(log_level="info")
    debug_actions = _composed_nodes(log_level="debug")
    info_nodes = _project_nodes(info_actions)
    debug_nodes = _project_nodes(debug_actions)

    for executable in (
        "motion_node",
        "detect_target_node",
        "safety_node",
        "orchestration_node",
    ):
        assert (
            _node_parameters(info_nodes[executable])["diagnostics.capture_enabled"]
            is False
        )
        assert (
            _node_parameters(debug_nodes[executable])["diagnostics.capture_enabled"]
            is True
        )

    info_summary = next(
        action for action in info_actions if isinstance(action, LogInfo)
    )
    debug_summary = next(
        action for action in debug_actions if isinstance(action, LogInfo)
    )
    assert "diagnostics=none" in "".join(
        part.text for part in info_summary._LogInfo__msg
    )
    assert "diagnostics=motion,orchestration,safety,vision" in "".join(
        part.text for part in debug_summary._LogInfo__msg
    )


def test_component_diagnostics_selects_only_requested_component_producers():
    actions = _composed_nodes(log_level="info", diagnostics="motion")
    nodes = _project_nodes(actions)
    debug_nodes = _project_nodes(
        _composed_nodes(log_level="debug", diagnostics="motion")
    )

    assert (
        _node_parameters(nodes["detect_target_node"])["diagnostics.capture_enabled"]
        is False
    )
    assert _node_parameters(nodes["motion_node"])["diagnostics.capture_enabled"] is True
    assert (
        _node_parameters(debug_nodes["motion_node"])["diagnostics.capture_enabled"]
        is True
    )
    assert (
        _node_parameters(debug_nodes["detect_target_node"])[
            "diagnostics.capture_enabled"
        ]
        is False
    )
    selection_summary = next(
        action for action in actions if isinstance(action, LogInfo)
    )
    summary_text = "".join(part.text for part in selection_summary._LogInfo__msg)
    assert "MoveIt Planning Scene logger=info" in summary_text
    assert "diagnostics=motion" in summary_text
    assert "Motion profile trace capture=enabled" in summary_text
    assert "Isaac Gripper baseline capture is always enabled" in summary_text
    assert "ROS session association=unassociated" in summary_text
    motion_arguments = _node_ros_arguments(nodes["motion_node"])
    assert "motion_node:=debug" not in motion_arguments
    assert "motion_core:=debug" not in motion_arguments
    assert "move_group_interface:=warn" in motion_arguments
    assert "isaac_motion_backend.rclcpp_action:=warn" in motion_arguments
    assert "motion_node.rclcpp_action:=warn" in motion_arguments
    orchestration_nodes = _project_nodes(
        _composed_nodes(log_level="info", diagnostics="orchestration")
    )
    orchestration_arguments = _node_ros_arguments(
        orchestration_nodes["orchestration_node"]
    )
    assert orchestration_arguments == [
        "--log-level",
        "info",
        "--log-level",
        "orchestration_node.rclcpp_action:=warn",
    ]
    assert (
        _node_parameters(orchestration_nodes["orchestration_node"])[
            "diagnostics.capture_enabled"
        ]
        is True
    )
    assert "orchestration_node:=debug" not in orchestration_arguments


def test_all_diagnostics_enables_every_runtime_producer():
    actions = _composed_nodes(log_level="info", diagnostics="all")
    nodes = _project_nodes(actions)

    assert (
        _node_parameters(nodes["detect_target_node"])["diagnostics.capture_enabled"]
        is True
    )
    assert _node_parameters(nodes["motion_node"])["diagnostics.capture_enabled"] is True
    assert "safety_node:=debug" not in _node_ros_arguments(nodes["safety_node"])
    assert "orchestration_node:=debug" not in _node_ros_arguments(
        nodes["orchestration_node"]
    )

    debug_all = _project_nodes(_composed_nodes(log_level="debug", diagnostics="all"))
    for executable in (
        "detect_target_node",
        "motion_node",
        "safety_node",
        "orchestration_node",
    ):
        assert (
            _node_parameters(debug_all[executable])["diagnostics.capture_enabled"]
            is True
        )
    for executable in ("motion_node", "safety_node", "orchestration_node"):
        assert _node_ros_arguments(debug_all[executable]) == _node_ros_arguments(
            _project_nodes(_composed_nodes(log_level="debug", diagnostics="none"))[
                executable
            ]
        )


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
        "amr_tray_marker_post",
    }.issubset(objects)
    assert objects["amr_tray"]["geometry"]["dimensions_m"] == [0.6336, 0.4416, 0.045]
    for rail_id in (
        "amr_tray_rail_left",
        "amr_tray_rail_right",
        "amr_tray_rail_front",
        "amr_tray_rail_rear",
    ):
        assert objects[rail_id]["geometry"]["dimensions_m"][2] == 0.08
    assert not {
        "amr_raw_slot_support",
        "amr_raw_slot_marker_guard",
        "amr_finished_slot_support",
    }.intersection(objects)
    assert "required_vda_heartbeat" not in profile["readiness"]
    assert "transfer_duration_ms" not in profile["material_handoff"]
    assert not any(key.startswith("degraded_") for key in profile["safety"])


def test_vision_node_parameters_include_the_shared_rawpart_definition():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    module = _load_launch_module()
    parameters = module.profile_parameters(profile, "vision")

    assert parameters["material_definitions"]["RawPart"] == {
        "material_id": "RawPart",
        "shape": "box",
        "nominal_dimensions_m": [0.07, 0.07, 0.08],
    }


def test_vision_calibration_path_uses_bringup_profile_path_convention():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    module = _load_launch_module()
    parameters = module.vision_profile_parameters(profile, PROFILE_PATH)

    selected_file = (PACKAGE_ROOT / profile["vision"]["calibration_file"]).resolve()
    assert parameters["calibration_file"] == str(selected_file)
    assert selected_file.is_file()


def test_operational_profile_uses_w06_place_destination():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    rawpart = profile["vision"]["detector_profiles"]["RawPart"]
    mission = profile["orchestration"]
    assert mission["mission_target_id"] == rawpart["target_id"]
    recipe = json.loads((PACKAGE_ROOT / "config/recipes/RawPart.json").read_text())
    assert recipe["target_id"] == mission["mission_target_id"]
    nominal_dimensions = profile["material_definitions"]["RawPart"][
        "nominal_dimensions_m"
    ]
    expected_width_mm = nominal_dimensions[0] * 1000.0
    assert recipe["grasp"]["min_expected_contact_width_mm"] <= expected_width_mm
    assert recipe["grasp"]["max_expected_contact_width_mm"] >= expected_width_mm
    assert recipe["grasp"]["unobserved_yaw_policy"] == "forbidden"
    assert rawpart["material_id"] == "RawPart"
    assert "dimensions_m" not in rawpart
    assert "shape" not in rawpart
    observation_offset = profile["motion"]["observation_to_object_translation"]
    assert observation_offset[:2] == [0.0, 0.0]
    assert observation_offset[2] == -nominal_dimensions[2] / 2.0
    pick_approach_distance = recipe["pick"]["approach_distance_m"]
    rawpart_top_surface_offset = nominal_dimensions[2] / 2.0
    assert recipe["pick"]["approach_axis"] == "tcp_insertion_axis"
    assert profile["motion"]["insertion_axis_tcp"] == [0.0, 0.0, 1.0]
    assert recipe["place"]["approach"]["direction_object"] == [0.0, 0.0, 1.0]
    assert pick_approach_distance - rawpart_top_surface_offset >= 0.10
    assert recipe["place"]["desired_object_pose"]["frame"] == "base_link"
    assert recipe["place"]["desired_object_pose"]["position"] == [0.50, 0.25, 0.3675]
    assert recipe["place"]["position_tolerance_m"] > 0.0
    assert recipe["place"]["orientation_constraint"] == "bounded"
    assert "tool_orientation" not in recipe["place"]
    assert recipe["place"]["orientation_tolerance_rad"] > 0.0
    assert recipe["place"]["desired_object_pose"]["orientation_xyzw"] == [
        0.7071067811865476,
        0.0,
        0.0,
        0.7071067811865476,
    ]


def test_place_approach_geometry_is_recipe_owned_and_not_injected_into_motion():
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    motion = next(
        action
        for action in _composed_nodes()
        if isinstance(action, Node) and action.node_executable == "task_execution_node"
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
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    module = _load_launch_module()
    assert module.profile_parameters(profile, "motion")["backend_mode"] == "isaac"
    assert "backend_available" not in profile["motion"]
    assert (
        module.profile_parameters(profile, "motion")["enable_test_backend_controls"]
        is False
    )
    vision = _node_parameters(_project_nodes(nodes)["detect_target_node"])
    for key in ("rgb_topic", "depth_topic", "camera_info_topic"):
        assert vision[key] == profile["vision"][key]


def test_operational_nodes_have_launch_encodable_parameters():
    context = LaunchContext()
    for node in [action for action in _composed_nodes() if isinstance(action, Node)]:
        evaluate_parameters(context, node._Node__parameters)


def test_operational_profile_runs_one_provider_neutral_gateway():
    module = _load_launch_module()
    gateways = [
        node
        for node in _composed_nodes()
        if isinstance(node, Node) and node.node_package == "arm_cell_gateway"
    ]
    assert len(gateways) == 1
    assert gateways[0].node_executable == "material_handoff_node"
    (parameters,) = evaluate_parameters(LaunchContext(), gateways[0]._Node__parameters)
    assert "material_source" not in parameters


def test_motion_backend_receives_canonical_arm_kinematics_parameters():
    motion = next(
        node
        for node in _composed_nodes()
        if isinstance(node, Node) and node.node_executable == "task_execution_node"
    )
    (parameters,) = evaluate_parameters(LaunchContext(), motion._Node__parameters)
    kinematics = yaml.safe_load(
        (
            PACKAGE_ROOT.parent / "arm_cell_moveit_config/config/kinematics.yaml"
        ).read_text()
    )
    for key, value in kinematics["arm"].items():
        assert parameters[f"robot_description_kinematics.arm.{key}"] == value


def test_operational_motion_receives_home_from_single_source_of_truth():
    home = yaml.safe_load(HOME_PATH.read_text())["home_joint_positions"]
    motion = next(
        node
        for node in _composed_nodes()
        if isinstance(node, Node) and node.node_executable == "task_execution_node"
    )
    (parameters,) = evaluate_parameters(LaunchContext(), motion._Node__parameters)

    assert list(parameters["home_joint_positions"]) == home
    assert (
        "home_joint_positions" not in yaml.safe_load(PROFILE_PATH.read_text())["motion"]
    )


def test_vision_parameters_keep_cuboid_settings_without_algorithm_selection():
    module = _load_launch_module()
    profile = yaml.safe_load(PROFILE_PATH.read_text())
    parameters = module.vision_profile_parameters(profile, PROFILE_PATH)
    rawpart = parameters["detector_profiles"]["RawPart"]
    assert "algorithm" not in rawpart
    assert (
        rawpart["cuboid"] == profile["vision"]["detector_profiles"]["RawPart"]["cuboid"]
    )


def test_bringup_joins_only_a_unique_live_isaac_session(tmp_path, monkeypatch):
    monkeypatch.delenv("SFTWIN_PROJECT_ROOT", raising=False)
    module = _load_launch_module()
    artifact_root = tmp_path / ".local_artifacts"
    session_dir = artifact_root / "diagnostic_records" / "20261020" / "120000"
    session_dir.mkdir(parents=True)
    descriptor = session_dir / "session.json"
    start_token = (
        Path(f"/proc/{os.getpid()}/stat").read_text().rsplit(")", 1)[1].split()[19]
    )
    descriptor.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "session_id": "isaac-session-1",
                "status": "open",
                "participants": [
                    {
                        "process": "isaac",
                        "instance_id": f"isaac-{os.getpid()}",
                        "process_start_token": start_token,
                    }
                ],
                "artifacts": {},
            }
        )
    )
    assert module.join_live_test_session(artifact_root) == session_dir
    data = json.loads(descriptor.read_text())
    assert [p["process"] for p in data["participants"]] == ["isaac", "ros_bringup"]
    ros_participant = data["participants"][-1]
    assert ros_participant["git_provenance"]["source"] == "git worktree"
    assert ros_participant["git_provenance"]["source_root"] == str(tmp_path.resolve())
    assert ros_participant["git_provenance"]["dirty_state"] == "unavailable"
    assert (
        ros_participant["git_provenance"]["unavailable_reason"]
        == "revision_command_failed"
    )
    assert "vision_snapshots" not in data["artifacts"]
    assert data["recording_locations"]["motion_planning_details"].endswith(
        "motion/motion_planning_records/motion_planning_details"
    )
    assert data["recording_locations"]["motion_planning_summaries"].endswith(
        "motion/motion_planning_records/motion_planning_summaries"
    )
    assert "pick_execution_records" in data["recording_locations"]
    assert data["recording_locations"]["vision_diagnostic_records"] == str(
        session_dir / "vision"
    )
    module.project_artifacts_directory = lambda: str(artifact_root)
    joined_nodes = _project_nodes(_composed_nodes(module=module))
    vision_parameters = _node_parameters(joined_nodes["detect_target_node"])
    assert vision_parameters["diagnostics.live_test_session_id"] == "isaac-session-1"
    assert vision_parameters["diagnostics.snapshot_run_directory"] == str(
        artifact_root / "snapshots" / "20261020" / "120000"
    )
    motion_parameters = _node_parameters(joined_nodes["motion_node"])
    assert motion_parameters["diagnostics.snapshot_run_directory"] == str(
        artifact_root / "snapshots" / "20261020" / "120000"
    )
    assert module.vision_snapshot_run_directory(artifact_root, None) is None

    second_dir = artifact_root / "diagnostic_records" / "20261020" / "120001"
    second_dir.mkdir(parents=True)
    (second_dir / "session.json").write_text(descriptor.read_text())
    assert module.join_live_test_session(artifact_root) is None


def test_bringup_preserves_explicit_pick_execution_directory(tmp_path, monkeypatch):
    module = _load_launch_module()
    artifact_root = tmp_path / ".local_artifacts"
    session_dir = artifact_root / "diagnostic_records" / "20261020" / "120000"
    session_dir.mkdir(parents=True)
    descriptor = session_dir / "session.json"
    start_token = (
        Path(f"/proc/{os.getpid()}/stat").read_text().rsplit(")", 1)[1].split()[19]
    )
    descriptor.write_text(
        json.dumps(
            {
                "session_id": "isaac-session-2",
                "status": "open",
                "participants": [
                    {
                        "process": "isaac",
                        "instance_id": f"isaac-{os.getpid()}",
                        "process_start_token": start_token,
                    }
                ],
                "artifacts": {},
            }
        )
    )
    external = str(tmp_path / "external-pick-records")
    monkeypatch.setenv("ARM_CELL_PICK_EXECUTION_RECORD_DIR", external)
    assert module.join_live_test_session(artifact_root) == session_dir
    data = json.loads(descriptor.read_text())
    assert data["recording_locations"]["pick_execution_records"] == external


def test_bringup_rejects_stale_or_unreadable_session_descriptors(tmp_path):
    module = _load_launch_module()
    artifact_root = tmp_path / ".local_artifacts"
    session_dir = artifact_root / "diagnostic_records" / "20261020" / "120000"
    session_dir.mkdir(parents=True)
    descriptor = session_dir / "session.json"
    descriptor.write_text(
        json.dumps(
            {
                "session_id": "stale",
                "status": "open",
                "participants": [
                    {
                        "process": "isaac",
                        "instance_id": "isaac-999999999",
                        "process_start_token": "1",
                    }
                ],
            }
        )
    )
    assert module.join_live_test_session(artifact_root) is None
    descriptor.write_text("not json")
    assert module.join_live_test_session(artifact_root) is None


def test_bringup_session_write_failure_is_unassociated(tmp_path, monkeypatch):
    module = _load_launch_module()
    artifact_root = tmp_path / ".local_artifacts"
    session_dir = artifact_root / "diagnostic_records" / "20261020" / "120000"
    session_dir.mkdir(parents=True)
    descriptor = session_dir / "session.json"
    start_token = (
        Path(f"/proc/{os.getpid()}/stat").read_text().rsplit(")", 1)[1].split()[19]
    )
    descriptor.write_text(
        json.dumps(
            {
                "session_id": "isaac-session-write-failure",
                "status": "open",
                "participants": [
                    {
                        "process": "isaac",
                        "instance_id": f"isaac-{os.getpid()}",
                        "process_start_token": start_token,
                    }
                ],
                "artifacts": {},
            }
        )
    )
    monkeypatch.setattr(
        module.os,
        "replace",
        lambda *_args: (_ for _ in ()).throw(OSError("read only")),
    )

    assert module.join_live_test_session(artifact_root) is None
    assert json.loads(descriptor.read_text())["status"] == "open"
