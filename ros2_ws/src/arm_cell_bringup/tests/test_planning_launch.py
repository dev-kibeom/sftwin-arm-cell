"""Check the installed profile can compose without simulator imports."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def test_composition_has_one_base_owner():
    share = Path(get_package_share_directory("arm_cell_bringup"))
    spec = importlib.util.spec_from_file_location(
        "isaac_launch", share / "launch/isaac_planning.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations["profile"] = str(share / "config/isaac_planning.yaml")
    context.launch_configurations["use_rviz"] = "false"
    context.launch_configurations["log_level"] = "info"
    actions = module.compose(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(nodes) == 3
    packages = [node.node_package for node in nodes]
    assert packages == ["arm_cell_sim_adapter", "robot_state_publisher", "tf2_ros"]
    for node in nodes:
        parameters = node._Node__parameters
        use_sim_time = [
            value
            for parameter_set in parameters
            for name, value in parameter_set.items()
            if perform_substitutions(context, name) == "use_sim_time"
        ]
        assert use_sim_time == [True], node.node_package


def test_debug_level_reaches_planning_helpers_and_nested_moveit():
    bringup_share = Path(get_package_share_directory("arm_cell_bringup"))
    spec = importlib.util.spec_from_file_location(
        "isaac_launch_debug", bringup_share / "launch/isaac_planning.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    info_arguments = module.node_ros_arguments("info")
    assert info_arguments[:2] == ["--log-level", "info"]
    assert "moveit_kinematic_constraints.kinematic_constraints:=warn" in info_arguments
    assert "moveit_collision_detection_fcl.collision_common:=warn" in info_arguments
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "profile": str(bringup_share / "config/isaac_planning.yaml"),
            "use_rviz": "false",
            "static_scene": "false",
            "planning_profile": "",
            "log_level": "debug",
        }
    )

    actions = module.compose(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(nodes) == 3
    for node in nodes:
        assert node._Node__ros_arguments[:2] == ["--log-level", "debug"]
        assert "pluginlib:=warn" in node._Node__ros_arguments
    planning_include = next(
        action for action in actions if isinstance(action, IncludeLaunchDescription)
    )
    include_arguments = dict(
        planning_include._IncludeLaunchDescription__launch_arguments
    )
    assert include_arguments["log_level"].perform(context) == "debug"

    moveit_share = Path(get_package_share_directory("arm_cell_moveit_config"))
    planning_spec = importlib.util.spec_from_file_location(
        "planning_only_debug", moveit_share / "launch/planning_only.launch.py"
    )
    planning_module = importlib.util.module_from_spec(planning_spec)
    planning_spec.loader.exec_module(planning_module)
    planning_info_arguments = planning_module.node_ros_arguments("info")
    assert (
        "moveit_kinematic_constraints.kinematic_constraints:=warn"
        in planning_info_arguments
    )
    assert (
        "moveit_collision_detection_fcl.collision_common:=warn"
        in planning_info_arguments
    )
    assert (
        "moveit_move_group_default_capabilities.move_action_capability:=warn"
        in planning_info_arguments
    )
    assert (
        "moveit.ompl_planning.model_based_planning_context:=warn"
        in planning_info_arguments
    )
    planning_nodes = planning_module.compose(context)
    for node in planning_nodes:
        assert node._Node__ros_arguments[:2] == ["--log-level", "debug"]
        assert "ompl:=fatal" in node._Node__ros_arguments
        assert "pluginlib:=warn" in node._Node__ros_arguments
        assert (
            "moveit_move_group_default_capabilities.move_action_capability:=warn"
            in node._Node__ros_arguments
        )
        assert (
            "moveit.ompl_planning.model_based_planning_context:=warn"
            in node._Node__ros_arguments
        )


def test_planning_launches_filter_moveit_raw_detail_loggers(monkeypatch):
    package_root = Path(__file__).parents[1]
    package_roots = {
        "arm_cell_bringup": package_root,
        "arm_cell_moveit_config": package_root.parent / "arm_cell_moveit_config",
    }
    monkeypatch.setattr(
        "ament_index_python.packages.get_package_share_directory",
        lambda package: str(package_roots[package]),
    )

    launch_specs = (
        (
            "isaac_planning",
            package_root / "launch/isaac_planning.launch.py",
        ),
        (
            "planning_only",
            package_roots["arm_cell_moveit_config"] / "launch/planning_only.launch.py",
        ),
    )
    expected = {
        "moveit_kinematic_constraints.kinematic_constraints:=" "warn",
        "moveit_collision_detection_fcl.collision_common:=" "warn",
        "ompl:=fatal",
    }
    for module_name, source in launch_specs:
        spec = importlib.util.spec_from_file_location(module_name, source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for log_level in ("info", "debug"):
            arguments = module.node_ros_arguments(log_level)
            assert expected.issubset(set(arguments))
            assert "ompl:=warn" not in arguments
        context = LaunchContext()
        output_format = next(
            action.value[0].perform(context)
            for action in module.generate_launch_description().entities
            if isinstance(action, SetEnvironmentVariable)
            and action.name[0].perform(context) == "RCUTILS_CONSOLE_OUTPUT_FORMAT"
        )
        assert output_format == "[{severity}] [{name}] {message}"
