"""MoveIt robot-only planning; simulator and TF composition belong to bringup."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def _load_console_output_format():
    helper_path = (
        Path(get_package_share_directory("arm_cell_moveit_config"))
        / "launch/console_output_format.py"
    )
    spec = importlib.util.spec_from_file_location(
        "arm_cell_console_output_format", helper_path
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load console formatter from {helper_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.ConsoleOutputFormat("{line}")


CONSOLE_OUTPUT_FORMAT = _load_console_output_format()


# Keep the spdlog process sink at DEBUG without enabling framework chatter.
DEBUG_FRAMEWORK_LOGGERS = (
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
)
MOVEIT_PROGRESS_LOGGERS = (
    "moveit_move_group_default_capabilities.move_action_capability",
    "moveit.ompl_planning.model_based_planning_context",
)
MOVEIT_RAW_LOG_LEVELS = (
    "ompl:=fatal",
    "moveit_kinematic_constraints.kinematic_constraints:=warn",
    "moveit_collision_detection_fcl.collision_common:=warn",
)


def node_ros_arguments(log_level="info"):
    """Keep console and file levels aligned while filtering framework chatter."""
    arguments = ["--log-level", log_level]
    if log_level == "debug":
        for logger_name in DEBUG_FRAMEWORK_LOGGERS:
            arguments.extend(["--log-level", f"{logger_name}:=warn"])
    for logger_name in MOVEIT_PROGRESS_LOGGERS:
        arguments.extend(["--log-level", f"{logger_name}:=warn"])
    for logger_setting in MOVEIT_RAW_LOG_LEVELS:
        arguments.extend(["--log-level", logger_setting])
    return arguments


def compose(context):
    ros_arguments = node_ros_arguments(
        LaunchConfiguration("log_level").perform(context)
    )
    share = Path(get_package_share_directory("arm_cell_moveit_config"))
    description = Path(get_package_share_directory("arm_cell_description"))

    def config(name):
        return yaml.safe_load((share / "config" / name).read_text())

    planning_limits = config("joint_limits.yaml")
    planning_pipeline = config("ompl_planning.yaml")
    planning_profile = LaunchConfiguration("planning_profile").perform(context)
    if planning_profile:
        profile = yaml.safe_load(Path(planning_profile).read_text()) or {}
        acceleration = profile.get("motion", {}).get("planning_max_acceleration_rad_s2")
        if acceleration is not None:
            acceleration = float(acceleration)
            if acceleration <= 0.0:
                raise RuntimeError("planning acceleration seed must be positive")
            planning_limits.setdefault("joint_limits", {})
            for joint_name in (
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ):
                planning_limits["joint_limits"][joint_name] = {
                    "has_acceleration_limits": True,
                    "max_acceleration": acceleration,
                }
            planning_pipeline["request_adapters"] = (
                planning_pipeline["request_adapters"]
                + " default_planner_request_adapters/AddTimeOptimalParameterization"
            )

    parameters = [
        {
            "use_sim_time": True,
            "robot_description": xacro.process_file(
                str(description / "urdf/m0609_robotiq_2f85.urdf.xacro")
            ).toxml(),
            "robot_description_semantic": (
                share / "config/m0609_robotiq_2f85.srdf"
            ).read_text(),
            "robot_description_kinematics": config("kinematics.yaml"),
            "robot_description_planning": planning_limits,
            "planning_pipelines": ["ompl"],
            "default_planning_pipeline": "ompl",
            "ompl": planning_pipeline,
        },
        config("planning_only.yaml"),
    ]
    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            ros_arguments=ros_arguments,
            parameters=parameters,
            remappings=[("joint_states", LaunchConfiguration("joint_states"))],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", str(share / "config/moveit.rviz")],
            ros_arguments=ros_arguments,
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "RCUTILS_CONSOLE_OUTPUT_FORMAT",
                "[{severity}] [{name}] {message}",
            ),
            SetEnvironmentVariable("TZ", "UTC"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("joint_states", default_value="/joint_states"),
            DeclareLaunchArgument("planning_profile", default_value=""),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=compose),
        ]
    )
