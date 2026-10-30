"""Compose ROS processes for the externally started Isaac simulation."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    for logger_setting in MOVEIT_RAW_LOG_LEVELS:
        arguments.extend(["--log-level", logger_setting])
    return arguments


def compose(context):
    selected_log_level = LaunchConfiguration("log_level").perform(context)
    ros_arguments = node_ros_arguments(selected_log_level)
    config = yaml.safe_load(
        Path(LaunchConfiguration("profile").perform(context)).read_text()
    )
    description = Path(get_package_share_directory("arm_cell_description"))
    model = {
        "robot_description": xacro.process_file(
            str(description / "urdf/m0609_robotiq_2f85.urdf.xacro")
        ).toxml(),
        "use_sim_time": True,
    }
    pose = config["world_to_base"]
    arguments = ["--frame-id", "world", "--child-frame-id", "base_link"]
    for key in ("x", "y", "z", "roll", "pitch", "yaw"):
        arguments.extend(["--" + key, str(pose[key])])
    moveit = Path(get_package_share_directory("arm_cell_moveit_config"))
    default_scene_file = str(moveit / "config/m0609_static_scene.json")
    static_loader = Node(
        package="arm_cell_moveit_config",
        executable="load_static_scene.py",
        output="screen",
        output_format=CONSOLE_OUTPUT_FORMAT,
        ros_arguments=ros_arguments,
        parameters=[
            {
                "use_sim_time": True,
                "scene_file": str(
                    LaunchConfiguration(
                        "scene_file", default=default_scene_file
                    ).perform(context)
                ),
                "verify_nominal_state": LaunchConfiguration(
                    "verify_nominal_state", default="false"
                )
                .perform(context)
                .lower()
                == "true",
            }
        ],
    )
    actions = [
        Node(
            package="arm_cell_sim_adapter",
            executable="joint_state_adapter",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            ros_arguments=ros_arguments,
            parameters=[model],
            remappings=[
                ("raw_joint_states", config["raw_joint_states"]),
                ("joint_states", config["joint_states"]),
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            parameters=[model],
            ros_arguments=ros_arguments,
            remappings=[("joint_states", config["joint_states"])],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_base",
            output="screen",
            output_format=CONSOLE_OUTPUT_FORMAT,
            arguments=arguments,
            parameters=[{"use_sim_time": True}],
            ros_arguments=ros_arguments,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit / "launch/planning_only.launch.py")
            ),
            launch_arguments={
                "use_rviz": LaunchConfiguration("use_rviz"),
                "joint_states": config["joint_states"],
                "planning_profile": LaunchConfiguration("planning_profile", default=""),
                "log_level": LaunchConfiguration("log_level"),
            }.items(),
        ),
    ]
    if (
        LaunchConfiguration("static_scene", default="false").perform(context).lower()
        == "true"
    ):
        actions.extend(
            [
                static_loader,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=static_loader,
                        on_exit=[
                            EmitEvent(
                                event=Shutdown(reason="static scene loader exited")
                            )
                        ],
                    )
                ),
            ]
        )
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("arm_cell_bringup"))
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "RCUTILS_CONSOLE_OUTPUT_FORMAT",
                "[{severity}] [{name}] {message}",
            ),
            SetEnvironmentVariable("TZ", "UTC"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("static_scene", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("verify_nominal_state", default_value="false"),
            DeclareLaunchArgument(
                "scene_file",
                default_value=str(
                    Path(get_package_share_directory("arm_cell_moveit_config"))
                    / "config/m0609_static_scene.json"
                ),
            ),
            DeclareLaunchArgument(
                "profile", default_value=str(share / "config/isaac_planning.yaml")
            ),
            DeclareLaunchArgument("planning_profile", default_value=""),
            OpaqueFunction(function=compose),
        ]
    )
