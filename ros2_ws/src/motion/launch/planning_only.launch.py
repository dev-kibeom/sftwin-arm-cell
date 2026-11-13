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
        Path(get_package_share_directory("arm_cell_motion"))
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
    share = Path(get_package_share_directory("arm_cell_motion"))
    robot_path = Path(LaunchConfiguration(
        'robot_profile', default=str(share / 'config/robots/m0609.yaml')).perform(context))
    robot = yaml.safe_load(robot_path.read_text())
    model_path = Path(LaunchConfiguration(
        'model_configuration', default=str(share / 'config/models/m0609_robotiq_2f85.yaml')).perform(context))
    model = yaml.safe_load(model_path.read_text())
    if model['robot_id'] != robot['robot_id']:
        raise ValueError('Selected model is incompatible with robot profile')
    policy = yaml.safe_load((share / 'config/planning.yaml').read_text())

    def resource(reference, base):
        if reference.startswith('package://'):
            package, relative = reference[10:].split('/', 1)
            return Path(get_package_share_directory(package)) / relative
        path = Path(reference).expanduser()
        return path if path.is_absolute() else (base / path).resolve()

    def selected_path(argument, reference, base):
        default = str(resource(reference, base))
        return Path(LaunchConfiguration(argument, default=default).perform(context))

    def config(name):
        return yaml.safe_load((share / 'config/moveit' / name).read_text())

    description = selected_path('robot_description_file', model['urdf'], model_path.parent)
    semantic = selected_path('robot_semantic_file', model['semantic'], model_path.parent)
    mesh_root = LaunchConfiguration('mesh_root', default=model['mesh_root']).perform(context)
    planning_limits = yaml.safe_load(selected_path('joint_limits_file', robot['joint_limits'], robot_path.parent).read_text())
    planning_pipeline = yaml.safe_load(selected_path('planning_pipeline_file', policy['planning_pipeline'], share / 'config').read_text())
    planning_profile = LaunchConfiguration("planning_profile").perform(context)
    if planning_profile:
        profile = yaml.safe_load(Path(planning_profile).read_text()) or {}
        if profile.get("kind") == "arm-cell-runtime-configuration":
            profile = profile["configuration"]
        acceleration = profile.get("motion", {}).get("planning_max_acceleration_rad_s2")
        if acceleration is not None:
            acceleration = float(acceleration)
            if acceleration <= 0.0:
                raise RuntimeError("planning acceleration seed must be positive")
            planning_limits.setdefault("joint_limits", {})
            for joint_name in LaunchConfiguration('arm_joint_names', default=','.join(robot['motion']['arm_joint_names'])).perform(context).split(','):
                planning_limits["joint_limits"].setdefault(joint_name, {}).update(
                    has_acceleration_limits=True, max_acceleration=acceleration)
            planning_pipeline["request_adapters"] = (
                planning_pipeline["request_adapters"]
                + " default_planner_request_adapters/AddTimeOptimalParameterization"
            )

    parameters = [
        {
            "use_sim_time": True,
            "robot_description": xacro.process_file(
                str(description), mappings={"mesh_root": mesh_root},
            ).toxml(),
            "robot_description_semantic": semantic.read_text(),
            "robot_description_kinematics": yaml.safe_load(selected_path('kinematics_file', policy['kinematics'], share / 'config').read_text()),
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
            arguments=["-d", str(share / "config/moveit/moveit.rviz")],
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
            DeclareLaunchArgument(
                "robot_profile", default_value=str(Path(get_package_share_directory("arm_cell_motion")) / "config/robots/m0609.yaml")),
            DeclareLaunchArgument(
                "model_configuration", default_value=str(Path(get_package_share_directory("arm_cell_motion")) / "config/models/m0609_robotiq_2f85.yaml")),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("joint_states", default_value="/joint_states"),
            DeclareLaunchArgument("planning_profile", default_value=""),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=compose),
        ]
    )
