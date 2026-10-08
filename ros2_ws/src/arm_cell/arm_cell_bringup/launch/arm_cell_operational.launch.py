"""Compose the supported software-only ARM Cell operational profile."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def profile_parameters(config, section):
    """Return the exact profile values passed to a composed runtime node."""
    parameters = {
        "use_sim_time": config.get("use_sim_time", True),
        **config.get(section, {}),
    }
    # launch_ros cannot encode an empty YAML sequence from a parameter dict;
    # omitting it preserves the node's declared empty-vector default.
    return {key: value for key, value in parameters.items() if value != []}


PROJECT_DEBUG_LOGGERS = {
    "arm_cell_integration": ("material_handoff_node",),
    "arm_cell_safety": ("safety_node",),
    "arm_cell_vision": ("detect_target_node",),
    "arm_cell_motion_moveit2": (
        "motion_node",
        "isaac_motion_backend",
        "motion_core",
        "motion_action_server",
        "task_executor",
    ),
    "arm_cell_bt": ("orchestration_node",),
}


def node_ros_arguments(package, log_level, action_logger_level="warn"):
    """Keep the process default at INFO and opt in owned diagnostics by logger."""
    arguments = ["--log-level", "info"]
    if log_level == "debug":
        for logger_name in PROJECT_DEBUG_LOGGERS.get(package, ()):
            arguments.extend(["--log-level", f"{logger_name}:=debug"])
    action_logger = {
        "arm_cell_motion_moveit2": "isaac_motion_backend.rclcpp_action",
        "arm_cell_bt": "orchestration_node.rclcpp_action",
    }.get(package)
    if action_logger:
        arguments.extend(["--log-level", f"{action_logger}:={action_logger_level}"])
    return arguments


def compose(context):
    selected_log_level = LaunchConfiguration("log_level").perform(context)
    selected_action_logger_level = LaunchConfiguration("action_logger_level").perform(
        context
    )
    profile_path = Path(LaunchConfiguration("profile").perform(context))
    config = yaml.safe_load(profile_path.read_text()) or {}
    share = Path(get_package_share_directory("arm_cell_bringup"))
    home_config = (
        yaml.safe_load((share / "config/home_joint_positions.yaml").read_text()) or {}
    )
    home_joint_positions = home_config["home_joint_positions"]
    if len(home_joint_positions) != 6:
        raise RuntimeError("home_joint_positions must contain six arm joints")
    use_sim_time = config.get("use_sim_time", True)
    description = Path(get_package_share_directory("arm_cell_description"))
    moveit_share = Path(get_package_share_directory("arm_cell_moveit_config"))
    robot_description = xacro.process_file(
        str(description / "urdf/m0609_robotiq_2f85.urdf.xacro")
    ).toxml()
    robot_description_semantic = (
        moveit_share / "config/m0609_robotiq_2f85.srdf"
    ).read_text()
    kinematics = yaml.safe_load((moveit_share / "config/kinematics.yaml").read_text())
    kinematics_parameters = {
        f"robot_description_kinematics.arm.{key}": value
        for key, value in kinematics["arm"].items()
    }
    readiness = config.get("readiness", {})
    mission_target_id = config["orchestration"]["mission_target_id"]
    detector_profile = config["vision"]["detector_profiles"][mission_target_id]
    observation_reference = detector_profile.get("observation_reference")
    if not isinstance(observation_reference, str) or not observation_reference:
        raise RuntimeError(
            "mission detector profile must declare its observation_reference"
        )
    include_simulation = (
        LaunchConfiguration("include_simulation").perform(context).lower() == "true"
    )
    if not readiness.get("fail_closed_until_inputs_fresh", False):
        raise RuntimeError("operational profile must keep Safety fail-closed")
    if readiness.get("required_motion_backend", False) and not include_simulation:
        raise RuntimeError(
            "mission-capable operational profile requires the external Isaac/MoveIt composition"
        )

    actions = [
        Node(
            package="arm_cell_vda",
            executable="external_state_mock_node",
            name="external_state_mock",
            output="screen",
            ros_arguments=node_ros_arguments("arm_cell_vda", selected_log_level),
            parameters=[profile_parameters(config, "vda")],
        ),
        Node(
            package="arm_cell_integration",
            executable="material_handoff_node",
            name="material_handoff_node",
            output="screen",
            ros_arguments=node_ros_arguments(
                "arm_cell_integration", selected_log_level
            ),
            parameters=[profile_parameters(config, "material_handoff")],
        ),
        Node(
            package="arm_cell_safety",
            executable="safety_node",
            name="safety_node",
            output="screen",
            ros_arguments=node_ros_arguments("arm_cell_safety", selected_log_level),
            parameters=[
                profile_parameters(
                    {"use_sim_time": use_sim_time, "safety": config["safety"]}, "safety"
                )
            ],
        ),
        Node(
            package="arm_cell_vision",
            executable="detect_target_node",
            name="detect_target_node",
            output="screen",
            ros_arguments=node_ros_arguments("arm_cell_vision", selected_log_level),
            parameters=[profile_parameters(config, "vision")],
        ),
        Node(
            package="arm_cell_motion_moveit2",
            executable="motion_node",
            name="motion_node",
            output="screen",
            ros_arguments=node_ros_arguments(
                "arm_cell_motion_moveit2",
                selected_log_level,
                selected_action_logger_level,
            ),
            parameters=[
                {
                    **profile_parameters(config, "motion"),
                    "observation_reference": observation_reference,
                    "home_joint_positions": home_joint_positions,
                    "robot_description": robot_description,
                    "robot_description_semantic": robot_description_semantic,
                    **kinematics_parameters,
                }
            ],
        ),
        Node(
            package="arm_cell_bt",
            executable="orchestration_node",
            name="orchestration_node",
            output="screen",
            ros_arguments=node_ros_arguments(
                "arm_cell_bt", selected_log_level, selected_action_logger_level
            ),
            parameters=[
                {
                    **profile_parameters(config, "orchestration"),
                    "recipe_directory": str(
                        share
                        / "config"
                        / config["orchestration"].get("recipe_directory", "recipes")
                    ),
                }
            ],
        ),
    ]

    if include_simulation:
        planning_profile = config.get("simulation", {}).get("planning_profile")
        if not planning_profile:
            raise RuntimeError("operational profile has no simulation planning profile")
        planning_path = Path(planning_profile)
        if not planning_path.is_absolute():
            planning_path = share / "config" / planning_path
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch/isaac_planning.launch.py")
                ),
                launch_arguments={
                    "profile": str(planning_path),
                    "static_scene": str(
                        config.get("simulation", {}).get("static_scene", False)
                    ).lower(),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch/isaac_camera.launch.py")
                )
            )
        )

    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("arm_cell_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="0"),
            ),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
            ),
            DeclareLaunchArgument(
                "profile", default_value=str(share / "config/operational_profile.yaml")
            ),
            DeclareLaunchArgument("include_simulation", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("action_logger_level", default_value="warn"),
            OpaqueFunction(function=compose),
        ]
    )
