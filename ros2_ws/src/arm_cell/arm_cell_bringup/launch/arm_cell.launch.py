"""Compose the production ARM Cell runtime on the established Isaac topology."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("arm_cell_bringup"))
    profile = LaunchConfiguration("profile")
    log_level = LaunchConfiguration("log_level")
    action_logger_level = LaunchConfiguration("action_logger_level")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value=str(share / "config/operational_profile.yaml"),
                description="Production ARM Cell runtime profile",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for project-owned production ARM Cell nodes",
            ),
            DeclareLaunchArgument(
                "action_logger_level",
                default_value="warn",
                description="ROS log level for internal rclcpp_action loggers",
            ),
            DeclareLaunchArgument(
                "ros_domain_id",
                default_value=EnvironmentVariable("ROS_DOMAIN_ID", default_value="0"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch/arm_cell_operational.launch.py")
                ),
                launch_arguments={
                    "profile": profile,
                    "include_simulation": "true",
                    "log_level": log_level,
                    "action_logger_level": action_logger_level,
                    "ros_domain_id": LaunchConfiguration("ros_domain_id"),
                }.items(),
            ),
        ]
    )
