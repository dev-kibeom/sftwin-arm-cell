"""Compose the M09 mocked end-to-end ROS runtime around the PnP seam.

The Simulation UI Hub is an Isaac Kit extension and is loaded in the Isaac
process.  This launch owns the ROS composition: VDA/AMR mock, Integration
material handoff, Safety, Fixed Vision, Motion, Orchestration, and MoveIt/PnP.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("arm_cell_bringup"))
    default_profile = share / "config/pnp_validation_profile.yaml"
    profile = LaunchConfiguration("profile")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value=str(default_profile),
                description="Approved deterministic fixed-target PnP profile",
            ),
            Node(
                package="arm_cell_integration",
                executable="material_handoff_node",
                name="material_handoff_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch/arm_cell_pnp_validation.launch.py")
                ),
                launch_arguments={"profile": profile}.items(),
            ),
        ]
    )
