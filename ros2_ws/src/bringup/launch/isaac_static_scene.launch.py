"""Isaac planning profile with the explicitly selected bounded static scene."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory("arm_cell_bringup")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{share}/launch/isaac_planning.launch.py"
                ),
                launch_arguments={"static_scene": "true"}.items(),
            )
        ]
    )
