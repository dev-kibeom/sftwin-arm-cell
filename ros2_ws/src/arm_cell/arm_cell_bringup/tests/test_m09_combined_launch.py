"""Static composition contract for the M09 end-to-end bringup entrypoint."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch/arm_cell_m09_combined.launch.py"


def _launch_description():
    spec = importlib.util.spec_from_file_location(
        name="m09_combined_launch", location=LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda package: str(PACKAGE_ROOT)
    return module.generate_launch_description()


def test_combined_launch_connects_integration_to_existing_pnp_composition():
    actions = _launch_description().entities
    assert any(
        isinstance(action, DeclareLaunchArgument) and action.name == "profile"
        for action in actions
    )
    integration = [
        action
        for action in actions
        if isinstance(action, Node)
        and action.node_package == "arm_cell_integration"
        and action.node_executable == "material_handoff_node"
    ]
    assert len(integration) == 1
    includes = []
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            includes.append(action)
    assert len(includes) == 1
    source = includes[0].launch_description_source
    location_key = "_LaunchDescriptionSource__location"
    resolved_location = source.__dict__[location_key][0]
    assert resolved_location.perform(LaunchContext()).endswith(
        "arm_cell_pnp_validation.launch.py"
    )
    assert "profile" in dict(includes[0].launch_arguments)
