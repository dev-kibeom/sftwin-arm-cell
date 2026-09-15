"""Check the installed profile can compose without simulator imports."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
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
