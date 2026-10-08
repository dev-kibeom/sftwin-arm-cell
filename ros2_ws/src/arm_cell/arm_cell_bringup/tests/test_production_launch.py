"""Static composition contract for the public production runtime entrypoint."""

import importlib.util
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch/arm_cell.launch.py"


def _launch_description():
    spec = importlib.util.spec_from_file_location(
        name="arm_cell_launch", location=LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.get_package_share_directory = lambda package: str(PACKAGE_ROOT)
    return module.generate_launch_description()


def test_public_entrypoint_preserves_production_profile_and_simulator_composition():
    actions = _launch_description().entities
    declared_arguments = [
        action.name for action in actions if isinstance(action, DeclareLaunchArgument)
    ]
    assert {"profile", "log_level", "ros_domain_id"}.issubset(declared_arguments)
    profile = next(
        action
        for action in actions
        if isinstance(action, DeclareLaunchArgument) and action.name == "profile"
    )
    default_profile = Path(profile.default_value[0].perform(LaunchContext()))
    profile_data = yaml.safe_load(default_profile.read_text())
    assert profile_data["use_sim_time"] is True
    assert "RawPart" in profile_data["vision"]["detector_profiles"]
    log_level = next(
        action
        for action in actions
        if isinstance(action, DeclareLaunchArgument) and action.name == "log_level"
    )
    assert log_level.default_value[0].text == "info"

    includes = [
        action for action in actions if isinstance(action, IncludeLaunchDescription)
    ]
    assert len(includes) == 1
    assert "launch/arm_cell_operational.launch.py" in LAUNCH_PATH.read_text()
    arguments = next(
        dict(action.launch_arguments)
        for action in includes
        if dict(action.launch_arguments).get("include_simulation") == "true"
    )
    assert isinstance(arguments["profile"], LaunchConfiguration)
    assert isinstance(arguments["log_level"], LaunchConfiguration)
    assert isinstance(arguments["ros_domain_id"], LaunchConfiguration)
    context = LaunchContext()
    context.launch_configurations["profile"] = "selected-production-profile.yaml"
    assert arguments["profile"].perform(context) == "selected-production-profile.yaml"
    assert arguments["include_simulation"] == "true"
    context.launch_configurations["log_level"] = "debug"
    assert arguments["log_level"].perform(context) == "debug"
