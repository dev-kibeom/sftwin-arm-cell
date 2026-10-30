"""Static composition contract for the public production runtime entrypoint."""

import importlib.util
from pathlib import Path

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
    assert {
        "profile",
        "log_level",
        "action_logger_level",
        "ros_domain_id",
        "diagnostics",
    }.issubset(declared_arguments)
    profile = next(
        action
        for action in actions
        if isinstance(action, DeclareLaunchArgument) and action.name == "profile"
    )
    default_profile = Path(profile.default_value[0].perform(LaunchContext()))
    assert (
        default_profile.resolve()
        == (PACKAGE_ROOT / "config/operational_profile.yaml").resolve()
    )
    assert default_profile.is_file()
    log_level = next(
        action
        for action in actions
        if isinstance(action, DeclareLaunchArgument) and action.name == "log_level"
    )
    assert log_level.default_value[0].text == "info"
    diagnostics = next(
        action
        for action in actions
        if isinstance(action, DeclareLaunchArgument) and action.name == "diagnostics"
    )
    assert diagnostics.default_value[0].text == "__log_level_default__"

    includes = [
        action for action in actions if isinstance(action, IncludeLaunchDescription)
    ]
    operational_include = next(
        action
        for action in includes
        if dict(action.launch_arguments).get("include_simulation") == "true"
    )
    arguments = dict(operational_include.launch_arguments)
    assert isinstance(arguments["profile"], LaunchConfiguration)
    assert isinstance(arguments["log_level"], LaunchConfiguration)
    assert isinstance(arguments["action_logger_level"], LaunchConfiguration)
    assert isinstance(arguments["ros_domain_id"], LaunchConfiguration)
    assert isinstance(arguments["diagnostics"], LaunchConfiguration)
    context = LaunchContext()
    context.launch_configurations["profile"] = "selected-production-profile.yaml"
    assert arguments["profile"].perform(context) == "selected-production-profile.yaml"
    assert arguments["include_simulation"] == "true"
    context.launch_configurations["log_level"] = "debug"
    assert arguments["log_level"].perform(context) == "debug"
    context.launch_configurations["action_logger_level"] = "error"
    assert arguments["action_logger_level"].perform(context) == "error"
    context.launch_configurations["diagnostics"] = "vision,motion"
    assert arguments["diagnostics"].perform(context) == "vision,motion"
