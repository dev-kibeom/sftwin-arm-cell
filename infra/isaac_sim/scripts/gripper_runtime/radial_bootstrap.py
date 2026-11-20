"""Scene/Play entrypoints for the physical radial Tool."""
from gripper_runtime.configured_bootstrap import bootstrap  # Exported snapshots reference this callable.


def prepare(*, configuration, stage, **unused):
    from isaac_runtime.robot_binding import RobotBinding
    from gripper_runtime.radial_binding import prepare_drives
    prepare_drives(stage, RobotBinding.from_runtime(configuration), configuration['tool']['runtime'])
