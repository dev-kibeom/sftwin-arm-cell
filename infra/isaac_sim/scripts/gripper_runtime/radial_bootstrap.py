"""Common Scene/Play entrypoints for the selected physical radial Tool."""


def prepare(*, configuration, stage, **unused):
    from isaac_runtime.robot_binding import RobotBinding
    from gripper_runtime.radial_binding import prepare_drives
    prepare_drives(stage, RobotBinding.from_runtime(configuration), configuration['tool']['runtime'])


def bootstrap(*, configuration, namespace):
    import os
    from pathlib import Path
    import omni.physx
    from gripper_runtime.session import create_gripper_runtime_session, initial_status_sequence
    from gripper_runtime.tool_plugin import ToolIntent
    previous = namespace.get('runtime_session')
    sequence = getattr(previous, '_status_sequence', 0)
    session = create_gripper_runtime_session(
        omni.physx.get_physx_interface(), configuration=configuration,
        previous_session=previous, initial_status_sequence=initial_status_sequence(sequence),
        diagnostic_output_directory=(
            Path(os.environ['SFTWIN_LIVE_TEST_SESSION_PATH']).parent / 'gripper'
            if os.environ.get('SFTWIN_LIVE_TEST_SESSION_PATH') else None))
    namespace.update(runtime_session=session, gripper=session.gripper,
                     grasp_manager=session.grasp_manager, sim_adapter=session.sim_adapter,
                     on_physics_step=session.on_physics_step, physics_sub=session.physics_sub)
    supply = namespace.get('_material_supply_adapter')
    if supply is not None and callable(getattr(supply, 'set_tool_session', None)):
        supply.set_tool_session(session)
    # Startup contraction uses drive targets. It does not claim material release:
    # only a subsequent correlated runtime RELEASE can publish RELEASED.
    session.tool.command(ToolIntent.RELEASE)
    return session
