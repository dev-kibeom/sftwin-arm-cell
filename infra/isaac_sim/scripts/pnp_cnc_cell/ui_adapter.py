"""CNC-owned optional operator capabilities; the common Hub imports no CNC code."""


def capabilities():
    from graph_builder.ros_action_graph import apply_rgbd_source_scenario
    from gripper_runtime.holding_scenario import set_holding_unknown
    from scene_builder.cuboid_demonstrations import scenarios, display_name
    from scene_builder.cuboid_demonstration import demonstration_loaded, load_unavailable_reason, load_demonstration
    return dict(rgbd_source=lambda intent:apply_rgbd_source_scenario(intent.scenario,intent.active),
        motion_backend=lambda intent:set_holding_unknown(intent.active) if intent.scenario=='holding_unknown' else False,
        scenarios=scenarios,display_name=display_name,demonstration_loaded=demonstration_loaded,
        load_unavailable_reason=load_unavailable_reason,load_demonstration=load_demonstration)
