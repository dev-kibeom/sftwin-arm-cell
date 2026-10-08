"""M0609 ROS 2 Action Graph declaration and Isaac graph construction."""

from graph_builder.graph_contract import (
    ARTICULATION_ROOT_PATH,
    CAMERA_FRAME_ID,
    CAMERA_HEIGHT,
    CAMERA_INFO_TOPIC,
    CAMERA_PATH,
    CAMERA_WIDTH,
    DEPTH_TOPIC,
    ENABLE_ARM_COMMAND_SUBSCRIBER,
    ENABLE_CAMERA,
    ENABLE_GRIPPER_COMMAND_SUBSCRIBER,
    GRAPH_PATH,
    GRIPPER_COMMAND_TOPIC,
    GRIPPER_STATUS_TOPIC,
    JOINT_COMMAND_TOPIC,
    JOINT_STATE_TOPIC,
    RGB_TOPIC,
    ROBOT_PATH,
    ROS2_ROOT,
)
from graph_builder.graph_lifecycle import release_graph_controller_node


def graph_declaration(sdf_path, *, enable_arm_command_subscriber=None):
    """Return the complete graph declaration without importing Isaac runtime APIs."""
    if enable_arm_command_subscriber is None:
        enable_arm_command_subscriber = ENABLE_ARM_COMMAND_SUBSCRIBER
    create_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("PublishGripperStatus", "isaacsim.ros2.bridge.ROS2Publisher"),
    ]
    set_values = [
        ("ROS2Context.inputs:useDomainIDEnvVar", True),
        ("ReadSimTime.inputs:resetOnStop", True),
        ("PublishClock.inputs:topicName", "/clock"),
        ("PublishJointState.inputs:topicName", JOINT_STATE_TOPIC),
        ("PublishJointState.inputs:targetPrim", [sdf_path(ARTICULATION_ROOT_PATH)]),
        ("PublishGripperStatus.inputs:topicName", GRIPPER_STATUS_TOPIC),
        ("PublishGripperStatus.inputs:messagePackage", "std_msgs"),
        ("PublishGripperStatus.inputs:messageSubfolder", "msg"),
        ("PublishGripperStatus.inputs:messageName", "String"),
        ("PublishGripperStatus.inputs:data", ""),
    ]
    create_attributes = []
    connect = [
        ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("ROS2Context.outputs:context", "PublishClock.inputs:context"),
        ("OnPlaybackTick.outputs:tick", "PublishGripperStatus.inputs:execIn"),
        ("ROS2Context.outputs:context", "PublishGripperStatus.inputs:context"),
        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
    ]

    if enable_arm_command_subscriber:
        create_nodes.extend(
            [
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                (
                    "ArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
            ]
        )
        set_values.extend(
            [
                ("SubscribeJointState.inputs:topicName", JOINT_COMMAND_TOPIC),
                (
                    "ArticulationController.inputs:targetPrim",
                    [sdf_path(ARTICULATION_ROOT_PATH)],
                ),
            ]
        )
        connect.extend(
            [
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("ROS2Context.outputs:context", "SubscribeJointState.inputs:context"),
                (
                    "SubscribeJointState.outputs:jointNames",
                    "ArticulationController.inputs:jointNames",
                ),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
            ]
        )

    if ENABLE_GRIPPER_COMMAND_SUBSCRIBER:
        create_nodes.append(
            ("SubscribeGripperCommand", "isaacsim.ros2.bridge.ROS2Subscriber")
        )
        create_nodes.append(("GripperCommandGeneration", "omni.graph.action.Counter"))
        set_values.extend(
            [
                ("SubscribeGripperCommand.inputs:topicName", GRIPPER_COMMAND_TOPIC),
                ("SubscribeGripperCommand.inputs:messagePackage", "std_msgs"),
                ("SubscribeGripperCommand.inputs:messageSubfolder", "msg"),
                ("SubscribeGripperCommand.inputs:messageName", "Float64"),
            ]
        )
        connect.extend(
            [
                (
                    "OnPlaybackTick.outputs:tick",
                    "SubscribeGripperCommand.inputs:execIn",
                ),
                (
                    "ROS2Context.outputs:context",
                    "SubscribeGripperCommand.inputs:context",
                ),
                (
                    "SubscribeGripperCommand.outputs:execOut",
                    "GripperCommandGeneration.inputs:execIn",
                ),
            ]
        )

    if ENABLE_CAMERA:
        create_nodes.extend(
            [
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CameraHelperColor", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoHelper", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ]
        )
        set_values.extend(
            [
                ("CreateRenderProduct.inputs:cameraPrim", [sdf_path(CAMERA_PATH)]),
                ("CreateRenderProduct.inputs:enabled", True),
                ("CreateRenderProduct.inputs:width", CAMERA_WIDTH),
                ("CreateRenderProduct.inputs:height", CAMERA_HEIGHT),
                ("CameraHelperColor.inputs:enabled", True),
                ("CameraHelperColor.inputs:topicName", RGB_TOPIC),
                ("CameraHelperColor.inputs:type", "rgb"),
                ("CameraHelperColor.inputs:frameId", CAMERA_FRAME_ID),
                ("CameraHelperDepth.inputs:enabled", True),
                ("CameraHelperDepth.inputs:topicName", DEPTH_TOPIC),
                ("CameraHelperDepth.inputs:type", "depth"),
                ("CameraHelperDepth.inputs:frameId", CAMERA_FRAME_ID),
                ("CameraInfoHelper.inputs:enabled", True),
                ("CameraInfoHelper.inputs:topicName", CAMERA_INFO_TOPIC),
                ("CameraInfoHelper.inputs:frameId", CAMERA_FRAME_ID),
            ]
        )
        connect.extend(
            [
                ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "CameraHelperColor.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "CameraHelperDepth.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "CameraHelperColor.inputs:renderProductPath",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "CameraHelperDepth.inputs:renderProductPath",
                ),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "CameraInfoHelper.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "CameraInfoHelper.inputs:renderProductPath",
                ),
                ("ROS2Context.outputs:context", "CameraHelperColor.inputs:context"),
                ("ROS2Context.outputs:context", "CameraHelperDepth.inputs:context"),
                ("ROS2Context.outputs:context", "CameraInfoHelper.inputs:context"),
            ]
        )
    return create_nodes, set_values, connect, create_attributes


_active_rgbd_scenarios = set()
_rgbd_stale_stimulus = None


def apply_rgbd_source_scenario(
    scenario, active, *, controller=None, stale_adapter_factory=None
):
    """Private source-side control port for the approved RGB-D fault seam.

    Unavailable withholds the source's color publication. Stale captures and
    replays a real image with an older source stamp on the same production
    image topic. Invalid changes source CameraInfo frame metadata. No ROS
    control interface is created.
    """
    allowed = {"target_unavailable", "target_stale", "target_invalid"}
    if scenario not in allowed:
        raise ValueError(f"unsupported RGB-D scenario: {scenario}")
    if controller is None:
        try:
            import omni.graph.core as og

            controller = og.Controller
        except ImportError:
            return False
    color_enabled = controller.attribute(
        f"{GRAPH_PATH}/CameraHelperColor.inputs:enabled"
    )
    camera_frame = controller.attribute(f"{GRAPH_PATH}/CameraInfoHelper.inputs:frameId")
    if color_enabled is None or not color_enabled.is_valid():
        return False
    if camera_frame is None or not camera_frame.is_valid():
        return False
    global _rgbd_stale_stimulus
    if active:
        if scenario == "target_stale" and scenario not in _active_rgbd_scenarios:
            if stale_adapter_factory is None:
                from graph_builder.rgbd_source_stimulus import RgbdStaleStimulus

                stale_adapter_factory = RgbdStaleStimulus
            stimulus = stale_adapter_factory(RGB_TOPIC)
            if not stimulus.start():
                return False
            _rgbd_stale_stimulus = stimulus
        _active_rgbd_scenarios.add(scenario)
    else:
        if scenario == "target_stale" and scenario in _active_rgbd_scenarios:
            if _rgbd_stale_stimulus is not None:
                _rgbd_stale_stimulus.close()
                _rgbd_stale_stimulus = None
        _active_rgbd_scenarios.discard(scenario)
    color_enabled.set(
        ENABLE_CAMERA
        and not bool(_active_rgbd_scenarios & {"target_unavailable", "target_stale"})
    )
    camera_frame.set(
        "scenario_invalid_optical_frame"
        if "target_invalid" in _active_rgbd_scenarios
        else CAMERA_FRAME_ID
    )
    return True


def build_ros_action_graph(
    *,
    stage,
    controller,
    sdf_path,
    usd_geom,
    usd_physics,
    report=print,
    enable_arm_command_subscriber=None,
):
    """Validate stage prims, replace the graph, and author the declared graph."""
    if stage is None:
        raise RuntimeError("No USD stage is currently open.")
    if not stage.GetPrimAtPath(ROS2_ROOT).IsValid():
        usd_geom.Scope.Define(stage, ROS2_ROOT)
        report(f">>> [INFO] Created ROS2 scope: {ROS2_ROOT}")

    robot_prim = stage.GetPrimAtPath(ROBOT_PATH)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Robot prim not found: {ROBOT_PATH}")
    articulation_root_prim = stage.GetPrimAtPath(ARTICULATION_ROOT_PATH)
    if not articulation_root_prim.IsValid():
        raise RuntimeError(f"Articulation root not found: {ARTICULATION_ROOT_PATH}")
    if not articulation_root_prim.HasAPI(usd_physics.ArticulationRootAPI):
        raise RuntimeError(f"ArticulationRootAPI missing: {ARTICULATION_ROOT_PATH}")

    existing_graph = stage.GetPrimAtPath(GRAPH_PATH)
    if existing_graph.IsValid():
        release_graph_controller_node(controller, GRAPH_PATH)
        stage.RemovePrim(GRAPH_PATH)
        report(f">>> [INFO] Removed existing graph: {GRAPH_PATH}")

    create_nodes, set_values, connect, create_attributes = graph_declaration(
        sdf_path,
        enable_arm_command_subscriber=enable_arm_command_subscriber,
    )
    keys = controller.Keys
    controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: create_nodes,
            keys.CREATE_ATTRIBUTES: create_attributes,
            keys.SET_VALUES: set_values,
            keys.CONNECT: connect,
        },
    )
    graph_prim = stage.GetPrimAtPath(GRAPH_PATH)
    if not graph_prim.IsValid():
        raise RuntimeError(f"Action Graph was not created: {GRAPH_PATH}")
    _report_summary(report, enable_arm_command_subscriber)


def _report_summary(report, enable_arm_command_subscriber=None):
    report()
    report("============================================================")
    report(" SF-Twin ROS 2 Action Graph")
    report("============================================================")
    report(f">>> [SUCCESS] Graph       : {GRAPH_PATH}")
    report(f">>> [SUCCESS] Robot       : {ROBOT_PATH}")
    report(f">>> [SUCCESS] Joint state : {JOINT_STATE_TOPIC}")
    arm_command_subscriber_enabled = (
        ENABLE_ARM_COMMAND_SUBSCRIBER
        if enable_arm_command_subscriber is None
        else enable_arm_command_subscriber
    )
    if arm_command_subscriber_enabled:
        report(f">>> [SUCCESS] Joint cmd   : {JOINT_COMMAND_TOPIC}")
    else:
        report(">>> [INFO]    Joint cmd   : DISABLED")
    if ENABLE_CAMERA:
        report(f">>> [SUCCESS] Camera      : {CAMERA_PATH}")
        report(f">>> [SUCCESS] RGB         : {RGB_TOPIC}")
        report(f">>> [SUCCESS] Depth       : {DEPTH_TOPIC}")
        report(f">>> [SUCCESS] Camera info : {CAMERA_INFO_TOPIC}")
        report(f">>> [SUCCESS] Resolution  : {CAMERA_WIDTH} x {CAMERA_HEIGHT}")
    else:
        report(">>> [INFO]    Camera      : DISABLED")
    if ENABLE_GRIPPER_COMMAND_SUBSCRIBER:
        report(
            f">>> [SUCCESS] Gripper cmd : {GRIPPER_COMMAND_TOPIC} [std_msgs/Float64, mm]"
        )
    else:
        report(">>> [INFO]    Gripper cmd : DISABLED")
    report("============================================================")
    report("Press Play, then verify from a ROS 2 sourced terminal:")
    report("  ros2 topic list")
    report("  ros2 topic echo /isaac/joint_states --once")
    report("============================================================")
