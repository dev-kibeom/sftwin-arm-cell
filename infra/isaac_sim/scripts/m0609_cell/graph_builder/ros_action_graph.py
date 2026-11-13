"""ROS 2 Action Graph declaration using selected robot articulation bindings."""

from graph_builder.graph_contract import (
    ENABLE_ARM_COMMAND_SUBSCRIBER,
    ENABLE_CAMERA,
    ENABLE_GRIPPER_COMMAND_SUBSCRIBER,
    GRAPH_PATH,
    GRIPPER_COMMAND_TOPIC,
    GRIPPER_CLOSE_TOPIC,
    GRIPPER_CAPTURE_REFERENCE_TOPIC,
    GRIPPER_STATUS_TOPIC,
    JOINT_COMMAND_TOPIC,
    JOINT_STATE_TOPIC,
    ROS2_ROOT,
)
from runtime_configuration import selected_camera
from robot_binding import load_robot_binding
from graph_builder.graph_lifecycle import release_graph_controller_node


def graph_declaration(sdf_path, *, binding=None, camera=None, enable_arm_command_subscriber=None):
    """Return the complete graph declaration without importing Isaac runtime APIs."""
    binding = binding or load_robot_binding()
    camera = camera if camera is not None else selected_camera()
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
        ("PublishJointState.inputs:targetPrim", [sdf_path(binding.articulation_root_path)]),
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
                    [sdf_path(binding.articulation_root_path)],
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
        create_nodes.append(
            ("SubscribeGripperCaptureReference", "isaacsim.ros2.bridge.ROS2Subscriber")
        )
        create_nodes.append(
            ("SubscribeGripperClose", "isaacsim.ros2.bridge.ROS2Subscriber")
        )
        create_nodes.extend([
            ("SubscribeToolIntent", "isaacsim.ros2.bridge.ROS2Subscriber"),
            ("ToolIntentGeneration", "omni.graph.action.Counter"),
        ])
        set_values.extend([
            ("SubscribeToolIntent.inputs:topicName", "/tool/intent"),
            ("SubscribeToolIntent.inputs:messagePackage", "std_msgs"),
            ("SubscribeToolIntent.inputs:messageSubfolder", "msg"),
            ("SubscribeToolIntent.inputs:messageName", "String"),
        ])
        connect.extend([
            ("OnPlaybackTick.outputs:tick", "SubscribeToolIntent.inputs:execIn"),
            ("ROS2Context.outputs:context", "SubscribeToolIntent.inputs:context"),
            ("SubscribeToolIntent.outputs:execOut", "ToolIntentGeneration.inputs:execIn"),
        ])
        create_nodes.append(("GripperCloseGeneration", "omni.graph.action.Counter"))
        create_nodes.append(("GripperCommandGeneration", "omni.graph.action.Counter"))
        set_values.extend(
            [
                ("SubscribeGripperCommand.inputs:topicName", GRIPPER_COMMAND_TOPIC),
                ("SubscribeGripperCommand.inputs:messagePackage", "std_msgs"),
                ("SubscribeGripperCommand.inputs:messageSubfolder", "msg"),
                ("SubscribeGripperCommand.inputs:messageName", "Float64"),
                (
                    "SubscribeGripperCaptureReference.inputs:topicName",
                    GRIPPER_CAPTURE_REFERENCE_TOPIC,
                ),
                ("SubscribeGripperCaptureReference.inputs:messagePackage", "std_msgs"),
                ("SubscribeGripperCaptureReference.inputs:messageSubfolder", "msg"),
                (
                    "SubscribeGripperCaptureReference.inputs:messageName",
                    "String",
                ),
                ("SubscribeGripperClose.inputs:topicName", GRIPPER_CLOSE_TOPIC),
                ("SubscribeGripperClose.inputs:messagePackage", "std_msgs"),
                ("SubscribeGripperClose.inputs:messageSubfolder", "msg"),
                ("SubscribeGripperClose.inputs:messageName", "String"),
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
                    "OnPlaybackTick.outputs:tick",
                    "SubscribeGripperCaptureReference.inputs:execIn",
                ),
                (
                    "ROS2Context.outputs:context",
                    "SubscribeGripperCaptureReference.inputs:context",
                ),
                ("OnPlaybackTick.outputs:tick", "SubscribeGripperClose.inputs:execIn"),
                ("ROS2Context.outputs:context", "SubscribeGripperClose.inputs:context"),
                (
                    "SubscribeGripperClose.outputs:execOut",
                    "GripperCloseGeneration.inputs:execIn",
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
                ("CreateRenderProduct.inputs:cameraPrim", [sdf_path(camera['prim_path'])]),
                ("CreateRenderProduct.inputs:enabled", True),
                ("CreateRenderProduct.inputs:width", camera['resolution'][0]),
                ("CreateRenderProduct.inputs:height", camera['resolution'][1]),
                ("CameraHelperColor.inputs:enabled", True),
                ("CameraHelperColor.inputs:topicName", camera['rgb_topic']),
                ("CameraHelperColor.inputs:type", "rgb"),
                ("CameraHelperColor.inputs:frameId", camera['frame_id']),
                ("CameraHelperDepth.inputs:enabled", True),
                ("CameraHelperDepth.inputs:topicName", camera['depth_topic']),
                ("CameraHelperDepth.inputs:type", "depth"),
                ("CameraHelperDepth.inputs:frameId", camera['frame_id']),
                ("CameraInfoHelper.inputs:enabled", True),
                ("CameraInfoHelper.inputs:topicName", camera['camera_info_topic']),
                ("CameraInfoHelper.inputs:frameId", camera['frame_id']),
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
    scenario, active, *, controller=None, stale_adapter_factory=None, camera=None
):
    """Private source-side control port for the approved RGB-D fault seam.

    Unavailable withholds the source's color publication. Stale captures and
    replays a real image with an older source stamp on the same production
    image topic. Invalid changes source CameraInfo frame metadata. No ROS
    control interface is created.
    """
    camera = camera if camera is not None else selected_camera()
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
            stimulus = stale_adapter_factory(camera['rgb_topic'])
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
        else camera['frame_id']
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
    binding=None,
    camera=None,
):
    """Validate stage prims, replace the graph, and author the declared graph."""
    binding = binding or load_robot_binding()
    camera = camera if camera is not None else selected_camera()
    if stage is None:
        raise RuntimeError("No USD stage is currently open.")
    if not stage.GetPrimAtPath(ROS2_ROOT).IsValid():
        usd_geom.Scope.Define(stage, ROS2_ROOT)
        report(f">>> [INFO] Created ROS2 scope: {ROS2_ROOT}")

    robot_prim = stage.GetPrimAtPath(binding.robot_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Robot prim not found: {binding.robot_path}")
    articulation_root_prim = stage.GetPrimAtPath(binding.articulation_root_path)
    if not articulation_root_prim.IsValid():
        raise RuntimeError(f"Articulation root not found: {binding.articulation_root_path}")
    if not articulation_root_prim.HasAPI(usd_physics.ArticulationRootAPI):
        raise RuntimeError(f"ArticulationRootAPI missing: {binding.articulation_root_path}")

    existing_graph = stage.GetPrimAtPath(GRAPH_PATH)
    if existing_graph.IsValid():
        release_graph_controller_node(controller, GRAPH_PATH)
        stage.RemovePrim(GRAPH_PATH)
        report(f">>> [INFO] Removed existing graph: {GRAPH_PATH}")

    create_nodes, set_values, connect, create_attributes = graph_declaration(
        sdf_path,
        binding=binding,
        camera=camera,
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
    _report_summary(report, enable_arm_command_subscriber, binding=binding, camera=camera)


def _report_summary(report, enable_arm_command_subscriber=None, *, binding=None, camera=None):
    binding = binding or load_robot_binding()
    camera = camera if camera is not None else selected_camera()
    report()
    report("============================================================")
    report(" SF-Twin ROS 2 Action Graph")
    report("============================================================")
    report(f">>> [SUCCESS] Graph       : {GRAPH_PATH}")
    report(f">>> [SUCCESS] Robot       : {binding.robot_path}")
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
        report(f">>> [SUCCESS] Camera      : {camera['prim_path']}")
        report(f">>> [SUCCESS] RGB         : {camera['rgb_topic']}")
        report(f">>> [SUCCESS] Depth       : {camera['depth_topic']}")
        report(f">>> [SUCCESS] Camera info : {camera['camera_info_topic']}")
        report(f">>> [SUCCESS] Resolution  : {camera['resolution'][0]} x {camera['resolution'][1]}")
    else:
        report(">>> [INFO]    Camera      : DISABLED")
    if ENABLE_GRIPPER_COMMAND_SUBSCRIBER:
        report(
            f">>> [SUCCESS] Gripper width / close : {GRIPPER_COMMAND_TOPIC} [std_msgs/Float64] / "
            f"{GRIPPER_CLOSE_TOPIC} [std_msgs/String]"
        )
    else:
        report(">>> [INFO]    Gripper cmd : DISABLED")
    report("============================================================")
    report("Press Play, then verify from a ROS 2 sourced terminal:")
    report("  ros2 topic list")
    report("  ros2 topic echo /isaac/joint_states --once")
    report("============================================================")
