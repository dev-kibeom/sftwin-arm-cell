from graph_builder.graph_contract import (
    ARTICULATION_ROOT_PATH,
    CAMERA_INFO_TOPIC,
    CAMERA_PATH,
    DEPTH_TOPIC,
    GRAPH_PATH,
    JOINT_COMMAND_TOPIC,
    JOINT_STATE_TOPIC,
    RGB_TOPIC,
    GRIPPER_STATUS_TOPIC,
)
from graph_builder.ros_action_graph import _report_summary, graph_declaration


def test_graph_declaration_preserves_required_nodes_and_ros_contracts():
    create_nodes, set_values, connect, _ = graph_declaration(
        lambda path: f"SdfPath({path})"
    )
    nodes = dict(create_nodes)
    values = dict(set_values)

    assert nodes["PublishJointState"] == "isaacsim.ros2.bridge.ROS2PublishJointState"
    assert nodes["PublishClock"] == "isaacsim.ros2.bridge.ROS2PublishClock"
    assert (
        nodes["SubscribeJointState"] == "isaacsim.ros2.bridge.ROS2SubscribeJointState"
    )
    assert (
        nodes["ArticulationController"]
        == "isaacsim.core.nodes.IsaacArticulationController"
    )
    assert values["PublishJointState.inputs:topicName"] == JOINT_STATE_TOPIC
    assert values["SubscribeJointState.inputs:topicName"] == JOINT_COMMAND_TOPIC
    assert nodes["PublishGripperStatus"] == "isaacsim.ros2.bridge.ROS2Publisher"
    assert nodes["GripperCommandGeneration"] == "omni.graph.action.Counter"
    assert values["PublishGripperStatus.inputs:topicName"] == GRIPPER_STATUS_TOPIC
    assert values["PublishGripperStatus.inputs:messageName"] == "String"
    assert (
        "SubscribeGripperCommand.outputs:execOut",
        "GripperCommandGeneration.inputs:execIn",
    ) in connect
    assert values["PublishJointState.inputs:targetPrim"] == [
        f"SdfPath({ARTICULATION_ROOT_PATH})"
    ]
    assert values["ArticulationController.inputs:targetPrim"] == [
        f"SdfPath({ARTICULATION_ROOT_PATH})"
    ]
    assert ("ReadSimTime.inputs:resetOnStop", True) in set_values
    assert ("PublishClock.inputs:topicName", "/clock") in set_values
    assert (
        "ReadSimTime.outputs:simulationTime",
        "PublishClock.inputs:timeStamp",
    ) in connect


def test_camera_helpers_share_one_render_product_and_camera_info_contract():
    create_nodes, set_values, connect, _ = graph_declaration(lambda path: path)
    nodes = dict(create_nodes)
    values = dict(set_values)

    assert (
        nodes["CreateRenderProduct"] == "isaacsim.core.nodes.IsaacCreateRenderProduct"
    )
    assert nodes["CameraHelperColor"] == "isaacsim.ros2.bridge.ROS2CameraHelper"
    assert nodes["CameraHelperDepth"] == "isaacsim.ros2.bridge.ROS2CameraHelper"
    assert nodes["CameraInfoHelper"] == "isaacsim.ros2.bridge.ROS2CameraInfoHelper"
    assert "CameraLifecycleSequence" not in nodes
    assert values["CreateRenderProduct.inputs:cameraPrim"] == [CAMERA_PATH]
    assert values["CameraHelperColor.inputs:topicName"] == RGB_TOPIC
    assert values["CameraHelperDepth.inputs:topicName"] == DEPTH_TOPIC
    assert values["CameraInfoHelper.inputs:topicName"] == CAMERA_INFO_TOPIC

    render_product_links = {
        destination
        for source, destination in connect
        if source == "CreateRenderProduct.outputs:renderProductPath"
    }
    assert render_product_links == {
        "CameraHelperColor.inputs:renderProductPath",
        "CameraHelperDepth.inputs:renderProductPath",
        "CameraInfoHelper.inputs:renderProductPath",
    }
    assert (
        "CreateRenderProduct.outputs:execOut",
        "CameraHelperColor.inputs:execIn",
    ) in connect
    assert (
        "CreateRenderProduct.outputs:execOut",
        "CameraHelperDepth.inputs:execIn",
    ) in connect
    assert GRAPH_PATH == "/World/SF_Twin_Cell/ROS2/ActionGraph"


def test_terminal_cleanup_graph_can_publish_without_recreating_physics_view():
    create_nodes, _, connect, _ = graph_declaration(
        lambda path: path, enable_arm_command_subscriber=False
    )

    node_names = {name for name, _ in create_nodes}
    assert "PublishJointState" in node_names
    assert "PublishClock" in node_names
    assert "ArticulationController" not in node_names
    assert "SubscribeJointState" not in node_names
    assert not any(
        "ArticulationController" in endpoint for edge in connect for endpoint in edge
    )


def test_graph_summary_uses_the_selected_arm_command_subscriber_setting():
    messages = []

    _report_summary(
        lambda message="": messages.append(message),
        enable_arm_command_subscriber=False,
    )

    assert ">>> [INFO]    Joint cmd   : DISABLED" in messages


def test_graph_summary_reports_arm_command_subscriber_when_enabled():
    messages = []

    _report_summary(
        lambda message="": messages.append(message),
        enable_arm_command_subscriber=True,
    )

    assert f">>> [SUCCESS] Joint cmd   : {JOINT_COMMAND_TOPIC}" in messages
