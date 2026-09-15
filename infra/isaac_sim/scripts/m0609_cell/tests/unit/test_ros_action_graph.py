from graph_builder.graph_contract import (
    ARTICULATION_ROOT_PATH,
    CAMERA_INFO_TOPIC,
    CAMERA_PATH,
    DEPTH_TOPIC,
    GRAPH_PATH,
    JOINT_COMMAND_TOPIC,
    JOINT_STATE_TOPIC,
    RGB_TOPIC,
)
from graph_builder.ros_action_graph import graph_declaration


def test_graph_declaration_preserves_required_nodes_and_ros_contracts():
    create_nodes, set_values, connect = graph_declaration(
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
    create_nodes, set_values, connect = graph_declaration(lambda path: path)
    nodes = dict(create_nodes)
    values = dict(set_values)

    assert (
        nodes["CreateRenderProduct"] == "isaacsim.core.nodes.IsaacCreateRenderProduct"
    )
    assert nodes["CameraHelperColor"] == "isaacsim.ros2.bridge.ROS2CameraHelper"
    assert nodes["CameraHelperDepth"] == "isaacsim.ros2.bridge.ROS2CameraHelper"
    assert nodes["CameraInfoHelper"] == "isaacsim.ros2.bridge.ROS2CameraInfoHelper"
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
    assert GRAPH_PATH == "/World/SF_Twin_Cell/ROS2/ActionGraph"
