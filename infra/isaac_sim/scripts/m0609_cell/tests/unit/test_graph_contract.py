from graph_builder.graph_contract import (
    CAMERA_FRAME_ID,
    CAMERA_HEIGHT,
    CAMERA_INFO_TOPIC,
    CAMERA_PATH,
    CAMERA_WIDTH,
    DEPTH_TOPIC,
    GRAPH_PATH,
    GRIPPER_COMMAND_TOPIC,
    JOINT_COMMAND_TOPIC,
    JOINT_STATE_TOPIC,
    RGB_TOPIC,
)


def test_graph_contract_preserves_approved_ros_camera_and_command_strings():
    assert GRAPH_PATH == "/World/SF_Twin_Cell/ROS2/ActionGraph"
    assert CAMERA_PATH == "/World/SF_Twin_Cell/Vision/Camera_Sensor"
    assert (RGB_TOPIC, DEPTH_TOPIC, CAMERA_INFO_TOPIC) == (
        "/camera/color/image_raw",
        "/camera/aligned_depth_to_color/image_raw",
        "/camera/color/camera_info",
    )
    assert CAMERA_FRAME_ID == "camera_color_optical_frame"
    assert (CAMERA_WIDTH, CAMERA_HEIGHT) == (1280, 720)
    assert (JOINT_STATE_TOPIC, JOINT_COMMAND_TOPIC, GRIPPER_COMMAND_TOPIC) == (
        "/isaac/joint_states",
        "/joint_commands",
        "/gripper/command",
    )
