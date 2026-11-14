from runtime_configuration import selected_camera
from graph_builder.graph_contract import (
    GRAPH_PATH,
    GRIPPER_COMMAND_TOPIC,
    GRIPPER_STATUS_TOPIC,
    JOINT_COMMAND_TOPIC,
    JOINT_STATE_TOPIC,
)


def test_graph_contract_preserves_approved_ros_camera_and_command_strings():
    assert GRAPH_PATH == "/World/SF_Twin_Cell/ROS2/ActionGraph"
    assert selected_camera()['prim_path'] == "/World/SF_Twin_Cell/Vision/Camera_Sensor"
    assert (selected_camera()['rgb_topic'], selected_camera()['depth_topic'], selected_camera()['camera_info_topic']) == (
        "/camera/color/image_raw",
        "/camera/aligned_depth_to_color/image_raw",
        "/camera/color/camera_info",
    )
    assert selected_camera()['frame_id'] == "camera_color_optical_frame"
    assert (selected_camera()['resolution'][0], selected_camera()['resolution'][1]) == (1280, 720)
    assert (JOINT_STATE_TOPIC, JOINT_COMMAND_TOPIC, GRIPPER_COMMAND_TOPIC) == (
        "/isaac/joint_states",
        "/joint_commands",
        "/gripper/command",
    )
    assert GRIPPER_STATUS_TOPIC == "/gripper/status"
