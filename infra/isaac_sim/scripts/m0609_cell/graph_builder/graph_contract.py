"""String, path, topic, and profile constants for the M0609 ROS Action Graph."""

CELL_ROOT = "/World/SF_Twin_Cell"
ROS2_ROOT = f"{CELL_ROOT}/ROS2"

ROBOT_PATH = f"{CELL_ROOT}/m0609"
ROBOT_BASE_PATH = f"{ROBOT_PATH}/base_link"
ARTICULATION_ROOT_PATH = f"{ROBOT_PATH}/root_joint"
CAMERA_PATH = f"{CELL_ROOT}/Vision/Camera_Sensor"
GRAPH_PATH = f"{CELL_ROOT}/ROS2/ActionGraph"

GRIPPER_COMMAND_TOPIC = "/gripper/command"
JOINT_STATE_TOPIC = "/isaac/joint_states"
JOINT_COMMAND_TOPIC = "/joint_commands"
RGB_TOPIC = "/camera/color/image_raw"
DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC = "/camera/color/camera_info"
CAMERA_FRAME_ID = "camera_color_optical_frame"

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
ENABLE_GRIPPER_COMMAND_SUBSCRIBER = True
ENABLE_CAMERA = True
ENABLE_ARM_COMMAND_SUBSCRIBER = True
