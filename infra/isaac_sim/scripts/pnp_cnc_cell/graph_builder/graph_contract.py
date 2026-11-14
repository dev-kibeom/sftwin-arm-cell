"""ROS contracts and existing Cell camera/graph placement constants."""

CELL_ROOT = "/World/SF_Twin_Cell"
ROS2_ROOT = f"{CELL_ROOT}/ROS2"

GRAPH_PATH = f"{CELL_ROOT}/ROS2/ActionGraph"

GRIPPER_COMMAND_TOPIC = "/gripper/command"
GRIPPER_CLOSE_TOPIC = "/gripper/close"
GRIPPER_CAPTURE_REFERENCE_TOPIC = "/gripper/capture_reference"
GRIPPER_STATUS_TOPIC = "/gripper/status"
JOINT_STATE_TOPIC = "/isaac/joint_states"
JOINT_COMMAND_TOPIC = "/joint_commands"

ENABLE_GRIPPER_COMMAND_SUBSCRIBER = True
ENABLE_CAMERA = True
ENABLE_ARM_COMMAND_SUBSCRIBER = True
