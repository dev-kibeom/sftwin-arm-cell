import os
import sys
import importlib

import omni.physx


SCRIPT_DIR = os.path.expanduser("~/sftwin_project/infra/isaac_sim/scripts/m0609_cell")

if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)


import gripper_runtime.action_graph_command_adapter as adapter_module
import gripper_runtime.grasp_attachment as grasp_module
import gripper_runtime.grasp_policy as grasp_policy_module
import gripper_runtime.robotiq_actuator as gripper_module
import gripper_runtime.session as session_module
from graph_builder.graph_contract import GRAPH_PATH

importlib.reload(gripper_module)
importlib.reload(adapter_module)
importlib.reload(grasp_policy_module)
importlib.reload(grasp_module)
importlib.reload(session_module)


ACTION_GRAPH_PATH = GRAPH_PATH

runtime_session = session_module.create_gripper_runtime_session(
    omni.physx.get_physx_interface()
)
GRASP_CONFIG = runtime_session.grasp_config
grasp_manager = runtime_session.grasp_manager
gripper = runtime_session.gripper
sim_adapter = runtime_session.sim_adapter
on_physics_step = runtime_session.on_physics_step
physics_sub = runtime_session.physics_sub


print()
print("============================================================")
print(" SF-Twin Gripper Runtime")
print("============================================================")
print(">>> [READY] Robotiq2F85Actuator")
print(">>> [READY] SimGripperAdapter")
print(">>> [READY] GraspManager")
print(f">>> ActionGraph : {ACTION_GRAPH_PATH}")
print(">>> ROS topic   : /gripper/command")
print(">>> ROS type    : std_msgs/msg/Float64")
print(">>> Unit        : mm")
print(">>> Grasp target: NONE")
print("============================================================")
