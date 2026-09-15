# header.stamp is the Motion status publication time in the active ROS clock.
std_msgs/Header header
uint8 MOTION_STATE_UNKNOWN=0
uint8 MOTION_STATE_IDLE=1
uint8 MOTION_STATE_EXECUTING=2
uint8 MOTION_STATE_STOPPING=3
uint8 MOTION_STATE_STOPPED=4
uint8 MOTION_STATE_EMERGENCY_STOPPED=5
uint8 MOTION_STATE_FAULTED=6

uint8 execution_state
unique_identifier_msgs/UUID active_execution_id
bool has_active_execution
MotionTaskType active_task_type
bool execution_active
bool backend_inactivity_confirmed
unique_identifier_msgs/UUID last_stop_request_id
bool has_last_stop_request
string diagnostic_detail
