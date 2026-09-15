# header.stamp is the Safety evaluation/publication time in the active ROS clock.
# Local receipt time, not header.stamp alone, is used by consumers for watchdogs.
std_msgs/Header header
uint8 SAFETY_STATE_UNKNOWN=0
uint8 SAFETY_STATE_SAFE=1
uint8 SAFETY_STATE_INTERLOCKED=2
uint8 SAFETY_STATE_STOPPING=3
uint8 SAFETY_STATE_STOPPED=4
uint8 SAFETY_STATE_RECOVERY_REQUIRED=5
uint8 SAFETY_STATE_EMERGENCY_STOPPED=6

uint8 safety_state
MotionCapability motion_capability
SafetyCauseSet active_causes
SafetyCauseSet latched_causes
bool valid
bool required_inputs_fresh
string diagnostic_detail
