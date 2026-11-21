# Identity observed by the single-slot Mission. Empty until it has a reservation.
string material_instance_id
string reservation_id
# Zero until the child Action accepts the goal.
unique_identifier_msgs/UUID task_goal_id
# action_msgs/GoalStatus terminal status; zero means no observed Task terminal.
uint8 task_status
uint8 result_code
string failure_stage
string diagnostic_detail
