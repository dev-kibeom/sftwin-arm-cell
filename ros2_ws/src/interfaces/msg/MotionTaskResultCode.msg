uint8 TASK_RESULT_SUCCESS=0
uint8 TASK_RESULT_INVALID_GOAL=1
uint8 TASK_RESULT_IK_UNREACHABLE=2
uint8 TASK_RESULT_PATH_OBSTRUCTED=3
uint8 TASK_RESULT_GRASP_FAILURE=4
uint8 TASK_RESULT_CANCELED=5
uint8 TASK_RESULT_SAFETY_PREEMPTED=6
uint8 TASK_RESULT_BACKEND_UNAVAILABLE=7
uint8 TASK_RESULT_PERMISSION_DENIED=8
uint8 TASK_RESULT_BACKEND_ERROR=9
uint8 TASK_RESULT_BUSY=10
# Bounded grasp/IK/planning search exhausted without a feasible selected plan.
# This is one possible terminal result of Motion's PICK Action, not a summary
# code for every failed Material Candidate. Motion owns eligible candidate
# transitions before Grasp insertion, including planning failures after a
# successfully completed Approach; actual execution failures are terminal.
# Orchestration does not select the next Material Candidate.
uint8 TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED=11
# Fresh joint feedback showed no progress until the Motion progress watchdog expired.
uint8 TASK_RESULT_JOINT_PROGRESS_STALLED=12

uint8 value
