#pragma once

#include <cstdint>

#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/motion_task_phase.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>
#include <arm_cell_interfaces/msg/stop_mode.hpp>


namespace arm_cell_motion
{

inline const char * motion_result_code_name(uint8_t value)
{
  using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
  switch (value) {
    case Code::TASK_RESULT_SUCCESS:
      return "TASK_RESULT_SUCCESS";
    case Code::TASK_RESULT_INVALID_GOAL:
      return "TASK_RESULT_INVALID_GOAL";
    case Code::TASK_RESULT_IK_UNREACHABLE:
      return "TASK_RESULT_IK_UNREACHABLE";
    case Code::TASK_RESULT_PATH_OBSTRUCTED:
      return "TASK_RESULT_PATH_OBSTRUCTED";
    case Code::TASK_RESULT_GRASP_FAILURE:
      return "TASK_RESULT_GRASP_FAILURE";
    case Code::TASK_RESULT_CANCELED:
      return "TASK_RESULT_CANCELED";
    case Code::TASK_RESULT_SAFETY_PREEMPTED:
      return "TASK_RESULT_SAFETY_PREEMPTED";
    case Code::TASK_RESULT_BACKEND_UNAVAILABLE:
      return "TASK_RESULT_BACKEND_UNAVAILABLE";
    case Code::TASK_RESULT_PERMISSION_DENIED:
      return "TASK_RESULT_PERMISSION_DENIED";
    case Code::TASK_RESULT_BACKEND_ERROR:
      return "TASK_RESULT_BACKEND_ERROR";
    case Code::TASK_RESULT_BUSY:
      return "TASK_RESULT_BUSY";
    case Code::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED:
      return "TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED";
    case Code::TASK_RESULT_JOINT_PROGRESS_STALLED:
      return "TASK_RESULT_JOINT_PROGRESS_STALLED";
    default:
      return "UNKNOWN";
  }
}

inline const char * motion_task_type_name(uint8_t value)
{
  using Type = arm_cell_interfaces::msg::MotionTaskType;
  switch (value) {
    case Type::TASK_TYPE_UNSPECIFIED: return "TASK_TYPE_UNSPECIFIED";
    case Type::TASK_TYPE_PICK: return "TASK_TYPE_PICK";
    case Type::TASK_TYPE_PLACE: return "TASK_TYPE_PLACE";
    case Type::TASK_TYPE_GO_HOME: return "TASK_TYPE_GO_HOME";
    case Type::TASK_TYPE_RETRACT: return "TASK_TYPE_RETRACT";
    default: return "UNKNOWN";
  }
}

inline const char * motion_task_phase_name(uint8_t value)
{
  using Phase = arm_cell_interfaces::msg::MotionTaskPhase;
  switch (value) {
    case Phase::TASK_PHASE_UNKNOWN: return "TASK_PHASE_UNKNOWN";
    case Phase::TASK_PHASE_VALIDATING: return "TASK_PHASE_VALIDATING";
    case Phase::TASK_PHASE_PLANNING: return "TASK_PHASE_PLANNING";
    case Phase::TASK_PHASE_APPROACHING: return "TASK_PHASE_APPROACHING";
    case Phase::TASK_PHASE_EXECUTING: return "TASK_PHASE_EXECUTING";
    case Phase::TASK_PHASE_GRIPPER: return "TASK_PHASE_GRIPPER";
    case Phase::TASK_PHASE_ATTACHING: return "TASK_PHASE_ATTACHING";
    case Phase::TASK_PHASE_DETACHING: return "TASK_PHASE_DETACHING";
    case Phase::TASK_PHASE_RETRACTING: return "TASK_PHASE_RETRACTING";
    case Phase::TASK_PHASE_COMPLETING: return "TASK_PHASE_COMPLETING";
    default: return "UNKNOWN";
  }
}

inline const char * motion_state_name(uint8_t value)
{
  using State = arm_cell_interfaces::msg::MotionStatus;
  switch (value) {
    case State::MOTION_STATE_UNKNOWN: return "MOTION_STATE_UNKNOWN";
    case State::MOTION_STATE_IDLE: return "MOTION_STATE_IDLE";
    case State::MOTION_STATE_EXECUTING: return "MOTION_STATE_EXECUTING";
    case State::MOTION_STATE_STOPPING: return "MOTION_STATE_STOPPING";
    case State::MOTION_STATE_STOPPED: return "MOTION_STATE_STOPPED";
    case State::MOTION_STATE_EMERGENCY_STOPPED: return "MOTION_STATE_EMERGENCY_STOPPED";
    case State::MOTION_STATE_FAULTED: return "MOTION_STATE_FAULTED";
    default: return "UNKNOWN";
  }
}

inline const char * motion_stop_mode_name(uint8_t value)
{
  using Mode = arm_cell_interfaces::msg::StopMode;
  switch (value) {
    case Mode::STOP_MODE_CONTROLLED: return "STOP_MODE_CONTROLLED";
    case Mode::STOP_MODE_IMMEDIATE: return "STOP_MODE_IMMEDIATE";
    case Mode::STOP_MODE_EMERGENCY: return "STOP_MODE_EMERGENCY";
    default: return "UNKNOWN";
  }
}

}  // namespace arm_cell_motion
