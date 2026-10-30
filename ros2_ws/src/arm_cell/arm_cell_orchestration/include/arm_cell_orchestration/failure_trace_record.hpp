#pragma once

#include <cstdint>
#include <sstream>
#include <string>

#include <arm_cell_interfaces/msg/mission_exit_reason.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>
#include <arm_cell_interfaces/msg/detect_target_result_code.hpp>

namespace arm_cell_orchestration
{

inline const char * mission_exit_reason_name(uint8_t value)
{
  using Reason = arm_cell_interfaces::msg::MissionExitReason;
  switch (value) {
    case Reason::MISSION_EXIT_NONE: return "MISSION_EXIT_NONE";
    case Reason::MISSION_EXIT_DEPLETED: return "MISSION_EXIT_DEPLETED";
    case Reason::MISSION_EXIT_VISION_ERROR: return "MISSION_EXIT_VISION_ERROR";
    case Reason::MISSION_EXIT_MOTION_ERROR: return "MISSION_EXIT_MOTION_ERROR";
    case Reason::MISSION_EXIT_SAFETY_PREEMPTED: return "MISSION_EXIT_SAFETY_PREEMPTED";
    case Reason::MISSION_EXIT_CANCELED: return "MISSION_EXIT_CANCELED";
    case Reason::MISSION_EXIT_CONFIGURATION_ERROR: return "MISSION_EXIT_CONFIGURATION_ERROR";
    case Reason::MISSION_EXIT_QUANTITY_REACHED: return "MISSION_EXIT_QUANTITY_REACHED";
    case Reason::MISSION_EXIT_QUANTITY_SHORTFALL: return "MISSION_EXIT_QUANTITY_SHORTFALL";
    default: return "UNKNOWN";
  }
}

inline const char * motion_result_code_name(uint8_t value)
{
  using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
  switch (value) {
    case Code::TASK_RESULT_SUCCESS: return "TASK_RESULT_SUCCESS";
    case Code::TASK_RESULT_INVALID_GOAL: return "TASK_RESULT_INVALID_GOAL";
    case Code::TASK_RESULT_IK_UNREACHABLE: return "TASK_RESULT_IK_UNREACHABLE";
    case Code::TASK_RESULT_PATH_OBSTRUCTED: return "TASK_RESULT_PATH_OBSTRUCTED";
    case Code::TASK_RESULT_GRASP_FAILURE: return "TASK_RESULT_GRASP_FAILURE";
    case Code::TASK_RESULT_CANCELED: return "TASK_RESULT_CANCELED";
    case Code::TASK_RESULT_SAFETY_PREEMPTED: return "TASK_RESULT_SAFETY_PREEMPTED";
    case Code::TASK_RESULT_BACKEND_UNAVAILABLE: return "TASK_RESULT_BACKEND_UNAVAILABLE";
    case Code::TASK_RESULT_PERMISSION_DENIED: return "TASK_RESULT_PERMISSION_DENIED";
    case Code::TASK_RESULT_BACKEND_ERROR: return "TASK_RESULT_BACKEND_ERROR";
    case Code::TASK_RESULT_BUSY: return "TASK_RESULT_BUSY";
    case Code::TASK_RESULT_JOINT_PROGRESS_STALLED: return "TASK_RESULT_JOINT_PROGRESS_STALLED";
    default: return "UNKNOWN";
  }
}

inline const char * detect_target_result_code_name(uint8_t value)
{
  using Code = arm_cell_interfaces::msg::DetectTargetResultCode;
  switch (value) {
    case Code::DETECT_RESULT_SUCCESS: return "DETECT_RESULT_SUCCESS";
    case Code::DETECT_RESULT_OBJECT_NOT_FOUND: return "DETECT_RESULT_OBJECT_NOT_FOUND";
    case Code::DETECT_RESULT_TIMEOUT: return "DETECT_RESULT_TIMEOUT";
    case Code::DETECT_RESULT_SENSOR_ERROR: return "DETECT_RESULT_SENSOR_ERROR";
    case Code::DETECT_RESULT_TF_ERROR: return "DETECT_RESULT_TF_ERROR";
    case Code::DETECT_RESULT_GEOMETRY_ERROR: return "DETECT_RESULT_GEOMETRY_ERROR";
    case Code::DETECT_RESULT_INVALID_RESULT: return "DETECT_RESULT_INVALID_RESULT";
    case Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET:
      return "DETECT_RESULT_TEMPORARY_INVALID_TARGET";
    default: return "UNKNOWN";
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

inline std::string motion_result_code_field(uint8_t value, bool available = true)
{
  return available ? motion_result_code_name(value) : "unavailable";
}

struct TerminalFailureRecord
{
  std::string execute_cycle_id;
  std::string delivery_id;
  std::string target_id;
  std::string motion_execution_id;
  uint8_t task_type{0};
  uint8_t motion_result_code{0};
  uint8_t terminal_exit_reason{0};
  std::string diagnostic_detail;
  bool motion_result_code_available{true};
  uint64_t processed_count{0};
  uint32_t quantity{0};
};

inline std::string format_terminal_failure_record(const TerminalFailureRecord & record)
{
  std::ostringstream stream;
  stream << "mission_failure origin=Motion"
         << " execute_cycle_id=" << record.execute_cycle_id
         << " delivery_id=" << record.delivery_id
         << " target_id=" << record.target_id
         << " motion_execution_id=" << record.motion_execution_id
         << " task_type=" << motion_task_type_name(record.task_type)
         << " motion_result_code=";
  if (record.motion_result_code_available) {
    stream << motion_result_code_name(record.motion_result_code);
  } else {
    stream << "unavailable";
  }
  stream << " exit_reason=" << mission_exit_reason_name(record.terminal_exit_reason)
         << " processed=" << record.processed_count << "/" << record.quantity
         << " diagnostic=\"";
  for (const auto character : record.diagnostic_detail) {
    if (character == '\n' || character == '\r' || character == '"' || character == '\\') {
      stream << ' ';
    } else {
      stream << character;
    }
  }
  stream << '"';
  return stream.str();
}

template<typename LogFn>
inline void emit_terminal_failure_record(const TerminalFailureRecord & record, LogFn && log)
{
  log(format_terminal_failure_record(record));
}

}  // namespace arm_cell_orchestration
