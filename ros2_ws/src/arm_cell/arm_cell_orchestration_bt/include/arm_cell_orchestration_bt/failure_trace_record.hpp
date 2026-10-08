#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace arm_cell_orchestration_bt
{

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
};

inline std::string format_terminal_failure_record(const TerminalFailureRecord & record)
{
  std::ostringstream stream;
  stream << "mission_failure origin=Motion"
         << " execute_cycle_id=" << record.execute_cycle_id
         << " delivery_id=" << record.delivery_id
         << " target_id=" << record.target_id
         << " motion_execution_id=" << record.motion_execution_id
         << " task_type=" << static_cast<unsigned int>(record.task_type)
         << " motion_result_code=" << static_cast<unsigned int>(record.motion_result_code)
         << " exit_reason=" << static_cast<unsigned int>(record.terminal_exit_reason)
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

}  // namespace arm_cell_orchestration_bt
