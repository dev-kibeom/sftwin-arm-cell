#pragma once

#include <cstdint>
#include <string>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;

using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
using MotionTaskResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;

// Action-level output exchanged at the BT boundary. Module progress remains local.
struct TaskExecutionReport
{
  std::string motion_execution_id;
  std::string failure_stage;
  std::uint8_t result_code{MotionTaskResultCode::TASK_RESULT_INVALID_GOAL};
  std::uint8_t object_disposition{ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN};
};

}  // namespace arm_cell_task_execution
