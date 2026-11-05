#pragma once

#include <cstdint>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>

namespace arm_cell_task_execution
{

using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
using MotionTaskResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;
using MotionTaskType = arm_cell_interfaces::msg::MotionTaskType;

enum class HoldingWaitResult : std::uint8_t
{
  CONFIRMED,
  TIMEOUT,
  UNKNOWN,
  CANCELED,
  SAFETY_PREEMPTED,
  BACKEND_UNAVAILABLE
};

}  // namespace arm_cell_task_execution
