#pragma once

#include <cstdint>
#include <string>

#include <arm_cell_interfaces/action/execute_task.hpp>

#include "arm_cell_task_execution/task_execution_types.hpp"

namespace arm_cell_task_execution
{

void queue_pick_execution_record(
  const arm_cell_interfaces::action::ExecuteTask::Goal & goal,
  const std::string & motion_goal_uuid,
  const std::string & stage,
  std::uint8_t result_code,
  bool held,
  bool retract_succeeded,
  const std::string & candidate_evaluation);

}  // namespace arm_cell_task_execution
