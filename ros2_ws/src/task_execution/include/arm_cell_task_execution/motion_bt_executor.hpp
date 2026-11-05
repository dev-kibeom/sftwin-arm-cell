#pragma once

#include <memory>
#include <string>
#include <vector>

#include <arm_cell_interfaces/action/execute_task.hpp>

#include "arm_cell_task_execution/task_execution_support.hpp"
#include "arm_cell_task_execution/task_execution_report.hpp"
#include "arm_cell_task_execution/task_execution_strategy.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;

struct MotionBtExecutionResult
{
  std::vector<std::string> attempted_material_candidate_ids;
  std::string outcome_material_candidate_id;
};

class MotionBtExecutor final
{
public:
  explicit MotionBtExecutor(TaskExecutionSupport & support)
  : support_(support) {}
  MotionBtExecutionResult execute(
    const ExecuteTask::Goal & goal, const std::string & motion_execution_id,
    TaskExecutionReport & report, const PickPlaceStrategies & strategies);
  MotionDisturbResult execute_disturb(const MotionDisturbRequest & request);

private:
  TaskExecutionSupport & support_;
};

}  // namespace arm_cell_task_execution
