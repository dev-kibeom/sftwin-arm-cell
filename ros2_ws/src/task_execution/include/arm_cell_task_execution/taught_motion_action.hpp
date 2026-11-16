#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include "arm_cell_task_execution/task_execution_support.hpp"
#include "arm_cell_task_execution/task_execution_report.hpp"

namespace arm_cell_task_execution
{
class TaughtMotionAction final : public BT::SyncActionNode
{
public:
  TaughtMotionAction(
    const std::string & name, const BT::NodeConfiguration & config,
    TaskExecutionSupport & support, TaskExecutionReport & report)
  : BT::SyncActionNode(name, config), support_(support), report_(report) {}

  static BT::PortsList providedPorts()
  {return {BT::InputPort<std::string>("motion")};}

private:
  BT::NodeStatus tick() override
  {
    std::string motion;
    if (!getInput("motion", motion) || motion.empty()) {
      report_.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
      report_.failure_stage = "execution:taught_motion_name_required";
      return BT::NodeStatus::FAILURE;
    }
    report_.failure_stage = "execution:taught_motion:" + motion;
    report_.result_code = support_.execute_taught_motion(motion);
    if (report_.result_code == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      return BT::NodeStatus::SUCCESS;
    }
    if (report_.object_disposition == ExecuteTask::Result::OBJECT_DISPOSITION_HELD ||
      report_.object_disposition == ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN) {
      report_.object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN;
    }
    return BT::NodeStatus::FAILURE;
  }
  TaskExecutionSupport & support_;
  TaskExecutionReport & report_;
};

inline void register_taught_motion_action(
  BT::BehaviorTreeFactory & factory, TaskExecutionSupport & support, TaskExecutionReport & report)
{
  factory.registerBuilder<TaughtMotionAction>(
    "MoveTaught",
    [&support, &report](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<TaughtMotionAction>(name, config, support, report);
    });
}
}  // namespace arm_cell_task_execution
