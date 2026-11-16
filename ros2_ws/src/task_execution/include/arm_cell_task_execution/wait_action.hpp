#pragma once

#include <chrono>
#include <functional>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "arm_cell_task_execution/task_execution_report.hpp"

namespace arm_cell_task_execution
{
// Tick-driven dwell: ROS callbacks and Safety processing continue independently.
class WaitAction final : public BT::StatefulActionNode
{
public:
  using Checker = std::function<std::uint8_t()>;
  WaitAction(
    const std::string & name, const BT::NodeConfiguration & config,
    Checker checker, TaskExecutionReport & report)
  : BT::StatefulActionNode(name, config), checker_(std::move(checker)), report_(report) {}
  static BT::PortsList providedPorts()
  {return {BT::InputPort<unsigned>("duration_ms", 2000U, "Dwell duration")};}

private:
  BT::NodeStatus onStart() override
  {
    unsigned duration;
    if (!getInput("duration_ms", duration)) {
      report_.result_code = MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
      return BT::NodeStatus::FAILURE;
    }
    deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration);
    return onRunning();
  }
  BT::NodeStatus onRunning() override
  {
    const auto result = checker_ ? checker_() : MotionTaskResultCode::TASK_RESULT_SUCCESS;
    if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      report_.result_code = result;
      report_.failure_stage = "gripper:settle_wait_interrupted";
      return BT::NodeStatus::FAILURE;
    }
    return std::chrono::steady_clock::now() >= deadline_ ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  void onHalted() override {deadline_ = {};}
  Checker checker_;
  TaskExecutionReport & report_;
  std::chrono::steady_clock::time_point deadline_{};
};
inline void register_wait_action(
  BT::BehaviorTreeFactory & factory,
  WaitAction::Checker checker, TaskExecutionReport & report)
{
  factory.registerBuilder<WaitAction>(
    "Wait",
    [checker, &report](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<WaitAction>(name, config, checker, report);
    });
}
}  // namespace arm_cell_task_execution
