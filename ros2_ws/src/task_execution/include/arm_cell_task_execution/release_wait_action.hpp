#pragma once

#include "arm_cell_task_execution/wait_action.hpp"
#include "arm_cell_task_execution/task_execution_support.hpp"

namespace arm_cell_task_execution
{
// Device-neutral observation completion: adapters own the RELEASED predicate.
inline void register_release_wait_action(
  BT::BehaviorTreeFactory & factory, const std::string & name,
  TaskExecutionSupport & support, const std::uint64_t & baseline,
  HoldingWaitResult & result, TaskExecutionReport & report, WaitAction::Timeout timeout = {})
{
  factory.registerBuilder<WaitAction>(
    name,
    [&support, &baseline, &result, &report, timeout](
      const std::string & node_name, const BT::NodeConfiguration & config) {
      auto completed = [&support, &baseline, &result]() {
        const auto gripper = support.gripper();
        if (!gripper) {return false;}
        const auto observation = gripper->holding();
        if (observation.sequence <= baseline) {return false;}
        if (observation.state == HoldingState::RELEASED && observation.fresh()) {
          result = HoldingWaitResult::CONFIRMED;
          return true;
        }
        if (observation.state == HoldingState::UNKNOWN && observation.recent()) {
          result = HoldingWaitResult::UNKNOWN;
        }
        return false;
      };
      return std::make_unique<WaitAction>(
        node_name, config, support.interruption_checker(),
        report, completed, timeout, support.holding_confirmation_timeout());
    });
}
}  // namespace arm_cell_task_execution
