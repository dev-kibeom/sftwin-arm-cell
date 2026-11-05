#pragma once

#include <functional>
#include <string>
#include <utility>

#include <behaviortree_cpp_v3/bt_factory.h>

#include "arm_cell_motion/motion_port.hpp"
#include "arm_cell_task_execution/bt_callback_node.hpp"
#include "arm_cell_task_execution/task_execution_support.hpp"

namespace arm_cell_task_execution
{

using MotionRequestFactory = std::function<arm_cell_motion::MotionPortRequest()>;
using MotionPlanFactory = std::function<arm_cell_motion::MotionPlan()>;
using MotionPlanResultHandler = std::function<BT::NodeStatus(
      const arm_cell_motion::MotionPlanningResult &)>;
using MotionExecutionResultHandler = std::function<BT::NodeStatus(
      const arm_cell_motion::MotionPortResult &)>;

inline void register_plan_motion_action(
  BT::BehaviorTreeFactory & factory, const std::string & registration_name,
  TaskExecutionSupport & support, MotionRequestFactory make_request,
  std::uint8_t phase,
  MotionPlanResultHandler on_result)
{
  arm_cell_motion::register_callback_action(
    factory, registration_name,
    [&support, phase, make_request = std::move(make_request),
    on_result = std::move(on_result)]() {
      const auto result = support.plan_motion_request(make_request(), phase);
      return on_result(result);
    });
}

inline void register_execute_planned_motion_action(
  BT::BehaviorTreeFactory & factory, const std::string & registration_name,
  TaskExecutionSupport & support, MotionPlanFactory make_plan, bool completes_operation,
  MotionExecutionResultHandler on_result)
{
  arm_cell_motion::register_callback_action(
    factory, registration_name,
    [&support, make_plan = std::move(make_plan), completes_operation,
    on_result = std::move(on_result)]() {
      const auto result = support.execute_planned_motion(make_plan(), completes_operation);
      return on_result(result);
    });
}

}  // namespace arm_cell_task_execution
