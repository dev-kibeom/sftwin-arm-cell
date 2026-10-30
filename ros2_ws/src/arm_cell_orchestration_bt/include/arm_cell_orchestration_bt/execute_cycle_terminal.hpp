#pragma once

#include <memory>

#include <arm_cell_interfaces/action/execute_cycle.hpp>
#include <arm_cell_interfaces/msg/mission_exit_reason.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace arm_cell_orchestration_bt
{

using ExecuteCycle = arm_cell_interfaces::action::ExecuteCycle;
using ExecuteCycleGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteCycle>;

inline void complete_execute_cycle_goal(
  const std::shared_ptr<ExecuteCycleGoalHandle> & handle,
  const std::shared_ptr<ExecuteCycle::Result> & result)
{
  switch (result->exit_reason.value) {
    case arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_DEPLETED:
      handle->succeed(result);
      return;
    case arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_CANCELED:
      handle->canceled(result);
      return;
    case arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_VISION_ERROR:
    case arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_MOTION_ERROR:
    case arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED:
    case arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR:
    default:
      handle->abort(result);
      return;
  }
}

}  // namespace arm_cell_orchestration_bt
