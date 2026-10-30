#pragma once

#include <functional>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>

namespace arm_cell_motion
{
// Executes exactly one plan-only request with the supplied request scene diff.
moveit::core::MoveItErrorCode plan_with_request_scene_diff(
  moveit::planning_interface::MoveGroupInterface & move_group,
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const moveit_msgs::msg::PlanningScene & planning_scene_diff,
  const std::function<bool()> & interrupted);

// Plans once using the already selected scene policy; retry/result selection stays with caller.
moveit::core::MoveItErrorCode plan_pick_approach(
  moveit::planning_interface::MoveGroupInterface & move_group,
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const moveit_msgs::msg::PlanningScene * planning_scene_diff,
  const std::function<bool()> & interrupted);
}  // namespace arm_cell_motion
