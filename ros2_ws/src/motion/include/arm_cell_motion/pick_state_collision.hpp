#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <string>
#include <utility>
#include <vector>

namespace arm_cell_motion
{
struct PickCollisionValidationResult
{
  bool collision{false};
  std::vector<std::pair<std::string, std::string>> contacts;
};

// Checks one state against the caller's existing request-scoped scene.
PickCollisionValidationResult validate_pick_state_collision(
  const planning_scene::PlanningScene & scene,
  const moveit::core::RobotState & state);
// Reject only finger/rail intersections invariant under the arm IK solution.
PickCollisionValidationResult validate_pick_grasp_pose_rail_collision(
  const planning_scene::PlanningScene & scene,
  const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * arm,
  const geometry_msgs::msg::PoseStamped & tcp_goal);
}  // namespace arm_cell_motion
