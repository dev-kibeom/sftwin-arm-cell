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
}  // namespace arm_cell_motion
