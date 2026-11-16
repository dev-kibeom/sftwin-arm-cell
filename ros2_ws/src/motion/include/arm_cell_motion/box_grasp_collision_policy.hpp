#pragma once

#include <optional>
#include <moveit/planning_scene/planning_scene.h>
#include "arm_cell_motion/box_grasp_geometry.hpp"

namespace arm_cell_motion
{
// Box strategy's 2F85 endpoint policy: close the private state to the contact
// width and install only its target/fingertip contact rules in private scenes.
// Returns false when the requested closed Box geometry cannot be established.
// The caller must still check all common collisions with the whole robot.
bool prepare_closed_box_grasp_collision(
  moveit::core::RobotState & grasp_state,
  planning_scene::PlanningScene & common_scene, planning_scene::PlanningScene & full_scene,
  const Eigen::Vector3d & object_center, const Eigen::Quaterniond & object_orientation,
  const std::optional<BoxGraspContact> & contact, const Eigen::Vector3d & target_dimensions);
}  // namespace arm_cell_motion
