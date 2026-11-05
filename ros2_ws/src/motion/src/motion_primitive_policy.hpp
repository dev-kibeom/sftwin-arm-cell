#pragma once

#include <cstdint>
#include <cmath>

#include <geometry_msgs/msg/pose.hpp>

#include "arm_cell_interfaces/msg/motion_task_phase.hpp"
#include "arm_cell_interfaces/msg/motion_task_type.hpp"
#include "arm_cell_motion/pick_approach_axis.hpp"

namespace arm_cell_motion
{

inline bool make_world_vertical_pick_descent_target(
  const geometry_msgs::msg::Pose & actual_approach,
  const geometry_msgs::msg::Pose & grasp_target,
  geometry_msgs::msg::Pose & descent_target)
{
  if (!std::isfinite(actual_approach.position.x) ||
    !std::isfinite(actual_approach.position.y) ||
    !std::isfinite(actual_approach.position.z) ||
    !std::isfinite(grasp_target.position.x) ||
    !std::isfinite(grasp_target.position.y) ||
    !std::isfinite(grasp_target.position.z) ||
    actual_approach.position.z <= grasp_target.position.z)
  {
    return false;
  }
  descent_target = grasp_target;
  descent_target.position.x = actual_approach.position.x;
  descent_target.position.y = actual_approach.position.y;
  return true;
}

inline bool uses_cartesian_manipulation(uint8_t task_type, uint8_t phase)
{
  const bool manipulation_task =
    task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ||
    task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE;
  const bool cartesian_phase =
    phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING ||
    phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING;
  return manipulation_task && cartesian_phase;
}

inline bool requires_world_vertical_pick_descent_guard(
  uint8_t task_type, uint8_t phase, PickApproachAxis approach_axis)
{
  return task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
         phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING &&
         approach_axis == PickApproachAxis::PLANNING_FRAME_VERTICAL;
}

inline geometry_msgs::msg::Pose cartesian_start_pose(
  const geometry_msgs::msg::Pose & measured_start,
  const geometry_msgs::msg::Pose & target,
  uint8_t task_type)
{
  auto start = measured_start;
  if (task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ||
    task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE)
  {
    start.orientation = target.orientation;
  }
  return start;
}

}  // namespace arm_cell_motion
