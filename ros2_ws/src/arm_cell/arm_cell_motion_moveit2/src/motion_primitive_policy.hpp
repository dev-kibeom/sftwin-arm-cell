#pragma once

#include <cstdint>

#include <geometry_msgs/msg/pose.hpp>

#include "arm_cell_interfaces/msg/motion_task_phase.hpp"
#include "arm_cell_interfaces/msg/motion_task_type.hpp"

namespace arm_cell_motion_moveit2
{

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

inline bool uses_validated_approach_start_state(uint8_t task_type, uint8_t phase)
{
  return (task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ||
         task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE) &&
         phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING;
}

inline bool requires_approach_entry_acceptance(uint8_t task_type, uint8_t phase)
{
  const bool manipulation_task =
    task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ||
    task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE;
  return manipulation_task &&
         phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
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

}  // namespace arm_cell_motion_moveit2
