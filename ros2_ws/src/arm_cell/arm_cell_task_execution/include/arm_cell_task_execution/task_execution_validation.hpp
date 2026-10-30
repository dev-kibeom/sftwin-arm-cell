#pragma once

#include <cmath>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "arm_cell_motion/motion_backend.hpp"
#include "arm_cell_task_execution/task_execution_report.hpp"

namespace arm_cell_task_execution::task_execution_validation
{

inline bool finite(double value)
{
  return std::isfinite(value);
}

inline bool valid_unit_vector(const geometry_msgs::msg::Vector3 & vector)
{
  const auto norm_squared = vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
  return finite(vector.x) && finite(vector.y) && finite(vector.z) && finite(norm_squared) &&
         std::abs(norm_squared - 1.0) <= 1e-6;
}

inline bool valid_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const auto q_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return pose.header.frame_id == "base_link" && finite(p.x) && finite(p.y) && finite(p.z) &&
         finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) && finite(q_norm) &&
         std::abs(q_norm - 1.0) <= 1e-6;
}

inline bool valid_observation_position(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  return pose.header.frame_id == "base_link" && finite(p.x) && finite(p.y) && finite(p.z);
}

inline bool valid_unit_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const auto norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) && finite(norm_squared) &&
         std::abs(norm_squared - 1.0) <= 1e-6;
}

inline PlaceOrientationConstraint place_orientation_constraint(std::uint8_t value)
{
  using Goal = ExecuteTask::Goal;
  switch (value) {
    case Goal::PLACE_ORIENTATION_FIXED:
      return PlaceOrientationConstraint::FIXED;
    case Goal::PLACE_ORIENTATION_BOUNDED:
      return PlaceOrientationConstraint::BOUNDED;
    case Goal::PLACE_ORIENTATION_FREE:
      return PlaceOrientationConstraint::FREE;
    default:
      return PlaceOrientationConstraint::FIXED;
  }
}

inline bool valid_place_orientation_constraint(std::uint8_t value)
{
  using Goal = ExecuteTask::Goal;
  return value == Goal::PLACE_ORIENTATION_FIXED || value == Goal::PLACE_ORIENTATION_BOUNDED ||
         value == Goal::PLACE_ORIENTATION_FREE;
}

}  // namespace arm_cell_task_execution::task_execution_validation
