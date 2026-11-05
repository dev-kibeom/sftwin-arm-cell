#pragma once

#include <cmath>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace arm_cell_motion::detail
{
inline bool matches_pose(
  const Eigen::Isometry3d & fk, const geometry_msgs::msg::PoseStamped & goal,
  const Eigen::Quaterniond & expected_orientation, bool check_orientation)
{
  const Eigen::Vector3d expected_position(
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  const Eigen::Quaterniond actual_orientation(fk.rotation());
  return fk.matrix().allFinite() &&
         (fk.translation() - expected_position).norm() <= 0.001 &&
         (!check_orientation || std::abs(
           actual_orientation.normalized().dot(
             expected_orientation.normalized())) >= std::cos(0.001 / 2.0));
}
}  // namespace arm_cell_motion::detail
