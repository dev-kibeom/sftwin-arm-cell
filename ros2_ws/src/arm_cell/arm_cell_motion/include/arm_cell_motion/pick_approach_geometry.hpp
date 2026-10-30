#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "arm_cell_motion/pick_approach_axis.hpp"

namespace arm_cell_motion
{

template<typename Candidate, typename TryCandidate>
std::optional<Candidate> first_successful_candidate(
  const std::vector<Candidate> & candidates, TryCandidate try_candidate)
{
  for (const auto & candidate : candidates) {
    if (try_candidate(candidate)) {
      return candidate;
    }
  }
  return std::nullopt;
}

inline geometry_msgs::msg::PoseStamped pick_approach_pose(
  const geometry_msgs::msg::PoseStamped & grasp, PickApproachAxis approach_axis,
  const geometry_msgs::msg::Vector3 & insertion_axis_tcp, double distance)
{
  auto approach = grasp;
  if (approach_axis == PickApproachAxis::PLANNING_FRAME_VERTICAL) {
    // Grasp is the lower endpoint; Approach is one configured distance above it.
    approach.pose.position.z += distance;
    return approach;
  }
  const Eigen::Quaterniond rotation(
    grasp.pose.orientation.w, grasp.pose.orientation.x,
    grasp.pose.orientation.y, grasp.pose.orientation.z);
  Eigen::Vector3d axis = rotation.normalized() * Eigen::Vector3d(
    insertion_axis_tcp.x, insertion_axis_tcp.y, insertion_axis_tcp.z);
  axis.normalize();
  approach.pose.position.x -= distance * axis.x();
  approach.pose.position.y -= distance * axis.y();
  approach.pose.position.z -= distance * axis.z();
  return approach;
}

}  // namespace arm_cell_motion
