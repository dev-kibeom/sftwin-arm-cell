#pragma once

#include <cstdint>
#include <cmath>
#include "arm_cell_motion/taught_motion_trajectory.hpp"
#include <functional>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace arm_cell_motion
{
// A full turn is geometrically equivalent, but only choose equivalents inside
// the configured position limits and closer to the measured seed.
inline double nearest_revolute_equivalent(
  double position, double seed, const moveit::core::VariableBounds & bounds)
{
  constexpr double turn = 6.28318530717958647692;
  const double nearest = position + turn * std::round((seed - position) / turn);
  double selected = position;
  for (double candidate : {nearest - turn, nearest, nearest + turn}) {
    if ((!bounds.position_bounded_ ||
      (candidate >= bounds.min_position_ && candidate <= bounds.max_position_)) &&
      std::abs(candidate - seed) < std::abs(selected - seed)) {selected = candidate;}
  }
  return selected;
}

// Keep a movement-specific IK allocator separate from the planner's model.
moveit::core::RobotModelPtr make_kinematic_model(
  const moveit::core::RobotModelConstPtr & source, const std::string & group,
  const kinematics::KinematicsBasePtr & solver);

// Targets are expressed in the robot model frame. These operations use only
// kinematics and joint bounds; neither requires a Scene or a planner service.
struct LinearJointChangeDiagnostics
{
  bool available{false};
  double relative_threshold_factor{1.5};
  double average_step_distance{0.0};
  double maximum_step_distance{0.0};
  std::size_t maximum_step_end_index{0};  // Zero-based trajectory point index.
  std::size_t exceeding_step_count{0};
  std::vector<std::string> joint_names;
  std::vector<double> maximum_step_joint_deltas;
};

std::uint8_t resolve_pose_joint_target(
  const moveit::core::RobotState & start, const std::string & group,
  const std::string & tcp_link, const Eigen::Isometry3d & target,
  std::vector<double> & joints);
std::uint8_t generate_linear_joint_trajectory(
  const moveit::core::RobotState & start, const std::string & group,
  const std::string & tcp_link, const Eigen::Isometry3d & target,
  moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::function<std::uint8_t()> & interruption = {},
  LinearJointChangeDiagnostics * diagnostics = nullptr);
bool retime_joint_trajectory(
  const moveit::core::RobotModelConstPtr & model, const std::string & group,
  const moveit::core::RobotState & start, moveit_msgs::msg::RobotTrajectory & trajectory,
  double velocity_scale, double acceleration_scale,
  const std::vector<JointMotionLimit> * limits = nullptr);
}  // namespace arm_cell_motion
