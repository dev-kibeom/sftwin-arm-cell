#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arm_cell_motion_moveit2/trajectory_continuity.hpp"

namespace arm_cell_motion_moveit2
{

using CartesianIkSolver = std::function<bool (
      const geometry_msgs::msg::Pose &,
      const std::vector<double> &,
      std::vector<double> &, std::string &)>;

using CartesianCollisionChecker = std::function<bool (
      std::size_t,
      const geometry_msgs::msg::Pose &,
      const std::vector<double> &, std::string &)>;

using CartesianExecutableStateValidator = std::function<bool (
      std::size_t,
      const std::vector<double> &, std::string &, std::vector<std::string> &)>;

struct CartesianSampleFailure
{
  std::size_t sample_index{0U};
  geometry_msgs::msg::Pose tcp;
  std::string reason;
  std::vector<std::string> contact_bodies;
};

struct CartesianLinearGenerationResult
{
  bool valid{false};
  double path_fraction{0.0};
  std::size_t sample_count{0U};
  std::string reason;
  CartesianSampleFailure failure;
  trajectory_msgs::msg::JointTrajectory trajectory;
  JointTrajectoryContinuityResult continuity;
};

CartesianLinearGenerationResult generate_cartesian_linear_trajectory(
  const geometry_msgs::msg::Pose & start_tcp,
  const geometry_msgs::msg::Pose & target_tcp,
  const std::vector<double> & start_positions,
  const std::vector<std::string> & joint_names,
  const std::vector<JointContinuityLimit> & limits,
  double spatial_step_m,
  const CartesianIkSolver & solve_ik,
  const CartesianCollisionChecker & check_collision);

CartesianLinearGenerationResult validate_cartesian_joint_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & measured_start,
  const std::vector<JointContinuityLimit> & limits,
  const CartesianExecutableStateValidator & validate_state);

}  // namespace arm_cell_motion_moveit2
