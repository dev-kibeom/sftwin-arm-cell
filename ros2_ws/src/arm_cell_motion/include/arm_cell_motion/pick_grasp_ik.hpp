#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <vector>

namespace arm_cell_motion
{
struct PickGraspIkResult
{
  bool solver_available{false};
  bool solver_returned{false};
  std::vector<double> joint_positions;
  moveit_msgs::msg::MoveItErrorCodes error;
};

// Performs exactly one caller-selected query; callback validation and retry policy stay outside.
PickGraspIkResult solve_pick_grasp_ik(
  const kinematics::KinematicsBasePtr & solver,
  const geometry_msgs::msg::Pose & goal,
  const std::vector<double> & seed,
  double timeout_s,
  const kinematics::KinematicsBase::IKCallbackFn & validate_solution,
  int initial_error_code);
}  // namespace arm_cell_motion
