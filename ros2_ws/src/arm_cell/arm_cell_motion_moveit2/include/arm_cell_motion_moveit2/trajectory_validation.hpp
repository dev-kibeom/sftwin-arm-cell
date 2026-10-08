#pragma once

#include <string>
#include <cstdint>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace arm_cell_motion_moveit2
{

struct TrajectoryValidationResult
{
  bool valid{false};
  std::string reason;
};

TrajectoryValidationResult validate_trajectory_joint_order(
  const std::vector<std::string> & trajectory_joint_names,
  const std::vector<std::string> & configured_joint_names);

TrajectoryValidationResult validate_time_parameterized_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & velocity_limits,
  double acceleration_limit);

}  // namespace arm_cell_motion_moveit2
