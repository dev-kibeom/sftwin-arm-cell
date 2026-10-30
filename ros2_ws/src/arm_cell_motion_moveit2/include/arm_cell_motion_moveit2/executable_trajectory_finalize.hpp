#pragma once

#include <functional>
#include <string>
#include <vector>

#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace arm_cell_motion_moveit2
{

struct ExecutableTrajectoryFinalizeResult
{
  bool success{false};
  double duration_s{0.0};
  std::string reason;
};

using TimeParameterizeFunction = std::function<bool(moveit_msgs::msg::RobotTrajectory &)>;

ExecutableTrajectoryFinalizeResult finalize_executable_trajectory(
  moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::vector<std::string> & expected_joint_names,
  const std::vector<double> & velocity_limits,
  double acceleration_limit,
  const TimeParameterizeFunction & time_parameterize);

}  // namespace arm_cell_motion_moveit2
