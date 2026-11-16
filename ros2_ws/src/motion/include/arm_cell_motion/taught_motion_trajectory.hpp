#pragma once
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace arm_cell_motion
{
// Stretch a trajectory parameterized at robot limits to obey both requested
// limits. Uniform time scaling preserves the planned joint path.
bool scale_taught_trajectory(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  double velocity_scale, double acceleration_scale);
}  // namespace arm_cell_motion
