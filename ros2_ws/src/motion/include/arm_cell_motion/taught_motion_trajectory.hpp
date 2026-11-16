#pragma once

#include <string>
#include <vector>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace arm_cell_motion
{
struct JointMotionLimit
{
  double lower;
  double upper;
  double max_velocity;
  double max_acceleration;
  bool position_bounded{true};
};

struct TaughtJointTrajectoryResult
{
  bool valid{false};
  std::string reason;
  trajectory_msgs::msg::JointTrajectory trajectory;
};

std::vector<JointMotionLimit> joint_motion_limits_from_urdf(
  const std::string & description, const std::vector<std::string> & joint_names,
  double max_acceleration);

// Standard synchronized KDL trapezoidal profiles, sampled for the existing
// trajectory worker. No IK, collision-scene lookup, or path planner is used.
TaughtJointTrajectoryResult make_taught_joint_trajectory(
  const std::vector<std::string> & joint_names, const std::vector<double> & start,
  const std::vector<double> & target, const std::vector<JointMotionLimit> & limits,
  double velocity_scale, double acceleration_scale);
}  // namespace arm_cell_motion
