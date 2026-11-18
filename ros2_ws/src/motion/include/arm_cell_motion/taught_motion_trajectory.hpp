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

// Uniform time dilation keeps the planned path while bounding timed fields
// and the piecewise-linear executor's segment velocities.
bool stretch_taught_trajectory_to_limits(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & velocity_limits,
  const std::vector<double> & acceleration_limits);

std::vector<JointMotionLimit> joint_motion_limits_from_urdf(
  const std::string & description, const std::vector<std::string> & joint_names,
  double max_acceleration);

TaughtJointTrajectoryResult make_taught_path_trajectory(
  const std::vector<std::string> & joint_names, const std::vector<double> & start,
  const std::vector<std::vector<double>> & path, const std::vector<JointMotionLimit> & limits,
  double duration_s, double velocity_scale, double acceleration_scale);

// Standard synchronized KDL trapezoidal profiles, sampled for the existing
// trajectory worker. No IK, collision-scene lookup, or path planner is used.
TaughtJointTrajectoryResult make_taught_joint_trajectory(
  const std::vector<std::string> & joint_names, const std::vector<double> & start,
  const std::vector<double> & target, const std::vector<JointMotionLimit> & limits,
  double velocity_scale, double acceleration_scale);
}  // namespace arm_cell_motion
