#include "arm_cell_motion_moveit2/trajectory_validation.hpp"

#include <cmath>
#include <sstream>

namespace arm_cell_motion_moveit2
{

TrajectoryValidationResult validate_trajectory_joint_order(
  const std::vector<std::string> & trajectory_joint_names,
  const std::vector<std::string> & configured_joint_names)
{
  if (trajectory_joint_names != configured_joint_names) {
    return {false, "trajectory joint order/count does not exactly match configured arm order"};
  }
  return {true, ""};
}

TrajectoryValidationResult validate_time_parameterized_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & velocity_limits,
  const double acceleration_limit)
{
  if (trajectory.joint_names.empty() || trajectory.points.empty()) {
    return {false, "trajectory has no joints or points"};
  }
  if (velocity_limits.size() != trajectory.joint_names.size()) {
    return {false, "velocity limit count does not match trajectory joint count"};
  }
  if (!std::isfinite(acceleration_limit) || acceleration_limit <= 0.0) {
    return {false, "acceleration policy limit is not positive and finite"};
  }

  std::int64_t previous_time_ns = -1;
  for (std::size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
    const auto & point = trajectory.points[point_index];
    const auto time_ns = static_cast<std::int64_t>(point.time_from_start.sec) * 1000000000LL +
      static_cast<std::int64_t>(point.time_from_start.nanosec);
    if (point_index == 0U) {
      if (time_ns < 0) {
        return {false, "time_from_start is negative"};
      }
    } else if (time_ns <= previous_time_ns) {
      return {false, "time_from_start must be strictly increasing after the initial point"};
    }
    previous_time_ns = time_ns;

    if (point.positions.size() != trajectory.joint_names.size() ||
      point.velocities.size() != trajectory.joint_names.size() ||
      point.accelerations.size() != trajectory.joint_names.size())
    {
      return {false, "trajectory point fields are incomplete"};
    }
    for (std::size_t joint_index = 0; joint_index < trajectory.joint_names.size(); ++joint_index) {
      const auto position = point.positions[joint_index];
      const auto velocity = point.velocities[joint_index];
      const auto acceleration = point.accelerations[joint_index];
      if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(acceleration)) {
        return {false, "trajectory position, velocity, or acceleration is not finite"};
      }
      if (!std::isfinite(velocity_limits[joint_index]) || velocity_limits[joint_index] <= 0.0) {
        return {false, "configured velocity limit is not positive and finite"};
      }
      if (std::abs(velocity) > velocity_limits[joint_index] + 1e-6 ||
        std::abs(acceleration) > acceleration_limit + 1e-6)
      {
        std::ostringstream reason;
        reason << "trajectory velocity or acceleration limit violation at point " << point_index;
        return {false, reason.str()};
      }
    }
  }

  if (previous_time_ns <= 0) {
    return {false, "trajectory final time_from_start must be greater than zero"};
  }
  return {true, "valid"};
}

}  // namespace arm_cell_motion_moveit2
