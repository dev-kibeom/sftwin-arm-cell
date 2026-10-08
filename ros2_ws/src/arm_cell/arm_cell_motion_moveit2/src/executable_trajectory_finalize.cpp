#include "arm_cell_motion_moveit2/executable_trajectory_finalize.hpp"

#include <cmath>
#include <exception>

#include "arm_cell_motion_moveit2/trajectory_validation.hpp"

namespace arm_cell_motion_moveit2
{

ExecutableTrajectoryFinalizeResult finalize_executable_trajectory(
  moveit_msgs::msg::RobotTrajectory & trajectory,
  const std::vector<std::string> & expected_joint_names,
  const std::vector<double> & velocity_limits,
  const double acceleration_limit,
  const TimeParameterizeFunction & time_parameterize)
{
  if (!time_parameterize) {
    return {false, 0.0, "time_parameterizer_missing"};
  }
  const auto joint_order = validate_trajectory_joint_order(
    trajectory.joint_trajectory.joint_names, expected_joint_names);
  if (!joint_order.valid) {
    return {false, 0.0, joint_order.reason};
  }
  if (trajectory.joint_trajectory.points.empty()) {
    return {false, 0.0, "trajectory_has_no_points"};
  }

  try {
    if (!time_parameterize(trajectory)) {
      return {false, 0.0, "time_parameterization_failed"};
    }
  } catch (const std::exception &) {
    return {false, 0.0, "time_parameterization_exception"};
  }

  const auto & points = trajectory.joint_trajectory.points;
  if (points.empty()) {
    return {false, 0.0, "time_parameterization_returned_no_points"};
  }
  const auto & end = points.back().time_from_start;
  const double duration_s = static_cast<double>(end.sec) +
    static_cast<double>(end.nanosec) * 1e-9;
  if (!std::isfinite(duration_s) || duration_s <= 0.0) {
    return {false, duration_s, "time_parameterization_non_positive_duration"};
  }

  const auto validation = validate_time_parameterized_trajectory(
    trajectory.joint_trajectory, velocity_limits, acceleration_limit);
  if (!validation.valid) {
    return {false, duration_s, validation.reason};
  }
  return {true, duration_s, "valid"};
}

}  // namespace arm_cell_motion_moveit2
