#include "arm_cell_motion_moveit2/trajectory_continuity.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arm_cell_motion_moveit2
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kContinuityStepLimit = M_PI;
constexpr double kLimitTolerance = 1e-9;

bool in_bounds(double value, const JointContinuityLimit & limit)
{
  return value >= limit.lower - kLimitTolerance && value <= limit.upper + kLimitTolerance;
}

bool finite_vector(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

bool resolve_nearest(
  double raw, double previous, const JointContinuityLimit & limit, double & resolved)
{
  if (!limit.wrap_equivalent) {
    if (!in_bounds(raw, limit)) {
      return false;
    }
    resolved = raw;
    return true;
  }

  const auto nearest_turn = static_cast<int>(std::llround((previous - raw) / kTwoPi));
  double best_distance = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int turn = nearest_turn - 2; turn <= nearest_turn + 2; ++turn) {
    const double candidate = raw + static_cast<double>(turn) * kTwoPi;
    if (!in_bounds(candidate, limit)) {
      continue;
    }
    const double distance = std::abs(candidate - previous);
    if (distance < best_distance) {
      best_distance = distance;
      resolved = candidate;
      found = true;
    }
  }
  return found;
}
}  // namespace

JointGoalSelectionResult select_nearest_joint_goal(
  const std::vector<double> & raw_goal,
  const std::vector<double> & measured_start,
  const std::vector<JointContinuityLimit> & limits,
  const JointGoalStateValidator & state_validator)
{
  JointGoalSelectionResult result;
  if (raw_goal.empty() || raw_goal.size() != measured_start.size() ||
    limits.size() != measured_start.size())
  {
    result.reason = "goal continuity input is empty or dimensionally inconsistent";
    return result;
  }
  if (!finite_vector(raw_goal) || !finite_vector(measured_start)) {
    result.reason = "goal continuity input contains a non-finite value";
    return result;
  }

  result.raw_goal = raw_goal;
  result.selected_goal.resize(raw_goal.size());
  result.raw_endpoint_delta.resize(raw_goal.size());
  result.wrap_aware_endpoint_delta.resize(raw_goal.size());
  result.selected_endpoint_delta.resize(raw_goal.size());
  result.selected_wrap_aware_endpoint_delta.resize(raw_goal.size());
  for (std::size_t index = 0; index < raw_goal.size(); ++index) {
    const auto & limit = limits[index];
    result.raw_endpoint_delta[index] = raw_goal[index] - measured_start[index];
    result.wrap_aware_endpoint_delta[index] = limit.wrap_equivalent ?
      std::remainder(result.raw_endpoint_delta[index], kTwoPi) :
      result.raw_endpoint_delta[index];
    if (!resolve_nearest(
        raw_goal[index], measured_start[index], limit,
        result.selected_goal[index]))
    {
      result.reason = "goal candidate is outside joint bounds";
      return result;
    }
    result.selected_endpoint_delta[index] =
      result.selected_goal[index] - measured_start[index];
    result.selected_wrap_aware_endpoint_delta[index] = limit.wrap_equivalent ?
      std::remainder(result.selected_endpoint_delta[index], kTwoPi) :
      result.selected_endpoint_delta[index];
    if (limit.wrap_equivalent) {
      if (std::abs(result.selected_endpoint_delta[index]) -
        std::abs(result.selected_wrap_aware_endpoint_delta[index]) >= kTwoPi - 0.05)
      {
        result.unnecessary_full_turn = true;
      }
    }
    if (std::abs(result.selected_goal[index] - measured_start[index]) > kContinuityStepLimit) {
      result.reason = "selected goal requires a discontinuous joint step";
      return result;
    }
    if (std::abs(result.selected_goal[index] - raw_goal[index]) > kLimitTolerance) {
      result.normalized = true;
    }
  }
  if (result.unnecessary_full_turn) {
    result.reason = "selected goal retains an unnecessary wrap-equivalent full turn";
    return result;
  }
  if (state_validator) {
    std::string validation_reason;
    if (!state_validator(result.selected_goal, validation_reason)) {
      result.reason =
        validation_reason.empty() ? "selected goal state is invalid" : validation_reason;
      return result;
    }
  }
  result.valid = true;
  return result;
}

JointTrajectoryContinuityResult normalize_joint_trajectory(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & measured_start,
  const std::vector<JointContinuityLimit> & limits,
  bool allow_representation_normalization)
{
  JointTrajectoryContinuityResult result;
  const auto joint_count = measured_start.size();
  if (joint_count == 0U || limits.size() != joint_count || trajectory.points.empty()) {
    result.reason = "continuity input is empty or dimensionally inconsistent";
    return result;
  }
  if (!finite_vector(measured_start)) {
    result.reason = "measured continuity start contains a non-finite value";
    return result;
  }
  for (const auto & point : trajectory.points) {
    if (point.positions.size() != joint_count || !finite_vector(point.positions)) {
      result.reason = "trajectory point positions are dimensionally inconsistent or non-finite";
      return result;
    }
  }

  result.raw_endpoint_delta.resize(joint_count);
  result.wrap_aware_endpoint_delta.resize(joint_count);
  result.normalized_joints.assign(joint_count, false);
  const auto & raw_endpoint = trajectory.points.back().positions;
  for (std::size_t index = 0; index < joint_count; ++index) {
    result.raw_endpoint_delta[index] = raw_endpoint[index] - measured_start[index];
    result.wrap_aware_endpoint_delta[index] = limits[index].wrap_equivalent ?
      std::remainder(result.raw_endpoint_delta[index], kTwoPi) :
      result.raw_endpoint_delta[index];
  }

  std::vector<double> previous = measured_start;
  std::vector<double> raw_previous = measured_start;
  std::vector<double> raw_cumulative_motion(joint_count, 0.0);
  for (auto & point : trajectory.points) {
    for (std::size_t index = 0; index < joint_count; ++index) {
      raw_cumulative_motion[index] += std::abs(point.positions[index] - raw_previous[index]);
      raw_previous[index] = point.positions[index];
      double resolved = 0.0;
      if (allow_representation_normalization) {
        if (!resolve_nearest(point.positions[index], previous[index], limits[index], resolved)) {
          result.reason = "trajectory point cannot be resolved within joint bounds";
          return result;
        }
      } else {
        resolved = point.positions[index];
        if (!in_bounds(resolved, limits[index])) {
          result.reason = "trajectory point is outside joint bounds";
          return result;
        }
      }
      if (std::abs(resolved - previous[index]) > kContinuityStepLimit) {
        result.reason = "trajectory contains a discontinuous joint step";
        return result;
      }
      if (allow_representation_normalization &&
        std::abs(resolved - point.positions[index]) > kLimitTolerance)
      {
        result.normalized = true;
        result.normalized_joints[index] = true;
      }
      point.positions[index] = resolved;
      previous[index] = resolved;
    }
  }

  if (!allow_representation_normalization) {
    for (std::size_t index = 0; index < joint_count; ++index) {
      if (!limits[index].wrap_equivalent) {
        continue;
      }
      const double wrap_aware_endpoint = result.wrap_aware_endpoint_delta[index];
      if (raw_cumulative_motion[index] - std::abs(wrap_aware_endpoint) >= kTwoPi - 0.05) {
        result.reason = "trajectory accumulates an unnecessary wrap-equivalent full turn";
        return result;
      }
    }
  }

  result.raw_cumulative_motion = std::move(raw_cumulative_motion);
  result.valid = true;
  return result;
}

}  // namespace arm_cell_motion_moveit2
