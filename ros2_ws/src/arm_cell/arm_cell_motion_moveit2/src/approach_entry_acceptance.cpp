#include "arm_cell_motion_moveit2/approach_entry_acceptance.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace arm_cell_motion_moveit2
{

ActualFeedbackStabilityTracker::ActualFeedbackStabilityTracker(
  double max_joint_delta_rad, std::size_t required_samples)
: max_joint_delta_rad_(max_joint_delta_rad), required_samples_(required_samples)
{
}

void ActualFeedbackStabilityTracker::reset()
{
  previous_positions_.clear();
  stable_sample_count_ = 0;
}

bool ActualFeedbackStabilityTracker::observe(const std::vector<double> & positions)
{
  if (positions.empty() || !std::all_of(
      positions.begin(), positions.end(),
      [](double value) {return std::isfinite(value);}))
  {
    reset();
    return false;
  }
  if (previous_positions_.size() != positions.size()) {
    previous_positions_ = positions;
    stable_sample_count_ = 1;
    return settled();
  }
  const auto max_delta = [&]() {
      double result = 0.0;
      for (std::size_t index = 0; index < positions.size(); ++index) {
        result = std::max(result, std::abs(positions[index] - previous_positions_[index]));
      }
      return result;
    }();
  stable_sample_count_ = max_delta <= max_joint_delta_rad_ ? stable_sample_count_ + 1 : 1;
  previous_positions_ = positions;
  return settled();
}

bool ActualFeedbackStabilityTracker::settled() const
{
  return required_samples_ > 0 && stable_sample_count_ >= required_samples_;
}

ApproachEntryAcceptance evaluate_approach_entry(
  const geometry_msgs::msg::Pose & requested,
  const geometry_msgs::msg::Pose & actual,
  bool settled,
  bool fresh_feedback,
  bool state_valid,
  const ApproachEntryPolicy & policy)
{
  ApproachEntryAcceptance result;
  result.settled = settled;
  result.fresh_feedback = fresh_feedback;
  result.state_valid = state_valid;
  const auto dx = actual.position.x - requested.position.x;
  const auto dy = actual.position.y - requested.position.y;
  const auto dz = actual.position.z - requested.position.z;
  result.position_error_m = std::sqrt(dx * dx + dy * dy + dz * dz);

  const Eigen::Quaterniond requested_q(
    requested.orientation.w, requested.orientation.x, requested.orientation.y,
    requested.orientation.z);
  const Eigen::Quaterniond actual_q(
    actual.orientation.w, actual.orientation.x, actual.orientation.y, actual.orientation.z);
  if (requested_q.norm() == 0.0 || actual_q.norm() == 0.0) {
    result.reason = "invalid orientation quaternion";
    return result;
  }
  const auto normalized_requested = requested_q.normalized();
  const auto normalized_actual = actual_q.normalized();
  const auto dot = std::clamp(
    std::abs(normalized_requested.dot(normalized_actual)), 0.0, 1.0);
  result.orientation_error_rad = 2.0 * std::acos(dot);

  if (!fresh_feedback) {
    result.reason = "actual approach feedback is stale";
  } else if (!state_valid) {
    result.reason = "actual approach state is invalid or in collision";
  } else if (result.position_error_m > policy.position_tolerance_m) {
    result.reason = "actual TCP is outside approach position tolerance";
  } else if (result.orientation_error_rad > policy.orientation_tolerance_rad) {
    result.reason = "actual TCP is outside approach orientation tolerance";
  } else if (policy.require_settled && !settled) {
    result.reason = "actual approach state is not settled";
  } else {
    result.accepted = true;
    result.reason = "accepted";
  }
  return result;
}

bool joint_positions_within_tolerance(
  const std::vector<double> & expected,
  const std::vector<double> & measured,
  double tolerance)
{
  if (expected.empty() || expected.size() != measured.size() ||
    !std::isfinite(tolerance) || tolerance < 0.0)
  {
    return false;
  }
  return std::equal(
    expected.begin(), expected.end(), measured.begin(),
    [tolerance](double expected_value, double measured_value) {
      return std::isfinite(expected_value) && std::isfinite(measured_value) &&
      std::abs(expected_value - measured_value) <= tolerance;
    });
}

double select_trajectory_tracking_tolerance(
  bool is_place, bool is_approaching, double place_tolerance_rad)
{
  return is_place && is_approaching ? place_tolerance_rad : 0.01;
}

double select_approach_entry_position_tolerance(
  bool is_place, bool is_approaching,
  double default_tolerance_m, double place_tolerance_m)
{
  return is_place && is_approaching ? place_tolerance_m : default_tolerance_m;
}

}  // namespace arm_cell_motion_moveit2
