#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

namespace arm_cell_motion_moveit2
{

struct ApproachEntryPolicy
{
  double position_tolerance_m{0.005};
  double orientation_tolerance_rad{0.08726646259971647};
  bool require_settled{true};
};

struct ApproachEntryAcceptance
{
  bool accepted{false};
  bool settled{false};
  bool fresh_feedback{false};
  bool state_valid{false};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
  std::string reason;
};

class ActualFeedbackStabilityTracker final
{
public:
  ActualFeedbackStabilityTracker(double max_joint_delta_rad, std::size_t required_samples);

  void reset();
  bool observe(const std::vector<double> & positions);
  bool settled() const;

private:
  double max_joint_delta_rad_{0.0};
  std::size_t required_samples_{0};
  std::vector<double> previous_positions_;
  std::size_t stable_sample_count_{0};
};

ApproachEntryAcceptance evaluate_approach_entry(
  const geometry_msgs::msg::Pose & requested,
  const geometry_msgs::msg::Pose & actual,
  bool settled,
  bool fresh_feedback,
  bool state_valid,
  const ApproachEntryPolicy & policy);

bool joint_positions_within_tolerance(
  const std::vector<double> & expected,
  const std::vector<double> & measured,
  double tolerance);

double select_trajectory_tracking_tolerance(
  bool is_place, bool is_approaching, double place_tolerance_rad);
double select_approach_entry_position_tolerance(
  bool is_place, bool is_approaching,
  double default_tolerance_m, double place_tolerance_m);

}  // namespace arm_cell_motion_moveit2
