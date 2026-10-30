#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace arm_cell_motion_moveit2
{

struct TrajectorySampleResult
{
  bool valid{false};
  std::string reason;
};

struct TimedJointSample
{
  std::vector<double> positions;
  std::vector<double> velocities;
  std::size_t segment_index{0};
  double trajectory_time{0.0};
};

class TrajectorySampler
{
public:
  virtual ~TrajectorySampler() = default;

  virtual TrajectorySampleResult sample(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    double trajectory_time,
    std::vector<double> & positions,
    std::size_t & segment_index) const = 0;

  virtual TrajectorySampleResult sample_timed(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    double trajectory_time,
    TimedJointSample & sample) const = 0;
};

// Linear sampling preserves the geometric waypoint segments and their timing.
// Its velocity is the derivative of that exact piecewise-linear position path;
// it does not inject TOTG waypoint velocities or accelerations.
class LinearTrajectorySampler final : public TrajectorySampler
{
public:
  explicit LinearTrajectorySampler(std::vector<double> velocity_limits = {})
  : velocity_limits_(std::move(velocity_limits)) {}

  TrajectorySampleResult sample(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    double trajectory_time,
    std::vector<double> & positions,
    std::size_t & segment_index) const override;

  TrajectorySampleResult sample_timed(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    double trajectory_time,
    TimedJointSample & sample) const override;

private:
  std::vector<double> velocity_limits_;
};

std::shared_ptr<TrajectorySampler> make_trajectory_sampler(const std::string & name);

}  // namespace arm_cell_motion_moveit2
