#include "arm_cell_motion_moveit2/trajectory_sampler.hpp"

#include <cmath>
#include <memory>
#include <sstream>

namespace arm_cell_motion_moveit2
{
namespace
{
double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) / 1e9;
}

TrajectorySampleResult validate_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (trajectory.joint_names.empty() || trajectory.points.empty()) {
    return {false, "trajectory must contain joint names and points"};
  }
  const auto joint_count = trajectory.joint_names.size();
  double previous_time = -1.0;
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points[index];
    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000U) {
      return {false, "trajectory contains an invalid time_from_start"};
    }
    const auto time = duration_seconds(point.time_from_start);
    if (!std::isfinite(time) || time < 0.0 || (index > 0 && time <= previous_time)) {
      return {false, "trajectory time_from_start must be strictly increasing"};
    }
    if (point.positions.size() != joint_count) {
      return {false, "trajectory point position count does not match joint count"};
    }
    for (const auto position : point.positions) {
      if (!std::isfinite(position)) {
        return {false, "trajectory contains a non-finite joint position"};
      }
    }
    previous_time = time;
  }
  return {true, ""};
}
}  // namespace

TrajectorySampleResult LinearTrajectorySampler::sample(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  double trajectory_time,
  std::vector<double> & positions,
  std::size_t & segment_index) const
{
  TimedJointSample timed_sample;
  const auto result = sample_timed(trajectory, trajectory_time, timed_sample);
  positions = std::move(timed_sample.positions);
  segment_index = timed_sample.segment_index;
  return result;
}

TrajectorySampleResult LinearTrajectorySampler::sample_timed(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  double trajectory_time,
  TimedJointSample & sample) const
{
  const auto validation = validate_trajectory(trajectory);
  if (!validation.valid) {
    return validation;
  }
  if (!std::isfinite(trajectory_time)) {
    return {false, "trajectory sample time must be finite"};
  }

  const auto & points = trajectory.points;
  for (std::size_t index = 0; index + 1 < points.size(); ++index) {
    const auto previous_time = duration_seconds(points[index].time_from_start);
    const auto current_time = duration_seconds(points[index + 1].time_from_start);
    const auto segment_duration = current_time - previous_time;
    if (!(segment_duration > 0.0) || !std::isfinite(segment_duration)) {
      return {false, "trajectory segment duration is invalid"};
    }
    for (std::size_t joint = 0; joint < points[index].positions.size(); ++joint) {
      const auto velocity =
        (points[index + 1].positions[joint] - points[index].positions[joint]) /
        segment_duration;
      if (!std::isfinite(velocity)) {
        return {false, "derived trajectory velocity is non-finite"};
      }
      if (!velocity_limits_.empty() &&
        (joint >= velocity_limits_.size() || !std::isfinite(velocity_limits_[joint]) ||
        velocity_limits_[joint] < 0.0 || std::abs(velocity) > velocity_limits_[joint] + 1e-9))
      {
        return {false, "derived trajectory velocity exceeds velocity limit"};
      }
    }
  }

  if (trajectory_time <= duration_seconds(points.front().time_from_start)) {
    sample.positions = points.front().positions;
    sample.segment_index = 0;
    sample.trajectory_time = trajectory_time;
  } else {
    bool sampled = false;
    for (std::size_t index = 1; index < points.size(); ++index) {
      const auto previous_time = duration_seconds(points[index - 1].time_from_start);
      const auto current_time = duration_seconds(points[index].time_from_start);
      if (trajectory_time <= current_time) {
        const auto fraction = (trajectory_time - previous_time) / (current_time - previous_time);
        sample.positions.resize(points[index].positions.size());
        for (std::size_t joint = 0; joint < sample.positions.size(); ++joint) {
          sample.positions[joint] = points[index - 1].positions[joint] +
            fraction * (points[index].positions[joint] - points[index - 1].positions[joint]);
        }
        if (trajectory_time == current_time) {
          sample.positions = points[index].positions;
        }
        sample.segment_index = index - 1;
        sample.trajectory_time = trajectory_time;
        sampled = true;
        break;
      }
    }
    if (!sampled) {
      sample.positions = points.back().positions;
      sample.segment_index = points.size() > 1 ? points.size() - 2 : 0;
      sample.trajectory_time = trajectory_time;
    }
  }

  const auto final_time = duration_seconds(points.back().time_from_start);
  sample.velocities.assign(sample.positions.size(), 0.0);
  if (trajectory_time < final_time && points.size() > 1) {
    const auto index = sample.segment_index;
    const auto previous_time = duration_seconds(points[index].time_from_start);
    const auto current_time = duration_seconds(points[index + 1].time_from_start);
    const auto segment_duration = current_time - previous_time;
    if (!(segment_duration > 0.0) || !std::isfinite(segment_duration)) {
      return {false, "trajectory segment duration is invalid"};
    }
    for (std::size_t joint = 0; joint < sample.velocities.size(); ++joint) {
      const auto velocity =
        (points[index + 1].positions[joint] - points[index].positions[joint]) /
        segment_duration;
      if (!std::isfinite(velocity)) {
        return {false, "derived trajectory velocity is non-finite"};
      }
      if (!velocity_limits_.empty()) {
        if (joint >= velocity_limits_.size() || !std::isfinite(velocity_limits_[joint]) ||
          velocity_limits_[joint] < 0.0 || std::abs(velocity) > velocity_limits_[joint] + 1e-9)
        {
          return {false, "derived trajectory velocity exceeds velocity limit"};
        }
      }
      sample.velocities[joint] = velocity;
    }
  }

  return {true, ""};
}

std::shared_ptr<TrajectorySampler> make_trajectory_sampler(const std::string & name)
{
  if (name == "linear") {
    return std::make_shared<LinearTrajectorySampler>();
  }
  return nullptr;
}
}  // namespace arm_cell_motion_moveit2
