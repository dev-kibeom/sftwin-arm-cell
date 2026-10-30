#pragma once

#include <cstdint>
#include <utility>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace arm_cell_motion
{

inline int64_t trajectory_time_ns(const builtin_interfaces::msg::Duration & time)
{
  return static_cast<int64_t>(time.sec) * 1'000'000'000LL + time.nanosec;
}

inline trajectory_msgs::msg::JointTrajectory reverse_joint_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  trajectory_msgs::msg::JointTrajectory reversed;
  reversed.header = trajectory.header;
  reversed.joint_names = trajectory.joint_names;
  if (trajectory.points.empty()) {return reversed;}
  const auto duration_ns = trajectory_time_ns(trajectory.points.back().time_from_start);
  reversed.points.reserve(trajectory.points.size());
  for (auto source = trajectory.points.rbegin(); source != trajectory.points.rend(); ++source) {
    auto point = *source;
    const auto reversed_time_ns = duration_ns - trajectory_time_ns(source->time_from_start);
    point.time_from_start.sec = static_cast<int32_t>(reversed_time_ns / 1'000'000'000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(reversed_time_ns % 1'000'000'000LL);
    for (auto & velocity : point.velocities) {velocity = -velocity;}
    reversed.points.push_back(std::move(point));
  }
  return reversed;
}

}  // namespace arm_cell_motion
