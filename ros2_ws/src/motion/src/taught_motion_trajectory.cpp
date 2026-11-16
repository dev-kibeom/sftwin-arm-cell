#include "arm_cell_motion/taught_motion_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

#include <kdl/velocityprofile_trap.hpp>
#include <urdf/model.h>

namespace arm_cell_motion
{
std::vector<JointMotionLimit> joint_motion_limits_from_urdf(
  const std::string & description, const std::vector<std::string> & joint_names,
  double max_acceleration)
{
  urdf::Model model;
  if (!model.initString(description) || !std::isfinite(max_acceleration) || max_acceleration <= 0) {
    throw std::invalid_argument(
            "Named joint motion requires a valid robot model and acceleration limit");
  }
  std::vector<JointMotionLimit> result;
  for (const auto & name : joint_names) {
    const auto joint = model.getJoint(name);
    if (!joint || !joint->limits || !std::isfinite(joint->limits->velocity) ||
      joint->limits->velocity <= 0)
    {
      throw std::invalid_argument("Named joint motion joint limits unavailable: " + name);
    }
    result.push_back(
      {joint->limits->lower, joint->limits->upper, joint->limits->velocity,
        max_acceleration, joint->type != urdf::Joint::CONTINUOUS});
  }
  return result;
}

TaughtJointTrajectoryResult make_taught_joint_trajectory(
  const std::vector<std::string> & joint_names, const std::vector<double> & start,
  const std::vector<double> & target, const std::vector<JointMotionLimit> & limits,
  double velocity_scale, double acceleration_scale)
{
  const auto positive = [](double value) {return std::isfinite(value) && value > 0;};
  const auto fail = [](const char * reason) {
      TaughtJointTrajectoryResult result;
      result.reason = reason;
      return result;
    };
  if (joint_names.empty() || start.size() != joint_names.size() || target.size() != start.size() ||
    limits.size() != start.size() || !positive(velocity_scale) || velocity_scale > 1 ||
    !positive(acceleration_scale) || acceleration_scale > 1 ||
    std::set<std::string>(joint_names.begin(), joint_names.end()).size() != joint_names.size())
  {
    return fail("invalid_named_joint_motion_input");
  }
  // This is only the output sampling resolution; KDL owns profile generation.
  constexpr double kSamplePeriodS = 0.02;
  constexpr std::size_t kMaxSamples = 100000U;
  double duration = kSamplePeriodS;
  std::vector<KDL::VelocityProfile_Trap> profiles;
  for (std::size_t joint = 0; joint < start.size(); ++joint) {
    const auto & limit = limits[joint];
    if (joint_names[joint].empty() || !std::isfinite(start[joint]) ||
      !std::isfinite(target[joint]) ||
      !positive(limit.max_velocity) || !positive(limit.max_acceleration) ||
      (limit.position_bounded && (!std::isfinite(limit.lower) || !std::isfinite(limit.upper) ||
      limit.lower > limit.upper || start[joint] < limit.lower || start[joint] > limit.upper ||
      target[joint] < limit.lower || target[joint] > limit.upper)))
    {
      return fail("named_joint_motion_outside_limits");
    }
    profiles.emplace_back(
      limit.max_velocity * velocity_scale,
      limit.max_acceleration * acceleration_scale);
    profiles.back().SetProfile(start[joint], target[joint]);
    if (!std::isfinite(profiles.back().Duration())) {return fail("invalid_joint_profile_duration");}
    duration = std::max(duration, profiles.back().Duration());
  }
  const auto intervals = std::ceil(duration / kSamplePeriodS);
  if (!std::isfinite(intervals) || intervals >= kMaxSamples) {
    return fail("named_joint_motion_sample_budget_exceeded");
  }
  const auto count = static_cast<std::size_t>(intervals);
  // Use the same integer time grid for KDL samples and ROS durations. Rounding
  // timestamps independently can make a constant-speed segment exceed its limit.
  const auto step_ns = static_cast<std::int64_t>(std::ceil(duration * 1e9 / count));
  duration = static_cast<double>(step_ns * static_cast<std::int64_t>(count)) * 1e-9;
  for (std::size_t joint = 0; joint < profiles.size(); ++joint) {
    profiles[joint].SetProfileDuration(start[joint], target[joint], duration);
  }
  TaughtJointTrajectoryResult result;
  result.trajectory.joint_names = joint_names;
  for (std::size_t index = 0; index <= count; ++index) {
    const auto ns = static_cast<std::int64_t>(index) * step_ns;
    const auto time = static_cast<double>(ns) * 1e-9;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    for (const auto & profile : profiles) {
      point.positions.push_back(profile.Pos(time));
      point.velocities.push_back(profile.Vel(time));
      point.accelerations.push_back(profile.Acc(time));
    }
    result.trajectory.points.push_back(std::move(point));
  }
  result.trajectory.points.front().positions = start;
  auto & final = result.trajectory.points.back();
  final.positions = target;
  final.velocities.assign(start.size(), 0);
  final.accelerations.assign(start.size(), 0);
  result.valid = true;
  return result;
}
}  // namespace arm_cell_motion
