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
bool stretch_taught_trajectory_to_limits(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<double> & velocities, const std::vector<double> & accelerations)
{
  const auto count = trajectory.joint_names.size();
  if (count == 0 || velocities.size() != count || accelerations.size() != count ||
    trajectory.points.size() < 2) {return false;}
  double stretch = 1.0;
  std::int64_t previous_ns = -1;
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i];
    const auto ns = static_cast<std::int64_t>(point.time_from_start.sec) * 1000000000LL +
      point.time_from_start.nanosec;
    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000U ||
      ns <= previous_ns || point.positions.size() != count ||
      point.velocities.size() != count || point.accelerations.size() != count) {return false;}
    for (std::size_t j = 0; j < count; ++j) {
      if (!std::isfinite(velocities[j]) || velocities[j] <= 0 ||
        !std::isfinite(accelerations[j]) || accelerations[j] <= 0 ||
        !std::isfinite(point.positions[j]) || !std::isfinite(point.velocities[j]) ||
        !std::isfinite(point.accelerations[j])) {return false;}
      stretch = std::max(stretch, std::abs(point.velocities[j]) / velocities[j]);
      stretch = std::max(stretch, std::sqrt(std::abs(point.accelerations[j]) / accelerations[j]));
      if (i > 0) {
        const double dt = (ns - previous_ns) * 1e-9;
        const auto segment_velocity =
          (point.positions[j] - trajectory.points[i - 1].positions[j]) / dt;
        stretch = std::max(stretch, std::abs(segment_velocity) / velocities[j]);
      }
    }
    previous_ns = ns;
  }
  if (!std::isfinite(stretch) || previous_ns <= 0) {return false;}
  if (stretch == 1.0) {return true;}
  // Dilation changes only time: velocity scales by 1/s and acceleration by
  // 1/s^2. Keep every collision-checked position and round timestamps upward.
  auto adjusted = trajectory;
  std::int64_t original_previous_ns = 0, adjusted_previous_ns = 0;
  for (auto & point : adjusted.points) {
    const auto ns = static_cast<std::int64_t>(point.time_from_start.sec) * 1000000000LL +
      point.time_from_start.nanosec;
    const auto scaled_ns = adjusted_previous_ns +
      std::ceil(static_cast<long double>(ns - original_previous_ns) * stretch);
    constexpr auto max_seconds = std::numeric_limits<std::int32_t>::max();
    if (scaled_ns > static_cast<long double>(max_seconds) * 1000000000LL) {return false;}
    const auto duration_ns = static_cast<std::int64_t>(scaled_ns);
    original_previous_ns = ns;
    adjusted_previous_ns = duration_ns;
    point.time_from_start.sec = static_cast<std::int32_t>(duration_ns / 1000000000LL);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(duration_ns % 1000000000LL);
    for (std::size_t j = 0; j < count; ++j) {
      point.velocities[j] /= stretch;
      point.accelerations[j] = point.accelerations[j] / stretch / stretch;
    }
  }
  trajectory = std::move(adjusted);
  return true;
}

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

TaughtJointTrajectoryResult make_taught_path_trajectory(
  const std::vector<std::string> & names, const std::vector<double> & start,
  const std::vector<std::vector<double>> & path, const std::vector<JointMotionLimit> & limits,
  double duration, double velocity_scale, double acceleration_scale)
{
  TaughtJointTrajectoryResult result;
  result.reason = "invalid_taught_path";
  if (names.empty() || start.size() != names.size() || limits.size() != names.size() ||
    path.size() < 2 || path.size() > 100000 || !std::isfinite(duration) || duration <= 0 || duration > 1800 ||
    !std::isfinite(velocity_scale) || velocity_scale <= 0 || velocity_scale > 1 ||
    !std::isfinite(acceleration_scale) || acceleration_scale <= 0 || acceleration_scale > 1 ||
    std::set<std::string>(names.begin(), names.end()).size() != names.size()) {return result;}
  const auto step_ns = static_cast<std::int64_t>(std::ceil(duration * 1e9 / (path.size() - 1)));
  const double dt = step_ns * 1e-9;
  // Cell compilation supplies uniformly timed Cartesian/IK samples. Keep the
  // timing and every knot; independently profiling joints would change the path.
  std::vector<std::vector<double>> segment_velocity(path.size()-1);
  for (std::size_t k = 0; k < path.size(); ++k) {
    if (path[k].size() != names.size()) {return result;}
    for (std::size_t j = 0; j < names.size(); ++j) {
      const auto & limit = limits[j];
      const auto q = path[k][j];
      if (names[j].empty() || !std::isfinite(q) || !std::isfinite(start[j]) ||
        !std::isfinite(limit.max_velocity) || limit.max_velocity <= 0 ||
        !std::isfinite(limit.max_acceleration) || limit.max_acceleration <= 0 ||
        (limit.position_bounded && (!std::isfinite(limit.lower) || !std::isfinite(limit.upper) ||
        limit.lower > limit.upper || q < limit.lower || q > limit.upper ||
        start[j] < limit.lower || start[j] > limit.upper))) {return result;}
      if (k == 0 && std::abs(start[j] - q) > 0.01) {
        result.reason = "taught_path_start_mismatch"; return result;
      }
      if (k > 0) {
        const double v = (q-path[k-1][j])/dt;
        segment_velocity[k-1].push_back(v);
        if (std::abs(v) > limit.max_velocity * velocity_scale) {
          result.reason = "taught_path_velocity_exceeded"; return result;
        }
        const double prior = k > 1 ? segment_velocity[k-2][j] : 0.;
        if (std::abs(v-prior)/dt > limit.max_acceleration * acceleration_scale ||
          (k + 1 == path.size() && std::abs(v)/dt > limit.max_acceleration * acceleration_scale))
        {result.reason = "taught_path_acceleration_exceeded"; return result;}
      }
    }
  }
  result.trajectory.joint_names = names;
  for (std::size_t k = 0; k < path.size(); ++k) {
    const auto ns = static_cast<std::int64_t>(k) * step_ns;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start.sec = static_cast<std::int32_t>(ns / 1000000000LL);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(ns % 1000000000LL);
    point.positions = path[k];
    for (std::size_t j = 0; j < names.size(); ++j) {
      const double before = k == 0 ? 0. : segment_velocity[k-1][j];
      const double after = k + 1 == path.size() ? 0. : segment_velocity[k][j];
      point.velocities.push_back(k == 0 || k+1 == path.size() ? 0. : (before+after)*.5);
      point.accelerations.push_back((after-before)/dt);
    }
    result.trajectory.points.push_back(std::move(point));
  }
  result.valid = true;
  result.reason.clear();
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
