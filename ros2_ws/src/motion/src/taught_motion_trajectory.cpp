#include "arm_cell_motion/taught_motion_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arm_cell_motion
{
bool scale_taught_trajectory(
  trajectory_msgs::msg::JointTrajectory & trajectory,
  double velocity_scale, double acceleration_scale)
{
  if (!std::isfinite(velocity_scale) || velocity_scale <= 0.0 || velocity_scale > 1.0 ||
    !std::isfinite(acceleration_scale) || acceleration_scale <= 0.0 || acceleration_scale > 1.0)
  {
    return false;
  }
  const auto stretch = std::max(1.0 / velocity_scale, 1.0 / std::sqrt(acceleration_scale));
  auto scaled = trajectory;
  for (auto & point : scaled.points) {
    const auto seconds = (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9) *
      stretch;
    if (!std::isfinite(seconds) || seconds < 0.0 ||
      seconds >= std::numeric_limits<std::int32_t>::max()) {return false;}
    const auto nanoseconds = static_cast<std::int64_t>(std::llround(seconds * 1e9));
    point.time_from_start.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    for (auto & velocity : point.velocities) {
      velocity /= stretch;
    }
    for (auto & acceleration : point.accelerations) {
      acceleration /= stretch * stretch;
    }
  }
  trajectory = std::move(scaled);
  return true;
}
}  // namespace arm_cell_motion
