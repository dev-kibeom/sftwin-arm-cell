#include "arm_cell_motion_moveit2/motion_progress_watchdog.hpp"

#include <cmath>
#include <stdexcept>

namespace arm_cell_motion_moveit2
{

MotionProgressWatchdog::MotionProgressWatchdog(
  double stall_timeout_s, double minimum_joint_delta_rad)
: stall_timeout_s_(stall_timeout_s), minimum_joint_delta_rad_(minimum_joint_delta_rad)
{
  if (!std::isfinite(stall_timeout_s_) || stall_timeout_s_ <= 0.0) {
    throw std::invalid_argument("motion progress stall timeout must be finite and positive");
  }
  if (!std::isfinite(minimum_joint_delta_rad_) || minimum_joint_delta_rad_ <= 0.0) {
    throw std::invalid_argument("minimum joint progress must be finite and positive");
  }
}

void MotionProgressWatchdog::start(
  const std::vector<double> & positions, Clock::time_point now)
{
  last_progress_positions_ = positions;
  last_progress_time_ = now;
  started_ = true;
}

bool MotionProgressWatchdog::observe(
  const std::vector<double> & positions, Clock::time_point now)
{
  if (!started_ || positions.size() != last_progress_positions_.size()) {
    return false;
  }
  bool progressed = false;
  for (std::size_t index = 0; index < positions.size(); ++index) {
    if (std::abs(positions[index] - last_progress_positions_[index]) >=
      minimum_joint_delta_rad_)
    {
      progressed = true;
      break;
    }
  }
  if (progressed) {
    last_progress_positions_ = positions;
    last_progress_time_ = now;
  }
  return progressed;
}

bool MotionProgressWatchdog::stalled(Clock::time_point now) const
{
  return started_ && now - last_progress_time_ >=
         std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(stall_timeout_s_));
}

MotionProgressWatchdog::Clock::time_point MotionProgressWatchdog::next_deadline() const
{
  return last_progress_time_ + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(stall_timeout_s_));
}

}  // namespace arm_cell_motion_moveit2
