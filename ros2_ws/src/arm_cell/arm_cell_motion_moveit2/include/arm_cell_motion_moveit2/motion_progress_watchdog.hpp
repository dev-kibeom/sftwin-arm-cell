#pragma once

#include <chrono>
#include <vector>

namespace arm_cell_motion_moveit2
{

class MotionProgressWatchdog final
{
public:
  using Clock = std::chrono::steady_clock;

  MotionProgressWatchdog(double stall_timeout_s, double minimum_joint_delta_rad = 1e-3);

  void start(const std::vector<double> & positions, Clock::time_point now);
  bool observe(const std::vector<double> & positions, Clock::time_point now);
  bool stalled(Clock::time_point now) const;
  Clock::time_point next_deadline() const;

private:
  double stall_timeout_s_{0.0};
  double minimum_joint_delta_rad_{0.0};
  std::vector<double> last_progress_positions_;
  Clock::time_point last_progress_time_{};
  bool started_{false};
};

}  // namespace arm_cell_motion_moveit2
