#include "arm_cell_motion_moveit2/trajectory_watchdog.hpp"

#include <algorithm>
#include <cmath>

namespace arm_cell_motion_moveit2
{

TrajectoryWatchdog::TrajectoryWatchdog(double wall_clock_liveness_timeout_s)
: wall_clock_liveness_timeout_s_(wall_clock_liveness_timeout_s)
{
}

void TrajectoryWatchdog::start(
  double start_simulation_time,
  double expected_simulation_duration,
  Clock::time_point wall_start)
{
  start_simulation_time_ = start_simulation_time;
  last_progress_simulation_time_ = start_simulation_time;
  first_observed_simulation_time_ = start_simulation_time;
  wall_start_ = wall_start;
  last_progress_wall_ = wall_start;
  started_ = true;
  diagnostics_ = {};
  diagnostics_.expected_simulation_duration = expected_simulation_duration;
  diagnostics_.current_simulation_time = 0.0;
}

void TrajectoryWatchdog::observe(double simulation_time, Clock::time_point wall_now)
{
  if (!started_ || !std::isfinite(simulation_time)) {
    return;
  }
  const auto wall_elapsed = std::chrono::duration<double>(wall_now - wall_start_).count();
  diagnostics_.elapsed_wall_s = std::max(0.0, wall_elapsed);
  if (simulation_time > last_progress_simulation_time_) {
    last_progress_simulation_time_ = simulation_time;
    last_progress_wall_ = wall_now;
  }
  diagnostics_.current_simulation_time = std::max(0.0, simulation_time - start_simulation_time_);
  const auto simulation_elapsed =
    std::max(0.0, simulation_time - first_observed_simulation_time_);
  diagnostics_.observed_rtf = wall_elapsed > 0.0 ? simulation_elapsed / wall_elapsed : 0.0;
  diagnostics_.wall_since_progress_s =
    std::max(0.0, std::chrono::duration<double>(wall_now - last_progress_wall_).count());
}

TrajectoryWatchdogStatus TrajectoryWatchdog::evaluate(
  double simulation_time,
  bool execution_complete,
  Clock::time_point wall_now)
{
  observe(simulation_time, wall_now);
  if (!started_) {
    return TrajectoryWatchdogStatus::SIMULATION_CLOCK_STALL;
  }
  if (diagnostics_.current_simulation_time >= diagnostics_.expected_simulation_duration &&
    execution_complete)
  {
    return TrajectoryWatchdogStatus::COMPLETED;
  }
  if (diagnostics_.wall_since_progress_s >= wall_clock_liveness_timeout_s_) {
    return TrajectoryWatchdogStatus::SIMULATION_CLOCK_STALL;
  }
  return TrajectoryWatchdogStatus::RUNNING;
}

TrajectoryWatchdog::Clock::time_point TrajectoryWatchdog::next_liveness_deadline() const
{
  return last_progress_wall_ + std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(wall_clock_liveness_timeout_s_));
}

const TrajectoryWatchdogDiagnostics & TrajectoryWatchdog::diagnostics() const
{
  return diagnostics_;
}

}  // namespace arm_cell_motion_moveit2
