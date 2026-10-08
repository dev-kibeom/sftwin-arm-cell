#pragma once

#include <chrono>

namespace arm_cell_motion_moveit2
{

enum class TrajectoryWatchdogStatus
{
  RUNNING,
  COMPLETED,
  SIMULATION_CLOCK_STALL
};

struct TrajectoryWatchdogDiagnostics
{
  double expected_simulation_duration{0.0};
  double current_simulation_time{0.0};
  double elapsed_wall_s{0.0};
  double observed_rtf{0.0};
  double wall_since_progress_s{0.0};
};

class TrajectoryWatchdog final
{
public:
  using Clock = std::chrono::steady_clock;

  explicit TrajectoryWatchdog(double wall_clock_liveness_timeout_s);

  void start(
    double start_simulation_time,
    double expected_simulation_duration,
    Clock::time_point wall_start);
  void observe(double simulation_time, Clock::time_point wall_now);
  TrajectoryWatchdogStatus evaluate(
    double simulation_time,
    bool execution_complete,
    Clock::time_point wall_now);
  Clock::time_point next_liveness_deadline() const;
  const TrajectoryWatchdogDiagnostics & diagnostics() const;

private:
  double wall_clock_liveness_timeout_s_{0.0};
  double start_simulation_time_{0.0};
  double last_progress_simulation_time_{0.0};
  double first_observed_simulation_time_{0.0};
  Clock::time_point wall_start_{};
  Clock::time_point last_progress_wall_{};
  bool started_{false};
  TrajectoryWatchdogDiagnostics diagnostics_{};
};

}  // namespace arm_cell_motion_moveit2
