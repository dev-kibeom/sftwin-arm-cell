#include <gtest/gtest.h>

#include <chrono>

#include "arm_cell_motion_moveit2/trajectory_watchdog.hpp"

namespace arm_cell_motion_moveit2
{
namespace
{
using Clock = std::chrono::steady_clock;

Clock::time_point wall(double seconds)
{
  return Clock::time_point(
    std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(seconds)));
}

TEST(TrajectoryWatchdog, SimulationProgressAtHalfRtfDoesNotWallTimeout)
{
  TrajectoryWatchdog watchdog(3.0);
  watchdog.start(0.0, 10.0, wall(0.0));
  watchdog.observe(5.0, wall(10.0));

  EXPECT_EQ(
    watchdog.evaluate(10.0, true, wall(20.0)),
    TrajectoryWatchdogStatus::COMPLETED);
  EXPECT_DOUBLE_EQ(watchdog.diagnostics().observed_rtf, 0.5);
}

TEST(TrajectoryWatchdog, SimulationClockStallFailsClosed)
{
  TrajectoryWatchdog watchdog(3.0);
  watchdog.start(0.0, 10.0, wall(0.0));
  watchdog.observe(1.0, wall(1.0));

  EXPECT_EQ(
    watchdog.evaluate(1.0, false, wall(4.1)),
    TrajectoryWatchdogStatus::SIMULATION_CLOCK_STALL);
  EXPECT_NEAR(watchdog.diagnostics().wall_since_progress_s, 3.1, 1e-8);
}

TEST(TrajectoryWatchdog, ProgressPastPlannedDurationDoesNotTimeout)
{
  TrajectoryWatchdog watchdog(3.0);
  watchdog.start(0.0, 10.0, wall(0.0));
  watchdog.observe(10.0, wall(2.0));

  EXPECT_EQ(
    watchdog.evaluate(11.9, false, wall(20.0)),
    TrajectoryWatchdogStatus::RUNNING);
  EXPECT_EQ(
    watchdog.evaluate(100.0, false, wall(108.0)),
    TrajectoryWatchdogStatus::RUNNING);
  EXPECT_EQ(
    watchdog.evaluate(101.0, true, wall(109.0)),
    TrajectoryWatchdogStatus::COMPLETED);
}

TEST(TrajectoryWatchdog, FinalSimulationTimeAndConvergedFeedbackCompletes)
{
  TrajectoryWatchdog watchdog(3.0);
  watchdog.start(0.0, 10.0, wall(0.0));
  watchdog.observe(10.0, wall(20.0));

  EXPECT_EQ(
    watchdog.evaluate(10.0, true, wall(20.0)),
    TrajectoryWatchdogStatus::COMPLETED);
}
}  // namespace
}  // namespace arm_cell_motion_moveit2
