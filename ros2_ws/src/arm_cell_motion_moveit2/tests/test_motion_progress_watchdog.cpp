#include <gtest/gtest.h>

#include <chrono>
#include <vector>

#include "arm_cell_motion_moveit2/motion_progress_watchdog.hpp"

namespace arm_cell_motion_moveit2
{
namespace
{
using Clock = MotionProgressWatchdog::Clock;

Clock::time_point wall(double seconds)
{
  return Clock::time_point(
    std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds)));
}

TEST(MotionProgressWatchdog, ProgressingMotionDoesNotExpireAtOriginalWallDeadline)
{
  MotionProgressWatchdog watchdog(1.0);
  watchdog.start({0.0, 0.0}, wall(0.0));

  EXPECT_TRUE(watchdog.observe({0.1, 0.0}, wall(0.9)));
  EXPECT_FALSE(watchdog.stalled(wall(1.5)));
  EXPECT_TRUE(watchdog.observe({0.2, 0.0}, wall(1.8)));
  EXPECT_FALSE(watchdog.stalled(wall(2.7)));
}

TEST(MotionProgressWatchdog, FreshButStationaryFeedbackDetectsStoppedMotion)
{
  MotionProgressWatchdog watchdog(1.0);
  watchdog.start({0.0, 0.0}, wall(0.0));
  EXPECT_FALSE(watchdog.observe({0.0, 0.0}, wall(0.9)));
  EXPECT_TRUE(watchdog.stalled(wall(1.0)));
}

TEST(MotionProgressWatchdog, SmallDeltasAccumulateUntilMeaningfulProgress)
{
  MotionProgressWatchdog watchdog(1.0, 0.01);
  watchdog.start({0.0}, wall(0.0));
  EXPECT_FALSE(watchdog.observe({0.004}, wall(0.5)));
  EXPECT_TRUE(watchdog.observe({0.011}, wall(0.8)));
  EXPECT_FALSE(watchdog.stalled(wall(1.7)));
}

TEST(MotionProgressWatchdog, SmallFeedbackOscillationDoesNotCountAsProgress)
{
  MotionProgressWatchdog watchdog(1.0);
  watchdog.start({0.0, 0.0}, wall(0.0));
  EXPECT_FALSE(watchdog.observe({0.0, 0.0001}, wall(0.4)));
  EXPECT_FALSE(watchdog.observe({0.0, 0.0}, wall(0.8)));
  EXPECT_TRUE(watchdog.stalled(wall(1.0)));
}
}  // namespace
}  // namespace arm_cell_motion_moveit2
