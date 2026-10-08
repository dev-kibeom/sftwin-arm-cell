#include <gtest/gtest.h>

#include <chrono>

#include "arm_cell_safety_fsm/safety_input_tracker.hpp"
#include "arm_cell_safety_fsm/source_timestamp.hpp"

namespace arm_cell_safety_fsm
{

TEST(SafetyInputTrackerTest, SeparatesReceiptFreshnessFromSourceTimestampValidity)
{
  SafetyInputTracker tracker(std::chrono::milliseconds(100));
  const auto received = SafetyInputTracker::Clock::time_point{};
  tracker.record(SafetyInput::AMR, true, false, received);

  EXPECT_FALSE(tracker.is_fresh(SafetyInput::AMR, received + std::chrono::milliseconds(1)));
  EXPECT_TRUE(tracker.received(SafetyInput::AMR));

  tracker.record(SafetyInput::AMR, true, true, received);
  EXPECT_TRUE(tracker.is_fresh(SafetyInput::AMR, received + std::chrono::milliseconds(1)));
}

TEST(SafetyInputTrackerTest, MissingOrExpiredInputsAreNotFresh)
{
  SafetyInputTracker tracker(std::chrono::milliseconds(100));
  const auto now = SafetyInputTracker::Clock::time_point{};

  EXPECT_FALSE(tracker.all_required_fresh(now));
  tracker.record(SafetyInput::AMR, true, true, now);
  tracker.record(SafetyInput::PACKML, true, true, now);
  tracker.record(SafetyInput::HARDWARE, true, true, now);
  tracker.record(SafetyInput::MOTION, true, true, now);

  EXPECT_TRUE(tracker.all_required_fresh(now + std::chrono::milliseconds(99)));
  EXPECT_FALSE(tracker.all_required_fresh(now + std::chrono::milliseconds(101)));
}

TEST(SafetyInputTrackerTest, MotionAndHardwareAgeDoNotEnterExternalDegradedBand)
{
  SafetyInputTracker tracker(std::chrono::milliseconds(500));
  const auto now = SafetyInputTracker::Clock::time_point{};
  tracker.record(SafetyInput::HARDWARE, true, true, now);
  tracker.record(SafetyInput::MOTION, true, true, now);

  const std::vector<SafetyInput> external_inputs{SafetyInput::AMR, SafetyInput::PACKML};
  EXPECT_FALSE(
    tracker.any_in_degraded_band(
      now + std::chrono::milliseconds(200), std::chrono::milliseconds(100),
      external_inputs));
  tracker.record(SafetyInput::AMR, true, true, now);
  EXPECT_TRUE(
    tracker.any_in_degraded_band(
      now + std::chrono::milliseconds(200), std::chrono::milliseconds(100),
      external_inputs));
}

TEST(SafetyInputTrackerTest, SourceTimestampPlausibilityBoundsAgeInActiveClockDomain)
{
  constexpr int64_t now_ns = 10'000'000'000;
  EXPECT_TRUE(
    source_timestamp_is_plausible(
      now_ns - 1'000'000'000, now_ns, std::chrono::milliseconds(2000)));
  EXPECT_FALSE(
    source_timestamp_is_plausible(
      now_ns + 1, now_ns, std::chrono::milliseconds(2000)));
  EXPECT_FALSE(
    source_timestamp_is_plausible(
      now_ns - 2'001'000'000, now_ns, std::chrono::milliseconds(2000)));
}

}  // namespace arm_cell_safety_fsm
