#include <gtest/gtest.h>

#include <chrono>
#include <vector>

#include <rcutils/logging.h>

#include "arm_cell_safety_fsm/safety_input_tracker.hpp"
#include "arm_cell_safety_fsm/source_timestamp.hpp"

namespace arm_cell_safety_fsm
{

namespace
{
std::vector<int> captured_log_levels;

void capture_log_level(
  const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
  const char *, va_list *)
{
  captured_log_levels.push_back(severity);
}
}

TEST(SafetyInputTrackerTest, SourceTimestampQualityDoesNotAffectReceiptFreshness)
{
  SafetyInputTracker tracker(std::chrono::milliseconds(100));
  const auto received = SafetyInputTracker::Clock::time_point{};
  tracker.record(SafetyInput::HARDWARE, true, false, received);

  EXPECT_TRUE(tracker.is_fresh(SafetyInput::HARDWARE, received + std::chrono::milliseconds(1)));
  EXPECT_TRUE(tracker.received(SafetyInput::HARDWARE));

  tracker.record(SafetyInput::HARDWARE, true, true, received);
  EXPECT_TRUE(tracker.is_fresh(SafetyInput::HARDWARE, received + std::chrono::milliseconds(1)));
}

TEST(SafetyInputTrackerTest, ReportsPerInputVerdictAndReceiptAge)
{
  SafetyInputTracker tracker(std::chrono::milliseconds(100));
  const auto received = SafetyInputTracker::Clock::time_point{};

  const auto missing = tracker.diagnostic(
    SafetyInput::HARDWARE, received + std::chrono::milliseconds(10));
  EXPECT_FALSE(missing.received);
  EXPECT_FALSE(missing.age_available);
  EXPECT_EQ(missing.reason, SafetyInputReason::NOT_RECEIVED);

  tracker.record(SafetyInput::HARDWARE, false, true, received);
  const auto invalid = tracker.diagnostic(
    SafetyInput::HARDWARE, received + std::chrono::milliseconds(25));
  EXPECT_TRUE(invalid.received);
  EXPECT_FALSE(invalid.message_valid);
  EXPECT_TRUE(invalid.source_timestamp_plausible);
  EXPECT_EQ(invalid.receipt_age, std::chrono::milliseconds(25));
  EXPECT_EQ(invalid.reason, SafetyInputReason::INVALID);

  tracker.record(SafetyInput::HARDWARE, true, false, received);
  const auto invalid_stamp = tracker.diagnostic(
    SafetyInput::HARDWARE, received + std::chrono::milliseconds(25));
  EXPECT_TRUE(invalid_stamp.message_valid);
  EXPECT_FALSE(invalid_stamp.source_timestamp_plausible);
  EXPECT_EQ(invalid_stamp.reason, SafetyInputReason::OK);
  EXPECT_FALSE(invalid_stamp.source_timestamp_plausible);

  tracker.record(SafetyInput::HARDWARE, true, true, received);
  const auto stale = tracker.diagnostic(
    SafetyInput::HARDWARE, received + std::chrono::milliseconds(101));
  EXPECT_TRUE(stale.message_valid);
  EXPECT_TRUE(stale.source_timestamp_plausible);
  EXPECT_EQ(stale.receipt_age, std::chrono::milliseconds(101));
  EXPECT_EQ(stale.reason, SafetyInputReason::STALE);

  const auto fresh = tracker.diagnostic(
    SafetyInput::HARDWARE, received + std::chrono::milliseconds(50));
  EXPECT_EQ(fresh.reason, SafetyInputReason::OK);
}

TEST(SafetyInputTrackerTest, MissingOrExpiredInputsAreNotFresh)
{
  SafetyInputTracker tracker(std::chrono::milliseconds(100));
  const auto now = SafetyInputTracker::Clock::time_point{};

  EXPECT_FALSE(tracker.all_required_fresh(now));
  tracker.record(SafetyInput::HARDWARE, true, true, now);
  tracker.record(SafetyInput::MOTION, true, true, now);
  tracker.record(SafetyInput::HARDWARE, true, true, now);
  tracker.record(SafetyInput::MOTION, true, true, now);

  EXPECT_TRUE(tracker.all_required_fresh(now + std::chrono::milliseconds(99)));
  EXPECT_FALSE(tracker.all_required_fresh(now + std::chrono::milliseconds(101)));
}

TEST(SafetyInputTrackerTest, MotionTimestampDiagnosticDoesNotMaskReceiptStaleness)
{
  SafetyInputTracker tracker(std::chrono::milliseconds(100));
  const auto received = SafetyInputTracker::Clock::time_point{};
  tracker.record(SafetyInput::MOTION, true, false, received);

  const auto fresh = tracker.diagnostic(
    SafetyInput::MOTION, received + std::chrono::milliseconds(50));
  EXPECT_EQ(fresh.reason, SafetyInputReason::OK);
  EXPECT_FALSE(fresh.source_timestamp_plausible);

  const auto stale = tracker.diagnostic(
    SafetyInput::MOTION, received + std::chrono::milliseconds(101));
  EXPECT_EQ(stale.reason, SafetyInputReason::STALE);
  EXPECT_FALSE(stale.source_timestamp_plausible);
}

TEST(SafetyInputTrackerTest, RetainsSourceClockAndBothFreshnessThresholds)
{
  using namespace std::chrono_literals;
  SafetyInputTracker tracker(500ms);
  const auto receipt = SafetyInputTracker::Clock::time_point{} + 2s;
  tracker.record(
    SafetyInput::HARDWARE, true, false, receipt, 1'000'000'000LL,
    3'500'000'000LL, 2'000'000'000LL);

  const auto diagnostic = tracker.diagnostic(SafetyInput::HARDWARE, receipt + 25ms);
  EXPECT_TRUE(diagnostic.source_time_available);
  EXPECT_EQ(diagnostic.source_stamp_ns, 1'000'000'000LL);
  EXPECT_EQ(diagnostic.source_evaluation_ros_time_ns, 3'500'000'000LL);
  EXPECT_EQ(diagnostic.source_age_ns, 2'500'000'000LL);
  EXPECT_EQ(diagnostic.source_maximum_age_ns, 2'000'000'000LL);
  EXPECT_EQ(diagnostic.receipt_timeout_ns, 500'000'000LL);
  EXPECT_EQ(diagnostic.receipt_age, 25ms);
  EXPECT_FALSE(diagnostic.source_timestamp_plausible);
  EXPECT_EQ(diagnostic.reason, SafetyInputReason::OK);
  EXPECT_TRUE(tracker.is_fresh(SafetyInput::HARDWARE, receipt + 25ms));
}

TEST(SafetyInputTrackerTest, DecisionSignatureIgnoresChangingTimestampAndAgeValues)
{
  SafetyInputDiagnostic first;
  first.received = true;
  first.message_valid = true;
  first.source_timestamp_plausible = true;
  first.age_available = true;
  first.reason = SafetyInputReason::OK;
  first.source_stamp_ns = 10'000;
  first.source_age_ns = 20;
  first.receipt_age = std::chrono::milliseconds(10);
  auto later = first;
  later.source_stamp_ns += 1'000'000;
  later.source_age_ns += 1'000'000;
  later.receipt_age += std::chrono::milliseconds(20);

  EXPECT_FALSE(
    safety_input_decision_changed(
      std::array<SafetyInputDiagnostic, 1>{later},
      std::array<SafetyInputDiagnostic, 1>{first}));
}

TEST(SafetyInputTrackerTest, DecisionSignatureDetectsValidityFreshnessAndTimestampQualityChanges)
{
  SafetyInputDiagnostic previous;
  previous.received = true;
  previous.message_valid = true;
  previous.source_timestamp_plausible = true;
  previous.age_available = true;
  previous.reason = SafetyInputReason::OK;

  auto stale = previous;
  stale.reason = SafetyInputReason::STALE;
  EXPECT_TRUE(
    safety_input_decision_changed(
      std::array<SafetyInputDiagnostic, 1>{stale},
      std::array<SafetyInputDiagnostic, 1>{previous}));

  auto invalid_stamp = previous;
  invalid_stamp.source_timestamp_plausible = false;
  EXPECT_TRUE(
    safety_input_decision_changed(
      std::array<SafetyInputDiagnostic, 1>{invalid_stamp},
      std::array<SafetyInputDiagnostic, 1>{previous}));

  auto invalid_message = previous;
  invalid_message.message_valid = false;
  invalid_message.reason = SafetyInputReason::INVALID;
  EXPECT_TRUE(
    safety_input_decision_changed(
      std::array<SafetyInputDiagnostic, 1>{invalid_message},
      std::array<SafetyInputDiagnostic, 1>{previous}));
}

TEST(SafetyInputTrackerTest, DiagnosticGateKeepsInputChangesOptionalAndStateLogsIndependent)
{
  EXPECT_TRUE(safety_diagnostic_record_required(true, false, true));
  EXPECT_TRUE(safety_diagnostic_record_required(true, true, false));
  EXPECT_FALSE(safety_diagnostic_record_required(false, true, false));
  EXPECT_FALSE(safety_diagnostic_record_required(false, false, true));
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

TEST(SafetyInputTrackerTest, TimestampAnomalyReportsOnlyEntryAndRecoveryTransitions)
{
  SourceTimestampAnomalyTracker tracker;
  EXPECT_EQ(tracker.observe(false), SourceTimestampQualityEvent::ANOMALY_ENTERED);
  EXPECT_EQ(tracker.observe(false), SourceTimestampQualityEvent::NONE);
  EXPECT_EQ(tracker.observe(true), SourceTimestampQualityEvent::RECOVERED);
  EXPECT_EQ(tracker.observe(true), SourceTimestampQualityEvent::NONE);
}

TEST(SafetyInputTrackerTest, TimestampAnomalyLogsWarnOnceAndRecoveryAtInfo)
{
  auto logger = rclcpp::get_logger("source_timestamp_quality_test");
  const auto previous_level = rcutils_logging_get_logger_level(logger.get_name());
  const auto previous_handler = rcutils_logging_get_output_handler();
  ASSERT_EQ(
    rcutils_logging_set_logger_level(logger.get_name(), RCUTILS_LOG_SEVERITY_DEBUG), RCUTILS_RET_OK);
  captured_log_levels.clear();
  rcutils_logging_set_output_handler(capture_log_level);

  SourceTimestampAnomalyTracker tracker;
  log_source_timestamp_quality_event(
    logger, tracker.observe(false), "Safety-HW");
  log_source_timestamp_quality_event(
    logger, tracker.observe(false), "Safety-HW");
  log_source_timestamp_quality_event(
    logger, tracker.observe(true), "Safety-HW");
  log_source_timestamp_quality_event(
    logger, tracker.observe(true), "Safety-HW");

  rcutils_logging_set_output_handler(previous_handler);
  EXPECT_EQ(
    rcutils_logging_set_logger_level(logger.get_name(), previous_level), RCUTILS_RET_OK);
  EXPECT_EQ(captured_log_levels, (std::vector<int>{
      RCUTILS_LOG_SEVERITY_WARN, RCUTILS_LOG_SEVERITY_INFO}));
}

}  // namespace arm_cell_safety_fsm
