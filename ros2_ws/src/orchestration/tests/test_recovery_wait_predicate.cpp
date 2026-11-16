#include <gtest/gtest.h>

#include "arm_cell_orchestration/recovery_entry_latch.hpp"
#include "arm_cell_orchestration/recovery_wait_predicate.hpp"

namespace
{
using MotionCapability = arm_cell_interfaces::msg::MotionCapability;
using MotionStatus = arm_cell_interfaces::msg::MotionStatus;
using SafetyState = arm_cell_interfaces::msg::SafetyState;
using StopMode = arm_cell_interfaces::msg::StopMode;
using Observation = arm_cell_orchestration::SafetyObservation;

Observation safety(bool valid, bool fresh, uint8_t capability, uint8_t mode)
{
  SafetyState state;
  state.valid = valid;
  state.required_inputs_fresh = fresh;
  state.motion_capability.value = capability;
  state.selected_stop_mode.value = mode;
  const auto now = Observation::Clock::now();
  return arm_cell_orchestration::evaluate_safety_observation(
    state, true, now, now, std::chrono::milliseconds(500));
}

MotionStatus stopped()
{
  MotionStatus motion;
  motion.execution_state = MotionStatus::MOTION_STATE_STOPPED;
  motion.backend_inactivity_confirmed = true;
  return motion;
}
}  // namespace

TEST(RecoveryWaitPredicateTest, InvalidOrStaleRecoverySnapshotIsNotImmediatelyReady)
{
  const auto motion = stopped();
  EXPECT_FALSE(
    arm_cell_orchestration::recovery_ready_for_immediate_return(
      safety(false, true, MotionCapability::MOTION_RECOVERY_ONLY, StopMode::STOP_MODE_CONTROLLED),
      motion));
  EXPECT_FALSE(
    arm_cell_orchestration::recovery_ready_for_immediate_return(
      safety(true, false, MotionCapability::MOTION_RECOVERY_ONLY, StopMode::STOP_MODE_CONTROLLED),
      motion));
}

TEST(RecoveryWaitPredicateTest, ReadyRequiresRecoveryCapabilityAndStoppedMotion)
{
  EXPECT_TRUE(
    arm_cell_orchestration::recovery_ready_for_immediate_return(
      safety(true, true, MotionCapability::MOTION_RECOVERY_ONLY, StopMode::STOP_MODE_CONTROLLED),
      stopped()));
  EXPECT_FALSE(
    arm_cell_orchestration::recovery_ready_for_immediate_return(
      safety(true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_CONTROLLED),
      stopped()));
}

TEST(RecoveryWaitPredicateTest, NormalContinuationRequiresStoppedMotionAndReleasedObject)
{
  auto motion = stopped();
  motion.holding_state = MotionStatus::HOLDING_RELEASED;
  const auto normal = safety(
    true, true, MotionCapability::MOTION_NORMAL, StopMode::STOP_MODE_CONTROLLED);
  EXPECT_TRUE(
    arm_cell_orchestration::recovery_ready_for_batch_continuation(normal, motion));
  EXPECT_FALSE(
    arm_cell_orchestration::recovery_ready_for_batch_continuation(normal, motion, false));

  motion.holding_state = MotionStatus::HOLDING_HELD;
  EXPECT_FALSE(
    arm_cell_orchestration::recovery_ready_for_batch_continuation(normal, motion));
  motion.holding_state = MotionStatus::HOLDING_UNKNOWN;
  EXPECT_FALSE(
    arm_cell_orchestration::recovery_ready_for_batch_continuation(normal, motion));
  motion.holding_state = MotionStatus::HOLDING_RELEASED;
  motion.execution_active = true;
  EXPECT_FALSE(
    arm_cell_orchestration::recovery_ready_for_batch_continuation(normal, motion));
}

TEST(RecoveryWaitPredicateTest, HigherSeverityIsAnImmediateTerminationWakeup)
{
  EXPECT_FALSE(
    arm_cell_orchestration::recovery_severity_escalation_ready_for_immediate_return(
      safety(true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_CONTROLLED)));
  EXPECT_TRUE(
    arm_cell_orchestration::recovery_severity_escalation_ready_for_immediate_return(
      safety(true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_IMMEDIATE)));
}

TEST(RecoveryWaitPredicateTest, DiagnosticSignatureSuppressesRepeatsAndPreservesDecisions)
{
  auto recovery_safety = safety(
    true, true, MotionCapability::MOTION_RECOVERY_ONLY, StopMode::STOP_MODE_CONTROLLED);
  auto motion = stopped();
  const auto original = arm_cell_orchestration::recovery_observation_diagnostic_signature(
    recovery_safety, motion, true, true);

  EXPECT_EQ(
    original,
    arm_cell_orchestration::recovery_observation_diagnostic_signature(
      recovery_safety, motion, true, true));

  motion.execution_active = true;
  EXPECT_NE(
    original,
    arm_cell_orchestration::recovery_observation_diagnostic_signature(
      recovery_safety, motion, true, true));

  motion.execution_active = false;
  EXPECT_NE(
    original,
    arm_cell_orchestration::recovery_observation_diagnostic_signature(
      recovery_safety, motion, true, false));

  EXPECT_NE(
    original,
    arm_cell_orchestration::recovery_observation_diagnostic_signature(
      recovery_safety, motion, false, false));

  recovery_safety.state.selected_stop_mode.value = StopMode::STOP_MODE_IMMEDIATE;
  EXPECT_NE(
    original,
    arm_cell_orchestration::recovery_observation_diagnostic_signature(
      recovery_safety, motion, true, true));
}

TEST(RecoveryEntryLatchTest, FirstCapabilityLossWinsPerMissionAndResets)
{
  arm_cell_orchestration::RecoveryEntryMap entries;
  const auto controlled = safety(
    true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_CONTROLLED);
  const auto immediate = safety(
    true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_IMMEDIATE);

  arm_cell_orchestration::begin_recovery_entry("mission-a", entries);
  arm_cell_orchestration::observe_recovery_entry("mission-a", controlled, entries);
  arm_cell_orchestration::observe_recovery_entry("mission-a", immediate, entries);
  ASSERT_TRUE(arm_cell_orchestration::read_recovery_entry("mission-a", entries).has_value());
  EXPECT_EQ(
    arm_cell_orchestration::read_recovery_entry(
      "mission-a", entries)->state.selected_stop_mode.value,
    StopMode::STOP_MODE_CONTROLLED);

  arm_cell_orchestration::erase_recovery_entry("mission-a", entries);
  arm_cell_orchestration::begin_recovery_entry("mission-b", entries);
  arm_cell_orchestration::observe_recovery_entry("mission-b", immediate, entries);
  ASSERT_TRUE(arm_cell_orchestration::read_recovery_entry("mission-b", entries).has_value());
  EXPECT_EQ(
    arm_cell_orchestration::read_recovery_entry(
      "mission-b", entries)->state.selected_stop_mode.value,
    StopMode::STOP_MODE_IMMEDIATE);
}

TEST(RecoveryEntryLatchTest, ExpiredObservationCannotCreateRecoveryEntry)
{
  arm_cell_orchestration::RecoveryEntryMap entries;
  const auto now = Observation::Clock::now();
  auto state = safety(
    true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_CONTROLLED).state;
  const auto expired = arm_cell_orchestration::evaluate_safety_observation(
    state, true, now - std::chrono::seconds(1), now, std::chrono::milliseconds(500));

  arm_cell_orchestration::begin_recovery_entry("mission", entries);
  arm_cell_orchestration::observe_recovery_entry("mission", expired, entries);

  EXPECT_FALSE(arm_cell_orchestration::read_recovery_entry("mission", entries));
}

TEST(RecoveryEntryLatchTest, RecordedEventRemainsAvailableAfterReceiptAgeExpires)
{
  arm_cell_orchestration::RecoveryEntryMap entries;
  auto controlled = safety(
    true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_CONTROLLED);
  controlled.receipt_time -= std::chrono::milliseconds(501);

  arm_cell_orchestration::begin_recovery_entry("mission", entries);
  arm_cell_orchestration::observe_recovery_entry("mission", controlled, entries);

  const auto recorded = arm_cell_orchestration::read_recovery_entry("mission", entries);
  ASSERT_TRUE(recorded.has_value());
  EXPECT_TRUE(recorded->delivery_valid);
  EXPECT_EQ(
    recorded->state.selected_stop_mode.value,
    StopMode::STOP_MODE_CONTROLLED);
}
