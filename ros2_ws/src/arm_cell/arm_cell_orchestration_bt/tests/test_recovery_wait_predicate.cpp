#include <gtest/gtest.h>

#include "arm_cell_orchestration_bt/recovery_entry_latch.hpp"
#include "arm_cell_orchestration_bt/recovery_wait_predicate.hpp"

namespace
{
using MotionCapability = arm_cell_interfaces::msg::MotionCapability;
using MotionStatus = arm_cell_interfaces::msg::MotionStatus;
using SafetyState = arm_cell_interfaces::msg::SafetyState;
using StopMode = arm_cell_interfaces::msg::StopMode;

SafetyState safety(bool valid, bool fresh, uint8_t capability, uint8_t mode)
{
  SafetyState state;
  state.valid = valid;
  state.required_inputs_fresh = fresh;
  state.motion_capability.value = capability;
  state.selected_stop_mode.value = mode;
  return state;
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
    arm_cell_orchestration_bt::recovery_ready_for_immediate_return(
      safety(false, true, MotionCapability::MOTION_RECOVERY_ONLY, StopMode::STOP_MODE_CONTROLLED),
      motion));
  EXPECT_FALSE(
    arm_cell_orchestration_bt::recovery_ready_for_immediate_return(
      safety(true, false, MotionCapability::MOTION_RECOVERY_ONLY, StopMode::STOP_MODE_CONTROLLED),
      motion));
}

TEST(RecoveryWaitPredicateTest, ReadyRequiresRecoveryCapabilityAndStoppedMotion)
{
  EXPECT_TRUE(
    arm_cell_orchestration_bt::recovery_ready_for_immediate_return(
      safety(true, true, MotionCapability::MOTION_RECOVERY_ONLY, StopMode::STOP_MODE_CONTROLLED),
      stopped()));
  EXPECT_FALSE(
    arm_cell_orchestration_bt::recovery_ready_for_immediate_return(
      safety(true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_CONTROLLED),
      stopped()));
}

TEST(RecoveryWaitPredicateTest, HigherSeverityIsAnImmediateTerminationWakeup)
{
  EXPECT_FALSE(
    arm_cell_orchestration_bt::recovery_severity_escalation_ready_for_immediate_return(
      safety(true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_CONTROLLED)));
  EXPECT_TRUE(
    arm_cell_orchestration_bt::recovery_severity_escalation_ready_for_immediate_return(
      safety(true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_IMMEDIATE)));
}

TEST(RecoveryEntryLatchTest, FirstCapabilityLossWinsPerMissionAndResets)
{
  arm_cell_orchestration_bt::RecoveryEntryMap entries;
  const auto controlled = safety(
    true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_CONTROLLED);
  const auto immediate = safety(
    true, true, MotionCapability::MOTION_NONE, StopMode::STOP_MODE_IMMEDIATE);

  arm_cell_orchestration_bt::begin_recovery_entry("mission-a", entries);
  arm_cell_orchestration_bt::observe_recovery_entry("mission-a", controlled, entries);
  arm_cell_orchestration_bt::observe_recovery_entry("mission-a", immediate, entries);
  ASSERT_TRUE(arm_cell_orchestration_bt::read_recovery_entry("mission-a", entries).has_value());
  EXPECT_EQ(
    arm_cell_orchestration_bt::read_recovery_entry("mission-a", entries)->selected_stop_mode.value,
    StopMode::STOP_MODE_CONTROLLED);

  arm_cell_orchestration_bt::erase_recovery_entry("mission-a", entries);
  arm_cell_orchestration_bt::begin_recovery_entry("mission-b", entries);
  arm_cell_orchestration_bt::observe_recovery_entry("mission-b", immediate, entries);
  ASSERT_TRUE(arm_cell_orchestration_bt::read_recovery_entry("mission-b", entries).has_value());
  EXPECT_EQ(
    arm_cell_orchestration_bt::read_recovery_entry("mission-b", entries)->selected_stop_mode.value,
    StopMode::STOP_MODE_IMMEDIATE);
}
