#include <gtest/gtest.h>

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "arm_cell_orchestration_bt/cycle_coordinator.hpp"
#include "arm_cell_orchestration_bt/recovery_entry_latch.hpp"

namespace
{
using Coordinator = arm_cell_orchestration_bt::CycleCoordinator;
using arm_cell_orchestration_bt::RecoveryEntryMap;
using arm_cell_orchestration_bt::begin_recovery_entry;
using arm_cell_orchestration_bt::observe_recovery_entry;
using arm_cell_orchestration_bt::read_recovery_entry;
using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
using MotionCapability = arm_cell_interfaces::msg::MotionCapability;
using MotionState = arm_cell_interfaces::msg::MotionStatus;
using MotionTaskResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;
using MotionTaskType = arm_cell_interfaces::msg::MotionTaskType;
using SafetyState = arm_cell_interfaces::msg::SafetyState;
using StopMode = arm_cell_interfaces::msg::StopMode;

SafetyState safety(
  uint8_t capability,
  uint8_t safety_state = SafetyState::SAFETY_STATE_STOPPED,
  uint8_t selected_stop_mode = StopMode::STOP_MODE_CONTROLLED)
{
  SafetyState result;
  result.valid = true;
  result.required_inputs_fresh = true;
  result.motion_capability.value = capability;
  result.selected_stop_mode.value = selected_stop_mode;
  result.safety_state = safety_state;
  return result;
}

MotionState stopped_motion()
{
  MotionState result;
  result.execution_state = MotionState::MOTION_STATE_STOPPED;
  result.execution_active = false;
  result.backend_inactivity_confirmed = true;
  result.holding_state = MotionState::HOLDING_RELEASED;
  return result;
}

MotionState stopping_motion()
{
  MotionState result;
  result.execution_state = MotionState::MOTION_STATE_STOPPING;
  result.execution_active = true;
  result.backend_inactivity_confirmed = false;
  return result;
}

Coordinator::MotionResult result(uint8_t code)
{
  Coordinator::MotionResult motion_result;
  motion_result.code.value = code;
  return motion_result;
}

struct RecoveryFixture
{
  std::deque<Coordinator::RecoveryObservation> observations;
  std::vector<uint8_t> tasks;
  int cancel_count{0};
  uint8_t retract_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
  SafetyState current_safety_ = safety(
    MotionCapability::MOTION_NORMAL, SafetyState::SAFETY_STATE_SAFE);
  std::vector<std::string> events;
  std::vector<Coordinator::MotionResult> recovery_results;
  bool hold_retract{false};
  bool retract_started{false};
  bool release_retract{false};
  bool hold_initial_motion{false};
  bool initial_motion_started{false};
  bool release_initial_motion_execution{false};
  RecoveryEntryMap recovery_entries;
  std::mutex state_mutex;
  std::condition_variable state_condition;

  Coordinator make()
  {
    Coordinator::Callbacks callbacks;
    callbacks.read_safety = [this]() {
        std::lock_guard<std::mutex> lock(state_mutex);
        return current_safety_;
      };
    callbacks.read_recovery_entry = [this]() {
        std::lock_guard<std::mutex> lock(state_mutex);
        return read_recovery_entry("mission", recovery_entries);
      };
    callbacks.wait_for_recovery = [this]() {
        if (observations.empty()) {
          Coordinator::RecoveryObservation observation;
          observation.safety = safety(
            MotionCapability::MOTION_NONE, SafetyState::SAFETY_STATE_EMERGENCY_STOPPED);
          return observation;
        }
        auto observation = observations.front();
        observations.pop_front();
        return observation;
      };
    callbacks.execute_task = [this](const ExecuteTask::Goal & goal) {
        tasks.push_back(goal.task_type.value);
        events.push_back(
          goal.task_type.value == MotionTaskType::TASK_TYPE_RETRACT ?
          "retract" : "normal");
        if (tasks.size() == 1U) {
          set_safety(
            safety(
              MotionCapability::MOTION_NONE, SafetyState::SAFETY_STATE_STOPPING));
          if (hold_initial_motion) {
            std::unique_lock<std::mutex> lock(state_mutex);
            initial_motion_started = true;
            state_condition.notify_all();
            state_condition.wait(lock, [this]() {return release_initial_motion_execution;});
            events.push_back("motion-result");
          }
          return result(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
        }
        if (hold_retract) {
          std::unique_lock<std::mutex> lock(state_mutex);
          retract_started = true;
          state_condition.notify_all();
          state_condition.wait(lock, [this]() {return release_retract;});
        }
        SafetyState retract_safety;
        {
          std::lock_guard<std::mutex> lock(state_mutex);
          retract_safety = current_safety_;
        }
        if (retract_safety.selected_stop_mode.value >= StopMode::STOP_MODE_EMERGENCY) {
          events.push_back("retract-terminated");
          return result(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
        }
        return result(retract_result);
      };
    callbacks.cancel_motion = [this]() {
        ++cancel_count;
        events.push_back("cancel");
      };
    callbacks.record_recovery_result = [this](const Coordinator::MotionResult & result_value) {
        recovery_results.push_back(result_value);
      };
    return Coordinator(std::move(callbacks));
  }

  void set_safety(const SafetyState & state)
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    current_safety_ = state;
  }

  void wait_for_retract_start()
  {
    std::unique_lock<std::mutex> lock(state_mutex);
    state_condition.wait(lock, [this]() {return retract_started;});
  }

  void wait_for_initial_motion_start()
  {
    std::unique_lock<std::mutex> lock(state_mutex);
    state_condition.wait(lock, [this]() {return initial_motion_started;});
  }

  void release_initial_motion_barrier()
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    release_initial_motion_execution = true;
    state_condition.notify_all();
  }

  void latch_current_safety()
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    begin_recovery_entry("mission", recovery_entries);
    observe_recovery_entry("mission", current_safety_, recovery_entries);
  }

  void release_retract_execution()
  {
    std::lock_guard<std::mutex> lock(state_mutex);
    release_retract = true;
    state_condition.notify_all();
  }

  int retract_count() const
  {
    return static_cast<int>(std::count(
             tasks.begin(), tasks.end(), MotionTaskType::TASK_TYPE_RETRACT));
  }
};
}  // namespace

TEST(ControlledRecoveryTest, WaitsForStoppedEvidenceBeforeRetract)
{
  RecoveryFixture fixture;
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPING),
    stopping_motion()});
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  auto coordinator = fixture.make();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(
    result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.retract_count(), 1);
}

TEST(ControlledRecoveryTest, InitialNoRecoveryCapabilityWaitsForLaterAuthorization)
{
  RecoveryFixture fixture;
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_NONE, SafetyState::SAFETY_STATE_STOPPING),
    stopping_motion()});
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  auto coordinator = fixture.make();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(
    result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.retract_count(), 1);
}

TEST(ControlledRecoveryTest, UnknownHoldingBlocksAutomaticRecoveryMotion)
{
  RecoveryFixture fixture;
  auto motion = stopped_motion();
  motion.holding_state = MotionState::HOLDING_UNKNOWN;
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    motion});
  auto coordinator = fixture.make();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(
    result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.retract_count(), 0);
  EXPECT_EQ(fixture.tasks.size(), 1U);
}

TEST(ControlledRecoveryTest, HigherSeverityDuringRetractPreservesEpisodeOutcome)
{
  RecoveryFixture fixture;
  fixture.hold_retract = true;
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  Coordinator::Result mission_result;
  auto coordinator = fixture.make();
  std::thread mission([&]() {mission_result = coordinator.execute("target");});

  fixture.wait_for_retract_start();
  fixture.set_safety(
    safety(
      MotionCapability::MOTION_NONE, SafetyState::SAFETY_STATE_INTERLOCKED,
      StopMode::STOP_MODE_EMERGENCY));
  fixture.events.push_back("severity-escalated");
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  fixture.release_retract_execution();
  mission.join();

  EXPECT_EQ(
    mission_result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.retract_count(), 1);
  EXPECT_EQ(fixture.cancel_count, 1);
  ASSERT_EQ(fixture.recovery_results.size(), 1U);
  EXPECT_EQ(
    fixture.recovery_results.front().code.value,
    MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(
    std::count(
      fixture.tasks.begin(), fixture.tasks.end(), MotionTaskType::TASK_TYPE_GO_HOME), 1);
  ASSERT_GE(fixture.events.size(), 3U);
  EXPECT_EQ(fixture.events[fixture.events.size() - 3], "retract");
  EXPECT_EQ(fixture.events[fixture.events.size() - 2], "severity-escalated");
  EXPECT_EQ(fixture.events[fixture.events.size() - 1], "retract-terminated");
}

TEST(ControlledRecoveryTest, HigherSeverityWhileWaitingClosesEpisode)
{
  RecoveryFixture fixture;
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_NONE, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  fixture.observations.push_back(
  {
    safety(
      MotionCapability::MOTION_NONE, SafetyState::SAFETY_STATE_INTERLOCKED,
      StopMode::STOP_MODE_IMMEDIATE),
    stopped_motion()});
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  auto coordinator = fixture.make();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(
    result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.retract_count(), 0);
}

TEST(ControlledRecoveryTest, PreemptionEntrySeverityClosesOnFirstHigherObservation)
{
  RecoveryFixture fixture;
  fixture.observations.push_back(
  {
    safety(
      MotionCapability::MOTION_NONE, SafetyState::SAFETY_STATE_INTERLOCKED,
      StopMode::STOP_MODE_IMMEDIATE),
    stopped_motion()});
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  auto coordinator = fixture.make();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(
    result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.retract_count(), 0);
}

TEST(ControlledRecoveryTest, ActiveMotionFirstLossSnapshotWinsBeforeResultRace)
{
  RecoveryFixture fixture;
  fixture.hold_initial_motion = true;
  fixture.observations.push_back(
  {
    safety(
      MotionCapability::MOTION_NONE, SafetyState::SAFETY_STATE_INTERLOCKED,
      StopMode::STOP_MODE_IMMEDIATE),
    stopped_motion()});
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  auto coordinator = fixture.make();
  Coordinator::Result mission_result;
  std::thread mission([&]() {mission_result = coordinator.execute("target");});

  fixture.wait_for_initial_motion_start();
  fixture.latch_current_safety();
  fixture.events.push_back("loss-controlled-latched");
  fixture.set_safety(
    safety(
      MotionCapability::MOTION_NONE, SafetyState::SAFETY_STATE_INTERLOCKED,
      StopMode::STOP_MODE_IMMEDIATE));
  fixture.events.push_back("escalation-immediate");
  fixture.release_initial_motion_barrier();
  mission.join();

  const auto entry = read_recovery_entry("mission", fixture.recovery_entries);
  ASSERT_TRUE(entry.has_value());
  EXPECT_EQ(entry->selected_stop_mode.value, StopMode::STOP_MODE_CONTROLLED);
  EXPECT_EQ(
    mission_result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.retract_count(), 0);
  ASSERT_GE(fixture.events.size(), 5U);
  EXPECT_EQ(fixture.events[fixture.events.size() - 4], "loss-controlled-latched");
  EXPECT_EQ(fixture.events[fixture.events.size() - 3], "escalation-immediate");
  EXPECT_EQ(fixture.events[fixture.events.size() - 2], "motion-result");
  EXPECT_EQ(fixture.events[fixture.events.size() - 1], "cancel");
}

TEST(ControlledRecoveryTest, RetractsOnceWhenStoppedAndRecoveryAuthorized)
{
  RecoveryFixture fixture;
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  auto coordinator = fixture.make();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(
    result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  ASSERT_EQ(fixture.tasks.size(), 2U);
  EXPECT_EQ(fixture.tasks[1], MotionTaskType::TASK_TYPE_RETRACT);
  EXPECT_EQ(fixture.cancel_count, 1);
}

TEST(ControlledRecoveryTest, EmergencyBeforeAuthorizationStopsProgression)
{
  RecoveryFixture fixture;
  fixture.observations.push_back(
  {
    safety(
      MotionCapability::MOTION_NONE, SafetyState::SAFETY_STATE_EMERGENCY_STOPPED,
      StopMode::STOP_MODE_EMERGENCY),
    stopped_motion()});
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  auto coordinator = fixture.make();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(
    result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.retract_count(), 0);
}

TEST(ControlledRecoveryTest, AuthorizationRemovalPreventsReentryRetry)
{
  RecoveryFixture fixture;
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPING),
    stopping_motion()});
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_NONE, SafetyState::SAFETY_STATE_STOPPING),
    stopping_motion()});
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  auto coordinator = fixture.make();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(
    result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.retract_count(), 0);
}

TEST(ControlledRecoveryTest, SecondaryCancellationPrecedesRecoveryObservation)
{
  RecoveryFixture fixture;
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  auto coordinator = fixture.make();

  (void)coordinator.execute("target");

  ASSERT_GE(fixture.events.size(), 3U);
  EXPECT_EQ(fixture.events[0], "normal");
  EXPECT_EQ(fixture.events[1], "cancel");
  EXPECT_EQ(fixture.events[2], "retract");
}

TEST(ControlledRecoveryTest, FailedRetractPreservesOriginalExitAndDoesNotRetry)
{
  RecoveryFixture fixture;
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  fixture.retract_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  auto coordinator = fixture.make();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(
    result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.retract_count(), 1);
  ASSERT_EQ(fixture.recovery_results.size(), 1U);
  EXPECT_EQ(
    fixture.recovery_results.front().code.value,
    MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_EQ(
    std::count(
      fixture.tasks.begin(), fixture.tasks.end(), MotionTaskType::TASK_TYPE_GO_HOME), 1);
}

TEST(ControlledRecoveryTest, InterruptedRetractPreservesOriginalExitAndDoesNotRetry)
{
  RecoveryFixture fixture;
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  fixture.observations.push_back(
  {
    safety(MotionCapability::MOTION_RECOVERY_ONLY, SafetyState::SAFETY_STATE_STOPPED),
    stopped_motion()});
  fixture.retract_result = MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
  auto coordinator = fixture.make();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(
    result.exit_reason.value,
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.retract_count(), 1);
  ASSERT_EQ(fixture.recovery_results.size(), 1U);
  EXPECT_EQ(
    fixture.recovery_results.front().code.value,
    MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(
    std::count(
      fixture.tasks.begin(), fixture.tasks.end(), MotionTaskType::TASK_TYPE_GO_HOME), 1);
}
