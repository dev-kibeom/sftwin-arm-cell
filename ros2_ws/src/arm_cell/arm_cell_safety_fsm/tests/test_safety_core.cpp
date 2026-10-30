#include <gtest/gtest.h>

#include <chrono>
#include <string>

#include "arm_cell_safety_fsm/safety_core.hpp"

namespace arm_cell_safety_fsm
{
namespace
{
using Clock = SafetyInputTracker::Clock;

SafetyInputs valid_inputs()
{
  SafetyInputs inputs;
  inputs.hardware.valid = true;
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_IDLE;
  inputs.motion.backend_inactivity_confirmed = true;
  return inputs;
}

void record_all(SafetyCore & core, Clock::time_point now)
{
  core.update_hardware(valid_inputs().hardware, now);
  core.update_motion(valid_inputs().motion, now);
}
}  // namespace

TEST(SafetyCoreTest, StartsUnknownAndDeniesMotion)
{
  SafetyCore core;
  const auto state = core.evaluate(Clock::time_point{});

  EXPECT_EQ(state.safety_state, SafetyState::SAFETY_STATE_UNKNOWN);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_FALSE(state.valid);
  EXPECT_FALSE(state.required_inputs_fresh);
}

TEST(SafetyCoreTest, FreshValidInputsGrantNormalCapability)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  record_all(core, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));

  EXPECT_EQ(state.safety_state, SafetyState::SAFETY_STATE_SAFE);
  EXPECT_EQ(
    state.selected_stop_mode.value,
    arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NORMAL);
  EXPECT_TRUE(state.valid);
  EXPECT_TRUE(state.required_inputs_fresh);
  EXPECT_TRUE(state.motion_envelope_valid);
  EXPECT_FLOAT_EQ(state.max_velocity_scale, 1.0F);
  EXPECT_FLOAT_EQ(state.max_acceleration_scale, 1.0F);
}

TEST(SafetyCoreTest, ExposesDiagnosticsForEachRequiredInput)
{
  SafetyCore core(std::chrono::milliseconds(100));
  const auto start = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.hardware.valid = false;
  core.update_hardware(inputs.hardware, start);
  core.update_motion(inputs.motion, start + std::chrono::milliseconds(7));

  const auto diagnostics = core.input_diagnostics(start + std::chrono::milliseconds(20));

  EXPECT_EQ(
    diagnostics[static_cast<std::size_t>(SafetyInput::HARDWARE)].reason,
    SafetyInputReason::INVALID);
  EXPECT_EQ(
    diagnostics[static_cast<std::size_t>(SafetyInput::HARDWARE)].receipt_age,
    std::chrono::milliseconds(20));
  EXPECT_EQ(
    diagnostics[static_cast<std::size_t>(SafetyInput::MOTION)].reason,
    SafetyInputReason::OK);
  EXPECT_EQ(
    diagnostics[static_cast<std::size_t>(SafetyInput::MOTION)].receipt_age,
    std::chrono::milliseconds(13));
}

TEST(SafetyCoreTest, DiagnosticDetailDoesNotChangeForReceiptAgeAlone)
{
  SafetyCore core;
  const auto start = Clock::time_point{};
  record_all(core, start);

  const auto first = core.evaluate(start + std::chrono::milliseconds(10));
  const auto later = core.evaluate(start + std::chrono::milliseconds(20));

  EXPECT_EQ(first.diagnostic_detail, later.diagnostic_detail);
  EXPECT_NE(first.diagnostic_detail.find("required_input_status="), std::string::npos);
}

TEST(SafetyCoreTest, CommunicationDelayWithinFreshnessDoesNotRestrictMotionEnvelope)
{
  SafetyCore core(std::chrono::milliseconds(500));
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  inputs.motion.has_active_execution = true;
  inputs.motion.applied_envelope_valid = true;
  inputs.motion.applied_velocity_scale = 1.0F;
  inputs.motion.applied_acceleration_scale = 1.0F;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto delayed = core.evaluate(now + std::chrono::milliseconds(200));
  EXPECT_TRUE(delayed.required_inputs_fresh);
  EXPECT_EQ(delayed.safety_state, SafetyState::SAFETY_STATE_SAFE);
  EXPECT_EQ(delayed.motion_capability.value, MotionCapability::MOTION_NORMAL);
  EXPECT_EQ(delayed.active_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(delayed.latched_causes.value, SafetyCauseSet::NONE);
  EXPECT_TRUE(delayed.motion_envelope_valid);
  EXPECT_FLOAT_EQ(delayed.max_velocity_scale, 1.0F);
  EXPECT_FLOAT_EQ(delayed.max_acceleration_scale, 1.0F);
  EXPECT_FALSE(core.should_dispatch_stop());
}

TEST(SafetyCoreTest, StaleInputsInvalidateMotionEnvelope)
{
  SafetyCore core(std::chrono::milliseconds(500));
  const auto now = Clock::time_point{};
  record_all(core, now);

  const auto stale = core.evaluate(now + std::chrono::milliseconds(501));
  EXPECT_FALSE(stale.required_inputs_fresh);
  EXPECT_EQ(stale.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_FALSE(stale.motion_envelope_valid);
}

TEST(SafetyCoreTest, NormalSafetyEvidenceAllowsMotionWithoutExternalState)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NORMAL);
  EXPECT_TRUE(state.motion_envelope_valid);
  EXPECT_EQ(state.active_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(state.latched_causes.value, SafetyCauseSet::NONE);
}

TEST(SafetyCoreTest, EStopDeniesMotionRegardlessOfExternalState)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.hardware.e_stop_active = true;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_NE(state.active_causes.value & SafetyCauseSet::E_STOP, 0u);
}

TEST(SafetyCoreTest, StaleInputFailsClosed)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  record_all(core, now);

  const auto state = core.evaluate(now + std::chrono::seconds(2));

  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_FALSE(state.valid);
  EXPECT_FALSE(state.required_inputs_fresh);
}

TEST(SafetyCoreTest, EStopRemainsEmergencyWhenAnotherInputIsStale)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.hardware.e_stop_active = true;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::seconds(2));

  EXPECT_EQ(state.safety_state, SafetyState::SAFETY_STATE_EMERGENCY_STOPPED);
  EXPECT_EQ(
    state.selected_stop_mode.value,
    arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
  EXPECT_NE(state.active_causes.value & SafetyCauseSet::E_STOP, 0u);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
}

TEST(SafetyCoreTest, SelectedSeverityRemainsLatchedUntilReset)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  inputs.motion.has_active_execution = true;
  inputs.hardware.e_stop_active = true;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto escalated = core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(
    escalated.selected_stop_mode.value,
    StopMode::STOP_MODE_EMERGENCY);

  inputs.motion.execution_state = MotionStatus::MOTION_STATE_STOPPED;
  inputs.motion.execution_active = false;
  inputs.motion.backend_inactivity_confirmed = true;
  inputs.hardware.e_stop_active = false;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(2));
  core.update_motion(inputs.motion, now + std::chrono::milliseconds(2));
  const auto recovery = core.evaluate(now + std::chrono::milliseconds(3));
  EXPECT_EQ(
    recovery.selected_stop_mode.value,
    StopMode::STOP_MODE_EMERGENCY);
}

TEST(SafetyCoreTest, FreshUnknownInputFailsClosed)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_UNKNOWN;
  inputs.motion.execution_active = false;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));

  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_FALSE(state.valid);
}

TEST(SafetyCoreTest, MotionSourceTimestampAnomalyIsDiagnosticOnlyDuringExecution)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  inputs.motion.has_active_execution = true;
  inputs.motion.applied_envelope_valid = true;
  inputs.motion.applied_velocity_scale = 1.0F;
  inputs.motion.applied_acceleration_scale = 1.0F;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now, false);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));
  const auto diagnostics = core.input_diagnostics(now + std::chrono::milliseconds(1));

  EXPECT_EQ(state.safety_state, SafetyState::SAFETY_STATE_SAFE);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NORMAL);
  EXPECT_TRUE(state.valid);
  EXPECT_TRUE(state.required_inputs_fresh);
  EXPECT_TRUE(state.motion_envelope_valid);
  EXPECT_FALSE(core.should_dispatch_stop());
  EXPECT_EQ(
    diagnostics[static_cast<std::size_t>(SafetyInput::MOTION)].reason,
    SafetyInputReason::OK);
}

TEST(SafetyCoreTest, ImplausibleHardwareSourceTimestampIsDiagnosticOnly)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  const auto inputs = valid_inputs();
  core.update_hardware(inputs.hardware, now, false);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));
  const auto diagnostics = core.input_diagnostics(now + std::chrono::milliseconds(1));

  EXPECT_EQ(state.safety_state, SafetyState::SAFETY_STATE_SAFE);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NORMAL);
  EXPECT_TRUE(state.valid);
  EXPECT_TRUE(state.required_inputs_fresh);
  EXPECT_TRUE(state.motion_envelope_valid);
  EXPECT_EQ(state.active_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(state.latched_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(
    diagnostics[static_cast<std::size_t>(SafetyInput::HARDWARE)].reason,
    SafetyInputReason::OK);
  EXPECT_FALSE(
    diagnostics[static_cast<std::size_t>(SafetyInput::HARDWARE)].source_timestamp_plausible);
}

TEST(SafetyCoreTest, StartupInvalidInputRemainsUnknownWithoutRecoveryLatch)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_UNKNOWN;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));

  EXPECT_FALSE(state.valid);
  EXPECT_EQ(state.safety_state, SafetyState::SAFETY_STATE_UNKNOWN);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_EQ(state.active_causes.value & SafetyCauseSet::COMMUNICATION_LOSS, 0u);
  EXPECT_EQ(state.active_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(state.latched_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
}

TEST(SafetyCoreTest, StartupInputsCanRecoverWithoutOperatorReset)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.hardware.valid = false;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);
  const auto startup = core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(startup.safety_state, SafetyState::SAFETY_STATE_UNKNOWN);
  EXPECT_EQ(startup.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_EQ(startup.latched_causes.value, SafetyCauseSet::NONE);

  inputs.hardware.valid = true;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(2));
  const auto recovered = core.evaluate(now + std::chrono::milliseconds(3));
  EXPECT_EQ(recovered.safety_state, SafetyState::SAFETY_STATE_SAFE);
  EXPECT_EQ(recovered.motion_capability.value, MotionCapability::MOTION_NORMAL);
  EXPECT_EQ(recovered.latched_causes.value, SafetyCauseSet::NONE);
}

TEST(SafetyCoreTest, StartupMotionTimestampAnomalyDoesNotBlockFreshSample)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  const auto inputs = valid_inputs();
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now, false);
  const auto startup = core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(startup.safety_state, SafetyState::SAFETY_STATE_SAFE);
  EXPECT_EQ(startup.motion_capability.value, MotionCapability::MOTION_NORMAL);
  EXPECT_TRUE(startup.required_inputs_fresh);
  EXPECT_EQ(startup.latched_causes.value, SafetyCauseSet::NONE);
}

TEST(SafetyCoreTest, StartupEStopAndStoLatchUntilAcknowledgedReset)
{
  for (const bool e_stop_active : {true, false}) {
    SafetyCore core;
    const auto now = Clock::time_point{};
    auto inputs = valid_inputs();
    inputs.hardware.e_stop_active = e_stop_active;
    inputs.hardware.sto_active = !e_stop_active;
    core.update_hardware(inputs.hardware, now);

    const auto emergency = core.evaluate(now + std::chrono::milliseconds(1));
    EXPECT_EQ(emergency.safety_state, SafetyState::SAFETY_STATE_EMERGENCY_STOPPED);
    EXPECT_EQ(emergency.motion_capability.value, MotionCapability::MOTION_NONE);
    EXPECT_NE(
      emergency.latched_causes.value &
      (e_stop_active ? SafetyCauseSet::E_STOP : SafetyCauseSet::STO), 0u);
    EXPECT_EQ(
      emergency.latched_causes.value & SafetyCauseSet::REQUIRED_INPUT_INVALID, 0u);

    inputs.hardware.e_stop_active = false;
    inputs.hardware.sto_active = false;
    core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(2));
    core.update_motion(inputs.motion, now + std::chrono::milliseconds(2));
    const auto cleared = core.evaluate(now + std::chrono::milliseconds(3));
    EXPECT_EQ(cleared.active_causes.value, SafetyCauseSet::NONE);
    EXPECT_NE(
      cleared.latched_causes.value &
      (e_stop_active ? SafetyCauseSet::E_STOP : SafetyCauseSet::STO), 0u);
    EXPECT_EQ(cleared.motion_capability.value, MotionCapability::MOTION_NONE);
    EXPECT_FALSE(core.reset(false, now + std::chrono::milliseconds(4)));
    EXPECT_TRUE(core.reset(true, now + std::chrono::milliseconds(5)));
    EXPECT_EQ(
      core.evaluate(now + std::chrono::milliseconds(6)).motion_capability.value,
      MotionCapability::MOTION_NORMAL);
  }
}

TEST(SafetyCoreTest, ResetReportsConcreteRejectedPrecondition)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  record_all(core, now);
  core.evaluate(now + std::chrono::milliseconds(1));

  std::string diagnostic_detail;
  EXPECT_FALSE(core.reset(false, now + std::chrono::milliseconds(2), &diagnostic_detail));
  EXPECT_EQ(diagnostic_detail, "operator_acknowledgement_missing");
}

TEST(SafetyCoreTest, InvalidRequiredInputDuringExecutionRequiresSafetyStop)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  inputs.motion.has_active_execution = true;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  EXPECT_TRUE(core.evaluate(now + std::chrono::milliseconds(1)).valid);
  inputs.hardware.valid = false;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(2));

  const auto state = core.evaluate(now + std::chrono::milliseconds(3));
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_EQ(state.active_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(state.latched_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(state.selected_stop_mode.value, StopMode::STOP_MODE_IMMEDIATE);
  EXPECT_TRUE(core.should_dispatch_stop());
}

TEST(SafetyCoreTest, InvalidMotionSampleWithActiveExecutionEvidenceRequiresStop)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_UNKNOWN;
  inputs.motion.execution_active = true;
  inputs.motion.has_active_execution = true;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(state.active_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(state.latched_causes.value, SafetyCauseSet::NONE);
  EXPECT_TRUE(core.should_dispatch_stop());
}

TEST(SafetyCoreTest, StaleRequiredInputDuringExecutionRequiresSafetyStop)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  inputs.motion.has_active_execution = true;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);
  EXPECT_TRUE(core.evaluate(now + std::chrono::milliseconds(1)).valid);

  const auto state = core.evaluate(now + std::chrono::seconds(2));
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_EQ(state.active_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(state.latched_causes.value, SafetyCauseSet::NONE);
  EXPECT_TRUE(core.should_dispatch_stop());
}

TEST(SafetyCoreTest, StaleMotionReceiptDuringExecutionStillRequiresSafetyStop)
{
  SafetyCore core(std::chrono::milliseconds(500));
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  inputs.motion.has_active_execution = true;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(501));

  EXPECT_FALSE(state.required_inputs_fresh);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_TRUE(core.should_dispatch_stop());
}

TEST(SafetyCoreTest, NeverReceivedRequiredInputDuringExecutionRequiresSafetyStop)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  inputs.motion.has_active_execution = true;
  core.update_motion(inputs.motion, now);

  auto state = core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_EQ(state.latched_causes.value & SafetyCauseSet::COMMUNICATION_LOSS, 0u);
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(2));
  EXPECT_TRUE(core.evaluate(now + std::chrono::milliseconds(3)).valid);
  state = core.evaluate(now + std::chrono::seconds(2));
  EXPECT_EQ(state.active_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(state.latched_causes.value, SafetyCauseSet::NONE);
  EXPECT_TRUE(core.should_dispatch_stop());
}

TEST(SafetyCoreTest, InvalidOrStaleInputWithoutActiveMotionDoesNotRequireStop)
{
  for (const bool stale : {false, true}) {
    SafetyCore core;
    const auto now = Clock::time_point{};
    auto inputs = valid_inputs();
    core.update_hardware(inputs.hardware, now);
    core.update_motion(inputs.motion, now);

    EXPECT_TRUE(core.evaluate(now + std::chrono::milliseconds(1)).valid);
    if (!stale) {
      inputs.hardware.valid = false;
      core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(2));
    }

    const auto state = core.evaluate(
      now + (stale ? std::chrono::seconds(2) : std::chrono::milliseconds(3)));
    EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
    EXPECT_EQ(state.latched_causes.value, SafetyCauseSet::NONE);
    EXPECT_FALSE(core.should_dispatch_stop());
  }
}

TEST(SafetyCoreTest, ActiveCauseIsLatchedAndEscalatesToEmergency)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  record_all(core, now);
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  inputs.motion.has_active_execution = true;
  inputs.hardware.e_stop_active = true;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(1));
  core.update_motion(inputs.motion, now + std::chrono::milliseconds(1));

  auto state = core.evaluate(now + std::chrono::milliseconds(2));
  EXPECT_NE(state.active_causes.value & SafetyCauseSet::E_STOP, 0u);
  EXPECT_NE(state.latched_causes.value & SafetyCauseSet::E_STOP, 0u);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);

  inputs.hardware.e_stop_active = true;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(3));
  state = core.evaluate(now + std::chrono::milliseconds(4));
  EXPECT_EQ(state.safety_state, SafetyState::SAFETY_STATE_EMERGENCY_STOPPED);
  EXPECT_NE(state.active_causes.value & SafetyCauseSet::E_STOP, 0u);
}

TEST(SafetyCoreTest, NormalInputsDoNotCreateExternalStateInterlocks)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));

  EXPECT_EQ(state.active_causes.value & SafetyCauseSet::PREMATURE_UNDOCK, 0u);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NORMAL);
}

TEST(SafetyCoreTest, ActiveMotionWithAppliedEnvelopeNeedsNoExternalSafetyInterlock)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  inputs.motion.applied_envelope_valid = true;
  inputs.motion.applied_velocity_scale = 1.0F;
  inputs.motion.applied_acceleration_scale = 1.0F;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(state.active_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NORMAL);
  EXPECT_EQ(state.selected_stop_mode.value, StopMode::STOP_MODE_CONTROLLED);
  EXPECT_FALSE(core.should_dispatch_stop());
}

TEST(SafetyCoreTest, StoppedStateWithoutConfirmedInactivityCannotGrantRecovery)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_active = true;
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_STOPPED;
  inputs.motion.execution_active = false;
  inputs.motion.backend_inactivity_confirmed = false;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);
  core.evaluate(now + std::chrono::milliseconds(1));

  const auto state = core.evaluate(now + std::chrono::milliseconds(2));
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_FALSE(core.motion_inactive());
}

TEST(SafetyCoreTest, MotionInactivityWithoutHazardDoesNotCreateRecoveryEpisode)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);
  EXPECT_EQ(
    core.evaluate(now + std::chrono::milliseconds(1)).motion_capability.value,
    MotionCapability::MOTION_NORMAL);

  inputs.motion.execution_state = MotionStatus::MOTION_STATE_STOPPED;
  inputs.motion.execution_active = false;
  inputs.motion.has_active_execution = false;
  inputs.motion.backend_inactivity_confirmed = true;
  core.update_motion(inputs.motion, now + std::chrono::milliseconds(2));

  const auto state = core.evaluate(now + std::chrono::milliseconds(3));
  EXPECT_EQ(state.safety_state, SafetyState::SAFETY_STATE_SAFE);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NORMAL);
}

TEST(SafetyCoreTest, ResetAcceptsConfirmedInactiveStoppedMotionAndOperatorAcknowledgement)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  record_all(core, now);
  auto inputs = valid_inputs();
  inputs.hardware.e_stop_active = true;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(1));
  inputs.motion.execution_active = true;
  core.update_motion(inputs.motion, now + std::chrono::milliseconds(1));
  core.evaluate(now + std::chrono::milliseconds(2));

  EXPECT_FALSE(core.reset(false, now + std::chrono::milliseconds(3)));
  EXPECT_FALSE(core.reset(true, now + std::chrono::milliseconds(3)));

  inputs.motion.execution_state = MotionStatus::MOTION_STATE_STOPPED;
  inputs.motion.execution_active = false;
  inputs.motion.has_active_execution = false;
  inputs.motion.backend_inactivity_confirmed = true;
  inputs.hardware.e_stop_active = false;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(4));
  core.update_motion(inputs.motion, now + std::chrono::milliseconds(4));
  EXPECT_TRUE(core.reset(true, now + std::chrono::milliseconds(5)));
  EXPECT_EQ(
    core.evaluate(now + std::chrono::milliseconds(6)).motion_capability.value,
    MotionCapability::MOTION_NORMAL);
}

TEST(SafetyCoreTest, TransientRequiredInputInvalidityRecoversWithoutReset)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);
  EXPECT_TRUE(core.motion_inactive());
  EXPECT_TRUE(core.evaluate(now + std::chrono::milliseconds(1)).valid);

  inputs.hardware.valid = false;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(2));
  const auto fault = core.evaluate(now + std::chrono::milliseconds(3));
  EXPECT_FALSE(fault.required_inputs_fresh);
  EXPECT_EQ(fault.latched_causes.value, SafetyCauseSet::NONE);
  EXPECT_FALSE(core.should_dispatch_stop());

  inputs.hardware.valid = true;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(4));
  const auto recovered = core.evaluate(now + std::chrono::milliseconds(5));
  EXPECT_TRUE(recovered.valid);
  EXPECT_EQ(recovered.motion_capability.value, MotionCapability::MOTION_NORMAL);
  EXPECT_EQ(recovered.latched_causes.value, SafetyCauseSet::NONE);
  EXPECT_FALSE(core.reset(true, now + std::chrono::milliseconds(6)));
}

TEST(SafetyCoreTest, ResetRejectsInvalidOrStaleRequiredInputs)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);
  core.evaluate(now + std::chrono::milliseconds(1));

  inputs.hardware.valid = false;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(2));
  core.evaluate(now + std::chrono::milliseconds(3));
  EXPECT_FALSE(core.reset(true, now + std::chrono::milliseconds(4)));
  EXPECT_FALSE(core.reset(true, now + std::chrono::seconds(2)));
}

TEST(SafetyCoreTest, ResetRejectsActiveEStopOrSto)
{
  for (const bool e_stop_active : {true, false}) {
    SafetyCore core;
    const auto now = Clock::time_point{};
    auto inputs = valid_inputs();
    core.update_hardware(inputs.hardware, now);
    core.update_motion(inputs.motion, now);
    core.evaluate(now + std::chrono::milliseconds(1));

    core.evaluate(now + std::chrono::milliseconds(3));
    inputs.hardware.e_stop_active = e_stop_active;
    inputs.hardware.sto_active = !e_stop_active;
    core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(4));
    EXPECT_FALSE(core.reset(true, now + std::chrono::milliseconds(5)));
  }
}

TEST(SafetyCoreTest, ActiveExecutionFaultCannotResetFromIdleWithoutStopCompletion)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  inputs.motion.has_active_execution = true;
  inputs.hardware.e_stop_active = true;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);
  EXPECT_NE(
    core.evaluate(now + std::chrono::milliseconds(1)).latched_causes.value &
    SafetyCauseSet::E_STOP, 0u);

  core.evaluate(now + std::chrono::milliseconds(3));
  inputs.hardware.e_stop_active = false;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(3));
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_IDLE;
  inputs.motion.execution_active = false;
  inputs.motion.has_active_execution = false;
  inputs.motion.backend_inactivity_confirmed = false;
  core.update_motion(inputs.motion, now + std::chrono::milliseconds(4));

  EXPECT_FALSE(core.reset(true, now + std::chrono::milliseconds(5)));
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_STOPPING;
  core.update_motion(inputs.motion, now + std::chrono::milliseconds(6));
  EXPECT_TRUE(core.reset(true, now + std::chrono::milliseconds(7)));
  auto state = core.evaluate(now + std::chrono::milliseconds(8));
  EXPECT_EQ(state.latched_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_STOPPED;
  inputs.motion.backend_inactivity_confirmed = true;
  core.update_motion(inputs.motion, now + std::chrono::milliseconds(9));
  state = core.evaluate(now + std::chrono::milliseconds(10));
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NORMAL);
}

TEST(SafetyCoreTest, ActiveExecutionIdentityBlocksRecoveryAndReset)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_STOPPED;
  inputs.motion.has_active_execution = true;
  inputs.motion.execution_active = false;
  inputs.motion.backend_inactivity_confirmed = true;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_FALSE(core.reset(true, now + std::chrono::milliseconds(2)));
}

TEST(SafetyCoreTest, MotionStoppingFaultedOrEmergencyStateFailsClosed)
{
  for (const auto motion_state : {
      MotionStatus::MOTION_STATE_STOPPING,
      MotionStatus::MOTION_STATE_FAULTED,
      MotionStatus::MOTION_STATE_EMERGENCY_STOPPED})
  {
    SafetyCore core;
    const auto now = Clock::time_point{};
    auto inputs = valid_inputs();
    inputs.motion.execution_state = motion_state;
    core.update_hardware(inputs.hardware, now);
    core.update_motion(inputs.motion, now);

    EXPECT_EQ(
      core.evaluate(now + std::chrono::milliseconds(1)).motion_capability.value,
      MotionCapability::MOTION_NONE);
  }
}

TEST(SafetyCoreTest, ContradictoryStoppedMotionStatusFailsClosed)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = valid_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_STOPPED;
  inputs.motion.has_active_execution = true;
  inputs.motion.backend_inactivity_confirmed = false;
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);

  const auto state = core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(state.motion_capability.value, MotionCapability::MOTION_NONE);
  EXPECT_FALSE(state.valid);
}

}  // namespace arm_cell_safety_fsm
