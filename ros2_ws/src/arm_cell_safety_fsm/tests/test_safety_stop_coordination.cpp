#include <gtest/gtest.h>

#include "arm_cell_safety_fsm/safety_core.hpp"

namespace arm_cell_safety_fsm
{
namespace
{
using Clock = SafetyCore::Clock;

SafetyInputs active_motion_inputs()
{
  SafetyInputs inputs;
  inputs.hardware.valid = true;
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  return inputs;
}

void record(SafetyCore & core, const SafetyInputs & inputs, Clock::time_point now)
{
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);
}
}  // namespace

TEST(SafetyStopCoordinationTest, MissingRequiredEvidenceStopsActiveMotionImmediately)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = active_motion_inputs();
  inputs.hardware.valid = false;
  record(core, inputs, now);
  const auto state = core.evaluate(now + std::chrono::milliseconds(1));

  EXPECT_TRUE(core.should_dispatch_stop());
  EXPECT_EQ(state.active_causes.value, SafetyCauseSet::NONE);
  EXPECT_EQ(
    core.selected_stop_mode(), arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE);
}

TEST(SafetyStopCoordinationTest, InactiveMotionDoesNotDispatchRedundantStop)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = active_motion_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_IDLE;
  inputs.motion.execution_active = false;
  record(core, inputs, now);
  core.evaluate(now + std::chrono::milliseconds(1));

  EXPECT_FALSE(core.should_dispatch_stop());
}

TEST(SafetyStopCoordinationTest, EmergencyCauseEscalatesWithoutDowngrade)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = active_motion_inputs();
  inputs.hardware.valid = false;
  record(core, inputs, now);
  core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(
    core.selected_stop_mode(), arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE);

  inputs.hardware.valid = true;
  inputs.hardware.e_stop_active = true;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(2));
  core.evaluate(now + std::chrono::milliseconds(3));
  EXPECT_EQ(
    core.selected_stop_mode(), arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
}

TEST(SafetyStopCoordinationTest, EmergencyCauseEscalatesWhileMotionIsStopping)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = active_motion_inputs();
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_STOPPING;
  record(core, inputs, now);

  inputs.hardware.e_stop_active = true;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(1));
  core.evaluate(now + std::chrono::milliseconds(2));

  EXPECT_TRUE(core.should_dispatch_stop());
  EXPECT_EQ(
    core.selected_stop_mode(), arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
}

}  // namespace arm_cell_safety_fsm
