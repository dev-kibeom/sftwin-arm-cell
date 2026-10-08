#include <gtest/gtest.h>

#include "arm_cell_safety_fsm/safety_core.hpp"

namespace arm_cell_safety_fsm
{
namespace
{
using Clock = SafetyCore::Clock;

SafetyInputs inputs_for_stop()
{
  SafetyInputs inputs;
  inputs.amr.valid = true;
  inputs.amr.docking_state = AMRDockingState::AMR_DOCKING_DOCKED;
  inputs.packml.valid = true;
  inputs.packml.state = PackMLState::PACKML_STATE_EXECUTE;
  inputs.hardware.valid = true;
  inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
  inputs.motion.execution_active = true;
  return inputs;
}

void record(SafetyCore & core, const SafetyInputs & inputs, Clock::time_point now)
{
  core.update_amr(inputs.amr, now);
  core.update_packml(inputs.packml, now);
  core.update_hardware(inputs.hardware, now);
  core.update_motion(inputs.motion, now);
}
}  // namespace

TEST(SafetyStopCoordinationTest, ActiveExecutionSelectsControlledStopForPrematureUndock)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = inputs_for_stop();
  inputs.amr.docking_state = AMRDockingState::AMR_DOCKING_UNDOCKED;
  record(core, inputs, now);
  const auto state = core.evaluate(now + std::chrono::milliseconds(1));

  EXPECT_TRUE(core.should_dispatch_stop());
  EXPECT_EQ(
    core.selected_stop_mode(), arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED);
  EXPECT_NE(state.active_causes.value & SafetyCauseSet::PREMATURE_UNDOCK, 0u);
}

TEST(SafetyStopCoordinationTest, InactiveMotionDoesNotDispatchRedundantStop)
{
  SafetyCore core;
  const auto now = Clock::time_point{};
  auto inputs = inputs_for_stop();
  inputs.packml.state = PackMLState::PACKML_STATE_COMPLETE;
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
  auto inputs = inputs_for_stop();
  inputs.amr.docking_state = AMRDockingState::AMR_DOCKING_UNDOCKED;
  record(core, inputs, now);
  core.evaluate(now + std::chrono::milliseconds(1));
  EXPECT_EQ(
    core.selected_stop_mode(), arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED);

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
  auto inputs = inputs_for_stop();
  inputs.amr.docking_state = AMRDockingState::AMR_DOCKING_UNDOCKED;
  record(core, inputs, now);
  core.evaluate(now + std::chrono::milliseconds(1));

  inputs.motion.execution_state = MotionStatus::MOTION_STATE_STOPPING;
  core.update_motion(inputs.motion, now + std::chrono::milliseconds(2));
  inputs.hardware.e_stop_active = true;
  core.update_hardware(inputs.hardware, now + std::chrono::milliseconds(3));
  core.evaluate(now + std::chrono::milliseconds(4));

  EXPECT_TRUE(core.should_dispatch_stop());
  EXPECT_EQ(
    core.selected_stop_mode(), arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
}

}  // namespace arm_cell_safety_fsm
