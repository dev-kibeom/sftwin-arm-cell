#include <chrono>

#include <gtest/gtest.h>

#include "arm_cell_orchestration/safety_observation.hpp"

namespace arm_cell_orchestration
{
namespace
{
using Clock = SafetyObservation::Clock;
using SafetyState = arm_cell_interfaces::msg::SafetyState;
using MotionCapability = arm_cell_interfaces::msg::MotionCapability;

SafetyState normal_safety()
{
  SafetyState state;
  state.safety_state = SafetyState::SAFETY_STATE_SAFE;
  state.valid = true;
  state.required_inputs_fresh = true;
  state.motion_capability.value = MotionCapability::MOTION_NORMAL;
  return state;
}

TEST(SafetyObservationTest, CurrentNormalPermissionIsAvailable)
{
  const auto now = Clock::now();
  const auto observation = evaluate_safety_observation(
    normal_safety(), true, now - std::chrono::milliseconds(10), now,
    std::chrono::milliseconds(500));

  EXPECT_TRUE(observation.delivery_valid);
  EXPECT_TRUE(observation.normal_motion_allowed());
}

TEST(SafetyObservationTest, MissingAndExpiredPermissionAreUnavailable)
{
  const auto now = Clock::now();
  const auto missing = evaluate_safety_observation(
    normal_safety(), false, Clock::time_point{}, now, std::chrono::milliseconds(500));
  const auto expired = evaluate_safety_observation(
    normal_safety(), true, now - std::chrono::milliseconds(501), now,
    std::chrono::milliseconds(500));

  EXPECT_FALSE(missing.delivery_valid);
  EXPECT_FALSE(missing.normal_motion_allowed());
  EXPECT_FALSE(expired.delivery_valid);
  EXPECT_FALSE(expired.normal_motion_allowed());
}

TEST(SafetyObservationTest, InvalidOrUnsupervisedStateIsUnavailable)
{
  const auto now = Clock::now();
  auto state = normal_safety();
  state.valid = false;
  const auto invalid = evaluate_safety_observation(
    state, true, now, now, std::chrono::milliseconds(500));

  state = normal_safety();
  state.required_inputs_fresh = false;
  const auto unsupervised = evaluate_safety_observation(
    state, true, now, now, std::chrono::milliseconds(500));

  EXPECT_FALSE(invalid.delivery_valid);
  EXPECT_FALSE(invalid.normal_motion_allowed());
  EXPECT_FALSE(unsupervised.delivery_valid);
  EXPECT_FALSE(unsupervised.normal_motion_allowed());
}

}  // namespace
}  // namespace arm_cell_orchestration
