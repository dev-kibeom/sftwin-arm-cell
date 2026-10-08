#include <gtest/gtest.h>

#include <chrono>

#include <arm_cell_interfaces/msg/material_readiness.hpp>
#include <arm_cell_interfaces/msg/motion_capability.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>

#include "arm_cell_orchestration_bt/material_admission_gate.hpp"

namespace
{

using Gate = arm_cell_orchestration_bt::MaterialAdmissionGate;
using Readiness = arm_cell_interfaces::msg::MaterialReadiness;
using Safety = arm_cell_interfaces::msg::SafetyState;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

Readiness readiness(uint8_t identity, bool valid = true, bool ready = true)
{
  Readiness state;
  state.delivery_id.uuid[15] = identity;
  state.valid = valid;
  state.material_ready = ready;
  return state;
}

Safety safe_state()
{
  Safety state;
  state.valid = true;
  state.required_inputs_fresh = true;
  state.safety_state = Safety::SAFETY_STATE_SAFE;
  state.motion_capability.value =
    arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL;
  return state;
}

TEST(MaterialAdmissionGate, KeepsValidDeliveryPendingUntilSafetyPermits)
{
  Gate gate;
  const auto now = Clock::now();
  const auto timeout = 500ms;
  const auto state = readiness(1);
  gate.observe(state, now, 0ns);

  auto blocked = safe_state();
  blocked.motion_capability.value =
    arm_cell_interfaces::msg::MotionCapability::MOTION_NONE;
  EXPECT_FALSE(gate.try_admit(now, blocked, now, timeout));

  auto admitted = gate.try_admit(now, safe_state(), now, timeout);
  ASSERT_TRUE(admitted);
  EXPECT_EQ(*admitted, state.delivery_id);
}

TEST(MaterialAdmissionGate, InvalidOrExpiredReadinessCannotAdmit)
{
  Gate gate;
  const auto now = Clock::now();
  const auto timeout = 500ms;
  gate.observe(readiness(2, false, true), now, 0ns);
  EXPECT_FALSE(gate.try_admit(now, safe_state(), now, timeout));

  gate.observe(readiness(3), now, timeout);
  EXPECT_FALSE(gate.try_admit(now, safe_state(), now, timeout));
}

TEST(MaterialAdmissionGate, RevalidationCanReenableUnadmittedIdentityOnlyOnce)
{
  Gate gate;
  const auto now = Clock::now();
  const auto timeout = 500ms;
  const auto state = readiness(4);
  gate.observe(state, now, 0ns);
  gate.observe(readiness(4, false, false), now + 1ms, 0ns);
  EXPECT_FALSE(gate.try_admit(now + 2ms, safe_state(), now + 2ms, timeout));

  gate.observe(state, now + 3ms, 0ns);
  ASSERT_TRUE(gate.try_admit(now + 4ms, safe_state(), now + 4ms, timeout));
  gate.observe(state, now + 5ms, 0ns);
  EXPECT_FALSE(gate.try_admit(now + 6ms, safe_state(), now + 6ms, timeout));
}

TEST(MaterialAdmissionGate, StaleSafetyBlocksAdmission)
{
  Gate gate;
  const auto now = Clock::now();
  const auto timeout = 500ms;
  gate.observe(readiness(5), now, 0ns);
  EXPECT_FALSE(gate.try_admit(now, safe_state(), now - timeout, timeout));
}

TEST(MaterialAdmissionGate, NewDeliveryReplacesPreviousCurrentReadyState)
{
  Gate gate;
  const auto now = Clock::now();
  const auto timeout = 500ms;
  const auto old_delivery = readiness(6);
  const auto new_delivery = readiness(7, true, false);
  gate.observe(old_delivery, now, 0ns);
  gate.observe(new_delivery, now + 1ms, 0ns);
  EXPECT_FALSE(gate.try_admit(now + 2ms, safe_state(), now + 2ms, timeout));

  auto new_ready = readiness(7);
  gate.observe(new_ready, now + 3ms, 0ns);
  const auto admitted = gate.try_admit(now + 4ms, safe_state(), now + 4ms, timeout);
  ASSERT_TRUE(admitted);
  EXPECT_EQ(*admitted, new_delivery.delivery_id);
}

TEST(MaterialAdmissionGate, FreshCurrentReadyStillAdmits)
{
  Gate gate;
  const auto now = Clock::now();
  const auto state = readiness(8);
  gate.observe(state, now, 10ms);
  const auto admitted = gate.try_admit(now + 1ms, safe_state(), now + 1ms, 500ms);
  ASSERT_TRUE(admitted);
  EXPECT_EQ(*admitted, state.delivery_id);
}

}  // namespace
