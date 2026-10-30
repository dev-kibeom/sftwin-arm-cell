#include <gtest/gtest.h>

#include <chrono>
#include <optional>
#include <string>

#include <arm_cell_interfaces/msg/material_readiness.hpp>
#include <arm_cell_interfaces/msg/motion_capability.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>

#include "arm_cell_orchestration/material_admission_gate.hpp"

namespace
{

using Gate = arm_cell_orchestration::MaterialAdmissionGate;
using Readiness = arm_cell_interfaces::msg::MaterialReadiness;
using Safety = arm_cell_orchestration::SafetyObservation;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

Readiness readiness(uint8_t identity, bool valid = true, bool ready = true)
{
  Readiness state;
  state.delivery_id.uuid[15] = identity;
  state.material_type = "RawPart";
  state.quantity = 2;
  state.valid = valid;
  state.material_ready = ready;
  return state;
}

Safety safe_state(
  Clock::time_point receipt = Clock::now(), Clock::time_point now = Clock::now())
{
  arm_cell_interfaces::msg::SafetyState state;
  state.valid = true;
  state.required_inputs_fresh = true;
  state.safety_state = arm_cell_interfaces::msg::SafetyState::SAFETY_STATE_SAFE;
  state.motion_capability.value =
    arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL;
  return arm_cell_orchestration::evaluate_safety_observation(
    state, true, receipt, now, 500ms);
}

std::optional<Gate::Admission> try_admit(
  Gate & gate, const Safety & safety, std::string * block_reason = nullptr)
{
  return gate.try_admit(safety, "RawPart", block_reason);
}

TEST(MaterialAdmissionGate, KeepsValidDeliveryPendingUntilSafetyPermits)
{
  Gate gate;
  const auto now = Clock::now();
  const auto state = readiness(1);
  gate.observe(state);

  auto blocked = safe_state(now);
  blocked.state.motion_capability.value =
    arm_cell_interfaces::msg::MotionCapability::MOTION_NONE;
  EXPECT_FALSE(try_admit(gate, blocked));

  const auto after_wait = now + 2s;
  auto admitted = try_admit(gate, safe_state(after_wait, after_wait));
  ASSERT_TRUE(admitted);
  EXPECT_EQ(admitted->delivery_id, state.delivery_id);
  EXPECT_EQ(admitted->material_type, state.material_type);
  EXPECT_EQ(admitted->quantity, state.quantity);
}

TEST(MaterialAdmissionGate, InvalidReadinessCannotEstablishCompletion)
{
  Gate gate;
  const auto now = Clock::now();
  gate.observe(readiness(2, false, true));
  EXPECT_FALSE(try_admit(gate, safe_state(now)));

  gate.observe(readiness(3));
  const auto later = now + 2s;
  EXPECT_TRUE(try_admit(gate, safe_state(later, later)));
}

TEST(MaterialAdmissionGate, InvalidUpdateCannotCancelConfirmedCompletion)
{
  Gate gate;
  const auto now = Clock::now();
  const auto completed = readiness(9);
  gate.observe(completed);
  gate.observe(readiness(9, false, false));

  const auto later = now + 2s;
  const auto admitted = try_admit(gate, safe_state(later, later));
  ASSERT_TRUE(admitted);
  EXPECT_EQ(admitted->delivery_id, completed.delivery_id);
}

TEST(MaterialAdmissionGate, AdmittedDeliveryFactSurvivesLaterDeliveryUpdates)
{
  Gate gate;
  const auto accepted = readiness(21);
  gate.observe(accepted);
  const auto now = Clock::now();
  ASSERT_TRUE(try_admit(gate, safe_state(now + 1ms, now + 1ms)));
  gate.observe(readiness(22, true, false));

  EXPECT_TRUE(gate.has_admitted_delivery(accepted.delivery_id, accepted.material_type));
}

TEST(MaterialAdmissionGate, DuplicateCompletionCannotAdmitDeliveryTwice)
{
  Gate gate;
  const auto now = Clock::now();
  const auto completed = readiness(13);
  gate.observe(completed);
  ASSERT_TRUE(try_admit(gate, safe_state(now)));

  gate.observe(completed);
  EXPECT_FALSE(try_admit(gate, safe_state(now + 1s, now + 1s)));

  gate.observe(readiness(19));
  const auto next = try_admit(gate, safe_state(now + 2s, now + 2s));
  ASSERT_TRUE(next);
  EXPECT_EQ(next->quantity, 4U);
}

TEST(MaterialAdmissionGate, NewNotReadyDeliveryDoesNotEraseOlderCompletion)
{
  Gate gate;
  const auto now = Clock::now();
  const auto completed = readiness(14);
  gate.observe(completed);
  gate.observe(readiness(15, true, false));

  const auto later = now + 2s;
  const auto admitted = try_admit(gate, safe_state(later, later));
  ASSERT_TRUE(admitted);
  EXPECT_EQ(admitted->delivery_id, completed.delivery_id);
}

TEST(MaterialAdmissionGate, RejectsMalformedCompletionTuple)
{
  Gate gate;
  const auto now = Clock::now();
  auto zero_quantity = readiness(16);
  zero_quantity.quantity = 0;
  gate.observe(zero_quantity);
  auto empty_type = readiness(17);
  empty_type.material_type.clear();
  gate.observe(empty_type);
  EXPECT_FALSE(gate.observe(readiness(0)));

  EXPECT_FALSE(try_admit(gate, safe_state(now)));
}

TEST(MaterialAdmissionGate, UnsupportedMaterialTypeCannotBeAdmitted)
{
  Gate gate;
  const auto now = Clock::now();
  auto unsupported = readiness(18);
  unsupported.material_type = "OtherPart";
  gate.observe(unsupported);

  std::string reason;
  EXPECT_FALSE(gate.try_admit(safe_state(now), "RawPart", &reason));
  EXPECT_EQ(reason, "readiness_material_type_unsupported");
  EXPECT_TRUE(gate.try_admit(safe_state(now), "OtherPart"));
}

TEST(MaterialAdmissionGate, InvalidReplayCannotUndoOrRepeatConfirmedAdmission)
{
  Gate gate;
  const auto now = Clock::now();
  const auto state = readiness(4);
  gate.observe(state);
  gate.observe(readiness(4, false, false));

  ASSERT_TRUE(try_admit(gate, safe_state(now)));
  gate.observe(state);
  EXPECT_FALSE(try_admit(gate, safe_state(now + 1s, now + 1s)));
}

TEST(MaterialAdmissionGate, StaleSafetyBlocksAdmission)
{
  Gate gate;
  const auto now = Clock::now();
  gate.observe(readiness(5));
  EXPECT_FALSE(try_admit(gate, safe_state(now - 500ms, now)));
}

TEST(MaterialAdmissionGate, ReportsTheFirstAdmissionBlockReason)
{
  Gate gate;
  const auto now = Clock::now();
  std::string reason;
  auto safety = safe_state(now);
  safety.state.required_inputs_fresh = false;
  safety = arm_cell_orchestration::evaluate_safety_observation(
    safety.state, true, now, now, 500ms);
  EXPECT_FALSE(try_admit(gate, safety, &reason));
  EXPECT_EQ(reason, "safety_permission_unavailable");

  reason.clear();
  EXPECT_FALSE(try_admit(gate, safe_state(now), &reason));
  EXPECT_EQ(reason, "readiness_unavailable");

  gate.observe(readiness(10, false, true));
  reason.clear();
  EXPECT_FALSE(try_admit(gate, safe_state(now), &reason));
  EXPECT_EQ(reason, "readiness_invalid");

  gate.observe(readiness(11, true, false));
  reason.clear();
  EXPECT_FALSE(try_admit(gate, safe_state(now), &reason));
  EXPECT_EQ(reason, "readiness_not_ready");

  reason.clear();
  EXPECT_FALSE(try_admit(gate, safe_state(now - 500ms, now), &reason));
  EXPECT_EQ(reason, "safety_permission_unavailable");
}

TEST(MaterialAdmissionGate, AdmitsReadyDeliveriesInObservedOrder)
{
  Gate gate;
  const auto now = Clock::now();
  const auto old_delivery = readiness(6);
  const auto new_delivery = readiness(7);
  gate.observe(old_delivery);
  gate.observe(new_delivery);
  const auto latest_admission = try_admit(gate, safe_state(now));
  ASSERT_TRUE(latest_admission);
  EXPECT_EQ(latest_admission->delivery_id, old_delivery.delivery_id);

  const auto retained_admission = try_admit(gate, safe_state(now + 1ms, now + 1ms));
  ASSERT_TRUE(retained_admission);
  EXPECT_EQ(retained_admission->delivery_id, new_delivery.delivery_id);
}

TEST(MaterialAdmissionGate, AdmissionSnapshotsAggregateOutstandingSupply)
{
  Gate gate;
  const auto now = Clock::now();
  const auto first = readiness(21);
  auto second = readiness(22);
  second.quantity = 3;
  gate.observe(first);
  gate.observe(second);

  const auto first_admission = try_admit(gate, safe_state(now));
  ASSERT_TRUE(first_admission);
  EXPECT_EQ(first_admission->delivery_id, first.delivery_id);
  EXPECT_EQ(first_admission->quantity, 5U);

  EXPECT_EQ(gate.record_successful_place("cycle-one", 5U),
    Gate::PlaceCommitResult::COMMITTED);
  auto third = readiness(25);
  third.quantity = 2;
  gate.observe(third);
  const auto next_admission = try_admit(gate, safe_state(now + 1s, now + 1s));
  ASSERT_TRUE(next_admission);
  EXPECT_EQ(next_admission->delivery_id, third.delivery_id);
  EXPECT_EQ(next_admission->quantity, 2U);
}

TEST(MaterialAdmissionGate, NewReadinessCannotChangeAnExistingAdmissionSnapshot)
{
  Gate gate;
  const auto now = Clock::now();
  const auto first = readiness(26);
  gate.observe(first);
  const auto active_cycle = try_admit(gate, safe_state(now));
  ASSERT_TRUE(active_cycle);
  ASSERT_EQ(active_cycle->quantity, first.quantity);

  auto later = readiness(27);
  later.quantity = 3;
  gate.observe(later);
  EXPECT_EQ(active_cycle->quantity, first.quantity);
  const auto pending_cycle = try_admit(gate, safe_state(now + 1s, now + 1s));
  ASSERT_TRUE(pending_cycle);
  EXPECT_EQ(pending_cycle->delivery_id, later.delivery_id);
  EXPECT_EQ(pending_cycle->quantity, 5U);
}

TEST(MaterialAdmissionGate, LaterAdmissionIncludesResidualAndNewSupply)
{
  Gate gate;
  const auto now = Clock::now();
  auto first = readiness(23);
  first.quantity = 3;
  gate.observe(first);
  const auto initial = try_admit(gate, safe_state(now));
  ASSERT_TRUE(initial);
  EXPECT_EQ(initial->quantity, 3U);

  EXPECT_EQ(gate.record_successful_place("cycle-one", 1U),
    Gate::PlaceCommitResult::COMMITTED);
  EXPECT_EQ(gate.record_successful_place("cycle-one", 1U),
    Gate::PlaceCommitResult::DUPLICATE);

  auto second = readiness(24);
  second.quantity = 2;
  gate.observe(second);
  const auto next = try_admit(gate, safe_state(now + 1s, now + 1s));
  ASSERT_TRUE(next);
  EXPECT_EQ(next->delivery_id, second.delivery_id);
  EXPECT_EQ(next->quantity, 4U);
}

TEST(MaterialAdmissionGate, AccountingMismatchPreservesPlaceFactAndBlocksFutureAdmission)
{
  Gate gate;
  const auto now = Clock::now();
  const auto supplied = readiness(28);
  gate.observe(supplied);
  ASSERT_EQ(gate.record_successful_place("cycle-one", 1U),
    Gate::PlaceCommitResult::COMMITTED);
  ASSERT_EQ(gate.record_successful_place("cycle-one", 1U),
    Gate::PlaceCommitResult::DUPLICATE);

  EXPECT_EQ(gate.record_successful_place("cycle-one", 6U),
    Gate::PlaceCommitResult::ACCOUNTING_INCONSISTENT);
  auto later = readiness(29);
  gate.observe(later);
  std::string reason;
  EXPECT_FALSE(gate.try_admit(safe_state(now), "raw_part", &reason));
  EXPECT_EQ(reason, "material_accounting_inconsistent");
}

TEST(MaterialAdmissionGate, ValidCompletionAdmits)
{
  Gate gate;
  const auto now = Clock::now();
  const auto state = readiness(8);
  gate.observe(state);
  const auto admitted = try_admit(gate, safe_state(now + 1ms, now + 1ms));
  ASSERT_TRUE(admitted);
  EXPECT_EQ(admitted->delivery_id, state.delivery_id);
}

}  // namespace
