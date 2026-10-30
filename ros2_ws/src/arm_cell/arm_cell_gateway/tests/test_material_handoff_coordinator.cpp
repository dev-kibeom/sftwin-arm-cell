#include <algorithm>
#include <chrono>

#include <gtest/gtest.h>

#include "material_handoff_coordinator.hpp"

namespace arm_cell_gateway
{
namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using Trigger = arm_cell_interfaces::msg::HandoffTrigger;
using Handoff = arm_cell_interfaces::msg::MaterialHandoffState;
using Completion = arm_cell_interfaces::msg::TransferCompletion;
using DeliveryId = unique_identifier_msgs::msg::UUID;

Trigger trigger(uint8_t identity, uint32_t quantity = 2)
{
  Trigger value;
  value.delivery_id.uuid[0] = identity;
  value.quantity = quantity;
  value.material_type = "RawPart";
  value.valid = true;
  return value;
}

Handoff phase(const DeliveryId & id, uint8_t value, bool valid = true)
{
  Handoff message;
  message.delivery_id = id;
  message.phase = value;
  message.valid = valid;
  return message;
}

Completion completion(const Trigger & request)
{
  Completion value;
  value.delivery_id = request.delivery_id;
  value.material_type = request.material_type;
  value.requested_quantity = request.quantity;
  value.supplied_quantity = request.quantity;
  value.completed = true;
  value.valid = true;
  return value;
}

builtin_interfaces::msg::Time stamp()
{
  builtin_interfaces::msg::Time value;
  value.sec = 1;
  return value;
}
}  // namespace

TEST(MaterialHandoffCoordinator, RequiresValidUniquePositiveConfiguredTrigger)
{
  MaterialHandoffCoordinator coordinator(500ms);
  const auto start = Clock::time_point{};
  auto invalid = trigger(1);
  invalid.valid = false;
  EXPECT_FALSE(coordinator.begin(invalid, start));
  auto zero_quantity = trigger(2, 0);
  EXPECT_FALSE(coordinator.begin(zero_quantity, start));
  auto unsupported = trigger(3);
  unsupported.material_type = "Other";
  EXPECT_FALSE(coordinator.begin(unsupported, start));
  const auto valid = trigger(4);
  ASSERT_TRUE(coordinator.begin(valid, start));
  EXPECT_FALSE(coordinator.begin(valid, start + 1ms));
}

TEST(MaterialHandoffCoordinator, ExplainsEachTriggerRejectionWithoutChangingAcceptance)
{
  MaterialHandoffCoordinator coordinator(5s);
  auto request = trigger(41);

  EXPECT_EQ(coordinator.rejection_reason(request), "");
  auto invalid = request;
  invalid.valid = false;
  EXPECT_EQ(coordinator.rejection_reason(invalid), "invalid_trigger");
  invalid = request;
  invalid.quantity = 0;
  EXPECT_EQ(coordinator.rejection_reason(invalid), "invalid_quantity");
  invalid = request;
  invalid.material_type = "Unknown";
  EXPECT_EQ(coordinator.rejection_reason(invalid), "unsupported_material_type");
  invalid = request;
  std::fill(invalid.delivery_id.uuid.begin(), invalid.delivery_id.uuid.end(), 0);
  EXPECT_EQ(coordinator.rejection_reason(invalid), "invalid_delivery_id");

  ASSERT_TRUE(coordinator.begin(request, MaterialHandoffCoordinator::Clock::now()));
  EXPECT_EQ(coordinator.rejection_reason(request), "duplicate_delivery_id");
  auto next = trigger(42);
  EXPECT_EQ(coordinator.rejection_reason(next), "");
  ASSERT_TRUE(coordinator.begin(next, MaterialHandoffCoordinator::Clock::now()));
  EXPECT_EQ(coordinator.rejection_reason(next), "duplicate_delivery_id");
}

TEST(MaterialHandoffCoordinator, ReadinessRequiresOrderedVdaAndFullCorrelatedCompletion)
{
  MaterialHandoffCoordinator coordinator(500ms);
  const auto request = trigger(5);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(request, start));
  EXPECT_EQ(
    coordinator.observe_completion(completion(request), start, stamp()).disposition,
    MaterialHandoffCoordinator::CompletionDisposition::kCompleted);
  EXPECT_FALSE(coordinator.observe_vda(
    phase(request.delivery_id, Handoff::HANDOFF_ARRIVED), start + 1ms, stamp()));
  EXPECT_FALSE(coordinator.observe_vda(
    phase(request.delivery_id, Handoff::HANDOFF_DOCKED), start + 2ms, stamp()));
  const auto ready = coordinator.observe_vda(
    phase(request.delivery_id, Handoff::HANDOFF_UNLOADED), start + 3ms, stamp());
  ASSERT_TRUE(ready);
  EXPECT_TRUE(ready->valid);
  EXPECT_TRUE(ready->material_ready);
  EXPECT_EQ(ready->delivery_id, request.delivery_id);
  EXPECT_EQ(ready->material_type, request.material_type);
  EXPECT_EQ(ready->quantity, request.quantity);
}

TEST(MaterialHandoffCoordinator, LateCompletionFromPriorProcessCannotAdmitDelivery)
{
  // A restarted Integration coordinator has no accepted delivery identity.
  // A delayed completion from the previous Bringup is therefore ignored.
  MaterialHandoffCoordinator fresh_process(500ms);
  const auto old_delivery = trigger(88);
  const auto observation = fresh_process.observe_completion(
    completion(old_delivery), Clock::time_point{} + 1ms, stamp());

  EXPECT_EQ(
    observation.disposition,
    MaterialHandoffCoordinator::CompletionDisposition::kNoActiveDelivery);
  EXPECT_FALSE(observation.readiness);
}

TEST(MaterialHandoffCoordinator, PartialOrMismatchedCompletionNeverCreatesReadiness)
{
  MaterialHandoffCoordinator coordinator(500ms);
  const auto request = trigger(6);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(request, start));
  EXPECT_FALSE(coordinator.observe_vda(
    phase(request.delivery_id, Handoff::HANDOFF_ARRIVED), start, stamp()));
  EXPECT_FALSE(coordinator.observe_vda(
    phase(request.delivery_id, Handoff::HANDOFF_DOCKED), start + 1ms, stamp()));
  EXPECT_FALSE(coordinator.observe_vda(
    phase(request.delivery_id, Handoff::HANDOFF_UNLOADED), start + 2ms, stamp()));
  auto partial = completion(request);
  partial.supplied_quantity = 1;
  const auto rejected = coordinator.observe_completion(partial, start + 3ms, stamp());
  EXPECT_EQ(rejected.disposition, MaterialHandoffCoordinator::CompletionDisposition::kQuantityMismatch);
  ASSERT_TRUE(rejected.readiness);
  EXPECT_FALSE(rejected.readiness->material_ready);
  EXPECT_TRUE(rejected.readiness->valid);
  EXPECT_FALSE(coordinator.tick(start + 1s, stamp()));
}

TEST(MaterialHandoffCoordinator, RequiresActualSuccessfulSupplyResult)
{
  MaterialHandoffCoordinator coordinator(500ms);
  const auto request = trigger(13);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(request, start));

  auto failed = completion(request);
  failed.completed = false;
  failed.failure_reason = Completion::TRANSFER_FAILURE_SUPPLY;
  const auto rejected = coordinator.observe_completion(failed, start + 1ms, stamp());
  EXPECT_EQ(rejected.disposition, MaterialHandoffCoordinator::CompletionDisposition::kTransferFailure);
  ASSERT_TRUE(rejected.readiness);
  EXPECT_FALSE(rejected.readiness->material_ready);
  EXPECT_TRUE(rejected.readiness->valid);
}

TEST(MaterialHandoffCoordinator, ReportsCompletionDispositionFromOwnedJudgment)
{
  MaterialHandoffCoordinator coordinator(500ms);
  const auto request = trigger(51);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(request, start));

  const auto mismatch = coordinator.observe_completion(
    completion(trigger(52)), start + 1ms, stamp());
  EXPECT_EQ(mismatch.disposition, MaterialHandoffCoordinator::CompletionDisposition::kDeliveryMismatch);
  EXPECT_FALSE(mismatch.readiness);

  const auto accepted = coordinator.observe_completion(
    completion(request), start + 2ms, stamp());
  EXPECT_EQ(accepted.disposition, MaterialHandoffCoordinator::CompletionDisposition::kCompleted);

  auto partial = completion(request);
  partial.supplied_quantity = 1;
  const auto rejected = coordinator.observe_completion(partial, start + 3ms, stamp());
  EXPECT_EQ(rejected.disposition, MaterialHandoffCoordinator::CompletionDisposition::kQuantityMismatch);

  const auto after_terminal = coordinator.observe_completion(
    completion(request), start + 4ms, stamp());
  EXPECT_EQ(after_terminal.disposition, MaterialHandoffCoordinator::CompletionDisposition::kPreviousTerminalFailure);
}

TEST(MaterialHandoffCoordinator, ReportsDeadlineAndCompletionFailureDispositions)
{
  MaterialHandoffCoordinator coordinator(500ms);
  const auto request = trigger(53);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(request, start));

  const auto late = coordinator.observe_completion(
    completion(request), start + 500ms, stamp());
  EXPECT_EQ(late.disposition, MaterialHandoffCoordinator::CompletionDisposition::kDeadlineExpired);
  ASSERT_TRUE(late.readiness);
  EXPECT_FALSE(late.readiness->valid);

  ASSERT_TRUE(coordinator.begin(trigger(54), start + 501ms));
  auto failed = completion(trigger(54));
  failed.failure_reason = Completion::TRANSFER_FAILURE_SUPPLY;
  failed.completed = false;
  const auto result = coordinator.observe_completion(failed, start + 502ms, stamp());
  EXPECT_EQ(result.disposition, MaterialHandoffCoordinator::CompletionDisposition::kTransferFailure);
  EXPECT_EQ(result.failure_reason, Completion::TRANSFER_FAILURE_SUPPLY);
}

TEST(MaterialHandoffCoordinator, FailedVdaContextPreventsLaterCompletionFromReadying)
{
  MaterialHandoffCoordinator coordinator(500ms);
  const auto request = trigger(7);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(request, start));
  EXPECT_FALSE(coordinator.observe_vda(
    phase(request.delivery_id, Handoff::HANDOFF_ARRIVED), start, stamp()));
  const auto failure = coordinator.observe_vda(
    phase(request.delivery_id, Handoff::HANDOFF_FAILED), start + 1ms, stamp());
  ASSERT_TRUE(failure);
  EXPECT_FALSE(failure->material_ready);
  EXPECT_EQ(
    coordinator.observe_completion(completion(request), start + 2ms, stamp()).disposition,
    MaterialHandoffCoordinator::CompletionDisposition::kPreviousTerminalFailure);
}

TEST(MaterialHandoffCoordinator, LateCompletionAfterVdaDepartureRemainsUsable)
{
  MaterialHandoffCoordinator coordinator(2s);
  const auto request = trigger(8);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(request, start));
  auto at = start;
  for (const auto state : {
      Handoff::HANDOFF_ARRIVED, Handoff::HANDOFF_DOCKED,
      Handoff::HANDOFF_UNLOADED, Handoff::HANDOFF_DEPARTED})
  {
    EXPECT_FALSE(coordinator.observe_vda(phase(request.delivery_id, state), at, stamp()));
    at += 10ms;
  }
  EXPECT_FALSE(coordinator.tick(start + 600ms, stamp()));
  const auto ready = coordinator.observe_completion(
    completion(request), start + 600ms, stamp());
  ASSERT_TRUE(ready.readiness);
  EXPECT_TRUE(ready.readiness->valid);
  EXPECT_TRUE(ready.readiness->material_ready);
}

TEST(MaterialHandoffCoordinator, MissingVdaExpiresAtDeliveryDeadline)
{
  MaterialHandoffCoordinator coordinator(2s);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(trigger(9), start));
  EXPECT_FALSE(coordinator.tick(start + 1999ms, stamp()));
  const auto expired = coordinator.tick(start + 2s, stamp());
  ASSERT_TRUE(expired);
  EXPECT_FALSE(expired->valid);
  EXPECT_FALSE(expired->material_ready);
  EXPECT_TRUE(coordinator.begin(trigger(10), start + 2s));
}

TEST(MaterialHandoffCoordinator, MissingCompletionExpiresAtDeliveryDeadline)
{
  MaterialHandoffCoordinator coordinator(2s);
  const auto request = trigger(11);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(request, start));
  auto at = start;
  for (const auto state : {
      Handoff::HANDOFF_ARRIVED, Handoff::HANDOFF_DOCKED,
      Handoff::HANDOFF_UNLOADED, Handoff::HANDOFF_DEPARTED})
  {
    EXPECT_FALSE(coordinator.observe_vda(phase(request.delivery_id, state), at, stamp()));
    at += 10ms;
  }
  EXPECT_FALSE(coordinator.tick(start + 1999ms, stamp()));
  const auto expired = coordinator.tick(start + 2s, stamp());
  ASSERT_TRUE(expired);
  EXPECT_FALSE(expired->valid);
  EXPECT_FALSE(expired->material_ready);
  EXPECT_TRUE(coordinator.begin(trigger(12), start + 2s));
}
TEST(MaterialHandoffCoordinator, KeepsInterleavedDeliveryStateIndependent)
{
  MaterialHandoffCoordinator coordinator(2s);
  const auto start = Clock::time_point{};
  const auto first = trigger(61, 2);
  const auto second = trigger(62, 3);
  ASSERT_TRUE(coordinator.begin(first, start));
  ASSERT_TRUE(coordinator.begin(second, start + 1ms));

  EXPECT_FALSE(coordinator.observe_vda(
    phase(first.delivery_id, Handoff::HANDOFF_ARRIVED), start + 2ms, stamp()));
  EXPECT_FALSE(coordinator.observe_vda(
    phase(second.delivery_id, Handoff::HANDOFF_ARRIVED), start + 3ms, stamp()));
  EXPECT_FALSE(coordinator.observe_vda(
    phase(first.delivery_id, Handoff::HANDOFF_DOCKED), start + 4ms, stamp()));
  EXPECT_FALSE(coordinator.observe_vda(
    phase(second.delivery_id, Handoff::HANDOFF_DOCKED), start + 5ms, stamp()));
  EXPECT_FALSE(coordinator.observe_vda(
    phase(first.delivery_id, Handoff::HANDOFF_UNLOADED), start + 6ms, stamp()));
  auto second_ready = coordinator.observe_vda(
    phase(second.delivery_id, Handoff::HANDOFF_UNLOADED), start + 7ms, stamp());
  EXPECT_FALSE(second_ready);

  auto second_result = coordinator.observe_completion(
    completion(second), start + 8ms, stamp());
  ASSERT_TRUE(second_result.readiness);
  EXPECT_EQ(second_result.readiness->delivery_id, second.delivery_id);
  EXPECT_TRUE(second_result.readiness->material_ready);
  auto first_result = coordinator.observe_completion(
    completion(first), start + 9ms, stamp());
  ASSERT_TRUE(first_result.readiness);
  EXPECT_EQ(first_result.readiness->delivery_id, first.delivery_id);
  EXPECT_TRUE(first_result.readiness->material_ready);
}

TEST(MaterialHandoffCoordinator, OneDeliveryFailureDoesNotFailAnotherDelivery)
{
  MaterialHandoffCoordinator coordinator(2s);
  const auto start = Clock::time_point{};
  const auto failed_delivery = trigger(63, 1);
  const auto successful_delivery = trigger(64, 2);
  ASSERT_TRUE(coordinator.begin(failed_delivery, start));
  ASSERT_TRUE(coordinator.begin(successful_delivery, start + 1ms));

  const auto failure = coordinator.observe_vda(
    phase(failed_delivery.delivery_id, Handoff::HANDOFF_FAILED), start + 2ms, stamp());
  ASSERT_TRUE(failure);
  EXPECT_FALSE(failure->material_ready);

  auto at = start + 3ms;
  for (const auto state : {
      Handoff::HANDOFF_ARRIVED, Handoff::HANDOFF_DOCKED,
      Handoff::HANDOFF_UNLOADED})
  {
    EXPECT_FALSE(coordinator.observe_vda(
      phase(successful_delivery.delivery_id, state), at, stamp()));
    at += 1ms;
  }
  const auto result = coordinator.observe_completion(
    completion(successful_delivery), at, stamp());
  ASSERT_TRUE(result.readiness);
  EXPECT_TRUE(result.readiness->material_ready);
  EXPECT_EQ(result.readiness->delivery_id, successful_delivery.delivery_id);
  EXPECT_FALSE(coordinator.observe_completion(
    completion(failed_delivery), at + 1ms, stamp()).readiness);
}

}  // namespace arm_cell_gateway
