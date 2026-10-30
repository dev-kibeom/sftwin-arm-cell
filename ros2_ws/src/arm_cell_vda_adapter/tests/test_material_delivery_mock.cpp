#include <chrono>
#include <optional>

#include <gtest/gtest.h>
#include <arm_cell_interfaces/msg/handoff_trigger.hpp>

#include "arm_cell_vda_adapter/material_delivery_mock.hpp"

namespace arm_cell_vda_adapter
{
namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using Handoff = arm_cell_interfaces::msg::MaterialHandoffState;
using AcceptedTrigger = arm_cell_interfaces::msg::HandoffTrigger;

std::optional<Handoff> snapshot(MaterialDeliveryMock & mock, Clock::time_point now)
{
  auto states = mock.snapshot_all(now, builtin_interfaces::msg::Time{});
  if (states.empty()) {
    return std::nullopt;
  }
  return states.front();
}
}  // namespace

TEST(MaterialDeliveryMock, StartsFromAcceptedTriggerAndPreservesDeliveryIdentity)
{
  MaterialDeliveryMock mock(100ms);
  AcceptedTrigger trigger;
  trigger.delivery_id.uuid[0] = 0x2a;
  trigger.material_type = "RawPart";
  trigger.quantity = 1;
  trigger.valid = true;
  const auto start = Clock::time_point{};

  EXPECT_TRUE(mock.consume_accepted_trigger(trigger, start));
  auto arrived = snapshot(mock, start);
  ASSERT_TRUE(arrived);
  EXPECT_EQ(arrived->phase, Handoff::HANDOFF_ARRIVED);
  EXPECT_EQ(arrived->delivery_id, trigger.delivery_id);

  EXPECT_EQ(snapshot(mock, start + 100ms)->phase, Handoff::HANDOFF_DOCKED);
  EXPECT_EQ(snapshot(mock, start + 200ms)->phase, Handoff::HANDOFF_UNLOADED);
  auto departed = snapshot(mock, start + 300ms);
  ASSERT_TRUE(departed);
  EXPECT_EQ(departed->phase, Handoff::HANDOFF_DEPARTED);
  EXPECT_EQ(departed->delivery_id, trigger.delivery_id);
}

TEST(MaterialDeliveryMock, RejectsDuplicateAndInvalidDeliveryIdentities)
{
  MaterialDeliveryMock mock(100ms);
  AcceptedTrigger trigger;
  const auto start = Clock::time_point{};
  trigger.material_type = "RawPart";
  trigger.quantity = 1;
  trigger.valid = true;
  EXPECT_FALSE(mock.consume_accepted_trigger(trigger, start));

  trigger.delivery_id.uuid[0] = 7;
  trigger.valid = false;
  EXPECT_FALSE(mock.consume_accepted_trigger(trigger, start));
  trigger.valid = true;
  trigger.quantity = 0;
  EXPECT_FALSE(mock.consume_accepted_trigger(trigger, start));
  trigger.quantity = 1;
  trigger.material_type.clear();
  EXPECT_FALSE(mock.consume_accepted_trigger(trigger, start));
  trigger.material_type = "RawPart";
  ASSERT_TRUE(mock.consume_accepted_trigger(trigger, start));
  EXPECT_FALSE(mock.consume_accepted_trigger(trigger, start + 1s));
  (void)snapshot(mock, start + 1s);
  EXPECT_FALSE(mock.consume_accepted_trigger(trigger, start + 2s));
}

TEST(MaterialDeliveryMock, FailureAfterUnloadNeverAdvancesToDeparted)
{
  MaterialDeliveryMock mock(100ms);
  AcceptedTrigger trigger;
  trigger.delivery_id.uuid[0] = 9;
  trigger.material_type = "RawPart";
  trigger.quantity = 1;
  trigger.valid = true;
  const auto start = Clock::time_point{};
  ASSERT_TRUE(mock.consume_accepted_trigger(trigger, start));
  (void)snapshot(mock, start + 100ms);
  ASSERT_EQ(snapshot(mock, start + 200ms)->phase, Handoff::HANDOFF_UNLOADED);

  mock.fail_active();
  auto failed = snapshot(mock, start + 300ms);
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->phase, Handoff::HANDOFF_FAILED);
  EXPECT_FALSE(failed->valid);
  EXPECT_EQ(failed->delivery_id, trigger.delivery_id);
  AcceptedTrigger next_trigger;
  next_trigger.delivery_id.uuid[0] = 10;
  next_trigger.material_type = "RawPart";
  next_trigger.quantity = 1;
  next_trigger.valid = true;
  EXPECT_TRUE(mock.consume_accepted_trigger(next_trigger, start + 400ms));
  const auto retried = snapshot(mock, start + 400ms);
  ASSERT_TRUE(retried);
  EXPECT_EQ(retried->phase, Handoff::HANDOFF_ARRIVED);
  EXPECT_EQ(retried->delivery_id, next_trigger.delivery_id);
}

TEST(MaterialDeliveryMock, SuccessfulDepartureClosesEpisodeAndAllowsNextIdentity)
{
  MaterialDeliveryMock mock(100ms);
  AcceptedTrigger trigger_a;
  trigger_a.delivery_id.uuid[0] = 21;
  trigger_a.material_type = "RawPart";
  trigger_a.quantity = 1;
  trigger_a.valid = true;
  const auto start = Clock::time_point{};

  ASSERT_TRUE(mock.consume_accepted_trigger(trigger_a, start));
  EXPECT_EQ(snapshot(mock, start)->phase, Handoff::HANDOFF_ARRIVED);
  EXPECT_EQ(snapshot(mock, start + 100ms)->phase, Handoff::HANDOFF_DOCKED);
  EXPECT_EQ(snapshot(mock, start + 200ms)->phase, Handoff::HANDOFF_UNLOADED);
  const auto departed_a = snapshot(mock, start + 300ms);
  ASSERT_TRUE(departed_a);
  EXPECT_EQ(departed_a->phase, Handoff::HANDOFF_DEPARTED);
  EXPECT_FALSE(snapshot(mock, start + 400ms));

  EXPECT_FALSE(mock.consume_accepted_trigger(trigger_a, start + 400ms));

  AcceptedTrigger trigger_b;
  trigger_b.delivery_id.uuid[0] = 22;
  trigger_b.material_type = "RawPart";
  trigger_b.quantity = 1;
  trigger_b.valid = true;
  ASSERT_TRUE(mock.consume_accepted_trigger(trigger_b, start + 400ms));
  EXPECT_EQ(snapshot(mock, start + 400ms)->phase, Handoff::HANDOFF_ARRIVED);
  EXPECT_EQ(snapshot(mock, start + 500ms)->phase, Handoff::HANDOFF_DOCKED);
  EXPECT_EQ(snapshot(mock, start + 600ms)->phase, Handoff::HANDOFF_UNLOADED);
  const auto departed_b = snapshot(mock, start + 700ms);
  ASSERT_TRUE(departed_b);
  EXPECT_EQ(departed_b->phase, Handoff::HANDOFF_DEPARTED);
  EXPECT_EQ(departed_b->delivery_id, trigger_b.delivery_id);
}

TEST(MaterialDeliveryMock, AdvancesMultipleAcceptedDeliveriesIndependently)
{
  MaterialDeliveryMock mock(100ms);
  AcceptedTrigger first;
  first.delivery_id.uuid[0] = 31;
  first.material_type = "RawPart";
  first.quantity = 1;
  first.valid = true;
  auto second = first;
  second.delivery_id.uuid[0] = 32;
  const auto start = Clock::time_point{};

  ASSERT_TRUE(mock.consume_accepted_trigger(first, start));
  ASSERT_TRUE(mock.consume_accepted_trigger(second, start + 1ms));
  const builtin_interfaces::msg::Time stamp{};
  const auto arrived = mock.snapshot_all(start + 1ms, stamp);
  ASSERT_EQ(arrived.size(), 2u);
  EXPECT_EQ(arrived[0].delivery_id, first.delivery_id);
  EXPECT_EQ(arrived[1].delivery_id, second.delivery_id);
  EXPECT_EQ(arrived[0].phase, Handoff::HANDOFF_ARRIVED);
  EXPECT_EQ(arrived[1].phase, Handoff::HANDOFF_ARRIVED);

  const auto later = mock.snapshot_all(start + 101ms, stamp);
  ASSERT_EQ(later.size(), 2u);
  EXPECT_EQ(later[0].phase, Handoff::HANDOFF_DOCKED);
  EXPECT_EQ(later[1].phase, Handoff::HANDOFF_DOCKED);
  EXPECT_EQ(later[0].delivery_id, first.delivery_id);
  EXPECT_EQ(later[1].delivery_id, second.delivery_id);
}

}  // namespace arm_cell_vda_adapter
