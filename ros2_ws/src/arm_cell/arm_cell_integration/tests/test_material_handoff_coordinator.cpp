#include <chrono>
#include <optional>

#include <gtest/gtest.h>

#include "arm_cell_integration/material_handoff_coordinator.hpp"

namespace arm_cell_integration
{
namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using Handoff = arm_cell_interfaces::msg::MaterialHandoffState;
using Readiness = arm_cell_interfaces::msg::MaterialReadiness;
using DeliveryId = unique_identifier_msgs::msg::UUID;

Handoff phase(const DeliveryId & id, uint8_t value, bool valid = true)
{
  Handoff message;
  message.delivery_id = id;
  message.phase = value;
  message.valid = valid;
  return message;
}

DeliveryId delivery_id(uint8_t value)
{
  DeliveryId id;
  id.uuid[0] = value;
  return id;
}

builtin_interfaces::msg::Time stamp()
{
  builtin_interfaces::msg::Time value;
  value.sec = 1;
  return value;
}
}  // namespace

TEST(MaterialHandoffCoordinator, UnloadAloneDoesNotPublishReadiness)
{
  MaterialHandoffCoordinator coordinator(50ms, 500ms);
  const auto id = delivery_id(11);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(id, start));

  EXPECT_FALSE(coordinator.observe(phase(id, Handoff::HANDOFF_ARRIVED), start, stamp()));
  EXPECT_FALSE(coordinator.observe(phase(id, Handoff::HANDOFF_DOCKED), start + 1ms, stamp()));
  EXPECT_FALSE(coordinator.observe(phase(id, Handoff::HANDOFF_UNLOADED), start + 2ms, stamp()));
  EXPECT_FALSE(coordinator.tick(start + 20ms, stamp()));
  const auto ready = coordinator.tick(start + 60ms, stamp());
  ASSERT_TRUE(ready);
  EXPECT_TRUE(ready->material_ready);
  EXPECT_TRUE(ready->valid);
  EXPECT_EQ(ready->delivery_id, id);
}

TEST(MaterialHandoffCoordinator, InvalidSequenceAndFailedTransferNeverBecomeReady)
{
  MaterialHandoffCoordinator coordinator(10ms, 500ms);
  const auto id = delivery_id(12);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(id, start));

  const auto invalid = coordinator.observe(
    phase(id, Handoff::HANDOFF_UNLOADED), start, stamp());
  ASSERT_TRUE(invalid);
  EXPECT_FALSE(invalid->material_ready);
  EXPECT_FALSE(invalid->valid);
  EXPECT_FALSE(coordinator.tick(start + 1s, stamp()));
}

TEST(MaterialHandoffCoordinator, IgnoresOtherDeliveryAndInvalidatesStaleSource)
{
  MaterialHandoffCoordinator coordinator(10ms, 100ms);
  const auto id = delivery_id(13);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(id, start));
  EXPECT_FALSE(coordinator.observe(
    phase(delivery_id(14), Handoff::HANDOFF_ARRIVED), start, stamp()));
  EXPECT_FALSE(coordinator.observe(phase(id, Handoff::HANDOFF_ARRIVED), start, stamp()));
  EXPECT_FALSE(coordinator.observe(phase(id, Handoff::HANDOFF_DOCKED), start + 1ms, stamp()));
  EXPECT_FALSE(coordinator.observe(phase(id, Handoff::HANDOFF_UNLOADED), start + 2ms, stamp()));
  auto ready = coordinator.tick(start + 20ms, stamp());
  ASSERT_TRUE(ready);
  ASSERT_TRUE(ready->material_ready);

  auto stale = coordinator.tick(start + 200ms, stamp());
  ASSERT_TRUE(stale);
  EXPECT_FALSE(stale->material_ready);
  EXPECT_FALSE(stale->valid);
  EXPECT_EQ(stale->delivery_id, id);
}

TEST(MaterialHandoffCoordinator, RejectsDuplicateDeliveryIdentity)
{
  MaterialHandoffCoordinator coordinator(10ms, 500ms);
  const auto id = delivery_id(15);
  const auto start = Clock::time_point{};
  ASSERT_TRUE(coordinator.begin(id, start));
  EXPECT_FALSE(coordinator.begin(id, start + 1s));
}

TEST(MaterialHandoffCoordinator, RejectedPrearmedRequestRollsBackIdentityAndEpisode)
{
  MaterialHandoffCoordinator coordinator(10ms, 500ms);
  const auto rejected = delivery_id(25);
  const auto next = delivery_id(26);
  const auto start = Clock::time_point{};

  ASSERT_TRUE(coordinator.begin(rejected, start));
  coordinator.reject_unaccepted(rejected);

  EXPECT_TRUE(coordinator.can_begin(rejected));
  EXPECT_TRUE(coordinator.can_begin(next));
  EXPECT_FALSE(coordinator.observe(
    phase(rejected, Handoff::HANDOFF_ARRIVED), start + 1ms, stamp()));
  ASSERT_TRUE(coordinator.begin(next, start + 2ms));
  EXPECT_FALSE(coordinator.begin(rejected, start + 3ms));
}

TEST(MaterialHandoffCoordinator, TimedOutPrearmedRequestRollsBackIdentityAndEpisode)
{
  MaterialHandoffCoordinator coordinator(10ms, 500ms);
  const auto timed_out = delivery_id(27);
  const auto next = delivery_id(28);
  const auto start = Clock::time_point{};

  ASSERT_TRUE(coordinator.begin(timed_out, start));
  coordinator.reject_unaccepted(timed_out);

  EXPECT_TRUE(coordinator.can_begin(timed_out));
  ASSERT_TRUE(coordinator.begin(next, start + 2s));
  EXPECT_FALSE(coordinator.begin(timed_out, start + 3s));
}

TEST(MaterialHandoffCoordinator, RejectionAfterObservedAcceptanceKeepsIdentityConsumed)
{
  MaterialHandoffCoordinator coordinator(10ms, 500ms);
  const auto id = delivery_id(29);
  const auto next = delivery_id(30);
  const auto start = Clock::time_point{};

  ASSERT_TRUE(coordinator.begin(id, start));
  EXPECT_FALSE(coordinator.observe(phase(id, Handoff::HANDOFF_ARRIVED), start, stamp()));
  coordinator.reject_unaccepted(id);

  EXPECT_FALSE(coordinator.can_begin(id));
  EXPECT_FALSE(coordinator.can_begin(next));
  EXPECT_FALSE(coordinator.begin(id, start + 1ms));
  EXPECT_FALSE(coordinator.observe(phase(id, Handoff::HANDOFF_DOCKED), start + 1ms, stamp()));
  EXPECT_FALSE(coordinator.observe(phase(id, Handoff::HANDOFF_UNLOADED), start + 2ms, stamp()));
  const auto ready = coordinator.tick(start + 20ms, stamp());
  ASSERT_TRUE(ready);
  EXPECT_TRUE(ready->material_ready);
  EXPECT_FALSE(coordinator.observe(phase(id, Handoff::HANDOFF_DEPARTED), start + 30ms, stamp()));
  EXPECT_FALSE(coordinator.can_begin(id));
  EXPECT_FALSE(coordinator.begin(id, start + 31ms));
  EXPECT_TRUE(coordinator.can_begin(next));
}

TEST(MaterialHandoffCoordinator, CanCheckAdmissionWithoutCreatingEpisode)
{
  MaterialHandoffCoordinator coordinator(10ms, 500ms);
  const auto id = delivery_id(16);
  const auto start = Clock::time_point{};
  EXPECT_TRUE(coordinator.can_begin(id));
  EXPECT_FALSE(coordinator.observe(phase(id, Handoff::HANDOFF_ARRIVED), start, stamp()));
  EXPECT_TRUE(coordinator.can_begin(id));
  ASSERT_TRUE(coordinator.begin(id, start));
  EXPECT_FALSE(coordinator.can_begin(delivery_id(17)));
}

TEST(MaterialHandoffCoordinator, SuccessfulDepartureClosesEpisodeAndAllowsNextIdentity)
{
  MaterialHandoffCoordinator coordinator(10ms, 500ms);
  const auto start = Clock::time_point{};
  const auto first = delivery_id(18);
  const auto second = delivery_id(19);

  ASSERT_TRUE(coordinator.begin(first, start));
  EXPECT_FALSE(coordinator.observe(phase(first, Handoff::HANDOFF_ARRIVED), start, stamp()));
  EXPECT_FALSE(coordinator.observe(
    phase(first, Handoff::HANDOFF_DOCKED), start + 1ms, stamp()));
  EXPECT_FALSE(coordinator.observe(
    phase(first, Handoff::HANDOFF_UNLOADED), start + 2ms, stamp()));
  const auto first_readiness = coordinator.tick(start + 20ms, stamp());
  ASSERT_TRUE(first_readiness);
  EXPECT_TRUE(first_readiness->material_ready);
  EXPECT_EQ(first_readiness->delivery_id, first);
  EXPECT_FALSE(coordinator.observe(
    phase(first, Handoff::HANDOFF_DEPARTED), start + 30ms, stamp()));

  EXPECT_FALSE(coordinator.can_begin(first));
  EXPECT_FALSE(coordinator.begin(first, start + 31ms));
  EXPECT_TRUE(coordinator.can_begin(second));
  ASSERT_TRUE(coordinator.begin(second, start + 31ms));

  EXPECT_FALSE(coordinator.observe(
    phase(second, Handoff::HANDOFF_ARRIVED), start + 31ms, stamp()));
  EXPECT_FALSE(coordinator.observe(
    phase(second, Handoff::HANDOFF_DOCKED), start + 32ms, stamp()));
  EXPECT_FALSE(coordinator.observe(
    phase(second, Handoff::HANDOFF_UNLOADED), start + 33ms, stamp()));
  const auto second_readiness = coordinator.tick(start + 50ms, stamp());
  ASSERT_TRUE(second_readiness);
  EXPECT_TRUE(second_readiness->material_ready);
  EXPECT_EQ(second_readiness->delivery_id, second);
  EXPECT_FALSE(coordinator.observe(
    phase(second, Handoff::HANDOFF_DEPARTED), start + 60ms, stamp()));
  EXPECT_FALSE(coordinator.can_begin(first));
  EXPECT_TRUE(coordinator.can_begin(delivery_id(20)));
}

TEST(MaterialHandoffCoordinator, NewDeliverySupersedesUnpublishedPriorReadiness)
{
  MaterialHandoffCoordinator coordinator(100ms, 500ms);
  const auto start = Clock::time_point{};
  const auto first = delivery_id(23);
  const auto second = delivery_id(24);

  ASSERT_TRUE(coordinator.begin(first, start));
  EXPECT_FALSE(coordinator.observe(phase(first, Handoff::HANDOFF_ARRIVED), start, stamp()));
  EXPECT_FALSE(coordinator.observe(
    phase(first, Handoff::HANDOFF_DOCKED), start + 1ms, stamp()));
  EXPECT_FALSE(coordinator.observe(
    phase(first, Handoff::HANDOFF_UNLOADED), start + 2ms, stamp()));
  EXPECT_FALSE(coordinator.observe(
    phase(first, Handoff::HANDOFF_DEPARTED), start + 3ms, stamp()));

  ASSERT_TRUE(coordinator.can_begin(second));
  ASSERT_TRUE(coordinator.begin(second, start + 4ms));
  EXPECT_FALSE(coordinator.tick(start + 102ms, stamp()));

  EXPECT_FALSE(coordinator.observe(
    phase(second, Handoff::HANDOFF_ARRIVED), start + 103ms, stamp()));
  EXPECT_FALSE(coordinator.observe(
    phase(second, Handoff::HANDOFF_DOCKED), start + 104ms, stamp()));
  EXPECT_FALSE(coordinator.observe(
    phase(second, Handoff::HANDOFF_UNLOADED), start + 105ms, stamp()));
  const auto second_readiness = coordinator.tick(start + 205ms, stamp());
  ASSERT_TRUE(second_readiness);
  EXPECT_TRUE(second_readiness->material_ready);
  EXPECT_EQ(second_readiness->delivery_id, second);
}

}  // namespace arm_cell_integration
