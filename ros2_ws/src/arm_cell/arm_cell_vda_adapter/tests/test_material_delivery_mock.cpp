#include <chrono>
#include <optional>

#include <gtest/gtest.h>
#include <arm_cell_interfaces/srv/request_material.hpp>

#include "arm_cell_vda_adapter/material_delivery_mock.hpp"

namespace arm_cell_vda_adapter
{
namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using Handoff = arm_cell_interfaces::msg::MaterialHandoffState;
using Request = arm_cell_interfaces::srv::RequestMaterial;

std::optional<Handoff> snapshot(MaterialDeliveryMock & mock, Clock::time_point now)
{
  return mock.snapshot(now, builtin_interfaces::msg::Time{});
}
}  // namespace

TEST(MaterialDeliveryMock, PublishesOrderedPhasesWithTheAcceptedDeliveryIdentity)
{
  MaterialDeliveryMock mock(100ms);
  Request::Request request;
  request.delivery_id.uuid[0] = 0x2a;
  const auto start = Clock::time_point{};

  EXPECT_TRUE(mock.request(request.delivery_id, start));
  auto arrived = snapshot(mock, start);
  ASSERT_TRUE(arrived);
  EXPECT_EQ(arrived->phase, Handoff::HANDOFF_ARRIVED);
  EXPECT_EQ(arrived->delivery_id, request.delivery_id);

  EXPECT_EQ(snapshot(mock, start + 100ms)->phase, Handoff::HANDOFF_DOCKED);
  EXPECT_EQ(snapshot(mock, start + 200ms)->phase, Handoff::HANDOFF_UNLOADED);
  auto departed = snapshot(mock, start + 300ms);
  ASSERT_TRUE(departed);
  EXPECT_EQ(departed->phase, Handoff::HANDOFF_DEPARTED);
  EXPECT_EQ(departed->delivery_id, request.delivery_id);
}

TEST(MaterialDeliveryMock, RejectsDuplicateAndInvalidDeliveryIdentities)
{
  MaterialDeliveryMock mock(100ms);
  Request::Request request;
  const auto start = Clock::time_point{};
  EXPECT_FALSE(mock.request(request.delivery_id, start));

  request.delivery_id.uuid[0] = 7;
  ASSERT_TRUE(mock.request(request.delivery_id, start));
  EXPECT_FALSE(mock.request(request.delivery_id, start + 1s));
  (void)snapshot(mock, start + 1s);
  EXPECT_FALSE(mock.request(request.delivery_id, start + 2s));
}

TEST(MaterialDeliveryMock, FailureAfterUnloadNeverAdvancesToDeparted)
{
  MaterialDeliveryMock mock(100ms);
  Request::Request request;
  request.delivery_id.uuid[0] = 9;
  const auto start = Clock::time_point{};
  ASSERT_TRUE(mock.request(request.delivery_id, start));
  (void)snapshot(mock, start + 100ms);
  ASSERT_EQ(snapshot(mock, start + 200ms)->phase, Handoff::HANDOFF_UNLOADED);

  mock.fail_active();
  auto failed = snapshot(mock, start + 300ms);
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->phase, Handoff::HANDOFF_FAILED);
  EXPECT_FALSE(failed->valid);
  EXPECT_EQ(failed->delivery_id, request.delivery_id);
  Request::Request next_request;
  next_request.delivery_id.uuid[0] = 10;
  EXPECT_TRUE(mock.request(next_request.delivery_id, start + 400ms));
  const auto retried = snapshot(mock, start + 400ms);
  ASSERT_TRUE(retried);
  EXPECT_EQ(retried->phase, Handoff::HANDOFF_ARRIVED);
  EXPECT_EQ(retried->delivery_id, next_request.delivery_id);
}

TEST(MaterialDeliveryMock, SuccessfulDepartureClosesEpisodeAndAllowsNextIdentity)
{
  MaterialDeliveryMock mock(100ms);
  Request::Request request_a;
  request_a.delivery_id.uuid[0] = 21;
  const auto start = Clock::time_point{};

  ASSERT_TRUE(mock.request(request_a.delivery_id, start));
  EXPECT_EQ(snapshot(mock, start)->phase, Handoff::HANDOFF_ARRIVED);
  EXPECT_EQ(snapshot(mock, start + 100ms)->phase, Handoff::HANDOFF_DOCKED);
  EXPECT_EQ(snapshot(mock, start + 200ms)->phase, Handoff::HANDOFF_UNLOADED);
  const auto departed_a = snapshot(mock, start + 300ms);
  ASSERT_TRUE(departed_a);
  EXPECT_EQ(departed_a->phase, Handoff::HANDOFF_DEPARTED);
  EXPECT_FALSE(snapshot(mock, start + 400ms));

  EXPECT_FALSE(mock.request(request_a.delivery_id, start + 400ms));

  Request::Request request_b;
  request_b.delivery_id.uuid[0] = 22;
  ASSERT_TRUE(mock.request(request_b.delivery_id, start + 400ms));
  EXPECT_EQ(snapshot(mock, start + 400ms)->phase, Handoff::HANDOFF_ARRIVED);
  EXPECT_EQ(snapshot(mock, start + 500ms)->phase, Handoff::HANDOFF_DOCKED);
  EXPECT_EQ(snapshot(mock, start + 600ms)->phase, Handoff::HANDOFF_UNLOADED);
  const auto departed_b = snapshot(mock, start + 700ms);
  ASSERT_TRUE(departed_b);
  EXPECT_EQ(departed_b->phase, Handoff::HANDOFF_DEPARTED);
  EXPECT_EQ(departed_b->delivery_id, request_b.delivery_id);
}

}  // namespace arm_cell_vda_adapter
