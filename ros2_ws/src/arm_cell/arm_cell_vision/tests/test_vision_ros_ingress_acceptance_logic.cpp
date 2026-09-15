#include <gtest/gtest.h>

#include "vision_ros_ingress_acceptance_logic.hpp"

namespace arm_cell_vision::acceptance
{

TEST(DeadlineStatusLatch, PreservesFirstTimeout)
{
  DeadlineStatusLatch latch;
  latch.observe(IngressStatus::kWaiting);
  latch.observe(IngressStatus::kRequestTimeout);
  latch.observe(IngressStatus::kNoActiveAcquisition);
  ASSERT_TRUE(latch.status());
  EXPECT_EQ(*latch.status(), IngressStatus::kRequestTimeout);
}

TEST(DeadlineStatusLatch, PreservesFirstTerminalRollback)
{
  DeadlineStatusLatch latch;
  latch.observe(IngressStatus::kTimeRollback);
  latch.observe(IngressStatus::kRequestTimeout);
  EXPECT_EQ(latch.status(), IngressStatus::kTimeRollback);
}

TEST(AcceptanceMode, NormalModeFailsWithoutObservation)
{
  EXPECT_FALSE(
    normal_mode_pass(
      IngressStatus::kWaiting, IngressStatus::kRequestTimeout, 0, false));
}

TEST(AcceptanceMode, TimeoutModePassesOnlyForCleanTimeout)
{
  EXPECT_TRUE(
    expect_timeout_mode_pass(
      IngressStatus::kWaiting, IngressStatus::kRequestTimeout, 0, false, false, false));
}

TEST(AcceptanceMode, TimeoutModeRejectsNonTimeoutOutcomes)
{
  EXPECT_FALSE(
    expect_timeout_mode_pass(
      IngressStatus::kWaiting, std::nullopt, 0, false, false, false));
  EXPECT_FALSE(
    expect_timeout_mode_pass(
      IngressStatus::kWaiting, IngressStatus::kNoActiveAcquisition, 0, false, false, false));
  EXPECT_FALSE(
    expect_timeout_mode_pass(
      IngressStatus::kWaiting, IngressStatus::kRequestTimeout, 1, false, false, false));
  EXPECT_FALSE(
    expect_timeout_mode_pass(
      IngressStatus::kWaiting, IngressStatus::kRequestTimeout, 0, true, false, false));
  EXPECT_FALSE(
    expect_timeout_mode_pass(
      IngressStatus::kWaiting, IngressStatus::kRequestTimeout, 0, false, true, false));
  EXPECT_FALSE(
    expect_timeout_mode_pass(
      IngressStatus::kWaiting, IngressStatus::kRequestTimeout, 0, false, false, true));
}

TEST(AcceptanceMode, NormalModeStillRequiresReadyObservation)
{
  EXPECT_TRUE(
    normal_mode_pass(
      IngressStatus::kWaiting, std::nullopt, 1, false));
}

}  // namespace arm_cell_vision::acceptance
