#include <gtest/gtest.h>

#include "arm_cell_vda_adapter/external_state_mock.hpp"

namespace arm_cell_vda_adapter
{

TEST(ExternalStateMock, PublishesValidCanonicalStatesByDefault)
{
  ExternalStateMock mock;
  const auto states = mock.snapshot(rclcpp::Time(10, 0));

  ASSERT_TRUE(states.publish);
  EXPECT_TRUE(states.amr.valid);
  EXPECT_EQ(states.amr.docking_state, AMRDockingState::AMR_DOCKING_DOCKED);
  EXPECT_FALSE(states.amr.driving);
  EXPECT_TRUE(states.packml.valid);
  EXPECT_EQ(states.packml.state, PackMLState::PACKML_STATE_IDLE);
  EXPECT_TRUE(states.hardware.valid);
  EXPECT_FALSE(states.hardware.e_stop_active);
  EXPECT_FALSE(states.hardware.sto_active);
}

TEST(ExternalStateMock, UnknownFaultPublishesInvalidUnknownSubset)
{
  ExternalStateMock mock;
  mock.set_fault(Fault::kInvalidState, true);

  const auto states = mock.snapshot(rclcpp::Time(10, 0));

  ASSERT_TRUE(states.publish);
  EXPECT_EQ(states.amr.docking_state, AMRDockingState::AMR_DOCKING_UNKNOWN);
  EXPECT_FALSE(states.amr.valid);
  EXPECT_EQ(states.packml.state, PackMLState::PACKML_STATE_UNKNOWN);
  EXPECT_FALSE(states.packml.valid);
  EXPECT_FALSE(states.hardware.e_stop_active);
  EXPECT_FALSE(states.hardware.valid);
}

TEST(ExternalStateMock, CommunicationLossSuppressesAllPeriodicPublication)
{
  ExternalStateMock mock;
  mock.set_fault(Fault::kCommunicationLoss, true);

  const auto states = mock.snapshot(rclcpp::Time(10, 0));

  EXPECT_FALSE(states.publish);
}

TEST(ExternalStateMock, RestoringCommunicationRequestsImmediateFreshPublication)
{
  ExternalStateMock mock;
  mock.set_fault(Fault::kCommunicationLoss, true);
  ASSERT_FALSE(mock.snapshot(rclcpp::Time(10, 0)).publish);

  mock.set_fault(Fault::kCommunicationLoss, false);
  const auto states = mock.snapshot(rclcpp::Time(11, 0));

  ASSERT_TRUE(states.publish);
  EXPECT_EQ(states.amr.header.stamp.sec, 11);
  EXPECT_EQ(states.packml.header.stamp.sec, 11);
  EXPECT_EQ(states.hardware.header.stamp.sec, 11);
}

TEST(ExternalStateMock, CommunicationDegradationIsObservableWithoutSuppressingState)
{
  ExternalStateMock mock;
  mock.set_fault(Fault::kCommunicationDegradation, true);

  const auto states = mock.snapshot(rclcpp::Time(10, 0));

  EXPECT_TRUE(states.publish);
  EXPECT_TRUE(states.communication_degraded);
  EXPECT_TRUE(states.amr.valid);
  EXPECT_TRUE(states.packml.valid);
  EXPECT_TRUE(states.hardware.valid);
}

TEST(ExternalStateMock, ClearingEStopDoesNotExposeSafetyResetControl)
{
  ExternalStateMock mock;
  mock.set_fault(Fault::kEStop, true);
  EXPECT_TRUE(mock.snapshot(rclcpp::Time(10, 0)).hardware.e_stop_active);
  mock.set_fault(Fault::kEStop, false);

  const auto states = mock.snapshot(rclcpp::Time(10, 0));

  EXPECT_FALSE(states.hardware.e_stop_active);
  EXPECT_FALSE(states.hardware.sto_active);
}

TEST(ExternalStateMock, PackmlAbortUsesCanonicalSubset)
{
  ExternalStateMock mock;
  mock.set_fault(Fault::kPackmlAbort, true);

  const auto states = mock.snapshot(rclcpp::Time(10, 0));

  EXPECT_EQ(states.packml.state, PackMLState::PACKML_STATE_ABORTED);
  EXPECT_TRUE(states.packml.valid);
}

TEST(ExternalStateMock, ClearingUndockDoesNotSynthesizeRedock)
{
  ExternalStateMock mock;
  mock.set_fault(Fault::kPrematureUndock, true);
  ASSERT_EQ(
    mock.snapshot(rclcpp::Time(10, 0)).amr.docking_state,
    AMRDockingState::AMR_DOCKING_UNDOCKED);

  mock.set_fault(Fault::kPrematureUndock, false);
  const auto states = mock.snapshot(rclcpp::Time(11, 0));

  EXPECT_EQ(states.amr.docking_state, AMRDockingState::AMR_DOCKING_UNDOCKED);
  EXPECT_TRUE(states.amr.driving);
}

}  // namespace arm_cell_vda_adapter
