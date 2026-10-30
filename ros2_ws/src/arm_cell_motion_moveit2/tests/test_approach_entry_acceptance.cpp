#include <gtest/gtest.h>

#include "arm_cell_motion_moveit2/approach_entry_acceptance.hpp"

namespace arm_cell_motion_moveit2
{
namespace
{
geometry_msgs::msg::Pose pose(double x, double y, double z, double w = 1.0)
{
  geometry_msgs::msg::Pose result;
  result.position.x = x;
  result.position.y = y;
  result.position.z = z;
  result.orientation.w = w;
  return result;
}

ApproachEntryPolicy policy()
{
  return {0.005, 0.05, true};
}

TEST(ApproachEntryAcceptance, AcceptsSettledPoseInsideEntryTolerances)
{
  const auto result = evaluate_approach_entry(
    pose(1.0, 2.0, 3.0), pose(1.001, 2.0, 3.0), true, true, true, policy());
  EXPECT_TRUE(result.accepted);
  EXPECT_EQ(result.reason, "accepted");
}

TEST(ApproachEntryAcceptance, RejectsPositionOutsideEntryTolerance)
{
  const auto result = evaluate_approach_entry(
    pose(1.0, 2.0, 3.0), pose(1.01, 2.0, 3.0), true, true, true, policy());
  EXPECT_FALSE(result.accepted);
  EXPECT_NE(result.reason.find("position"), std::string::npos);
}

TEST(ApproachEntryAcceptance, RejectsOrientationOutsideEntryTolerance)
{
  auto actual = pose(1.0, 2.0, 3.0);
  actual.orientation.z = 0.2;
  actual.orientation.w = 0.9797958971;
  const auto result = evaluate_approach_entry(
    pose(1.0, 2.0, 3.0), actual, true, true, true, policy());
  EXPECT_FALSE(result.accepted);
  EXPECT_NE(result.reason.find("orientation"), std::string::npos);
}

TEST(ApproachEntryAcceptance, RejectsUnsettledPose)
{
  const auto result = evaluate_approach_entry(
    pose(1.0, 2.0, 3.0), pose(1.0, 2.0, 3.0), false, true, true, policy());
  EXPECT_FALSE(result.accepted);
  EXPECT_NE(result.reason.find("settled"), std::string::npos);
}

TEST(ApproachEntryAcceptance, RequiresExpectedAndMeasuredJointsWithinTolerance)
{
  EXPECT_TRUE(
    joint_positions_within_tolerance({0.1, -0.2}, {0.1005, -0.1995}, 0.001));
  EXPECT_FALSE(
    joint_positions_within_tolerance({0.1, -0.2}, {0.102, -0.1995}, 0.001));
  EXPECT_FALSE(joint_positions_within_tolerance({0.1}, {0.1, 0.2}, 0.001));
}

TEST(ApproachEntryAcceptance, PlaceApproachAloneReceivesRelaxedTrackingTolerance)
{
  EXPECT_DOUBLE_EQ(select_trajectory_tracking_tolerance(true, true, 0.020), 0.020);
  EXPECT_DOUBLE_EQ(select_trajectory_tracking_tolerance(true, false, 0.020), 0.01);
  EXPECT_DOUBLE_EQ(select_trajectory_tracking_tolerance(false, true, 0.020), 0.01);
  EXPECT_DOUBLE_EQ(select_trajectory_tracking_tolerance(false, false, 0.020), 0.01);
}

TEST(ApproachEntryAcceptance, PlaceApproachAloneReceivesRelaxedPositionTolerance)
{
  EXPECT_DOUBLE_EQ(
    select_approach_entry_position_tolerance(true, true, 0.010, 0.015), 0.015);
  EXPECT_DOUBLE_EQ(
    select_approach_entry_position_tolerance(true, false, 0.010, 0.015), 0.010);
  EXPECT_DOUBLE_EQ(
    select_approach_entry_position_tolerance(false, true, 0.010, 0.015), 0.010);
  EXPECT_DOUBLE_EQ(
    select_approach_entry_position_tolerance(false, false, 0.010, 0.015), 0.010);
}

TEST(ApproachEntryAcceptance, RejectsInvalidState)
{
  const auto result = evaluate_approach_entry(
    pose(1.0, 2.0, 3.0), pose(1.0, 2.0, 3.0), true, true, false, policy());
  EXPECT_FALSE(result.accepted);
  EXPECT_NE(result.reason.find("invalid"), std::string::npos);
}

TEST(ApproachFeedbackStability, ReferenceCompleteButChangingFeedbackIsNotSettled)
{
  ActualFeedbackStabilityTracker tracker(0.001, 3);
  tracker.reset();
  EXPECT_FALSE(tracker.observe({1.0, 2.0}));
  EXPECT_FALSE(tracker.observe({1.0015, 2.0}));
  EXPECT_FALSE(tracker.observe({1.0030, 2.0}));
  EXPECT_FALSE(tracker.settled());
}

TEST(ApproachFeedbackStability, StableFreshFeedbackReachesSettled)
{
  ActualFeedbackStabilityTracker tracker(0.001, 3);
  tracker.reset();
  EXPECT_FALSE(tracker.observe({1.0, 2.0}));
  EXPECT_FALSE(tracker.observe({1.0005, 2.0002}));
  EXPECT_TRUE(tracker.observe({1.0007, 2.0003}));
  EXPECT_TRUE(tracker.settled());
}

TEST(ApproachFeedbackStability, StableResidualDoesNotRequireTargetTolerance)
{
  ActualFeedbackStabilityTracker tracker(0.001, 3);
  tracker.reset();
  tracker.observe({1.0133, 2.0});
  tracker.observe({1.0137, 2.0002});
  EXPECT_TRUE(tracker.observe({1.0135, 2.0003}));
}

TEST(ApproachEntryAcceptance, StaleFeedbackIsRejected)
{
  const auto result = evaluate_approach_entry(
    pose(1.0, 2.0, 3.0), pose(1.0, 2.0, 3.0), true, false, true, policy());
  EXPECT_FALSE(result.accepted);
  EXPECT_NE(result.reason.find("stale"), std::string::npos);
}
}  // namespace
}  // namespace arm_cell_motion_moveit2
