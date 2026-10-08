#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "arm_cell_motion_moveit2/executable_trajectory_finalize.hpp"

namespace arm_cell_motion_moveit2
{
namespace
{
moveit_msgs::msg::RobotTrajectory raw_plan()
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  trajectory.joint_trajectory.joint_names = {"joint_a", "joint_b"};
  trajectory.joint_trajectory.points.resize(2);
  trajectory.joint_trajectory.points[0].positions = {0.0, 0.0};
  trajectory.joint_trajectory.points[1].positions = {0.2, -0.1};
  return trajectory;
}

bool add_valid_timing(moveit_msgs::msg::RobotTrajectory & trajectory)
{
  auto & points = trajectory.joint_trajectory.points;
  points[0].velocities = {0.0, 0.0};
  points[0].accelerations = {0.0, 0.0};
  points[0].time_from_start.sec = 0;
  points[0].time_from_start.nanosec = 0;
  points[1].velocities = {0.1, -0.1};
  points[1].accelerations = {0.0, 0.0};
  points[1].time_from_start.sec = 1;
  points[1].time_from_start.nanosec = 0;
  return true;
}
}  // namespace

TEST(ExecutableTrajectoryFinalize, PickAndPlaceRawPlansUseOneValidatedFinalizeBoundary)
{
  for (const std::string task : {"PICK", "PLACE"}) {
    auto trajectory = raw_plan();
    const auto finalized = finalize_executable_trajectory(
      trajectory, {"joint_a", "joint_b"}, {1.0, 1.0}, 1.0, add_valid_timing);

    EXPECT_TRUE(finalized.success) << task << ": " << finalized.reason;
    EXPECT_DOUBLE_EQ(finalized.duration_s, 1.0) << task;
    ASSERT_EQ(trajectory.joint_trajectory.points.size(), 2U) << task;
    for (const auto & point : trajectory.joint_trajectory.points) {
      EXPECT_EQ(point.velocities.size(), 2U) << task;
      EXPECT_EQ(point.accelerations.size(), 2U) << task;
    }
  }
}

TEST(ExecutableTrajectoryFinalize, RejectsRawPlanWhenTimedFieldsAreIncomplete)
{
  auto trajectory = raw_plan();
  const auto finalized = finalize_executable_trajectory(
    trajectory, {"joint_a", "joint_b"}, {1.0, 1.0}, 1.0,
    [](moveit_msgs::msg::RobotTrajectory & message) {
      message.joint_trajectory.points.back().time_from_start.sec = 1;
      return true;
    });

  EXPECT_FALSE(finalized.success);
  EXPECT_EQ(finalized.reason, "trajectory point fields are incomplete");
}

}  // namespace arm_cell_motion_moveit2
