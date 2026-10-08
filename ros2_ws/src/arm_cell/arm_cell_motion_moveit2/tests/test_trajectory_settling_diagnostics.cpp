#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "arm_cell_motion_moveit2/trajectory_settling_diagnostics.hpp"

namespace arm_cell_motion_moveit2
{
namespace
{
TEST(TrajectorySettlingDiagnostics, FormatsPerJointSnapshotAndMaxErrorJoint)
{
  const auto snapshot = format_trajectory_settling_snapshot(
    1.75,
    {1.0, 2.0},
    {0.9, 1.8},
    {"joint_1", "joint_2"});

  EXPECT_NE(
    snapshot.find("simulation_time_since_trajectory_end=1.750000000"), std::string::npos);
  EXPECT_NE(snapshot.find("commanded=[1.000000000,2.000000000]"), std::string::npos);
  EXPECT_NE(snapshot.find("actual=[0.900000000,1.800000000]"), std::string::npos);
  EXPECT_NE(snapshot.find("absolute_error=[0.100000000,0.200000000]"), std::string::npos);
  EXPECT_NE(snapshot.find("max_error_joint_index=1"), std::string::npos);
  EXPECT_NE(snapshot.find("max_error_joint_name=joint_2"), std::string::npos);
  EXPECT_NE(snapshot.find("max_tracking_error=0.200000000"), std::string::npos);
}

TEST(TrajectorySettlingDiagnostics, RateLimitUsesSimulationTime)
{
  EXPECT_TRUE(should_log_settling_snapshot(0.0, 0.25, 0.25));
  EXPECT_FALSE(should_log_settling_snapshot(0.25, 0.40, 0.25));
  EXPECT_TRUE(should_log_settling_snapshot(0.25, 0.50, 0.25));
}
}  // namespace
}  // namespace arm_cell_motion_moveit2
