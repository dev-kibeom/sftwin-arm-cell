#include <gtest/gtest.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#define private public
#include "arm_cell_motion/isaac_ros_motion_transport.hpp"
#undef private

TEST(TaughtEntryTiming, PreservesRequestedVelocityAndAccelerationScales)
{
  moveit::core::RobotModelBuilder builder("entry", "base_link");
  builder.addChain("base_link->sf_grasp_tcp", "prismatic");
  builder.addGroupChain("base_link", "sf_grasp_tcp", "arm");
  auto model = builder.build();
  const auto names = model->getJointModelGroup("arm")->getVariableNames();
  ASSERT_EQ(names.size(), 1U);
  auto bounds = model->getVariableBounds(names.front());
  bounds.velocity_bounded_ = true; bounds.max_velocity_ = 1.; bounds.min_velocity_ = -1.;
  bounds.acceleration_bounded_ = true; bounds.max_acceleration_ = 1.;
  bounds.min_acceleration_ = -1.;
  model->getJointModel(names.front())->setVariableBounds(names.front(), bounds);
  moveit::core::RobotState start(model); start.setToDefaultValues(); start.update();
  moveit_msgs::msg::RobotTrajectory raw;
  raw.joint_trajectory.joint_names = names;
  for (double q : {0., 1.}) {
    trajectory_msgs::msg::JointTrajectoryPoint point; point.positions = {q};
    raw.joint_trajectory.points.push_back(point);
  }
  auto full = raw, limited = raw;
  ASSERT_TRUE(
    arm_cell_motion::IsaacRosMotionTransport::retime_taught_entry(
      model, "arm", start,
      full, {1., 1.}));
  ASSERT_TRUE(
    arm_cell_motion::IsaacRosMotionTransport::retime_taught_entry(
      model, "arm", start,
      limited, {.3, .2}));
  const auto seconds = [](const auto & t) {return t.sec + t.nanosec * 1e-9;};
  EXPECT_GT(
    seconds(limited.joint_trajectory.points.back().time_from_start),
    seconds(full.joint_trajectory.points.back().time_from_start));
  for (const auto & point:limited.joint_trajectory.points) {
    EXPECT_LE(std::abs(point.velocities.front()), .3 + 1e-6);
    EXPECT_LE(std::abs(point.accelerations.front()), .2 + 1e-6);
  }
}
