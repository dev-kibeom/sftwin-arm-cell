#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

#include "arm_cell_motion/planning_start_state.hpp"
#include "arm_cell_motion/task_accuracy.hpp"

namespace arm_cell_motion
{

TEST(TaskAccuracy, MeasuresPositionAndShortestOrientationDifference)
{
  const Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d actual = Eigen::Isometry3d::Identity();
  actual.translation() = Eigen::Vector3d(0.003, 0.004, 0.0);
  actual.linear() = Eigen::AngleAxisd(
    1.2 * 3.14159265358979323846 / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto error = measure_tcp_pose_accuracy(target, actual);

  EXPECT_NEAR(error.position_error_m, 0.005, 1e-12);
  EXPECT_NEAR(error.orientation_error_rad, 1.2 * 3.14159265358979323846 / 180.0, 1e-12);
}

TEST(TaskAccuracy, MeasuresEachJointFinalPositionErrorAndMaximum)
{
  const auto accuracy = measure_joint_position_accuracy(
    {"joint_1", "joint_2"}, {1.0, -0.5}, {0.98, -0.56});

  ASSERT_TRUE(accuracy);
  ASSERT_EQ(accuracy->joints.size(), 2U);
  EXPECT_EQ(accuracy->joints[0].name, "joint_1");
  EXPECT_DOUBLE_EQ(accuracy->joints[0].target_position_rad, 1.0);
  EXPECT_DOUBLE_EQ(accuracy->joints[0].actual_position_rad, 0.98);
  EXPECT_NEAR(accuracy->joints[0].position_error_rad, 0.02, 1e-12);
  EXPECT_NEAR(accuracy->joints[1].position_error_rad, 0.06, 1e-12);
  EXPECT_NEAR(accuracy->maximum_position_error_rad, 0.06, 1e-12);
}

TEST(TaskAccuracy, RejectsMismatchedJointAccuracyInputs)
{
  EXPECT_FALSE(measure_joint_position_accuracy(
      {"joint_1", "joint_2"}, {1.0}, {0.9, 0.8}));
}

TEST(PlanningStartState, UsesLatestMeasuredPositionsInsteadOfPriorEndpoint)
{
  moveit::core::RobotModelBuilder builder("planning_start_test", "base_link");
  builder.addChain("base_link->link_1->link_2", "revolute");
  builder.addGroupChain("base_link", "link_2", "arm");
  ASSERT_TRUE(builder.isValid());
  const auto model = builder.build();
  ASSERT_NE(model, nullptr);
  const auto joint_names = model->getJointModelGroup("arm")->getVariableNames();
  ASSERT_EQ(joint_names.size(), 2U);

  moveit::core::RobotState planning_start(model);
  const std::vector<double> previous_planned_endpoint{0.1, 0.2};
  planning_start.setVariablePositions(joint_names, previous_planned_endpoint);
  planning_start.update();

  const std::vector<double> latest_measured_positions{-0.3, 0.45};
  apply_measured_joint_positions(planning_start, joint_names, latest_measured_positions);

  EXPECT_DOUBLE_EQ(planning_start.getVariablePosition(joint_names[0]), -0.3);
  EXPECT_DOUBLE_EQ(planning_start.getVariablePosition(joint_names[1]), 0.45);
  EXPECT_NE(
    planning_start.getVariablePosition(joint_names[0]), previous_planned_endpoint[0]);
}

}  // namespace arm_cell_motion
