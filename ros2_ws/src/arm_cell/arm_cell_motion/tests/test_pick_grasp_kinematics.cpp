#include <gtest/gtest.h>

#include <limits>
#include <memory>

#include <moveit/utils/robot_model_test_utils.h>

#include "arm_cell_motion/pick_grasp_kinematics.hpp"

namespace arm_cell_motion
{
namespace
{
class PickGraspKinematicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    moveit::core::RobotModelBuilder builder("pick_operations", "base_link");
    builder.addChain("base_link->sf_grasp_tcp", "prismatic");
    builder.addGroupChain("base_link", "sf_grasp_tcp", "arm");
    ASSERT_TRUE(builder.isValid());
    model_ = builder.build();
    ASSERT_NE(model_, nullptr);
    group_ = model_->getJointModelGroup("arm");
    ASSERT_NE(group_, nullptr);
    state_ = std::make_unique<moveit::core::RobotState>(model_);
    state_->setToDefaultValues();
    state_->update();
  }

  moveit::core::RobotModelPtr model_;
  const moveit::core::JointModelGroup * group_{nullptr};
  std::unique_ptr<moveit::core::RobotState> state_;
};

TEST_F(PickGraspKinematicsTest, AcceptsInBoundsGraspWithMatchingFk)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.orientation.w = 1.0;
  const auto result = validate_pick_grasp_kinematics(*state_, group_, {0.0}, 1U, goal, true);

  EXPECT_EQ(result.check, PickGraspKinematicCheck::VALID);
  EXPECT_TRUE(result.reason.empty());
  EXPECT_TRUE(result.actual_orientation.isApprox(Eigen::Quaterniond::Identity()));
}

TEST_F(PickGraspKinematicsTest, DistinguishesJointBoundsAndFkFailures)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.orientation.w = 1.0;
  const auto out_of_bounds = validate_pick_grasp_kinematics(
    *state_, group_, {std::numeric_limits<double>::quiet_NaN()}, 1U, goal, true);
  EXPECT_EQ(out_of_bounds.check, PickGraspKinematicCheck::JOINT_BOUNDS);
  EXPECT_EQ(out_of_bounds.reason, "grasp_joint_bounds");

  goal.pose.position.x = 0.01;
  const auto fk_mismatch = validate_pick_grasp_kinematics(*state_, group_, {0.0}, 1U, goal, true);
  EXPECT_EQ(fk_mismatch.check, PickGraspKinematicCheck::FK_TARGET_MISMATCH);
  EXPECT_EQ(fk_mismatch.reason, "grasp_fk_target_mismatch");
}
}  // namespace
}  // namespace arm_cell_motion
