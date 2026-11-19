#include <gtest/gtest.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include "arm_cell_motion/trajectory_generation.hpp"

using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
using namespace arm_cell_motion;

TEST(TrajectoryGeneration, MissingIkSolverAndInvalidGroupNeverProduceATrajectory)
{
  moveit::core::RobotModelBuilder builder("slide", "base_link");
  builder.addChain("base_link->sf_grasp_tcp", "prismatic");
  builder.addGroupChain("base_link", "sf_grasp_tcp", "arm");
  auto model = builder.build();
  moveit::core::RobotState start(model); start.setToDefaultValues(); start.update();
  auto target = Eigen::Isometry3d::Identity(); target.translation().x() = .1;
  std::vector<double> joints;
  EXPECT_EQ(resolve_pose_joint_target(start, "arm", "sf_grasp_tcp", target, joints),
    Code::TASK_RESULT_IK_UNREACHABLE);
  EXPECT_TRUE(joints.empty());
  EXPECT_EQ(resolve_pose_joint_target(start, "missing", "sf_grasp_tcp", target, joints),
    Code::TASK_RESULT_INVALID_GOAL);
  moveit_msgs::msg::RobotTrajectory trajectory;
  LinearJointChangeDiagnostics diagnostics;
  diagnostics.available = true;
  EXPECT_EQ(generate_linear_joint_trajectory(start, "arm", "sf_grasp_tcp", target,
    trajectory, [] {return Code::TASK_RESULT_CANCELED;}, &diagnostics), Code::TASK_RESULT_CANCELED);
  EXPECT_TRUE(trajectory.joint_trajectory.points.empty());
  EXPECT_FALSE(diagnostics.available);
}

TEST(NearbyJoint, KeepsTheSamePoseWithoutUnnecessaryFullTurnsOrLimitClamping) {
  moveit::core::VariableBounds bounds;
  bounds.position_bounded_=true; bounds.min_position_=-6.283185307179586;bounds.max_position_=6.283185307179586;
  EXPECT_NEAR(arm_cell_motion::nearest_revolute_equivalent(6.2682,0.,bounds),-.014985307179586,1e-12);
  bounds.min_position_=0.;
  EXPECT_DOUBLE_EQ(arm_cell_motion::nearest_revolute_equivalent(6.2682,0.,bounds),6.2682);
}
