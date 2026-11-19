#include <gtest/gtest.h>
#include "arm_cell_motion/trajectory_motion_bt.hpp"
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

using namespace arm_cell_motion;
using Code = arm_cell_interfaces::msg::MotionTaskResultCode;

TEST(TrajectoryMotionBt, MoveJAndMoveLExecuteGeneratedTrajectoryWithoutSceneOrPlanner)
{
  for (const auto * name : {"MoveJ", "MoveL"}) {
    std::vector<double> trajectory, executed;
    TrajectoryMotionOperations ops;
    ops.resolve_joint_target = [] {return Code::TASK_RESULT_SUCCESS;};
    ops.generate_joint_trajectory = [&] {trajectory = {0., .2, .4}; return Code::TASK_RESULT_SUCCESS;};
    ops.generate_linear_trajectory = [&] {trajectory = {0., .1, .2}; return Code::TASK_RESULT_SUCCESS;};
    ops.check_trajectory_collision = [] {ADD_FAILURE() << "Unexpected Scene check"; return Code::TASK_RESULT_BACKEND_ERROR;};
    ops.plan_collision_free_path = [] {ADD_FAILURE() << "Unexpected planner"; return Code::TASK_RESULT_BACKEND_ERROR;};
    ops.execute_trajectory = [&] {executed = trajectory; return Code::TASK_RESULT_SUCCESS;};
    const auto result = run_trajectory_motion(name, ops);
    EXPECT_EQ(result.code, Code::TASK_RESULT_SUCCESS) << result.failure_stage;
    ASSERT_FALSE(executed.empty());
    EXPECT_EQ(executed, trajectory);
    EXPECT_DOUBLE_EQ(executed.back(), std::string(name) == "MoveJ" ? .4 : .2);
  }
}

TEST(TrajectoryMotionBt, CheckedMotionRejectsCollisionBeforeExecution)
{
  for (const auto * name : {"MoveJChecked", "MoveLChecked"}) {
    TrajectoryMotionOperations ops;
    ops.resolve_joint_target = ops.generate_joint_trajectory = ops.generate_linear_trajectory =
      [] {return Code::TASK_RESULT_SUCCESS;};
    ops.check_trajectory_collision = [] {return Code::TASK_RESULT_PATH_OBSTRUCTED;};
    ops.execute_trajectory = [] {ADD_FAILURE() << "Executed colliding trajectory"; return Code::TASK_RESULT_SUCCESS;};
    const auto result = run_trajectory_motion(name, ops);
    EXPECT_EQ(result.code, Code::TASK_RESULT_PATH_OBSTRUCTED);
    EXPECT_EQ(result.failure_stage, "CheckTrajectoryCollision");
  }
}

TEST(TrajectoryMotionBt, PlanningAndIkFailuresKeepTheirCauseAndDoNotExecute)
{
  TrajectoryMotionOperations ops;
  ops.resolve_joint_target = [] {return Code::TASK_RESULT_IK_UNREACHABLE;};
  ops.plan_collision_free_path = [] {return Code::TASK_RESULT_PATH_OBSTRUCTED;};
  ops.execute_trajectory = [] {ADD_FAILURE(); return Code::TASK_RESULT_SUCCESS;};
  EXPECT_EQ(run_trajectory_motion("MoveJ", ops).code, Code::TASK_RESULT_IK_UNREACHABLE);
  const auto result = run_trajectory_motion("MovePlanned", ops);
  EXPECT_EQ(result.code, Code::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(result.failure_stage, "PlanCollisionFreePath");
}

TEST(TrajectoryMotionBt, CancellationBetweenGenerationAndExecutionDiscardsTrajectory)
{
  bool canceled = false;
  TrajectoryMotionOperations ops;
  ops.interruption = [&] {return canceled ? Code::TASK_RESULT_CANCELED : Code::TASK_RESULT_SUCCESS;};
  ops.resolve_joint_target = [] {return Code::TASK_RESULT_SUCCESS;};
  ops.generate_joint_trajectory = [&] {canceled = true; return Code::TASK_RESULT_SUCCESS;};
  ops.execute_trajectory = [] {ADD_FAILURE() << "Executed after cancellation"; return Code::TASK_RESULT_SUCCESS;};
  EXPECT_EQ(run_trajectory_motion("MoveJ", ops).code, Code::TASK_RESULT_CANCELED);
}
