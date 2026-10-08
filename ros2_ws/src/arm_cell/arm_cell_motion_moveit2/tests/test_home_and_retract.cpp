#include <gtest/gtest.h>

#include <memory>

#include <arm_cell_interfaces/msg/motion_capability.hpp>

#include "arm_cell_motion_moveit2/fake_motion_backend.hpp"
#include "arm_cell_motion_moveit2/task_executor.hpp"

namespace arm_cell_motion_moveit2
{

TEST(TaskExecutorTest, SubmitsGoHomeWithoutObjectOrGripperSideEffects)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  TaskExecutor executor(backend);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(backend->submitted_task_type().value, MotionTaskType::TASK_TYPE_GO_HOME);
  EXPECT_EQ(backend->hold_calls(), 0);
  EXPECT_EQ(backend->stop_calls(), 0);
}

TEST(TaskExecutorTest, MapsBlockedAndUnreachableRecoveryToCanonicalResults)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  TaskExecutor executor(backend);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_RETRACT;

  backend->set_task_result(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);

  backend->set_task_result(MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE);
  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE);
}

TEST(TaskExecutorTest, RejectsUncertainObjectRecoveryWithoutSubmittingMotion)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  TaskExecutor executor(backend);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_RETRACT;

  EXPECT_EQ(
    executor.execute(goal, MotionObjectState::UNCERTAIN),
    MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_EQ(backend->submitted_task_type().value, MotionTaskType::TASK_TYPE_UNSPECIFIED);
}

TEST(TaskExecutorTest, AttachedRetractPreservesExecutionWithoutGripperOrStopSideEffects)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  TaskExecutor executor(backend);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_RETRACT;

  EXPECT_EQ(
    executor.execute(goal, MotionObjectState::ATTACHED),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_TRUE(backend->execution_active());
  EXPECT_EQ(backend->hold_calls(), 0);
  EXPECT_EQ(backend->stop_calls(), 0);
}

}  // namespace arm_cell_motion_moveit2
