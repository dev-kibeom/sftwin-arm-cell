#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <utility>

#include <arm_cell_interfaces/msg/motion_capability.hpp>

#include "arm_cell_motion/fake_motion_backend.hpp"
#include "arm_cell_task_execution/task_executor.hpp"
#include "legacy_backend_motion_port.hpp"

namespace arm_cell_motion
{
using namespace arm_cell_task_execution;

class RefreshingGripperPort final : public GripperPort
{
public:
  explicit RefreshingGripperPort(std::shared_ptr<GripperPort> delegate)
  : delegate_(std::move(delegate)) {}

  std::uint8_t close() override {return delegate_->close();}
  std::uint8_t open() override {return delegate_->open();}
  HoldingObservation holding() const override
  {
    auto observation = delegate_->holding();
    if (observation.state != HoldingState::UNKNOWN) {
      observation.observed_at = HoldingObservation::Clock::now();
    }
    return observation;
  }
  bool active() const override {return delegate_->active();}
  void stop() override {delegate_->stop();}

private:
  std::shared_ptr<GripperPort> delegate_;
};

TEST(TaskExecutorTest, SubmitsGoHomeWithoutObjectOrGripperSideEffects)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  TaskExecutor executor(std::make_shared<test::LegacyBackendMotionPort>(backend, MotionGeometry{}),
      backend->gripper_port());
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
  TaskExecutor executor(std::make_shared<test::LegacyBackendMotionPort>(backend, MotionGeometry{}),
      backend->gripper_port());
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
  TaskExecutor executor(std::make_shared<test::LegacyBackendMotionPort>(backend, MotionGeometry{}),
      backend->gripper_port());
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
  TaskExecutor executor(std::make_shared<test::LegacyBackendMotionPort>(backend, MotionGeometry{}),
      backend->gripper_port());
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_RETRACT;

  EXPECT_EQ(
    executor.execute(goal, MotionObjectState::ATTACHED),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_TRUE(backend->execution_active());
  EXPECT_EQ(backend->hold_calls(), 0);
  EXPECT_EQ(backend->stop_calls(), 0);
}

TEST(TaskExecutorTest, ReturnsHeldObjectToTrayAndConfirmsRelease)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto gripper = backend->gripper_port();
  ASSERT_EQ(gripper->close(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, MotionGeometry{}),
    std::make_shared<RefreshingGripperPort>(gripper));
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_RETURN_HELD;

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
  EXPECT_EQ(backend->joint_target_calls(), 1U);
  EXPECT_EQ(backend->submitted_task_type().value, MotionTaskType::TASK_TYPE_UNSPECIFIED);
  EXPECT_EQ(gripper->holding().state, HoldingState::RELEASED);
}

TEST(TaskExecutorTest, DoesNotMoveToTrayWithoutFreshHeldObservation)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, MotionGeometry{}),
    backend->gripper_port());
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_RETURN_HELD;

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_EQ(backend->joint_target_calls(), 0U);
  EXPECT_EQ(backend->gripper_port()->holding().state, HoldingState::UNKNOWN);
}

TEST(TaskExecutorTest, SafetyPreemptionInterruptsTrayReleaseSettleWait)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto gripper = backend->gripper_port();
  ASSERT_EQ(gripper->close(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  auto support = std::make_shared<TaskExecutionSupport>(
    std::make_shared<test::LegacyBackendMotionPort>(backend, MotionGeometry{}),
    gripper, MotionGeometry{});
  std::size_t interruption_checks = 0U;
  support->set_interruption_checker([&interruption_checks]() {
      return ++interruption_checks == 3U ?
             MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED :
             MotionTaskResultCode::TASK_RESULT_SUCCESS;
    });
  TaskExecutor executor(support);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_RETURN_HELD;

  const auto started_at = std::chrono::steady_clock::now();
  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_LT(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at).count(), 500);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN);
  EXPECT_EQ(interruption_checks, 3U);
}

}  // namespace arm_cell_motion
