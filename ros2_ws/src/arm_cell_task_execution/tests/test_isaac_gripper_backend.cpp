#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

#include "arm_cell_motion/isaac_motion_backend.hpp"
#include "arm_cell_motion/isaac_gripper_port.hpp"
#include "arm_cell_task_execution/task_executor.hpp"
#include "legacy_backend_motion_port.hpp"

namespace arm_cell_motion
{
using namespace arm_cell_task_execution;

namespace
{
class RecordingIsaacTransport final : public IsaacMotionTransport
{
public:
  bool available() const override {return available_value;}
  bool submit(const arm_cell_interfaces::action::ExecuteTask::Goal &) override
  {
    ++submit_calls;
    return submit_value;
  }
  bool submit(
    const arm_cell_interfaces::action::ExecuteTask::Goal &,
    const std::string & motion_goal_uuid) override
  {
    forwarded_motion_goal_uuid = motion_goal_uuid;
    ++submit_calls;
    return submit_value;
  }
  bool submit_normalized(const NormalizedMotionRequest & request) override
  {
    normalized_phase = request.phase;
    normalized_tcp_z = request.tcp_target.pose.position.z;
    return submit_value;
  }
  void record_task_accuracy(const NormalizedMotionRequest &, uint8_t) override
  {
    ++task_accuracy_record_calls;
  }
  MotionCompletionOutcome wait_for_motion_completion() override
  {
    {
      std::lock_guard<std::mutex> lock(wait_mutex);
      ++wait_for_completion_calls;
    }
    wait_condition.notify_all();
    if (block_wait) {
      std::unique_lock<std::mutex> lock(wait_mutex);
      wait_condition.wait(lock, [this]() {return release_wait;});
    }
    return wait_for_completion_outcome;
  }
  void cancel() override {}
  void safety_preempt(uint8_t) override {}
  void hold() override {}
  void stop() override {}
  bool execution_active() const override {return active_value;}
  bool inactivity_confirmed() const override {return inactivity_value;}
  bool command_normalized_gripper(double opening) override
  {
    last_opening = opening;
    return gripper_value;
  }
  bool command_gripper_close() override
  {
    ++close_command_calls;
    return gripper_value;
  }
  bool gripper_active() const override {return gripper_active_value;}
  HoldingObservation holding_observation() const override
  {
    return {HoldingState::RELEASED, HoldingObservation::Clock::now(),
      ++observation_sequence, std::nullopt};
  }
  void stop_gripper() override {++stop_gripper_calls;}

  bool available_value{true};
  bool submit_value{true};
  bool active_value{false};
  bool inactivity_value{true};
  bool gripper_value{true};
  bool gripper_active_value{false};
  double last_opening{-1.0};
  uint8_t normalized_phase{0};
  double normalized_tcp_z{0.0};
  int stop_gripper_calls{0};
  int close_command_calls{0};
  int task_accuracy_record_calls{0};
  mutable uint64_t observation_sequence{0U};
  int submit_calls{0};
  std::string forwarded_motion_goal_uuid;
  int wait_for_completion_calls{0};
  bool block_wait{false};
  bool release_wait{false};
  std::mutex wait_mutex;
  std::condition_variable wait_condition;
  MotionCompletionOutcome wait_for_completion_outcome{MotionCompletionOutcome::COMPLETED};
};
}  // namespace

TEST(IsaacMotionBackendTest, TCPAccuracyDoesNotGateGripperAndHeldConfirmationStillDoes)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  auto backend = std::make_shared<IsaacMotionBackend>(transport);
  auto gripper = std::make_shared<IsaacGripperPort>(transport);
  MotionGeometry geometry;
  geometry.configured = true;
  geometry.observation_reference = "test_reference";
  geometry.observation_to_object.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.rotation.w = 1.0;
  geometry.insertion_axis_tcp.z = 1.0;
  geometry.pick_retract_distance_m = 0.05;
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, geometry), gripper, geometry);

  arm_cell_interfaces::action::ExecuteTask::Goal goal;
  goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK;
  goal.has_target_pose = true;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.pose.orientation.w = 1.0;
  goal.orientation_symmetry_order =
    arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_UNIQUE;
  goal.vision_acquisition_id = "grasp-validation-test";
  goal.pick_approach_distance_m = 0.15;
  goal.pick_grasp_position_tolerance_m = 0.01;
  goal.pick_grasp_orientation_tolerance_rad = 0.08726646259971647;

  EXPECT_EQ(executor.execute(goal),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(transport->task_accuracy_record_calls, 2);
  EXPECT_EQ(transport->close_command_calls, 1);
}

TEST(IsaacGripperPortTest, LogicalCloseDoesNotSendAWidthTarget)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  IsaacGripperPort gripper(transport);

  EXPECT_EQ(gripper.close(), arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(transport->close_command_calls, 1);
  EXPECT_DOUBLE_EQ(transport->last_opening, -1.0);
}

TEST(IsaacGripperPortTest, MissingTransportFailsClosed)
{
  IsaacGripperPort gripper(nullptr);

  EXPECT_EQ(gripper.close(), arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
}

TEST(IsaacMotionBackendTest, AvailabilityQueryReturnsPromptlyForReadyAndUnavailableTransport)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  IsaacMotionBackend backend(transport);

  auto ready = std::async(std::launch::async, [&backend]() {return backend.available();});
  ASSERT_EQ(ready.wait_for(std::chrono::milliseconds(100)), std::future_status::ready);
  EXPECT_TRUE(ready.get());

  transport->available_value = false;
  auto unavailable = std::async(
    std::launch::async, [&backend]() {return backend.available();});
  ASSERT_EQ(
    unavailable.wait_for(std::chrono::milliseconds(100)), std::future_status::ready);
  EXPECT_FALSE(unavailable.get());
}

TEST(IsaacMotionBackendTest, ReadyGoHomeProceedsIntoBackendTaskExecution)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  IsaacMotionBackend backend(transport);
  arm_cell_interfaces::action::ExecuteTask::Goal goal;
  goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;

  EXPECT_EQ(
    backend.execute_task(goal),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(transport->submit_calls, 1);
  EXPECT_EQ(transport->wait_for_completion_calls, 1);
}

TEST(IsaacMotionBackendTest, GoHomeForwardsActionGoalIdentityToTransport)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  IsaacMotionBackend backend(transport);
  arm_cell_interfaces::action::ExecuteTask::Goal goal;
  goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;

  EXPECT_EQ(
    backend.execute_task(goal, "motion-goal-home-1"),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(transport->forwarded_motion_goal_uuid, "motion-goal-home-1");
}

TEST(IsaacMotionBackendTest, GoHomeDoesNotReportSuccessBeforeMeasuredCompletion)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  transport->wait_for_completion_outcome = MotionCompletionOutcome::TIMEOUT;
  IsaacMotionBackend backend(transport);
  arm_cell_interfaces::action::ExecuteTask::Goal goal;
  goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;

  EXPECT_EQ(
    backend.execute_task(goal),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_EQ(transport->submit_calls, 1);
  EXPECT_EQ(transport->wait_for_completion_calls, 1);
}

TEST(IsaacMotionBackendTest, GoHomeMapsCancellationAndSafetyPreemption)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  IsaacMotionBackend backend(transport);
  arm_cell_interfaces::action::ExecuteTask::Goal goal;
  goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;

  transport->wait_for_completion_outcome = MotionCompletionOutcome::CANCELED;
  EXPECT_EQ(
    backend.execute_task(goal),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED);

  transport->wait_for_completion_outcome = MotionCompletionOutcome::SAFETY_PREEMPTED;
  EXPECT_EQ(
    backend.execute_task(goal),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);

  transport->wait_for_completion_outcome = MotionCompletionOutcome::JOINT_PROGRESS_STALLED;
  EXPECT_EQ(
    backend.execute_task(goal),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED);

  transport->wait_for_completion_outcome = MotionCompletionOutcome::STALE_FEEDBACK;
  EXPECT_EQ(
    backend.execute_task(goal),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
}

TEST(IsaacMotionBackendTest, GoHomePublishFailureDoesNotWaitForCompletion)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  transport->submit_value = false;
  IsaacMotionBackend backend(transport);
  arm_cell_interfaces::action::ExecuteTask::Goal goal;
  goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;

  EXPECT_EQ(
    backend.execute_task(goal),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_EQ(transport->submit_calls, 1);
  EXPECT_EQ(transport->wait_for_completion_calls, 0);
}

TEST(IsaacMotionBackendTest, GoHomeBlocksCallerUntilTransportCompletion)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  transport->block_wait = true;
  IsaacMotionBackend backend(transport);
  arm_cell_interfaces::action::ExecuteTask::Goal goal;
  goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;

  auto result = std::async(
    std::launch::async, [&backend, &goal]() {return backend.execute_task(goal);});
  {
    std::unique_lock<std::mutex> lock(transport->wait_mutex);
    ASSERT_TRUE(
      transport->wait_condition.wait_for(
        lock, std::chrono::seconds(1), [&transport]() {
          return transport->wait_for_completion_calls == 1;
        }));
  }
  EXPECT_EQ(result.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);

  {
    std::lock_guard<std::mutex> lock(transport->wait_mutex);
    transport->release_wait = true;
  }
  transport->wait_condition.notify_all();
  ASSERT_EQ(result.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(
    result.get(), arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

TEST(IsaacMotionBackendTest, TranslatesLogicalWidthToNormalizedOpening)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  IsaacMotionBackend backend(transport);

  EXPECT_EQ(
    backend.command_gripper(42.5F),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_DOUBLE_EQ(transport->last_opening, 0.5);
}

TEST(IsaacMotionBackendTest, ReceivesMotionNormalizedRequestBelowBackendSeam)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  IsaacMotionBackend backend(transport);
  NormalizedMotionRequest request;
  request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
  request.tcp_target.pose.position.z = 1.25;

  EXPECT_EQ(
    backend.execute_normalized(request),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    transport->normalized_phase,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING);
  EXPECT_DOUBLE_EQ(transport->normalized_tcp_z, 1.25);
  EXPECT_EQ(transport->wait_for_completion_calls, 1);
}

TEST(IsaacMotionBackendTest, DoesNotReportPhaseSuccessBeforeMeasuredCompletion)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  transport->wait_for_completion_outcome = MotionCompletionOutcome::TIMEOUT;
  IsaacMotionBackend backend(transport);
  NormalizedMotionRequest request;

  EXPECT_EQ(
    backend.execute_normalized(request),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_EQ(transport->wait_for_completion_calls, 1);
}

TEST(IsaacMotionBackendTest, PreservesCallerCancellationOutcome)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  transport->wait_for_completion_outcome = MotionCompletionOutcome::CANCELED;
  IsaacMotionBackend backend(transport);
  NormalizedMotionRequest request;

  EXPECT_EQ(
    backend.execute_normalized(request),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED);
}

TEST(IsaacMotionBackendTest, PreservesSafetyPreemptionOutcome)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  transport->wait_for_completion_outcome = MotionCompletionOutcome::SAFETY_PREEMPTED;
  IsaacMotionBackend backend(transport);
  NormalizedMotionRequest request;

  EXPECT_EQ(
    backend.execute_normalized(request),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
}

TEST(IsaacMotionBackendTest, MapsStaleFeedbackAndTimeoutToBackendError)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  IsaacMotionBackend backend(transport);
  NormalizedMotionRequest request;

  transport->wait_for_completion_outcome = MotionCompletionOutcome::STALE_FEEDBACK;
  EXPECT_EQ(
    backend.execute_normalized(request),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);

  transport->wait_for_completion_outcome = MotionCompletionOutcome::TIMEOUT;
  EXPECT_EQ(
    backend.execute_normalized(request),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);

  transport->wait_for_completion_outcome = MotionCompletionOutcome::JOINT_PROGRESS_STALLED;
  EXPECT_EQ(
    backend.execute_normalized(request),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED);
}

TEST(IsaacMotionBackendTest, MapsTransportUnavailableToBackendUnavailable)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  transport->wait_for_completion_outcome = MotionCompletionOutcome::TRANSPORT_UNAVAILABLE;
  IsaacMotionBackend backend(transport);
  NormalizedMotionRequest request;

  EXPECT_EQ(
    backend.execute_normalized(request),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
}

TEST(IsaacMotionBackendTest, RejectsWidthOutsideLogicalContract)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  IsaacMotionBackend backend(transport);

  EXPECT_EQ(
    backend.command_gripper(86.0F),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_DOUBLE_EQ(transport->last_opening, -1.0);
}

TEST(IsaacMotionBackendTest, PreservesBackendAvailabilityAndGripperFailure)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  transport->available_value = false;
  transport->gripper_value = false;
  IsaacMotionBackend backend(transport);
  arm_cell_interfaces::action::ExecuteTask::Goal goal;

  EXPECT_FALSE(backend.available());
  EXPECT_EQ(
    backend.execute_task(goal),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(
    backend.command_gripper(10.0F),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
}

TEST(IsaacMotionBackendTest, UnavailableGoHomeReturnsBoundedBackendUnavailable)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  transport->available_value = false;
  IsaacMotionBackend backend(transport);
  arm_cell_interfaces::action::ExecuteTask::Goal goal;
  goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;

  auto result = std::async(
    std::launch::async, [&backend, &goal]() {return backend.execute_task(goal);});
  ASSERT_EQ(result.wait_for(std::chrono::milliseconds(100)), std::future_status::ready);
  EXPECT_EQ(
    result.get(), arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
}

TEST(IsaacMotionBackendTest, SafetyPreemptionStopsActiveGripperThroughPort)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  transport->gripper_active_value = true;
  IsaacMotionBackend backend(transport);

  backend.safety_preempt(
    arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED);

  EXPECT_EQ(transport->stop_gripper_calls, 1);
}

TEST(IsaacMotionBackendTest, StopsActiveGripperThroughLowerTransport)
{
  auto transport = std::make_shared<RecordingIsaacTransport>();
  transport->gripper_value = false;
  transport->gripper_active_value = true;
  IsaacMotionBackend backend(transport);

  EXPECT_EQ(
    backend.command_gripper(10.0F),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  backend.stop();
  EXPECT_EQ(transport->stop_gripper_calls, 1);
}

}  // namespace arm_cell_motion
