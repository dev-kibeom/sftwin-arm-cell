#include <gtest/gtest.h>

#include "arm_cell_motion_moveit2/fake_motion_backend.hpp"
#include "arm_cell_motion_moveit2/motion_action_server.hpp"

namespace arm_cell_motion_moveit2
{

TEST(MotionActionServerTest, ConvertsActionGoalUuidToMotionStatusUuid)
{
  rclcpp_action::GoalUUID action_uuid{};
  action_uuid.fill(42);

  const auto message_uuid = action_goal_uuid_to_message_uuid(action_uuid);

  EXPECT_EQ(message_uuid.uuid, action_uuid);
}

TEST(MotionActionServerTest, ClientCancellationMapsToCanceledResult)
{
  MotionStatus status;
  status.execution_state = MotionStatus::MOTION_STATE_IDLE;

  EXPECT_EQ(
    action_result_code(status, true),
    MotionTaskResultCode::TASK_RESULT_CANCELED);
}

TEST(MotionActionServerTest, RejectsGoalWithoutCurrentSafetyCapability)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;

  EXPECT_EQ(
    action_goal_response(core, make_uuid(1), goal), rclcpp_action::GoalResponse::REJECT);
}

TEST(MotionActionServerTest, AcceptsGoalWithCurrentNormalSafetyCapability)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  SafetyState state;
  state.safety_state = SafetyState::SAFETY_STATE_SAFE;
  state.motion_capability.value = arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL;
  state.valid = true;
  state.required_inputs_fresh = true;
  state.motion_envelope_valid = true;
  state.max_velocity_scale = 1.0F;
  state.max_acceleration_scale = 1.0F;
  core.update_safety_state(state, MotionCore::Clock::now());

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;

  EXPECT_EQ(
    action_goal_response(core, make_uuid(2), goal),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

TEST(MotionActionServerTest, CapabilityDowngradeAfterActionAcceptanceBlocksBackend)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  auto state = SafetyState{};
  state.safety_state = SafetyState::SAFETY_STATE_SAFE;
  state.motion_capability.value = arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL;
  state.valid = true;
  state.required_inputs_fresh = true;
  state.motion_envelope_valid = true;
  state.max_velocity_scale = 1.0F;
  state.max_acceleration_scale = 1.0F;
  const auto receipt_time = MotionCore::Clock::now();
  core.update_safety_state(state, receipt_time);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_EQ(
    action_goal_response(core, make_uuid(42), goal),
    rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);

  state.motion_capability.value = arm_cell_interfaces::msg::MotionCapability::MOTION_NONE;
  core.update_safety_state(state, MotionCore::Clock::now());

  std::string diagnostic;
  EXPECT_EQ(
    core.accept_goal_with_diagnostic(make_uuid(42), goal, diagnostic),
    MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
  EXPECT_FALSE(backend->execution_active());
}

}  // namespace arm_cell_motion_moveit2
