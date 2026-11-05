#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include "../src/orchestration_diagnostic_record.hpp"

namespace arm_cell_orchestration::detail
{

TEST(OrchestrationDiagnosticSerializationTest, MapsTypedMotionReleaseFeedbackToExecutionDiagnostic)
{
  using Feedback = arm_cell_interfaces::action::ExecuteTask::Feedback;
  Feedback feedback;
  feedback.phase.value = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_GRIPPER;
  feedback.release_attempt = 2;
  feedback.release_retry_budget = 3;
  feedback.release_last_failure = Feedback::RELEASE_FAILURE_CONFIRMATION_TIMEOUT;
  feedback.diagnostic_detail = "PLACE release OPEN attempt 2/4 after confirmation timeout";

  const auto record = release_attempt_diagnostic(
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE, feedback,
    "0123456789abcdef");

  ASSERT_TRUE(record.has_value());
  EXPECT_EQ(record->event, CycleCoordinator::DiagnosticEvent::MOTION_GRIPPER_RELEASE_ATTEMPT);
  EXPECT_EQ(record->reason, CycleCoordinator::DiagnosticReason::RELEASE_CONFIRMATION_TIMEOUT);
  EXPECT_EQ(record->attempt, 2U);
  EXPECT_EQ(record->retry_budget, 3U);
  EXPECT_EQ(record->observed_motion_phase,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_GRIPPER);
  EXPECT_EQ(std::string(record->motion_execution_id.data(), record->motion_execution_id_size),
    "0123456789abcdef");
}

TEST(OrchestrationDiagnosticSerializationTest, RejectsInvalidTypedMotionReleaseFeedback)
{
  using Feedback = arm_cell_interfaces::action::ExecuteTask::Feedback;
  Feedback feedback;
  feedback.phase.value = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_GRIPPER;
  feedback.release_attempt = 5;
  feedback.release_retry_budget = 3;

  EXPECT_FALSE(release_attempt_diagnostic(
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE, feedback, "execution-id"));
  feedback.release_attempt = 1;
  feedback.release_retry_budget = 3;
  feedback.release_last_failure = Feedback::RELEASE_FAILURE_CONFIRMATION_TIMEOUT;
  EXPECT_FALSE(release_attempt_diagnostic(
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE, feedback, "execution-id"));
}

TEST(OrchestrationDiagnosticSerializationTest, RetainsTreeTransitionAndExecutionCorrelation)
{
  QueuedOrchestrationRecord queued;
  queued.execute_cycle_id[0] = 0x12;
  queued.execute_cycle_id[15] = 0x34;
  queued.delivery_id[0] = 0x56;
  queued.delivery_id[15] = 0x78;
  queued.target_id_size = 7;
  std::copy_n("RawPart", queued.target_id_size, queued.target_id.begin());
  queued.decision.event = CycleCoordinator::DiagnosticEvent::BT_NODE_STATUS;
  queued.decision.tree_node_size = 4;
  std::copy_n("PICK", queued.decision.tree_node_size, queued.decision.tree_node.begin());
  queued.decision.tree_previous_status_size = 7;
  std::copy_n(
    "RUNNING", queued.decision.tree_previous_status_size,
    queued.decision.tree_previous_status.begin());
  queued.decision.tree_current_status_size = 7;
  std::copy_n(
    "FAILURE", queued.decision.tree_current_status_size,
    queued.decision.tree_current_status.begin());

  const auto serialized = serialize_diagnostic(queued, 1U);

  EXPECT_NE(serialized.find("\"event\":\"behavior_tree_node_status\""), std::string::npos);
  EXPECT_NE(serialized.find("\"node\":\"PICK\""), std::string::npos);
  EXPECT_NE(serialized.find("\"previous_status\":\"RUNNING\""), std::string::npos);
  EXPECT_NE(serialized.find("\"current_status\":\"FAILURE\""), std::string::npos);
  EXPECT_NE(serialized.find("\"execute_cycle_id\":\"12000000000000000000000000000034\""),
    std::string::npos);
  EXPECT_NE(serialized.find("\"delivery_id\":\"56000000000000000000000000000078\""),
    std::string::npos);
  EXPECT_NE(serialized.find("\"target_id\":\"RawPart\""), std::string::npos);
}

TEST(OrchestrationDiagnosticSerializationTest, IdentifiesMissionStartWithoutClaimingOutcome)
{
  QueuedOrchestrationRecord queued;
  queued.decision.event = CycleCoordinator::DiagnosticEvent::MISSION_STARTED;
  queued.decision.requested_quantity = 3;
  queued.decision.result_available = false;

  const auto serialized = serialize_diagnostic(queued, 1U);

  EXPECT_NE(serialized.find("\"event\":\"mission_started\""), std::string::npos);
  EXPECT_NE(serialized.find("\"requested_quantity\":3"), std::string::npos);
  EXPECT_NE(serialized.find("\"result_available\":false"), std::string::npos);
  EXPECT_NE(serialized.find("\"failure_code\":null"), std::string::npos);
}

TEST(OrchestrationDiagnosticSerializationTest, RetainsUnexpectedMotionBusyResult)
{
  QueuedOrchestrationRecord queued;
  queued.decision.event = CycleCoordinator::DiagnosticEvent::MOTION_ACTION_RESULT;
  queued.decision.failure_domain = CycleCoordinator::DiagnosticFailureDomain::MOTION;
  queued.decision.failure_code =
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BUSY;
  queued.decision.reason = CycleCoordinator::DiagnosticReason::UNEXPECTED_BUSY;
  queued.decision.decision = CycleCoordinator::DiagnosticDecision::STOP;
  queued.decision.result_detail_size = 26;
  std::copy_n("reservation remains active", queued.decision.result_detail_size,
    queued.decision.result_detail.begin());
  queued.decision.motion_observation_age_ns = 5000000;
  queued.decision.motion_status_generation = 9;
  queued.decision.motion_status_stamp_ns = 123456789;
  queued.decision.motion_status_execution_id_size = 3;
  std::copy_n("abc", 3, queued.decision.motion_status_execution_id.begin());
  queued.decision.has_motion_status = true;
  queued.decision.backend_inactivity_confirmed = true;

  const auto serialized = serialize_diagnostic(queued, 1U);

  EXPECT_NE(serialized.find("\"event\":\"motion_action_result\""), std::string::npos);
  EXPECT_NE(serialized.find("\"failure_name\":\"TASK_RESULT_BUSY\""), std::string::npos);
  EXPECT_NE(serialized.find("\"motion_result_detail\":\"reservation remains active\""),
    std::string::npos);
  EXPECT_NE(serialized.find("\"reason\":\"unexpected_busy\""), std::string::npos);
  EXPECT_NE(serialized.find("\"backend_inactivity_confirmed\":true"), std::string::npos);
  EXPECT_NE(serialized.find("\"motion_observation_age_ns\":5000000"), std::string::npos);
  EXPECT_NE(serialized.find("\"motion_status_generation\":9"), std::string::npos);
  EXPECT_NE(serialized.find("\"motion_status_stamp_ns\":123456789"), std::string::npos);
  EXPECT_NE(serialized.find("\"motion_status_execution_id\":\"abc\""), std::string::npos);
}

TEST(OrchestrationDiagnosticSerializationTest, TerminalSuccessKeepsMotionStatusAsObservationOnly)
{
  QueuedOrchestrationRecord queued;
  queued.decision.event = CycleCoordinator::DiagnosticEvent::MOTION_ACTION_RESULT;
  queued.decision.failure_domain = CycleCoordinator::DiagnosticFailureDomain::MOTION;
  queued.decision.failure_code =
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
  queued.decision.result_available = true;
  queued.decision.has_motion_status = true;
  queued.decision.motion_active = true;
  queued.decision.has_active_execution = true;
  queued.decision.backend_inactivity_confirmed = false;

  const auto serialized = serialize_diagnostic(queued, 1U);

  EXPECT_NE(serialized.find("\"failure_name\":\"TASK_RESULT_SUCCESS\""), std::string::npos);
  EXPECT_NE(serialized.find("\"motion_status_observation\":\"active\""), std::string::npos);
  EXPECT_EQ(serialized.find("action_result_motion_status_mismatch"), std::string::npos);
}

TEST(OrchestrationDiagnosticSerializationTest, FeedbackGapIncludesMotionStatusCrossCheck)
{
  QueuedOrchestrationRecord queued;
  queued.decision.event = CycleCoordinator::DiagnosticEvent::MOTION_FEEDBACK_GAP;
  queued.decision.failure_domain = CycleCoordinator::DiagnosticFailureDomain::MOTION;
  queued.decision.reason = CycleCoordinator::DiagnosticReason::FEEDBACK_MISSING;
  queued.decision.observed_motion_phase =
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_PLANNING;
  queued.decision.wait_elapsed_ns = 1800000000;
  queued.decision.has_motion_status = true;
  queued.decision.motion_active = true;
  queued.decision.has_active_execution = true;
  queued.decision.motion_observation_age_ns = 250000000;

  const auto serialized = serialize_diagnostic(queued, 1U);

  EXPECT_NE(serialized.find("\"event\":\"motion_feedback_gap\""), std::string::npos);
  EXPECT_NE(serialized.find("\"reason\":\"feedback_missing\""), std::string::npos);
  EXPECT_NE(serialized.find("\"motion_phase\":\"TASK_PHASE_PLANNING\""), std::string::npos);
  EXPECT_NE(serialized.find("\"feedback_gap_elapsed_ns\":1800000000"), std::string::npos);
  EXPECT_NE(serialized.find("\"motion_status_observation\":\"active\""), std::string::npos);
  EXPECT_NE(serialized.find("\"motion_observation_age_ns\":250000000"), std::string::npos);
}

TEST(OrchestrationDiagnosticSerializationTest, CancellationKeepsBusyCauseAndMissionOutcomeDistinct)
{
  QueuedOrchestrationRecord queued;
  queued.decision.event = CycleCoordinator::DiagnosticEvent::CANCELLATION;
  queued.decision.failure_domain = CycleCoordinator::DiagnosticFailureDomain::MOTION;
  queued.decision.failure_code =
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BUSY;
  queued.decision.result_available = true;
  queued.decision.final_result =
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_CANCELED;
  queued.decision.mission_exit_reason =
    arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_CANCELED;
  queued.decision.result_detail_size = 26;
  std::copy_n("reservation remains active", queued.decision.result_detail_size,
    queued.decision.result_detail.begin());
  queued.decision.motion_action_terminal_size = 7;
  std::copy_n("ABORTED", queued.decision.motion_action_terminal_size,
    queued.decision.motion_action_terminal.begin());

  const auto serialized = serialize_diagnostic(queued, 2U);

  EXPECT_NE(serialized.find("\"failure_name\":\"TASK_RESULT_BUSY\""), std::string::npos);
  EXPECT_NE(serialized.find("\"final_result_name\":\"MISSION_EXIT_CANCELED\""),
    std::string::npos);
  EXPECT_NE(serialized.find("\"motion_result_detail\":\"reservation remains active\""),
    std::string::npos);
  EXPECT_NE(serialized.find("\"motion_action_terminal\":\"ABORTED\""), std::string::npos);
  EXPECT_EQ(serialized.find("TASK_RESULT_CANCELED"), std::string::npos);
}


}  // namespace arm_cell_orchestration::detail

TEST(OrchestrationDiagnosticRecordTest, ActionLifecycleRetainsSubmissionAndLateResultIdentity)
{
  arm_cell_orchestration::detail::QueuedOrchestrationRecord queued;
  queued.decision.event = arm_cell_orchestration::CycleCoordinator::DiagnosticEvent::ACTION_LIFECYCLE;
  const std::string submission = "cycle:2";
  const std::string goal = "actual-uuid";
  const std::string event = "late_terminal";
  std::copy(submission.begin(), submission.end(), queued.submission_id.begin());
  std::copy(goal.begin(), goal.end(), queued.action_goal_uuid.begin());
  std::copy(event.begin(), event.end(), queued.action_event.begin());
  queued.action_outcome = 12;
  queued.material_placed = 1;
  const auto result = arm_cell_orchestration::detail::serialize_diagnostic(queued, 1);
  EXPECT_NE(result.find("\"submission_id\":\"cycle:2\""), std::string::npos);
  EXPECT_NE(result.find("\"action_goal_uuid\":\"actual-uuid\""), std::string::npos);
  EXPECT_NE(result.find("\"action_event\":\"late_terminal\""), std::string::npos);
  EXPECT_NE(result.find("\"action_outcome\":12"), std::string::npos);
  EXPECT_NE(result.find("\"material_placed\":1"), std::string::npos);
}
