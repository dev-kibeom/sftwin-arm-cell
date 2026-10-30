#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include <arm_cell_interfaces/msg/motion_capability.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>
#include <arm_cell_interfaces/msg/stop_mode.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_cell_motion/fake_motion_backend.hpp"
#include "arm_cell_motion/motion_core.hpp"
#include "arm_cell_motion/motion_port.hpp"

namespace arm_cell_motion
{
namespace
{
void grant_normal(MotionCore & core)
{
  const auto now = MotionCore::Clock::now();
  arm_cell_interfaces::msg::SafetyState safety;
  safety.valid = true;
  safety.safety_state = SafetyState::SAFETY_STATE_SAFE;
  safety.motion_capability.value =
    arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL;
  safety.required_inputs_fresh = true;
  safety.motion_envelope_valid = true;
  safety.max_velocity_scale = 1.0F;
  safety.max_acceleration_scale = 1.0F;
  core.update_safety_state(safety, now);
}

void grant_recovery_only(MotionCore & core)
{
  const auto now = MotionCore::Clock::now();
  arm_cell_interfaces::msg::SafetyState safety;
  safety.valid = true;
  safety.safety_state = SafetyState::SAFETY_STATE_SAFE;
  safety.motion_capability.value =
    arm_cell_interfaces::msg::MotionCapability::MOTION_RECOVERY_ONLY;
  safety.required_inputs_fresh = true;
  safety.motion_envelope_valid = true;
  safety.max_velocity_scale = 1.0F;
  safety.max_acceleration_scale = 1.0F;
  core.update_safety_state(safety, now);
}

MotionPortRequest pick_approach_request(std::string movement_id)
{
  MotionPortRequest request;
  request.execution_id = make_uuid(7);
  request.movement_id = std::move(movement_id);
  request.segment = MotionSegmentKind::PICK_APPROACH;
  request.target_pose.header.frame_id = "base_link";
  request.target_pose.pose.orientation.w = 1.0;
  request.constraints.orientation_symmetry_order = MotionOrientationSymmetry::FOURFOLD;
  return request;
}

MotionPortRequest retract_request(MotionSegmentKind segment, std::string movement_id)
{
  auto request = pick_approach_request(std::move(movement_id));
  request.segment = segment;
  request.constraints.orientation_symmetry_order = MotionOrientationSymmetry::UNIQUE;
  return request;
}

TEST(MotionPortTest, ExecutesOneMovementThroughTheSharedMotionCore)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_complete_execution_on_return(true);
  sensor_msgs::msg::JointState measured;
  measured.name = {"joint_1"};
  measured.position = {0.25};
  backend->set_measured_joint_state(measured);
  MotionCore core(backend);
  MotionPort & port = core;
  grant_normal(core);
  const auto request = pick_approach_request("pick-approach-1");

  const auto result = port.planAndExecuteMotion(request);

  EXPECT_EQ(result.execution_id.uuid, request.execution_id.uuid);
  EXPECT_EQ(result.movement_id, request.movement_id);
  EXPECT_EQ(
    result.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_TRUE(result.motion_execution_completed);
  EXPECT_EQ(backend->planning_calls(), 1);
  EXPECT_EQ(backend->planned_execution_calls(), 1);
  EXPECT_EQ(backend->execute_task_calls(), 1);
  EXPECT_EQ(
    core.status().execution_state,
    arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_IDLE);
  EXPECT_EQ(backend->gripper_port(), core.gripper_port());
  const auto observation = port.observeMotionState();
  ASSERT_TRUE(observation.measured_joint_state_available);
  ASSERT_EQ(observation.measured_joint_state.name.size(), 1U);
  EXPECT_EQ(observation.measured_joint_state.name.front(), "joint_1");
  EXPECT_DOUBLE_EQ(observation.measured_joint_state.position.front(), 0.25);
}

TEST(MotionPortTest, PlanningRetainsOperationOwnershipUntilStoredTrajectoryExecutes)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_complete_execution_on_return(true);
  MotionCore core(backend);
  MotionPort & port = core;
  grant_normal(core);
  const auto request = pick_approach_request("split-plan-execute");

  const auto planning = port.planMotion(request);

  ASSERT_EQ(
    planning.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_FALSE(planning.plan.trajectory.points.empty());
  EXPECT_EQ(backend->planning_calls(), 1);
  EXPECT_EQ(backend->planned_execution_calls(), 0);
  EXPECT_TRUE(core.status().has_active_execution);

  const auto competing = port.planAndExecuteMotion(
    pick_approach_request("competing-operation"));
  EXPECT_EQ(
    competing.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BUSY);

  auto altered_plan = planning.plan;
  altered_plan.trajectory.points.front().positions.front() += 0.1;
  const auto rejected = port.executePlannedMotion(altered_plan);
  EXPECT_EQ(
    rejected.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_EQ(backend->planned_execution_calls(), 0);

  const auto execution = port.executePlannedMotion(planning.plan);
  EXPECT_EQ(execution.execution_id.uuid, request.execution_id.uuid);
  EXPECT_EQ(execution.movement_id, request.movement_id);
  EXPECT_EQ(
    execution.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_TRUE(execution.motion_execution_completed);
  EXPECT_EQ(backend->planning_calls(), 1);
  EXPECT_EQ(backend->planned_execution_calls(), 1);
  EXPECT_EQ(
    core.status().execution_state,
    arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_IDLE);
}

TEST(MotionPortTest, PlaceWaypointGenerationIsExplicitAndDoesNotReserveOrExecuteMotion)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionGeometry geometry;
  geometry.place_waypoint_configured = true;
  geometry.place_waypoint_target_pose.header.frame_id = "base_link";
  geometry.place_waypoint_target_pose.pose.orientation.w = 1.0;
  geometry.place_waypoint_approach_distance_m = 0.015;
  geometry.place_waypoint_approach_direction_object.z = 1.0;
  MotionCore core(backend, std::chrono::milliseconds(500), geometry);
  std::string detail;

  EXPECT_TRUE(core.generatePlaceWaypointApproach("/tmp/place-waypoint.json", detail));
  EXPECT_EQ(backend->place_waypoint_generation_calls(), 1);
  EXPECT_EQ(backend->planning_calls(), 0);
  EXPECT_EQ(backend->execute_task_calls(), 0);
  EXPECT_EQ(backend->generated_path(), "/tmp/place-waypoint.json");
  EXPECT_TRUE(backend->generated_request().place_waypoint_approach);
  EXPECT_FALSE(core.status().has_active_execution);
}

TEST(MotionPortTest, ConsecutivePlansCanChainStartAndEndStateUnderOneOperation)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_complete_execution_on_return(true);
  MotionCore core(backend);
  MotionPort & port = core;
  grant_normal(core);
  auto first_request = pick_approach_request("continuous-plan");
  first_request.has_start_joint_state = true;
  first_request.start_joint_state.name = {"joint_1"};
  first_request.start_joint_state.position = {0.42};

  const auto first = port.planMotion(first_request);
  ASSERT_EQ(
    first.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(first.plan.start_state.position.size(), 1U);
  EXPECT_DOUBLE_EQ(first.plan.start_state.position.front(), 0.42);
  ASSERT_FALSE(first.plan.trajectory.points.empty());
  EXPECT_DOUBLE_EQ(
    first.plan.trajectory.points.front().positions.front(),
    first.plan.start_state.position.front());

  auto second_request = first_request;
  second_request.segment = MotionSegmentKind::PICK_DESCENT;
  second_request.target_pose.pose.position.z = -0.05;
  second_request.start_joint_state = first.plan.end_state;
  const auto second = port.planMotion(second_request);
  ASSERT_EQ(
    second.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(second.plan.start_state.position.size(), 1U);
  EXPECT_DOUBLE_EQ(second.plan.start_state.position.front(), first.plan.end_state.position.front());
  EXPECT_DOUBLE_EQ(
    second.plan.trajectory.points.front().positions.front(),
    second.plan.start_state.position.front());
  EXPECT_TRUE(core.status().has_active_execution);

  const auto premature_second = port.executePlannedMotion(second.plan);
  EXPECT_EQ(
    premature_second.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BUSY);
  EXPECT_EQ(backend->planned_execution_calls(), 0);

  const auto first_execution = port.executePlannedMotion(first.plan);
  EXPECT_EQ(
    first_execution.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_TRUE(first_execution.motion_execution_completed);
  EXPECT_TRUE(core.status().has_active_execution);
  EXPECT_EQ(
    port.executePlannedMotion(second.plan).result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_FALSE(core.status().has_active_execution);
  EXPECT_EQ(backend->planning_calls(), 2);
  EXPECT_EQ(backend->planned_execution_calls(), 2);
}

TEST(MotionPortTest, PlanningFailureReleasesReservationForImmediateNextOperation)
{
  for (const auto failed_segment : {
        MotionSegmentKind::PICK_APPROACH, MotionSegmentKind::PLACE_APPROACH,
        MotionSegmentKind::PLACE_CLEARANCE})
  {
    auto backend = std::make_shared<FakeMotionBackend>();
    backend->set_complete_execution_on_return(true);
    backend->set_leave_inactivity_unconfirmed_after_completion(true);
    MotionCore core(backend);
    MotionPort & port = core;
    grant_normal(core);

    const auto completed_request = pick_approach_request("completed-pick-approach");
    const auto completed_plan = port.planMotion(completed_request);
    ASSERT_EQ(
      completed_plan.result_code,
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
    ASSERT_EQ(
      port.executePlannedMotion(completed_plan.plan).result_code,
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
    ASSERT_FALSE(core.status().execution_active);
    ASSERT_FALSE(core.status().backend_inactivity_confirmed);

    auto failed_request = pick_approach_request("failed-approach");
    failed_request.execution_id = make_uuid(8);
    failed_request.segment = failed_segment;
    backend->set_task_result(
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
    const auto failed_plan = port.planMotion(failed_request);

    ASSERT_EQ(
      failed_plan.result_code,
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
    EXPECT_FALSE(failed_plan.motion_status.execution_active);
    EXPECT_FALSE(failed_plan.motion_status.has_active_execution);
    EXPECT_EQ(
      failed_plan.motion_status.execution_state,
      arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_IDLE);

    backend->set_task_result(
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
    auto next_request = pick_approach_request("immediate-follow-up");
    next_request.execution_id = make_uuid(9);
    next_request.segment = MotionSegmentKind::PLACE_APPROACH;
    const auto next_plan = port.planMotion(next_request);

    EXPECT_EQ(
      next_plan.result_code,
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  }
}

TEST(MotionPortTest, CancelingStoredPlanReleasesItsOperationWithoutBackendExecution)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  MotionPort & port = core;
  grant_normal(core);
  const auto request = pick_approach_request("cancel-planned");
  const auto planning = port.planMotion(request);
  ASSERT_EQ(
    planning.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);

  const auto canceled = port.cancelMotion(request.execution_id, request.movement_id);

  EXPECT_TRUE(canceled.accepted);
  EXPECT_TRUE(canceled.backend_inactivity_confirmed);
  EXPECT_FALSE(canceled.motion_status.has_active_execution);
  EXPECT_EQ(backend->planned_execution_calls(), 0);
  EXPECT_EQ(
    core.planMotion(pick_approach_request("next-operation")).result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

TEST(MotionPortTest, CancelDuringPlannedExecutionHasOneTerminalOwner)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_block_execution(true);
  backend->set_defer_terminal_return(true);
  MotionCore core(backend);
  MotionPort & port = core;
  grant_normal(core);
  const auto planning = port.planMotion(pick_approach_request("cancel-running-plan"));
  ASSERT_EQ(
    planning.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);

  MotionPortResult execution_result;
  auto execution = std::async(
    std::launch::async, [&]() {
      execution_result = port.executePlannedMotion(planning.plan);
    });
  ASSERT_TRUE(backend->wait_until_execution_started());
  const auto canceled = port.cancelMotion(planning.plan.execution_id, planning.plan.movement_id);

  EXPECT_TRUE(canceled.accepted);
  EXPECT_TRUE(core.status().has_active_execution);
  EXPECT_EQ(backend->diagnostic_finalize_calls(), 0);
  ASSERT_TRUE(backend->wait_until_interruption_ready());
  backend->release_terminal_return();
  ASSERT_EQ(execution.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  execution.get();

  EXPECT_EQ(
    execution_result.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED);
  EXPECT_EQ(backend->diagnostic_finalize_calls(), 1);
  EXPECT_TRUE(core.status().has_active_execution);
  backend->set_execution_active(false);
  backend->set_inactivity_confirmed(true);
  core.refresh_stop_state();
  EXPECT_FALSE(core.status().has_active_execution);
}

TEST(MotionPortTest, CancelDuringChainedPlanningClearsTheRetainedPlan)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  MotionPort & port = core;
  grant_normal(core);
  auto first_request = pick_approach_request("cancel-chained-plan");
  const auto first = port.planMotion(first_request);
  ASSERT_EQ(
    first.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);

  backend->set_block_planning(true);
  auto second_request = first_request;
  second_request.segment = MotionSegmentKind::PICK_DESCENT;
  second_request.has_start_joint_state = true;
  second_request.start_joint_state = first.plan.end_state;
  MotionPlanningResult second_result;
  auto planning = std::async(
    std::launch::async, [&]() {
      second_result = port.planMotion(second_request);
    });
  ASSERT_TRUE(backend->wait_until_planning_started());
  const auto canceled = port.cancelMotion(first_request.execution_id, first_request.movement_id);
  EXPECT_TRUE(canceled.accepted);
  ASSERT_EQ(planning.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  planning.get();

  EXPECT_EQ(
    second_result.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED);
  EXPECT_EQ(backend->diagnostic_finalize_calls(), 1);
  EXPECT_FALSE(core.status().has_active_execution);
  EXPECT_EQ(
    port.executePlannedMotion(first.plan).result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  backend->set_block_planning(false);
  EXPECT_EQ(
    core.planMotion(pick_approach_request("after-chained-cancel")).result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

TEST(MotionPortTest, SafetyDuringPlannedExecutionKeepsStopUntilBackendEvidence)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_block_execution(true);
  backend->set_defer_terminal_return(true);
  MotionCore core(backend);
  MotionPort & port = core;
  grant_normal(core);
  const auto planning = port.planMotion(pick_approach_request("safety-running-plan"));
  ASSERT_EQ(
    planning.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);

  MotionPortResult execution_result;
  auto execution = std::async(
    std::launch::async, [&]() {
      execution_result = port.executePlannedMotion(planning.plan);
    });
  ASSERT_TRUE(backend->wait_until_execution_started());
  ASSERT_TRUE(
    core.request_stop(
      planning.plan.execution_id,
      arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED));

  EXPECT_EQ(
    core.status().execution_state,
    arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPING);
  EXPECT_TRUE(core.status().has_active_execution);
  EXPECT_FALSE(core.status().backend_inactivity_confirmed);
  EXPECT_EQ(backend->diagnostic_finalize_calls(), 0);
  ASSERT_TRUE(backend->wait_until_interruption_ready());
  backend->release_terminal_return();
  ASSERT_EQ(execution.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  execution.get();

  EXPECT_EQ(
    execution_result.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(backend->diagnostic_finalize_calls(), 1);
  backend->set_execution_active(false);
  backend->set_inactivity_confirmed(true);
  core.refresh_stop_state();
  EXPECT_EQ(
    core.status().execution_state,
    arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPED);
  EXPECT_FALSE(core.status().has_active_execution);
}

TEST(MotionPortTest, UnimplementedDisturbOperationFailsWithoutExecutingMotion)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  MotionDisturbRequest request;
  request.execution_id = make_uuid(19);
  request.acquisition_id = "acq-19";
  request.scene_generation = 23;
  arm_cell_interfaces::msg::DisturbanceCandidate candidate;
  candidate.candidate_id = "unresolved-19";
  request.disturbance_candidates.push_back(candidate);

  const auto result = core.executeDisturb(request);

  EXPECT_EQ(result.execution_id.uuid, request.execution_id.uuid);
  EXPECT_EQ(result.movement_id, "disturb");
  EXPECT_EQ(result.acquisition_id, "acq-19");
  EXPECT_EQ(
    result.outcome.value,
    arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_FAILED);
  EXPECT_EQ(
    result.failure_reason.value,
    arm_cell_interfaces::msg::DisturbanceFailureReason::FAILURE_REASON_NOT_IMPLEMENTED);
  EXPECT_FALSE(result.backend_inactivity_confirmed);
  EXPECT_EQ(backend->execute_task_calls(), 0);
}

TEST(MotionPortTest, RecoveryOnlyDoesNotAuthorizeNormalPickOrPlaceRetreat)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_recovery_only(core);

  const auto pick = core.planAndExecuteMotion(
    retract_request(MotionSegmentKind::PICK_RETRACT, "pick-retreat"));
  const auto place = core.planAndExecuteMotion(
    retract_request(MotionSegmentKind::PLACE_RETRACT, "place-retreat"));

  EXPECT_EQ(
    pick.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
  EXPECT_EQ(
    place.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
  EXPECT_EQ(backend->execute_task_calls(), 0);
}

TEST(MotionPortTest, NormalCapabilityAuthorizesPickAndPlaceRetreatMovements)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_complete_execution_on_return(true);
  MotionCore core(backend);
  grant_normal(core);

  const auto pick = core.planAndExecuteMotion(
    retract_request(MotionSegmentKind::PICK_RETRACT, "pick-retreat-normal"));
  const auto place = core.planAndExecuteMotion(
    retract_request(MotionSegmentKind::PLACE_RETRACT, "place-retreat-normal"));

  EXPECT_EQ(
    pick.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    place.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(backend->execute_task_calls(), 2);
}

TEST(MotionPortTest, ControlledLegacyRetractKeepsRecoveryPermissionBoundary)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_recovery_only(core);

  const auto controlled_retract = core.executeLegacyMovement(
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_RETRACT,
    "00000000000000000000000000000011", "controlled-retract");
  auto place_backend = std::make_shared<FakeMotionBackend>();
  MotionCore place_core(place_backend);
  grant_recovery_only(place_core);
  const auto place_recovery = place_core.executePlaceRecoveryJointTarget(
    make_uuid(12), "place-recovery-tray");

  EXPECT_EQ(
    controlled_retract,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    place_recovery,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
}

TEST(MotionPortTest, PreservesPlanningFailureAsPlanningFailure)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_task_result(
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  MotionCore core(backend);
  grant_normal(core);

  const auto result = core.planAndExecuteMotion(pick_approach_request("blocked-approach"));

  EXPECT_EQ(result.failure_kind, MotionFailureKind::PLANNING);
  EXPECT_EQ(
    result.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(result.failure_stage, "motion_port:planning");
}

TEST(MotionPortTest, PlanningPreservesSearchAndSystemFailuresEvenWhenDiagnosticsThrow)
{
  using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
  class ThrowingDiagnosticBackend : public MotionBackend
  {
  public:
    bool available() const override {return true;}
    bool submit(const arm_cell_interfaces::action::ExecuteTask::Goal &) override {return false;}
    bool supports_split_planning() const override {return true;}
    uint8_t plan_normalized(const NormalizedMotionRequest &, NormalizedTrajectoryPlan &) override
    {return result;}
    uint8_t execute_planned_normalized(const NormalizedTrajectoryPlan &) override
    {++executions; return result;}
    void cancel() override {}
    void hold() override {}
    void stop() override {}
    bool execution_active() const override {return false;}
    bool backend_inactivity_confirmed() const override {return true;}
    void finalize_motion_planning_diagnostic(uint8_t) override
    {throw std::runtime_error("diagnostic write failed");}
    uint8_t result{0U};
    int executions{0};
  };
  for (const auto code : {Code::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED,
      Code::TASK_RESULT_IK_UNREACHABLE, Code::TASK_RESULT_PATH_OBSTRUCTED,
      Code::TASK_RESULT_BACKEND_ERROR, Code::TASK_RESULT_BACKEND_UNAVAILABLE,
      Code::TASK_RESULT_CANCELED, Code::TASK_RESULT_SAFETY_PREEMPTED})
  {
    SCOPED_TRACE(static_cast<int>(code));
    auto backend = std::make_shared<ThrowingDiagnosticBackend>();
    backend->result = code;
    MotionCore core(backend);
    grant_normal(core);
    const auto result = core.planMotion(pick_approach_request("pick-failed-plan"));
    EXPECT_EQ(result.result_code, code);
    const bool planning = code == Code::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED ||
      code == Code::TASK_RESULT_IK_UNREACHABLE || code == Code::TASK_RESULT_PATH_OBSTRUCTED;
    EXPECT_EQ(result.failure_kind == MotionFailureKind::PLANNING, planning);
    EXPECT_EQ(backend->executions, 0);
  }
}

TEST(MotionPortTest, TrajectoryCompletionReleasesReservationWithoutInactivity)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_complete_execution_on_return(true);
  backend->set_leave_inactivity_unconfirmed_after_completion(true);
  MotionCore core(backend);
  grant_normal(core);

  const auto result = core.planAndExecuteMotion(pick_approach_request("measured-completion"));

  EXPECT_EQ(
    result.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_TRUE(result.motion_execution_completed);
  EXPECT_FALSE(result.motion_status.execution_active);
  EXPECT_FALSE(result.motion_status.backend_inactivity_confirmed);
  EXPECT_FALSE(result.motion_status.has_active_execution);
  auto next = retract_request(MotionSegmentKind::PLACE_APPROACH, "place-after-pick-retract");
  const auto following = core.planAndExecuteMotion(next);
  EXPECT_EQ(
    following.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);

}

TEST(MotionPortTest, PreservesBackendFailureAsExecutionFailure)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_task_result(
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  MotionCore core(backend);
  grant_normal(core);

  const auto result = core.planAndExecuteMotion(pick_approach_request("failed-approach"));

  EXPECT_EQ(result.failure_kind, MotionFailureKind::EXECUTION);
  EXPECT_EQ(
    result.result_code,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_EQ(result.failure_stage, "motion_port:execution");
}

TEST(MotionPortTest, CancelTargetsTheMatchingMovementAndUsesBackendCancel)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_block_execution(true);
  MotionCore core(backend);
  grant_normal(core);
  const auto request = pick_approach_request("cancel-me");
  MotionPortResult result;
  auto execution = std::async(
    std::launch::async, [&]() {
      result = core.planAndExecuteMotion(request);
    });

  ASSERT_TRUE(backend->wait_until_execution_started());
  EXPECT_FALSE(core.cancelMotion(request.execution_id, "another-movement").accepted);
  EXPECT_EQ(backend->cancel_calls(), 0);
  const auto canceled = core.cancelMotion(request.execution_id, request.movement_id);

  EXPECT_TRUE(canceled.accepted);
  EXPECT_EQ(backend->cancel_calls(), 1);
  ASSERT_EQ(execution.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  execution.get();
  EXPECT_EQ(result.failure_kind, MotionFailureKind::CANCELED);
}

TEST(MotionPortTest, SafetyStopStillPreemptsThroughTheCoreStopBoundary)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_block_execution(true);
  MotionCore core(backend);
  grant_normal(core);
  const auto request = pick_approach_request("safety-stop-me");
  MotionPortResult result;
  auto execution = std::async(
    std::launch::async, [&]() {
      result = core.planAndExecuteMotion(request);
    });

  ASSERT_TRUE(backend->wait_until_execution_started());
  ASSERT_TRUE(
    core.request_stop(
      request.execution_id,
      arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED));

  ASSERT_EQ(execution.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  execution.get();
  EXPECT_EQ(result.failure_kind, MotionFailureKind::SAFETY_PREEMPTED);
  EXPECT_EQ(backend->hold_calls(), 1);
}
}  // namespace
}  // namespace arm_cell_motion
