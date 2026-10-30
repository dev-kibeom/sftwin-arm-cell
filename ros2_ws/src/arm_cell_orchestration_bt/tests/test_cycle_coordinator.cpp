#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <filesystem>
#include <string>
#include <vector>

#include "arm_cell_orchestration_bt/cycle_coordinator.hpp"
#include "arm_cell_orchestration_bt/failure_trace_record.hpp"
#include "arm_cell_orchestration_bt/recipe_repository.hpp"
#include "arm_cell_orchestration_bt/vision_request_timeout.hpp"
#include "arm_cell_motion_moveit2/fake_motion_backend.hpp"
#include "arm_cell_motion_moveit2/motion_core.hpp"

namespace
{
using Coordinator = arm_cell_orchestration_bt::CycleCoordinator;
using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
using DetectTarget = arm_cell_interfaces::srv::DetectTarget;
using MissionExitReason = arm_cell_interfaces::msg::MissionExitReason;
using MotionCapability = arm_cell_interfaces::msg::MotionCapability;
using MotionTaskResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;
using MotionTaskType = arm_cell_interfaces::msg::MotionTaskType;
using SafetyState = arm_cell_interfaces::msg::SafetyState;

TEST(VisionRequestTimeout, UsesTheDurationSuppliedToDetectTarget)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = 0;
  duration.nanosec = 500'000'000U;

  const auto timeout = arm_cell_orchestration_bt::vision_request_timeout(duration);
  ASSERT_TRUE(timeout);
  EXPECT_EQ(*timeout, std::chrono::milliseconds(500));
  EXPECT_EQ(
    arm_cell_orchestration_bt::vision_response_wait_timeout(*timeout),
    std::chrono::milliseconds(600));
}

TEST(VisionRequestTimeout, RejectsMalformedNegativeDurations)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = -1;
  EXPECT_FALSE(arm_cell_orchestration_bt::vision_request_timeout(duration));
  duration.sec = 0;
  duration.nanosec = 1'000'000'000U;
  EXPECT_FALSE(arm_cell_orchestration_bt::vision_request_timeout(duration));
}

SafetyState normal_safety()
{
  SafetyState state;
  state.valid = true;
  state.required_inputs_fresh = true;
  state.safety_state = SafetyState::SAFETY_STATE_SAFE;
  state.motion_capability.value = MotionCapability::MOTION_NORMAL;
  state.motion_envelope_valid = true;
  state.max_velocity_scale = 1.0F;
  state.max_acceleration_scale = 1.0F;
  return state;
}

SafetyState recovery_safety()
{
  auto state = normal_safety();
  state.safety_state = SafetyState::SAFETY_STATE_RECOVERY_REQUIRED;
  state.motion_capability.value = MotionCapability::MOTION_RECOVERY_ONLY;
  return state;
}

bool valid_motion_goal(const ExecuteTask::Goal & goal)
{
  if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
    return goal.has_target_pose && goal.has_grasp_width &&
           goal.target_pose.header.frame_id == "base_link" &&
           goal.grasp_width_mm >= 0.0F && goal.grasp_width_mm <= 85.0F;
  }
  if (goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE) {
    return goal.has_target_pose && goal.target_pose.header.frame_id == "base_link";
  }
  return goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME;
}

DetectTarget::Response found_target()
{
  DetectTarget::Response response;
  response.result_code.value =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  response.has_target_pose = true;
  response.target_pose.header.frame_id = "base_link";
  response.target_pose.pose.position.x = 0.12;
  response.target_pose.pose.position.y = -0.34;
  response.target_pose.pose.position.z = 0.56;
  response.target_pose.pose.orientation.w = 1.0;
  response.has_target_yaw = true;
  response.has_estimated_height = true;
  response.estimated_height_m = 0.12F;
  return response;
}

DetectTarget::Response vision_result(uint8_t code)
{
  DetectTarget::Response response;
  response.result_code.value = code;
  return response;
}

Coordinator::MotionResult motion_result(uint8_t code)
{
  Coordinator::MotionResult result;
  result.code.value = code;
  return result;
}

struct Fixture
{
  std::vector<uint8_t> motion_tasks;
  std::vector<ExecuteTask::Goal> motion_goals;
  std::vector<uint8_t> phases;
  SafetyState safety = normal_safety();
  int cancel_count = 0;
  int safety_read_count = 0;
  int recipe_load_count = 0;
  bool has_recipe = true;
  Coordinator::MissionRecipe recipe;
  Coordinator::VisionFn vision = [this](const DetectTarget::Request &) {
      return found_target();
    };
  Coordinator::MotionFn motion = [this](const ExecuteTask::Goal & goal) {
      motion_tasks.push_back(goal.task_type.value);
      motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME &&
        motion_tasks.size() > 1U &&
        motion_tasks[motion_tasks.size() - 2U] == MotionTaskType::TASK_TYPE_PLACE)
      {
        recipe.grasp_width_mm = 55.0F;
        recipe.place_pose.pose.position.x = 1.3;
      }
      if (!valid_motion_goal(goal)) {
        return motion_result(MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };

  Fixture()
  {
    recipe.has_grasp_width = true;
    recipe.recipe_id = "fixture-recipe-v1";
    recipe.target_id = "target_fixture";
    recipe.schema_version = 1;
    recipe.grasp_width_mm = 42.0F;
    recipe.has_grasp_yaw = true;
    recipe.grasp_yaw_rad = 1.2;
    recipe.has_place_pose = true;
    recipe.place_pose.header.frame_id = "base_link";
    recipe.place_pose.pose.position.x = 0.8;
    recipe.place_pose.pose.position.y = -0.2;
    recipe.place_pose.pose.orientation.w = 1.0;
    recipe.place_position_tolerance_m = 0.012;
    recipe.place_orientation_tolerance_rad = 0.05;
    recipe.place_approach_direction_object.z = 1.0;
    recipe.place_approach_distance_m = 0.015;
    recipe.retract_distance_m = 0.05;
  }

  Coordinator make_coordinator()
  {
    Coordinator::Callbacks callbacks;
    callbacks.detect_target = vision;
    callbacks.execute_task = motion;
    callbacks.cancel_motion = [this]() {cancel_count++;};
    callbacks.read_safety = [this]() {
        ++safety_read_count;
        return safety;
    };
    callbacks.load_recipe = [this](const std::string &) {
        ++recipe_load_count;
        return has_recipe ? std::optional<Coordinator::MissionRecipe>{recipe} : std::nullopt;
      };
    callbacks.publish_phase = [this](uint8_t phase) {phases.push_back(phase);};
    return Coordinator(std::move(callbacks));
  }
};
}  // namespace

namespace
{
std::string action_uuid_hex(uint8_t byte)
{
  static constexpr char hex[] = "0123456789abcdef";
  std::string result;
  result.reserve(32);
  for (int index = 0; index < 16; ++index) {
    result.push_back(hex[byte >> 4]);
    result.push_back(hex[byte & 0x0f]);
  }
  return result;
}

Coordinator::Result run_backend_failure_episode(
  Fixture & fixture, const bool safety_preempts, std::vector<std::string> & terminal_records)
{
  auto backend = std::make_shared<arm_cell_motion_moveit2::FakeMotionBackend>();
  backend->set_available(false);
  arm_cell_motion_moveit2::MotionGeometry geometry;
  geometry.configured = true;
  geometry.observation_reference = "profile_reference";
  geometry.observation_to_object.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.rotation.w = 1.0;
  geometry.insertion_axis_tcp.z = 1.0;
  geometry.pick_approach_distance_m = 0.1;
  geometry.pick_retract_distance_m = 0.05;
  arm_cell_motion_moveit2::MotionCore motion_core(
    backend, std::chrono::milliseconds(500), geometry);
  const auto safety = normal_safety();
  const auto accepted_at = arm_cell_motion_moveit2::MotionCore::Clock::now();
  motion_core.update_safety_state(safety, accepted_at);
  const auto action_uuid = arm_cell_motion_moveit2::make_uuid(0x2a);
  const auto motion_identity = action_uuid_hex(0x2a);
  fixture.motion = [&fixture, &motion_core, &action_uuid, &motion_identity, safety_preempts](
    const ExecuteTask::Goal & goal)
    {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value != MotionTaskType::TASK_TYPE_PICK) {
        return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
      }
      std::string diagnostic;
      auto result = motion_result(
        motion_core.accept_goal_with_diagnostic(action_uuid, goal, diagnostic));
      result.diagnostic_detail = diagnostic;
      result.motion_execution_id = motion_identity;
      if (safety_preempts) {
        fixture.safety = recovery_safety();
      }
      return result;
    };
  auto coordinator = fixture.make_coordinator();
  auto result = coordinator.execute("target");
  if (result.motion_failure) {
    const auto & failure = *result.motion_failure;
    arm_cell_orchestration_bt::emit_terminal_failure_record(
      arm_cell_orchestration_bt::TerminalFailureRecord{
        "execute-cycle-uuid", "delivery-uuid", "target-profile",
        failure.motion_execution_id, failure.task_type, failure.code.value,
        result.exit_reason.value, failure.diagnostic_detail},
      [&terminal_records](const std::string & record) {terminal_records.push_back(record);});
  }
  EXPECT_FALSE(backend->available());
  EXPECT_EQ(backend->submit_calls(), 0);
  return result;
}
}  // namespace

TEST(CycleCoordinatorTest, InjectedPersistentMotionBackendFailureHasOneReconstructableEpisode)
{
  Fixture fixture;
  std::vector<std::string> records;

  const auto result = run_backend_failure_episode(fixture, false, records);

  ASSERT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_EQ(
    result.motion_failure->code.value,
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(result.motion_failure->diagnostic_detail, "backend unavailable");
  EXPECT_EQ(result.motion_failure->motion_execution_id, action_uuid_hex(0x2a));
  EXPECT_EQ(result.motion_failure->task_type, MotionTaskType::TASK_TYPE_PICK);
  EXPECT_EQ(fixture.safety_read_count, 6);
  ASSERT_EQ(fixture.motion_tasks.size(), 2U);
  EXPECT_EQ(fixture.motion_tasks.back(), MotionTaskType::TASK_TYPE_PICK);
  ASSERT_EQ(records.size(), 1U);
  EXPECT_NE(records.front().find("execute_cycle_id=execute-cycle-uuid"), std::string::npos);
  EXPECT_NE(records.front().find("delivery_id=delivery-uuid"), std::string::npos);
  EXPECT_NE(records.front().find("target_id=target-profile"), std::string::npos);
  EXPECT_NE(
    records.front().find("motion_execution_id=" + action_uuid_hex(0x2a)),
    std::string::npos);
  EXPECT_NE(records.front().find("motion_result_code=7"), std::string::npos);
  EXPECT_NE(records.front().find("exit_reason=3"), std::string::npos);
  EXPECT_NE(records.front().find("diagnostic=\"backend unavailable\""), std::string::npos);
}

TEST(CycleCoordinatorTest, InjectedMotionBackendFailureKeepsProvenanceWhenSafetyPreempts)
{
  Fixture fixture;
  std::vector<std::string> records;

  const auto result = run_backend_failure_episode(fixture, true, records);

  ASSERT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_EQ(
    result.motion_failure->code.value,
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(result.motion_failure->diagnostic_detail, "backend unavailable");
  EXPECT_EQ(result.motion_failure->motion_execution_id, action_uuid_hex(0x2a));
  EXPECT_EQ(fixture.safety_read_count, 5);
  ASSERT_EQ(records.size(), 1U);
  EXPECT_NE(records.front().find("motion_result_code=7"), std::string::npos);
  EXPECT_NE(
    records.front().find(
      "exit_reason=" + std::to_string(
        MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED)), std::string::npos);
  EXPECT_NE(records.front().find("diagnostic=\"backend unavailable\""), std::string::npos);
}

TEST(CycleCoordinatorTest, ExecutesNominalCycleUntilVisionReportsDepleted)
{
  Fixture fixture;
  auto call_count = 0;
  fixture.vision = [&call_count](const DetectTarget::Request &) {
      ++call_count;
      return call_count == 1 ? found_target() : vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_DEPLETED);
  EXPECT_EQ(
    std::count(
      fixture.phases.begin(), fixture.phases.end(),
      arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_FINISHED),
    1);
  ASSERT_EQ(fixture.motion_tasks.size(), 5U);
  EXPECT_EQ(fixture.motion_tasks[0], MotionTaskType::TASK_TYPE_GO_HOME);
  EXPECT_EQ(fixture.motion_tasks[1], MotionTaskType::TASK_TYPE_PICK);
  EXPECT_EQ(fixture.motion_tasks[2], MotionTaskType::TASK_TYPE_PLACE);
  EXPECT_EQ(fixture.motion_tasks[3], MotionTaskType::TASK_TYPE_GO_HOME);
  EXPECT_EQ(fixture.motion_tasks[4], MotionTaskType::TASK_TYPE_GO_HOME);
  ASSERT_EQ(fixture.motion_goals.size(), 5U);
  EXPECT_TRUE(fixture.motion_goals[1].has_target_pose);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[1].target_pose.pose.position.x, 0.12);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[1].target_pose.pose.position.y, -0.34);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[1].target_pose.pose.position.z, 0.56);
  EXPECT_FLOAT_EQ(fixture.motion_goals[1].estimated_object_height_m, 0.12F);
  EXPECT_TRUE(fixture.motion_goals[1].has_grasp_width);
  EXPECT_FLOAT_EQ(fixture.motion_goals[1].grasp_width_mm, 42.0F);
  EXPECT_TRUE(fixture.motion_goals[1].target_pose.pose.orientation.w > 0.99);
  EXPECT_TRUE(fixture.motion_goals[2].has_target_pose);
  EXPECT_EQ(fixture.motion_goals[2].target_pose.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.position.x, 0.8);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.position.y, -0.2);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].place_position_tolerance_m, 0.012);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].place_orientation_tolerance_rad, 0.05);
}

TEST(CycleCoordinatorTest, ValidatesRecipeBeforeDispatchingAnyMotionIncludingInitialHome)
{
  Fixture fixture;
  fixture.has_recipe = false;
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR);
  EXPECT_TRUE(fixture.motion_goals.empty());
  EXPECT_EQ(fixture.recipe_load_count, 1);
}

TEST(CycleCoordinatorTest, LoadsRecipeOnlyOnceAcrossMultipleTargetIterations)
{
  Fixture fixture;
  auto call_count = 0;
  fixture.vision = [&call_count](const DetectTarget::Request &) {
      ++call_count;
      return call_count <= 2 ? found_target() : vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_DEPLETED);
  EXPECT_EQ(fixture.recipe_load_count, 1);
}

TEST(CycleCoordinatorTest, LaterIterationUsesTheInitiallySelectedRecipe)
{
  Fixture fixture;
  auto call_count = 0;
  fixture.vision = [&call_count](const DetectTarget::Request &) {
      ++call_count;
      return call_count <= 2 ? found_target() : vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  ASSERT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_DEPLETED);
  ASSERT_EQ(fixture.motion_goals.size(), 9U);
  EXPECT_FLOAT_EQ(fixture.motion_goals[1].grasp_width_mm, 42.0F);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.position.x, 0.8);
  EXPECT_FLOAT_EQ(fixture.motion_goals[5].grasp_width_mm, 42.0F);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[6].target_pose.pose.position.x, 0.8);
  EXPECT_EQ(fixture.recipe_load_count, 1);
}

TEST(CycleCoordinatorTest, VisionYawAbsentUsesRequiredRecipeYawFallback)
{
  Fixture fixture;
  auto call_count = 0;
  fixture.vision = [&call_count](const DetectTarget::Request &) {
      ++call_count;
      if (call_count > 1) {
        return vision_result(
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
      }
      auto response = found_target();
      response.has_target_yaw = false;
      response.target_pose.pose.orientation.w = 0.0;
      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_DEPLETED);
  ASSERT_GE(fixture.motion_goals.size(), 2U);
  EXPECT_NEAR(fixture.motion_goals[1].target_pose.pose.orientation.z, std::sin(0.6), 1e-6);
  EXPECT_NEAR(fixture.motion_goals[1].target_pose.pose.orientation.w, std::cos(0.6), 1e-6);
}

TEST(CycleCoordinatorTest, PreservesVisionFailureAsVisionError)
{
  Fixture fixture;
  fixture.vision = [](const DetectTarget::Request &) {
      return vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
}

TEST(CycleCoordinatorTest, PreservesPersistentPickFailureCauseAndMotionIdentity)
{
  Fixture fixture;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
        auto result = motion_result(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
        result.diagnostic_detail = "persistent backend transport failure";
        result.motion_execution_id = "motion-action-uuid";
        return result;
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  EXPECT_EQ(result.diagnostic_detail, "persistent backend transport failure");
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_EQ(result.motion_failure->task_type, MotionTaskType::TASK_TYPE_PICK);
  EXPECT_EQ(
    result.motion_failure->code.value,
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(result.motion_failure->diagnostic_detail, "persistent backend transport failure");
  EXPECT_EQ(result.motion_failure->motion_execution_id, "motion-action-uuid");
  ASSERT_EQ(fixture.motion_tasks.size(), 2U);
  EXPECT_EQ(fixture.motion_tasks[0], MotionTaskType::TASK_TYPE_GO_HOME);
  EXPECT_EQ(fixture.motion_tasks[1], MotionTaskType::TASK_TYPE_PICK);
}

TEST(CycleCoordinatorTest, SafetyPrecedenceRetainsTheOriginalMotionFailureTrace)
{
  Fixture fixture;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
        fixture.safety = recovery_safety();
        auto result = motion_result(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
        result.diagnostic_detail = "persistent backend transport failure";
        result.motion_execution_id = "motion-action-uuid";
        return result;
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_EQ(
    result.motion_failure->code.value,
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(result.motion_failure->diagnostic_detail, "persistent backend transport failure");
  EXPECT_EQ(result.motion_failure->motion_execution_id, "motion-action-uuid");
  ASSERT_EQ(fixture.motion_tasks.size(), 2U);
  EXPECT_EQ(fixture.motion_tasks.back(), MotionTaskType::TASK_TYPE_PICK);
}

TEST(CycleCoordinatorTest, TerminalFailureRecordJoinsExistingMissionAndMotionIdentities)
{
  std::vector<std::string> records;
  arm_cell_orchestration_bt::emit_terminal_failure_record(
    arm_cell_orchestration_bt::TerminalFailureRecord{
    "execute-cycle-uuid", "delivery-uuid", "target-profile", "motion-action-uuid",
    MotionTaskType::TASK_TYPE_PICK,
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE,
    MissionExitReason::MISSION_EXIT_MOTION_ERROR,
    "persistent backend transport failure"},
    [&records](const std::string & record) {records.push_back(record);});

  ASSERT_EQ(records.size(), 1U);
  const auto & record = records.front();
  EXPECT_NE(record.find("execute_cycle_id=execute-cycle-uuid"), std::string::npos);
  EXPECT_NE(record.find("delivery_id=delivery-uuid"), std::string::npos);
  EXPECT_NE(record.find("target_id=target-profile"), std::string::npos);
  EXPECT_NE(record.find("motion_execution_id=motion-action-uuid"), std::string::npos);
  EXPECT_NE(record.find("motion_result_code=7"), std::string::npos);
  EXPECT_NE(record.find("exit_reason=3"), std::string::npos);
  EXPECT_NE(
    record.find("diagnostic=\"persistent backend transport failure\""),
    std::string::npos);
}

TEST(CycleCoordinatorTest, UnattributedMotionCancellationIsMotionError)
{
  Fixture fixture;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      return motion_result(MotionTaskResultCode::TASK_RESULT_CANCELED);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
}

TEST(CycleCoordinatorTest, CallerAttributedMotionCancellationIsCanceled)
{
  Fixture fixture;
  Coordinator * coordinator_ptr = nullptr;
  fixture.motion = [&fixture, &coordinator_ptr](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      coordinator_ptr->set_cancel_requested(true);
      return motion_result(MotionTaskResultCode::TASK_RESULT_CANCELED);
    };
  auto coordinator = fixture.make_coordinator();
  coordinator_ptr = &coordinator;

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CANCELED);
}

TEST(CycleCoordinatorTest, CallerCancellationCancelsMotionAndPreservesCancellation)
{
  Fixture fixture;
  auto coordinator = fixture.make_coordinator();
  coordinator.set_cancel_requested(true);

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CANCELED);
  EXPECT_EQ(fixture.cancel_count, 1);
}

TEST(CycleCoordinatorTest, SafetyPreemptionWinsOverCallerCancelAtMotionTerminalDecision)
{
  Fixture fixture;
  Coordinator * coordinator_ptr = nullptr;
  fixture.motion = [&fixture, &coordinator_ptr](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      fixture.safety = recovery_safety();
      coordinator_ptr->set_cancel_requested(true);
      return motion_result(MotionTaskResultCode::TASK_RESULT_CANCELED);
    };
  auto coordinator = fixture.make_coordinator();
  coordinator_ptr = &coordinator;

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
}

TEST(CycleCoordinatorTest, SafetyPreemptionCancelsWithoutRecoveryAndPreservesFailure)
{
  Fixture fixture;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME) {
        fixture.safety = recovery_safety();
        return motion_result(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(fixture.cancel_count, 1);
  ASSERT_EQ(fixture.motion_tasks.size(), 1U);
  EXPECT_EQ(fixture.motion_tasks[0], MotionTaskType::TASK_TYPE_GO_HOME);
}

TEST(CycleCoordinatorTest, SafetyPreemptionDoesNotRetryOrIssueRetract)
{
  Fixture fixture;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME) {
        fixture.safety = recovery_safety();
        return motion_result(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
      }
      fixture.safety = recovery_safety();
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  ASSERT_EQ(fixture.motion_tasks.size(), 1U);
  EXPECT_EQ(fixture.motion_tasks[0], MotionTaskType::TASK_TYPE_GO_HOME);
}

TEST(CycleCoordinatorTest, SafetyDowngradeAfterVisionBlocksNextNormalTask)
{
  Fixture fixture;
  fixture.vision = [&fixture](const DetectTarget::Request &) {
      fixture.safety = recovery_safety();
      return found_target();
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  ASSERT_EQ(fixture.motion_tasks.size(), 1U);
  EXPECT_EQ(fixture.motion_tasks[0], MotionTaskType::TASK_TYPE_GO_HOME);
}

TEST(CycleCoordinatorTest, SafetyPreemptionWinsOverVisionDepletionAtTerminalDecision)
{
  Fixture fixture;
  fixture.vision = [&fixture](const DetectTarget::Request &) {
      fixture.safety = recovery_safety();
      return vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
}

TEST(CycleCoordinatorTest, SafetyPreemptionBeforeDepletionCommitDoesNotPublishFinished)
{
  Fixture fixture;
  fixture.vision = [&fixture](const DetectTarget::Request &) {
      fixture.safety = recovery_safety();
      return vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(
    std::count(
      fixture.phases.begin(), fixture.phases.end(),
      arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_FINISHED),
    0);
}

TEST(CycleCoordinatorTest, SafetyPreemptionWinsOverVisionErrorAtTerminalDecision)
{
  Fixture fixture;
  fixture.vision = [&fixture](const DetectTarget::Request &) {
      fixture.safety = recovery_safety();
      return vision_result(arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
}

TEST(CycleCoordinatorTest, MissingGraspWidthCannotEmitPickGoal)
{
  Fixture fixture;
  fixture.recipe.has_grasp_width = false;
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR);
  EXPECT_TRUE(fixture.motion_goals.empty());
}

TEST(CycleCoordinatorTest, MissingRecipeYawCannotEmitPickGoal)
{
  Fixture fixture;
  fixture.recipe.has_grasp_yaw = false;
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR);
  EXPECT_TRUE(fixture.motion_goals.empty());
}

TEST(CycleCoordinatorTest, MissingRecipeCannotEmitPickOrPlaceGoal)
{
  Fixture fixture;
  fixture.has_recipe = false;
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR);
  EXPECT_TRUE(fixture.motion_goals.empty());
}

TEST(CycleCoordinatorTest, NonFiniteRecipeGraspWidthCannotEmitPickOrPlaceGoal)
{
  Fixture fixture;
  fixture.recipe.grasp_width_mm = std::numeric_limits<float>::quiet_NaN();
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR);
  EXPECT_TRUE(fixture.motion_goals.empty());
}

TEST(CycleCoordinatorTest, ZeroQuaternionRecipePoseCannotEmitPickOrPlaceGoal)
{
  Fixture fixture;
  fixture.recipe.place_pose.pose.orientation.w = 0.0;
  auto vision_calls = 0;
  fixture.vision = [&vision_calls](const DetectTarget::Request &) {
      ++vision_calls;
      return vision_calls == 1 ? found_target() : vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR);
  EXPECT_TRUE(fixture.motion_goals.empty());
}

TEST(CycleCoordinatorTest, PlaceUsesMissionRecipeInsteadOfVisionDestination)
{
  Fixture fixture;
  fixture.recipe.place_pose.pose.orientation.z = 0.7071067811865476;
  fixture.recipe.place_pose.pose.orientation.w = 0.7071067811865476;
  fixture.recipe.has_place_tool_orientation_preference = true;
  fixture.recipe.place_tool_orientation_preference.x = 1.0;
  fixture.recipe.place_tool_orientation_preference.w = 0.0;
  fixture.vision = [](const DetectTarget::Request &) {
      auto response = found_target();
      response.target_pose.pose.position.x = 1.7;
      response.target_pose.pose.position.y = 1.8;
      return response;
    };
  auto call_count = 0;
  auto original_vision = fixture.vision;
  fixture.vision = [original_vision, &call_count](const DetectTarget::Request & request) {
      ++call_count;
      if (call_count > 1) {
        return vision_result(
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
      }
      return original_vision(request);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_DEPLETED);
  ASSERT_GE(fixture.motion_goals.size(), 3U);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[1].target_pose.pose.position.x, 1.7);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.position.x, 0.8);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.position.y, -0.2);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.orientation.z, 0.7071067811865476);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.orientation.w, 0.7071067811865476);
  EXPECT_EQ(
    fixture.motion_goals[2].place_orientation_constraint,
    arm_cell_interfaces::action::ExecuteTask::Goal::PLACE_ORIENTATION_FIXED);
  EXPECT_DOUBLE_EQ(
    fixture.motion_goals[2].place_tool_orientation_preference.x, 1.0);
  EXPECT_DOUBLE_EQ(
    fixture.motion_goals[2].place_tool_orientation_preference.w, 0.0);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].place_approach_direction_object.z, 1.0);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].place_approach_distance_m, 0.015);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].place_retract_distance_m, 0.05);
}

TEST(CycleCoordinatorTest, RawPartBoundedSelectedObjectOrientationReachesPlaceGoal)
{
  const auto loaded = arm_cell_orchestration_bt::RecipeRepository(
    std::filesystem::path(ARM_CELL_TEST_RECIPE_DIR)).load("RawPart");
  ASSERT_TRUE(loaded.recipe);
  EXPECT_EQ(
    loaded.recipe->place_orientation_constraint,
    arm_cell_orchestration_bt::PlaceOrientationConstraint::BOUNDED);
  EXPECT_DOUBLE_EQ(loaded.recipe->place_pose.pose.orientation.x, 0.7071067811865476);
  EXPECT_DOUBLE_EQ(loaded.recipe->place_pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(loaded.recipe->place_pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(loaded.recipe->place_pose.pose.orientation.w, 0.7071067811865476);
  EXPECT_DOUBLE_EQ(loaded.recipe->place_orientation_tolerance_rad, 0.17453292519943295);
  EXPECT_FALSE(loaded.recipe->has_place_tool_orientation_preference);
  Fixture fixture;
  fixture.recipe = *loaded.recipe;
  auto vision_call_count = 0;
  fixture.vision = [&vision_call_count](const DetectTarget::Request &) {
      ++vision_call_count;
      return vision_call_count == 1 ? found_target() : vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("RawPart");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_DEPLETED);
  ASSERT_GE(fixture.motion_goals.size(), 3U);
  const auto & goal = fixture.motion_goals[2];
  EXPECT_EQ(
    goal.place_orientation_constraint,
    arm_cell_interfaces::action::ExecuteTask::Goal::PLACE_ORIENTATION_BOUNDED);
  EXPECT_FALSE(goal.has_place_tool_orientation_preference);
  EXPECT_DOUBLE_EQ(goal.place_orientation_tolerance_rad, 0.17453292519943295);
  EXPECT_DOUBLE_EQ(goal.target_pose.pose.orientation.x, 0.7071067811865476);
  EXPECT_DOUBLE_EQ(goal.target_pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(goal.target_pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(goal.target_pose.pose.orientation.w, 0.7071067811865476);
}

TEST(CycleCoordinatorTest, NonBaseLinkPlaceRecipeIsRejectedBeforePlaceDispatch)
{
  Fixture fixture;
  fixture.recipe.place_pose.header.frame_id = "map";
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR);
  EXPECT_TRUE(fixture.motion_goals.empty());
}

TEST(CycleCoordinatorTest, PickFakeOracleRejectsNonBaseLinkVisionPose)
{
  Fixture fixture;
  fixture.vision = [](const DetectTarget::Request &) {
      auto response = found_target();
      response.target_pose.header.frame_id = "camera_link";
      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  ASSERT_EQ(fixture.motion_goals.size(), 2U);
  EXPECT_EQ(fixture.motion_goals[1].task_type.value, MotionTaskType::TASK_TYPE_PICK);
}

TEST(CycleCoordinatorTest, SafetyPreemptionWinsOverConcurrentCallerCancellation)
{
  Fixture fixture;
  Coordinator * coordinator_ptr = nullptr;
  fixture.vision = [&fixture, &coordinator_ptr](const DetectTarget::Request &) {
      fixture.safety = recovery_safety();
      coordinator_ptr->set_cancel_requested(true);
      return found_target();
    };
  auto coordinator = fixture.make_coordinator();
  coordinator_ptr = &coordinator;

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
}

TEST(CancellationRegistryTest, CancellationBeforeRegistrationReachesExecutionAndMotion)
{
  arm_cell_orchestration_bt::CancellationRegistry registry;
  bool execution_canceled = false;
  bool motion_canceled = false;

  registry.request_caller_cancel("cycle");
  registry.register_execution("cycle", [&]() {execution_canceled = true;});
  registry.register_active_motion("cycle", [&]() {motion_canceled = true;});

  EXPECT_TRUE(execution_canceled);
  EXPECT_TRUE(motion_canceled);
}

TEST(CancellationRegistryTest, CallerCancellationForwardsToActiveMotion)
{
  arm_cell_orchestration_bt::CancellationRegistry registry;
  bool execution_canceled = false;
  int motion_cancel_count = 0;

  registry.register_execution("cycle", [&]() {execution_canceled = true;});
  registry.register_active_motion("cycle", [&]() {motion_cancel_count++;});
  registry.request_caller_cancel("cycle");

  EXPECT_TRUE(execution_canceled);
  EXPECT_EQ(motion_cancel_count, 1);
}

TEST(CancellationRegistryTest, SecondaryCancellationDoesNotSynthesizeCallerCancellation)
{
  arm_cell_orchestration_bt::CancellationRegistry registry;
  bool execution_canceled = false;
  int motion_cancel_count = 0;

  registry.register_execution("cycle", [&]() {execution_canceled = true;});
  registry.register_active_motion("cycle", [&]() {motion_cancel_count++;});
  registry.request_secondary_motion_cancel("cycle");

  EXPECT_FALSE(execution_canceled);
  EXPECT_EQ(motion_cancel_count, 1);
}
