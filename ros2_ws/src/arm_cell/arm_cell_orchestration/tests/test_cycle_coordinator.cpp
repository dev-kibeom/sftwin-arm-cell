#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <limits>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

#include "arm_cell_orchestration/cycle_coordinator.hpp"
#include "arm_cell_orchestration/failure_trace_record.hpp"
#include "arm_cell_orchestration/recipe_repository.hpp"
#include "arm_cell_orchestration/vision_request_timeout.hpp"

namespace
{
using Coordinator = arm_cell_orchestration::CycleCoordinator;
using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
using ExecuteDisturb = arm_cell_interfaces::action::ExecuteDisturb;
using DetectTarget = arm_cell_interfaces::srv::DetectTarget;
using MissionExitReason = arm_cell_interfaces::msg::MissionExitReason;
using MotionCapability = arm_cell_interfaces::msg::MotionCapability;
using MotionTaskResultCode = arm_cell_interfaces::msg::MotionTaskResultCode;
using MotionTaskType = arm_cell_interfaces::msg::MotionTaskType;
using SafetyState = arm_cell_interfaces::msg::SafetyState;
using SafetyObservation = arm_cell_orchestration::SafetyObservation;
using arm_cell_orchestration::UnobservedYawPolicy;
constexpr char kSyntheticMotionExecutionId[] = "2a2a2a2a2a2a2a2a2a2a2a2a2a2a2a2a";

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

SafetyObservation observation(const SafetyState & state)
{
  const auto now = SafetyObservation::Clock::now();
  return arm_cell_orchestration::evaluate_safety_observation(
    state, true, now, now, std::chrono::milliseconds(500));
}

Coordinator::MotionResult motion_result(uint8_t code)
{
  Coordinator::MotionResult result;
  result.code.value = code;
  return result;
}

Coordinator::MotionResult pick_work_failure(uint8_t code)
{
  auto result = motion_result(code);
  result.object_disposition =
    arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_UNCHANGED;
  return result;
}

Coordinator::MotionResult successful_motion_result(const ExecuteTask::Goal & goal)
{
  auto result = motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
  result.object_disposition = goal.task_type.value == MotionTaskType::TASK_TYPE_PICK ?
    arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_HELD :
    goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE ?
    arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_PLACED :
    goal.task_type.value == MotionTaskType::TASK_TYPE_RETURN_HELD ?
    arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED :
    arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_UNCHANGED;
  return result;
}

DetectTarget::Response vision_result(uint8_t code)
{
  DetectTarget::Response response;
  response.result_code.value = code;
  return response;
}

DetectTarget::Response found_target()
{
  DetectTarget::Response response;
  response.result_code.value =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  response.material_candidates.emplace_back();
  response.material_candidates.front().target_pose.header.frame_id = "base_link";
  response.material_candidates.front().target_pose.pose.position.x = 0.12;
  response.material_candidates.front().target_pose.pose.position.y = -0.34;
  response.material_candidates.front().target_pose.pose.position.z = 0.56;
  response.material_candidates.front().target_pose.pose.orientation.w = 1.0;
  response.material_candidates.front().orientation_symmetry_order = 4;
  response.acquisition_id = "acquisition-test";
  for (auto & candidate : response.material_candidates) {
    candidate.acquisition_id = response.acquisition_id;
  }
  response.material_candidates.front().candidate_id = "candidate-test";
  response.material_candidates.front().has_estimated_height = true;
  response.material_candidates.front().estimated_height_m = 0.12F;
  response.material_candidates.front().has_observed_rectangle = true;
  response.material_candidates.front().rectangle_long_side_m = 0.07F;
  response.material_candidates.front().rectangle_short_side_m = 0.07F;
  response.scene_generation = 1U;
  response.material_candidates.front().acquisition_id = response.acquisition_id;
  return response;
}

DetectTarget::Response disturbance_target()
{
  auto response = vision_result(
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_GEOMETRY_ERROR);
  response.acquisition_id = "disturbance-acquisition";
  for (auto & candidate : response.material_candidates) {
    candidate.acquisition_id = response.acquisition_id;
  }
  response.scene_generation = 73U;
  arm_cell_interfaces::msg::DisturbanceCandidate candidate;
  candidate.candidate_id = "unresolved-region";
  candidate.category = arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_OTHER_UNRESOLVED;
  candidate.center.header.frame_id = "base_link";
  candidate.center.header.stamp.sec = 12;
  candidate.center.pose.position.x = 0.31;
  candidate.center.pose.position.y = -0.17;
  candidate.center.pose.position.z = 0.08;
  candidate.center.pose.orientation.w = 1.0;
  candidate.observed_height_m = 0.08;
  response.disturbance_candidates = {candidate};
  return response;
}



struct Fixture
{
  using PnpAction = arm_cell_interfaces::action::ExecutePnp;
  using DisturbAction = arm_cell_interfaces::action::ExecuteDisturb;
  std::vector<uint8_t> motion_tasks;
  std::vector<ExecuteTask::Goal> motion_goals;
  std::vector<uint8_t> phases;
  std::vector<uint8_t> retry_counts;
  std::vector<bool> motion_status_wait_requests;
  SafetyState safety = normal_safety();
  std::atomic_int safety_read_count{0};
  int recipe_load_count = 0;
  int pnp_goal_count = 0;
  std::vector<PnpAction::Goal> pnp_goals;
  std::vector<DisturbAction::Goal> disturb_goals;
  std::vector<std::string> pnp_goal_uuids;
  std::function<void(uint64_t)> processed_count_hook;
  std::function<SafetyState()> safety_hook;
  std::function<SafetyObservation()> safety_observation_hook;
  std::function<std::optional<Coordinator::MotionStatus>()> motion_status = []() {
      return std::optional<Coordinator::MotionStatus>{};
    };
  Coordinator::RecordVisionResultFn record_vision_result;
  std::vector<Coordinator::DiagnosticRecord> diagnostic_records;
  std::mutex tree_status_mutex;
  struct TreeStatusEvent
  {
    std::string node;
    std::string previous;
    std::string current;
  };
  std::vector<TreeStatusEvent> tree_status_events;
  bool diagnostics_enabled{true};
  enum class DiagnosticSinkMode {STORE, QUEUE_FULL, STORAGE_FAILURE};
  DiagnosticSinkMode diagnostic_sink_mode{DiagnosticSinkMode::STORE};
  Coordinator::ReadVisionDiagnosticGenerationFn read_vision_diagnostic_generation = []() {
      return 0U;
    };
  Coordinator::WaitForVisionIdleFn wait_for_vision_idle = [](
    uint64_t, const Coordinator::CancelPredicateFn &) {return true;};
  bool emit_release_feedback = false;
  bool has_recipe = true;
  Coordinator::MissionRecipe recipe;
  std::function<DetectTarget::Response(const DetectTarget::Request &)> vision =
    [this](const DetectTarget::Request &) {
      return found_target();
    };
  std::function<Coordinator::DisturbResult(const DisturbAction::Goal &)> disturb =
    [this](const DisturbAction::Goal & goal) {
      disturb_goals.push_back(goal);
      return Coordinator::DisturbResult{
      arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_COMPLETED,
      "completed", "SUCCEEDED"};
    };
  Coordinator::MotionFn motion = [this](const ExecuteTask::Goal & goal) {
      motion_tasks.push_back(goal.task_type.value);
      motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME &&
        motion_tasks.size() > 1U &&
        motion_tasks[motion_tasks.size() - 2U] == MotionTaskType::TASK_TYPE_PLACE)
      {
        recipe.grasp_width_max_mm = 55.0F;
        recipe.place_pose.pose.position.x = 1.3;
      }
      return successful_motion_result(goal);
    };

  Fixture()
  {
    recipe.has_grasp_width_policy = true;
    recipe.recipe_id = "fixture-recipe-v1";
    recipe.target_id = "target_fixture";
    recipe.schema_version = 1;
    recipe.grasp_width_min_mm = 40.0F;
    recipe.grasp_width_max_mm = 80.0F;
    recipe.pick_approach_axis =
      arm_cell_orchestration::PickApproachAxis::TCP_INSERTION_AXIS;
    recipe.pick_approach_distance_m = 0.15;
    recipe.pick_grasp_position_tolerance_m = 0.01;
    recipe.pick_grasp_orientation_tolerance_rad = 0.08726646259971647;
    recipe.has_grasp_yaw = true;
    recipe.has_unobserved_yaw_policy = true;
    recipe.unobserved_yaw_policy = UnobservedYawPolicy::ALLOWED;
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

  Coordinator::Callbacks make_callbacks()
  {
    Coordinator::Callbacks callbacks;
    callbacks.detect_target = [this](
      const DetectTarget::Request & request) {
        EXPECT_EQ(request.algorithm, recipe.vision_algorithm);
        return vision(request);
      };
    callbacks.read_vision_diagnostic_generation = read_vision_diagnostic_generation;
    callbacks.wait_for_vision_idle = wait_for_vision_idle;
    callbacks.record_vision_result = record_vision_result;
    if (diagnostics_enabled) {
      callbacks.record_structured_diagnostic =
        [this](const Coordinator::DiagnosticRecord & record) {
          if (diagnostic_sink_mode == DiagnosticSinkMode::STORAGE_FAILURE) {
            throw std::runtime_error("diagnostic sink unavailable");
          }
          if (diagnostic_sink_mode == DiagnosticSinkMode::STORE) {
            diagnostic_records.push_back(record);
          }
        };
    }
    const auto execute_motion = [this](const ExecuteTask::Goal & goal) {
        auto result = motion(goal);
        if (result.result_code_available &&
          result.code.value == MotionTaskResultCode::TASK_RESULT_SUCCESS &&
          result.object_disposition ==
          arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN)
        {
          if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
            result.object_disposition =
              arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_HELD;
          } else if (goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE) {
            result.object_disposition =
              arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_PLACED;
          } else {
            result.object_disposition =
              arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_UNCHANGED;
          }
        }
        return result;
      };
    callbacks.execute_task = execute_motion;
    callbacks.execute_task_with_feedback = [this](
      const ExecuteTask::Goal & goal, const Coordinator::MotionFeedbackFn & feedback) {
        if (emit_release_feedback &&
          goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE && feedback)
        {
          feedback(arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_GRIPPER);
          feedback(arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_GRIPPER);
        }
        auto result = motion(goal);
        if (result.result_code_available &&
          result.code.value == MotionTaskResultCode::TASK_RESULT_SUCCESS &&
          result.object_disposition ==
          arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN)
        {
          if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
            result.object_disposition =
              arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_HELD;
          } else if (goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE) {
            result.object_disposition =
              arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_PLACED;
          } else {
            result.object_disposition =
              arm_cell_interfaces::action::ExecuteTask::Result::OBJECT_DISPOSITION_UNCHANGED;
          }
        }
        return result;
      };
    const auto execute_motion_with_feedback = callbacks.execute_task_with_feedback;
    callbacks.execute_pnp_with_feedback = [this, execute_motion, execute_motion_with_feedback](
      const PnpAction::Goal & pnp_goal, const Coordinator::PnpFeedbackFn & feedback) {
        pnp_goals.push_back(pnp_goal);
        ExecuteTask::Goal pick;
        pick.task_type.value = MotionTaskType::TASK_TYPE_PICK;
        pick.material_candidates = pnp_goal.material_candidates;
        pick.vision_acquisition_id = pnp_goal.vision_acquisition_id;
        pick.pick_approach_translation_direction =
          pnp_goal.pick_approach_translation_direction;
        pick.pick_approach_distance_m = pnp_goal.pick_approach_distance_m;
        pick.pick_grasp_position_tolerance_m = pnp_goal.pick_grasp_position_tolerance_m;
        pick.pick_grasp_orientation_tolerance_rad =
          pnp_goal.pick_grasp_orientation_tolerance_rad;
        pick.grasp_width_min_mm = pnp_goal.grasp_width_min_mm;
        pick.grasp_width_max_mm = pnp_goal.grasp_width_max_mm;
        pick.has_unobserved_yaw_fallback = pnp_goal.has_unobserved_yaw_fallback;
        pick.unobserved_yaw_fallback_rad = pnp_goal.unobserved_yaw_fallback_rad;
        auto picked = execute_motion(pick);
        Coordinator::PnpResult result;
        result.goal_uuid = "fixture-pnp-" + std::to_string(++pnp_goal_count);
        pnp_goal_uuids.push_back(result.goal_uuid);
        result.result_code_available = true;
        result.detail_code = picked.code;
        result.diagnostic_detail = picked.diagnostic_detail;
        result.action_terminal = picked.action_terminal;
        if (picked.code.value != MotionTaskResultCode::TASK_RESULT_SUCCESS ||
          picked.object_disposition != ExecuteTask::Result::OBJECT_DISPOSITION_HELD)
        {
          const bool retryable = picked.code.value ==
            MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE ||
            picked.code.value == MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED ||
            picked.code.value == MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE ||
            picked.code.value == MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED ||
            picked.code.value == MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED;
          const bool safely_unplaced = picked.object_disposition ==
            ExecuteTask::Result::OBJECT_DISPOSITION_UNCHANGED || picked.object_disposition ==
            ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED;
          result.outcome = picked.code.value == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED ?
            PnpAction::Result::PNP_OUTCOME_SAFETY_PREEMPTED : retryable && safely_unplaced ?
            picked.code.value == MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED ?
            PnpAction::Result::PNP_OUTCOME_CANDIDATES_EXHAUSTED :
            PnpAction::Result::PNP_OUTCOME_RETRYABLE : PnpAction::Result::PNP_OUTCOME_FAILED;
          return result;
        }
        ExecuteTask::Goal place;
        place.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
        place.target_pose = pnp_goal.place_target_pose;
        place.has_target_pose = pnp_goal.has_place_target_pose;
        place.place_position_tolerance_m = pnp_goal.place_position_tolerance_m;
        place.place_orientation_tolerance_rad = pnp_goal.place_orientation_tolerance_rad;
        place.place_orientation_constraint = pnp_goal.place_orientation_constraint;
        place.place_approach_direction_object = pnp_goal.place_approach_direction_object;
        place.place_approach_distance_m = pnp_goal.place_approach_distance_m;
        place.place_retract_distance_m = pnp_goal.place_retract_distance_m;
        place.has_place_tool_orientation_preference =
          pnp_goal.has_place_tool_orientation_preference;
        place.place_tool_orientation_preference = pnp_goal.place_tool_orientation_preference;
        const auto pnp_feedback = [feedback](std::uint8_t phase) {
            if (feedback &&
              phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_GRIPPER)
            {
              feedback(PnpAction::Feedback::NEXT_ACQUISITION_HINT_PREFETCH_NEXT);
            }
          };
        auto placed = emit_release_feedback ?
          execute_motion_with_feedback(place, pnp_feedback) : execute_motion(place);
        result.detail_code = placed.code;
        result.diagnostic_detail = placed.diagnostic_detail;
        result.action_terminal = placed.action_terminal;
        result.material_placed =
          placed.object_disposition == ExecuteTask::Result::OBJECT_DISPOSITION_PLACED;
        if (placed.code.value == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
          result.outcome = PnpAction::Result::PNP_OUTCOME_SAFETY_PREEMPTED;
        } else if (placed.code.value == MotionTaskResultCode::TASK_RESULT_SUCCESS &&
          result.material_placed)
        {
          result.outcome = PnpAction::Result::PNP_OUTCOME_COMPLETED;
        } else if (!result.material_placed &&
          (placed.code.value == MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE ||
          placed.code.value == MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED ||
          placed.code.value == MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE ||
          placed.code.value == MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED ||
          placed.code.value == MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED))
        {
          result.outcome = PnpAction::Result::PNP_OUTCOME_RETRYABLE;
        }
        return result;
      };
    callbacks.execute_disturb = [this](const DisturbAction::Goal & goal) {
        return disturb(goal);
      };
    callbacks.read_safety = [this]() {
        safety_read_count.fetch_add(1);
        return safety_observation_hook ? safety_observation_hook() :
               observation(safety_hook ? safety_hook() : safety);
      };
    callbacks.read_motion_status = [this](bool wait) {
        motion_status_wait_requests.push_back(wait);
        return motion_status();
      };
    callbacks.load_recipe = [this](const std::string &) {
        ++recipe_load_count;
        return has_recipe ? std::optional<Coordinator::MissionRecipe>{recipe} : std::nullopt;
      };
    callbacks.publish_phase = [this](uint8_t phase) {phases.push_back(phase);};
    callbacks.publish_processed_count = [this](uint64_t count) {
        if (processed_count_hook) {
          processed_count_hook(count);
        }
      };
    callbacks.publish_retry_count = [this](uint8_t count) {
        retry_counts.push_back(count);
      };
    callbacks.observe_tree_status = [this](
      const std::string & node_name, const std::string & previous,
      const std::string & status) {
        std::lock_guard<std::mutex> lock(tree_status_mutex);
        tree_status_events.push_back({node_name, previous, status});
      };
    return callbacks;
  }

  Coordinator make_coordinator()
  {
    return Coordinator(make_callbacks());
  }
};

void allow_normal_batch_recovery(Fixture & fixture, Coordinator::Callbacks & callbacks)
{
  callbacks.wait_for_recovery_cancellable = [&fixture](
    const Coordinator::CancelPredicateFn &, Coordinator::RecoveryObservation & recovery) {
      fixture.safety = normal_safety();
      recovery.safety = observation(fixture.safety);
      recovery.motion.execution_state = Coordinator::MotionStatus::MOTION_STATE_STOPPED;
      recovery.motion.backend_inactivity_confirmed = true;
      recovery.motion.holding_state = Coordinator::MotionStatus::HOLDING_RELEASED;
      return true;
    };
  callbacks.wait_for_batch_continuation = [](
    const Coordinator::CancelPredicateFn &) {return true;};
}

static std::shared_ptr<arm_cell_orchestration::CancellationRegistry::ActionSubmission>
register_test_submission(arm_cell_orchestration::CancellationRegistry & registry, const std::string & key)
{
  auto submission = std::make_shared<arm_cell_orchestration::CancellationRegistry::ActionSubmission>();
  return registry.register_action_submission(key, submission) ? submission : nullptr;
}

TEST(CycleCoordinatorTest, BehaviorTreeExposesIndependentMissionActionOrder)
{
  Fixture fixture;
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  std::lock_guard<std::mutex> lock(fixture.tree_status_mutex);
  const auto successful_node = [&fixture](const std::string & name) {
      const auto found = std::find_if(
        fixture.tree_status_events.begin(), fixture.tree_status_events.end(),
        [&name](const auto & event) {
          return event.node == name && event.current == "SUCCESS";
        });
      return found == fixture.tree_status_events.end() ?
             fixture.tree_status_events.size() :
             static_cast<std::size_t>(std::distance(fixture.tree_status_events.begin(), found));
    };
  const std::vector<std::string> expected{
    "ValidateRecipe", "InitialHome", "DetectTarget", "ExecutePnp"};
  std::size_t previous = 0U;
  for (const auto & name : expected) {
    const auto current = successful_node(name);
    EXPECT_LT(current, fixture.tree_status_events.size()) << name;
    EXPECT_GE(current, previous) << name;
    previous = current;
  }

  ASSERT_FALSE(fixture.tree_status_events.empty());
  for (const auto & event : fixture.tree_status_events) {
    EXPECT_NE(event.previous, event.current) << event.node;
  }
  const auto has_structured_transition = [&fixture](const std::string & name) {
      return std::any_of(
        fixture.diagnostic_records.begin(), fixture.diagnostic_records.end(),
        [&name](const auto & record) {
          return record.event == Coordinator::DiagnosticEvent::BT_NODE_STATUS &&
                 std::string(record.tree_node.data(), record.tree_node_size) == name &&
                 record.tree_previous_status != record.tree_current_status;
        });
    };
  EXPECT_TRUE(has_structured_transition("ExecutePnp"));
  EXPECT_FALSE(has_structured_transition("PICK"));
  EXPECT_FALSE(has_structured_transition("PLACE"));
  EXPECT_FALSE(has_structured_transition("NextIterationHome"));
  EXPECT_FALSE(has_structured_transition("CommitPlaced"));
  EXPECT_FALSE(has_structured_transition("BeginTargetEpisode"));
}

TEST(CycleCoordinatorTest, RepeatedRunningTicksDoNotRepeatNodeStatusEvents)
{
  Fixture fixture;
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  std::lock_guard<std::mutex> lock(fixture.tree_status_mutex);
  EXPECT_EQ(
    std::count_if(
      fixture.tree_status_events.begin(), fixture.tree_status_events.end(), [](const auto & event) {
        return event.node == "InitialHome" && event.current == "RUNNING";
      }),
    1);
  const auto initial_home = std::find_if(
    fixture.tree_status_events.begin(), fixture.tree_status_events.end(), [](const auto & event) {
      return event.node == "InitialHome" && event.current == "SUCCESS";
    });
  ASSERT_NE(initial_home, fixture.tree_status_events.end());
  EXPECT_EQ(initial_home->previous, "RUNNING");
}

TEST(CycleCoordinatorTest, CallerCancellationHaltsRunningVisionActionAndDropsItsLateResult)
{
  Fixture fixture;
  std::mutex gate_mutex;
  std::condition_variable gate_changed;
  bool vision_started = false;
  bool release_vision = false;
  std::atomic_bool vision_observed_cancel{false};
  fixture.vision = [](const DetectTarget::Request &) {return found_target();};
  auto callbacks = fixture.make_callbacks();
  callbacks.detect_target_cancellable = [
    &gate_mutex, &gate_changed, &vision_started, &release_vision, &vision_observed_cancel](
    const DetectTarget::Request &, const Coordinator::CancelPredicateFn & canceled) {
      std::unique_lock<std::mutex> lock(gate_mutex);
      vision_started = true;
      gate_changed.notify_all();
      gate_changed.wait(lock, [&release_vision]() {return release_vision;});
      lock.unlock();
      vision_observed_cancel.store(canceled());
      return found_target();
    };
  Coordinator coordinator(std::move(callbacks));
  std::optional<Coordinator::Result> result;
  std::thread execution([&]() {result = coordinator.execute("target_fixture");});
  bool entered_vision = false;
  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    entered_vision = gate_changed.wait_for(
      lock, std::chrono::seconds(1), [&vision_started]() {return vision_started;});
  }
  coordinator.set_cancel_requested(true);
  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    release_vision = true;
  }
  gate_changed.notify_all();
  execution.join();

  EXPECT_TRUE(entered_vision);
  EXPECT_TRUE(vision_observed_cancel.load());
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->exit_reason.value, MissionExitReason::MISSION_EXIT_CANCELED);
  EXPECT_EQ(
    std::count(
      fixture.motion_tasks.begin(), fixture.motion_tasks.end(), MotionTaskType::TASK_TYPE_PICK), 0);
}

TEST(CycleCoordinatorTest, CallerCancellationInterruptsRecoveryWaitWithoutChangingSafetyExit)
{
  Fixture fixture;
  std::mutex gate_mutex;
  std::condition_variable gate_changed;
  bool recovery_wait_started = false;
  bool release_recovery_wait = false;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
        fixture.safety = recovery_safety();
        return motion_result(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
      }
      return successful_motion_result(goal);
    };
  auto callbacks = fixture.make_callbacks();
  callbacks.wait_for_recovery_cancellable = [
    &gate_mutex, &gate_changed, &recovery_wait_started, &release_recovery_wait](
    const Coordinator::CancelPredicateFn & canceled,
    Coordinator::RecoveryObservation &) {
      std::unique_lock<std::mutex> lock(gate_mutex);
      recovery_wait_started = true;
      gate_changed.notify_all();
      gate_changed.wait(lock, [&release_recovery_wait]() {return release_recovery_wait;});
      return canceled();
    };
  Coordinator coordinator(std::move(callbacks));
  std::optional<Coordinator::Result> result;
  std::thread execution([&]() {result = coordinator.execute("target_fixture");});
  bool entered_recovery_wait = false;
  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    entered_recovery_wait = gate_changed.wait_for(
      lock, std::chrono::seconds(1), [&recovery_wait_started]() {return recovery_wait_started;});
  }
  coordinator.set_cancel_requested(true);
  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    release_recovery_wait = true;
  }
  gate_changed.notify_all();
  execution.join();

  EXPECT_TRUE(entered_recovery_wait);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(
    std::count(
      fixture.motion_tasks.begin(),
      fixture.motion_tasks.end(), MotionTaskType::TASK_TYPE_RETRACT), 0);
}

TEST(CycleCoordinatorTest, SignalLossRecoveryContinuesSameUnprocessedTargetEpisode)
{
  Fixture fixture;
  int vision_calls = 0;
  fixture.vision = [&vision_calls](const DetectTarget::Request &) {
      ++vision_calls;
      return found_target();
    };
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK &&
        std::count(
          fixture.motion_tasks.begin(), fixture.motion_tasks.end(),
          MotionTaskType::TASK_TYPE_PICK) == 1)
      {
        fixture.safety = recovery_safety();
        return motion_result(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto callbacks = fixture.make_callbacks();
  int recovery_observations = 0;
  callbacks.wait_for_recovery_cancellable = [&fixture, &recovery_observations](
    const Coordinator::CancelPredicateFn &, Coordinator::RecoveryObservation & recovery) {
      ++recovery_observations;
      recovery.safety =
        observation(recovery_observations == 1 ? normal_safety() : recovery_safety());
      if (recovery_observations == 1) {
        fixture.safety = normal_safety();
      } else {
        fixture.safety = recovery_safety();
      }
      recovery.motion.execution_state = Coordinator::MotionStatus::MOTION_STATE_STOPPED;
      recovery.motion.backend_inactivity_confirmed = true;
      recovery.motion.holding_state = Coordinator::MotionStatus::HOLDING_RELEASED;
      return recovery_observations == 1;
    };
  callbacks.wait_for_batch_continuation = [](
    const Coordinator::CancelPredicateFn &) {return true;};
  Coordinator coordinator(std::move(callbacks));

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(result.processed_count, 1U);
  EXPECT_EQ(vision_calls, 2);
  const std::vector<uint8_t> expected{
    MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
    MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
    MotionTaskType::TASK_TYPE_PLACE};
  EXPECT_EQ(fixture.motion_tasks, expected);
}

TEST(CycleCoordinatorTest, EmergencyResetResumesOnlyAfterNormalCapabilityReturns)
{
  Fixture fixture;
  int vision_calls = 0;
  fixture.vision = [&vision_calls](const DetectTarget::Request &) {
      auto target = found_target();
      target.acquisition_id = "estop-acquisition-" + std::to_string(++vision_calls);
      for (auto & candidate : target.material_candidates) {
        candidate.acquisition_id = target.acquisition_id;
      }

      return target;
    };
  int pick_calls = 0;
  fixture.motion = [&fixture, &pick_calls](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK && ++pick_calls == 2) {
        auto emergency = recovery_safety();
        emergency.motion_capability.value = MotionCapability::MOTION_NONE;
        emergency.selected_stop_mode.value =
          arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY;
        fixture.safety = emergency;
        return motion_result(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto callbacks = fixture.make_callbacks();
  callbacks.wait_for_recovery_cancellable = [&fixture](
    const Coordinator::CancelPredicateFn &, Coordinator::RecoveryObservation & recovery) {
      recovery.safety = observation(fixture.safety);
      recovery.motion.execution_state = Coordinator::MotionStatus::MOTION_STATE_STOPPED;
      recovery.motion.backend_inactivity_confirmed = true;
      recovery.motion.holding_state = Coordinator::MotionStatus::HOLDING_RELEASED;
      return true;
    };
  callbacks.wait_for_batch_continuation = [&fixture](
    const Coordinator::CancelPredicateFn &) {
      fixture.safety = normal_safety();  // Safety publishes NORMAL only after Clear/Reset.
      return true;
    };
  Coordinator coordinator(std::move(callbacks));

  const auto result = coordinator.execute("target_fixture", 2);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(result.processed_count, 2U);
  EXPECT_EQ(vision_calls, 3);
  const std::vector<uint8_t> expected{
    MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
    MotionTaskType::TASK_TYPE_PLACE, MotionTaskType::TASK_TYPE_PICK,
    MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
    MotionTaskType::TASK_TYPE_PLACE};
  EXPECT_EQ(fixture.motion_tasks, expected);
  ASSERT_EQ(fixture.motion_goals.size(), expected.size());
  ASSERT_FALSE(fixture.motion_goals[3].material_candidates.empty());
  ASSERT_FALSE(fixture.motion_goals[5].material_candidates.empty());
  EXPECT_EQ(
    fixture.motion_goals[3].material_candidates.front().acquisition_id,
    "estop-acquisition-2");
  EXPECT_EQ(
    fixture.motion_goals[5].material_candidates.front().acquisition_id,
    "estop-acquisition-3");
  EXPECT_TRUE(std::none_of(
    fixture.motion_tasks.begin(), fixture.motion_tasks.end(), [](uint8_t task) {
      return task == MotionTaskType::TASK_TYPE_RETRACT;
    }));
  const auto interruption = std::find_if(
    fixture.diagnostic_records.begin(), fixture.diagnostic_records.end(), [](const auto & record) {
      return record.event == Coordinator::DiagnosticEvent::CANCELLATION &&
             record.reason == Coordinator::DiagnosticReason::MISSION_INTERRUPTION;
    });
  ASSERT_NE(interruption, fixture.diagnostic_records.end());
  EXPECT_EQ(
    interruption->failure_code,
    MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
}

TEST(CycleCoordinatorTest, EmergencyResetReturnsHeldMaterialBeforeHome)
{
  Fixture fixture;
  int pick_calls = 0;
  fixture.motion = [&fixture, &pick_calls](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK && ++pick_calls == 2) {
        auto emergency = normal_safety();
        emergency.safety_state = SafetyState::SAFETY_STATE_EMERGENCY_STOPPED;
        emergency.motion_capability.value = MotionCapability::MOTION_NONE;
        emergency.selected_stop_mode.value =
          arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY;
        emergency.active_causes.value = arm_cell_interfaces::msg::SafetyCauseSet::E_STOP;
        emergency.latched_causes.value = arm_cell_interfaces::msg::SafetyCauseSet::E_STOP;
        fixture.safety = emergency;
        return motion_result(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
      }
      return successful_motion_result(goal);
    };
  auto callbacks = fixture.make_callbacks();
  int recovery_observations = 0;
  callbacks.wait_for_recovery_cancellable = [&fixture, &recovery_observations](
    const Coordinator::CancelPredicateFn &, Coordinator::RecoveryObservation & recovery) {
      ++recovery_observations;
      if (recovery_observations > 1) {fixture.safety = normal_safety();}
      recovery.safety = observation(fixture.safety);
      recovery.motion.execution_state = Coordinator::MotionStatus::MOTION_STATE_STOPPED;
      recovery.motion.backend_inactivity_confirmed = true;
      recovery.motion.holding_state = Coordinator::MotionStatus::HOLDING_HELD;
      return true;
    };
  callbacks.wait_for_batch_continuation = [&fixture](
    const Coordinator::CancelPredicateFn &) {
      fixture.safety = normal_safety();
      return true;
    };

  Coordinator coordinator(std::move(callbacks));
  const auto result = coordinator.execute("target_fixture", 2);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  const auto return_it = std::find(
    fixture.motion_tasks.begin(), fixture.motion_tasks.end(),
    MotionTaskType::TASK_TYPE_RETURN_HELD);
  ASSERT_NE(return_it, fixture.motion_tasks.end());
  const auto home_it = std::find(
    return_it, fixture.motion_tasks.end(), MotionTaskType::TASK_TYPE_GO_HOME);
  ASSERT_NE(home_it, fixture.motion_tasks.end());
  EXPECT_LT(
    std::distance(fixture.motion_tasks.begin(), return_it),
    std::distance(fixture.motion_tasks.begin(), home_it));
  EXPECT_EQ(
    std::count(
      fixture.motion_tasks.begin(), fixture.motion_tasks.end(),
      MotionTaskType::TASK_TYPE_RETURN_HELD),
    1);
  EXPECT_TRUE(
    std::none_of(
      fixture.motion_tasks.begin(), fixture.motion_tasks.end(), [](uint8_t task) {
        return task == MotionTaskType::TASK_TYPE_RETRACT;
      }));
}

TEST(CycleCoordinatorTest, InitialHomePreemptionRepeatsHomeBeforeVision)
{
  Fixture fixture;
  bool interrupted_home = false;
  fixture.motion = [&fixture, &interrupted_home](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME && !interrupted_home) {
        interrupted_home = true;
        fixture.safety = recovery_safety();
        return motion_result(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto callbacks = fixture.make_callbacks();
  allow_normal_batch_recovery(fixture, callbacks);
  Coordinator coordinator(std::move(callbacks));

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(result.processed_count, 1U);
  const std::vector<uint8_t> expected{
    MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_GO_HOME,
    MotionTaskType::TASK_TYPE_PICK, MotionTaskType::TASK_TYPE_PLACE};
  EXPECT_EQ(fixture.motion_tasks, expected);
}

TEST(CycleCoordinatorTest, PlacePreemptionRetriesUnprocessedTargetWithNewAcquisition)
{
  Fixture fixture;
  int vision_calls = 0;
  fixture.vision = [&vision_calls](const DetectTarget::Request &) {
      auto target = found_target();
      target.acquisition_id = "acquisition-" + std::to_string(++vision_calls);
      for (auto & candidate : target.material_candidates) {
        candidate.acquisition_id = target.acquisition_id;
      }

      return target;
    };
  bool interrupted_place = false;
  fixture.motion = [&fixture, &interrupted_place](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE && !interrupted_place) {
        interrupted_place = true;
        fixture.safety = recovery_safety();
        return motion_result(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto callbacks = fixture.make_callbacks();
  allow_normal_batch_recovery(fixture, callbacks);
  Coordinator coordinator(std::move(callbacks));

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(result.processed_count, 1U);
  EXPECT_EQ(vision_calls, 2);
  const std::vector<uint8_t> expected{
    MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
    MotionTaskType::TASK_TYPE_PLACE, MotionTaskType::TASK_TYPE_GO_HOME,
    MotionTaskType::TASK_TYPE_PICK, MotionTaskType::TASK_TYPE_PLACE};
  EXPECT_EQ(fixture.motion_tasks, expected);
  ASSERT_EQ(fixture.motion_goals.size(), 6U);
  ASSERT_FALSE(fixture.motion_goals[1].material_candidates.empty());
  ASSERT_FALSE(fixture.motion_goals[4].material_candidates.empty());
  EXPECT_NE(fixture.motion_goals[1].material_candidates.front().acquisition_id,
    fixture.motion_goals[4].material_candidates.front().acquisition_id);
}

TEST(CycleCoordinatorTest, SuccessfulPnpNeedsNoMissionFinalHome)
{
  Fixture fixture;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto callbacks = fixture.make_callbacks();
  allow_normal_batch_recovery(fixture, callbacks);
  Coordinator coordinator(std::move(callbacks));

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(result.processed_count, 1U);
  const std::vector<uint8_t> expected{
    MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
    MotionTaskType::TASK_TYPE_PLACE};
  EXPECT_EQ(fixture.motion_tasks, expected);
}

TEST(CycleCoordinatorTest, RetryDecisionEvidenceDoesNotChangeSuccessfulMissionResult)
{
  Fixture fixture;
  int vision_calls = 0;
  fixture.vision = [&vision_calls](const DetectTarget::Request &) {
      ++vision_calls;
      return vision_calls == 1 ? vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT) :
             found_target();
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture", 1U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  const auto retry = std::find_if(
    fixture.diagnostic_records.begin(), fixture.diagnostic_records.end(), [](const auto & record) {
      return record.event == Coordinator::DiagnosticEvent::RETRY;
    });
  ASSERT_NE(retry, fixture.diagnostic_records.end());
  const auto & record = *retry;
  EXPECT_EQ(record.event, Coordinator::DiagnosticEvent::RETRY);
  EXPECT_EQ(record.phase, Coordinator::DiagnosticPhase::VISION);
  EXPECT_EQ(record.failure_domain, Coordinator::DiagnosticFailureDomain::VISION);
  EXPECT_EQ(
    record.failure_code,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
  EXPECT_EQ(record.attempt, 1U);
  EXPECT_EQ(record.retry_budget, 3U);
  EXPECT_EQ(record.decision, Coordinator::DiagnosticDecision::RETRY);
}

TEST(CycleCoordinatorTest, DiagnosticDisabledQueueFullAndStorageFailurePreserveMissionResult)
{
  const auto timeout = [](const DetectTarget::Request &) {
      return vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
    };
  Fixture diagnostics_off;
  diagnostics_off.vision = timeout;
  diagnostics_off.diagnostics_enabled = false;
  Fixture queue_full;
  queue_full.vision = timeout;
  queue_full.diagnostic_sink_mode = Fixture::DiagnosticSinkMode::QUEUE_FULL;
  Fixture storage_failure;
  storage_failure.vision = timeout;
  storage_failure.diagnostic_sink_mode = Fixture::DiagnosticSinkMode::STORAGE_FAILURE;

  auto ordinary_run = diagnostics_off.make_coordinator();
  auto full_queue_run = queue_full.make_coordinator();
  auto failed_storage_run = storage_failure.make_coordinator();

  const auto ordinary_result = ordinary_run.execute("target_fixture");
  const auto queue_result = full_queue_run.execute("target_fixture");
  const auto storage_result = failed_storage_run.execute("target_fixture");

  EXPECT_EQ(queue_result.exit_reason.value, ordinary_result.exit_reason.value);
  EXPECT_EQ(queue_result.diagnostic_detail, ordinary_result.diagnostic_detail);
  EXPECT_EQ(queue_result.processed_count, ordinary_result.processed_count);
  EXPECT_EQ(queue_full.motion_tasks, diagnostics_off.motion_tasks);
  EXPECT_EQ(storage_result.exit_reason.value, ordinary_result.exit_reason.value);
  EXPECT_EQ(storage_result.diagnostic_detail, ordinary_result.diagnostic_detail);
  EXPECT_EQ(storage_result.processed_count, ordinary_result.processed_count);
  EXPECT_EQ(storage_failure.motion_tasks, diagnostics_off.motion_tasks);
  EXPECT_EQ(ordinary_result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
}

TEST(CycleCoordinatorTest, ExpiredSafetyPermissionBlocksMotionWithoutSafetyPreemption)
{
  Fixture fixture;
  fixture.safety_observation_hook = []() {
      const auto now = SafetyObservation::Clock::now();
      return arm_cell_orchestration::evaluate_safety_observation(
        normal_safety(), true, now - std::chrono::seconds(1), now,
        std::chrono::milliseconds(500));
    };

  auto coordinator = fixture.make_coordinator();
  const auto result = coordinator.execute("target_fixture", 1U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  EXPECT_TRUE(fixture.motion_tasks.empty());
}


TEST(VisionRequestTimeout, UsesTheDurationSuppliedToDetectTarget)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = 0;
  duration.nanosec = 500'000'000U;

  const auto timeout = arm_cell_orchestration::vision_request_timeout(duration);
  ASSERT_TRUE(timeout);
  EXPECT_EQ(*timeout, std::chrono::milliseconds(500));
  EXPECT_EQ(
    arm_cell_orchestration::vision_response_wait_timeout(*timeout),
    std::chrono::milliseconds(600));
}

TEST(CycleCoordinatorTest, FailedPlaceWithPlacedDispositionCountsOnceAndStops)
{
  Fixture fixture;
  int place_calls = 0;
  fixture.motion = [&fixture, &place_calls](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE && ++place_calls == 1) {
        auto result = motion_result(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
        result.object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_PLACED;
        return result;
      }
      return successful_motion_result(goal);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  EXPECT_EQ(result.processed_count, 1U);
  EXPECT_EQ(place_calls, 1);
  EXPECT_EQ(
    fixture.motion_tasks, (std::vector<uint8_t>{
    MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
    MotionTaskType::TASK_TYPE_PLACE}));
}

TEST(CycleCoordinatorTest, PrefetchHintDoesNotStartAcquisitionBeforePnpTerminal)
{
  Fixture fixture;
  std::mutex gate_mutex;
  std::condition_variable gate_changed;
  int vision_calls = 0;
  int pnp_calls = 0;
  bool pnp_entered = false;
  bool allow_terminal = false;
  std::vector<std::string> events;
  auto callbacks = fixture.make_callbacks();
  callbacks.detect_target = [&vision_calls, &events, &gate_mutex](
    const DetectTarget::Request &) {
      std::lock_guard<std::mutex> lock(gate_mutex);
      ++vision_calls;
      events.push_back("detect");
      return found_target();
    };
  callbacks.publish_processed_count = [&events, &gate_mutex](uint64_t) {
      std::lock_guard<std::mutex> lock(gate_mutex);
      events.push_back("accounted");
    };
  callbacks.execute_pnp_with_feedback = [&fixture, &gate_mutex, &gate_changed,
    &pnp_calls, &pnp_entered, &allow_terminal, &events](
    const Fixture::PnpAction::Goal & goal, const Coordinator::PnpFeedbackFn & feedback) {
      fixture.pnp_goals.push_back(goal);
      ++pnp_calls;
      if (feedback) {
        feedback(Fixture::PnpAction::Feedback::NEXT_ACQUISITION_HINT_PREFETCH_NEXT);
      }
      std::unique_lock<std::mutex> lock(gate_mutex);
      pnp_entered = true;
      gate_changed.notify_all();
      gate_changed.wait(lock, [&allow_terminal]() {return allow_terminal;});
      events.push_back("pnp_terminal");
      Coordinator::PnpResult result;
      result.goal_uuid = "ordered-pnp-" + std::to_string(pnp_calls);
      result.result_code_available = true;
      result.material_placed = true;
      result.outcome = Fixture::PnpAction::Result::PNP_OUTCOME_COMPLETED;
      result.action_terminal = "SUCCEEDED";
      return result;
    };
  callbacks.execute_task = [&fixture, &events, &gate_mutex](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      std::lock_guard<std::mutex> lock(gate_mutex);
      events.push_back("home");
      return successful_motion_result(goal);
    };
  callbacks.execute_task_with_feedback = {};
  Coordinator coordinator(std::move(callbacks));
  std::optional<Coordinator::Result> result;
  std::thread execution([&]() {result = coordinator.execute("target_fixture", 2U);});
  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    const bool entered = gate_changed.wait_for(
      lock, std::chrono::seconds(1), [&pnp_entered]() {return pnp_entered;});
    EXPECT_EQ(vision_calls, 1);
    allow_terminal = true;
    EXPECT_TRUE(entered);
  }
  gate_changed.notify_all();
  execution.join();

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(vision_calls, 2);
  EXPECT_EQ(pnp_calls, 2);
  EXPECT_EQ(
    events, (std::vector<std::string>{
    "home", "detect", "pnp_terminal", "accounted", "detect", "pnp_terminal",
    "accounted"}));
}

TEST(CycleCoordinatorTest, SafetyIsRevalidatedAfterReleaseBeforeNextPick)
{
  Fixture fixture;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE) {
        fixture.safety = recovery_safety();
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture", 2U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(result.processed_count, 1U);
  const std::vector<uint8_t> expected{
    MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
    MotionTaskType::TASK_TYPE_PLACE};
  EXPECT_EQ(fixture.motion_tasks, expected);
}

Coordinator::Result run_motion_action_failure_episode(
  Fixture & fixture, const bool safety_preempts, std::vector<std::string> & terminal_records)
{
  fixture.motion = [&fixture, safety_preempts](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value != MotionTaskType::TASK_TYPE_PICK) {
        return successful_motion_result(goal);
      }
      auto result = motion_result(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
      result.diagnostic_detail = "backend unavailable";
      result.motion_execution_id = kSyntheticMotionExecutionId;
      if (safety_preempts) {
        fixture.safety = recovery_safety();
      }
      return result;
    };
  auto coordinator = fixture.make_coordinator();
  auto result = coordinator.execute("target");
  if (result.motion_failure) {
    const auto & failure = *result.motion_failure;
    arm_cell_orchestration::emit_terminal_failure_record(
      arm_cell_orchestration::TerminalFailureRecord{
        "execute-cycle-uuid", "delivery-uuid", "target-profile",
        failure.motion_execution_id, failure.task_type, failure.code.value,
        result.exit_reason.value, failure.diagnostic_detail},
      [&terminal_records](const std::string & record) {terminal_records.push_back(record);});
  }
  return result;
}

TEST(CycleCoordinatorTest, ForwardsCompleteVisionCandidatesAndRecipeInOnePnpGoal)
{
  Fixture fixture;
  auto response = found_target();
  auto second = response.material_candidates.front(); second.candidate_id = "second";
  second.target_pose.pose.position.z += 0.02;
  second.orientation_symmetry_order = 0;
  second.target_pose.pose.orientation.w = 0.0;
  response.material_candidates.push_back(second);
  fixture.vision = [&](const auto &) {return response;};
  Coordinator coordinator(fixture.make_callbacks());
  const auto result = coordinator.execute("target_fixture", 1);
  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  ASSERT_EQ(fixture.pnp_goals.size(), 1U);
  EXPECT_TRUE(fixture.pnp_goals.front().has_unobserved_yaw_fallback);
  const auto & goal = fixture.pnp_goals.front();
  EXPECT_EQ(goal.material_candidates, response.material_candidates);
  EXPECT_EQ(goal.vision_acquisition_id, response.acquisition_id);
  EXPECT_TRUE(goal.has_place_target_pose);
  EXPECT_DOUBLE_EQ(goal.place_target_pose.pose.position.x, 0.8);
  EXPECT_DOUBLE_EQ(goal.place_target_pose.pose.position.y, -0.2);
  EXPECT_EQ(goal.pick_approach_distance_m, fixture.recipe.pick_approach_distance_m);
  EXPECT_EQ(goal.place_approach_distance_m, fixture.recipe.place_approach_distance_m);
}

TEST(CycleCoordinatorTest, ForwardsUnifiedMixedGradeListWithoutScalarSelection)
{
  Fixture fixture;
  auto response = found_target();
  auto a = response.material_candidates.front();
  a.candidate_id = "A";
  a.target_pose.pose.position.y = 0.2;
  auto b = a;
  b.candidate_id = "B";
  b.target_pose.pose.position.y = 0.3;
  auto c = a;
  c.candidate_id = "C";
  c.target_pose.pose.position.y = 0.1;
  // A/C originate as HIGH and B as MEDIUM; confidence is not a wire field.
  response.material_candidates = {a, b, c};
  response.disturbance_candidates = disturbance_target().disturbance_candidates;
  fixture.vision = [&](const auto &) {return response;};
  auto coordinator = fixture.make_coordinator();
  const auto result = coordinator.execute("target_fixture", 3U);
  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  ASSERT_EQ(fixture.pnp_goals.size(), 3U);
  for (const auto & goal : fixture.pnp_goals) {
    EXPECT_EQ(goal.material_candidates, response.material_candidates);
    EXPECT_EQ(goal.vision_acquisition_id, response.acquisition_id);
  }
  EXPECT_TRUE(fixture.disturb_goals.empty());
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U, 0U, 0U}));
}

TEST(CycleCoordinatorTest, RejectsInvalidUnifiedCandidateIdentityAndGeometry)
{
  for (const std::string invalid : {"acquisition", "duplicate", "pose", "geometry", "empty"}) {
    SCOPED_TRACE(invalid);
    Fixture fixture;
    auto response = found_target();
    auto second = response.material_candidates.front();
    second.candidate_id = "second";
    response.material_candidates.push_back(second);
    auto & candidate = response.material_candidates.back();
    if (invalid == "acquisition") {candidate.acquisition_id = "other-acquisition";}
    if (invalid == "duplicate") {
      candidate.candidate_id = response.material_candidates.front().candidate_id;
    }
    if (invalid == "pose") {candidate.target_pose.pose.position.y = NAN;}
    if (invalid == "geometry") {candidate.estimated_height_m = -1.0F;}
    if (invalid == "empty") {response.material_candidates.clear();}
    fixture.vision = [&](const auto &) {return response;};
    auto coordinator = fixture.make_coordinator();
    const auto result = coordinator.execute("target_fixture", 1U);
    EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
    EXPECT_TRUE(fixture.pnp_goals.empty());
    EXPECT_EQ(result.processed_count, 0U);
  }
}

TEST(CycleCoordinatorTest, UnifiedCandidatesRetryPreservesObservationRetryBudget)
{
  Fixture fixture;
  int acquisitions = 0;
  int pnp_calls = 0;
  auto callbacks = fixture.make_callbacks();
  callbacks.detect_target = [&](const auto &) {
      ++acquisitions;
      if (acquisitions == 1 || acquisitions >= 4) {
        auto response = vision_result(acquisitions == 1 ?
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT :
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
        response.diagnostic_detail = acquisitions == 1 ? "first timeout" : "later invalidity";
        return response;
      }
      auto response = found_target();
      response.acquisition_id = "pnp-acquisition-" + std::to_string(acquisitions);
      for (auto & candidate : response.material_candidates) {
        candidate.acquisition_id = response.acquisition_id;
      }

      auto medium = response.material_candidates.front();
      medium.candidate_id = "second-candidate";
      response.material_candidates.push_back(medium);
      return response;
    };
  callbacks.execute_pnp_with_feedback = [&](const Coordinator::ExecutePnp::Goal & goal, const auto &) {
      fixture.pnp_goals.push_back(goal);
      ++pnp_calls;
      Coordinator::PnpResult result;
      result.outcome = Coordinator::ExecutePnp::Result::PNP_OUTCOME_RETRYABLE;
      result.action_terminal = "ABORTED";
      return result;
    };
  Coordinator coordinator(callbacks);
  const auto result = coordinator.execute("target_fixture", 1U);
  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
  EXPECT_EQ(result.processed_count, 0U);
  EXPECT_EQ(result.diagnostic_detail, "first timeout");
  EXPECT_EQ(acquisitions, 6);
  EXPECT_EQ(pnp_calls, 2);
  ASSERT_EQ(fixture.pnp_goals.size(), 2U);
  EXPECT_EQ(fixture.pnp_goals[0].vision_acquisition_id, "pnp-acquisition-2");
  EXPECT_EQ(fixture.pnp_goals[1].vision_acquisition_id, "pnp-acquisition-3");
  for (const auto & goal : fixture.pnp_goals) {
    ASSERT_EQ(goal.material_candidates.size(), 2U);
    EXPECT_EQ(goal.material_candidates.front().candidate_id, "candidate-test");
    EXPECT_EQ(goal.material_candidates[1].candidate_id, "second-candidate");
  }
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U, 1U, 2U, 3U}));
}

TEST(CycleCoordinatorTest, TemporaryInvalidTargetWithDisturbanceEvidenceOnlyRetriesVision)
{
  Fixture fixture;
  int acquisitions = 0;
  fixture.vision = [&](const auto &) {
      ++acquisitions;
      auto response = disturbance_target();
      response.result_code.value =
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET;
      return response;
    };
  auto coordinator = fixture.make_coordinator();
  const auto result = coordinator.execute("target_fixture", 3U);
  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
  EXPECT_EQ(acquisitions, 4);
  EXPECT_TRUE(fixture.disturb_goals.empty());
  EXPECT_TRUE(fixture.pnp_goals.empty());
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U, 1U, 2U, 3U}));
}

TEST(CycleCoordinatorTest, AccountsARepeatedPnpGoalUuidOnlyOnce)
{
  Fixture fixture;
  int pnp_calls = 0;
  int acquisitions = 0;
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      auto response = found_target();
      response.acquisition_id = "duplicate-goal-test-acquisition-" +
        std::to_string(++acquisitions);
      for (auto & candidate : response.material_candidates) {
        candidate.acquisition_id = response.acquisition_id;
      }
      response.material_candidates.front().candidate_id = "duplicate-goal-test-candidate-" +
        std::to_string(acquisitions);
      for (std::size_t index = 0; index < response.material_candidates.size(); ++index) {
        response.material_candidates[index].acquisition_id = response.acquisition_id;
        response.material_candidates[index].candidate_id = response.material_candidates.front().candidate_id + "-" +
          std::to_string(index);
      }

      return response;
    };
  auto callbacks = fixture.make_callbacks();
  callbacks.execute_pnp_with_feedback = [&fixture, &pnp_calls](
      const Fixture::PnpAction::Goal & goal, const Coordinator::PnpFeedbackFn &) {
      fixture.pnp_goals.push_back(goal);
      Coordinator::PnpResult result;
      result.goal_uuid = "replayed-pnp-goal";
      result.result_code_available = true;
      result.material_placed = true;
      result.detail_code.value = ++pnp_calls == 1 ? MotionTaskResultCode::TASK_RESULT_SUCCESS :
        MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
      result.outcome = pnp_calls == 1 ?
        Fixture::PnpAction::Result::PNP_OUTCOME_COMPLETED :
        Fixture::PnpAction::Result::PNP_OUTCOME_FAILED;
      result.action_terminal = pnp_calls == 1 ? "SUCCEEDED" : "ABORTED";
      return result;
    };
  Coordinator coordinator(std::move(callbacks));

  const auto result = coordinator.execute("target_fixture", 2U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  EXPECT_EQ(result.processed_count, 1U);
  EXPECT_EQ(pnp_calls, 2);
  EXPECT_EQ(acquisitions, 2);
  ASSERT_EQ(fixture.pnp_goals.size(), 2U);
  EXPECT_NE(
    fixture.pnp_goals[0].vision_acquisition_id,
    fixture.pnp_goals[1].vision_acquisition_id);
}

TEST(
  CycleCoordinatorTest,
  RepeatedPickFailuresReenterWithFreshAcquisitionWithoutConsumingVisionRetry)
{
  Fixture fixture;
  auto response = found_target();
  auto second = response.material_candidates.front();
  second.candidate_id = "second-candidate";
  second.target_pose.pose.position.x += 0.1;
  response.material_candidates.front().candidate_id = "first-candidate";
  response.material_candidates = {response.material_candidates.front(), second};
  int acquisitions = 0;
  int picks = 0;
  int homes = 0;
  auto callbacks = fixture.make_callbacks();
  callbacks.detect_target = [&](const auto &) {
      ++acquisitions;
      auto detected = response;
      detected.acquisition_id = "reentry-acquisition-" + std::to_string(acquisitions);
      for (auto & candidate : detected.material_candidates) {
        candidate.acquisition_id = detected.acquisition_id;
      }
      detected.material_candidates.front().candidate_id = "reentry-candidate-" + std::to_string(acquisitions);
      for (std::size_t index = 0; index < detected.material_candidates.size(); ++index) {
        detected.material_candidates[index].acquisition_id = detected.acquisition_id;
        detected.material_candidates[index].candidate_id = detected.material_candidates.front().candidate_id + "-" +
          std::to_string(index);
      }

      return detected;
    };
  callbacks.read_motion_status = [](bool) {
      Coordinator::MotionStatus status;
      status.holding_state = Coordinator::MotionStatus::HOLDING_RELEASED;
      status.backend_inactivity_confirmed = true;
      return std::optional<Coordinator::MotionStatus>{status};
    };
  fixture.motion = [&](const auto & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME) {++homes;}
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
        ++picks;
        if (picks <= 4) {
          auto result = pick_work_failure(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
          result.diagnostic_detail = "candidate search exhausted";
          return result;
        }
      }
      return successful_motion_result(goal);
    };

  Coordinator coordinator(callbacks);
  const auto result = coordinator.execute("target_fixture", 1);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(picks, 5);
  EXPECT_EQ(acquisitions, 5);
  ASSERT_EQ(fixture.pnp_goals.size(), 5U);
  ASSERT_EQ(fixture.pnp_goal_uuids.size(), 5U);
  for (std::size_t index = 1; index < 5U; ++index) {
    EXPECT_NE(fixture.pnp_goal_uuids[index - 1U], fixture.pnp_goal_uuids[index]);
    EXPECT_NE(
      fixture.pnp_goals[index - 1U].vision_acquisition_id,
      fixture.pnp_goals[index].vision_acquisition_id);
  }
  EXPECT_EQ(homes, 5);
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
  ASSERT_EQ(fixture.motion_goals.size(), 11U);
  for (std::size_t attempt = 0; attempt < 5U; ++attempt) {
    const auto pick_index = 1U + attempt * 2U;
    EXPECT_EQ(fixture.motion_goals[pick_index].task_type.value, MotionTaskType::TASK_TYPE_PICK);
    EXPECT_EQ(fixture.motion_goals[pick_index].material_candidates.front().acquisition_id,
      "reentry-acquisition-" + std::to_string(attempt + 1U));
  }
}

TEST(CycleCoordinatorTest, DisturbanceRepeatsWithoutLimitThenReturnsToNormalPnp)
{
  Fixture fixture;
  std::uint32_t acquisitions = 0U;
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      ++acquisitions;
      if (acquisitions <= 6U) {
        auto response = disturbance_target();
        response.acquisition_id = "disturbance-acquisition-" + std::to_string(acquisitions);
        for (auto & candidate : response.material_candidates) {
          candidate.acquisition_id = response.acquisition_id;
        }
        response.scene_generation = acquisitions;
        if (acquisitions % 2U == 0U) {
          response.disturbance_candidates.front().category =
            arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_INCLINED_SURFACE;
        }
        return response;
      }
      auto response = found_target();
      response.acquisition_id = "normal-pnp-acquisition";
      for (auto & candidate : response.material_candidates) {
        candidate.acquisition_id = response.acquisition_id;
      }

      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture", 3U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(result.processed_count, 3U);
  EXPECT_EQ(acquisitions, 9U);
  ASSERT_EQ(fixture.disturb_goals.size(), 6U);
  for (std::size_t index = 0; index < fixture.disturb_goals.size(); ++index) {
    const auto & goal = fixture.disturb_goals[index];
    EXPECT_EQ(goal.acquisition_id, "disturbance-acquisition-" + std::to_string(index + 1U));
    EXPECT_EQ(goal.scene_generation, index + 1U);
    EXPECT_EQ(goal.disturbance_candidates.front().category, (index + 1U) % 2U == 0U ?
      arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_INCLINED_SURFACE :
      arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_OTHER_UNRESOLVED);
    ASSERT_EQ(goal.disturbance_candidates.size(), 1U);
    EXPECT_EQ(goal.disturbance_candidates.front().candidate_id, "unresolved-region");
    EXPECT_DOUBLE_EQ(goal.disturbance_candidates.front().center.pose.position.x, 0.31);
  }
  ASSERT_EQ(fixture.pnp_goals.size(), 3U);
  EXPECT_EQ(fixture.pnp_goals.front().vision_acquisition_id, "normal-pnp-acquisition");
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U, 0U, 0U}));
}

TEST(CycleCoordinatorTest, DisturbanceRequiresThreeRemainingAndValidCandidates)
{
  {
    Fixture fixture;
    fixture.vision = [](const DetectTarget::Request &) {return disturbance_target();};
    auto coordinator = fixture.make_coordinator();
    const auto result = coordinator.execute("target_fixture", 2U);
    EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
    EXPECT_TRUE(fixture.disturb_goals.empty());
    EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
  }
  {
    Fixture fixture;
    fixture.vision = [](const DetectTarget::Request &) {
        auto response = disturbance_target();
        response.disturbance_candidates.clear();
        return response;
      };
    auto coordinator = fixture.make_coordinator();
    const auto result = coordinator.execute("target_fixture", 3U);
    EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
    EXPECT_TRUE(fixture.disturb_goals.empty());
  }
  {
    Fixture fixture;
    fixture.vision = [](const DetectTarget::Request &) {
        auto response = disturbance_target();
        response.result_code.value =
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SENSOR_ERROR;
        return response;
      };
    auto coordinator = fixture.make_coordinator();
    const auto result = coordinator.execute("target_fixture", 3U);
    EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
    EXPECT_TRUE(fixture.disturb_goals.empty());
  }
}

TEST(CycleCoordinatorTest, NormalPickCandidatesTakePriorityOverDisturbanceCandidates)
{
  Fixture fixture;
  fixture.vision = [](const DetectTarget::Request &) {
      auto response = found_target();
      response.disturbance_candidates = disturbance_target().disturbance_candidates;
      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture", 1U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_TRUE(fixture.disturb_goals.empty());
  ASSERT_EQ(fixture.pnp_goals.size(), 1U);
}

TEST(CycleCoordinatorTest, ObjectNotFoundRemainsQuantityShortfall)
{
  Fixture fixture;
  fixture.vision = [](const DetectTarget::Request &) {
      auto response = vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
      response.disturbance_candidates = disturbance_target().disturbance_candidates;
      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture", 3U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_SHORTFALL);
  EXPECT_TRUE(fixture.disturb_goals.empty());
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
}

TEST(CycleCoordinatorTest, DisturbanceCancellationAndSafetyPreemptionUseMissionTerminals)
{
  {
    Fixture fixture;
    Coordinator * coordinator_ptr = nullptr;
    fixture.vision = [](const DetectTarget::Request &) {return disturbance_target();};
    fixture.disturb = [&fixture, &coordinator_ptr](const ExecuteDisturb::Goal & goal) {
        fixture.disturb_goals.push_back(goal);
        coordinator_ptr->set_cancel_requested(true);
        return Coordinator::DisturbResult{
        arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_CANCELED, "canceled", "CANCELED"};
      };
    auto coordinator = fixture.make_coordinator();
    coordinator_ptr = &coordinator;
    const auto result = coordinator.execute("target_fixture", 3U);
    EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CANCELED);
    EXPECT_EQ(fixture.disturb_goals.size(), 1U);
    EXPECT_TRUE(fixture.pnp_goals.empty());
  }
  {
    Fixture fixture;
    fixture.vision = [](const DetectTarget::Request &) {return disturbance_target();};
    fixture.disturb = [&fixture](const ExecuteDisturb::Goal & goal) {
        fixture.disturb_goals.push_back(goal);
        fixture.safety = recovery_safety();
        return Coordinator::DisturbResult{
        arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_SAFETY_PREEMPTED,
        "safety preempted", "ABORTED"};
      };
    auto coordinator = fixture.make_coordinator();
    const auto result = coordinator.execute("target_fixture", 3U);
    EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
    EXPECT_EQ(fixture.disturb_goals.size(), 1U);
    EXPECT_TRUE(fixture.pnp_goals.empty());
  }
}

TEST(CycleCoordinatorTest, DisturbanceDoesNotConsumeOrResetVisionRetryBudget)
{
  Fixture fixture;
  std::uint32_t acquisitions = 0U;
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      ++acquisitions;
      if (acquisitions == 1U) {
        return vision_result(
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
      }
      if (acquisitions <= 3U) {
        auto response = disturbance_target();
        response.acquisition_id = "retry-disturbance-" + std::to_string(acquisitions);
        for (auto & candidate : response.material_candidates) {
          candidate.acquisition_id = response.acquisition_id;
        }
        return response;
      }
      auto response = found_target();
      response.acquisition_id = "retry-recovery-pnp-" + std::to_string(acquisitions);
      for (auto & candidate : response.material_candidates) {
        candidate.acquisition_id = response.acquisition_id;
      }

      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture", 3U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  ASSERT_EQ(fixture.disturb_goals.size(), 2U);
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U, 1U, 0U, 0U}));
}

TEST(CycleCoordinatorTest, FailedDisturbanceTerminatesWithoutRepeatingTheGoal)
{
  Fixture fixture;
  fixture.vision = [](const DetectTarget::Request &) {return disturbance_target();};
  fixture.disturb = [&fixture](const ExecuteDisturb::Goal & goal) {
      fixture.disturb_goals.push_back(goal);
      return Coordinator::DisturbResult{
      arm_cell_interfaces::msg::DisturbanceOutcome::OUTCOME_FAILED,
      "disturb failed", "ABORTED"};
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture", 3U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  EXPECT_EQ(result.processed_count, 0U);
  ASSERT_EQ(fixture.disturb_goals.size(), 1U);
  EXPECT_TRUE(fixture.pnp_goals.empty());
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
}

TEST(CycleCoordinatorTest, DisturbanceReacquisitionDoesNotGrantAnExtraVisionRetry)
{
  Fixture fixture;
  std::uint32_t acquisitions = 0U;
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      if (++acquisitions == 1U) {return disturbance_target();}
      auto response = vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SENSOR_ERROR);
      response.diagnostic_detail = "sensor unavailable";
      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture", 3U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
  EXPECT_EQ(acquisitions, 2U);
  ASSERT_EQ(fixture.disturb_goals.size(), 1U);
  EXPECT_TRUE(fixture.pnp_goals.empty());
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
}

TEST(CycleCoordinatorTest, DisturbanceAndVisionErrorsShareRetryBudgetWithoutResettingIt)
{
  Fixture fixture;
  std::uint32_t acquisitions = 0U;
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      ++acquisitions;
      if (acquisitions == 1U || acquisitions == 3U || acquisitions == 5U ||
        acquisitions == 7U)
      {
        auto response = disturbance_target();
        response.acquisition_id = "interleaved-disturbance-" + std::to_string(acquisitions);
        for (auto & candidate : response.material_candidates) {
          candidate.acquisition_id = response.acquisition_id;
        }
        return response;
      }
      auto response = vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
      response.diagnostic_detail = "transient sensor failure";
      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture", 3U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
  EXPECT_EQ(acquisitions, 8U);
  EXPECT_EQ(fixture.disturb_goals.size(), 4U);
  EXPECT_TRUE(fixture.pnp_goals.empty());
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U, 1U, 2U, 3U}));
}

}  // namespace

TEST(CycleCoordinatorTest, MotionActionBackendFailureHasOneReconstructableEpisode)
{
  Fixture fixture;
  std::vector<std::string> records;

  const auto result = run_motion_action_failure_episode(fixture, false, records);

  ASSERT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_EQ(
    result.motion_failure->code.value,
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(result.motion_failure->diagnostic_detail, "backend unavailable");
  EXPECT_EQ(result.motion_failure->motion_execution_id, "fixture-pnp-1");
  ASSERT_EQ(fixture.motion_tasks.size(), 2U);
  EXPECT_EQ(fixture.motion_tasks.back(), MotionTaskType::TASK_TYPE_PICK);
  ASSERT_EQ(records.size(), 1U);
  EXPECT_NE(records.front().find("execute_cycle_id=execute-cycle-uuid"), std::string::npos);
  EXPECT_NE(records.front().find("delivery_id=delivery-uuid"), std::string::npos);
  EXPECT_NE(records.front().find("target_id=target-profile"), std::string::npos);
  EXPECT_NE(
    records.front().find("motion_execution_id=fixture-pnp-1"),
    std::string::npos);
  EXPECT_NE(
    records.front().find(
      "motion_result_code=TASK_RESULT_BACKEND_UNAVAILABLE"), std::string::npos);
  EXPECT_NE(records.front().find("exit_reason=MISSION_EXIT_MOTION_ERROR"), std::string::npos);
  EXPECT_NE(records.front().find("diagnostic=\"backend unavailable\""), std::string::npos);
}

TEST(CycleCoordinatorTest, MotionActionFailureKeepsProvenanceWhenSafetyPreempts)
{
  Fixture fixture;
  std::vector<std::string> records;

  const auto result = run_motion_action_failure_episode(fixture, true, records);

  ASSERT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_EQ(
    result.motion_failure->code.value,
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(result.motion_failure->diagnostic_detail, "backend unavailable");
  EXPECT_EQ(result.motion_failure->motion_execution_id, "fixture-pnp-1");
  ASSERT_EQ(records.size(), 1U);
  EXPECT_NE(
    records.front().find(
      "motion_result_code=TASK_RESULT_BACKEND_UNAVAILABLE"), std::string::npos);
  EXPECT_NE(
    records.front().find(
      "exit_reason=MISSION_EXIT_SAFETY_PREEMPTED"), std::string::npos);
  EXPECT_NE(records.front().find("diagnostic=\"backend unavailable\""), std::string::npos);
}

TEST(CycleCoordinatorTest, ReportsShortfallWhenVisionDepletesBeforeQuantity)
{
  Fixture fixture;
  auto call_count = 0;
  fixture.vision = [&call_count](const DetectTarget::Request &) {
      ++call_count;
      return call_count == 1 ? found_target() : vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target", 2U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_SHORTFALL);
  EXPECT_EQ(result.processed_count, 1U);
  EXPECT_EQ(
    std::count(
      fixture.phases.begin(), fixture.phases.end(),
      arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_FINISHED),
    1);
  ASSERT_EQ(fixture.motion_tasks.size(), 3U);
  EXPECT_EQ(fixture.motion_tasks[0], MotionTaskType::TASK_TYPE_GO_HOME);
  EXPECT_EQ(fixture.motion_tasks[1], MotionTaskType::TASK_TYPE_PICK);
  EXPECT_EQ(fixture.motion_tasks[2], MotionTaskType::TASK_TYPE_PLACE);
  ASSERT_EQ(fixture.motion_goals.size(), 3U);
  ASSERT_EQ(fixture.motion_goals[1].material_candidates.size(), 1U);
  const auto & raw_candidate = fixture.motion_goals[1].material_candidates.front();
  EXPECT_DOUBLE_EQ(raw_candidate.target_pose.pose.position.x, 0.12);
  EXPECT_DOUBLE_EQ(raw_candidate.target_pose.pose.position.y, -0.34);
  EXPECT_DOUBLE_EQ(raw_candidate.target_pose.pose.position.z, 0.56);
  EXPECT_FLOAT_EQ(raw_candidate.estimated_height_m, 0.12F);
  EXPECT_TRUE(raw_candidate.has_observed_rectangle);
  EXPECT_FLOAT_EQ(fixture.motion_goals[1].grasp_width_min_mm, 40.0F);
  EXPECT_FLOAT_EQ(fixture.motion_goals[1].grasp_width_max_mm, 80.0F);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[1].pick_approach_distance_m, 0.15);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[1].pick_grasp_position_tolerance_m, 0.01);
  EXPECT_DOUBLE_EQ(
    fixture.motion_goals[1].pick_grasp_orientation_tolerance_rad, 0.08726646259971647);
  EXPECT_TRUE(raw_candidate.target_pose.pose.orientation.w > 0.99);
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

TEST(CycleCoordinatorTest, ForwardsEstimatedRectangleAndRecipeWidthWithoutObservedOutline)
{
  Fixture fixture;
  fixture.recipe.pick_approach_axis =
    arm_cell_orchestration::PickApproachAxis::PLANNING_FRAME_VERTICAL;
  fixture.vision = [](const DetectTarget::Request &) {
      auto response = found_target();
      response.material_candidates.front().orientation_symmetry_order = 2;
      response.material_candidates.front().has_observed_rectangle = false;
      response.material_candidates.front().rectangle_long_side_m = 0.08F;
      response.material_candidates.front().rectangle_short_side_m = 0.07F;

      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target", 1U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  ASSERT_EQ(fixture.pnp_goals.size(), 1U);
  const auto & pnp = fixture.pnp_goals.front();
  EXPECT_EQ(pnp.material_candidates.front().orientation_symmetry_order, 2U);
  EXPECT_FALSE(pnp.material_candidates.front().has_observed_rectangle);
  EXPECT_TRUE(pnp.material_candidates.front().has_estimated_height);
  EXPECT_FLOAT_EQ(pnp.material_candidates.front().estimated_height_m, 0.12F);
  EXPECT_FLOAT_EQ(pnp.material_candidates.front().rectangle_long_side_m, 0.08F);
  EXPECT_FLOAT_EQ(pnp.material_candidates.front().rectangle_short_side_m, 0.07F);
  EXPECT_FLOAT_EQ(pnp.grasp_width_min_mm, 40.0F);
  EXPECT_FLOAT_EQ(pnp.grasp_width_max_mm, 80.0F);
  EXPECT_EQ(
    pnp.pick_approach_translation_direction,
    arm_cell_interfaces::action::ExecutePnp::Goal::PICK_APPROACH_PLANNING_FRAME_VERTICAL);
  ASSERT_EQ(pnp.material_candidates.size(), 1U);
  EXPECT_DOUBLE_EQ(pnp.material_candidates.front().target_pose.pose.position.z, 0.56);
}

TEST(CycleCoordinatorTest, CompletesExactlyTheRequestedQuantityWithoutDuplicateHome)
{
  Fixture fixture;
  fixture.recipe.vision_algorithm = "cuboid_top_face";
  auto vision_count = 0;
  fixture.vision = [&vision_count](const DetectTarget::Request &) {
      ++vision_count;
      return found_target();
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target", 2U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(result.processed_count, 2U);
  EXPECT_EQ(vision_count, 2);
  ASSERT_EQ(fixture.motion_tasks.size(), 5U);
  EXPECT_EQ(fixture.motion_tasks[0], MotionTaskType::TASK_TYPE_GO_HOME);
  EXPECT_EQ(fixture.motion_tasks[1], MotionTaskType::TASK_TYPE_PICK);
  EXPECT_EQ(fixture.motion_tasks[2], MotionTaskType::TASK_TYPE_PLACE);
  EXPECT_EQ(fixture.motion_tasks[3], MotionTaskType::TASK_TYPE_PICK);
  EXPECT_EQ(fixture.motion_tasks[4], MotionTaskType::TASK_TYPE_PLACE);
}

TEST(CycleCoordinatorTest, FailedPnpFinalHomeKeepsPlacedCountAndMotionExitReason)
{
  Fixture fixture;
  auto callbacks = fixture.make_callbacks();
  callbacks.execute_pnp_with_feedback = [](
    const Fixture::PnpAction::Goal &, const Coordinator::PnpFeedbackFn &) {
      Coordinator::PnpResult result;
      result.outcome = Fixture::PnpAction::Result::PNP_OUTCOME_FAILED;
      result.material_placed = true;
      result.detail_code.value = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
      result.diagnostic_detail = "final GO_HOME failed";
      result.goal_uuid = "failed-final-home";
      result.action_terminal = "ABORTED";
      result.result_code_available = true;
      return result;
    };
  Coordinator coordinator(std::move(callbacks));

  const auto result = coordinator.execute("target_fixture", 1U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  EXPECT_EQ(result.processed_count, 1U);
  EXPECT_EQ(fixture.motion_tasks, (std::vector<uint8_t>{MotionTaskType::TASK_TYPE_GO_HOME}));
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

  const auto result = coordinator.execute("target", 3U);

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_SHORTFALL);
  EXPECT_EQ(result.processed_count, 2U);
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

  const auto result = coordinator.execute("target", 3U);

  ASSERT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_SHORTFALL);
  ASSERT_EQ(fixture.motion_goals.size(), 5U);
  EXPECT_FLOAT_EQ(fixture.motion_goals[1].grasp_width_max_mm, 80.0F);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[2].target_pose.pose.position.x, 0.8);
  EXPECT_FLOAT_EQ(fixture.motion_goals[3].grasp_width_max_mm, 80.0F);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[4].target_pose.pose.position.x, 0.8);
  EXPECT_EQ(fixture.recipe_load_count, 1);
}

TEST(CycleCoordinatorTest, AllowedUnobservableYawUsesSeparateFallbackInput)
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
      response.material_candidates.front().orientation_symmetry_order = 0;
      response.material_candidates.front().target_pose.pose.orientation.w = 0.0;

      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  ASSERT_GE(fixture.motion_goals.size(), 2U);
  EXPECT_EQ(fixture.motion_goals[1].orientation_symmetry_order, 0U);
  EXPECT_TRUE(fixture.motion_goals[1].has_unobserved_yaw_fallback);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[1].unobserved_yaw_fallback_rad, 1.2);
  EXPECT_DOUBLE_EQ(
    fixture.motion_goals[1].material_candidates.front().target_pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(
    fixture.motion_goals[1].material_candidates.front().target_pose.pose.orientation.w, 0.0);
  EXPECT_EQ(fixture.motion_goals[1].material_candidates.front().acquisition_id, "acquisition-test");
}

TEST(CycleCoordinatorTest, ForbiddenUnobservableYawDefersToPnpTerminalWithoutVisionRetry)
{
  Fixture fixture;
  fixture.recipe.unobserved_yaw_policy = UnobservedYawPolicy::FORBIDDEN;
  int acquisitions = 0;
  auto response = found_target();
  response.material_candidates.front().orientation_symmetry_order = 0;
  response.material_candidates.front().target_pose.pose.orientation.w = 0.0;

  fixture.vision = [&](const auto &) {++acquisitions; return response;};
  auto callbacks = fixture.make_callbacks();
  callbacks.execute_pnp_with_feedback = [&](const Coordinator::ExecutePnp::Goal & goal, const auto &) {
      fixture.pnp_goals.push_back(goal);
      Coordinator::PnpResult result;
      result.outcome = Coordinator::ExecutePnp::Result::PNP_OUTCOME_FAILED;
      result.action_terminal = "ABORTED";
      return result;
    };
  Coordinator coordinator(callbacks);
  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  EXPECT_EQ(acquisitions, 1);
  ASSERT_EQ(fixture.pnp_goals.size(), 1U);
  EXPECT_EQ(fixture.pnp_goals.front().material_candidates, response.material_candidates);
  EXPECT_FALSE(fixture.pnp_goals.front().has_unobserved_yaw_fallback);
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
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

TEST(CycleCoordinatorTest, RejectsInvalidRecipeAlgorithmBeforeVisionOrMotion)
{
  for (const auto & algorithm : {"", "unsupported"}) {
    Fixture fixture;
    fixture.recipe.vision_algorithm = algorithm;
    fixture.vision = [](const DetectTarget::Request &) {
        ADD_FAILURE() << "Invalid Recipe must not start Vision";
        return found_target();
      };
    auto coordinator = fixture.make_coordinator();
    const auto result = coordinator.execute("target_fixture");
    EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR);
    EXPECT_TRUE(fixture.motion_tasks.empty());
  }
}

TEST(CycleCoordinatorTest, RetriesTemporaryVisionInvalidityWithFreshAcquisition)
{
  Fixture fixture;
  fixture.recipe.vision_algorithm = "cuboid_top_face";
  int acquisitions = 0;
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      ++acquisitions;
      if (acquisitions == 1) {
        auto response = vision_result(
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
        response.diagnostic_detail = "temporary observation invalid";
        return response;
      }
      auto response = found_target();
      response.acquisition_id = "fresh-acquisition";
      for (auto & candidate : response.material_candidates) {
        candidate.acquisition_id = response.acquisition_id;
      }

      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(acquisitions, 2);
  ASSERT_GE(fixture.motion_goals.size(), 2U);
  EXPECT_EQ(fixture.motion_goals[1].material_candidates.front().acquisition_id, "fresh-acquisition");
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U, 1U}));
}

TEST(CycleCoordinatorTest, VisionRetriesExhaustAfterThreeAndRetainFirstFailure)
{
  Fixture fixture;
  int acquisitions = 0;
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      ++acquisitions;
      auto response = vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
      response.diagnostic_detail = "timeout attempt " + std::to_string(acquisitions);
      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
  EXPECT_EQ(acquisitions, 4);
  EXPECT_EQ(result.diagnostic_detail, "timeout attempt 1");
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U, 1U, 2U, 3U}));
  EXPECT_EQ(fixture.motion_tasks, (std::vector<uint8_t>{MotionTaskType::TASK_TYPE_GO_HOME}));
  const auto final_retry = std::find_if(
    fixture.diagnostic_records.rbegin(), fixture.diagnostic_records.rend(), [](const auto & record) {
      return record.event == Coordinator::DiagnosticEvent::RETRY;
    });
  ASSERT_NE(final_retry, fixture.diagnostic_records.rend());
  EXPECT_EQ(final_retry->attempt, 4U);
  EXPECT_EQ(final_retry->retry_budget, 3U);
  EXPECT_EQ(final_retry->decision, Coordinator::DiagnosticDecision::STOP);
  EXPECT_EQ(
    final_retry->reason,
    Coordinator::DiagnosticReason::RETRY_BUDGET_EXHAUSTED);
  EXPECT_EQ(
    final_retry->original_failure_code,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
}

TEST(CycleCoordinatorTest, VisionBusyWaitsWithoutConsumingRetryBudget)
{
  Fixture fixture;
  int acquisitions = 0;
  int idle_waits = 0;
  bool diagnostic_idle = false;
  uint64_t diagnostic_generation = 4U;
  fixture.read_vision_diagnostic_generation = [&diagnostic_generation]() {
      return diagnostic_generation;
    };
  fixture.wait_for_vision_idle = [&acquisitions, &idle_waits, &diagnostic_idle,
      &diagnostic_generation](
    uint64_t generation, const Coordinator::CancelPredicateFn &) {
      EXPECT_EQ(acquisitions, 1);
      EXPECT_EQ(generation, 4U);
      ++idle_waits;
      diagnostic_generation = 5U;  // BUSY is followed by a new IDLE observation.
      diagnostic_idle = true;
      return true;
    };
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      ++acquisitions;
      if (acquisitions == 1) {
        auto response = vision_result(8U);  // ICD: DETECT_RESULT_BUSY=8
        response.diagnostic_detail = "Vision acquisition slot is busy";
        return response;
      }
      return found_target();
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(acquisitions, 2);
  EXPECT_EQ(idle_waits, 1);
  EXPECT_TRUE(diagnostic_idle);
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
}

TEST(CycleCoordinatorTest, VisionBusyFailsClosedWhenIdleObservationIsUnavailable)
{
  Fixture fixture;
  int acquisitions = 0;
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      ++acquisitions;
      return vision_result(8U);
    };
  fixture.wait_for_vision_idle = [](
    uint64_t, const Coordinator::CancelPredicateFn &) {return false;};
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
  EXPECT_EQ(acquisitions, 1);
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
}

TEST(CycleCoordinatorTest, VisionBusyWaitCanBeCanceledWithoutAnotherServiceRequest)
{
  Fixture fixture;
  int acquisitions = 0;
  Coordinator * coordinator_ptr = nullptr;
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      ++acquisitions;
      return vision_result(8U);
    };
  fixture.wait_for_vision_idle = [&coordinator_ptr](
    uint64_t, const Coordinator::CancelPredicateFn &) {
      coordinator_ptr->set_cancel_requested(true);
      return false;
    };
  auto coordinator = fixture.make_coordinator();
  coordinator_ptr = &coordinator;

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CANCELED);
  EXPECT_EQ(acquisitions, 1);
}

TEST(CycleCoordinatorTest, VisionBusyRejectsIdleOlderThanServiceRequest)
{
  Fixture fixture;
  int acquisitions = 0;
  uint64_t diagnostic_generation = 7U;  // Existing IDLE predates this request.
  fixture.read_vision_diagnostic_generation = [&diagnostic_generation]() {
      return diagnostic_generation;
    };
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      ++acquisitions;
      return vision_result(8U);
    };
  fixture.wait_for_vision_idle = [&acquisitions](
    uint64_t request_generation, const Coordinator::CancelPredicateFn &) {
      EXPECT_EQ(request_generation, 7U);
      // The observed IDLE is still generation 7, so the bounded wait fails closed.
      return false;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
  EXPECT_EQ(acquisitions, 1);
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
}

TEST(CycleCoordinatorTest, VisionIdleBeforeBusyResponseDoesNotReleaseBusySlot)
{
  Fixture fixture;
  int acquisitions = 0;
  uint64_t diagnostic_generation = 7U;
  fixture.read_vision_diagnostic_generation = [&diagnostic_generation]() {
      return diagnostic_generation;
    };
  fixture.vision = [&acquisitions, &diagnostic_generation](const DetectTarget::Request &) {
      ++acquisitions;
      if (acquisitions == 1) {
        diagnostic_generation = 8U;  // Terminal IDLE delivered before BUSY response.
        return vision_result(8U);
      }
      return found_target();
    };
  fixture.wait_for_vision_idle = [&diagnostic_generation](
    uint64_t request_generation, const Coordinator::CancelPredicateFn &) {
      EXPECT_EQ(request_generation, 8U);
      return diagnostic_generation > request_generation;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
  EXPECT_EQ(acquisitions, 1);
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
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
  EXPECT_EQ(result.motion_failure->task_type, 0U);
  EXPECT_EQ(
    result.motion_failure->code.value,
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(result.motion_failure->diagnostic_detail, "persistent backend transport failure");
  EXPECT_EQ(result.motion_failure->motion_execution_id, "fixture-pnp-1");
  ASSERT_EQ(fixture.motion_tasks.size(), 2U);
  EXPECT_EQ(fixture.motion_tasks[0], MotionTaskType::TASK_TYPE_GO_HOME);
  EXPECT_EQ(fixture.motion_tasks[1], MotionTaskType::TASK_TYPE_PICK);
}

TEST(CycleCoordinatorTest, RetryablePickFailureHomesAndUsesANewDetectionBeforeNewPick)
{
  Fixture fixture;
  fixture.recipe.vision_algorithm = "cuboid_top_face";
  fixture.motion_status = []() {
      Coordinator::MotionStatus status;
      status.execution_state = Coordinator::MotionStatus::MOTION_STATE_IDLE;
      status.holding_state = Coordinator::MotionStatus::HOLDING_RELEASED;
      return std::optional<Coordinator::MotionStatus>{status};
    };
  int vision_calls = 0;
  int pick_calls = 0;
  fixture.vision = [&vision_calls](const DetectTarget::Request &) {
      auto response = found_target();
      response.material_candidates.front().target_pose.pose.position.x = 0.1 * static_cast<double>(++vision_calls);

      return response;
    };
  fixture.motion = [&fixture, &pick_calls](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK && ++pick_calls == 1) {
        return pick_work_failure(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
      }
      return successful_motion_result(goal);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(vision_calls, 2);
  const std::vector<uint8_t> expected{
    MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
    MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
    MotionTaskType::TASK_TYPE_PLACE};
  EXPECT_EQ(fixture.motion_tasks, expected);
  ASSERT_EQ(fixture.motion_goals.size(), expected.size());
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
  EXPECT_TRUE(fixture.motion_status_wait_requests.empty());
  const auto pnp_bt_failure = std::find_if(
    fixture.diagnostic_records.begin(), fixture.diagnostic_records.end(), [](const auto & record) {
      return record.event == Coordinator::DiagnosticEvent::BT_NODE_STATUS &&
          std::string(record.tree_node.data(), record.tree_node_size) == "ExecutePnp" &&
          std::string(record.tree_current_status.data(), record.tree_current_status_size) ==
          "FAILURE";
    });
  ASSERT_NE(pnp_bt_failure, fixture.diagnostic_records.end());
  EXPECT_EQ(pnp_bt_failure->failure_domain, Coordinator::DiagnosticFailureDomain::NONE);
  EXPECT_DOUBLE_EQ(
    fixture.motion_goals[1].material_candidates.front().target_pose.pose.position.x, 0.1);
  EXPECT_DOUBLE_EQ(
    fixture.motion_goals[3].material_candidates.front().target_pose.pose.position.x, 0.2);
  {
    std::lock_guard<std::mutex> lock(fixture.tree_status_mutex);
    EXPECT_EQ(
      std::count_if(
        fixture.tree_status_events.begin(), fixture.tree_status_events.end(),
        [](const auto & event) {
          return event.node == "DetectTarget" && event.current == "SUCCESS";
        }),
      2);
  }
}

TEST(CycleCoordinatorTest, BackendFailureWithReturnedPickIsNotReenteredAsWorkFailure)
{
  Fixture fixture;
  fixture.recipe.vision_algorithm = "cuboid_top_face";
  int vision_calls = 0;
  int pick_calls = 0;
  fixture.vision = [&vision_calls](const DetectTarget::Request &) {
      ++vision_calls;
      return found_target();
    };
  fixture.motion = [&fixture, &pick_calls](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK && ++pick_calls == 1) {
        auto result = motion_result(MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
        result.object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED;
        return result;
      }
      return successful_motion_result(goal);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR)
    << result.diagnostic_detail;
  EXPECT_EQ(vision_calls, 1);
  EXPECT_EQ(fixture.motion_tasks, (std::vector<uint8_t>{
      MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK}));
}

TEST(CycleCoordinatorTest, BackendFailureWithReturnedPlaceIsNotReenteredAsWorkFailure)
{
  Fixture fixture;
  fixture.recipe.vision_algorithm = "cuboid_top_face";
  int place_calls = 0;
  fixture.motion = [&fixture, &place_calls](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE && ++place_calls == 1) {
        auto result = motion_result(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
        result.object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED;
        return result;
      }
      return successful_motion_result(goal);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR)
    << result.diagnostic_detail;
  EXPECT_EQ(place_calls, 1);
  EXPECT_EQ(result.processed_count, 0U);
  EXPECT_EQ(fixture.motion_tasks, (std::vector<uint8_t>{
      MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
      MotionTaskType::TASK_TYPE_PLACE}));
}

TEST(CycleCoordinatorTest, JointProgressStallReentersFromDispositionWithoutMotionStatusGate)
{
  Fixture fixture;
  fixture.recipe.vision_algorithm = "cuboid_top_face";
  fixture.motion_status = []() {
      Coordinator::MotionStatus status;
      status.execution_state = Coordinator::MotionStatus::MOTION_STATE_STOPPED;
      status.backend_inactivity_confirmed = true;
      status.holding_state = Coordinator::MotionStatus::HOLDING_RELEASED;
      return std::optional<Coordinator::MotionStatus>{status};
    };
  int acquisitions = 0;
  int picks = 0;
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      auto response = found_target();
      response.acquisition_id = "stall-acquisition-" + std::to_string(++acquisitions);
      for (auto & candidate : response.material_candidates) {
        candidate.acquisition_id = response.acquisition_id;
      }

      return response;
    };
  fixture.motion = [&fixture, &picks](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK && ++picks == 1) {
        auto result = pick_work_failure(
          MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED);
        result.action_terminal = "ABORTED";
        result.pick_grasp_insertion_started_available = true;
        result.pick_grasp_insertion_started = true;
        return result;
      }
      return successful_motion_result(goal);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(fixture.motion_tasks, (std::vector<uint8_t>{
      MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
      MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK,
      MotionTaskType::TASK_TYPE_PLACE}));
  EXPECT_EQ(acquisitions, 2);
  EXPECT_EQ(picks, 2);
  EXPECT_TRUE(fixture.motion_status_wait_requests.empty());
  ASSERT_EQ(fixture.motion_goals.size(), 5U);
  ASSERT_FALSE(fixture.motion_goals[1].material_candidates.empty());
  ASSERT_FALSE(fixture.motion_goals[3].material_candidates.empty());
  EXPECT_EQ(
    fixture.motion_goals[1].material_candidates.front().acquisition_id,
    "stall-acquisition-1");
  EXPECT_EQ(
    fixture.motion_goals[3].material_candidates.front().acquisition_id,
    "stall-acquisition-2");
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
}

TEST(CycleCoordinatorTest, UnknownDispositionIsNotReclassifiedFromMotionStatus)
{
  for (const auto mode : {0, 1, 2, 3, 4}) {
    Fixture fixture;
    fixture.motion_status = [mode]() {
        Coordinator::MotionStatus status;
        status.execution_state = mode == 3 ?
          Coordinator::MotionStatus::MOTION_STATE_EXECUTING :
          Coordinator::MotionStatus::MOTION_STATE_STOPPED;
        status.execution_active = mode == 3;
        status.has_active_execution = mode == 3;
        status.backend_inactivity_confirmed = mode != 3 && mode != 4;
        status.holding_state = mode == 0 ? Coordinator::MotionStatus::HOLDING_HELD :
          mode == 1 ? Coordinator::MotionStatus::HOLDING_UNKNOWN :
          Coordinator::MotionStatus::HOLDING_RELEASED;
        if (mode == 4) {
          status.diagnostic_detail =
            "joint_progress_stalled;trajectory_worker_cancelled=true;"
            "hold_command_published=true;physical_stop_observation:collecting_stable_samples";
        }
        return std::optional<Coordinator::MotionStatus>{status};
      };
    int vision_calls = 0;
    int pick_calls = 0;
    fixture.vision = [&vision_calls](const DetectTarget::Request &) {
        ++vision_calls;
        return found_target();
      };
    fixture.motion = [&fixture, mode, &pick_calls](const ExecuteTask::Goal & goal) {
        fixture.motion_tasks.push_back(goal.task_type.value);
        fixture.motion_goals.push_back(goal);
        if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK && ++pick_calls == 1) {
          auto result = motion_result(
            MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED);
          result.action_terminal = "ABORTED";
          result.pick_grasp_insertion_started_available = true;
          result.pick_grasp_insertion_started = mode != 2;
          return result;
        }
        return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
      };
    auto coordinator = fixture.make_coordinator();

    const auto result = coordinator.execute("target_fixture");

    EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
    EXPECT_EQ(vision_calls, 1);
    EXPECT_EQ(pick_calls, 1);
    EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
    EXPECT_TRUE(fixture.motion_status_wait_requests.empty());
  }
}

TEST(CycleCoordinatorTest, NonStallTimeoutTransportCancellationAndSafetyFailuresDoNotRetry)
{
  for (const auto code : {
    MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR,
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE,
    MotionTaskResultCode::TASK_RESULT_CANCELED,
    MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED})
  {
    Fixture fixture;
    fixture.recipe.vision_algorithm = "cuboid_top_face";
    fixture.motion_status = []() {
        Coordinator::MotionStatus status;
        status.execution_state = Coordinator::MotionStatus::MOTION_STATE_STOPPED;
        status.backend_inactivity_confirmed = true;
        status.holding_state = Coordinator::MotionStatus::HOLDING_RELEASED;
        return std::optional<Coordinator::MotionStatus>{status};
      };
    int vision_calls = 0;
    int pick_calls = 0;
    fixture.vision = [&vision_calls](const DetectTarget::Request &) {
        ++vision_calls;
        return found_target();
      };
    fixture.motion = [&fixture, code, &pick_calls](const ExecuteTask::Goal & goal) {
        fixture.motion_tasks.push_back(goal.task_type.value);
        fixture.motion_goals.push_back(goal);
        if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
          ++pick_calls;
          auto result = motion_result(code);
          result.action_terminal = "ABORTED";
          result.pick_grasp_insertion_started_available = true;
          result.pick_grasp_insertion_started = true;
          return result;
        }
        return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
      };
    auto coordinator = fixture.make_coordinator();

    const auto result = coordinator.execute("target_fixture");

    EXPECT_EQ(vision_calls, 1);
    EXPECT_EQ(pick_calls, 1);
    EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
    if (code == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
      EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
    } else if (code == MotionTaskResultCode::TASK_RESULT_CANCELED) {
      EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
    } else {
      EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
    }
  }
}

TEST(CycleCoordinatorTest, BusyPickResultStopsWithoutSpendingPickRetryBudget)
{
  EXPECT_STREQ(
    arm_cell_orchestration::motion_result_code_name(
      MotionTaskResultCode::TASK_RESULT_BUSY),
    "TASK_RESULT_BUSY");
  Fixture fixture;
  fixture.motion_status = []() {
      Coordinator::MotionStatus status;
      status.execution_state = Coordinator::MotionStatus::MOTION_STATE_IDLE;
      status.holding_state = Coordinator::MotionStatus::HOLDING_RELEASED;
      return std::optional<Coordinator::MotionStatus>{status};
    };
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
        return motion_result(MotionTaskResultCode::TASK_RESULT_BUSY);
      }
      return successful_motion_result(goal);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  EXPECT_EQ(
    fixture.motion_tasks,
    (std::vector<uint8_t>{MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK}));
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
  EXPECT_EQ(
    std::count_if(
      fixture.diagnostic_records.begin(), fixture.diagnostic_records.end(), [](const auto & record) {
        return record.event == Coordinator::DiagnosticEvent::RETRY &&
               record.decision == Coordinator::DiagnosticDecision::RETRY;
      }), 0);
}

TEST(CycleCoordinatorTest, PickFailureWithUnknownHoldingStateDoesNotRetry)
{
  Fixture fixture;
  fixture.recipe.vision_algorithm = "cuboid_top_face";
  int vision_calls = 0;
  fixture.motion_status = []() {
      Coordinator::MotionStatus status;
      status.execution_state = Coordinator::MotionStatus::MOTION_STATE_IDLE;
      status.holding_state = Coordinator::MotionStatus::HOLDING_UNKNOWN;
      return std::optional<Coordinator::MotionStatus>{status};
    };
  fixture.vision = [&vision_calls](const DetectTarget::Request &) {
      ++vision_calls;
      return found_target();
    };
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      return goal.task_type.value == MotionTaskType::TASK_TYPE_PICK ?
             motion_result(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED) :
             motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  EXPECT_EQ(vision_calls, 1);
  EXPECT_EQ(fixture.motion_tasks.size(), 2U);
}

TEST(CycleCoordinatorTest, WorkRetryDoesNotReadMotionStatusOrConsumeVisionRetry)
{
  Fixture fixture;
  fixture.recipe.vision_algorithm = "cuboid_top_face";
  int pick_calls = 0;
  fixture.motion_status = []() {
      Coordinator::MotionStatus status;
      status.execution_state = Coordinator::MotionStatus::MOTION_STATE_IDLE;
      status.execution_active = true;
      status.has_active_execution = true;
      status.backend_inactivity_confirmed = false;
      status.holding_state = Coordinator::MotionStatus::HOLDING_RELEASED;
      return std::optional<Coordinator::MotionStatus>{status};
    };
  fixture.motion = [&fixture, &pick_calls](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      return goal.task_type.value == MotionTaskType::TASK_TYPE_PICK && ++pick_calls == 1 ?
             pick_work_failure(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED) :
             motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
  EXPECT_TRUE(fixture.motion_status_wait_requests.empty());
  EXPECT_EQ(pick_calls, 2);
}

TEST(CycleCoordinatorTest, SafetyRecoveryAtRetryAdmissionPreventsRetry)
{
  Fixture fixture;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
        fixture.safety = recovery_safety();
        return pick_work_failure(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  ASSERT_EQ(fixture.motion_tasks.size(), 2U);
  EXPECT_EQ(fixture.motion_tasks.front(), MotionTaskType::TASK_TYPE_GO_HOME);
  EXPECT_EQ(fixture.motion_tasks.back(), MotionTaskType::TASK_TYPE_PICK);
}

TEST(CycleCoordinatorTest, PickFailureWithHeldObjectDoesNotRetry)
{
  Fixture fixture;
  fixture.recipe.vision_algorithm = "cuboid_top_face";
  fixture.motion_status = []() {
      Coordinator::MotionStatus status;
      status.execution_state = Coordinator::MotionStatus::MOTION_STATE_IDLE;
      status.holding_state = Coordinator::MotionStatus::HOLDING_HELD;
      return std::optional<Coordinator::MotionStatus>{status};
    };
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      return goal.task_type.value == MotionTaskType::TASK_TYPE_PICK ?
             motion_result(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED) :
             motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  ASSERT_EQ(fixture.motion_tasks.size(), 2U);
  EXPECT_EQ(fixture.motion_tasks.back(), MotionTaskType::TASK_TYPE_PICK);
}

TEST(CycleCoordinatorTest, RetryHomeFailureDoesNotAcquireOrPickAgain)
{
  Fixture fixture;
  fixture.motion_status = []() {
      Coordinator::MotionStatus status;
      status.execution_state = Coordinator::MotionStatus::MOTION_STATE_IDLE;
      status.holding_state = Coordinator::MotionStatus::HOLDING_RELEASED;
      return std::optional<Coordinator::MotionStatus>{status};
    };
  int vision_calls = 0;
  int pick_calls = 0;
  fixture.vision = [&vision_calls](const DetectTarget::Request &) {
      ++vision_calls;
      return found_target();
    };
  fixture.motion = [&fixture, &pick_calls](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
        ++pick_calls;
        return pick_work_failure(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
      }
      if (fixture.motion_tasks.size() == 3U) {
        auto result = motion_result(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
        result.diagnostic_detail = "retry GO_HOME failed";
        return result;
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  EXPECT_EQ(vision_calls, 1);
  EXPECT_EQ(pick_calls, 1);
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_EQ(result.motion_failure->task_type, MotionTaskType::TASK_TYPE_GO_HOME);
  EXPECT_EQ(
    result.motion_failure->code.value,
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(result.motion_failure->diagnostic_detail, "retry GO_HOME failed");
}

TEST(CycleCoordinatorTest, UnlimitedPickWorkRetriesStayInsideOneExecuteCycle)
{
  Fixture fixture;
  fixture.recipe.vision_algorithm = "cuboid_top_face";
  fixture.motion_status = []() {
      Coordinator::MotionStatus status;
      status.execution_state = Coordinator::MotionStatus::MOTION_STATE_STOPPED;
      status.holding_state = Coordinator::MotionStatus::HOLDING_RELEASED;
      status.backend_inactivity_confirmed = true;
      return std::optional<Coordinator::MotionStatus>{status};
    };
  int vision_calls = 0;
  int pick_calls = 0;
  fixture.vision = [&vision_calls](const DetectTarget::Request &) {
      ++vision_calls;
      auto response = found_target();
      response.acquisition_id = "unlimited-reentry-" + std::to_string(vision_calls);
      for (auto & candidate : response.material_candidates) {
        candidate.acquisition_id = response.acquisition_id;
      }
      response.material_candidates.front().candidate_id = "unlimited-candidate-" + std::to_string(vision_calls);

      return response;
    };
  fixture.motion = [&fixture, &pick_calls](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK) {
        ++pick_calls;
        if (pick_calls <= 6) {
          return pick_work_failure(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
        }
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(vision_calls, 7);
  EXPECT_EQ(pick_calls, 7);
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
  ASSERT_EQ(fixture.motion_tasks.size(), 15U);
  EXPECT_EQ(fixture.motion_tasks.back(), MotionTaskType::TASK_TYPE_PLACE);
}

TEST(CycleCoordinatorTest, UnlimitedPlaceWorkRetriesStayInsideOneExecuteCycle)
{
  Fixture fixture;
  int vision_calls = 0;
  int place_calls = 0;
  fixture.vision = [&vision_calls](const DetectTarget::Request &) {
      auto response = found_target();
      response.acquisition_id = "place-reentry-" + std::to_string(++vision_calls);
      for (auto & candidate : response.material_candidates) {
        candidate.acquisition_id = response.acquisition_id;
      }

      return response;
    };
  fixture.motion = [&fixture, &place_calls](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE && ++place_calls <= 5) {
        auto result = motion_result(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
        result.object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED;
        return result;
      }
      return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(result.processed_count, 1U);
  EXPECT_EQ(vision_calls, 6);
  EXPECT_EQ(place_calls, 6);
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
  EXPECT_TRUE(fixture.motion_status_wait_requests.empty());
  EXPECT_EQ(std::count(fixture.motion_tasks.begin(), fixture.motion_tasks.end(),
      MotionTaskType::TASK_TYPE_PICK), 6);
  EXPECT_EQ(std::count(fixture.motion_tasks.begin(), fixture.motion_tasks.end(),
      MotionTaskType::TASK_TYPE_PLACE), 6);
}

TEST(CycleCoordinatorTest, ActionRejectionDoesNotInventMotionResultCode)
{
  Fixture fixture;
  fixture.motion = [](const ExecuteTask::Goal &) {
      auto result = motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
      result.result_code_available = false;
      result.diagnostic_detail = "Motion Action rejected Goal; no Action Result was returned";
      return result;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_FALSE(result.motion_failure->result_code_available);
  EXPECT_EQ(
    result.motion_failure->diagnostic_detail,
    "Motion Action rejected Goal; no Action Result was returned");
}

TEST(CycleCoordinatorTest, RejectedPickDoesNotSpendExecutionRetryBudget)
{
  Fixture fixture;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      if (goal.task_type.value != MotionTaskType::TASK_TYPE_PICK) {
        return motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
      }
      auto rejected = motion_result(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
      rejected.result_code_available = false;
      rejected.diagnostic_detail = "Motion Action rejected Goal; no Action Result was returned";
      return rejected;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target_fixture");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  EXPECT_EQ(fixture.retry_counts, (std::vector<uint8_t>{0U}));
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_TRUE(result.motion_failure->result_code_available);
  EXPECT_EQ(
    fixture.motion_tasks,
    (std::vector<uint8_t>{MotionTaskType::TASK_TYPE_GO_HOME, MotionTaskType::TASK_TYPE_PICK}));
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
  EXPECT_EQ(result.motion_failure->motion_execution_id, "fixture-pnp-1");
  ASSERT_EQ(fixture.motion_tasks.size(), 2U);
  EXPECT_EQ(fixture.motion_tasks.back(), MotionTaskType::TASK_TYPE_PICK);
}

TEST(CycleCoordinatorTest, TerminalFailureRecordJoinsExistingMissionAndMotionIdentities)
{
  std::vector<std::string> records;
  arm_cell_orchestration::emit_terminal_failure_record(
    arm_cell_orchestration::TerminalFailureRecord{
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
  EXPECT_NE(
    record.find("motion_result_code=TASK_RESULT_BACKEND_UNAVAILABLE"),
    std::string::npos);
  EXPECT_NE(record.find("exit_reason=MISSION_EXIT_MOTION_ERROR"), std::string::npos);
  EXPECT_NE(record.find("processed=0/0"), std::string::npos);
  EXPECT_NE(
    record.find("diagnostic=\"persistent backend transport failure\""),
    std::string::npos);
}

TEST(CycleCoordinatorTest, TerminalFailureRecordMarksUnavailableActionResultWithoutGuessing)
{
  const auto record = arm_cell_orchestration::format_terminal_failure_record(
    arm_cell_orchestration::TerminalFailureRecord{
    "execute-cycle-uuid", "delivery-uuid", "target-profile", "", MotionTaskType::TASK_TYPE_PICK,
    0, MissionExitReason::MISSION_EXIT_MOTION_ERROR,
    "Motion Action rejected Goal; no Action Result was returned", false});

  EXPECT_NE(record.find("motion_result_code=unavailable"), std::string::npos);
  EXPECT_NE(record.find("exit_reason=MISSION_EXIT_MOTION_ERROR"), std::string::npos);
  EXPECT_NE(record.find("Motion Action rejected Goal"), std::string::npos);
}

TEST(CycleCoordinatorTest, UnknownMotionResultCodeUsesSemanticName)
{
  EXPECT_EQ(arm_cell_orchestration::motion_result_code_field(255), "UNKNOWN");
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

TEST(CycleCoordinatorTest, CallerCancellationAfterBusyPreservesMotionResult)
{
  Fixture fixture;
  Coordinator * coordinator_ptr = nullptr;
  fixture.motion = [&fixture, &coordinator_ptr](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      coordinator_ptr->set_cancel_requested(true);
      auto result = motion_result(MotionTaskResultCode::TASK_RESULT_BUSY);
      result.diagnostic_detail = "reservation remains active";
      result.motion_execution_id = "busy-motion-goal";
      return result;
    };
  auto coordinator = fixture.make_coordinator();
  coordinator_ptr = &coordinator;

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CANCELED);
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_EQ(result.motion_failure->code.value, MotionTaskResultCode::TASK_RESULT_BUSY);
  EXPECT_EQ(result.motion_failure->diagnostic_detail, "reservation remains active");
  EXPECT_EQ(result.motion_failure->motion_execution_id, "busy-motion-goal");
  const auto cancellation_record = std::find_if(
    fixture.diagnostic_records.begin(), fixture.diagnostic_records.end(),
    [](const auto & record) {
      return record.event == Coordinator::DiagnosticEvent::CANCELLATION;
    });
  ASSERT_NE(cancellation_record, fixture.diagnostic_records.end());
  EXPECT_EQ(cancellation_record->failure_code, MotionTaskResultCode::TASK_RESULT_BUSY);
  EXPECT_EQ(cancellation_record->final_result, MissionExitReason::MISSION_EXIT_CANCELED);
  EXPECT_TRUE(cancellation_record->result_available);
  EXPECT_EQ(std::string(cancellation_record->result_detail.data(),
    cancellation_record->result_detail_size), "reservation remains active");
}

TEST(CycleCoordinatorTest, SafetyWithdrawalAfterBusyPreservesMotionResult)
{
  Fixture fixture;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      fixture.safety = recovery_safety();
      auto result = motion_result(MotionTaskResultCode::TASK_RESULT_BUSY);
      result.diagnostic_detail = "reservation remains active";
      result.motion_execution_id = "busy-motion-goal";
      result.action_terminal = "ABORTED";
      return result;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_EQ(result.motion_failure->code.value, MotionTaskResultCode::TASK_RESULT_BUSY);
  EXPECT_EQ(result.motion_failure->diagnostic_detail, "reservation remains active");
  EXPECT_EQ(result.motion_failure->motion_execution_id, "busy-motion-goal");
  EXPECT_EQ(result.motion_failure->action_terminal, "ABORTED");
  const auto safety_record = std::find_if(
    fixture.diagnostic_records.begin(), fixture.diagnostic_records.end(),
    [](const auto & record) {
      return record.event == Coordinator::DiagnosticEvent::CANCELLATION;
    });
  ASSERT_NE(safety_record, fixture.diagnostic_records.end());
  EXPECT_EQ(safety_record->failure_code, MotionTaskResultCode::TASK_RESULT_BUSY);
  EXPECT_EQ(safety_record->final_result, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_EQ(std::string(safety_record->motion_action_terminal.data(),
    safety_record->motion_action_terminal_size), "ABORTED");
}

TEST(CycleCoordinatorTest, NonSucceededActionTerminalCannotCompleteMission)
{
  Fixture fixture;
  fixture.motion = [&fixture](const ExecuteTask::Goal & goal) {
      fixture.motion_tasks.push_back(goal.task_type.value);
      fixture.motion_goals.push_back(goal);
      auto result = motion_result(MotionTaskResultCode::TASK_RESULT_SUCCESS);
      result.diagnostic_detail = "Motion reported success";
      result.motion_execution_id = "motion-goal-with-aborted-action";
      result.action_terminal = "ABORTED";
      return result;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
  ASSERT_TRUE(result.motion_failure.has_value());
  EXPECT_EQ(result.motion_failure->code.value, MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(result.motion_failure->action_terminal, "ABORTED");
}

TEST(CycleCoordinatorTest, CallerCancellationBeforeMotionPreservesCancellation)
{
  Fixture fixture;
  auto coordinator = fixture.make_coordinator();
  coordinator.set_cancel_requested(true);

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_CANCELED);
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

TEST(CycleCoordinatorTest, SafetyPreemptionWithoutRecoveryPreservesFailure)
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
  bool vision_result_classified = false;
  fixture.record_vision_result = [&vision_result_classified](uint8_t) {
      vision_result_classified = true;
    };
  fixture.vision = [&fixture](const DetectTarget::Request &) {
      fixture.safety = recovery_safety();
      return vision_result(
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
  EXPECT_FALSE(vision_result_classified);
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
  fixture.recipe.has_grasp_width_policy = false;
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
  fixture.recipe.grasp_width_min_mm = std::numeric_limits<float>::quiet_NaN();
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
      response.material_candidates.front().target_pose.pose.position.x = 1.7;
      response.material_candidates.front().target_pose.pose.position.y = 1.8;

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

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  ASSERT_GE(fixture.motion_goals.size(), 3U);
  EXPECT_DOUBLE_EQ(fixture.motion_goals[1].material_candidates.front().target_pose.pose.position.x, 1.7);
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
  const auto loaded = arm_cell_orchestration::RecipeRepository(
    std::filesystem::path(ARM_CELL_TEST_RECIPE_DIR)).load("RawPart");
  ASSERT_TRUE(loaded.recipe);
  EXPECT_EQ(
    loaded.recipe->place_orientation_constraint,
    arm_cell_orchestration::PlaceOrientationConstraint::BOUNDED);
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

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
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

TEST(CycleCoordinatorTest, RejectsNonBaseLinkVisionPoseBeforePick)
{
  Fixture fixture;
  fixture.vision = [](const DetectTarget::Request &) {
      auto response = found_target();
      response.material_candidates.front().target_pose.header.frame_id = "camera_link";

      return response;
    };
  auto coordinator = fixture.make_coordinator();

  const auto result = coordinator.execute("target");

  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_VISION_ERROR);
  ASSERT_EQ(fixture.motion_goals.size(), 1U);
  EXPECT_EQ(fixture.motion_goals.front().task_type.value, MotionTaskType::TASK_TYPE_GO_HOME);
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
  arm_cell_orchestration::CancellationRegistry registry;
  bool execution_canceled = false;
  bool motion_canceled = false;

  registry.request_caller_cancel("cycle");
  registry.register_execution("cycle", [&]() {execution_canceled = true;});
  auto submission = register_test_submission(registry, "cycle");
  submission->set_cancel_fn([&]() {motion_canceled = true;});

  EXPECT_TRUE(execution_canceled);
  EXPECT_TRUE(motion_canceled);
}

TEST(CancellationRegistryTest, CallerCancellationForwardsToActiveMotion)
{
  arm_cell_orchestration::CancellationRegistry registry;
  bool execution_canceled = false;
  int motion_cancel_count = 0;

  registry.register_execution("cycle", [&]() {execution_canceled = true;});
  auto submission = register_test_submission(registry, "cycle");
  submission->set_cancel_fn([&]() {motion_cancel_count++;});
  registry.request_caller_cancel("cycle");

  EXPECT_TRUE(execution_canceled);
  EXPECT_EQ(motion_cancel_count, 1);
}

TEST(CancellationRegistryTest, SecondaryCancellationDoesNotSynthesizeCallerCancellation)
{
  arm_cell_orchestration::CancellationRegistry registry;
  bool execution_canceled = false;
  int motion_cancel_count = 0;

  registry.register_execution("cycle", [&]() {execution_canceled = true;});
  auto submission = register_test_submission(registry, "cycle");
  submission->set_cancel_fn([&]() {motion_cancel_count++;});
  submission->request_cancel();

  EXPECT_FALSE(execution_canceled);
  EXPECT_EQ(motion_cancel_count, 1);
}

TEST(CancellationRegistryTest, CallerCancellationBeforeMotionAcceptanceCancelsLateHandle)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("cycle", []() {});
  auto submission = register_test_submission(registry, "cycle");
  int motion_cancel_count = 0;

  registry.request_caller_cancel("cycle");
  submission->set_cancel_fn([&]() {motion_cancel_count++;});

  EXPECT_EQ(motion_cancel_count, 1);
}

TEST(CancellationRegistryTest, CancellationRacingMotionAcceptanceCancelsExactlyOnce)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("cycle", []() {});
  auto submission = register_test_submission(registry, "cycle");
  std::atomic<int> motion_cancel_count{0};
  std::mutex start_mutex;
  std::condition_variable start_condition;
  bool start = false;
  const auto await_start = [&]() {
      std::unique_lock<std::mutex> lock(start_mutex);
      start_condition.wait(lock, [&]() {return start;});
    };

  std::thread cancellation_thread([&]() {
      await_start();
      submission->request_cancel();
    });
  std::thread acceptance_thread([&]() {
      await_start();
      submission->set_cancel_fn([&]() {motion_cancel_count.fetch_add(1);});
    });
  {
    std::lock_guard<std::mutex> lock(start_mutex);
    start = true;
  }
  start_condition.notify_all();
  cancellation_thread.join();
  acceptance_thread.join();

  EXPECT_EQ(motion_cancel_count.load(), 1);
}

TEST(CancellationRegistryTest, MissionTerminalBeforeMotionAcceptanceCancelsLateHandle)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("cycle", []() {});
  auto submission = register_test_submission(registry, "cycle");
  int motion_cancel_count = 0;

  registry.erase_execution("cycle");
  submission->set_cancel_fn([&]() {motion_cancel_count++;});

  EXPECT_EQ(motion_cancel_count, 1);
}

TEST(CancellationRegistryTest, MotionSubmissionAfterMissionTerminalIsRejected)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("cycle", []() {});
  registry.erase_execution("cycle");

  EXPECT_FALSE(register_test_submission(registry, "cycle"));
}

TEST(CancellationRegistryTest, TerminalMotionResultClearsSubmissionBeforeMissionErase)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("cycle", []() {});
  auto submission = register_test_submission(registry, "cycle");
  int motion_cancel_count = 0;
  submission->set_cancel_fn([&]() {motion_cancel_count++;});

  registry.clear_active_action("cycle", submission);
  registry.erase_execution("cycle");

  EXPECT_EQ(motion_cancel_count, 0);
}

TEST(CancellationRegistryTest, UnresolvedSubmissionBlocksAnotherGoalUntilResolved)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("cycle", []() {});
  auto first = register_test_submission(registry, "cycle");

  ASSERT_TRUE(first);
  EXPECT_TRUE(first->set_goal_uuid("original-goal-uuid"));
  EXPECT_EQ(first->goal_uuid(), "original-goal-uuid");
  EXPECT_FALSE(register_test_submission(registry, "cycle"));

  registry.clear_active_action("cycle", first);
  EXPECT_TRUE(register_test_submission(registry, "cycle"));
  registry.erase_execution("cycle");
}

TEST(CycleCoordinatorTest, ExhaustedPickCandidatesDisturbBeforeReacquisition)
{
  Fixture fixture;
  int acquisitions = 0;
  fixture.vision = [&acquisitions](const DetectTarget::Request &) {
      ++acquisitions;
      auto response = found_target();
      response.disturbance_candidates = disturbance_target().disturbance_candidates;
      return response;
    };
  int picks = 0;
  fixture.motion = [&picks](const ExecuteTask::Goal & goal) {
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_PICK && ++picks == 1) {
        auto failed = motion_result(MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED);
        failed.object_disposition = ExecuteTask::Result::OBJECT_DISPOSITION_UNCHANGED;
        return failed;
      }
      return successful_motion_result(goal);
    };
  auto coordinator = fixture.make_coordinator();
  const auto result = coordinator.execute("target_fixture", 3U);
  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(fixture.disturb_goals.size(), 1U);
  EXPECT_EQ(acquisitions, 4);
}

TEST(CancellationRegistryTest, SecondaryCancelWithoutActiveActionDoesNotCancelNextSubmission)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("cycle", []() {});
  auto previous = register_test_submission(registry, "cycle");
  ASSERT_TRUE(previous);
  registry.clear_active_action("cycle", previous);
  previous->request_cancel();
  auto next = register_test_submission(registry, "cycle");
  ASSERT_TRUE(next);
  int cancel_count = 0;
  next->set_cancel_fn([&]() {++cancel_count;});
  EXPECT_EQ(cancel_count, 0);
}

TEST(CancellationRegistryTest, LateOldCancellationAndClearDoNotAffectNewAction)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("cycle", []() {});
  auto old_action = register_test_submission(registry, "cycle");
  ASSERT_TRUE(old_action);
  registry.clear_active_action("cycle", old_action);
  auto new_action = register_test_submission(registry, "cycle");
  ASSERT_TRUE(new_action);
  int old_cancels = 0;
  int new_cancels = 0;
  old_action->request_cancel();
  old_action->set_cancel_fn([&]() {++old_cancels;});
  new_action->set_cancel_fn([&]() {++new_cancels;});
  registry.clear_active_action("cycle", old_action);
  registry.request_caller_cancel("cycle");
  EXPECT_EQ(old_cancels, 1);
  EXPECT_EQ(new_cancels, 1);
}

TEST(CancellationRegistryTest, CancelBeforeSubmissionRegistrationTargetsThatSubmission)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("cycle", []() {});
  auto action = std::make_shared<arm_cell_orchestration::CancellationRegistry::ActionSubmission>();
  action->request_cancel();
  ASSERT_TRUE(registry.register_action_submission("cycle", action));
  int cancels = 0;
  action->set_cancel_fn([&]() {++cancels;});
  EXPECT_EQ(cancels, 1);
}

TEST(CancellationRegistryTest, CallerCancelRacingRegistrationCancelsOnlyRegisteredSubmissionOnce)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("cycle", []() {});
  auto submission = std::make_shared<arm_cell_orchestration::CancellationRegistry::ActionSubmission>();
  std::atomic<int> cancels{0};
  submission->set_cancel_fn([&]() {++cancels;});
  std::mutex mutex;
  std::condition_variable condition;
  bool start = false;
  const auto wait = [&]() {
      std::unique_lock<std::mutex> lock(mutex);
      condition.wait(lock, [&]() {return start;});
    };
  std::thread registration([&]() {wait(); EXPECT_TRUE(registry.register_action_submission("cycle", submission));});
  std::thread cancellation([&]() {wait(); registry.request_caller_cancel("cycle");});
  {
    std::lock_guard<std::mutex> lock(mutex);
    start = true;
  }
  condition.notify_all();
  registration.join();
  cancellation.join();
  EXPECT_EQ(cancels.load(), 1);
}

TEST(CycleCoordinatorTest, HaltCancelsTheSubmissionOwnedByTheRunningLeaf)
{
  Fixture fixture;
  auto callbacks = fixture.make_callbacks();
  std::mutex mutex;
  std::condition_variable condition;
  bool started = false;
  bool canceled = false;
  int cancel_count = 0;
  callbacks.execute_task_scoped = [&](const ExecuteTask::Goal &,
      const Coordinator::MotionFeedbackFn &, const Coordinator::Submission & action) {
      action->set_cancel_fn([&]() {
          std::lock_guard<std::mutex> lock(mutex);
          ++cancel_count;
          canceled = true;
          condition.notify_all();
        });
      std::unique_lock<std::mutex> lock(mutex);
      started = true;
      condition.notify_all();
      condition.wait(lock, [&]() {return canceled;});
      return motion_result(MotionTaskResultCode::TASK_RESULT_CANCELED);
    };
  Coordinator coordinator(std::move(callbacks));
  std::thread worker([&]() {(void)coordinator.execute("target_fixture");});
  bool entered;
  {
    std::unique_lock<std::mutex> lock(mutex);
    entered = condition.wait_for(lock, std::chrono::seconds(2), [&]() {return started;});
  }
  coordinator.set_cancel_requested(true);
  worker.join();
  EXPECT_TRUE(entered);
  EXPECT_EQ(cancel_count, 1);
}

TEST(CycleCoordinatorTest, PnpSafetyHandoffContinuesWithNewHomeAndNoOldCancellation)
{
  Fixture fixture;
  auto callbacks = fixture.make_callbacks();
  allow_normal_batch_recovery(fixture, callbacks);
  Coordinator::Submission interrupted;
  int pnp_calls = 0;
  int homes = 0;
  int old_cancels = 0;
  int home_cancels = 0;
  callbacks.execute_pnp_scoped = [&](const Coordinator::ExecutePnp::Goal &,
      const Coordinator::PnpFeedbackFn &, const Coordinator::Submission & submission) {
      Coordinator::PnpResult result;
      if (++pnp_calls == 1) {
        interrupted = submission;
        interrupted->set_cancel_fn([&]() {++old_cancels;});
        result.safety_handoff = true;
        result.safety_preempted = true;
        fixture.safety = recovery_safety();
      } else {
        result.outcome = Coordinator::ExecutePnp::Result::PNP_OUTCOME_COMPLETED;
        result.material_placed = true;
        result.goal_uuid = "new-pnp";
        result.action_terminal = "SUCCEEDED";
      }
      return result;
    };
  callbacks.execute_task_scoped = [&](const ExecuteTask::Goal & goal,
      const Coordinator::MotionFeedbackFn &, const Coordinator::Submission & provided) {
      auto action = provided ? provided :
        std::make_shared<arm_cell_orchestration::CancellationRegistry::ActionSubmission>();
      action->set_cancel_fn([&]() {++home_cancels;});
      if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME && ++homes == 2) {
        interrupted->request_cancel();
      }
      return successful_motion_result(goal);
    };
  Coordinator coordinator(std::move(callbacks));
  const auto result = coordinator.execute("target_fixture");
  EXPECT_EQ(result.exit_reason.value, MissionExitReason::MISSION_EXIT_QUANTITY_REACHED);
  EXPECT_EQ(result.processed_count, 1U);
  EXPECT_EQ(pnp_calls, 2);
  EXPECT_EQ(homes, 2);
  EXPECT_EQ(old_cancels, 1);
  EXPECT_EQ(home_cancels, 0);
}

TEST(CancellationRegistryTest, SubmissionIdentityIsMonotonicAndScopedToCycle)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("cycle-a", []() {});
  registry.register_execution("cycle-b", []() {});
  auto first = register_test_submission(registry, "cycle-a");
  ASSERT_TRUE(first);
  EXPECT_EQ(first->submission_id(), "6379636c652d61:1");
  EXPECT_TRUE(first->goal_uuid().empty());
  registry.clear_active_action("cycle-a", first);
  auto second = register_test_submission(registry, "cycle-a");
  auto other = register_test_submission(registry, "cycle-b");
  ASSERT_TRUE(second);
  ASSERT_TRUE(other);
  EXPECT_EQ(second->submission_id(), "6379636c652d61:2");
  EXPECT_EQ(other->submission_id(), "6379636c652d62:1");
  EXPECT_TRUE(first->set_goal_uuid("actual-goal"));
  EXPECT_EQ(first->goal_uuid(), "actual-goal");
  EXPECT_TRUE(second->goal_uuid().empty());
}

TEST(CancellationRegistryTest, CancelIntentBeforeObserverIsRecordedExactlyOnce)
{
  arm_cell_orchestration::CancellationRegistry::ActionSubmission submission;
  int observations = 0;
  submission.request_cancel();
  submission.set_cancel_observer([&]() {++observations;});
  submission.request_cancel();
  EXPECT_EQ(observations, 1);
}
