#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "arm_cell_motion/fake_motion_backend.hpp"
#include "arm_cell_motion/motion_core.hpp"
#include "arm_cell_task_execution/task_executor.hpp"
#include "arm_cell_orchestration/cycle_coordinator.hpp"
#include "arm_cell_safety_fsm/safety_core.hpp"

namespace
{
using arm_cell_interfaces::action::ExecuteTask;
using arm_cell_interfaces::msg::MotionCapability;
using arm_cell_interfaces::msg::MotionStatus;
using arm_cell_interfaces::msg::MotionTaskResultCode;
using arm_cell_interfaces::msg::MotionTaskType;
using arm_cell_interfaces::msg::SafetyCauseSet;
using arm_cell_interfaces::msg::SafetyHardwareState;
using arm_cell_interfaces::msg::SafetyState;
using arm_cell_interfaces::msg::StopMode;
using Coordinator = arm_cell_orchestration::CycleCoordinator;
using Clock = arm_cell_motion::MotionCore::Clock;

arm_cell_motion::MotionGeometry synthetic_geometry()
{
  arm_cell_motion::MotionGeometry geometry;
  geometry.configured = true;
  geometry.observation_reference = "synthetic_profile_reference";
  geometry.observation_to_object.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.rotation.w = 1.0;
  geometry.insertion_axis_tcp.z = 1.0;
  geometry.pick_retract_distance_m = 0.2;
  return geometry;
}

const auto kFixture = std::string(__FILE__).substr(
  0, std::string(__FILE__).find_last_of("/\\")) +
  "/fixtures/full_cell_synthetic_scenarios.yaml";

struct Observation
{
  std::vector<std::string> events;
  std::vector<uint8_t> submitted_tasks;
  std::vector<uint8_t> stop_modes;
  uint8_t mission_exit{0};
  uint8_t last_motion_state{MotionStatus::MOTION_STATE_UNKNOWN};
  uint8_t normal_task_result{0};
  uint8_t retract_result{0};
  uint8_t recovery_capability{MotionCapability::MOTION_NONE};
  uint8_t selected_stop_mode{StopMode::STOP_MODE_CONTROLLED};
  bool latched_until_reset{false};
  bool reset_required{false};
};

struct SyntheticCell
{
  using SafetyCore = arm_cell_safety_fsm::SafetyCore;
  using MotionCore = arm_cell_motion::MotionCore;
  using FakeBackend = arm_cell_motion::FakeMotionBackend;

  std::shared_ptr<FakeBackend> backend{std::make_shared<FakeBackend>()};
  MotionCore motion{backend, std::chrono::milliseconds(500), synthetic_geometry()};
  std::unique_ptr<arm_cell_task_execution::TaskExecutor> task_executor;
  SafetyCore safety;
  Observation observation;
  Clock::time_point now{Clock::now()};
  arm_cell_safety_fsm::SafetyInputs inputs;
  SafetyState safety_state;
  std::string scenario;
  std::string vision_mode;
  std::string motion_mode;
  std::string safety_condition;
  std::string retract_mode;
  bool stop_injected{false};
  int vision_calls{0};

  SyntheticCell()
  {
    // Recovery scenarios begin from a known released state; an untouched fake
    // gripper now correctly reports UNKNOWN and must not authorize a retract.
    backend->gripper_port()->open();
    backend->set_complete_execution_on_return(true);
    task_executor = std::make_unique<arm_cell_task_execution::TaskExecutor>(
      std::shared_ptr<arm_cell_motion::MotionPort>(
        &motion, [](arm_cell_motion::MotionPort *) {}),
      backend->gripper_port(), synthetic_geometry());
    inputs.hardware.valid = true;
    inputs.motion.execution_state = MotionStatus::MOTION_STATE_IDLE;
    inputs.motion.backend_inactivity_confirmed = true;
    refresh_safety();
  }

  void event(const std::string & name)
  {
    observation.events.push_back(name);
  }

  void update_safety_inputs()
  {
    inputs.motion = motion.status();
    safety.update_hardware(inputs.hardware, now);
    safety.update_motion(inputs.motion, now);
  }

  void refresh_safety()
  {
    update_safety_inputs();
    safety_state = safety.evaluate(now + std::chrono::milliseconds(1));
    motion.update_safety_state(safety_state, now);
    observation.recovery_capability = safety_state.motion_capability.value;
    observation.selected_stop_mode = safety_state.selected_stop_mode.value;
    event("safety_state");
  }

  void complete_motion()
  {
    backend->set_execution_active(false);
    backend->set_inactivity_confirmed(true);
    motion.refresh_stop_state();
    event("motion_status_idle");
  }

  void inject_safety_condition()
  {
    event("external_input_change");
    if (safety_condition == "controlled_interlock" ||
      safety_condition == "immediate_interlock")
    {
      // Synthetic Safety publisher input for consumer behavior only. This does
      // not exercise SafetyCore cause or severity decisions.
      safety_state = safety.evaluate(now + std::chrono::milliseconds(1));
      safety_state.safety_state = SafetyState::SAFETY_STATE_INTERLOCKED;
      safety_state.motion_capability.value = MotionCapability::MOTION_NONE;
      safety_state.selected_stop_mode.value =
        safety_condition == "controlled_interlock" ?
        StopMode::STOP_MODE_CONTROLLED : StopMode::STOP_MODE_IMMEDIATE;
      motion.update_safety_state(safety_state, now);
      observation.selected_stop_mode = safety_state.selected_stop_mode.value;
      event("safety_state");
      return;
    } else if (safety_condition == "emergency_stop") {
      inputs.hardware.e_stop_active = true;
    } else if (safety_condition == "communication_loss") {
      inputs.motion = motion.status();
      safety.update_motion(inputs.motion, now);
      now += std::chrono::seconds(2);
      safety_state = safety.evaluate(now);
      motion.update_safety_state(safety_state, now);
      observation.selected_stop_mode = safety_state.selected_stop_mode.value;
      event("safety_state");
      return;
    }
    refresh_safety();
  }

  void complete_stop_and_recheck()
  {
    const auto selected_stop_mode = observation.selected_stop_mode;
    backend->set_execution_active(false);
    backend->set_inactivity_confirmed(true);
    motion.refresh_stop_state();
    observation.last_motion_state = motion.status().execution_state;
    event("motion_status_stopped");

    inputs.hardware.e_stop_active = false;
    now += std::chrono::milliseconds(1);
    refresh_safety();
    if (safety_condition != "emergency_stop") {
      // Synthetic recovery authorization for consumer behavior only;
      // SafetyCore transition behavior is covered by its own tests.
      safety_state.safety_state = SafetyState::SAFETY_STATE_RECOVERY_REQUIRED;
      safety_state.motion_capability.value = MotionCapability::MOTION_RECOVERY_ONLY;
      motion.update_safety_state(safety_state, now);
      observation.recovery_capability = safety_state.motion_capability.value;
    }
    observation.selected_stop_mode = selected_stop_mode;
    event("safety_recovery_capability");
  }

  Coordinator::MotionResult execute_task(const ExecuteTask::Goal & goal)
  {
    const auto task = goal.task_type.value;
    observation.submitted_tasks.push_back(task);
    event(task == MotionTaskType::TASK_TYPE_RETRACT ? "retract_submit" : "motion_submit");

    if (task == MotionTaskType::TASK_TYPE_RETRACT) {
      if (retract_mode == "path_obstructed") {
        backend->set_task_result(MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
      }
      const auto result = task_executor->execute(
        goal, task_executor->object_state(),
        std::to_string(observation.submitted_tasks.size()));
      observation.retract_result = result;
      if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        complete_motion();
      }
      Coordinator::MotionResult motion_result;
      motion_result.code.value = result;
      motion_result.diagnostic_detail = "synthetic retract result";
      motion_result.task_type = task;
      motion_result.motion_execution_id = std::string(31, '0') + std::to_string(observation.submitted_tasks.size());
      motion_result.object_disposition = task_executor->object_disposition();
      motion_result.action_terminal = result == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
        "SUCCEEDED" : "ABORTED";
      motion_result.pick_grasp_insertion_started = task_executor->pick_grasp_insertion_started();
      motion_result.pick_grasp_insertion_started_available = true;
      return motion_result;
    }

    if (motion_mode == "backend_error") {
      backend->set_task_result(MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
    }
    const auto result = task_executor->execute(
      goal, task_executor->object_state(),
      std::to_string(observation.submitted_tasks.size()));
    observation.normal_task_result = result;
    if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      Coordinator::MotionResult motion_result;
      motion_result.code.value = result;
      motion_result.diagnostic_detail = "synthetic motion result";
      motion_result.task_type = task;
      motion_result.motion_execution_id = std::string(31, '0') + std::to_string(observation.submitted_tasks.size());
      motion_result.object_disposition = task_executor->object_disposition();
      motion_result.action_terminal = result == MotionTaskResultCode::TASK_RESULT_SUCCESS ?
        "SUCCEEDED" : "ABORTED";
      motion_result.pick_grasp_insertion_started = task_executor->pick_grasp_insertion_started();
      motion_result.pick_grasp_insertion_started_available = true;
      return motion_result;
    }

    if (!safety_condition.empty() && !stop_injected) {
      stop_injected = true;
      inject_safety_condition();
      // Direct Safety->Motion dispatch is proven by the real node integration
      // tests.  This core-only harness uses the production MotionCore stop API
      // only to establish the already-interrupted state needed to continue
      // deterministic recovery/result assertions; it does not claim dispatch
      // independence or record a synthetic direct-stop ordering.
      const bool stop_accepted = motion.request_stop(
        arm_cell_motion::make_uuid(200), observation.selected_stop_mode);
      EXPECT_TRUE(stop_accepted);
      complete_stop_and_recheck();
      Coordinator::MotionResult motion_result;
      motion_result.code.value = MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
      motion_result.diagnostic_detail = "synthetic safety preemption";
      motion_result.task_type = task;
      motion_result.motion_execution_id = std::string(31, '0') +
        std::to_string(observation.submitted_tasks.size());
      motion_result.object_disposition = task_executor->object_disposition();
      motion_result.pick_grasp_insertion_started = task_executor->pick_grasp_insertion_started();
      motion_result.pick_grasp_insertion_started_available = true;
      motion_result.action_terminal = "ABORTED";
      return motion_result;
    }

    complete_motion();
    Coordinator::MotionResult motion_result;
    motion_result.code.value = MotionTaskResultCode::TASK_RESULT_SUCCESS;
    motion_result.diagnostic_detail = "synthetic motion success";
    motion_result.task_type = task;
    motion_result.motion_execution_id = std::string(31, '0') +
      std::to_string(observation.submitted_tasks.size());
    motion_result.object_disposition = task_executor->object_disposition();
    motion_result.pick_grasp_insertion_started = task_executor->pick_grasp_insertion_started();
    motion_result.pick_grasp_insertion_started_available = true;
    motion_result.action_terminal = "SUCCEEDED";
    return motion_result;
  }

  Coordinator make_coordinator()
  {
    Coordinator::Callbacks callbacks;
    callbacks.read_safety = [this]() {
        return arm_cell_orchestration::evaluate_safety_observation(
          safety_state, true, now, now, std::chrono::milliseconds(500));
      };
    callbacks.execute_task = [this](const ExecuteTask::Goal & goal) {
        return execute_task(goal);
      };
    callbacks.publish_phase = [this](uint8_t) {event("mission_phase");};
    callbacks.load_recipe = [](const std::string &) {
        Coordinator::MissionRecipe recipe;
        recipe.has_grasp_width_policy = true;
        recipe.grasp_width_min_mm = 40.0F;
        recipe.grasp_width_max_mm = 80.0F;
        recipe.pick_approach_axis =
          arm_cell_orchestration::PickApproachAxis::TCP_INSERTION_AXIS;
        recipe.has_grasp_yaw = true;
        recipe.grasp_yaw_rad = 0.0;
        recipe.pick_approach_distance_m = 0.02;
        recipe.pick_grasp_position_tolerance_m = 0.01;
        recipe.pick_grasp_orientation_tolerance_rad = 0.2;
        recipe.has_unobserved_yaw_policy = true;
        recipe.unobserved_yaw_policy =
          arm_cell_orchestration::UnobservedYawPolicy::FORBIDDEN;
        recipe.has_place_pose = true;
        recipe.place_pose.header.frame_id = "base_link";
        recipe.place_pose.pose.orientation.w = 1.0;
        recipe.place_approach_direction_object.z = 1.0;
        recipe.place_approach_distance_m = 0.015;
        recipe.retract_distance_m = 0.05;
        return std::optional<Coordinator::MissionRecipe>{recipe};
      };
    callbacks.detect_target = [this](const Coordinator::DetectTarget::Request &) {
        ++vision_calls;
        Coordinator::DetectTarget::Response response;
        if (vision_mode == "timeout") {
          response.result_code.value =
            arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT;
        } else if (vision_mode == "object_not_found" ||
          (vision_mode == "target_then_depleted" && vision_calls > 1))
        {
          response.result_code.value =
            arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND;
        } else {
          response.result_code.value =
            arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
          arm_cell_interfaces::msg::PickCandidate candidate;
          response.acquisition_id = "synthetic-acquisition";
          candidate.candidate_id = "candidate-1";
          candidate.target_pose.header.frame_id = "base_link";
          candidate.target_pose.pose.orientation.w = 1.0;
          candidate.orientation_symmetry_order = 4;
          candidate.has_estimated_height = true;
          candidate.estimated_height_m = 0.12F;
          candidate.has_observed_rectangle = true;
          candidate.rectangle_long_side_m = 0.07F;
          candidate.rectangle_short_side_m = 0.07F;
          candidate.acquisition_id = response.acquisition_id;
          response.material_candidates = {candidate};
        }
        return response;
      };
    callbacks.wait_for_recovery = [this]() {
        Coordinator::RecoveryObservation observation;
        observation.safety = arm_cell_orchestration::evaluate_safety_observation(
          safety_state, true, now, now, std::chrono::milliseconds(500));
        observation.motion = motion.status();
        return observation;
      };
    return Coordinator(std::move(callbacks));
  }
};

void expect_vr(const YAML::Node & scenario, const std::string & vr)
{
  const auto requirements = scenario["vr"];
  bool found = false;
  for (const auto & requirement : requirements) {
    found = found || requirement.as<std::string>() == vr;
  }
  EXPECT_TRUE(found)
    << "missing scenario-to-assertion VR mapping: " << vr;
}

void run_scenario(const YAML::Node & scenario)
{
  SyntheticCell cell;
  cell.scenario = scenario["name"].as<std::string>();
  const auto stimulus = scenario["stimulus"];
  const auto expected = scenario["expected"];
  if (stimulus["vision"]) {
    cell.vision_mode = stimulus["vision"].as<std::string>();
  }
  if (stimulus["motion_result"]) {
    cell.motion_mode = stimulus["motion_result"].as<std::string>();
  }
  if (stimulus["safety_condition"]) {
    cell.safety_condition = stimulus["safety_condition"].as<std::string>();
  }
  if (stimulus["external_fault"]) {
    cell.safety_condition = stimulus["external_fault"].as<std::string>();
  }
  if (stimulus["retract_result"] && stimulus["retract_result"].as<std::string>() != "success") {
    cell.retract_mode = stimulus["retract_result"].as<std::string>();
  }

  if (cell.scenario == "communication_loss") {
    cell.inputs.motion.execution_state = MotionStatus::MOTION_STATE_EXECUTING;
    cell.inputs.motion.execution_active = true;
    cell.inputs.motion.has_active_execution = true;
    cell.inputs.motion.backend_inactivity_confirmed = false;
    cell.safety.update_hardware(cell.inputs.hardware, cell.now);
    cell.safety.update_motion(cell.inputs.motion, cell.now);
    cell.now += std::chrono::seconds(2);
    cell.safety_state = cell.safety.evaluate(cell.now);
    EXPECT_EQ(
      cell.safety_state.motion_capability.value,
      expected["motion_capability_during_loss"].as<std::string>() == "none" ?
      MotionCapability::MOTION_NONE : MotionCapability::MOTION_NORMAL);
    EXPECT_EQ(
      cell.safety_state.required_inputs_fresh,
      expected["required_inputs_fresh_during_loss"].as<bool>());
    EXPECT_EQ(
      cell.safety_state.latched_causes.value != SafetyCauseSet::NONE,
      expected["latched_until_reset"].as<bool>());

    cell.inputs.motion.execution_state = MotionStatus::MOTION_STATE_STOPPED;
    cell.inputs.motion.execution_active = false;
    cell.inputs.motion.has_active_execution = false;
    cell.inputs.motion.backend_inactivity_confirmed = true;
    cell.now += std::chrono::milliseconds(1);
    cell.safety.update_hardware(cell.inputs.hardware, cell.now);
    cell.safety.update_motion(cell.inputs.motion, cell.now);
    cell.safety_state = cell.safety.evaluate(cell.now + std::chrono::milliseconds(1));
    EXPECT_EQ(
      cell.safety_state.motion_capability.value,
      expected["recovery_capability_after_restore"].as<std::string>() == "normal" ?
      MotionCapability::MOTION_NORMAL : MotionCapability::MOTION_NONE);
    EXPECT_EQ(
      cell.safety_state.required_inputs_fresh,
      expected["required_inputs_fresh_after_restore"].as<bool>());
    EXPECT_EQ(
      cell.safety_state.latched_causes.value != SafetyCauseSet::NONE,
      expected["latched_until_reset"].as<bool>());
    EXPECT_EQ(cell.observation.submitted_tasks.size(), 0U);
    return;
  }

  if (cell.scenario == "capability_downgrade") {
    // Inject current Safety authority as a consumer input; this case checks
    // Motion enforcement and does not test SafetyCore's recovery transition.
    SafetyState recovery = cell.safety_state;
    recovery.motion_capability.value = MotionCapability::MOTION_RECOVERY_ONLY;
    recovery.safety_state = SafetyState::SAFETY_STATE_RECOVERY_REQUIRED;
    cell.motion.update_safety_state(recovery, cell.now);
    const auto denied = cell.motion.executeLegacyMovement(
      MotionTaskType::TASK_TYPE_GO_HOME, "00000000000000000000000000000001", "gate-normal");
    EXPECT_EQ(denied, MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
    const auto permitted = cell.motion.executeLegacyMovement(
      MotionTaskType::TASK_TYPE_RETRACT, "00000000000000000000000000000002", "gate-retract");
    EXPECT_EQ(permitted, MotionTaskResultCode::TASK_RESULT_SUCCESS);
    EXPECT_EQ(
      denied, expected["normal_task_result"].as<std::string>() == "permission_denied" ?
      MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED : MotionTaskResultCode::TASK_RESULT_SUCCESS);
    EXPECT_EQ(
      permitted, expected["retract_result"].as<std::string>() == "permitted" ?
      MotionTaskResultCode::TASK_RESULT_SUCCESS : MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
    expect_vr(scenario, "VR-ICD-SAFE-MOT-02");
    return;
  }

  if (cell.scenario == "mission_cancellation") {
    auto coordinator = cell.make_coordinator();
    coordinator.set_cancel_requested(true);
    const auto result = coordinator.execute("synthetic-target");
    cell.observation.mission_exit = result.exit_reason.value;
    EXPECT_EQ(
      result.exit_reason.value,
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_CANCELED);
    EXPECT_EQ(cell.observation.submitted_tasks.size(), 0U);
    expect_vr(scenario, "VR-ICD-ORCH-CYCLE-03");
    return;
  }

  auto coordinator = cell.make_coordinator();
  const auto result = coordinator.execute("synthetic-target");
  cell.observation.mission_exit = result.exit_reason.value;

  if (expected["mission_exit"]) {
    const auto mission = expected["mission_exit"].as<std::string>();
    const auto expected_reason =
      mission == "depleted" ? arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_DEPLETED :
      mission == "quantity_reached" ?
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_QUANTITY_REACHED :
      mission == "quantity_shortfall" ?
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_QUANTITY_SHORTFALL :
      mission == "vision_error" ? arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_VISION_ERROR :
      mission == "motion_error" ? arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_MOTION_ERROR :
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED;
    EXPECT_EQ(result.exit_reason.value, expected_reason);
  }
  if (expected["motion_submissions"]) {
    EXPECT_EQ(
      cell.observation.submitted_tasks.size(), expected["motion_submissions"].as<std::size_t>());
  }
  if (expected["retract_count"]) {
    EXPECT_EQ(
      std::count(cell.observation.submitted_tasks.begin(), cell.observation.submitted_tasks.end(),
      MotionTaskType::TASK_TYPE_RETRACT), expected["retract_count"].as<int>());
  }
  if (expected["recovery_result"]) {
    const auto recovery_result = expected["recovery_result"].as<std::string>();
    const auto expected_code = recovery_result == "path_obstructed" ?
      MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED :
      MotionTaskResultCode::TASK_RESULT_SUCCESS;
    EXPECT_EQ(cell.observation.retract_result, expected_code);
  }
  if (expected["retry_count"]) {
    EXPECT_EQ(
      std::count(cell.observation.events.begin(), cell.observation.events.end(), "retract_submit"),
      expected["retry_count"].as<int>() + 1);
  }

  if (cell.scenario == "target_depletion" || cell.scenario == "vision_error") {
    expect_vr(scenario, "VR-ICD-VIS-ORCH-02");
  }

  if (!cell.safety_condition.empty()) {
    expect_vr(scenario, "VR-ICD-ORCH-CYCLE-03");
    if (expected["terminal_motion_state"]) {
      const auto state = expected["terminal_motion_state"].as<std::string>();
      if (state == "stopped") {
        EXPECT_EQ(cell.observation.last_motion_state, MotionStatus::MOTION_STATE_STOPPED);
      } else if (state == "emergency_stopped") {
        EXPECT_EQ(
          cell.observation.last_motion_state, MotionStatus::MOTION_STATE_EMERGENCY_STOPPED);
      }
    }
    if (expected["retract_count"].as<int>() == 1) {
      const auto stopped_index = std::find(
        cell.observation.events.begin(), cell.observation.events.end(), "motion_status_stopped");
      const auto retract_index = std::find(
        cell.observation.events.begin(), cell.observation.events.end(), "retract_submit");
      ASSERT_NE(stopped_index, cell.observation.events.end());
      ASSERT_NE(retract_index, cell.observation.events.end());
      EXPECT_LT(stopped_index, retract_index);
      expect_vr(scenario, "VR-ICD-SAFE-MOT-06");
    }
  }
}
}  // namespace

TEST(SyntheticFullCellTest, ExecutesEveryFixtureScenarioThroughProductionSeams)
{
  const auto document = YAML::LoadFile(kFixture);
  ASSERT_EQ(document["schema_version"].as<int>(), 2);
  for (const auto & scenario : document["scenarios"]) {
    SCOPED_TRACE(scenario["name"].as<std::string>());
    run_scenario(scenario);
  }
}
