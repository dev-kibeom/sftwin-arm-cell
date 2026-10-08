#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "arm_cell_motion_moveit2/fake_motion_backend.hpp"
#include "arm_cell_motion_moveit2/motion_core.hpp"
#include "arm_cell_orchestration_bt/cycle_coordinator.hpp"
#include "arm_cell_safety_fsm/safety_core.hpp"

namespace
{
using arm_cell_interfaces::action::ExecuteTask;
using arm_cell_interfaces::msg::AMRDockingState;
using arm_cell_interfaces::msg::MotionCapability;
using arm_cell_interfaces::msg::MotionStatus;
using arm_cell_interfaces::msg::MotionTaskResultCode;
using arm_cell_interfaces::msg::MotionTaskType;
using arm_cell_interfaces::msg::PackMLState;
using arm_cell_interfaces::msg::SafetyCauseSet;
using arm_cell_interfaces::msg::SafetyHardwareState;
using arm_cell_interfaces::msg::SafetyState;
using arm_cell_interfaces::msg::StopMode;
using Coordinator = arm_cell_orchestration_bt::CycleCoordinator;
using Clock = arm_cell_motion_moveit2::MotionCore::Clock;

arm_cell_motion_moveit2::MotionGeometry synthetic_geometry()
{
  arm_cell_motion_moveit2::MotionGeometry geometry;
  geometry.configured = true;
  geometry.observation_reference = "synthetic_profile_reference";
  geometry.observation_to_object.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.rotation.w = 1.0;
  geometry.insertion_axis_tcp.z = 1.0;
  geometry.pick_approach_distance_m = 0.1;
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
  using MotionCore = arm_cell_motion_moveit2::MotionCore;
  using FakeBackend = arm_cell_motion_moveit2::FakeMotionBackend;

  std::shared_ptr<FakeBackend> backend{std::make_shared<FakeBackend>()};
  MotionCore motion{backend, std::chrono::milliseconds(500), synthetic_geometry()};
  SafetyCore safety;
  Observation observation;
  Clock::time_point now{};
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
    inputs.amr.valid = true;
    inputs.amr.docking_state = AMRDockingState::AMR_DOCKING_DOCKED;
    inputs.packml.valid = true;
    inputs.packml.state = PackMLState::PACKML_STATE_IDLE;
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
    safety.update_amr(inputs.amr, now);
    safety.update_packml(inputs.packml, now);
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
    if (safety_condition == "premature_undock") {
      inputs.amr.docking_state = AMRDockingState::AMR_DOCKING_UNDOCKED;
      inputs.packml.state = PackMLState::PACKML_STATE_EXECUTE;
    } else if (safety_condition == "packml_abort") {
      inputs.packml.state = PackMLState::PACKML_STATE_ABORTED;
    } else if (safety_condition == "emergency_stop") {
      inputs.hardware.e_stop_active = true;
    } else if (safety_condition == "communication_loss") {
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
    backend->set_execution_active(false);
    backend->set_inactivity_confirmed(true);
    motion.refresh_stop_state();
    observation.last_motion_state = motion.status().execution_state;
    event("motion_status_stopped");

    inputs.amr.docking_state = AMRDockingState::AMR_DOCKING_DOCKED;
    inputs.packml.state = PackMLState::PACKML_STATE_IDLE;
    inputs.hardware.e_stop_active = false;
    now += std::chrono::milliseconds(1);
    refresh_safety();
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
      const auto result = motion.accept_goal(
        arm_cell_motion_moveit2::make_uuid(static_cast<uint8_t>(observation.submitted_tasks.size())),
        goal, now);
      observation.retract_result = result;
      if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        complete_motion();
      }
      Coordinator::MotionResult motion_result;
      motion_result.code.value = result;
      motion_result.diagnostic_detail = "synthetic retract result";
      return motion_result;
    }

    if (motion_mode == "backend_error") {
      backend->set_task_result(MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
    }
    const auto result = motion.accept_goal(
      arm_cell_motion_moveit2::make_uuid(static_cast<uint8_t>(observation.submitted_tasks.size())),
      goal, now);
    observation.normal_task_result = result;
    if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      Coordinator::MotionResult motion_result;
      motion_result.code.value = result;
      motion_result.diagnostic_detail = "synthetic motion result";
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
        arm_cell_motion_moveit2::make_uuid(200), observation.selected_stop_mode);
      EXPECT_TRUE(stop_accepted);
      complete_stop_and_recheck();
      Coordinator::MotionResult motion_result;
      motion_result.code.value = MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
      motion_result.diagnostic_detail = "synthetic safety preemption";
      return motion_result;
    }

    complete_motion();
    Coordinator::MotionResult motion_result;
    motion_result.code.value = MotionTaskResultCode::TASK_RESULT_SUCCESS;
    motion_result.diagnostic_detail = "synthetic motion success";
    return motion_result;
  }

  Coordinator make_coordinator()
  {
    Coordinator::Callbacks callbacks;
    callbacks.read_safety = [this]() {return safety_state;};
    callbacks.execute_task = [this](const ExecuteTask::Goal & goal) {
        return execute_task(goal);
      };
    callbacks.cancel_motion = [this]() {event("orchestration_cancel");};
    callbacks.publish_phase = [this](uint8_t) {event("mission_phase");};
    callbacks.load_recipe = [](const std::string &) {
        Coordinator::MissionRecipe recipe;
        recipe.has_grasp_width = true;
        recipe.grasp_width_mm = 42.0F;
        recipe.has_grasp_yaw = true;
        recipe.grasp_yaw_rad = 0.0;
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
          response.has_target_pose = true;
          response.target_pose.header.frame_id = "base_link";
          response.target_pose.pose.orientation.w = 1.0;
          response.has_target_yaw = true;
          response.has_estimated_height = true;
          response.estimated_height_m = 0.12F;
        }
        return response;
      };
    callbacks.wait_for_recovery = [this]() {
        Coordinator::RecoveryObservation observation;
        observation.safety = safety_state;
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
  if (stimulus["external_state"]) {
    const auto external_state = stimulus["external_state"].as<std::string>();
    if (external_state == "undocked_while_execute") {
      cell.safety_condition = "premature_undock";
    } else if (external_state == "packml_aborted") {
      cell.safety_condition = "packml_abort";
    }
  }
  if (stimulus["external_fault"]) {
    cell.safety_condition = stimulus["external_fault"].as<std::string>();
  }
  if (stimulus["retract_result"] && stimulus["retract_result"].as<std::string>() != "success") {
    cell.retract_mode = stimulus["retract_result"].as<std::string>();
  }

  if (cell.scenario == "capability_downgrade") {
    SafetyState recovery = cell.safety_state;
    recovery.motion_capability.value = MotionCapability::MOTION_RECOVERY_ONLY;
    recovery.safety_state = SafetyState::SAFETY_STATE_RECOVERY_REQUIRED;
    cell.motion.update_safety_state(recovery, cell.now);
    ExecuteTask::Goal normal;
    normal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
    const auto denied = cell.motion.accept_goal(arm_cell_motion_moveit2::make_uuid(1), normal, cell.now);
    EXPECT_EQ(denied, MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
    ExecuteTask::Goal retract;
    retract.task_type.value = MotionTaskType::TASK_TYPE_RETRACT;
    const auto permitted = cell.motion.accept_goal(
      arm_cell_motion_moveit2::make_uuid(2), retract, cell.now);
    EXPECT_EQ(permitted, MotionTaskResultCode::TASK_RESULT_SUCCESS);
    EXPECT_EQ(
      denied, expected["normal_task_result"].as<std::string>() == "permission_denied" ?
      MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED : MotionTaskResultCode::TASK_RESULT_SUCCESS);
    EXPECT_EQ(
      permitted, expected["retract_result"].as<std::string>() == "permitted" ?
      MotionTaskResultCode::TASK_RESULT_SUCCESS : MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
    expect_vr(scenario, "VR-ICD-SAFE-MOT-02");
    expect_vr(scenario, "VR-ICD-SAFE-ORCH-03");
    return;
  }

  if (cell.scenario == "external_clear_without_safety_reset") {
    cell.safety_condition = "communication_loss";
    cell.inject_safety_condition();
    EXPECT_NE(cell.safety_state.latched_causes.value, SafetyCauseSet::NONE);
    cell.inputs.amr.valid = true;
    cell.inputs.packml.valid = true;
    cell.inputs.hardware.valid = true;
    cell.inputs.motion = cell.motion.status();
    cell.now += std::chrono::milliseconds(1);
    cell.refresh_safety();
    cell.observation.latched_until_reset = cell.safety_state.latched_causes.value != SafetyCauseSet::NONE;
    cell.observation.reset_required = !cell.safety.reset(false, cell.now);
    EXPECT_EQ(cell.observation.latched_until_reset, expected["latched_until_reset"].as<bool>());
    EXPECT_EQ(cell.observation.reset_required, expected["reset_required"].as<bool>());
    expect_vr(scenario, "VR-ICD-EXT-SAFE-02");
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
    EXPECT_EQ(
      std::count(cell.observation.events.begin(), cell.observation.events.end(),
      "orchestration_cancel"), 1);
    expect_vr(scenario, "VR-ICD-ORCH-CYCLE-02");
    return;
  }

  auto coordinator = cell.make_coordinator();
  const auto result = coordinator.execute("synthetic-target");
  cell.observation.mission_exit = result.exit_reason.value;

  if (expected["mission_exit"]) {
    const auto mission = expected["mission_exit"].as<std::string>();
    const auto expected_reason =
      mission == "depleted" ? arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_DEPLETED :
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

  if (!cell.safety_condition.empty()) {
    if (expected["selected_stop_mode"]) {
      const auto mode = expected["selected_stop_mode"].as<std::string>();
      const auto expected_mode =
        mode == "controlled" ? StopMode::STOP_MODE_CONTROLLED :
        mode == "immediate" ? StopMode::STOP_MODE_IMMEDIATE :
        StopMode::STOP_MODE_EMERGENCY;
      EXPECT_EQ(cell.observation.selected_stop_mode, expected_mode);
    }
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
