#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <arm_cell_interfaces/msg/motion_task_phase.hpp>
#include <arm_cell_interfaces/action/execute_pnp.hpp>
#include <nlohmann/json.hpp>
#include <rcutils/logging.h>

#include "arm_cell_task_execution/task_executor.hpp"
#include "legacy_backend_motion_port.hpp"

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;

namespace
{
struct CapturedLog
{
  int severity;
  std::string format;
};

std::vector<CapturedLog> captured_logs;

void capture_log(
  const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
  const char * format, va_list *)
{
  captured_logs.push_back(CapturedLog{severity, format ? format : ""});
}

class ScopedTaskExecutorLogCapture final
{
public:
  ScopedTaskExecutorLogCapture()
  : previous_handler_(rcutils_logging_get_output_handler()),
    previous_level_(rcutils_logging_get_logger_level("task_executor"))
  {
    captured_logs.clear();
    rcutils_logging_set_output_handler(capture_log);
    const auto result = rcutils_logging_set_logger_level(
      "task_executor", RCUTILS_LOG_SEVERITY_INFO);
    EXPECT_EQ(result, RCUTILS_RET_OK);
  }

  ~ScopedTaskExecutorLogCapture()
  {
    const auto result = rcutils_logging_set_logger_level("task_executor", previous_level_);
    EXPECT_EQ(result, RCUTILS_RET_OK);
    rcutils_logging_set_output_handler(previous_handler_);
  }

  void set_level(int severity)
  {
    captured_logs.clear();
    const auto result = rcutils_logging_set_logger_level("task_executor", severity);
    EXPECT_EQ(result, RCUTILS_RET_OK);
  }

private:
  rcutils_logging_output_handler_t previous_handler_;
  int previous_level_;
};

class ScopedEnvironmentVariable final
{
public:
  explicit ScopedEnvironmentVariable(const char * name)
  : name_(name)
  {
    const auto * value = std::getenv(name);
    if (value != nullptr) {previous_ = value;}
  }

  ~ScopedEnvironmentVariable()
  {
    if (previous_) {
      ::setenv(name_.c_str(), previous_->c_str(), 1);
    } else {
      ::unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  std::optional<std::string> previous_;
};

class RecordingBackend final : public MotionBackend
{
public:
  bool available() const override {return true;}
  bool diagnostics_enabled = true;
  bool reflect_clearance_pose_in_measurement = true;
  bool throw_on_diagnostic_finalize = false;
  std::string diagnostic_failure_stage;
  std::string diagnostic_failure_detail;
  std::vector<MaterialCandidateSearchAttempt> material_candidate_attempts;
  std::string material_candidate_outcome;
  void record_material_candidate_search(
    const std::vector<MaterialCandidateSearchAttempt> & attempts,
    const std::string & outcome) override
  {material_candidate_attempts = attempts;material_candidate_outcome = outcome;}
  void begin_pick_candidate_search() override {++candidate_search_starts;}
  bool submit(const ExecuteTask::Goal &) override {return true;}

  uint8_t execute_normalized(const NormalizedMotionRequest & request) override
  {
    requests.push_back(request);
    if (request.target_is_tcp_pose) {waypoint_events.push_back("clearance");}
    if (request.place_waypoint_approach) {waypoint_events.push_back("cached_approach");}
    events.push_back("motion:" + std::to_string(request.phase));
    if (request.task_type.value == MotionTaskType::TASK_TYPE_PICK &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING &&
      fail_pick_retract)
    {
      return MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
    }
    if (request.task_type.value == MotionTaskType::TASK_TYPE_PLACE &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING &&
      place_retreat_failures_remaining > 0)
    {
      --place_retreat_failures_remaining;
      last_place_stage = "place_retreat_execution";
      return MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
    }
    if (request.task_type.value == MotionTaskType::TASK_TYPE_PICK &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING &&
      !pick_approach_results.empty())
    {
      const auto result = pick_approach_results.front();
      pick_approach_results.pop_front();
      if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {return result;}
    }
    if (request.task_type.value == MotionTaskType::TASK_TYPE_PLACE) {
      const bool approach = request.phase ==
        arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
      if ((approach && fail_place_approach_planning) ||
        (!approach && fail_place_insertion_planning))
      {
        last_place_stage = approach ? "place_approach_planning" : "place_insertion_planning";
        return MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
      }
    }
    if (request.task_type.value == MotionTaskType::TASK_TYPE_PICK &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING)
    {
      geometry_msgs::msg::Transform relation;
      relation.rotation.w = 1.0;
      stage_selected_pick_grasp_relation(relation);
    }
    {
      std::unique_lock<std::mutex> lock(completion_mutex);
      ++motion_call_count;
      completion_condition.notify_all();
      if (block_motion) {
        completion_condition.wait(lock, [this]() {return release_motion;});
        release_motion = false;
      }
    }
    tcp_x_values.push_back(request.tcp_target.pose.position.x);
    tcp_y_values.push_back(request.tcp_target.pose.position.y);
    tcp_z_values.push_back(request.tcp_target.pose.position.z);
    tcp_orientations.push_back(request.tcp_target.pose.orientation);
    approach_z_values.push_back(request.approach_target.pose.position.z);
    retract_x_values.push_back(request.retract_target.pose.position.x);
    if (request.task_type.value == MotionTaskType::TASK_TYPE_PICK &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING &&
      fail_first_pick_final_plan && pick_final_calls++ == 0)
    {
      candidate_fallback_available = true;
      return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    }
    if (request.task_type.value == MotionTaskType::TASK_TYPE_PICK &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING &&
      fail_first_pick_final_without_candidate && pick_final_without_candidate_calls++ == 0)
    {
      return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    }
    if (request.task_type.value == MotionTaskType::TASK_TYPE_PICK &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING)
    {
      if (reject_pick_insertion_before_submission) {
        return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
      }
      insertion_submission_uncertain = true;
      if (throw_pick_insertion_after_submission) {
        throw std::runtime_error("uncertain insertion submission result");
      }
      if (fail_pick_insertion_after_submission) {
        return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
      }
    }
    if (motion_result == MotionTaskResultCode::TASK_RESULT_SUCCESS &&
      request.target_is_tcp_pose && reflect_clearance_pose_in_measurement)
    {
      measured_tcp_state = request.tcp_target;
    }
    return motion_result;
  }


  bool measured_tcp_pose(geometry_msgs::msg::PoseStamped & pose) const override
  {
    pose = measured_tcp_state;
    return measured_tcp_state.header.frame_id == "base_link";
  }

  bool advance_to_next_planning_candidate() override
  {
    if (!candidate_fallback_available) {return false;}
    candidate_fallback_available = false;
    ++candidate_advance_calls;
    return true;
  }

  uint8_t execute_task(const ExecuteTask::Goal & goal) override
  {
    events.push_back(goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME ? "home" : "task");
    if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME) {
      waypoint_events.push_back("home");
    }
    return goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME ? home_result :
           MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  uint8_t execute_joint_target() override
  {
    events.push_back("tray");
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  std::string last_place_failure_stage() const override {return last_place_stage;}
  bool retry_place_retreat() override
  {
    if (place_retreat_retries_remaining == 0) {return false;}
    --place_retreat_retries_remaining;
    return true;
  }

  void reset_pick_grasp_insertion_started() override
  {insertion_submission_uncertain = false;}

  bool pick_grasp_insertion_started() const override
  {return insertion_submission_uncertain;}

  std::string last_pick_candidate_evaluation() const override
  {return pick_candidate_evaluation;}

  uint8_t command_gripper(float width_mm) override
  {
    events.push_back("gripper");
    gripper_widths.push_back(width_mm);
    return gripper_result;
  }

  bool gripper_active() const override {return gripper_active_value;}

  void stop_gripper() override {++stop_gripper_calls;}

  void record_gripper_command(bool opening)
  {
    std::lock_guard<std::mutex> lock(completion_mutex);
    ++gripper_command_count;
    if (opening) {
      ++gripper_open_command_count;
    } else {
      ++gripper_close_command_count;
    }
    completion_condition.notify_all();
  }

  void release_current_motion()
  {
    std::lock_guard<std::mutex> lock(completion_mutex);
    release_motion = true;
    completion_condition.notify_all();
  }

  bool wait_for_motion_call(int expected)
  {
    std::unique_lock<std::mutex> lock(completion_mutex);
    return completion_condition.wait_for(
      lock, std::chrono::seconds(1), [this, expected]() {
        return motion_call_count >= expected;
      });
  }

  bool wait_for_gripper_commands(
    int expected_total, int expected_opens = 0, int expected_closes = 0,
    std::chrono::milliseconds timeout = std::chrono::seconds(1))
  {
    std::unique_lock<std::mutex> lock(completion_mutex);
    return completion_condition.wait_for(
      lock, timeout, [this, expected_total, expected_opens, expected_closes]() {
        return gripper_command_count >= expected_total &&
               gripper_open_command_count >= expected_opens &&
               gripper_close_command_count >= expected_closes;
      });
  }

  void cancel() override {}
  void hold() override {}
  void stop() override {}
  bool execution_active() const override {return false;}
  bool backend_inactivity_confirmed() const override {return true;}

  void finalize_motion_planning_diagnostic(uint8_t result_code) override
  {
    finalized_planning_results.push_back(result_code);
  }

  void finalize_motion_planning_diagnostic(
    uint8_t result_code, double elapsed_s, const std::string & failure_stage,
    const std::string & failure_detail) override
  {
    (void)elapsed_s;
    if (!diagnostics_enabled) {return;}
    finalized_planning_results.push_back(result_code);
    diagnostic_failure_stage = failure_stage;
    diagnostic_failure_detail = failure_detail;
    if (throw_on_diagnostic_finalize) {throw std::runtime_error("diagnostic write failed");}
  }

  void begin_pick_grasp_relation() override
  {
    pending_relation = false;
    accepted_relation = false;
    ++begin_relation_count;
  }

  void stage_selected_pick_grasp_relation(
    const geometry_msgs::msg::Transform &) override
  {
    pending_relation = true;
  }

  void discard_pending_grasp_relation() override
  {
    pending_relation = false;
    ++discard_relation_count;
  }

  bool commit_pending_grasp_relation() override
  {
    if (!pending_relation) {
      return false;
    }
    pending_relation = false;
    accepted_relation = true;
    ++promote_relation_count;
    return true;
  }

  void clear_grasp_relation_after_release() override
  {
    pending_relation = false;
    accepted_relation = false;
    ++release_relation_count;
  }

  std::vector<std::string> events;
  std::vector<std::string> waypoint_events;
  std::vector<double> tcp_z_values;
  std::vector<double> tcp_x_values;
  std::vector<double> tcp_y_values;
  std::vector<geometry_msgs::msg::Quaternion> tcp_orientations;
  std::vector<double> approach_z_values;
  std::vector<double> retract_x_values;
  std::vector<NormalizedMotionRequest> requests;
  std::deque<uint8_t> pick_approach_results;
  std::string pick_candidate_evaluation;
  std::vector<float> gripper_widths;
  std::vector<uint8_t> finalized_planning_results;
  uint8_t motion_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
  uint8_t home_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
  bool insertion_submission_uncertain{false};
  uint8_t gripper_result{MotionTaskResultCode::TASK_RESULT_SUCCESS};
  bool gripper_active_value{false};
  int stop_gripper_calls{0};
  bool block_motion{false};
  bool release_motion{false};
  int motion_call_count{0};
  int gripper_command_count{0};
  int gripper_open_command_count{0};
  int gripper_close_command_count{0};
  bool pending_relation{false};
  bool accepted_relation{false};
  geometry_msgs::msg::PoseStamped measured_tcp_state;
  bool fail_first_pick_final_plan{false};
  bool fail_first_pick_final_without_candidate{false};
  bool reject_pick_insertion_before_submission{false};
  bool fail_pick_insertion_after_submission{false};
  bool throw_pick_insertion_after_submission{false};
  int pick_final_without_candidate_calls{0};
  bool candidate_fallback_available{false};
  bool fail_place_approach_planning{false};
  bool fail_place_insertion_planning{false};
  bool fail_pick_retract{false};
  std::string last_place_stage;
  int place_retreat_failures_remaining{0};
  int place_retreat_retries_remaining{0};
  int pick_final_calls{0};
  int candidate_advance_calls{0};
  int candidate_search_starts{0};
  int open_calls{0};
  int begin_relation_count{0};
  int discard_relation_count{0};
  int promote_relation_count{0};
  int release_relation_count{0};
  std::mutex completion_mutex;
  std::condition_variable completion_condition;
};

class RecordingGripperPort final : public GripperPort
{
public:
  explicit RecordingGripperPort(RecordingBackend & backend)
  : backend_(backend)
  {
  }

  uint8_t close() override
  {
    backend_.events.push_back("gripper");
    backend_.record_gripper_command(false);
    mark_command_sent();
    if (backend_.gripper_result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      return backend_.gripper_result;
    }
    if (auto_observe) {
      publish(HoldingState::HELD);
    }
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  uint8_t open() override
  {
    ++backend_.open_calls;
    backend_.events.push_back("gripper");
    backend_.gripper_widths.push_back(85.0F);
    backend_.record_gripper_command(true);
    mark_command_sent();
    if (backend_.gripper_result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      return backend_.gripper_result;
    }
    if (open_auto_observe) {
      publish(HoldingState::RELEASED);
    }
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  HoldingObservation holding() const override
  {
    {
      std::lock_guard<std::mutex> lock(observation_mutex_);
      observed_command_generation_ = command_generation_;
    }
    observation_condition_.notify_all();
    auto observation = observation_;
    if (refresh_observation_on_read && observation.state != HoldingState::UNKNOWN) {
      observation.observed_at = HoldingObservation::Clock::now();
    }
    return observation;
  }
  bool active() const override {return backend_.gripper_active();}
  void stop() override {backend_.stop_gripper();}

  void publish(HoldingState state)
  {
    observation_.state = state;
    observation_.observed_at = HoldingObservation::Clock::now();
    ++observation_.sequence;
  }

  void set_observation(HoldingState state, std::uint64_t sequence)
  {
    observation_.state = state;
    observation_.observed_at = HoldingObservation::Clock::now();
    observation_.sequence = sequence;
  }

  void set_width(double width_mm) {observation_.width_mm = width_mm;}

  bool wait_for_holding_after_command(
    int command_generation, std::chrono::milliseconds timeout = std::chrono::seconds(1))
  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    return observation_condition_.wait_for(
      lock, timeout, [this, command_generation]() {
        return observed_command_generation_ >= command_generation;
      });
  }

  bool auto_observe{true};
  bool open_auto_observe{true};
  bool refresh_observation_on_read{false};

private:
  void mark_command_sent()
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    ++command_generation_;
  }

  RecordingBackend & backend_;
  HoldingObservation observation_;
  mutable std::mutex observation_mutex_;
  mutable std::condition_variable observation_condition_;
  int command_generation_{0};
  mutable int observed_command_generation_{0};
};

class TestTaskExecutor : public arm_cell_task_execution::TaskExecutor
{
public:
  using arm_cell_task_execution::TaskExecutor::TaskExecutor;
  template<class Backend>
  explicit TestTaskExecutor(std::shared_ptr<Backend> backend)
  : arm_cell_task_execution::TaskExecutor(
      std::make_shared<test::LegacyBackendMotionPort>(backend, MotionGeometry{}), nullptr) {}
  template<class Backend>
  TestTaskExecutor(std::shared_ptr<Backend> backend, MotionGeometry geometry)
  : arm_cell_task_execution::TaskExecutor(
      std::make_shared<test::LegacyBackendMotionPort>(backend, geometry),
      backend->gripper_port(), geometry) {}
  template<class Backend>
  TestTaskExecutor(
    std::shared_ptr<Backend> backend, std::shared_ptr<GripperPort> gripper,
    MotionGeometry geometry, InterruptionChecker checker = {},
    std::chrono::milliseconds holding_timeout = std::chrono::milliseconds(1000))
  : arm_cell_task_execution::TaskExecutor(
      make_port(backend, geometry, checker), std::move(gripper),
      geometry, std::move(checker), holding_timeout) {}

private:
  template<class Backend>
  static std::shared_ptr<test::LegacyBackendMotionPort> make_port(
    std::shared_ptr<Backend> backend, const MotionGeometry & geometry,
    const InterruptionChecker & checker)
  {
    auto port = std::make_shared<test::LegacyBackendMotionPort>(backend, geometry);
    port->set_interruption_checker(checker);
    return port;
  }
};

#define TaskExecutor TestTaskExecutor

ExecuteTask::Goal pick_goal()
{
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_PICK;
  goal.has_target_pose = true;
  goal.orientation_symmetry_order = 4;
  goal.vision_acquisition_id = "test-acquisition";
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.pose.orientation.w = 1.0;
  goal.target_pose.pose.position.z = 1.0;
  goal.pick_approach_distance_m = 0.15;
  goal.pick_grasp_position_tolerance_m = 0.01;
  goal.pick_grasp_orientation_tolerance_rad = 0.08726646259971647;
  goal.has_estimated_object_height = true;
  goal.estimated_object_height_m = 0.1F;
  goal.has_observed_rectangle = true;
  goal.rectangle_long_side_m = 0.07F;
  goal.rectangle_short_side_m = 0.07F;
  goal.grasp_width_min_mm = 10.0F;
  goal.grasp_width_max_mm = 85.0F;
  return goal;
}

arm_cell_interfaces::msg::PickCandidate material_candidate(
  const ExecuteTask::Goal & goal, const std::string & id, double x, double z,
  double y = 0.0)
{
  arm_cell_interfaces::msg::PickCandidate candidate;
  candidate.acquisition_id = "same-acquisition";
  candidate.candidate_id = id;
  candidate.target_pose = goal.target_pose;
  candidate.target_pose.pose.position.x = x;
  candidate.target_pose.pose.position.y = y;
  candidate.target_pose.pose.position.z = z;
  candidate.orientation_symmetry_order = goal.orientation_symmetry_order;
  candidate.estimated_height_m = goal.estimated_object_height_m;
  candidate.has_estimated_height = goal.has_estimated_object_height;
  candidate.has_observed_rectangle = goal.has_observed_rectangle;
  candidate.rectangle_long_side_m = goal.rectangle_long_side_m;
  candidate.rectangle_short_side_m = goal.rectangle_short_side_m;
  return candidate;
}

MotionGeometry valid_geometry()
{
  MotionGeometry geometry;
  geometry.configured = true;
  geometry.observation_reference = "profile_reference";
  geometry.observation_to_object.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.translation.z = 0.1;
  geometry.object_to_grasp_tcp.rotation.w = 1.0;
  geometry.insertion_axis_tcp.z = 1.0;
  geometry.pick_retract_distance_m = 0.4;
  return geometry;
}

MotionGeometry waypoint_geometry()
{
  auto geometry = valid_geometry();
  geometry.place_waypoint_configured = true;
  geometry.place_waypoint_clearance_toward_positive_y = true;
  geometry.place_waypoint_clearance_y_boundary_m = -0.926;
  geometry.place_waypoint_approach_distance_m = 0.015;
  geometry.place_waypoint_approach_direction_object.z = 1.0;
  geometry.place_waypoint_target_pose.header.frame_id = "base_link";
  geometry.place_waypoint_target_pose.pose.position.x = 0.5;
  geometry.place_waypoint_target_pose.pose.position.y = 0.25;
  geometry.place_waypoint_target_pose.pose.position.z = 0.3675;
  geometry.place_waypoint_target_pose.pose.orientation.w = 1.0;
  return geometry;
}

ExecuteTask::Goal place_goal()
{
  auto goal = pick_goal();
  goal.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  goal.place_position_tolerance_m = 0.01;
  goal.place_orientation_tolerance_rad = 0.02;
  goal.place_orientation_constraint = ExecuteTask::Goal::PLACE_ORIENTATION_BOUNDED;
  goal.place_approach_direction_object.z = 1.0;
  goal.place_approach_distance_m = 0.2;
  goal.place_retract_distance_m = 0.4;
  return goal;
}

arm_cell_interfaces::action::ExecutePnp::Goal execute_pnp_goal()
{
  arm_cell_interfaces::action::ExecutePnp::Goal goal;
  goal.vision_acquisition_id = "acq-pnp-1";
  auto candidate = material_candidate(pick_goal(), "candidate-pnp-1", 0.1, 1.0);
  candidate.acquisition_id = goal.vision_acquisition_id;
  goal.material_candidates.push_back(candidate);
  goal.has_place_target_pose = true;
  goal.place_target_pose.header.frame_id = "base_link";
  goal.place_target_pose.pose.orientation.w = 1.0;
  goal.place_target_pose.pose.position.z = 0.5;
  goal.pick_approach_distance_m = 0.15;
  goal.pick_grasp_position_tolerance_m = 0.01;
  goal.pick_grasp_orientation_tolerance_rad = 0.08;
  goal.grasp_width_min_mm = 10.0F;
  goal.grasp_width_max_mm = 85.0F;
  goal.place_position_tolerance_m = 0.01;
  goal.place_orientation_tolerance_rad = 0.02;
  goal.place_orientation_constraint =
    arm_cell_interfaces::action::ExecutePnp::Goal::PLACE_ORIENTATION_BOUNDED;
  goal.place_approach_direction_object.z = 1.0;
  goal.place_approach_distance_m = 0.2;
  goal.place_retract_distance_m = 0.4;
  return goal;
}
}  // namespace

TEST(TaskExecutorPickPlaceTest, ExecutePnpCompletesPickPlaceAndHomeBeforeReturningCompleted)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  const auto goal = execute_pnp_goal();

  const auto result = executor.execute_pnp(goal, "pnp-goal-uuid");

  EXPECT_EQ(
    result.pnp_outcome,
    arm_cell_interfaces::action::ExecutePnp::Result::PNP_OUTCOME_COMPLETED);
  EXPECT_TRUE(result.material_placed);
  EXPECT_EQ(result.selected_candidate_id, "candidate-pnp-1");
  ASSERT_FALSE(backend->requests.empty());
  EXPECT_EQ(backend->requests.front().task_type.value, MotionTaskType::TASK_TYPE_PICK);
  EXPECT_EQ(backend->requests.back().task_type.value, MotionTaskType::TASK_TYPE_PLACE);
  EXPECT_EQ(backend->requests.back().object_pose.pose.position.z, 0.5);
  ASSERT_FALSE(backend->events.empty());
  EXPECT_EQ(backend->events.back(), "home");
}

TEST(TaskExecutorPickPlaceTest, FailedFinalHomeDoesNotCompletePlacedPnp)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->home_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  const auto result = executor.execute_pnp(execute_pnp_goal(), "pnp-final-home-failure");

  EXPECT_EQ(
    result.pnp_outcome,
    arm_cell_interfaces::action::ExecutePnp::Result::PNP_OUTCOME_FAILED);
  EXPECT_TRUE(result.material_placed);
  EXPECT_EQ(result.object_disposition, ExecuteTask::Result::OBJECT_DISPOSITION_PLACED);
  EXPECT_EQ(result.result_code.value, MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  ASSERT_FALSE(backend->events.empty());
  EXPECT_EQ(backend->events.back(), "home");
}

TEST(TaskExecutorPickPlaceTest, PlaceWaypointClearsTrayThenGoesHomeAndUsesFixedCachedApproach)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->measured_tcp_state.header.frame_id = "base_link";
  backend->measured_tcp_state.pose.position.x = 0.12;
  backend->measured_tcp_state.pose.position.y = -0.95;
  backend->measured_tcp_state.pose.position.z = 0.31;
  backend->measured_tcp_state.pose.orientation.w = 1.0;
  const auto geometry = waypoint_geometry();
  const PickPlaceStrategies strategies{PickPlaceStrategy::PLANNED, PickPlaceStrategy::WAYPOINT};
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, geometry),
    std::make_shared<RecordingGripperPort>(*backend), geometry,
    TaskExecutionSupport::InterruptionChecker{}, std::chrono::milliseconds(1000), strategies);

  const auto result = executor.execute_pnp(execute_pnp_goal(), "pnp-place-waypoint-clearance");

  EXPECT_EQ(
    result.pnp_outcome,
    arm_cell_interfaces::action::ExecutePnp::Result::PNP_OUTCOME_COMPLETED);
  EXPECT_TRUE(result.material_placed);
  std::vector<std::uint8_t> place_phases;
  for (const auto & request : backend->requests) {
    if (request.task_type.value == MotionTaskType::TASK_TYPE_PLACE) {
      place_phases.push_back(request.phase);
    }
  }
  EXPECT_EQ(place_phases, (std::vector<std::uint8_t>{
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING}));
  EXPECT_GE(backend->gripper_open_command_count, 1);
  const auto clearance = std::find(
    backend->waypoint_events.begin(), backend->waypoint_events.end(), "clearance");
  const auto home = std::find(
    backend->waypoint_events.begin(), backend->waypoint_events.end(), "home");
  const auto approach = std::find(
    backend->waypoint_events.begin(), backend->waypoint_events.end(), "cached_approach");
  ASSERT_NE(clearance, backend->waypoint_events.end());
  ASSERT_NE(home, backend->waypoint_events.end());
  ASSERT_NE(approach, backend->waypoint_events.end());
  EXPECT_LT(clearance, home);
  EXPECT_LT(home, approach);
  EXPECT_EQ(backend->waypoint_events.back(), "home");

  const auto clearance_request = std::find_if(
    backend->requests.begin(), backend->requests.end(),
    [](const auto & request) {return request.target_is_tcp_pose;});
  ASSERT_NE(clearance_request, backend->requests.end());
  EXPECT_DOUBLE_EQ(clearance_request->tcp_target.pose.position.x, 0.12);
  EXPECT_DOUBLE_EQ(clearance_request->tcp_target.pose.position.y, -0.876);
  EXPECT_DOUBLE_EQ(clearance_request->tcp_target.pose.position.z, 0.31);
  EXPECT_DOUBLE_EQ(clearance_request->tcp_target.pose.orientation.w, 1.0);
  EXPECT_EQ(clearance_request->segment, MotionSegmentKind::PLACE_CLEARANCE);
  EXPECT_DOUBLE_EQ(clearance_request->place_waypoint_clearance_y_margin_m, 0.05);

  const auto approach_request = std::find_if(
    backend->requests.begin(), backend->requests.end(),
    [](const auto & request) {return request.place_waypoint_approach;});
  ASSERT_NE(approach_request, backend->requests.end());
  EXPECT_DOUBLE_EQ(approach_request->object_pose.pose.position.x, 0.5);
  EXPECT_DOUBLE_EQ(approach_request->object_pose.pose.position.y, 0.25);
  EXPECT_DOUBLE_EQ(approach_request->object_pose.pose.position.z, 0.3675);
  EXPECT_DOUBLE_EQ(approach_request->approach_distance_m, 0.015);
}

TEST(
  TaskExecutorPickPlaceTest,
  PlaceWaypointSkipsClearanceAtBoundaryAndExecutesFixedApproachAcrossCycles)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->measured_tcp_state.header.frame_id = "base_link";
  backend->measured_tcp_state.pose.position.y = -0.926;
  backend->measured_tcp_state.pose.orientation.w = 1.0;
  const auto geometry = waypoint_geometry();
  const PickPlaceStrategies strategies{PickPlaceStrategy::PLANNED, PickPlaceStrategy::WAYPOINT};
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, geometry),
    std::make_shared<RecordingGripperPort>(*backend), geometry,
    TaskExecutionSupport::InterruptionChecker{}, std::chrono::milliseconds(1000), strategies);

  for (int cycle = 0; cycle < 2; ++cycle) {
    const auto result = executor.execute_pnp(
      execute_pnp_goal(), "pnp-place-waypoint-reuse-" + std::to_string(cycle));
    EXPECT_EQ(
      result.pnp_outcome,
      arm_cell_interfaces::action::ExecutePnp::Result::PNP_OUTCOME_COMPLETED);
  }

  EXPECT_EQ(
    std::count(
      backend->waypoint_events.begin(), backend->waypoint_events.end(), "clearance"), 0);
  EXPECT_EQ(
    std::count(
      backend->waypoint_events.begin(), backend->waypoint_events.end(), "cached_approach"), 2);
}

TEST(TaskExecutorPickPlaceTest, PlaceWaypointRequiresMeasuredYBoundaryAfterClearance)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->measured_tcp_state.header.frame_id = "base_link";
  backend->measured_tcp_state.pose.position.y = -0.95;
  backend->measured_tcp_state.pose.orientation.w = 1.0;
  backend->reflect_clearance_pose_in_measurement = false;
  const auto geometry = waypoint_geometry();
  const PickPlaceStrategies strategies{PickPlaceStrategy::PLANNED, PickPlaceStrategy::WAYPOINT};
  TaskExecutor executor(
    std::make_shared<test::LegacyBackendMotionPort>(backend, geometry),
    std::make_shared<RecordingGripperPort>(*backend), geometry,
    TaskExecutionSupport::InterruptionChecker{}, std::chrono::milliseconds(1000), strategies);

  const auto result = executor.execute_pnp(execute_pnp_goal(), "pnp-place-clearance-unverified");

  EXPECT_EQ(
    result.pnp_outcome,
    arm_cell_interfaces::action::ExecutePnp::Result::PNP_OUTCOME_RETRYABLE);
  EXPECT_FALSE(result.material_placed);
  EXPECT_EQ(
    std::count(
      backend->waypoint_events.begin(), backend->waypoint_events.end(), "cached_approach"), 0);
}

TEST(TaskExecutorPickPlaceTest, PnpRetreatFailureKeepsPlacedFactAndCannotRetry)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->place_retreat_failures_remaining = 3;
  backend->place_retreat_retries_remaining = 2;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  const auto result = executor.execute_pnp(execute_pnp_goal(), "pnp-retreat-failure");

  EXPECT_EQ(
    result.pnp_outcome,
    arm_cell_interfaces::action::ExecutePnp::Result::PNP_OUTCOME_FAILED);
  EXPECT_TRUE(result.material_placed);
  EXPECT_EQ(
    result.object_disposition,
    ExecuteTask::Result::OBJECT_DISPOSITION_PLACED);
  EXPECT_EQ(result.result_code.value, MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(backend->open_calls, 2);
}

TEST(TaskExecutorPickPlaceTest, ResolvedReturnedPlaceFailureMapsToRetryable)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->fail_place_approach_planning = true;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  const auto result = executor.execute_pnp(execute_pnp_goal(), "pnp-returned-retry");

  EXPECT_EQ(
    result.pnp_outcome,
    arm_cell_interfaces::action::ExecutePnp::Result::PNP_OUTCOME_RETRYABLE);
  EXPECT_FALSE(result.material_placed);
  EXPECT_EQ(
    result.object_disposition,
    ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
}

TEST(TaskExecutorPickPlaceTest, CancelAndSafetyAfterReleaseKeepMaterialPlaced)
{
  for (const auto interruption_code : {
      MotionTaskResultCode::TASK_RESULT_CANCELED,
      MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED})
  {
    auto backend = std::make_shared<RecordingBackend>();
    auto interruption = std::make_shared<std::atomic<uint8_t>>(
      MotionTaskResultCode::TASK_RESULT_SUCCESS);
    TaskExecutor executor(
      backend, std::make_shared<RecordingGripperPort>(*backend), valid_geometry(),
      [interruption]() {return interruption->load();});

    const auto result = executor.execute_pnp(
      execute_pnp_goal(), "pnp-post-release-interruption",
      [interruption, interruption_code](
        std::uint8_t phase, std::uint8_t, std::uint8_t, std::uint8_t) {
        if (phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_DETACHING) {
          interruption->store(interruption_code);
        }
      });

    EXPECT_TRUE(result.material_placed);
    EXPECT_EQ(
      result.object_disposition,
      ExecuteTask::Result::OBJECT_DISPOSITION_PLACED);
    EXPECT_EQ(
      result.pnp_outcome,
      interruption_code == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED ?
      arm_cell_interfaces::action::ExecutePnp::Result::PNP_OUTCOME_SAFETY_PREEMPTED :
      arm_cell_interfaces::action::ExecutePnp::Result::PNP_OUTCOME_FAILED);
  }
}

TEST(TaskExecutorPickPlaceTest, OrdersRobotSideTrayYThenTopZAndKeepsFallback)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->pick_approach_results = {
    MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE,
    MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE,
    MotionTaskResultCode::TASK_RESULT_SUCCESS};
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  auto goal = pick_goal();
  auto outer_high_z = material_candidate(goal, "outer_high_z", -3.0, 0.99, -0.40);
  auto inner_low_z = material_candidate(goal, "inner_low_z", 0.0, 0.60, 0.30);
  auto inner_high_z = material_candidate(goal, "inner_high_z", 3.0, 0.65, 0.30);
  outer_high_z.estimated_height_m = 0.05F;
  inner_low_z.estimated_height_m = 0.05F;
  inner_high_z.estimated_height_m = 0.20F;
  for (auto * candidate : {&outer_high_z, &inner_low_z, &inner_high_z}) {
    candidate->top_height_tolerance_m = 0.003;
  }
  goal.material_candidates = {outer_high_z, inner_low_z, inner_high_z};
  goal.has_target_pose = false;
  goal.vision_acquisition_id.clear();

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS)
    << executor.last_failure_stage();
  EXPECT_EQ(
    executor.attempted_material_candidate_ids(),
    (std::vector<std::string>{"inner_high_z", "inner_low_z", "outer_high_z"}));
  EXPECT_EQ(executor.outcome_material_candidate_id(), "outer_high_z");
  ASSERT_EQ(backend->material_candidate_attempts.size(), 3U);
  EXPECT_EQ(backend->material_candidate_attempts[0].candidate_id, "inner_high_z");
  EXPECT_EQ(
    backend->material_candidate_attempts[0].result_code,
    MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE);
  EXPECT_EQ(backend->material_candidate_attempts[2].candidate_id, "outer_high_z");
  EXPECT_EQ(
    backend->material_candidate_attempts[2].result_code,
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(backend->material_candidate_outcome, "outer_high_z");
  EXPECT_EQ(backend->candidate_search_starts, 3);
  EXPECT_EQ(backend->open_calls, 1);
}

TEST(TaskExecutorPickPlaceTest, EqualTrayYUsesObservedTopHeightThenCandidateId)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->pick_approach_results = {
    MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE,
    MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE,
    MotionTaskResultCode::TASK_RESULT_SUCCESS};
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  auto goal = pick_goal();
  goal.material_candidates = {
    material_candidate(goal, "zeta", 0.2, 0.6, 0.3),
    material_candidate(goal, "lower-z", -2.0, 0.5, 0.3),
    material_candidate(goal, "alpha", -0.2, 0.6, 0.3)};
  goal.has_target_pose = false;
  goal.vision_acquisition_id.clear();

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    executor.attempted_material_candidate_ids(),
    (std::vector<std::string>{"alpha", "zeta", "lower-z"}));
}

TEST(TaskExecutorPickPlaceTest, SingleLayerTrayOrderDoesNotDependOnVisionEnumeration)
{
  const auto run = [](bool reverse_enumeration) {
      auto backend = std::make_shared<RecordingBackend>();
      backend->pick_approach_results = {
        MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE,
        MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE,
        MotionTaskResultCode::TASK_RESULT_SUCCESS};
      TaskExecutor executor(
        backend, std::make_shared<RecordingGripperPort>(*backend), valid_geometry());
      auto goal = pick_goal();
      // material_candidate arguments are x, z, y; greater Tray Y is the robot side.
      auto robot_side = material_candidate(goal, "robot-side", 0.1, 0.55, 1.20);
      auto middle = material_candidate(goal, "middle", 0.1, 0.55, 0.95);
      auto outer = material_candidate(goal, "outer", 0.1, 0.55, 0.55);
      for (auto * candidate : {&robot_side, &middle, &outer}) {
        candidate->top_height_tolerance_m = 0.003;
      }
      goal.material_candidates = reverse_enumeration ?
        std::vector{outer, middle, robot_side} : std::vector{robot_side, middle, outer};
      goal.has_target_pose = false;
      goal.vision_acquisition_id.clear();

      EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
      EXPECT_EQ(
        executor.attempted_material_candidate_ids(),
        (std::vector<std::string>{"robot-side", "middle", "outer"}));
      return executor.attempted_material_candidate_ids();
    };

  const std::vector<std::string> expected{"robot-side", "middle", "outer"};
  EXPECT_EQ(run(false), expected);
  EXPECT_EQ(run(true), expected);
}

TEST(TaskExecutorPickPlaceTest, InvalidMaterialCandidateIsRecordedAndSkipped)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  auto goal = pick_goal();
  auto invalid = material_candidate(goal, "invalid", 0.1, 0.7);
  invalid.target_pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  goal.material_candidates = {
    invalid, material_candidate(goal, "lower", 0.2, 0.5),
    material_candidate(goal, "higher", 0.3, 0.7)};
  goal.has_target_pose = false;
  goal.vision_acquisition_id.clear();

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS)
    << executor.last_failure_stage();
  EXPECT_EQ(executor.attempted_material_candidate_ids(), (std::vector<std::string>{"higher"}));
  ASSERT_EQ(backend->material_candidate_attempts.size(), 2U);
  EXPECT_EQ(backend->material_candidate_attempts.front().candidate_id, "invalid");
  EXPECT_EQ(
    backend->material_candidate_attempts.front().result_code,
    MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_EQ(
    backend->material_candidate_attempts.front().failure_stage,
    "validation:invalid_target_pose");
}

TEST(TaskExecutorPickPlaceTest, BackendFailureDoesNotAdvanceMaterialCandidate)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->pick_approach_results = {
    MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR,
    MotionTaskResultCode::TASK_RESULT_SUCCESS};
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  auto goal = pick_goal();
  goal.material_candidates = {
    material_candidate(goal, "first", 0.1, 0.6),
    material_candidate(goal, "second", 0.2, 0.5)};
  goal.has_target_pose = false;
  goal.vision_acquisition_id.clear();

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_EQ(executor.attempted_material_candidate_ids(), (std::vector<std::string>{"first"}));
}

TEST(TaskExecutorPickPlaceTest, SearchFailuresAdvanceAndKeepLastResultWhileSystemFailuresStop)
{
  for (const auto code : {MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED,
      MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE,
      MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED,
      MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR,
      MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE,
      MotionTaskResultCode::TASK_RESULT_CANCELED,
      MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED})
  {
    SCOPED_TRACE(static_cast<int>(code));
    auto backend = std::make_shared<RecordingBackend>();
    backend->pick_approach_results = {code, code, code};
    TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
      valid_geometry());
    auto goal = pick_goal();
    goal.material_candidates = {
      material_candidate(goal, "first", 0.1, 0.6),
      material_candidate(goal, "second", 0.2, 0.5),
      material_candidate(goal, "third", 0.3, 0.4)};
    goal.has_target_pose = false;
    goal.vision_acquisition_id.clear();
    EXPECT_EQ(executor.execute(goal), code);
    const bool may_advance = code == MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED ||
      code == MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE ||
      code == MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
    const std::vector<std::string> expected = may_advance ?
      std::vector<std::string>{"first", "second", "third"} : std::vector<std::string>{"first"};
    EXPECT_EQ(executor.attempted_material_candidate_ids(), expected);
    ASSERT_EQ(backend->material_candidate_attempts.size(), expected.size());
    for (const auto & attempt : backend->material_candidate_attempts) {
      EXPECT_EQ(attempt.result_code, code);
    }
    EXPECT_EQ(backend->material_candidate_outcome, expected.back());
  }
}

TEST(TaskExecutorPickPlaceTest, InsertionFailureDoesNotAdvanceMaterialCandidate)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->fail_pick_insertion_after_submission = true;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  auto goal = pick_goal();
  goal.material_candidates = {
    material_candidate(goal, "first", 0.1, 0.6),
    material_candidate(goal, "second", 0.2, 0.5)};
  goal.has_target_pose = false;
  goal.vision_acquisition_id.clear();

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_TRUE(executor.pick_grasp_insertion_started());
  EXPECT_EQ(executor.attempted_material_candidate_ids(), (std::vector<std::string>{"first"}));
}

TEST(TaskExecutorPickPlaceTest, SafetyPreemptionDoesNotAdvanceMaterialCandidate)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->pick_approach_results = {
    MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED,
    MotionTaskResultCode::TASK_RESULT_SUCCESS};
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  auto goal = pick_goal();
  goal.material_candidates = {
    material_candidate(goal, "first", 0.1, 0.6),
    material_candidate(goal, "second", 0.2, 0.5)};
  goal.has_target_pose = false;
  goal.vision_acquisition_id.clear();

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(executor.attempted_material_candidate_ids(), (std::vector<std::string>{"first"}));
}

TEST(TaskExecutorPickPlaceTest, CancellationDoesNotAdvanceMaterialCandidate)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->pick_approach_results = {
    MotionTaskResultCode::TASK_RESULT_CANCELED,
    MotionTaskResultCode::TASK_RESULT_SUCCESS};
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  auto goal = pick_goal();
  goal.material_candidates = {
    material_candidate(goal, "first", 0.1, 0.6),
    material_candidate(goal, "second", 0.2, 0.5)};
  goal.has_target_pose = false;
  goal.vision_acquisition_id.clear();

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_CANCELED);
  EXPECT_EQ(executor.attempted_material_candidate_ids(), (std::vector<std::string>{"first"}));
}

TEST(TaskExecutorPickPlaceTest, PickOrdersApproachGraspAttachRetractAndConvertsTargetInMotion)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    geometry);
  auto goal = pick_goal();
  goal.has_place_tool_orientation_preference = true;
  goal.place_tool_orientation_preference.x = 1.0;
  goal.place_tool_orientation_preference.w = 0.0;

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{
      "gripper", "motion:3", "motion:4", "gripper", "motion:8"}));
  EXPECT_EQ(backend->gripper_widths, (std::vector<float>{85.0F}));
  ASSERT_EQ(backend->tcp_z_values.size(), 3U);
  ASSERT_EQ(backend->approach_z_values.size(), 3U);
  ASSERT_EQ(backend->requests.size(), 3U);
  for (const auto & request : backend->requests) {
    EXPECT_FALSE(request.has_place_tool_orientation_preference);
  }
  EXPECT_NEAR(backend->tcp_z_values.front(), 0.9, 1e-6);
  EXPECT_NEAR(backend->tcp_z_values[1], 1.05, 1e-6);
  EXPECT_NEAR(backend->tcp_z_values.back(), 0.65, 1e-6);
  EXPECT_NEAR(backend->approach_z_values.front(), 0.9, 1e-6);
  EXPECT_DOUBLE_EQ(backend->requests.front().approach_distance_m, 0.15);
  ASSERT_EQ(backend->retract_x_values.size(), 3U);
  EXPECT_DOUBLE_EQ(backend->retract_x_values.back(), 0.0);
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
  EXPECT_TRUE(backend->accepted_relation);
  EXPECT_EQ(backend->promote_relation_count, 1);
}

TEST(TaskExecutorPickPlaceTest, PlanningFailureAdvancesCandidateInsideOnePickAction)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->fail_first_pick_final_plan = true;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  const auto result = executor.execute(pick_goal());

  EXPECT_EQ(result, MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(backend->candidate_advance_calls, 1);
  ASSERT_EQ(backend->requests.size(), 5U);
  EXPECT_EQ(
    backend->requests[0].phase,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING);
  EXPECT_EQ(
    backend->requests[1].phase,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING);
  EXPECT_EQ(
    backend->requests[2].phase,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING);
  EXPECT_EQ(
    backend->requests[3].phase,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING);
  EXPECT_EQ(
    backend->requests[4].phase,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING);
  EXPECT_EQ(std::count(backend->events.begin(), backend->events.end(), "gripper"), 2);
}

TEST(TaskExecutorPickPlaceTest, CandidateTransitionPassesFullAcquisitionAndSelectedId)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->pick_approach_results = {
    MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED,
    MotionTaskResultCode::TASK_RESULT_SUCCESS};
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  auto goal = pick_goal();
  auto first = material_candidate(goal, "candidate-first", 0.1, 0.18);
  auto second = material_candidate(goal, "candidate-second", 0.2, 0.16);
  goal.material_candidates = {first, second};

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_GE(backend->requests.size(), 3U);
  EXPECT_EQ(backend->requests[0].vision_candidate_id, "candidate-first");
  EXPECT_EQ(
    backend->requests[0].phase,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING);
  ASSERT_EQ(backend->requests[0].material_candidates.size(), 2U);
  EXPECT_EQ(backend->requests[0].material_candidates[0].candidate_id, first.candidate_id);
  EXPECT_EQ(backend->requests[0].material_candidates[1].candidate_id, second.candidate_id);
  EXPECT_EQ(backend->requests[1].vision_candidate_id, "candidate-second");
  EXPECT_EQ(
    backend->requests[1].phase,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING);
  ASSERT_EQ(backend->requests[1].material_candidates.size(), 2U);
  EXPECT_EQ(backend->requests[1].material_candidates[0].candidate_id, first.candidate_id);
  EXPECT_EQ(backend->requests[1].material_candidates[1].candidate_id, second.candidate_id);
}

TEST(TaskExecutorPickPlaceTest, ExecutionFailureDoesNotAdvancePlanningCandidate)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->fail_first_pick_final_without_candidate = true;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  const auto result = executor.execute(pick_goal());

  EXPECT_EQ(result, MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_EQ(backend->candidate_advance_calls, 0);
  ASSERT_EQ(backend->requests.size(), 2U);
  EXPECT_EQ(
    backend->requests[0].phase,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING);
  EXPECT_EQ(
    backend->requests[1].phase,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING);
  EXPECT_EQ(std::count(backend->events.begin(), backend->events.end(), "gripper"), 1);
  EXPECT_FALSE(executor.pick_grasp_insertion_started());
}

TEST(TaskExecutorPickPlaceTest, DiagnosticFailurePreservesReturnedAndThrownTaskOutcome)
{
  auto returned_backend = std::make_shared<RecordingBackend>();
  returned_backend->motion_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  returned_backend->throw_on_diagnostic_finalize = true;
  TaskExecutor returned_executor(returned_backend,
    std::make_shared<RecordingGripperPort>(*returned_backend), valid_geometry());
  EXPECT_EQ(
    returned_executor.execute(pick_goal()),
    MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);

  auto disabled_backend = std::make_shared<RecordingBackend>();
  disabled_backend->motion_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  disabled_backend->diagnostics_enabled = false;
  TaskExecutor disabled_executor(disabled_backend,
    std::make_shared<RecordingGripperPort>(*disabled_backend), valid_geometry());
  EXPECT_EQ(
    disabled_executor.execute(pick_goal()),
    MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_TRUE(disabled_backend->finalized_planning_results.empty());

  auto throwing_backend = std::make_shared<RecordingBackend>();
  throwing_backend->throw_pick_insertion_after_submission = true;
  throwing_backend->throw_on_diagnostic_finalize = true;
  TaskExecutor throwing_executor(throwing_backend,
    std::make_shared<RecordingGripperPort>(*throwing_backend), valid_geometry());
  try {
    (void)throwing_executor.execute(pick_goal());
    FAIL() << "expected task execution exception";
  } catch (const std::runtime_error & error) {
    EXPECT_STREQ(error.what(), "uncertain insertion submission result");
  }
  EXPECT_EQ(throwing_backend->diagnostic_failure_stage, "execution_worker:exception");
  EXPECT_EQ(
    throwing_backend->diagnostic_failure_detail,
    "uncertain insertion submission result");
}

TEST(TaskExecutorPickPlaceTest, PlaceBackendErrorRetainsTaskExecutorFailureStage)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->motion_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(
    executor.execute(place_goal(), MotionObjectState::ATTACHED),
    MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_EQ(backend->diagnostic_failure_stage, "place_approach_backend_request");
}

TEST(TaskExecutorPickPlaceTest, ApproachFailureDoesNotMarkGraspInsertionStarted)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->motion_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_FALSE(executor.pick_grasp_insertion_started());
}

TEST(TaskExecutorPickPlaceTest, ConfirmedInsertionRejectionKeepsStartedFalse)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->reject_pick_insertion_before_submission = true;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_FALSE(executor.pick_grasp_insertion_started());
}

TEST(TaskExecutorPickPlaceTest, InsertionFailureAfterSubmissionKeepsStartedEvidence)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->fail_pick_insertion_after_submission = true;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_TRUE(executor.pick_grasp_insertion_started());
}

TEST(TaskExecutorPickPlaceTest, NewTaskExecutionResetsPreviousInsertionState)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_TRUE(executor.pick_grasp_insertion_started());

  ExecuteTask::Goal next_goal;
  next_goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  EXPECT_EQ(executor.execute(next_goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_FALSE(executor.pick_grasp_insertion_started());
}

TEST(TaskExecutorPickPlaceTest, UncertainInsertionSubmissionIsReportedConservatively)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->throw_pick_insertion_after_submission = true;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_THROW(executor.execute(pick_goal()), std::runtime_error);
  EXPECT_TRUE(executor.pick_grasp_insertion_started());
}

TEST(
  TaskExecutorPickPlaceTest,
  PickRecordIsWrittenAsynchronouslyAndStorageFailureDoesNotChangeResult)
{
  ScopedEnvironmentVariable record_directory("ARM_CELL_PICK_EXECUTION_RECORD_DIR");
  const auto directory = std::filesystem::temp_directory_path() /
    ("arm-cell-pick-record-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  ASSERT_EQ(::setenv("ARM_CELL_PICK_EXECUTION_RECORD_DIR", directory.c_str(), 1), 0);
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  const auto goal = pick_goal();

  EXPECT_EQ(
    executor.execute(goal, MotionObjectState::NO_OBJECT, "motion-goal-diagnostic-test"),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);

  const auto record_path = directory / "motion-goal-diagnostic-test.json";
  std::string record;
  for (int attempt = 0; attempt < 100; ++attempt) {
    std::ifstream input(record_path);
    record.assign((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    if (record.find("candidate_evaluation_status") != std::string::npos) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_NE(record.find("candidate_evaluation_status"), std::string::npos);
  EXPECT_NE(record.find("motion-goal-diagnostic-test"), std::string::npos);
  EXPECT_NE(record.find("\"schema_version\": 1"), std::string::npos);
  EXPECT_NE(
    record.find("\"motion_goal_uuid\": \"motion-goal-diagnostic-test\""),
    std::string::npos);
  EXPECT_NE(
    record.find(
      "\"capture_diagnostic_record_relative_path\": "
      "\"capture_diagnostic_records/motion-goal-diagnostic-test.json\""),
    std::string::npos);
  EXPECT_EQ(record.find("\"execution_id\""), std::string::npos);
  EXPECT_NE(record.find("test-acquisition"), std::string::npos);
  EXPECT_NE(record.find("candidate_evaluation"), std::string::npos);
  EXPECT_NE(
    record.find("\"candidate_evaluation_status\": \"unavailable\""),
    std::string::npos);

  ASSERT_EQ(
    ::setenv(
    "ARM_CELL_PICK_EXECUTION_RECORD_DIR", "/proc/arm-cell-record-write-denied", 1), 0);
  auto second_backend = std::make_shared<RecordingBackend>();
  TaskExecutor second_executor(
    second_backend, std::make_shared<RecordingGripperPort>(*second_backend), valid_geometry());
  EXPECT_EQ(
    second_executor.execute(goal, MotionObjectState::NO_OBJECT, "storage-failure-goal"),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  std::filesystem::remove_all(directory);
}

TEST(TaskExecutorPickPlaceTest, PickExecutionRecordKeepsCompactCandidateSummaryAndPlanningReference)
{
  ScopedEnvironmentVariable record_directory("ARM_CELL_PICK_EXECUTION_RECORD_DIR");
  const auto directory = std::filesystem::temp_directory_path() /
    ("arm-cell-pick-compact-record-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  ASSERT_EQ(::setenv("ARM_CELL_PICK_EXECUTION_RECORD_DIR", directory.c_str(), 1), 0);
  auto backend = std::make_shared<RecordingBackend>();
  auto evidence = nlohmann::json::object();
  evidence["generated_candidate_count"] = 48;
  evidence["evaluated_candidate_count"] = 6;
  evidence["candidate_result"] = "PLANNED_SELECTED";
  evidence["selected_candidate"] = 5;
  evidence["selected_tcp_orientation_xyzw"] = {0.0, 0.0, 0.0, 1.0};
  evidence["grasp_pose_candidates"] = nlohmann::json::array();
  for (std::size_t index = 0; index < 48U; ++index) {
    evidence["grasp_pose_candidates"].push_back(
      {
      {"candidate", index + 1U}, {"diagnostic_padding", std::string(2048U, 'x')}});
  }
  backend->pick_candidate_evaluation = evidence.dump();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(
    executor.execute(pick_goal(), MotionObjectState::NO_OBJECT, "motion-goal-compact-test"),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);

  const auto record_path = directory / "motion-goal-compact-test.json";
  std::optional<nlohmann::json> record;
  std::string serialized;
  for (int attempt = 0; attempt < 100 && !record; ++attempt) {
    if (std::filesystem::exists(record_path)) {
      std::ifstream input(record_path);
      serialized.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
      try {
        record = nlohmann::json::parse(serialized);
      } catch (const nlohmann::json::parse_error &) {
        serialized.clear();
      }
    }
    if (!record) {std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  }
  ASSERT_TRUE(record.has_value());
  EXPECT_LT(serialized.size(), 10U * 1024U);
  EXPECT_EQ(record->at("candidate_evaluation").at("candidate_result"), "PLANNED_SELECTED");
  EXPECT_EQ(record->at("candidate_evaluation").at("generated_candidate_count"), 48);
  EXPECT_EQ(
    record->at("candidate_evaluation").at("motion_planning_summary_reference").at("path"),
    "motion_planning_summaries/motion-goal-compact-test.json");
  EXPECT_FALSE(record->at("candidate_evaluation").contains("grasp_pose_candidates"));
  std::filesystem::remove_all(directory);
}

TEST(TaskExecutorPickPlaceTest, PickGoalCanSelectPlanningFrameVerticalApproach)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);
  auto goal = pick_goal();
  goal.pick_approach_translation_direction =
    ExecuteTask::Goal::PICK_APPROACH_PLANNING_FRAME_VERTICAL;

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->requests.size(), 3U);
  EXPECT_EQ(
    backend->requests.front().pick_approach_axis,
    PickApproachAxis::PLANNING_FRAME_VERTICAL);
  EXPECT_NEAR(backend->requests.front().tcp_target.pose.position.z, 1.20, 1e-6);
  EXPECT_DOUBLE_EQ(
    backend->requests.front().tcp_target.pose.orientation.x,
    backend->requests.front().approach_target.pose.orientation.x);
  EXPECT_DOUBLE_EQ(
    backend->requests.front().tcp_target.pose.orientation.w,
    backend->requests.front().approach_target.pose.orientation.w);
}

TEST(TaskExecutorPickPlaceTest, OrderZeroUsesOnlyTheExplicitFallbackForObservationConversion)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.observation_to_object.translation.x = 0.1;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);
  auto goal = pick_goal();
  goal.orientation_symmetry_order = ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE;
  goal.has_unobserved_yaw_fallback = true;
  goal.unobserved_yaw_fallback_rad = 1.57079632679489661923;
  goal.target_pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  goal.target_pose.pose.orientation.w = 0.0;

  EXPECT_EQ(
    executor.execute(goal, MotionObjectState::NO_OBJECT, "order-zero-test"),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_FALSE(backend->requests.empty());
  EXPECT_EQ(
    backend->requests.front().orientation_symmetry_order,
    ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE);
  EXPECT_TRUE(backend->requests.front().has_unobserved_yaw_fallback);
  EXPECT_NEAR(backend->requests.front().object_pose.pose.position.x, 0.0, 1e-12);
  EXPECT_NEAR(backend->requests.front().object_pose.pose.position.y, 0.0, 1e-12);
  EXPECT_NEAR(
    backend->requests.front().object_pose.pose.orientation.z,
    std::sin(0.25 * 3.14159265358979323846), 1e-12);
}

TEST(TaskExecutorPickPlaceTest, FailedPickDiscardsSelectedRelationBeforeHeld)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->motion_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_FALSE(backend->pending_relation);
  EXPECT_FALSE(backend->accepted_relation);
  EXPECT_EQ(backend->promote_relation_count, 0);
  EXPECT_EQ(backend->discard_relation_count, 1);
}

TEST(TaskExecutorPickPlaceTest, FreshReleaseInvalidatesAcceptedRelation)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  ASSERT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_TRUE(backend->accepted_relation);
  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_FALSE(backend->pending_relation);
  EXPECT_FALSE(backend->accepted_relation);
  EXPECT_EQ(backend->release_relation_count, 1);
}

TEST(TaskExecutorPickPlaceTest, ResolvesProfileObservationToObjectCenterBeforeTargetTransform)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.observation_to_object.translation.z = -0.04;
  geometry.object_to_grasp_tcp.translation.z = 0.0;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);

  auto goal = pick_goal();
  goal.pick_approach_distance_m = 0.2;
  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_FALSE(backend->requests.empty());
  const auto & approach_request = backend->requests.front();
  EXPECT_DOUBLE_EQ(approach_request.observation_pose.pose.position.z, 1.0);
  EXPECT_NEAR(approach_request.object_pose.pose.position.z, 0.95, 1e-6);
  EXPECT_NEAR(approach_request.target_pose.pose.position.z, 0.95, 1e-6);
  EXPECT_NEAR(approach_request.approach_target.pose.position.z, 0.75, 1e-6);
  ASSERT_EQ(backend->tcp_z_values.size(), 3U);
  EXPECT_NEAR(backend->tcp_z_values[0], 0.75, 1e-6);
  EXPECT_NEAR(backend->tcp_z_values[1], 0.95, 1e-6);
  EXPECT_NEAR(backend->tcp_z_values[2], 0.55, 1e-6);
}

TEST(TaskExecutorPickPlaceTest, LegacyProfilePickWithoutRectangleRetainsConfiguredTransform)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.observation_to_object.translation.z = -0.04;
  geometry.object_to_grasp_tcp.translation.z = 0.0;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);
  auto goal = pick_goal();
  goal.has_observed_rectangle = false;
  goal.has_estimated_object_height = false;
  goal.rectangle_long_side_m = 0.0F;
  goal.rectangle_short_side_m = 0.0F;
  goal.orientation_symmetry_order = ExecuteTask::Goal::ORIENTATION_UNIQUE;

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_FALSE(backend->requests.empty());
  EXPECT_DOUBLE_EQ(backend->requests.front().object_pose.pose.position.z, 0.96);
  EXPECT_FALSE(backend->requests.front().has_observed_rectangle);
}

TEST(TaskExecutorPickPlaceTest, EstimatedRectangleGeometryIsConsumedWithoutObservedOutline)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(
    backend, std::make_shared<RecordingGripperPort>(*backend), valid_geometry());
  auto goal = pick_goal();
  goal.has_observed_rectangle = false;
  goal.orientation_symmetry_order = ExecuteTask::Goal::ORIENTATION_TWOFOLD;
  goal.rectangle_long_side_m = 0.08F;
  goal.rectangle_short_side_m = 0.05F;

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS)
    << executor.last_failure_stage();
  ASSERT_FALSE(backend->requests.empty());
  EXPECT_DOUBLE_EQ(backend->requests.front().observation_pose.pose.position.z, 1.0);
  EXPECT_NEAR(backend->requests.front().object_pose.pose.position.z, 0.95, 1e-6);
  EXPECT_FALSE(backend->requests.front().has_observed_rectangle);
  EXPECT_FLOAT_EQ(backend->requests.front().rectangle_long_side_m, 0.08F);
  EXPECT_FLOAT_EQ(backend->requests.front().rectangle_short_side_m, 0.05F);
}

TEST(TaskExecutorPickPlaceTest, PlaceTreatsRecipePoseAsDesiredObjectPose)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.observation_to_object.translation.z = -0.04;
  geometry.object_to_grasp_tcp.translation.z = 0.1;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);
  auto goal = place_goal();
  goal.target_pose.pose.position.z = 0.23;
  goal.target_pose.pose.orientation.z = 0.7071067811865476;
  goal.target_pose.pose.orientation.w = 0.7071067811865476;
  goal.has_place_tool_orientation_preference = true;
  goal.place_tool_orientation_preference.x = 1.0;
  goal.place_tool_orientation_preference.w = 0.0;
  goal.place_approach_direction_object.z = 1.0;
  goal.place_approach_distance_m = 0.2;
  goal.place_retract_distance_m = 0.4;
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_FALSE(backend->requests.empty());
  const auto & approach_request = backend->requests.front();
  EXPECT_DOUBLE_EQ(approach_request.observation_pose.pose.position.z, 0.0);
  EXPECT_DOUBLE_EQ(approach_request.object_pose.pose.position.z, 0.23);
  EXPECT_DOUBLE_EQ(approach_request.object_pose.pose.orientation.z, 0.7071067811865476);
  EXPECT_DOUBLE_EQ(approach_request.target_pose.pose.position.z, 0.33);
  EXPECT_DOUBLE_EQ(approach_request.target_pose.pose.orientation.z, 0.7071067811865476);
  EXPECT_DOUBLE_EQ(approach_request.target_pose.pose.orientation.w, 0.7071067811865476);
  EXPECT_TRUE(approach_request.has_place_tool_orientation_preference);
  EXPECT_DOUBLE_EQ(approach_request.place_tool_orientation_preference.x, 1.0);
  EXPECT_DOUBLE_EQ(approach_request.approach_target.pose.position.z, 0.53);
  EXPECT_DOUBLE_EQ(approach_request.place_position_tolerance_m, 0.01);
  EXPECT_DOUBLE_EQ(approach_request.place_orientation_tolerance_rad, 0.02);
  EXPECT_EQ(
    approach_request.place_orientation_constraint, PlaceOrientationConstraint::BOUNDED);
}

TEST(TaskExecutorPickPlaceTest, FinalizesPickPlaceHomeDiagnosticWithTerminalResult)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(
    backend, std::make_shared<RecordingGripperPort>(*backend), valid_geometry());
  ExecuteTask::Goal home;
  home.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  EXPECT_EQ(executor.execute(home), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->finalized_planning_results.size(), 1U);
  EXPECT_EQ(backend->finalized_planning_results.back(), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  backend->motion_result = MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
  EXPECT_EQ(
    executor.execute(place_goal(), MotionObjectState::ATTACHED),
    MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  ASSERT_EQ(backend->finalized_planning_results.size(), 2U);
  EXPECT_EQ(
    backend->finalized_planning_results.back(), MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);

  backend->motion_result = MotionTaskResultCode::TASK_RESULT_SUCCESS;
  EXPECT_EQ(
    executor.execute(pick_goal(), MotionObjectState::NO_OBJECT),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->finalized_planning_results.size(), 3U);
  EXPECT_EQ(
    backend->finalized_planning_results.back(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

TEST(TaskExecutorPickPlaceTest, RecipeApproachDistanceChangesObjectCentricPlaceApproach)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.object_to_grasp_tcp.translation.z = 0.1;
  auto goal = place_goal();
  goal.target_pose.pose.position.z = 0.23;
  goal.place_approach_distance_m = 0.3;
  goal.place_retract_distance_m = 0.4;
  TaskExecutor executor_with_recipe_distance(
    backend, std::make_shared<RecordingGripperPort>(*backend), geometry);
  executor_with_recipe_distance.set_object_state(MotionObjectState::ATTACHED);

  ASSERT_EQ(executor_with_recipe_distance.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_FALSE(backend->requests.empty());
  EXPECT_DOUBLE_EQ(backend->requests.front().approach_target.pose.position.z, 0.63);
  EXPECT_DOUBLE_EQ(backend->requests.front().retract_target.pose.position.z, 0.73);
}

TEST(TaskExecutorPickPlaceTest, NonUnitInsertionAxisDoesNotScaleConfiguredDistances)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.insertion_axis_tcp.z = 2.0;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);
  auto goal = pick_goal();
  goal.pick_approach_distance_m = 0.2;

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->tcp_z_values.size(), 3U);
  EXPECT_NEAR(backend->tcp_z_values[0], 0.85, 1e-6);
  EXPECT_NEAR(backend->tcp_z_values[1], 1.05, 1e-6);
  EXPECT_NEAR(backend->tcp_z_values[2], 0.65, 1e-6);
}

TEST(TaskExecutorPickPlaceTest, PickWaitsForEachMotionPhaseBeforeContinuing)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->block_motion = true;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});

  ASSERT_TRUE(backend->wait_for_motion_call(1));
  EXPECT_EQ(backend->events, (std::vector<std::string>{"gripper", "motion:3"}));
  EXPECT_EQ(
    result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  backend->release_current_motion();
  ASSERT_TRUE(backend->wait_for_motion_call(2));
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"gripper", "motion:3", "motion:4"}));
  EXPECT_EQ(
    result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  backend->release_current_motion();
  ASSERT_TRUE(backend->wait_for_motion_call(3));
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{
      "gripper", "motion:3", "motion:4", "gripper", "motion:8"}));
  EXPECT_EQ(
    result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);

  backend->release_current_motion();
  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

TEST(TaskExecutorPickPlaceTest, SafetyStopDuringApproachInvalidatesLateMotionResult)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->block_motion = true;
  auto interrupted = std::make_shared<std::atomic<uint8_t>>(
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    backend, std::make_shared<RecordingGripperPort>(*backend), valid_geometry(),
    [interrupted]() {return interrupted->load();});
  auto result = std::async(
    std::launch::async, [&executor]() {
      return executor.execute(pick_goal());
    });

  ASSERT_TRUE(backend->wait_for_motion_call(1));
  interrupted->store(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  backend->release_current_motion();
  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(backend->requests.size(), 1U);
  EXPECT_EQ(backend->events, (std::vector<std::string>{"gripper", "motion:3"}));
}

TEST(TaskExecutorPickPlaceTest, CancelAndSafetyBeforeMovementDoNotSubmitMotion)
{
  for (const auto interrupted_code : {
      MotionTaskResultCode::TASK_RESULT_CANCELED,
      MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED})
  {
    auto backend = std::make_shared<RecordingBackend>();
    TaskExecutor executor(
      backend, std::make_shared<RecordingGripperPort>(*backend), valid_geometry(),
      [interrupted_code]() {return interrupted_code;});
    EXPECT_EQ(executor.execute(pick_goal()), interrupted_code);
    EXPECT_TRUE(backend->events.empty());
    EXPECT_TRUE(backend->requests.empty());
  }
}

TEST(TaskExecutorPickPlaceTest, MotionPhaseFailureDoesNotContinuePick)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->motion_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  EXPECT_EQ(backend->events, (std::vector<std::string>{"gripper", "motion:3"}));
}

TEST(TaskExecutorPickPlaceTest, ObjectYawRotatesTcpTranslationAndComposesToolOrientation)
{
  constexpr double pi = 3.14159265358979323846;
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.object_to_grasp_tcp.translation.x = 0.1;
  geometry.object_to_grasp_tcp.translation.z = 0.0;
  geometry.object_to_grasp_tcp.rotation.z = std::sin(pi / 4.0);
  geometry.object_to_grasp_tcp.rotation.w = std::cos(pi / 4.0);
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);

  auto goal = pick_goal();
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.z = std::sin(pi / 4.0);
  goal.target_pose.pose.orientation.w = std::cos(pi / 4.0);

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->tcp_orientations.size(), 3U);
  ASSERT_EQ(backend->tcp_z_values.size(), 3U);
  EXPECT_NEAR(backend->tcp_x_values[1], 0.0, 1e-9);
  EXPECT_NEAR(backend->tcp_y_values[1], 0.1, 1e-9);
  EXPECT_NEAR(backend->tcp_z_values[1], -0.05, 1e-6);
  EXPECT_NEAR(backend->tcp_orientations[1].z, 1.0, 1e-9);
  EXPECT_NEAR(backend->tcp_orientations[1].w, 0.0, 1e-9);
}

TEST(TaskExecutorPickPlaceTest, PickFailureBeforeAttachmentCannotReportSuccessOrAttach)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
}

TEST(TaskExecutorPickPlaceTest, GripperSuccessWithoutConfirmationCannotAttachOrSucceed)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
  EXPECT_EQ(backend->events.back(), "gripper");
}

TEST(TaskExecutorPickPlaceTest, FullyClosedWidthBoundariesControlRetryReopen)
{
  for (const auto & [width_mm, expect_reopen] :
    std::vector<std::pair<double, bool>>{{-1.0, false}, {0.0, true}, {0.5, true}, {0.51, false}})
  {
    auto backend = std::make_shared<RecordingBackend>();
    auto gripper = std::make_shared<RecordingGripperPort>(*backend);
    gripper->auto_observe = false;
    gripper->set_observation(HoldingState::RELEASED, 1);
    gripper->set_width(width_mm);
    TaskExecutor executor(backend, gripper, valid_geometry(), {}, std::chrono::milliseconds(1));

    EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
    EXPECT_EQ(backend->gripper_widths.size(), expect_reopen ? 2U : 1U) << "width_mm=" << width_mm;
    if (expect_reopen) {
      EXPECT_EQ(backend->gripper_widths.back(), 85.0F) << "width_mm=" << width_mm;
      EXPECT_EQ(gripper->holding().state, HoldingState::RELEASED);
      EXPECT_GT(gripper->holding().sequence, 1U);
    }
    EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
  }
}

TEST(TaskExecutorPickPlaceTest, GraspConfirmationFailureCannotAttachOrSucceed)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
}

TEST(TaskExecutorPickPlaceTest, FailedGripperCommandIsStoppedWhenStillActive)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  backend->gripper_active_value = true;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(backend->stop_gripper_calls, 1);
}

TEST(TaskExecutorPickPlaceTest, AttachmentFailureCannotReportSuccessOrMarkHeld)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
  EXPECT_EQ(backend->events.back(), "gripper");
}

TEST(TaskExecutorPickPlaceTest, PlaceOrdersReleaseDetachRetreatAndClearsOwnership)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{
      "motion:3", "motion:4", "gripper", "motion:8"}));
  EXPECT_EQ(backend->gripper_widths, (std::vector<float>{85.0F}));
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
}

TEST(TaskExecutorPickPlaceTest, PlaceReleaseTimeoutUsesOnlyTheThreeAllowedOpenRetries)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto gripper = std::make_shared<RecordingGripperPort>(*backend);
  gripper->set_observation(HoldingState::HELD, 1U);
  gripper->open_auto_observe = false;
  TaskExecutor executor(
    backend, gripper, valid_geometry(), TaskExecutor::InterruptionChecker{},
    std::chrono::milliseconds(5));
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_UNKNOWN);
  EXPECT_EQ(backend->open_calls, 4);
  EXPECT_EQ(std::count(backend->events.begin(), backend->events.end(), "motion:8"), 0);
}

TEST(TaskExecutorPickPlaceTest, NormalPickPlaceReportsHeldThenPlacedDisposition)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  ASSERT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_HELD);
  ASSERT_EQ(
    executor.execute(place_goal(), MotionObjectState::ATTACHED),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_PLACED);
}

TEST(TaskExecutorPickPlaceTest, HeldPickRetractFailureOpensAtCurrentPositionAndReturns)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->fail_pick_retract = true;
  auto gripper = std::make_shared<RecordingGripperPort>(*backend);
  TaskExecutor executor(backend, gripper, valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
  EXPECT_EQ(backend->open_calls, 2);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{
      "gripper", "motion:3", "motion:4", "gripper", "motion:8", "gripper"}));
}

TEST(TaskExecutorPickPlaceTest, RetreatFailureCanReplanAndPreservesPlacedDisposition)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->place_retreat_failures_remaining = 1;
  backend->place_retreat_retries_remaining = 1;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_PLACED);
  EXPECT_EQ(backend->open_calls, 1);
  EXPECT_EQ(
    std::count(backend->events.begin(), backend->events.end(), "motion:8"), 2);
}

TEST(TaskExecutorPickPlaceTest, ExhaustedRetreatReplanningReturnsFailureWithPlacedDisposition)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->place_retreat_failures_remaining = 3;
  backend->place_retreat_retries_remaining = 2;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_PLACED);
  EXPECT_EQ(backend->open_calls, 1);
  EXPECT_EQ(
    std::count(backend->events.begin(), backend->events.end(), "motion:8"), 3);
}

TEST(TaskExecutorPickPlaceTest, PlaceApproachPlanningExhaustionReturnsHeldObjectInPlace)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->fail_place_approach_planning = true;
  auto gripper = std::make_shared<RecordingGripperPort>(*backend);
  gripper->set_observation(HoldingState::HELD, 1U);
  TaskExecutor executor(backend, gripper, valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
  EXPECT_EQ(backend->events, (std::vector<std::string>{"motion:3", "gripper"}));
  EXPECT_EQ(backend->open_calls, 1);
}

TEST(TaskExecutorPickPlaceTest, PlaceInsertionExhaustionReturnsViaHomeAndTray)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->fail_place_insertion_planning = true;
  auto gripper = std::make_shared<RecordingGripperPort>(*backend);
  gripper->set_observation(HoldingState::HELD, 1U);
  gripper->refresh_observation_on_read = true;
  TaskExecutor executor(backend, gripper, valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_EQ(executor.object_disposition(), ExecuteTask::Result::OBJECT_DISPOSITION_RETURNED);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"motion:3", "motion:4", "home", "tray", "gripper"}));
  EXPECT_EQ(backend->open_calls, 1);
}

TEST(TaskExecutorPickPlaceTest, GripperCommandAcknowledgementIsDiagnosticDetail)
{
  ScopedEnvironmentVariable record_directory("ARM_CELL_PICK_EXECUTION_RECORD_DIR");
  const auto directory = std::filesystem::temp_directory_path() /
    ("arm-cell-command-log-record-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  ASSERT_EQ(::setenv("ARM_CELL_PICK_EXECUTION_RECORD_DIR", directory.c_str(), 1), 0);

  ScopedTaskExecutorLogCapture capture;
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  ASSERT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    std::find_if(
      captured_logs.begin(), captured_logs.end(), [](const auto & entry) {
      return entry.format.find("gripper open command issued") != std::string::npos;
    }), captured_logs.end());

  auto info_pick_backend = std::make_shared<RecordingBackend>();
  TaskExecutor info_pick_executor(
    info_pick_backend, std::make_shared<RecordingGripperPort>(*info_pick_backend),
    valid_geometry());
  ASSERT_EQ(info_pick_executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    std::find_if(
      captured_logs.begin(), captured_logs.end(), [](const auto & entry) {
      return entry.format.find("gripper close command issued") != std::string::npos;
    }), captured_logs.end());

  capture.set_level(RCUTILS_LOG_SEVERITY_DEBUG);
  executor.set_object_state(MotionObjectState::ATTACHED);
  ASSERT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  const auto command_log = std::find_if(
    captured_logs.begin(), captured_logs.end(), [](const auto & entry) {
      return entry.format.find("gripper open command issued") != std::string::npos;
    });
  ASSERT_NE(command_log, captured_logs.end());
  EXPECT_EQ(command_log->severity, RCUTILS_LOG_SEVERITY_DEBUG);

  auto pick_backend = std::make_shared<RecordingBackend>();
  TaskExecutor pick_executor(
    pick_backend, std::make_shared<RecordingGripperPort>(*pick_backend), valid_geometry());
  ASSERT_EQ(pick_executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  const auto close_command_log = std::find_if(
    captured_logs.begin(), captured_logs.end(), [](const auto & entry) {
      return entry.format.find("gripper close command issued") != std::string::npos;
    });
  ASSERT_NE(close_command_log, captured_logs.end());
  EXPECT_EQ(close_command_log->severity, RCUTILS_LOG_SEVERITY_DEBUG);
}

TEST(TaskExecutorPickPlaceTest, PlaceObjectReferenceComposesConfiguredToolOrientation)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto geometry = valid_geometry();
  geometry.object_to_grasp_tcp.translation.z = -0.0025;
  geometry.object_to_grasp_tcp.rotation.x = 1.0;
  geometry.object_to_grasp_tcp.rotation.w = 0.0;
  geometry.pick_retract_distance_m = 0.4;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend), geometry);
  executor.set_object_state(MotionObjectState::ATTACHED);

  auto goal = place_goal();
  goal.target_pose.pose.position.x = 0.35;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.position.z = 0.80;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  goal.place_approach_direction_object.z = 1.0;
  goal.place_approach_distance_m = 0.05;
  goal.place_retract_distance_m = 0.05;

  EXPECT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(backend->tcp_z_values.size(), 3U);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[0], 0.8475);
  EXPECT_DOUBLE_EQ(backend->requests.front().approach_distance_m, 0.05);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[1], 0.7975);
  EXPECT_DOUBLE_EQ(backend->tcp_z_values[2], 0.8475);
  ASSERT_EQ(backend->tcp_orientations.size(), 3U);
  for (const auto & orientation : backend->tcp_orientations) {
    EXPECT_DOUBLE_EQ(orientation.x, 1.0);
    EXPECT_DOUBLE_EQ(orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(orientation.z, 0.0);
    EXPECT_DOUBLE_EQ(orientation.w, 0.0);
  }
  EXPECT_DOUBLE_EQ(goal.target_pose.pose.orientation.w, 1.0);
}

TEST(TaskExecutorPickPlaceTest, SuccessfulConfirmationPermitsAttachThenRetract)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{
      "gripper", "motion:3", "motion:4", "gripper", "motion:8"}));
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
}

TEST(TaskExecutorPickPlaceTest, DetachFailureCannotReportSuccessOrMarkReleased)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->gripper_result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
  EXPECT_EQ(backend->events.back(), "gripper");
}

TEST(TaskExecutorPickPlaceTest, InvalidGeometryFailsClosedBeforePlanning)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend);

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_TRUE(backend->events.empty());
}

namespace
{
class MandatoryOnlyBackend final : public MotionBackend
{
public:
  bool available() const override {return true;}
  bool submit(const ExecuteTask::Goal &) override {return true;}
  uint8_t execute_normalized(const NormalizedMotionRequest &) override
  {
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }
  void cancel() override {}
  void hold() override {}
  void stop() override {}
  bool execution_active() const override {return false;}
  bool backend_inactivity_confirmed() const override {return true;}
};

TEST(TaskExecutorPickPlaceTest, PickRejectsCommandSuccessUntilNewHeldObservationArrives)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 7);
  TaskExecutor executor(backend, port, valid_geometry());

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});

  ASSERT_TRUE(backend->wait_for_gripper_commands(2, 0, 1));
  ASSERT_TRUE(port->wait_for_holding_after_command(2));
  EXPECT_EQ(
    result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  port->publish(HoldingState::HELD);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
}

TEST(TaskExecutorPickPlaceTest, PickWaitsThroughNewUnknownUntilHeldIsObserved)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 7);
  TaskExecutor executor(backend, port, valid_geometry());

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});

  ASSERT_TRUE(backend->wait_for_gripper_commands(2, 0, 1));
  ASSERT_TRUE(port->wait_for_holding_after_command(2));
  port->publish(HoldingState::UNKNOWN);
  EXPECT_EQ(
    result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  port->publish(HoldingState::HELD);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
}

TEST(TaskExecutorPickPlaceTest, PickRejectsUnknownThatPersistsUntilConfirmationDeadline)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 7);
  TaskExecutor executor(
    backend, port, valid_geometry(), {}, std::chrono::milliseconds(10));

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});

  ASSERT_TRUE(backend->wait_for_gripper_commands(2, 0, 1));
  ASSERT_TRUE(port->wait_for_holding_after_command(2));
  port->publish(HoldingState::UNKNOWN);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
}

TEST(TaskExecutorPickPlaceTest, LateHeldAfterConfirmationTimeoutCannotSucceed)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 7);
  TaskExecutor executor(
    backend, port, valid_geometry(), {}, std::chrono::milliseconds(10));

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});

  ASSERT_TRUE(backend->wait_for_gripper_commands(2, 0, 1));
  ASSERT_TRUE(port->wait_for_holding_after_command(2));
  ASSERT_EQ(result.wait_for(std::chrono::milliseconds(200)), std::future_status::ready);
  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);

  port->publish(HoldingState::HELD);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"gripper", "motion:3", "motion:4", "gripper"}));
}

TEST(TaskExecutorPickPlaceTest, PlaceRejectsCommandSuccessUntilNewReleasedObservationArrives)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->open_auto_observe = false;
  port->set_observation(HoldingState::HELD, 11);
  TaskExecutor executor(backend, port, valid_geometry());
  executor.set_object_state(MotionObjectState::ATTACHED);

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(place_goal());});

  ASSERT_TRUE(backend->wait_for_gripper_commands(1, 1));
  ASSERT_TRUE(port->wait_for_holding_after_command(1));
  EXPECT_EQ(
    result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  port->publish(HoldingState::RELEASED);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);
}


TEST(TaskExecutorPickPlaceTest, PickCancellationDuringHoldingWaitBlocksRetract)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 20);
  auto interruption = std::make_shared<std::atomic<uint8_t>>(
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    backend, port, valid_geometry(),
    [interruption]() {return interruption->load();});

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});
  ASSERT_TRUE(backend->wait_for_gripper_commands(2, 0, 1));
  ASSERT_TRUE(port->wait_for_holding_after_command(2));

  interruption->store(MotionTaskResultCode::TASK_RESULT_CANCELED);
  port->publish(HoldingState::HELD);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_CANCELED);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"gripper", "motion:3", "motion:4", "gripper"}));
}

TEST(TaskExecutorPickPlaceTest, PickSafetyPreemptionDuringHoldingWaitBlocksRetract)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 20);
  auto interruption = std::make_shared<std::atomic<uint8_t>>(
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    backend, port, valid_geometry(),
    [interruption]() {return interruption->load();});

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});
  ASSERT_TRUE(backend->wait_for_gripper_commands(2, 0, 1));
  ASSERT_TRUE(port->wait_for_holding_after_command(2));

  interruption->store(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  port->publish(HoldingState::HELD);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"gripper", "motion:3", "motion:4", "gripper"}));
}

TEST(TaskExecutorPickPlaceTest, PlaceCancellationDuringHoldingWaitBlocksRetreat)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->open_auto_observe = false;
  port->set_observation(HoldingState::HELD, 20);
  auto interruption = std::make_shared<std::atomic<uint8_t>>(
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    backend, port, valid_geometry(),
    [interruption]() {return interruption->load();});
  executor.set_object_state(MotionObjectState::ATTACHED);

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(place_goal());});
  ASSERT_TRUE(backend->wait_for_gripper_commands(1, 1));
  ASSERT_TRUE(port->wait_for_holding_after_command(1));

  interruption->store(MotionTaskResultCode::TASK_RESULT_CANCELED);
  port->publish(HoldingState::RELEASED);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_CANCELED);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"motion:3", "motion:4", "gripper"}));
}

TEST(TaskExecutorPickPlaceTest, PlaceSafetyPreemptionDuringHoldingWaitBlocksRetreat)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->open_auto_observe = false;
  port->set_observation(HoldingState::HELD, 20);
  auto interruption = std::make_shared<std::atomic<uint8_t>>(
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    backend, port, valid_geometry(),
    [interruption]() {return interruption->load();});
  executor.set_object_state(MotionObjectState::ATTACHED);

  auto result = std::async(
    std::launch::async, [&executor]() {return executor.execute(place_goal());});
  ASSERT_TRUE(backend->wait_for_gripper_commands(1, 1));
  ASSERT_TRUE(port->wait_for_holding_after_command(1));

  interruption->store(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  port->publish(HoldingState::RELEASED);

  EXPECT_EQ(result.get(), MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(
    backend->events,
    (std::vector<std::string>{"motion:3", "motion:4", "gripper"}));
}

TEST(TaskExecutorPickPlaceTest, InterruptedPickDoesNotPoisonNextIndependentPick)
{
  auto backend = std::make_shared<RecordingBackend>();
  auto port = std::make_shared<RecordingGripperPort>(*backend);
  port->auto_observe = false;
  port->set_observation(HoldingState::RELEASED, 20);
  auto interruption = std::make_shared<std::atomic<uint8_t>>(
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  TaskExecutor executor(
    backend, port, valid_geometry(),
    [interruption]() {return interruption->load();});

  auto interrupted = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});
  ASSERT_TRUE(backend->wait_for_gripper_commands(2, 0, 1));
  ASSERT_TRUE(port->wait_for_holding_after_command(2));
  interruption->store(MotionTaskResultCode::TASK_RESULT_CANCELED);
  port->publish(HoldingState::HELD);
  EXPECT_EQ(interrupted.get(), MotionTaskResultCode::TASK_RESULT_CANCELED);

  interruption->store(MotionTaskResultCode::TASK_RESULT_SUCCESS);
  port->publish(HoldingState::RELEASED);
  auto next = std::async(
    std::launch::async, [&executor]() {return executor.execute(pick_goal());});
  ASSERT_TRUE(backend->wait_for_gripper_commands(4, 0, 2));
  ASSERT_TRUE(port->wait_for_holding_after_command(4));
  port->publish(HoldingState::HELD);

  EXPECT_EQ(next.get(), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
}

}  // namespace

TEST(TaskExecutorPickPlaceTest, UnsupportedOptionalCapabilitiesFailClosedForPickAndPlace)
{
  auto backend = std::make_shared<MandatoryOnlyBackend>();
  TaskExecutor executor(backend, valid_geometry());

  EXPECT_EQ(executor.execute(pick_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::NO_OBJECT);

  executor.set_object_state(MotionObjectState::ATTACHED);
  EXPECT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  EXPECT_EQ(executor.object_state(), MotionObjectState::ATTACHED);
}

TEST(TaskExecutorPickPlaceTest, ExhaustedMaterialSearchRequestsUpperRecovery)
{
  auto backend = std::make_shared<RecordingBackend>();
  backend->pick_approach_results = {
    MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED};
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  const auto result = executor.execute_pnp(execute_pnp_goal(), "exhausted-search");
  EXPECT_EQ(
    result.pnp_outcome,
    arm_cell_interfaces::action::ExecutePnp::Result::PNP_OUTCOME_CANDIDATES_EXHAUSTED);
  EXPECT_FALSE(result.material_placed);
}

}  // namespace arm_cell_task_execution

namespace arm_cell_task_execution
{
using namespace arm_cell_motion;
TEST(TaskExecutorPickPlaceTest, PickCarriesVisionOccupancyAndPlaceDoesNot)
{
  auto backend = std::make_shared<RecordingBackend>();
  TaskExecutor executor(backend, std::make_shared<RecordingGripperPort>(*backend),
    valid_geometry());
  auto goal = pick_goal();
  goal.vision_candidate_id = "candidate-2";
  goal.target_pose.pose.position.z = 0.16;
  goal.estimated_object_height_m = 0.08F;
  goal.has_non_target_occupancy = true;
  arm_cell_interfaces::msg::CollisionPrimitive occupancy;
  occupancy.occupancy_class = occupancy.UNKNOWN_OCCUPANCY;
  occupancy.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  occupancy.shape.dimensions = {0.1, 0.1, 0.08};
  occupancy.pose.orientation.w = 1.0;
  occupancy.pose.position.x = 0.2;
  goal.non_target_occupancy = {occupancy};

  ASSERT_EQ(executor.execute(goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  const auto pick_approach = std::find_if(
    backend->requests.begin(), backend->requests.end(),
    [](const auto & request) {
      return request.task_type.value == MotionTaskType::TASK_TYPE_PICK &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
    });
  ASSERT_NE(pick_approach, backend->requests.end());
  EXPECT_TRUE(pick_approach->has_non_target_occupancy);
  EXPECT_EQ(pick_approach->non_target_occupancy, goal.non_target_occupancy);
  EXPECT_EQ(pick_approach->vision_candidate_id, "candidate-2");

  backend->requests.clear();
  ASSERT_EQ(executor.execute(place_goal()), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_FALSE(backend->requests.empty());
  for (const auto & request : backend->requests) {
    EXPECT_EQ(request.task_type.value, MotionTaskType::TASK_TYPE_PLACE);
    EXPECT_FALSE(request.has_non_target_occupancy);
    EXPECT_TRUE(request.non_target_occupancy.empty());
  }
}

}  // namespace arm_cell_task_execution
