#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rcutils/logging.h>
#include <arm_cell_interfaces/msg/motion_capability.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>

#include "arm_cell_motion_moveit2/fake_motion_backend.hpp"
#include "arm_cell_motion_moveit2/motion_core.hpp"

namespace arm_cell_motion_moveit2
{

namespace
{
using Clock = MotionCore::Clock;

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

class ScopedLogCapture
{
public:
  ScopedLogCapture()
  : previous_handler_(rcutils_logging_get_output_handler())
  {
    captured_logs.clear();
    rcutils_logging_set_output_handler(capture_log);
  }

  ~ScopedLogCapture()
  {
    rcutils_logging_set_output_handler(previous_handler_);
  }

private:
  rcutils_logging_output_handler_t previous_handler_;
};

class CoordinatedAvailabilityBackend final : public MotionBackend
{
public:
  bool available() const override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    availability_entered_ = true;
    entered_condition_.notify_one();
    release_condition_.wait(lock, [this]() {return release_availability_;});
    return true;
  }

  bool submit(const ExecuteTask::Goal &) override
  {
    ++submit_calls_;
    execution_active_ = true;
    return true;
  }

  void cancel() override {}
  void hold() override {}
  void stop() override {++stop_calls_;}
  bool execution_active() const override {return execution_active_;}
  bool backend_inactivity_confirmed() const override {return !execution_active_;}

  void wait_until_available_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_condition_.wait(lock, [this]() {return availability_entered_;});
  }

  void release_availability()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    release_availability_ = true;
    release_condition_.notify_one();
  }

  int submit_calls() const {return submit_calls_;}
  int stop_calls() const {return stop_calls_;}

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable entered_condition_;
  mutable std::condition_variable release_condition_;
  mutable bool availability_entered_{false};
  mutable bool release_availability_{false};
  bool execution_active_{false};
  int submit_calls_{0};
  int stop_calls_{0};
};

class TaskExecutorAvailabilityBackend final : public MotionBackend
{
public:
  bool available() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++availability_calls_;
    if (availability_calls_ == 2) {
      if (on_second_availability_) {
        on_second_availability_();
      }
      return false;
    }
    return true;
  }

  bool submit(const ExecuteTask::Goal &) override
  {
    ++submit_calls_;
    execution_active_ = true;
    return true;
  }

  void cancel() override {}
  void hold() override {}
  void stop() override {}
  bool execution_active() const override {return execution_active_;}
  bool backend_inactivity_confirmed() const override {return !execution_active_;}

  int submit_calls() const {return submit_calls_;}
  int availability_calls() const {return availability_calls_;}
  void set_on_second_availability(std::function<void()> callback)
  {
    on_second_availability_ = std::move(callback);
  }

private:
  mutable std::mutex mutex_;
  mutable int availability_calls_{0};
  std::function<void()> on_second_availability_;
  bool execution_active_{false};
  int submit_calls_{0};
};

class InterruptingGripperPort final : public GripperPort
{
public:
  uint8_t close(float) override {return MotionTaskResultCode::TASK_RESULT_SUCCESS;}
  uint8_t open() override {return MotionTaskResultCode::TASK_RESULT_SUCCESS;}
  HoldingObservation holding() const override {return observation;}
  bool active() const override {return active_value;}
  void stop() override
  {
    ++stop_calls;
    active_value = false;
  }

  HoldingObservation observation;
  bool active_value{true};
  int stop_calls{0};
};

SafetyState safety_state(uint8_t capability, Clock::time_point receipt_time)
{
  SafetyState state;
  state.safety_state = SafetyState::SAFETY_STATE_SAFE;
  state.motion_capability.value = capability;
  state.valid = true;
  state.required_inputs_fresh = true;
  state.motion_envelope_valid = true;
  state.max_velocity_scale = 1.0F;
  state.max_acceleration_scale = 1.0F;
  state.header.stamp.sec = static_cast<int32_t>(
    std::chrono::duration_cast<std::chrono::seconds>(receipt_time.time_since_epoch()).count());
  return state;
}

void grant_normal(MotionCore & core, Clock::time_point now = Clock::now())
{
  core.update_safety_state(
    safety_state(arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL, now), now);
}

MotionGeometry valid_geometry()
{
  MotionGeometry geometry;
  geometry.configured = true;
  geometry.observation_reference = "profile_reference";
  geometry.observation_to_object.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.rotation.w = 1.0;
  geometry.insertion_axis_tcp.z = 1.0;
  geometry.pick_retract_distance_m = 0.2;
  geometry.pick_approach_distance_m = 0.1;
  return geometry;
}

ExecuteTask::Goal pick_goal()
{
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_PICK;
  goal.has_target_pose = true;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.pose.orientation.w = 1.0;
  goal.has_grasp_width = true;
  goal.grasp_width_mm = 20.0F;
  return goal;
}
}  // namespace

TEST(MotionEnvelope, CapsCalibratedPlanningScales)
{
  const MotionEnvelope envelope{0.5F, 0.25F, true};
  const auto scales = apply_motion_envelope_to_planning_scales(0.1, 0.2, envelope);

  EXPECT_DOUBLE_EQ(scales.max_velocity, 0.05);
  EXPECT_DOUBLE_EQ(scales.max_acceleration, 0.05);
}

TEST(MotionCoreTest, CallerCancellationStopsActiveGripperPort)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<InterruptingGripperPort>();
  backend->set_gripper_port(port);
  MotionCore core(backend);
  grant_normal(core);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  const auto goal_uuid = make_uuid(41);
  ASSERT_EQ(core.reserve_goal(goal_uuid, goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  EXPECT_TRUE(core.request_cancel(goal_uuid));
  EXPECT_EQ(port->stop_calls, 1);
  EXPECT_FALSE(port->active());
}

TEST(MotionCoreTest, AppliesAndReportsCurrentSafetyEnvelopeForNewTask)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  const auto now = Clock::now();
  auto state = safety_state(
    arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL, now);
  state.max_velocity_scale = 0.4F;
  state.max_acceleration_scale = 0.3F;
  core.update_safety_state(state, now);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  const auto uuid = make_uuid(93);
  ASSERT_EQ(core.reserve_goal(uuid, goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(core.accept_goal(uuid, goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  const auto status = core.status();
  EXPECT_TRUE(status.applied_envelope_valid);
  EXPECT_FLOAT_EQ(status.applied_velocity_scale, 0.4F);
  EXPECT_FLOAT_EQ(status.applied_acceleration_scale, 0.3F);
  EXPECT_TRUE(status.execution_active);
}

TEST(MotionCoreTest, RuntimeTuningIsSnapshottedForAnActionAndFrozenWhilePending)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend, std::chrono::milliseconds(500), valid_geometry());
  MotionTuning first;
  first.planning_time_s = 23.0;
  const auto geometry = valid_geometry();
  ASSERT_TRUE(core.set_runtime_tuning(first, geometry, std::chrono::milliseconds(2300)));

  grant_normal(core);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  const auto uuid = make_uuid(94);
  ASSERT_EQ(core.reserve_goal(uuid, goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  auto changed_geometry = geometry;
  changed_geometry.pick_approach_distance_m = 0.2;
  EXPECT_FALSE(core.set_runtime_tuning(
      first, changed_geometry, std::chrono::milliseconds(2300)));
  MotionTuning next = first;
  next.planning_time_s = 12.0;
  EXPECT_TRUE(core.set_runtime_tuning(
      next, geometry, std::chrono::milliseconds(1200), false));
  ASSERT_EQ(core.accept_goal(uuid, goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_DOUBLE_EQ(backend->applied_tuning().planning_time_s, 23.0);

  MotionTuning later = first;
  later.planning_time_s = 11.0;
  EXPECT_TRUE(core.set_runtime_tuning(
      later, geometry, std::chrono::milliseconds(1100), false));
  EXPECT_DOUBLE_EQ(backend->applied_tuning().planning_time_s, 23.0);
  backend->set_execution_active(false);
  backend->set_inactivity_confirmed(true);
  core.refresh_stop_state();
  const auto next_uuid = make_uuid(95);
  ASSERT_EQ(core.reserve_goal(next_uuid, goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(core.accept_goal(next_uuid, goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_DOUBLE_EQ(backend->applied_tuning().planning_time_s, 11.0);
}

TEST(MotionCoreTest, StatusPublishesFreshHoldingObservationAndFailsClosedWhenStale)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<InterruptingGripperPort>();
  backend->set_gripper_port(port);
  MotionCore core(backend);

  port->observation = HoldingObservation{
    HoldingState::HELD, Clock::now(), 1};
  EXPECT_EQ(core.status().holding_state, MotionStatus::HOLDING_HELD);

  port->observation = HoldingObservation{
    HoldingState::RELEASED, Clock::now(), 2};
  EXPECT_EQ(core.status().holding_state, MotionStatus::HOLDING_RELEASED);

  port->observation = HoldingObservation{
    HoldingState::HELD, Clock::now() - std::chrono::seconds(1), 3};
  EXPECT_EQ(core.status().holding_state, MotionStatus::HOLDING_UNKNOWN);

  port->observation = HoldingObservation{};
  EXPECT_EQ(core.status().holding_state, MotionStatus::HOLDING_UNKNOWN);
}

TEST(MotionCoreTest, SafetyPreemptionStopsActiveGripperPort)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<InterruptingGripperPort>();
  backend->set_gripper_port(port);
  MotionCore core(backend);
  grant_normal(core);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  const auto goal_uuid = make_uuid(42);
  ASSERT_EQ(core.reserve_goal(goal_uuid, goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  EXPECT_TRUE(
    core.request_stop(
      goal_uuid, arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED));
  EXPECT_EQ(port->stop_calls, 1);
  EXPECT_FALSE(port->active());
}

class DelayedHoldingPort final : public GripperPort
{
public:
  uint8_t close(float) override
  {
    active_.store(true);
    close_called_.store(true);
    command_called_.store(true);
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  uint8_t open() override
  {
    active_.store(true);
    command_called_.store(true);
    return MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  HoldingObservation holding() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return observation_;
  }

  bool active() const override {return active_.load();}

  void stop() override
  {
    active_.store(false);
    ++stop_calls_;
  }

  bool wait_for_command()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!command_called_.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return command_called_.load();
  }

  bool wait_for_close_command()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!close_called_.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return close_called_.load();
  }

  void publish(HoldingState state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    observation_.state = state;
    observation_.observed_at = HoldingObservation::Clock::now();
    ++observation_.sequence;
  }

  void set_observation(HoldingState state, std::uint64_t sequence)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    observation_.state = state;
    observation_.observed_at = HoldingObservation::Clock::now();
    observation_.sequence = sequence;
  }

  int stop_calls() const {return stop_calls_.load();}

private:
  mutable std::mutex mutex_;
  HoldingObservation observation_;
  std::atomic_bool active_{false};
  std::atomic_bool command_called_{false};
  std::atomic_bool close_called_{false};
  std::atomic_int stop_calls_{0};
};

ExecuteTask::Goal place_goal()
{
  auto goal = pick_goal();
  goal.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  goal.place_approach_direction_object.z = 1.0;
  goal.place_approach_distance_m = 0.1;
  goal.place_retract_distance_m = 0.2;
  return goal;
}

TEST(MotionCoreTest, StartsWithNoMotionCapability)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;

  EXPECT_EQ(
    core.accept_goal(make_uuid(1), goal),
    MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
  EXPECT_FALSE(backend->execution_active());
}

TEST(MotionCoreTest, NormalTasksRequireNormalCapability)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  EXPECT_EQ(core.accept_goal(make_uuid(2), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

TEST(MotionCoreTest, RecoveryOnlyCapabilityPermitsOnlyRetract)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  const auto now = Clock::time_point{};
  core.update_safety_state(
    safety_state(arm_cell_interfaces::msg::MotionCapability::MOTION_RECOVERY_ONLY, now), now);

  ExecuteTask::Goal normal_goal;
  normal_goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  EXPECT_EQ(
    core.accept_goal(make_uuid(3), normal_goal, now),
    MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);

  ExecuteTask::Goal retract_goal;
  retract_goal.task_type.value = MotionTaskType::TASK_TYPE_RETRACT;
  EXPECT_EQ(
    core.accept_goal(make_uuid(4), retract_goal, now),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

TEST(MotionCoreTest, UncertainObjectStateRejectsRetractWithoutBackendExecution)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  const auto now = Clock::time_point{};
  core.update_safety_state(
    safety_state(arm_cell_interfaces::msg::MotionCapability::MOTION_RECOVERY_ONLY, now), now);
  core.update_object_state(MotionObjectState::UNCERTAIN);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_RETRACT;

  EXPECT_EQ(
    core.accept_goal(make_uuid(40), goal, now),
    MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_FALSE(backend->execution_active());
}

TEST(MotionCoreTest, InvalidUnknownAndStaleSafetyStateFailClosed)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  const auto now = Clock::time_point{};
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;

  auto invalid = safety_state(
    arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL, now);
  invalid.valid = false;
  core.update_safety_state(invalid, now);
  EXPECT_EQ(
    core.accept_goal(make_uuid(5), goal),
    MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);

  auto unknown = safety_state(99, now);
  core.update_safety_state(unknown, now);
  EXPECT_EQ(
    core.accept_goal(make_uuid(6), goal),
    MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);

  auto unknown_state = safety_state(
    arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL, now);
  unknown_state.safety_state = 99;
  core.update_safety_state(unknown_state, now);
  EXPECT_EQ(
    core.accept_goal(make_uuid(7), goal, now),
    MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);

  auto not_fresh = safety_state(
    arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL, now);
  not_fresh.required_inputs_fresh = false;
  core.update_safety_state(not_fresh, now);
  EXPECT_EQ(
    core.accept_goal(make_uuid(8), goal, now),
    MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);

  grant_normal(core, now);
  EXPECT_EQ(
    core.accept_goal(make_uuid(9), goal, now + std::chrono::seconds(1)),
    MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
}

TEST(MotionCoreTest, CapabilityDowngradeAfterAuthorizationPreventsSubmission)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  const auto now = Clock::time_point{};
  grant_normal(core, now);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  EXPECT_EQ(core.validate_goal(goal, now), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  core.update_safety_state(
    safety_state(arm_cell_interfaces::msg::MotionCapability::MOTION_NONE, now), now);
  EXPECT_EQ(
    core.accept_goal(make_uuid(9), goal, now),
    MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
  EXPECT_FALSE(backend->execution_active());
}

TEST(MotionCoreTest, SafetyDowngradeDeliveredDuringPreSubmitWorkBlocksSubmission)
{
  auto backend = std::make_shared<CoordinatedAvailabilityBackend>();
  MotionCore core(backend);
  grant_normal(core);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;

  auto result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  std::thread goal_thread([&]() {result = core.accept_goal(make_uuid(10), goal);});
  backend->wait_until_available_entered();

  auto downgraded = safety_state(
    arm_cell_interfaces::msg::MotionCapability::MOTION_NONE, Clock::now());
  core.update_safety_state(downgraded, Clock::now());
  EXPECT_EQ(
    core.validate_goal(goal),
    MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);

  backend->release_availability();
  goal_thread.join();

  EXPECT_EQ(result, MotionTaskResultCode::TASK_RESULT_PERMISSION_DENIED);
  EXPECT_EQ(backend->submit_calls(), 0);
}

TEST(MotionCoreTest, TaskExecutorDoesNotPerformAvailabilityAfterFinalCapabilityRecheck)
{
  auto backend = std::make_shared<TaskExecutorAvailabilityBackend>();
  MotionCore core(backend);
  grant_normal(core);
  backend->set_on_second_availability(
    [&]() {
      core.update_safety_state(
        safety_state(
          arm_cell_interfaces::msg::MotionCapability::MOTION_NONE,
          Clock::now()),
        Clock::now());
    });

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;

  EXPECT_EQ(core.accept_goal(make_uuid(41), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(backend->availability_calls(), 1);
  EXPECT_EQ(backend->submit_calls(), 1);
}

TEST(MotionCoreTest, PickAndPlaceUseTheMotionOwnedLifecycle)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend, std::chrono::milliseconds(500), valid_geometry());
  grant_normal(core);

  ExecuteTask::Goal pick;
  pick.task_type.value = MotionTaskType::TASK_TYPE_PICK;
  pick.has_target_pose = true;
  pick.target_pose.header.frame_id = "base_link";
  pick.target_pose.pose.orientation.w = 1.0;
  pick.has_grasp_width = true;
  pick.grasp_width_mm = 20.0F;
  ExecuteTask::Goal place;
  place.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  place.has_target_pose = true;
  place.target_pose.header.frame_id = "base_link";
  place.target_pose.pose.orientation.w = 1.0;
  place.has_grasp_width = true;
  place.grasp_width_mm = 20.0F;
  place.place_approach_direction_object.z = 1.0;
  place.place_approach_distance_m = 0.1;
  place.place_retract_distance_m = 0.2;

  EXPECT_EQ(core.accept_goal(make_uuid(42), pick), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  backend->set_execution_active(false);
  backend->set_inactivity_confirmed(true);
  core.refresh_stop_state();
  EXPECT_EQ(core.accept_goal(make_uuid(43), place), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(backend->submit_calls(), 6);
  EXPECT_EQ(backend->execute_task_calls(), 6);
}

TEST(MotionCoreTest, CapabilityDowngradeDoesNotInventStopForActiveExecution)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_EQ(core.accept_goal(make_uuid(11), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  auto downgraded = safety_state(
    arm_cell_interfaces::msg::MotionCapability::MOTION_NONE, Clock::now());
  core.update_safety_state(downgraded, Clock::now());

  EXPECT_EQ(backend->stop_calls(), 0);
  EXPECT_TRUE(backend->execution_active());
}

TEST(MotionCoreTest, StartsIdleWithConfirmedInactivity)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);

  const auto status = core.status();

  EXPECT_EQ(status.execution_state, MotionStatus::MOTION_STATE_IDLE);
  EXPECT_FALSE(status.has_active_execution);
  EXPECT_FALSE(status.execution_active);
  EXPECT_TRUE(status.backend_inactivity_confirmed);
}

TEST(MotionCoreTest, AcceptedGoalCorrelatesStatusWithActionGoalUuid)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  const auto uuid = make_uuid(42);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;

  ASSERT_EQ(core.accept_goal(uuid, goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  const auto status = core.status();

  EXPECT_TRUE(status.has_active_execution);
  EXPECT_EQ(status.active_execution_id, uuid);
  EXPECT_EQ(status.active_task_type.value, MotionTaskType::TASK_TYPE_GO_HOME);
  EXPECT_TRUE(status.execution_active);
}

TEST(MotionCoreTest, UnavailableBackendMapsToCanonicalResult)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_available(false);
  MotionCore core(backend);
  grant_normal(core);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;

  EXPECT_EQ(
    core.accept_goal(make_uuid(7), goal),
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_FALSE(core.status().has_active_execution);
}

TEST(MotionCoreTest, StopDoesNotBecomeStoppedBeforeBackendConfirmsInactivity)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_EQ(core.accept_goal(make_uuid(8), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  ASSERT_TRUE(core.request_stop(make_uuid(9)));
  EXPECT_EQ(core.status().execution_state, MotionStatus::MOTION_STATE_STOPPING);

  backend->set_execution_active(false);
  backend->set_inactivity_confirmed(false);
  core.refresh_stop_state();
  EXPECT_EQ(core.status().execution_state, MotionStatus::MOTION_STATE_STOPPING);

  backend->set_inactivity_confirmed(true);
  core.refresh_stop_state();
  EXPECT_EQ(core.status().execution_state, MotionStatus::MOTION_STATE_STOPPED);
  EXPECT_FALSE(core.status().execution_active);
  EXPECT_TRUE(core.status().backend_inactivity_confirmed);
}

TEST(MotionCoreTest, ClientCancelUsesBackendCancelWithoutSafetyStopLifecycle)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_EQ(core.accept_goal(make_uuid(13), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  ASSERT_TRUE(core.request_cancel());
  EXPECT_EQ(backend->cancel_calls(), 1);
  EXPECT_EQ(backend->stop_calls(), 0);
  EXPECT_EQ(core.status().execution_state, MotionStatus::MOTION_STATE_EXECUTING);

  backend->set_execution_active(false);
  backend->set_inactivity_confirmed(false);
  core.refresh_stop_state();
  EXPECT_EQ(core.status().execution_state, MotionStatus::MOTION_STATE_EXECUTING);

  backend->set_inactivity_confirmed(true);
  core.refresh_stop_state();
  EXPECT_EQ(core.status().execution_state, MotionStatus::MOTION_STATE_IDLE);
}

TEST(MotionCoreTest, CallerCancelDuringHoldingWaitBlocksPickRetract)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<DelayedHoldingPort>();
  port->set_observation(HoldingState::RELEASED, 10);
  backend->set_gripper_port(port);
  MotionCore core(backend, std::chrono::milliseconds(500), valid_geometry());
  grant_normal(core);
  const auto uuid = make_uuid(56);

  auto execution = std::async(
    std::launch::async, [&]() {return core.accept_goal(uuid, pick_goal());});
  ASSERT_TRUE(port->wait_for_command());
  port->publish(HoldingState::RELEASED);
  ASSERT_TRUE(port->wait_for_close_command());

  ASSERT_TRUE(core.request_cancel(uuid));
  port->publish(HoldingState::HELD);

  ASSERT_EQ(execution.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(execution.get(), MotionTaskResultCode::TASK_RESULT_CANCELED);
  EXPECT_EQ(backend->execute_task_calls(), 2);
  EXPECT_EQ(port->stop_calls(), 1);
}

TEST(MotionCoreTest, SafetyPreemptionDuringHoldingWaitBlocksPickRetract)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<DelayedHoldingPort>();
  port->set_observation(HoldingState::RELEASED, 10);
  backend->set_gripper_port(port);
  MotionCore core(backend, std::chrono::milliseconds(500), valid_geometry());
  grant_normal(core);
  const auto uuid = make_uuid(57);

  auto execution = std::async(
    std::launch::async, [&]() {return core.accept_goal(uuid, pick_goal());});
  ASSERT_TRUE(port->wait_for_command());
  port->publish(HoldingState::RELEASED);
  ASSERT_TRUE(port->wait_for_close_command());

  ASSERT_TRUE(
    core.request_stop(
      uuid, arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED));
  port->publish(HoldingState::HELD);

  ASSERT_EQ(execution.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(execution.get(), MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(backend->execute_task_calls(), 2);
  EXPECT_EQ(port->stop_calls(), 1);
}

TEST(MotionCoreTest, CallerCancelDuringHoldingWaitBlocksPlaceRetreat)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<DelayedHoldingPort>();
  port->set_observation(HoldingState::HELD, 10);
  backend->set_gripper_port(port);
  MotionCore core(backend, std::chrono::milliseconds(500), valid_geometry());
  core.update_object_state(MotionObjectState::ATTACHED);
  grant_normal(core);
  const auto uuid = make_uuid(58);

  auto execution = std::async(
    std::launch::async, [&]() {return core.accept_goal(uuid, place_goal());});
  ASSERT_TRUE(port->wait_for_command());

  ASSERT_TRUE(core.request_cancel(uuid));
  port->publish(HoldingState::RELEASED);

  ASSERT_EQ(execution.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(execution.get(), MotionTaskResultCode::TASK_RESULT_CANCELED);
  EXPECT_EQ(backend->execute_task_calls(), 2);
  EXPECT_EQ(port->stop_calls(), 1);
}

TEST(MotionCoreTest, SafetyPreemptionDuringHoldingWaitBlocksPlaceRetreat)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  auto port = std::make_shared<DelayedHoldingPort>();
  port->set_observation(HoldingState::HELD, 10);
  backend->set_gripper_port(port);
  MotionCore core(backend, std::chrono::milliseconds(500), valid_geometry());
  core.update_object_state(MotionObjectState::ATTACHED);
  grant_normal(core);
  const auto uuid = make_uuid(59);

  auto execution = std::async(
    std::launch::async, [&]() {return core.accept_goal(uuid, place_goal());});
  ASSERT_TRUE(port->wait_for_command());

  ASSERT_TRUE(
    core.request_stop(
      uuid, arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED));
  port->publish(HoldingState::RELEASED);

  ASSERT_EQ(execution.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(execution.get(), MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(backend->execute_task_calls(), 2);
  EXPECT_EQ(port->stop_calls(), 1);
}

TEST(MotionCoreTest, CallerCancelDuringExecutionReturnsCanceledAndClearsState)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_block_execution(true);
  MotionCore core(backend, std::chrono::milliseconds(500), valid_geometry());
  grant_normal(core);
  const auto uuid = make_uuid(50);
  auto result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  auto execution = std::async(
    std::launch::async, [&]() {result = core.accept_goal(uuid, pick_goal());});

  ASSERT_TRUE(backend->wait_until_execution_started());
  ASSERT_TRUE(core.request_cancel(uuid));
  ASSERT_EQ(execution.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(result, MotionTaskResultCode::TASK_RESULT_CANCELED);
  EXPECT_TRUE(core.status().has_active_execution);

  backend->set_execution_active(false);
  backend->set_inactivity_confirmed(true);
  core.refresh_stop_state();
  EXPECT_FALSE(core.status().has_active_execution);
  EXPECT_EQ(
    core.accept_goal(make_uuid(51), ExecuteTask::Goal{}),
    MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  ExecuteTask::Goal home;
  home.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  EXPECT_EQ(
    core.accept_goal(make_uuid(52), home), MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

TEST(MotionCoreTest, SafetyPreemptionTakesPrecedenceOverCallerCancellation)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_block_execution(true);
  backend->set_defer_terminal_return(true);
  MotionCore core(backend, std::chrono::milliseconds(500), valid_geometry());
  grant_normal(core);
  const auto uuid = make_uuid(53);
  auto result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  auto execution = std::async(
    std::launch::async, [&]() {result = core.accept_goal(uuid, pick_goal());});

  ASSERT_TRUE(backend->wait_until_execution_started());
  ASSERT_TRUE(core.request_cancel(uuid));
  ASSERT_TRUE(backend->wait_until_interruption_ready());
  ASSERT_TRUE(core.request_stop(make_uuid(54)));
  backend->release_terminal_return();
  ASSERT_EQ(execution.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(result, MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
}

TEST(MotionCoreTest, SafetyPreemptionRemainsTerminalWhenCallerCancelsAfterward)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_block_execution(true);
  backend->set_defer_terminal_return(true);
  MotionCore core(backend, std::chrono::milliseconds(500), valid_geometry());
  grant_normal(core);
  const auto uuid = make_uuid(54);
  auto result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  auto execution = std::async(
    std::launch::async, [&]() {result = core.accept_goal(uuid, pick_goal());});

  ASSERT_TRUE(backend->wait_until_execution_started());
  ASSERT_TRUE(core.request_stop(make_uuid(55)));
  ASSERT_TRUE(backend->wait_until_interruption_ready());
  ASSERT_TRUE(core.request_cancel(uuid));
  backend->release_terminal_return();
  ASSERT_EQ(execution.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(result, MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
}

TEST(MotionCoreTest, NaturalBackendCompletionReturnsToIdle)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_EQ(core.accept_goal(make_uuid(10), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  backend->set_execution_active(false);
  backend->set_inactivity_confirmed(true);
  core.refresh_stop_state();
  const auto status = core.status();

  EXPECT_EQ(status.execution_state, MotionStatus::MOTION_STATE_IDLE);
  EXPECT_FALSE(status.has_active_execution);
  EXPECT_FALSE(status.execution_active);
}

TEST(MotionCoreTest, RejectsSecondGoalWhileExecutionIsActive)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_EQ(core.accept_goal(make_uuid(11), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  EXPECT_EQ(
    core.accept_goal(make_uuid(12), goal),
    MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_EQ(core.status().active_execution_id, make_uuid(11));
}

TEST(MotionCoreTest, AcceptedGoalReservationAllowsOnlyItsOwnExecution)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  const auto first_uuid = make_uuid(31);

  ASSERT_EQ(
    core.reserve_goal(first_uuid, goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  std::string diagnostic;
  EXPECT_EQ(
    core.accept_goal_with_diagnostic(first_uuid, goal, diagnostic),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_TRUE(diagnostic.empty());
  EXPECT_TRUE(core.status().has_active_execution);
}

TEST(MotionCoreTest, RejectsSecondGoalWhileFirstGoalIsPending)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;

  ASSERT_EQ(
    core.reserve_goal(make_uuid(32), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  EXPECT_EQ(
    core.reserve_goal(make_uuid(33), goal), MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
}

TEST(MotionCoreTest, TerminalCompletionClearsReservationForLaterGoal)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;

  ASSERT_EQ(
    core.reserve_goal(make_uuid(34), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(
    core.accept_goal(make_uuid(34), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
  backend->set_execution_active(false);
  backend->set_inactivity_confirmed(true);
  core.refresh_stop_state();

  EXPECT_EQ(
    core.reserve_goal(make_uuid(35), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);
}

TEST(MotionCoreTest, DiagnosesTransientDuplicateGoalRejection)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_EQ(core.accept_goal(make_uuid(21), goal), MotionTaskResultCode::TASK_RESULT_SUCCESS);

  std::string diagnostic;
  EXPECT_EQ(
    core.accept_goal_with_diagnostic(make_uuid(22), goal, diagnostic),
    MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_NE(diagnostic.find("duplicate/pending execution rejection"), std::string::npos);
  EXPECT_NE(diagnostic.find("has_active_execution=true"), std::string::npos);
}

TEST(MotionCoreTest, PickPreservesFailureResultWithoutLoggingPropagationAsFailure)
{
  ScopedLogCapture log_capture;
  auto backend = std::make_shared<FakeMotionBackend>();
  backend->set_task_result(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  MotionCore core(backend, std::chrono::milliseconds(500), valid_geometry());
  grant_normal(core);
  const auto goal = pick_goal();

  std::string diagnostic;
  EXPECT_EQ(
    core.accept_goal_with_diagnostic(make_uuid(23), goal, diagnostic),
    MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
  EXPECT_EQ(diagnostic, "backend unavailable");

  EXPECT_TRUE(std::all_of(
      captured_logs.begin(), captured_logs.end(), [](const CapturedLog & log) {
        return log.severity < RCUTILS_LOG_SEVERITY_WARN;
      }));
}

}  // namespace arm_cell_motion_moveit2
