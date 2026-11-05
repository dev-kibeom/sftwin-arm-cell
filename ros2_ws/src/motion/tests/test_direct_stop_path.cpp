#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <condition_variable>
#include <future>
#include <mutex>

#include <arm_cell_interfaces/msg/stop_mode.hpp>

#include "arm_cell_motion/fake_motion_backend.hpp"
#include "arm_cell_motion/motion_core.hpp"

namespace arm_cell_motion
{
namespace
{
void grant_normal(MotionCore & core)
{
  SafetyState state;
  state.valid = true;
  state.required_inputs_fresh = true;
  state.motion_envelope_valid = true;
  state.max_velocity_scale = 1.0F;
  state.max_acceleration_scale = 1.0F;
  state.safety_state = SafetyState::SAFETY_STATE_SAFE;
  state.motion_capability.value = arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL;
  core.update_safety_state(state, MotionCore::Clock::now());
}

std::string uuid_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  static constexpr char digits[] = "0123456789abcdef";
  std::string result;
  result.reserve(uuid.uuid.size() * 2);
  for (const auto byte : uuid.uuid) {
    result.push_back(digits[(byte >> 4U) & 0x0fU]);
    result.push_back(digits[byte & 0x0fU]);
  }
  return result;
}

class BlockingStatusBackend final : public MotionBackend
{
public:
  bool available() const override {return true;}
  bool submit(const ExecuteTask::Goal &) override
  {
    execution_active_ = true;
    inactivity_confirmed_ = false;
    return true;
  }
  void cancel() override {}
  void hold() override {}
  void stop() override {}
  bool execution_active() const override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // Goal reservation and completion each observe the backend before the
    // explicit refresh exercised by DirectStopDispatchIsNotBlockedBySlowStatusRefresh.
    if (status_queries_++ < 2) {
      return execution_active_;
    }
    entered_ = true;
    entered_condition_.notify_one();
    release_condition_.wait(lock, [this]() {return released_;});
    return execution_active_;
  }
  bool backend_inactivity_confirmed() const override {return inactivity_confirmed_;}

  void wait_until_status_query()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_condition_.wait(lock, [this]() {return entered_;});
  }
  void release_status_query()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    release_condition_.notify_one();
  }

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable entered_condition_;
  mutable std::condition_variable release_condition_;
  mutable bool entered_{false};
  mutable bool released_{false};
  mutable int status_queries_{0};
  bool execution_active_{false};
  bool inactivity_confirmed_{true};
};

class BlockingSubmitBackend final : public MotionBackend
{
public:
  bool available() const override {return true;}
  bool submit(const ExecuteTask::Goal &) override
  {
    stop_requested_.store(false);
    std::unique_lock<std::mutex> lock(mutex_);
    submit_entered_ = true;
    entered_condition_.notify_one();
    release_condition_.wait(lock, [this]() {return released_;});
    if (!submit_result_ || stop_requested_.load()) {
      return false;
    }
    execution_active_.store(true);
    inactivity_confirmed_.store(false);
    return true;
  }
  void cancel() override {}
  void hold() override {}
  void stop() override
  {
    ++stop_calls_;
    stop_requested_.store(true);
    execution_active_.store(false);
    inactivity_confirmed_.store(true);
  }
  bool execution_active() const override {return execution_active_.load();}
  bool backend_inactivity_confirmed() const override {return inactivity_confirmed_.load();}

  void wait_until_submit_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_condition_.wait(lock, [this]() {return submit_entered_;});
  }
  void release_submit()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    release_condition_.notify_one();
  }
  int stop_calls() const {return stop_calls_.load();}
  void set_submit_result(bool value) {submit_result_ = value;}

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable entered_condition_;
  mutable std::condition_variable release_condition_;
  bool submit_entered_{false};
  bool released_{false};
  std::atomic<bool> execution_active_{false};
  std::atomic<bool> inactivity_confirmed_{true};
  std::atomic<bool> stop_requested_{false};
  bool submit_result_{true};
  std::atomic<int> stop_calls_{0};
};
}  // namespace

TEST(DirectStopPathTest, DispatchesEachCanonicalStopModeToBoundedBackendPolicy)
{
  for (const auto stop_mode : {
      arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED,
      arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE,
      arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY})
  {
    auto backend = std::make_shared<FakeMotionBackend>();
    MotionCore core(backend);
    grant_normal(core);
    const auto execution_id = uuid_string(make_uuid(static_cast<uint8_t>(stop_mode + 1)));
    ASSERT_EQ(
      core.executeLegacyMovement(MotionTaskType::TASK_TYPE_GO_HOME, execution_id, "direct-stop-test-movement"),
      MotionTaskResultCode::TASK_RESULT_SUCCESS);

    ASSERT_TRUE(core.request_stop(make_uuid(static_cast<uint8_t>(stop_mode + 10)), stop_mode));
    EXPECT_EQ(core.status().execution_state, MotionStatus::MOTION_STATE_STOPPING);
    EXPECT_EQ(
      backend->cancel_calls(), stop_mode ==
      arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE ? 1 : 0);
    EXPECT_EQ(
      backend->hold_calls(), stop_mode ==
      arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED ? 1 :
      stop_mode == arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE ? 1 : 0);
    EXPECT_EQ(
      backend->stop_calls(), stop_mode ==
      arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY ? 1 : 0);
  }
}

TEST(DirectStopPathTest, AcceptedStopRequiresConfirmedBackendInactivity)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ASSERT_EQ(
    core.executeLegacyMovement(MotionTaskType::TASK_TYPE_GO_HOME, uuid_string(make_uuid(1)), "movement-1"),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);

  const auto request_id = make_uuid(2);
  ASSERT_TRUE(
    core.request_stop(
      request_id, arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE));
  auto status = core.status();
  EXPECT_EQ(status.execution_state, MotionStatus::MOTION_STATE_STOPPING);
  EXPECT_EQ(status.last_stop_request_id, request_id);
  EXPECT_TRUE(status.has_last_stop_request);

  backend->set_execution_active(false);
  backend->set_inactivity_confirmed(false);
  core.refresh_stop_state();
  EXPECT_EQ(core.status().execution_state, MotionStatus::MOTION_STATE_STOPPING);
  EXPECT_FALSE(core.status().execution_active);
  EXPECT_FALSE(core.status().backend_inactivity_confirmed);
  EXPECT_FALSE(core.status().has_active_execution);
  EXPECT_EQ(
    core.executeLegacyMovement(MotionTaskType::TASK_TYPE_GO_HOME, uuid_string(make_uuid(20)), "movement-20"),
    MotionTaskResultCode::TASK_RESULT_BUSY);
  EXPECT_TRUE(core.request_stop(
      make_uuid(21), arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY));
  EXPECT_EQ(backend->stop_calls(), 1);

  backend->set_inactivity_confirmed(true);
  core.refresh_stop_state();
  EXPECT_EQ(core.status().execution_state, MotionStatus::MOTION_STATE_STOPPED);
}

TEST(DirectStopPathTest, StoppingStatusProjectsBackendInactivityReason)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ASSERT_EQ(
    core.executeLegacyMovement(MotionTaskType::TASK_TYPE_GO_HOME, uuid_string(make_uuid(40)), "movement-40"),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);
  backend->set_inactivity_diagnostic("awaiting_stable_joint_feedback");

  ASSERT_TRUE(
    core.request_stop(
      make_uuid(41), arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY));

  EXPECT_EQ(
    core.status().diagnostic_detail, "awaiting_stable_joint_feedback");
}

TEST(DirectStopPathTest, DuplicateAndLowerSeverityStopsDoNotRedispatch)
{
  auto backend = std::make_shared<FakeMotionBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ASSERT_EQ(
    core.executeLegacyMovement(MotionTaskType::TASK_TYPE_GO_HOME, uuid_string(make_uuid(30)), "movement-30"),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);

  ASSERT_TRUE(
    core.request_stop(
      make_uuid(31), arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED));
  ASSERT_TRUE(
    core.request_stop(
      make_uuid(32), arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED));
  ASSERT_TRUE(
    core.request_stop(
      make_uuid(33), arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE));
  ASSERT_TRUE(
    core.request_stop(
      make_uuid(34), arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY));
  ASSERT_TRUE(
    core.request_stop(
      make_uuid(35), arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE));

  EXPECT_EQ(backend->hold_calls(), 2);
  EXPECT_EQ(backend->cancel_calls(), 1);
  EXPECT_EQ(backend->stop_calls(), 1);
  EXPECT_EQ(core.status().last_stop_request_id, make_uuid(35));
}

TEST(DirectStopPathTest, DirectStopDispatchIsNotBlockedBySlowStatusRefresh)
{
  auto backend = std::make_shared<BlockingStatusBackend>();
  MotionCore core(backend);
  grant_normal(core);
  ASSERT_EQ(
    core.executeLegacyMovement(MotionTaskType::TASK_TYPE_GO_HOME, uuid_string(make_uuid(3)), "movement-3"),
    MotionTaskResultCode::TASK_RESULT_SUCCESS);

  auto refresh = std::async(std::launch::async, [&core]() {core.refresh_stop_state();});
  backend->wait_until_status_query();

  auto stop = std::async(
    std::launch::async, [&core]() {
      return core.request_stop(
        make_uuid(4), arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
    });
  EXPECT_EQ(stop.wait_for(std::chrono::milliseconds(100)), std::future_status::ready);
  EXPECT_TRUE(stop.get());

  backend->release_status_query();
  refresh.get();
}

TEST(DirectStopPathTest, DirectStopDispatchesWhileNormalSubmissionIsBlocked)
{
  auto backend = std::make_shared<BlockingSubmitBackend>();
  MotionCore core(backend);
  grant_normal(core);

  auto goal = std::async(
    std::launch::async, [&core]() {
      return core.executeLegacyMovement(MotionTaskType::TASK_TYPE_GO_HOME, uuid_string(make_uuid(5)), "movement-5");
    });
  backend->wait_until_submit_entered();

  auto stop = std::async(
    std::launch::async, [&core]() {
      return core.request_stop(
        make_uuid(6), arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
    });
  EXPECT_EQ(stop.wait_for(std::chrono::milliseconds(100)), std::future_status::ready);
  EXPECT_TRUE(stop.get());
  EXPECT_EQ(backend->stop_calls(), 1);

  backend->release_submit();
  EXPECT_EQ(goal.get(), MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(backend->stop_calls(), 1);
  core.refresh_stop_state();
  EXPECT_EQ(core.status().execution_state, MotionStatus::MOTION_STATE_STOPPED);
  EXPECT_FALSE(core.status().execution_active);
}

TEST(DirectStopPathTest, FailedSubmissionAfterStopDoesNotSynthesizeStopped)
{
  auto backend = std::make_shared<BlockingSubmitBackend>();
  backend->set_submit_result(false);
  MotionCore core(backend);
  grant_normal(core);

  auto goal = std::async(
    std::launch::async, [&core]() {
      return core.executeLegacyMovement(MotionTaskType::TASK_TYPE_GO_HOME, uuid_string(make_uuid(7)), "movement-7");
    });
  backend->wait_until_submit_entered();
  ASSERT_TRUE(
    core.request_stop(
      make_uuid(8), arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY));

  backend->release_submit();
  EXPECT_EQ(goal.get(), MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(core.status().execution_state, MotionStatus::MOTION_STATE_STOPPING);
  EXPECT_EQ(
    core.executeLegacyMovement(MotionTaskType::TASK_TYPE_GO_HOME, uuid_string(make_uuid(9)), "movement-9"),
    MotionTaskResultCode::TASK_RESULT_BUSY);
}

}  // namespace arm_cell_motion
