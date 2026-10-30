#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "arm_cell_motion/motion_port.hpp"
#include "arm_cell_task_execution/execute_disturb_action_server.hpp"

namespace
{
using Action = arm_cell_interfaces::action::ExecuteDisturb;
using Outcome = arm_cell_interfaces::msg::DisturbanceOutcome;
using Failure = arm_cell_interfaces::msg::DisturbanceFailureReason;

class DisturbActionMotionPort final : public arm_cell_motion::MotionPort
{
public:
  ~DisturbActionMotionPort() override
  {
    if (on_destroy_) {on_destroy_();}
  }

  void on_destroy(std::function<void()> callback) {on_destroy_ = std::move(callback);}

  arm_cell_motion::MotionPortResult planAndExecuteMotion(
    const arm_cell_motion::MotionPortRequest &) override
  {arm_cell_motion::MotionPortResult result; return result;}

  arm_cell_motion::MotionDisturbResult executeDisturb(
    const arm_cell_motion::MotionDisturbRequest & request) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    request_ = request;
    ++calls_;
    started_ = true;
    active_ = true;
    condition_.notify_all();
    if (block_) {condition_.wait(lock, [this]() {return released_;});}
    auto result = result_;
    active_ = false;
    condition_.notify_all();
    result.execution_id = request.execution_id;
    result.acquisition_id = request.acquisition_id;
    return result;
  }

  arm_cell_motion::MotionCancelResult cancelMotion(
    const unique_identifier_msgs::msg::UUID & execution_id,
    const std::string & movement_id) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++cancel_calls_;
    cancel_id_ = execution_id;
    cancel_movement_id_ = movement_id;
    const bool operation_active_on_dispatch = active_;
    cancel_started_ = true;
    condition_.notify_all();
    if (block_cancel_) {condition_.wait(lock, [this]() {return cancel_released_;});}
    if (throw_cancel_) {throw std::runtime_error("injected cancel failure");}
    if (!operation_active_on_dispatch) {
      return {false, false, arm_cell_interfaces::msg::MotionStatus()};
    }
    return cancel_result_;
  }

  arm_cell_motion::MotionObservation observeMotionState() const override
  {arm_cell_motion::MotionObservation observation; return observation;}

  void set_result(arm_cell_motion::MotionDisturbResult result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result_ = std::move(result);
    block_ = false;
    released_ = true;
    started_ = false;
    active_ = false;
    condition_.notify_all();
  }

  void select_result(arm_cell_motion::MotionDisturbResult result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result_ = std::move(result);
  }

  void set_blocked()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block_ = true;
    released_ = false;
    started_ = false;
    active_ = false;
  }

  void set_cancel_result(arm_cell_motion::MotionCancelResult result)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_result_ = std::move(result);
  }

  void block_cancel()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block_cancel_ = true;
    cancel_released_ = false;
    cancel_started_ = false;
  }

  bool wait_cancel_started(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return cancel_started_;});
  }

  void release_cancel()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_released_ = true;
    block_cancel_ = false;
    condition_.notify_all();
  }

  void set_throw_cancel(bool value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    throw_cancel_ = value;
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

  bool wait_started(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return started_;});
  }

  size_t cancel_calls() const {std::lock_guard<std::mutex> lock(mutex_); return cancel_calls_;}

  arm_cell_motion::MotionDisturbRequest request() const
  {std::lock_guard<std::mutex> lock(mutex_); return request_;}
  size_t calls() const {std::lock_guard<std::mutex> lock(mutex_); return calls_;}
  bool active() const {std::lock_guard<std::mutex> lock(mutex_); return active_;}
  std::string cancel_movement_id() const
  {std::lock_guard<std::mutex> lock(mutex_); return cancel_movement_id_;}
  unique_identifier_msgs::msg::UUID cancel_id() const
  {std::lock_guard<std::mutex> lock(mutex_); return cancel_id_;}

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  arm_cell_motion::MotionDisturbResult result_;
  arm_cell_motion::MotionDisturbRequest request_;
  unique_identifier_msgs::msg::UUID cancel_id_;
  std::string cancel_movement_id_;
  arm_cell_motion::MotionCancelResult cancel_result_{
    true, false, arm_cell_interfaces::msg::MotionStatus()};
  size_t calls_{0};
  size_t cancel_calls_{0};
  bool block_{false};
  bool block_cancel_{false};
  bool cancel_released_{true};
  bool cancel_started_{false};
  bool throw_cancel_{false};
  bool released_{true};
  bool started_{false};
  bool active_{false};
  std::function<void()> on_destroy_;
};

class MotionStartBarrier
{
public:
  using Phase = arm_cell_task_execution::ExecuteDisturbActionServer::MotionStartPhase;

  void block_next(Phase phase)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = true;
    phase_ = phase;
    reached_ = false;
    released_ = false;
  }

  void at(Phase phase)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!enabled_ || phase != phase_) {return;}
    enabled_ = false;
    reached_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return released_;});
  }

  bool wait_reached(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return reached_;});
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool enabled_{false};
  Phase phase_{Phase::BEFORE_START_CLAIM};
  bool reached_{false};
  bool released_{true};
};

Action::Goal request_goal()
{
  Action::Goal goal;
  goal.acquisition_id = "acquisition-origin";
  goal.scene_generation = 73;
  arm_cell_interfaces::msg::DisturbanceCandidate first;
  first.candidate_id = "candidate-b";
  first.center.header.frame_id = "base_link";
  first.center.pose.position.x = 0.17;
  first.center.pose.orientation.w = 1.0;
  first.category = first.CATEGORY_OTHER_UNRESOLVED;
  first.observed_height_m = 0.42;
  arm_cell_interfaces::msg::DisturbanceCandidate second;
  second.candidate_id = "candidate-a";
  second.center.header.frame_id = "base_link";
  second.center.pose.position.x = -0.23;
  second.center.pose.orientation.w = 1.0;
  second.category = second.CATEGORY_INCLINED_SURFACE;
  second.observed_height_m = 0.18;
  goal.disturbance_candidates = {first, second};
  return goal;
}

}  // namespace

TEST(ExecuteDisturbActionServer, RelaysIdentityCandidatesOutcomesAndTerminalStatus)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp::Node>("execute_disturb_action_contract_test");
  auto motion = std::make_shared<DisturbActionMotionPort>();
  auto execution_mutex = std::make_shared<std::mutex>();
  MotionStartBarrier motion_start_barrier;
  arm_cell_task_execution::ExecuteDisturbActionServer server(
    node.get(), motion, execution_mutex, [&motion_start_barrier](
      arm_cell_task_execution::ExecuteDisturbActionServer::MotionStartPhase phase) {
      motion_start_barrier.at(phase);
    });
  auto client = rclcpp_action::create_client<Action>(node, "/task_execution/execute_disturb");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));

  rclcpp_action::Client<Action>::SendGoalOptions options;
  auto send_and_wait = [&](const Action::Goal & goal) {
      auto goal_future = client->async_send_goal(goal, options);
      EXPECT_EQ(goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
      auto handle = goal_future.get();
      EXPECT_NE(handle, nullptr);
      auto future = client->async_get_result(handle);
      EXPECT_EQ(future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
      return future.get();
    };

  arm_cell_motion::MotionDisturbResult completed;
  completed.outcome.value = Outcome::OUTCOME_COMPLETED;
  motion->set_result(completed);
  motion->set_blocked();
  const auto goal = request_goal();
  const auto completed_goal_future = client->async_send_goal(goal, options);
  ASSERT_EQ(completed_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto completed_goal = completed_goal_future.get();
  ASSERT_NE(completed_goal, nullptr);
  ASSERT_TRUE(motion->wait_started(std::chrono::seconds(2)));
  motion->release();
  const auto completed_result_future = client->async_get_result(completed_goal);
  ASSERT_EQ(completed_result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto succeeded = completed_result_future.get();
  ASSERT_NE(succeeded.result, nullptr);
  EXPECT_EQ(succeeded.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(succeeded.result->outcome.value, Outcome::OUTCOME_COMPLETED);
  ASSERT_EQ(motion->calls(), 1U);
  const auto forwarded = motion->request();
  EXPECT_EQ(forwarded.execution_id.uuid, succeeded.goal_id);
  EXPECT_EQ(forwarded.acquisition_id, goal.acquisition_id);
  EXPECT_EQ(forwarded.scene_generation, goal.scene_generation);
  ASSERT_EQ(forwarded.disturbance_candidates.size(), goal.disturbance_candidates.size());
  EXPECT_EQ(forwarded.disturbance_candidates[0].candidate_id,
    goal.disturbance_candidates[0].candidate_id);
  EXPECT_EQ(forwarded.disturbance_candidates[0].category, goal.disturbance_candidates[0].category);
  EXPECT_EQ(forwarded.disturbance_candidates[0].center.header.frame_id,
    goal.disturbance_candidates[0].center.header.frame_id);
  EXPECT_EQ(forwarded.disturbance_candidates[0].center.pose.position.x,
    goal.disturbance_candidates[0].center.pose.position.x);
  EXPECT_EQ(forwarded.disturbance_candidates[0].center.pose.orientation.w,
    goal.disturbance_candidates[0].center.pose.orientation.w);
  EXPECT_EQ(forwarded.disturbance_candidates[0].observed_height_m,
    goal.disturbance_candidates[0].observed_height_m);
  EXPECT_EQ(forwarded.disturbance_candidates[1].candidate_id,
    goal.disturbance_candidates[1].candidate_id);
  EXPECT_EQ(forwarded.disturbance_candidates[1].category, goal.disturbance_candidates[1].category);
  EXPECT_EQ(forwarded.disturbance_candidates[1].center.header.frame_id,
    goal.disturbance_candidates[1].center.header.frame_id);
  EXPECT_EQ(forwarded.disturbance_candidates[1].center.pose.position.x,
    goal.disturbance_candidates[1].center.pose.position.x);
  EXPECT_EQ(forwarded.disturbance_candidates[1].center.pose.orientation.w,
    goal.disturbance_candidates[1].center.pose.orientation.w);
  EXPECT_EQ(forwarded.disturbance_candidates[1].observed_height_m,
    goal.disturbance_candidates[1].observed_height_m);
  arm_cell_motion::MotionDisturbResult failed;
  failed.outcome.value = Outcome::OUTCOME_FAILED;
  failed.failure_reason.value = Failure::FAILURE_REASON_NOT_IMPLEMENTED;
  failed.diagnostic_detail = "Motion disturbance execution is not implemented";
  motion->set_result(failed);
  const auto failed_result = send_and_wait(goal);
  ASSERT_NE(failed_result.result, nullptr);
  EXPECT_EQ(failed_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(failed_result.result->outcome.value, Outcome::OUTCOME_FAILED);
  EXPECT_EQ(failed_result.result->failure_reason.value, Failure::FAILURE_REASON_NOT_IMPLEMENTED);

  arm_cell_motion::MotionDisturbResult safety;
  safety.outcome.value = Outcome::OUTCOME_SAFETY_PREEMPTED;
  safety.safety_cause.value = arm_cell_interfaces::msg::SafetyCauseSet::E_STOP;
  safety.backend_inactivity_confirmed = true;
  motion->set_result(safety);
  const auto safety_result = send_and_wait(goal);
  ASSERT_NE(safety_result.result, nullptr);
  EXPECT_EQ(safety_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(safety_result.result->outcome.value, Outcome::OUTCOME_SAFETY_PREEMPTED);
  EXPECT_EQ(safety_result.result->safety_cause.value,
    arm_cell_interfaces::msg::SafetyCauseSet::E_STOP);
  EXPECT_TRUE(safety_result.result->backend_inactivity_confirmed);

  auto cancel_then_motion_terminal = [&](arm_cell_motion::MotionDisturbResult motion_terminal) {
      motion->set_blocked();
      const auto pending_goal = client->async_send_goal(goal, options);
      EXPECT_EQ(pending_goal.wait_for(std::chrono::seconds(2)), std::future_status::ready);
      auto pending_handle = pending_goal.get();
      EXPECT_NE(pending_handle, nullptr);
      EXPECT_TRUE(motion->wait_started(std::chrono::seconds(2)));
      const auto cancel_request = client->async_cancel_goal(pending_handle);
      EXPECT_EQ(cancel_request.wait_for(std::chrono::seconds(2)), std::future_status::ready);
      if (cancel_request.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        EXPECT_EQ(cancel_request.get()->return_code,
          action_msgs::srv::CancelGoal::Response::ERROR_NONE);
      }
      motion->select_result(std::move(motion_terminal));
      motion->release();
      const auto terminal_future = client->async_get_result(pending_handle);
      EXPECT_EQ(terminal_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
      return terminal_future.get();
    };

  auto motion_terminal_before_cancel = [&](arm_cell_motion::MotionDisturbResult motion_terminal) {
      motion->set_blocked();
      motion->set_cancel_result(
        {false, false, arm_cell_interfaces::msg::MotionStatus()});
      motion->select_result(std::move(motion_terminal));
      const auto pending_goal = client->async_send_goal(goal, options);
      EXPECT_EQ(pending_goal.wait_for(std::chrono::seconds(2)), std::future_status::ready);
      auto pending_handle = pending_goal.get();
      EXPECT_NE(pending_handle, nullptr);
      EXPECT_TRUE(motion->wait_started(std::chrono::seconds(2)));
      const auto cancel_request = client->async_cancel_goal(pending_handle);
      EXPECT_EQ(cancel_request.wait_for(std::chrono::seconds(2)), std::future_status::ready);
      if (cancel_request.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        EXPECT_NE(cancel_request.get()->return_code,
          action_msgs::srv::CancelGoal::Response::ERROR_NONE);
      }
      motion->release();
      const auto terminal_future = client->async_get_result(pending_handle);
      EXPECT_EQ(terminal_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
      motion->set_cancel_result(
        {true, false, arm_cell_interfaces::msg::MotionStatus()});
      return terminal_future.get();
    };

  arm_cell_motion::MotionDisturbResult completed_before_cancel;
  completed_before_cancel.outcome.value = Outcome::OUTCOME_COMPLETED;
  const auto completed_before_cancel_result =
    motion_terminal_before_cancel(completed_before_cancel);
  ASSERT_NE(completed_before_cancel_result.result, nullptr);
  EXPECT_EQ(completed_before_cancel_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(completed_before_cancel_result.result->outcome.value, Outcome::OUTCOME_COMPLETED);

  arm_cell_motion::MotionDisturbResult failure_before_cancel;
  failure_before_cancel.outcome.value = Outcome::OUTCOME_FAILED;
  failure_before_cancel.failure_reason.value = Failure::FAILURE_REASON_EXECUTION_FAILURE;
  failure_before_cancel.diagnostic_detail = "Motion execution failed before Cancel";
  const auto failure_before_cancel_result = motion_terminal_before_cancel(failure_before_cancel);
  ASSERT_NE(failure_before_cancel_result.result, nullptr);
  EXPECT_EQ(failure_before_cancel_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(failure_before_cancel_result.result->outcome.value, Outcome::OUTCOME_FAILED);
  EXPECT_EQ(failure_before_cancel_result.result->failure_reason.value,
    Failure::FAILURE_REASON_EXECUTION_FAILURE);
  EXPECT_EQ(failure_before_cancel_result.result->diagnostic_detail,
    "Motion execution failed before Cancel");

  arm_cell_motion::MotionDisturbResult completed_after_cancel;
  completed_after_cancel.outcome.value = Outcome::OUTCOME_COMPLETED;
  const auto completed_after_cancel_result = cancel_then_motion_terminal(completed_after_cancel);
  ASSERT_NE(completed_after_cancel_result.result, nullptr);
  EXPECT_EQ(completed_after_cancel_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(completed_after_cancel_result.result->outcome.value, Outcome::OUTCOME_COMPLETED);

  arm_cell_motion::MotionDisturbResult failure_after_cancel;
  failure_after_cancel.outcome.value = Outcome::OUTCOME_FAILED;
  failure_after_cancel.failure_reason.value = Failure::FAILURE_REASON_EXECUTION_FAILURE;
  failure_after_cancel.diagnostic_detail = "Motion execution failed before cancel took effect";
  const auto failure_after_cancel_result = cancel_then_motion_terminal(failure_after_cancel);
  ASSERT_NE(failure_after_cancel_result.result, nullptr);
  EXPECT_EQ(failure_after_cancel_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(failure_after_cancel_result.result->outcome.value, Outcome::OUTCOME_FAILED);
  EXPECT_EQ(failure_after_cancel_result.result->failure_reason.value,
    Failure::FAILURE_REASON_EXECUTION_FAILURE);
  EXPECT_EQ(failure_after_cancel_result.result->diagnostic_detail,
    "Motion execution failed before cancel took effect");

  motion->set_blocked();
  arm_cell_motion::MotionDisturbResult canceled_motion;
  canceled_motion.outcome.value = Outcome::OUTCOME_CANCELED;
  canceled_motion.backend_inactivity_confirmed = true;
  motion->select_result(canceled_motion);
  const auto cancel_goal_future = client->async_send_goal(goal, options);
  ASSERT_EQ(cancel_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto cancel_goal = cancel_goal_future.get();
  ASSERT_NE(cancel_goal, nullptr);
  ASSERT_TRUE(motion->wait_started(std::chrono::seconds(2)));
  const auto cancel_calls_before_motion_stop = motion->cancel_calls();
  const auto cancel_future = client->async_cancel_goal(cancel_goal);
  ASSERT_EQ(cancel_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(cancel_future.get()->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  const auto duplicate_cancel = client->async_cancel_goal(cancel_goal);
  ASSERT_EQ(duplicate_cancel.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_NE(duplicate_cancel.get()->return_code,
    action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_EQ(motion->cancel_calls(), cancel_calls_before_motion_stop + 1);
  const auto cancel_result_future = client->async_get_result(cancel_goal);
  EXPECT_EQ(cancel_result_future.wait_for(std::chrono::milliseconds(100)),
    std::future_status::timeout);
  motion->release();
  ASSERT_EQ(cancel_result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto canceled = cancel_result_future.get();
  ASSERT_NE(canceled.result, nullptr);
  EXPECT_EQ(canceled.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(canceled.result->outcome.value, Outcome::OUTCOME_CANCELED);
  EXPECT_TRUE(canceled.result->backend_inactivity_confirmed);
  EXPECT_EQ(motion->cancel_id().uuid, canceled.goal_id);
  EXPECT_EQ(motion->cancel_movement_id(), arm_cell_motion::kMotionDisturbMovementId);

  const auto cancel_calls_before_safety_race = motion->cancel_calls();
  motion->set_blocked();
  motion->block_cancel();
  const auto safety_race_goal_future = client->async_send_goal(goal, options);
  ASSERT_EQ(safety_race_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto safety_race_goal = safety_race_goal_future.get();
  ASSERT_NE(safety_race_goal, nullptr);
  ASSERT_TRUE(motion->wait_started(std::chrono::seconds(2)));
  const auto safety_race_cancel = client->async_cancel_goal(safety_race_goal);
  ASSERT_TRUE(motion->wait_cancel_started(std::chrono::seconds(2)));
  EXPECT_EQ(motion->cancel_calls(), cancel_calls_before_safety_race + 1);

  arm_cell_motion::MotionDisturbResult safety_after_cancel;
  safety_after_cancel.outcome.value = Outcome::OUTCOME_SAFETY_PREEMPTED;
  safety_after_cancel.safety_cause.value =
    arm_cell_interfaces::msg::SafetyCauseSet::E_STOP;
  safety_after_cancel.backend_inactivity_confirmed = true;
  safety_after_cancel.diagnostic_detail = "Safety direct stop confirmed";
  motion->select_result(safety_after_cancel);
  motion->release();
  const auto safety_race_result_future = client->async_get_result(safety_race_goal);
  EXPECT_EQ(safety_race_result_future.wait_for(std::chrono::milliseconds(100)),
    std::future_status::timeout);
  motion->release_cancel();
  ASSERT_EQ(safety_race_cancel.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(safety_race_cancel.get()->return_code,
    action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  ASSERT_EQ(safety_race_result_future.wait_for(std::chrono::seconds(3)),
    std::future_status::ready);
  const auto safety_race_result = safety_race_result_future.get();
  ASSERT_NE(safety_race_result.result, nullptr);
  EXPECT_EQ(safety_race_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(safety_race_result.result->outcome.value, Outcome::OUTCOME_SAFETY_PREEMPTED);
  EXPECT_EQ(safety_race_result.result->safety_cause.value,
    arm_cell_interfaces::msg::SafetyCauseSet::E_STOP);
  EXPECT_TRUE(safety_race_result.result->backend_inactivity_confirmed);
  EXPECT_EQ(safety_race_result.result->diagnostic_detail, "Safety direct stop confirmed");

  motion->set_blocked();
  motion->set_throw_cancel(true);
  const auto throwing_cancel_goal_future = client->async_send_goal(goal, options);
  ASSERT_EQ(throwing_cancel_goal_future.wait_for(std::chrono::seconds(2)),
    std::future_status::ready);
  const auto throwing_cancel_goal = throwing_cancel_goal_future.get();
  ASSERT_NE(throwing_cancel_goal, nullptr);
  ASSERT_TRUE(motion->wait_started(std::chrono::seconds(2)));
  const auto throwing_cancel = client->async_cancel_goal(throwing_cancel_goal);
  ASSERT_EQ(throwing_cancel.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_NE(throwing_cancel.get()->return_code,
    action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  motion->set_throw_cancel(false);
  arm_cell_motion::MotionDisturbResult after_cancel_error;
  after_cancel_error.outcome.value = Outcome::OUTCOME_FAILED;
  after_cancel_error.failure_reason.value = Failure::FAILURE_REASON_EXECUTION_FAILURE;
  after_cancel_error.diagnostic_detail = "Motion failure remains authoritative";
  motion->set_result(after_cancel_error);
  const auto after_cancel_error_future = client->async_get_result(throwing_cancel_goal);
  ASSERT_EQ(after_cancel_error_future.wait_for(std::chrono::seconds(3)),
    std::future_status::ready);
  const auto after_cancel_error_result = after_cancel_error_future.get();
  ASSERT_NE(after_cancel_error_result.result, nullptr);
  EXPECT_EQ(after_cancel_error_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(after_cancel_error_result.result->outcome.value, Outcome::OUTCOME_FAILED);
  EXPECT_EQ(after_cancel_error_result.result->diagnostic_detail,
    "Motion failure remains authoritative");

  motion_start_barrier.block_next(
    arm_cell_task_execution::ExecuteDisturbActionServer::MotionStartPhase::BEFORE_START_CLAIM);
  const auto calls_before_prestart_cancel = motion->calls();
  const auto cancel_calls_before_prestart_cancel = motion->cancel_calls();
  const auto prestart_goal_future = client->async_send_goal(goal, options);
  ASSERT_EQ(prestart_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto prestart_goal = prestart_goal_future.get();
  ASSERT_NE(prestart_goal, nullptr);
  ASSERT_TRUE(motion_start_barrier.wait_reached(std::chrono::seconds(2)));
  EXPECT_EQ(motion->calls(), calls_before_prestart_cancel);
  const auto prestart_cancel = client->async_cancel_goal(prestart_goal);
  ASSERT_EQ(prestart_cancel.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(prestart_cancel.get()->return_code,
    action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_EQ(motion->cancel_calls(), cancel_calls_before_prestart_cancel);
  const auto prestart_result_future = client->async_get_result(prestart_goal);
  motion_start_barrier.release();
  ASSERT_EQ(prestart_result_future.wait_for(std::chrono::seconds(3)),
    std::future_status::ready);
  const auto prestart_result = prestart_result_future.get();
  ASSERT_NE(prestart_result.result, nullptr);
  EXPECT_EQ(prestart_result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(prestart_result.result->outcome.value, Outcome::OUTCOME_CANCELED);
  EXPECT_FALSE(prestart_result.result->backend_inactivity_confirmed);
  EXPECT_EQ(motion->calls(), calls_before_prestart_cancel);
  EXPECT_EQ(motion->cancel_calls(), cancel_calls_before_prestart_cancel);

  // After the start claim, MotionPort owns cancellation. If the operation has
  // not activated yet, its rejection must reject this Action Cancel and leave
  // the already accepted execution free to proceed.
  arm_cell_motion::MotionDisturbResult completion_after_rejected_cancel;
  completion_after_rejected_cancel.outcome.value = Outcome::OUTCOME_COMPLETED;
  motion->select_result(completion_after_rejected_cancel);
  motion->set_blocked();
  motion_start_barrier.block_next(
    arm_cell_task_execution::ExecuteDisturbActionServer::MotionStartPhase::AFTER_START_CLAIM);
  const auto calls_before_activation_race = motion->calls();
  const auto cancel_calls_before_activation_race = motion->cancel_calls();
  const auto activation_race_goal_future = client->async_send_goal(goal, options);
  ASSERT_EQ(activation_race_goal_future.wait_for(std::chrono::seconds(2)),
    std::future_status::ready);
  const auto activation_race_goal = activation_race_goal_future.get();
  ASSERT_NE(activation_race_goal, nullptr);
  ASSERT_TRUE(motion_start_barrier.wait_reached(std::chrono::seconds(2)));
  EXPECT_EQ(motion->calls(), calls_before_activation_race);
  const auto rejected_activation_cancel = client->async_cancel_goal(activation_race_goal);
  ASSERT_EQ(rejected_activation_cancel.wait_for(std::chrono::seconds(2)),
    std::future_status::ready);
  EXPECT_NE(rejected_activation_cancel.get()->return_code,
    action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_EQ(motion->cancel_calls(), cancel_calls_before_activation_race + 1);
  motion_start_barrier.release();
  ASSERT_TRUE(motion->wait_started(std::chrono::seconds(2)));
  EXPECT_TRUE(motion->active());
  motion->release();
  const auto activation_race_result_future = client->async_get_result(activation_race_goal);
  ASSERT_EQ(activation_race_result_future.wait_for(std::chrono::seconds(3)),
    std::future_status::ready);
  const auto activation_race_result = activation_race_result_future.get();
  ASSERT_NE(activation_race_result.result, nullptr);
  EXPECT_EQ(activation_race_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_EQ(activation_race_result.result->outcome.value, Outcome::OUTCOME_COMPLETED);
  EXPECT_FALSE(activation_race_result.result->backend_inactivity_confirmed);

  // Rejection is not latched: after activation a later Cancel reaches the
  // same UUID, and only Motion's actual canceled terminal yields CANCELED.
  arm_cell_motion::MotionDisturbResult canceled_after_retry;
  canceled_after_retry.outcome.value = Outcome::OUTCOME_CANCELED;
  canceled_after_retry.backend_inactivity_confirmed = true;
  motion->select_result(canceled_after_retry);
  motion->set_blocked();
  motion_start_barrier.block_next(
    arm_cell_task_execution::ExecuteDisturbActionServer::MotionStartPhase::AFTER_START_CLAIM);
  const auto retry_goal_future = client->async_send_goal(goal, options);
  ASSERT_EQ(retry_goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto retry_goal = retry_goal_future.get();
  ASSERT_NE(retry_goal, nullptr);
  ASSERT_TRUE(motion_start_barrier.wait_reached(std::chrono::seconds(2)));
  const auto first_retry_cancel = client->async_cancel_goal(retry_goal);
  ASSERT_EQ(first_retry_cancel.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_NE(first_retry_cancel.get()->return_code,
    action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  motion_start_barrier.release();
  ASSERT_TRUE(motion->wait_started(std::chrono::seconds(2)));
  const auto cancel_calls_before_retry = motion->cancel_calls();
  const auto accepted_retry_cancel = client->async_cancel_goal(retry_goal);
  ASSERT_EQ(accepted_retry_cancel.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(accepted_retry_cancel.get()->return_code,
    action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  EXPECT_EQ(motion->cancel_calls(), cancel_calls_before_retry + 1);
  motion->release();
  const auto retry_result_future = client->async_get_result(retry_goal);
  ASSERT_EQ(retry_result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto retry_result = retry_result_future.get();
  ASSERT_NE(retry_result.result, nullptr);
  EXPECT_EQ(retry_result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(retry_result.result->outcome.value, Outcome::OUTCOME_CANCELED);
  EXPECT_TRUE(retry_result.result->backend_inactivity_confirmed);

  const auto motion_calls_before_busy_goal = motion->calls();
  rclcpp_action::ClientGoalHandle<Action>::WrappedResult busy_result;
  {
    std::lock_guard<std::mutex> active_task(*execution_mutex);
    busy_result = send_and_wait(goal);
  }
  ASSERT_NE(busy_result.result, nullptr);
  EXPECT_EQ(busy_result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(busy_result.result->failure_reason.value,
    Failure::FAILURE_REASON_EXECUTION_FAILURE);
  EXPECT_EQ(busy_result.result->diagnostic_detail,
    "Task Execution already owns an active task");
  EXPECT_EQ(motion->calls(), motion_calls_before_busy_goal);

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
}

TEST(ExecuteDisturbActionServer, ShutdownReleasesWorkerWaitingForAcceptedCancel)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  rclcpp::NodeOptions node_options;
  node_options.context(context);
  auto node = std::make_shared<rclcpp::Node>("execute_disturb_shutdown_test", node_options);
  auto motion = std::make_shared<DisturbActionMotionPort>();
  std::promise<void> motion_destroyed;
  auto motion_destroyed_future = motion_destroyed.get_future();
  motion->on_destroy([&motion_destroyed]() {motion_destroyed.set_value();});
  MotionStartBarrier start_barrier;
  auto execution_mutex = std::make_shared<std::mutex>();
  auto server = std::make_unique<arm_cell_task_execution::ExecuteDisturbActionServer>(
    node.get(), motion, execution_mutex, [&start_barrier](
      arm_cell_task_execution::ExecuteDisturbActionServer::MotionStartPhase phase) {
      start_barrier.at(phase);
    });
  auto client = rclcpp_action::create_client<Action>(node, "/task_execution/execute_disturb");
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));

  start_barrier.block_next(
    arm_cell_task_execution::ExecuteDisturbActionServer::MotionStartPhase::BEFORE_START_CLAIM);
  const auto goal_future = client->async_send_goal(request_goal());
  ASSERT_EQ(goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(start_barrier.wait_reached(std::chrono::seconds(2)));
  const auto cancel_future = client->async_cancel_goal(goal_handle);
  ASSERT_EQ(cancel_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_EQ(cancel_future.get()->return_code,
    action_msgs::srv::CancelGoal::Response::ERROR_NONE);

  context->shutdown("test shutdown while accepted cancellation awaits worker");
  executor.cancel();
  spin_thread.join();
  executor.remove_node(node);
  start_barrier.release();
  server.reset();
  client.reset();
  goal_handle.reset();
  node.reset();
  motion.reset();
  EXPECT_EQ(motion_destroyed_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
}
