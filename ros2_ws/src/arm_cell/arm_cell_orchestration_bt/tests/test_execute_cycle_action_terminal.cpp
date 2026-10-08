#include <gtest/gtest.h>

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "arm_cell_orchestration_bt/execute_cycle_terminal.hpp"

namespace
{
using Action = arm_cell_interfaces::action::ExecuteCycle;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
using MissionExitReason = arm_cell_interfaces::msg::MissionExitReason;

class TerminalActionFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    server_node_ = std::make_shared<rclcpp::Node>("execute_cycle_terminal_server");
    client_node_ = std::make_shared<rclcpp::Node>("execute_cycle_terminal_client");
    action_server_ = rclcpp_action::create_server<Action>(
      server_node_,
      "/test/execute_cycle_terminal",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          cancel_requested_ = true;
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        std::thread([this, handle]() {execute(handle);}).detach();
      });
    action_client_ = rclcpp_action::create_client<Action>(
      client_node_, "/test/execute_cycle_terminal");
    executor_.add_node(server_node_);
    executor_.add_node(client_node_);
    executor_thread_ = std::thread([this]() {executor_.spin();});
    ASSERT_TRUE(action_client_->wait_for_action_server(std::chrono::seconds(2)));
  }

  void TearDown() override
  {
    executor_.cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    action_client_.reset();
    action_server_.reset();
    client_node_.reset();
    server_node_.reset();
  }

  rclcpp_action::Client<Action>::WrappedResult run_immediate(uint8_t exit_reason)
  {
    Action::Goal goal;
    goal.target_id = std::to_string(exit_reason);
    auto goal_future = action_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      ADD_FAILURE() << "ExecuteCycle goal was not accepted in time";
      return {};
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      ADD_FAILURE() << "ExecuteCycle goal was rejected";
      return {};
    }
    auto result_future = action_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      ADD_FAILURE() << "ExecuteCycle result was not returned in time";
      return {};
    }
    return result_future.get();
  }

  rclcpp_action::Client<Action>::WrappedResult run_canceled()
  {
    Action::Goal goal;
    goal.target_id = "cancel";
    auto goal_future = action_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      ADD_FAILURE() << "ExecuteCycle cancel goal was not accepted in time";
      return {};
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      ADD_FAILURE() << "ExecuteCycle cancel goal was rejected";
      return {};
    }
    auto cancel_future = action_client_->async_cancel_goal(goal_handle);
    if (cancel_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      ADD_FAILURE() << "ExecuteCycle cancel request was not accepted in time";
      return {};
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cancel_transition_observed_ = true;
    }
    condition_.notify_all();
    auto result_future = action_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      ADD_FAILURE() << "ExecuteCycle canceled result was not returned in time";
      return {};
    }
    return result_future.get();
  }

  void execute(const std::shared_ptr<GoalHandle> & handle)
  {
    const auto target_id = handle->get_goal()->target_id;
    if (target_id == "cancel") {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(
        lock, [this]() {return cancel_requested_ && cancel_transition_observed_;});
    }
    auto result = std::make_shared<Action::Result>();
    if (target_id == "cancel") {
      result->exit_reason.value = MissionExitReason::MISSION_EXIT_CANCELED;
    } else if (target_id == std::to_string(MissionExitReason::MISSION_EXIT_DEPLETED)) {
      result->exit_reason.value = MissionExitReason::MISSION_EXIT_DEPLETED;
    } else if (target_id == std::to_string(MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED)) {
      result->exit_reason.value = MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED;
    } else if (target_id == std::to_string(MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR)) {
      result->exit_reason.value = MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR;
    } else {
      result->exit_reason.value = MissionExitReason::MISSION_EXIT_MOTION_ERROR;
    }
    arm_cell_orchestration_bt::complete_execute_cycle_goal(handle, result);
  }

  rclcpp::Node::SharedPtr server_node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp_action::Client<Action>::SharedPtr action_client_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread executor_thread_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool cancel_requested_{false};
  bool cancel_transition_observed_{false};
};

TEST_F(TerminalActionFixture, DepletedCompletesWithSucceededAndPreservesReason)
{
  const auto result = run_immediate(MissionExitReason::MISSION_EXIT_DEPLETED);

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->exit_reason.value, MissionExitReason::MISSION_EXIT_DEPLETED);
}

TEST_F(TerminalActionFixture, CallerCancellationCompletesWithCanceledAndPreservesReason)
{
  const auto result = run_canceled();

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->exit_reason.value, MissionExitReason::MISSION_EXIT_CANCELED);
}

TEST_F(TerminalActionFixture, MotionFailureCompletesWithAbortedAndPreservesReason)
{
  const auto result = run_immediate(MissionExitReason::MISSION_EXIT_MOTION_ERROR);

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->exit_reason.value, MissionExitReason::MISSION_EXIT_MOTION_ERROR);
}

TEST_F(TerminalActionFixture, ConfigurationErrorCompletesWithAbortedAndPreservesReason)
{
  const auto result = run_immediate(MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR);

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(
    result.result->exit_reason.value,
    MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR);
}

TEST_F(TerminalActionFixture, SafetyPreemptionCompletesWithAbortedAndPreservesReason)
{
  const auto result = run_immediate(MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(
    result.result->exit_reason.value,
    MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED);
}
}  // namespace
