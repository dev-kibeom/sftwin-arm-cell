#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "arm_cell_orchestration/cycle_coordinator.hpp"

static std::shared_ptr<arm_cell_orchestration::CancellationRegistry::ActionSubmission>
register_test_submission(arm_cell_orchestration::CancellationRegistry & registry, const std::string & key)
{
  auto submission = std::make_shared<arm_cell_orchestration::CancellationRegistry::ActionSubmission>();
  return registry.register_action_submission(key, submission) ? submission : nullptr;
}

namespace
{
using Action = arm_cell_interfaces::action::ExecuteDisturb;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

class DelayedTaskExecutionGoalOwnershipTest : public ::testing::Test
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
    server_node_ = std::make_shared<rclcpp::Node>("delayed_motion_server");
    client_node_ = std::make_shared<rclcpp::Node>("delayed_motion_client");
    action_server_ = rclcpp_action::create_server<Action>(
      server_node_, "/test/delayed_motion_goal_ownership",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        std::unique_lock<std::mutex> lock(goal_submission_mutex_);
        goal_submission_entered_ = true;
        goal_submission_condition_.notify_all();
        goal_submission_condition_.wait(
          lock, [this]() {return release_goal_submission_;});
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (reject_cancel_) {
            cancel_rejected_ = true;
            condition_.notify_all();
            return rclcpp_action::CancelResponse::REJECT;
          }
          ++cancel_count_;
          cancel_received_ = true;
        }
        condition_.notify_all();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        accepted_thread_ = std::thread(
          [this, handle]() {
            std::unique_lock<std::mutex> lock(mutex_);
            if (hold_terminal_) {
              condition_.wait(lock, [this]() {return terminal_release_;});
            } else {
              condition_.wait_for(lock, std::chrono::seconds(3), [this]() {
                  return cancel_received_ || cancel_rejected_;
                });
            }
            const bool canceled = cancel_received_;
            lock.unlock();
            auto result = std::make_shared<Action::Result>();
            if (canceled) {
              handle->canceled(result);
            } else {
              result->diagnostic_detail = "Timed out waiting for Motion Goal cancel request";
              handle->abort(result);
            }
          });
      });
    action_client_ = rclcpp_action::create_client<Action>(
      client_node_, "/test/delayed_motion_goal_ownership");
    executor_.add_node(server_node_);
    executor_.add_node(client_node_);
    executor_thread_ = std::thread([this]() {executor_.spin();});
    ASSERT_TRUE(action_client_->wait_for_action_server(std::chrono::seconds(2)));
  }

  void TearDown() override
  {
    release_goal_submission();
    release_terminal();
    executor_.cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    if (accepted_thread_.joinable()) {
      accepted_thread_.join();
    }
    action_client_.reset();
    action_server_.reset();
    client_node_.reset();
    server_node_.reset();
  }

  bool wait_for_goal_submission()
  {
    std::unique_lock<std::mutex> lock(goal_submission_mutex_);
    return goal_submission_condition_.wait_for(
      lock, std::chrono::seconds(2), [this]() {return goal_submission_entered_;});
  }

  void release_goal_submission()
  {
    {
      std::lock_guard<std::mutex> lock(goal_submission_mutex_);
      release_goal_submission_ = true;
    }
    goal_submission_condition_.notify_all();
  }

  void release_terminal()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      terminal_release_ = true;
    }
    condition_.notify_all();
  }

  bool wait_for_cancel_rejection()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
      lock, std::chrono::seconds(2), [this]() {return cancel_rejected_;});
  }

  rclcpp::Node::SharedPtr server_node_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp_action::Client<Action>::SharedPtr action_client_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread executor_thread_;
  std::thread accepted_thread_;
  std::mutex mutex_;
  std::condition_variable condition_;
  int cancel_count_{0};
  bool cancel_received_{false};
  bool cancel_rejected_{false};
  bool reject_cancel_{false};
  bool hold_terminal_{false};
  bool terminal_release_{false};
  std::mutex goal_submission_mutex_;
  std::condition_variable goal_submission_condition_;
  bool goal_submission_entered_{false};
  bool release_goal_submission_{false};
};

TEST_F(
  DelayedTaskExecutionGoalOwnershipTest,
  MissionTerminalCancelsExecuteDisturbAcceptedAfterMissionTerminal)
{
  ASSERT_TRUE(action_client_->wait_for_action_server(std::chrono::seconds(2)));

  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("mission", []() {});
  auto submission = register_test_submission(registry, "mission");
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, submission](
    const rclcpp_action::ClientGoalHandle<Action>::SharedPtr & handle) {
      if (handle) {
        submission->set_cancel_fn([this, handle]() {action_client_->async_cancel_goal(handle);});
      }
    };
  auto goal_future = action_client_->async_send_goal(Action::Goal{}, options);

  ASSERT_TRUE(wait_for_goal_submission());
  registry.erase_execution("mission");
  release_goal_submission();

  ASSERT_EQ(goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto goal_handle = goal_future.get();
  ASSERT_TRUE(goal_handle);
  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(4)), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED)
    << (result.result ? result.result->diagnostic_detail : "No Action result was returned");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(cancel_received_);
  }
}

TEST_F(
  DelayedTaskExecutionGoalOwnershipTest,
  SafetyCancelBeforeAcceptanceTargetsExecuteDisturbGoal)
{
  arm_cell_orchestration::CancellationRegistry registry;
  bool mission_execution_canceled = false;
  registry.register_execution("mission", [&]() {mission_execution_canceled = true;});
  auto submission = register_test_submission(registry, "mission");
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, submission](
    const rclcpp_action::ClientGoalHandle<Action>::SharedPtr & handle) {
      if (handle) {
        const auto & uuid = handle->get_goal_id();
        EXPECT_TRUE(submission->set_goal_uuid(std::string(uuid.begin(), uuid.end())));
        submission->set_cancel_fn([this, handle]() {action_client_->async_cancel_goal(handle);});
      }
    };
  auto goal_future = action_client_->async_send_goal(Action::Goal{}, options);

  ASSERT_TRUE(wait_for_goal_submission());
  EXPECT_FALSE(register_test_submission(registry, "mission"));
  submission->request_cancel();
  release_goal_submission();

  ASSERT_EQ(goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto goal_handle = goal_future.get();
  ASSERT_TRUE(goal_handle);
  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(4)), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED)
    << (result.result ? result.result->diagnostic_detail : "No Action result was returned");
  EXPECT_EQ(
    submission->goal_uuid(), std::string(
      goal_handle->get_goal_id().begin(), goal_handle->get_goal_id().end()));
  EXPECT_FALSE(mission_execution_canceled);
  registry.clear_active_action("mission", submission);
  EXPECT_TRUE(register_test_submission(registry, "mission"));
  registry.erase_execution("mission");
}

TEST_F(
  DelayedTaskExecutionGoalOwnershipTest,
  CancelAcceptanceDoesNotReleaseOwnershipBeforeTerminal)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_terminal_ = true;
  }
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("mission", []() {});
  auto submission = register_test_submission(registry, "mission");
  ASSERT_TRUE(submission);

  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, submission](
    const rclcpp_action::ClientGoalHandle<Action>::SharedPtr & handle) {
      if (handle) {
        const auto & uuid = handle->get_goal_id();
        EXPECT_TRUE(submission->set_goal_uuid(std::string(uuid.begin(), uuid.end())));
        submission->set_cancel_fn([this, handle]() {action_client_->async_cancel_goal(handle);});
      }
    };
  auto goal_future = action_client_->async_send_goal(Action::Goal{}, options);

  ASSERT_TRUE(wait_for_goal_submission());
  release_goal_submission();
  ASSERT_EQ(goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto goal_handle = goal_future.get();
  ASSERT_TRUE(goal_handle);
  auto result_future = action_client_->async_get_result(goal_handle);

  submission->request_cancel();
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ASSERT_TRUE(
      condition_.wait_for(
        lock, std::chrono::seconds(2), [this]() {
          return cancel_received_;
        }));
  }
  EXPECT_FALSE(register_test_submission(registry, "mission"));

  release_terminal();
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(register_test_submission(registry, "mission"));
  registry.clear_active_action("mission", submission);
  EXPECT_TRUE(register_test_submission(registry, "mission"));
  registry.erase_execution("mission");
}

TEST_F(
  DelayedTaskExecutionGoalOwnershipTest,
  RejectedCancelAbortsServerGoalAndTerminatesWorker)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    reject_cancel_ = true;
  }
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("mission", []() {});
  auto submission = register_test_submission(registry, "mission");
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, submission](
    const rclcpp_action::ClientGoalHandle<Action>::SharedPtr & handle) {
      if (handle) {
        submission->set_cancel_fn([this, handle]() {action_client_->async_cancel_goal(handle);});
      }
    };
  auto goal_future = action_client_->async_send_goal(Action::Goal{}, options);

  ASSERT_TRUE(wait_for_goal_submission());
  submission->request_cancel();
  release_goal_submission();
  ASSERT_EQ(goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto goal_handle = goal_future.get();
  ASSERT_TRUE(goal_handle);
  auto result_future = action_client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(4)), std::future_status::ready);
  const auto result = result_future.get();
  ASSERT_TRUE(wait_for_cancel_rejection());

  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->diagnostic_detail, "Timed out waiting for Motion Goal cancel request");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_FALSE(cancel_received_);
  }
  registry.clear_active_action("mission", submission);
  registry.erase_execution("mission");
}
}  // namespace

TEST_F(DelayedTaskExecutionGoalOwnershipTest, LateOldCancelDoesNotCancelNewAcceptedGoal)
{
  arm_cell_orchestration::CancellationRegistry registry;
  registry.register_execution("mission", []() {});
  auto old_action = register_test_submission(registry, "mission");
  ASSERT_TRUE(old_action);
  int old_cancels = 0;
  old_action->set_cancel_fn([&]() {++old_cancels;});
  registry.clear_active_action("mission", old_action);
  auto new_action = register_test_submission(registry, "mission");
  ASSERT_TRUE(new_action);
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, new_action](
    const rclcpp_action::ClientGoalHandle<Action>::SharedPtr & handle) {
      if (handle) {
        new_action->set_cancel_fn([this, handle]() {action_client_->async_cancel_goal(handle);});
      }
    };
  auto goal_future = action_client_->async_send_goal(Action::Goal{}, options);
  ASSERT_TRUE(wait_for_goal_submission());
  release_goal_submission();
  ASSERT_EQ(goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  auto goal = goal_future.get();
  ASSERT_TRUE(goal);
  old_action->request_cancel();
  registry.clear_active_action("mission", old_action);
  EXPECT_EQ(old_cancels, 1);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_FALSE(cancel_received_);
    EXPECT_EQ(cancel_count_, 0);
  }
  registry.request_caller_cancel("mission");
  auto result_future = action_client_->async_get_result(goal);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(4)), std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(cancel_count_, 1);
  }
  registry.clear_active_action("mission", new_action);
}
