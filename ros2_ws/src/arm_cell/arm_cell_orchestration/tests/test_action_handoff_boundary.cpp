#include <gtest/gtest.h>

// The test includes the implementation to exercise its ROS callback boundary.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsubobject-linkage"
#endif
#define ARM_CELL_ORCHESTRATION_TEST
#include "../src/orchestration_node.cpp"
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

namespace arm_cell_orchestration
{
struct OrchestrationNodeTestAccess
{
  static void observe_action_records(
    OrchestrationNode & node,
    std::function<void(const detail::QueuedOrchestrationRecord &)> callback)
  {
    node.test_action_record_ = std::move(callback);
  }
  static void observe_late_feedback(OrchestrationNode & node, std::function<void()> callback)
  {
    node.test_late_feedback_ = std::move(callback);
  }
  static bool recovery_mutex_is_available(OrchestrationNode & node)
  {
    std::unique_lock<std::mutex> lock(node.recovery_mutex_, std::try_to_lock);
    return lock.owns_lock();
  }
  static bool terminal_wins_after_handle_release(OrchestrationNode & node)
  {
    auto lifecycle =
      std::make_shared<OrchestrationNode::TaskExecutionLifecycle<OrchestrationNode::PnpAction>>();
    lifecycle->goal_response_received = true;
    lifecycle->terminal_result.emplace();
    auto submission = std::make_shared<CancellationRegistry::ActionSubmission>();
    return node.await_task_execution_lifecycle<OrchestrationNode::PnpAction>(
      submission,
      lifecycle) ==
           OrchestrationNode::TaskExecutionLifecycleEvent::TERMINAL;
  }
  static bool terminal_handoff_race(OrchestrationNode & node)
  {
    auto lifecycle =
      std::make_shared<OrchestrationNode::TaskExecutionLifecycle<OrchestrationNode::PnpAction>>();
    auto submission = std::make_shared<CancellationRegistry::ActionSubmission>();
    std::mutex gate;
    std::condition_variable ready;
    bool start = false;
    const auto wait = [&]() {
        std::unique_lock<std::mutex> lock(gate);
        ready.wait(lock, [&]() {return start;});
      };
    OrchestrationNode::TaskExecutionLifecycleEvent event{};
    std::thread waiter([&]() {
        wait();
        event =
        node.await_task_execution_lifecycle<OrchestrationNode::PnpAction>(submission, lifecycle);
      });
    std::thread terminal([&]() {
        wait();
        std::lock_guard<std::mutex> lock(node.recovery_mutex_);
        if (!lifecycle->handed_off) {lifecycle->terminal_result.emplace();}
        node.recovery_cv_.notify_all();
      });
    {
      std::lock_guard<std::mutex> lock(gate);
      start = true;
    }
    ready.notify_all();
    waiter.join();
    terminal.join();
    return event == OrchestrationNode::TaskExecutionLifecycleEvent::TERMINAL ||
           event == OrchestrationNode::TaskExecutionLifecycleEvent::SAFETY_HANDOFF;
  }
  static void register_cycle(OrchestrationNode & node, const std::string & key)
  {
    node.cancellation_registry_.register_execution(key, []() {});
  }
  static bool register_action(
    OrchestrationNode & node, const std::string & key,
    const CycleCoordinator::Submission & submission)
  {
    return node.cancellation_registry_.register_action_submission(key, submission);
  }
  static void set_safety(OrchestrationNode & node, bool stopped)
  {
    std::lock_guard<std::mutex> lock(node.recovery_mutex_);
    node.latest_safety_.valid = true;
    node.latest_safety_.required_inputs_fresh = true;
    node.latest_safety_.safety_state = stopped ?
      OrchestrationNode::SafetyState::SAFETY_STATE_STOPPED :
      OrchestrationNode::SafetyState::SAFETY_STATE_SAFE;
    node.latest_safety_.motion_capability.value = stopped ?
      arm_cell_interfaces::msg::MotionCapability::MOTION_RECOVERY_ONLY :
      arm_cell_interfaces::msg::MotionCapability::MOTION_NORMAL;
    node.safety_state_received_ = true;
    node.latest_safety_receipt_ = std::chrono::steady_clock::now();
    node.latest_motion_.execution_state = OrchestrationNode::MotionStatus::MOTION_STATE_STOPPED;
    node.latest_motion_.execution_active = false;
    node.latest_motion_.has_active_execution = false;
    node.latest_motion_.backend_inactivity_confirmed = true;
    node.latest_motion_receipt_ = std::chrono::steady_clock::now();
    node.recovery_cv_.notify_all();
  }
  static CycleCoordinator::PnpResult pnp(
    OrchestrationNode & node, const std::string & key,
    const CycleCoordinator::Submission & submission,
    const CycleCoordinator::PnpFeedbackFn & feedback)
  {
    return node.execute_pnp(
      key, OrchestrationNode::PnpAction::Goal{}, "test", feedback,
      submission);
  }
  static CycleCoordinator::DisturbResult disturb(
    OrchestrationNode & node, const std::string & key,
    const CycleCoordinator::Submission & submission)
  {
    return node.execute_disturb(key, OrchestrationNode::DisturbAction::Goal{}, submission);
  }
};
}

namespace
{
template<class Action>
void verify_late_acceptance_after_handoff(bool pnp)
{
  using namespace arm_cell_orchestration;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  const std::string key(16, 'c');
  const auto diagnostic_dir = std::string("/tmp/sftwin-action-boundary-") +
    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("mission_target_id", "RawPart"),
      rclcpp::Parameter("recipe_directory", "/tmp"),
      rclcpp::Parameter("diagnostics.capture_enabled", true),
      rclcpp::Parameter("diagnostics.output_directory", diagnostic_dir)});
  auto node = std::make_shared<OrchestrationNode>(options);
  auto server_node = std::make_shared<rclcpp::Node>("handoff_action_server");
  auto home_node = std::make_shared<rclcpp::Node>("recovery_home_server");
  std::mutex mutex;
  std::condition_variable changed;
  bool entered = false;
  bool release = false;
  int cancels = 0;
  std::thread execution;
  std::thread home_execution;
  bool finish_home = false;
  bool execution_started = false;
  bool home_execution_started = false;
  int home_cancels = 0;
  bool late_terminal_seen = false;
  bool accepted_seen = false;
  bool late_feedback_seen = false;
  bool normal_feedback_started = false;
  bool release_normal_feedback = false;
  bool recovery_mutex_was_available = false;
  std::atomic<bool> safety_update_completed{false};
  std::string late_id;
  std::string late_uuid;
  int late_material_placed = -1;
  OrchestrationNodeTestAccess::observe_action_records(
    *node, [&](const auto & record) {
      if (std::string(record.action_event.data()) == "accepted") {
        std::lock_guard<std::mutex> lock(mutex);
        accepted_seen = true;
        changed.notify_all();
      }
      if (std::string(record.action_event.data()) == "late_terminal") {
        std::lock_guard<std::mutex> lock(mutex);
        late_id = record.submission_id.data();
        late_uuid = record.action_goal_uuid.data();
        late_material_placed = record.material_placed;
        late_terminal_seen = true;
        changed.notify_all();
      }
    });
  OrchestrationNodeTestAccess::observe_late_feedback(
    *node, [&]() {
      std::lock_guard<std::mutex> lock(mutex);
      late_feedback_seen = true;
      changed.notify_all();
    });
  using Motion = OrchestrationNode::MotionAction;
  using MotionHandle = rclcpp_action::ServerGoalHandle<Motion>;
  auto home_server = rclcpp_action::create_server<Motion>(
    home_node, "/test/recovery_home",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Motion::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [&](const std::shared_ptr<MotionHandle>) {
      std::lock_guard<std::mutex> lock(mutex);
      ++home_cancels;
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&](const std::shared_ptr<MotionHandle> goal) {
      std::lock_guard<std::mutex> guard(mutex);
      home_execution = std::thread(
        [&, goal]() {
          std::unique_lock<std::mutex> lock(mutex);
          changed.wait_for(lock, std::chrono::seconds(3), [&]() {return finish_home;});
          lock.unlock();
          auto result = std::make_shared<Motion::Result>();
          if (goal->is_canceling()) {goal->canceled(result);} else {goal->succeed(result);}
        });
      home_execution_started = true;
      changed.notify_all();
    });
  auto home_client = rclcpp_action::create_client<Motion>(home_node, "/test/recovery_home");
  const auto name = pnp ? "/task_execution/execute_pnp" : "/task_execution/execute_disturb";
  auto server = rclcpp_action::create_server<Action>(
    server_node, name,
    [&](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename Action::Goal>) {
      std::unique_lock<std::mutex> lock(mutex);
      entered = true;
      changed.notify_all();
      changed.wait(lock, [&]() {return release;});
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [&](const std::shared_ptr<Handle>) {
      std::lock_guard<std::mutex> lock(mutex);
      ++cancels;
      changed.notify_all();
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&](const std::shared_ptr<Handle> goal) {
      std::lock_guard<std::mutex> guard(mutex);
      execution = std::thread(
        [&, goal]() {
          {
            std::unique_lock<std::mutex> lock(mutex);
            changed.wait_for(lock, std::chrono::seconds(2), [&]() {return accepted_seen;});
          }
          auto feedback = std::make_shared<typename Action::Feedback>();
          goal->publish_feedback(feedback);
          std::unique_lock<std::mutex> lock(mutex);
          changed.wait_for(
            lock, std::chrono::seconds(2), [&]() {
              return cancels != 0;
            });
          lock.unlock();
          if (pnp) {
            goal->publish_feedback(feedback);
            std::unique_lock<std::mutex> late_feedback_lock(mutex);
            changed.wait_for(
              late_feedback_lock, std::chrono::seconds(2), [&]() {return late_feedback_seen;});
          }
          auto result = std::make_shared<typename Action::Result>();
          if constexpr (std::is_same_v<Action, OrchestrationNode::PnpAction>) {
            result->material_placed = true;
            result->pnp_outcome = Action::Result::PNP_OUTCOME_FAILED;
          }
          if (goal->is_canceling()) {goal->canceled(result);} else {goal->abort(result);}
        });
      execution_started = true;
      changed.notify_all();
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(server_node);
  executor.add_node(home_node);
  std::thread spin([&]() {executor.spin();});
  OrchestrationNodeTestAccess::register_cycle(*node, key);
  OrchestrationNodeTestAccess::set_safety(*node, false);
  auto original = std::make_shared<CancellationRegistry::ActionSubmission>();
  std::atomic<int> feedback_count{0};
  bool handed_off = false;
  std::thread worker([&]() {
      if (pnp) {
        auto result = OrchestrationNodeTestAccess::pnp(
          *node, key, original,
          [&](std::uint8_t) {
            recovery_mutex_was_available =
            OrchestrationNodeTestAccess::recovery_mutex_is_available(*node);
            ++feedback_count;
            std::unique_lock<std::mutex> lock(mutex);
            normal_feedback_started = true;
            changed.notify_all();
            changed.wait(lock, [&]() {return release_normal_feedback;});
          });
        handed_off = result.safety_handoff;
      } else {
        handed_off = OrchestrationNodeTestAccess::disturb(*node, key, original).safety_handoff;
      }
    });
  bool submitted;
  {
    std::unique_lock<std::mutex> lock(mutex);
    submitted = changed.wait_for(lock, std::chrono::seconds(3), [&]() {return entered;});
  }
  if (pnp && submitted) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      release = true;
    }
    changed.notify_all();
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(
      changed.wait_for(
        lock, std::chrono::seconds(3), [&]() {
          return accepted_seen && normal_feedback_started;
        }));
    lock.unlock();
    std::thread safety_update([&]() {
        OrchestrationNodeTestAccess::set_safety(*node, true);
        safety_update_completed.store(true);
        changed.notify_all();
      });
    {
      std::unique_lock<std::mutex> update_lock(mutex);
      EXPECT_TRUE(
        changed.wait_for(
          update_lock, std::chrono::seconds(2), [&]() {
            return safety_update_completed.load();
          }));
    }
    {
      std::lock_guard<std::mutex> release_lock(mutex);
      release_normal_feedback = true;
    }
    changed.notify_all();
    safety_update.join();
  } else {
    OrchestrationNodeTestAccess::set_safety(*node, true);
  }
  worker.join();
  EXPECT_TRUE(submitted);
  EXPECT_TRUE(handed_off);
  EXPECT_FALSE(original->submission_id().empty());
  EXPECT_EQ(original->goal_uuid().empty(), !pnp);
  auto next = std::make_shared<CancellationRegistry::ActionSubmission>();
  EXPECT_TRUE(OrchestrationNodeTestAccess::register_action(*node, key, next));
  EXPECT_TRUE(home_client->wait_for_action_server(std::chrono::seconds(2)));
  Motion::Goal home_goal;
  home_goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;
  auto home_future = home_client->async_send_goal(home_goal);
  const auto home_ready = home_future.wait_for(std::chrono::seconds(2));
  EXPECT_EQ(home_ready, std::future_status::ready);
  rclcpp_action::ClientGoalHandle<Motion>::SharedPtr home_handle;
  if (home_ready == std::future_status::ready) {home_handle = home_future.get();}
  EXPECT_TRUE(home_handle);
  if (home_handle) {
    next->set_goal_uuid(rclcpp_action::to_string(home_handle->get_goal_id()));
    next->set_cancel_fn(
      [home_client, home_handle]() {
        home_client->async_cancel_goal(home_handle);
      });
  }
  {
    std::lock_guard<std::mutex> lock(mutex);
    release = true;
  }
  changed.notify_all();
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(
      changed.wait_for(
        lock, std::chrono::seconds(3), [&]() {
          return cancels > 0 && execution_started && home_execution_started;
        }));
    EXPECT_EQ(cancels, 1);
  }
  if (execution.joinable()) {execution.join();}
  {
    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(
      changed.wait_for(
        lock, std::chrono::seconds(3),
        [&]() {return late_terminal_seen;}));
    EXPECT_EQ(late_id, original->submission_id());
    EXPECT_EQ(late_uuid, original->goal_uuid());
    if (pnp) {EXPECT_EQ(late_material_placed, 1);}
  }
  EXPECT_FALSE(original->goal_uuid().empty());
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(home_cancels, 0);
    finish_home = true;
  }
  changed.notify_all();
  if (home_handle) {
    auto home_result = home_client->async_get_result(home_handle);
    EXPECT_EQ(home_result.wait_for(std::chrono::seconds(3)), std::future_status::ready);
    if (home_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      EXPECT_EQ(home_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    }
  }
  if (home_execution.joinable()) {home_execution.join();}
  EXPECT_EQ(feedback_count.load(), pnp ? 1 : 0);
  if (pnp) {EXPECT_TRUE(recovery_mutex_was_available);}
  if (pnp) {EXPECT_TRUE(late_feedback_seen);}
  executor.cancel();
  spin.join();
  executor.remove_node(node);
  node.reset();
  std::string diagnostics;
  for (const auto & entry : std::filesystem::directory_iterator(diagnostic_dir)) {
    std::ifstream input(entry.path());
    diagnostics.append(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
  }
  EXPECT_NE(diagnostics.find("\"action_event\":\"late_terminal\""), std::string::npos);
  EXPECT_NE(diagnostics.find(original->submission_id()), std::string::npos);
  EXPECT_NE(diagnostics.find(original->goal_uuid()), std::string::npos);
  std::filesystem::remove_all(diagnostic_dir);
}

class ActionHandoffBoundaryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {int argc = 0; rclcpp::init(argc, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(ActionHandoffBoundaryTest, TerminalIsNotReclassifiedAsGoalRejection)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("mission_target_id", "RawPart"),
      rclcpp::Parameter("recipe_directory", "/tmp"),
      rclcpp::Parameter("diagnostics.output_directory", "/tmp/sftwin-terminal-precedence")});
  auto node = std::make_shared<arm_cell_orchestration::OrchestrationNode>(options);
  arm_cell_orchestration::OrchestrationNodeTestAccess::set_safety(*node, false);
  EXPECT_TRUE(
    arm_cell_orchestration::OrchestrationNodeTestAccess::terminal_wins_after_handle_release(
      *node));
}

TEST_F(ActionHandoffBoundaryTest, TerminalAndHandoffRacePreservesEitherOriginalOutcome)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("mission_target_id", "RawPart"),
      rclcpp::Parameter("recipe_directory", "/tmp"),
      rclcpp::Parameter("diagnostics.output_directory", "/tmp/sftwin-terminal-race")});
  auto node = std::make_shared<arm_cell_orchestration::OrchestrationNode>(options);
  for (int i = 0; i < 32; ++i) {
    arm_cell_orchestration::OrchestrationNodeTestAccess::set_safety(*node, true);
    EXPECT_TRUE(arm_cell_orchestration::OrchestrationNodeTestAccess::terminal_handoff_race(*node));
  }
}

TEST_F(ActionHandoffBoundaryTest, PnpLateAcceptanceCancelsOnlyOriginal)
{
  verify_late_acceptance_after_handoff<arm_cell_orchestration::OrchestrationNode::PnpAction>(true);
}

TEST_F(ActionHandoffBoundaryTest, DisturbLateAcceptanceCancelsOnlyOriginal)
{
  verify_late_acceptance_after_handoff<arm_cell_orchestration::OrchestrationNode::DisturbAction>(
    false);
}
}
