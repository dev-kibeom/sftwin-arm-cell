#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "arm_cell_task_execution/execute_pnp_action_server.hpp"
#include "arm_cell_task_execution/legacy_execute_task_action_server.hpp"
#include "arm_cell_task_execution/task_executor.hpp"
#include "arm_cell_motion/fake_motion_backend.hpp"
#include "arm_cell_motion/motion_core.hpp"

namespace
{
using Action = arm_cell_interfaces::action::ExecuteTask;
using PnpAction = arm_cell_interfaces::action::ExecutePnp;
using Code = arm_cell_interfaces::msg::MotionTaskResultCode;

class ActionMotionPort : public arm_cell_motion::MotionPort
{
public:
  arm_cell_motion::MotionPlanningResult planMotion(
    const arm_cell_motion::MotionPortRequest & request) override
  {
    arm_cell_motion::MotionPlanningResult planning;
    planning.result_code = Code::TASK_RESULT_SUCCESS;
    planning.plan.plan_id = next_plan_id_++;
    planning.plan.execution_id = request.execution_id;
    planning.plan.movement_id = request.movement_id;
    planned_requests_[planning.plan.plan_id] = request;
    return planning;
  }
  arm_cell_motion::MotionPortResult executePlannedMotion(
    const arm_cell_motion::MotionPlan & plan) override
  {
    const auto it = planned_requests_.find(plan.plan_id);
    if (it == planned_requests_.end()) {
      return {plan.execution_id, plan.movement_id, Code::TASK_RESULT_INVALID_GOAL,
        arm_cell_motion::MotionFailureKind::OTHER, {}, {}, false,
        arm_cell_interfaces::msg::MotionStatus()};
    }
    const auto request = it->second;
    planned_requests_.erase(it);
    return planAndExecuteMotion(request);
  }
  arm_cell_motion::MotionPortResult planAndExecuteMotion(
    const arm_cell_motion::MotionPortRequest & request) override
  {return {request.execution_id, request.movement_id, Code::TASK_RESULT_SUCCESS,
    arm_cell_motion::MotionFailureKind::NONE, {}, {}, true,
    arm_cell_interfaces::msg::MotionStatus()};}
  arm_cell_motion::MotionCancelResult cancelMotion(
    const unique_identifier_msgs::msg::UUID &, const std::string &) override
  {return {true, true, arm_cell_interfaces::msg::MotionStatus()};}
  arm_cell_motion::MotionObservation observeMotionState() const override
  {
    return {arm_cell_interfaces::msg::MotionStatus(), false, sensor_msgs::msg::JointState()};
  }
  std::uint8_t executeLegacyMovement(
    std::uint8_t, const std::string &, const std::string &) override
  {return Code::TASK_RESULT_SUCCESS;}
  bool stagePickCaptureReference(
    const geometry_msgs::msg::PoseStamped &, double, std::uint8_t, const std::string &) override
  {return true;}
  bool commitPendingGraspRelation() override {return true;}

private:
  std::unordered_map<std::uint64_t, arm_cell_motion::MotionPortRequest> planned_requests_;
  std::uint64_t next_plan_id_{1};
};

class CancellableActionMotionPort final : public ActionMotionPort
{
public:
  std::uint8_t executeLegacyMovement(
    std::uint8_t, const std::string &, const std::string &) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (started_) {return Code::TASK_RESULT_BUSY;}
    started_ = true;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return canceled_;});
    return Code::TASK_RESULT_CANCELED;
  }

  arm_cell_motion::MotionCancelResult cancelMotion(
    const unique_identifier_msgs::msg::UUID & execution_id,
    const std::string & movement_id) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_execution_id_ = execution_id;
    cancel_movement_id_ = movement_id;
    cancel_called_ = true;
    canceled_ = true;
    condition_.notify_all();
    return {true, true, arm_cell_interfaces::msg::MotionStatus()};
  }

  bool wait_started(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return started_;});
  }
  bool cancel_called() const {std::lock_guard<std::mutex> lock(mutex_); return cancel_called_;}
  std::string cancel_movement_id() const
  {std::lock_guard<std::mutex> lock(mutex_); return cancel_movement_id_;}
  bool has_cancel_execution_id() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(cancel_execution_id_.uuid.begin(), cancel_execution_id_.uuid.end(),
      [](std::uint8_t byte) {return byte != 0U;});
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool started_{false};
  bool canceled_{false};
  bool cancel_called_{false};
  unique_identifier_msgs::msg::UUID cancel_execution_id_;
  std::string cancel_movement_id_;
};

class LifecycleActionMotionPort final : public ActionMotionPort
{
public:
  enum class NextBehavior
  {
    NORMAL,
    BLOCK_PICK_APPROACH,
    THROW_PICK_APPROACH,
    THROW_PLACE_RETRACT,
    SAFETY_STOP_THEN_THROW_PICK_APPROACH,
    THROW_LEGACY_HOME
  };

  arm_cell_motion::MotionPortResult planAndExecuteMotion(
    const arm_cell_motion::MotionPortRequest & request) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (behavior_ == NextBehavior::BLOCK_PICK_APPROACH &&
      request.segment == arm_cell_motion::MotionSegmentKind::PICK_APPROACH)
    {
      started_ = true;
      condition_.notify_all();
      condition_.wait(lock, [this]() {return canceled_;});
      behavior_ = NextBehavior::NORMAL;
      canceled_ = false;
      return {request.execution_id, request.movement_id, Code::TASK_RESULT_CANCELED,
        arm_cell_motion::MotionFailureKind::CANCELED, {}, {}, false,
        arm_cell_interfaces::msg::MotionStatus()};
    }
    if (behavior_ == NextBehavior::THROW_PICK_APPROACH &&
      request.segment == arm_cell_motion::MotionSegmentKind::PICK_APPROACH)
    {
      behavior_ = NextBehavior::NORMAL;
      throw std::runtime_error("lifecycle motion exception");
    }
    if (behavior_ == NextBehavior::SAFETY_STOP_THEN_THROW_PICK_APPROACH &&
      request.segment == arm_cell_motion::MotionSegmentKind::PICK_APPROACH)
    {
      behavior_ = NextBehavior::NORMAL;
      motion_status_.active_execution_id = request.execution_id;
      motion_status_.execution_state =
        arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_STOPPED;
      throw std::runtime_error("exception after Safety Stop");
    }
    if (behavior_ == NextBehavior::THROW_PLACE_RETRACT &&
      request.segment == arm_cell_motion::MotionSegmentKind::PLACE_RETRACT)
    {
      behavior_ = NextBehavior::NORMAL;
      throw std::runtime_error("post-release motion exception");
    }
    return {request.execution_id, request.movement_id, Code::TASK_RESULT_SUCCESS,
      arm_cell_motion::MotionFailureKind::NONE, {}, {}, true,
      arm_cell_interfaces::msg::MotionStatus()};
  }

  arm_cell_motion::MotionCancelResult cancelMotion(
    const unique_identifier_msgs::msg::UUID & execution_id,
    const std::string & movement_id) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_execution_id_ = execution_id;
    cancel_movement_id_ = movement_id;
    canceled_ = true;
    condition_.notify_all();
    return {true, true, arm_cell_interfaces::msg::MotionStatus()};
  }

  std::uint8_t executeLegacyMovement(
    std::uint8_t task_type, const std::string &, const std::string &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (behavior_ == NextBehavior::THROW_LEGACY_HOME &&
      task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME)
    {
      behavior_ = NextBehavior::NORMAL;
      throw std::runtime_error("legacy lifecycle motion exception");
    }
    return Code::TASK_RESULT_SUCCESS;
  }

  arm_cell_motion::MotionObservation observeMotionState() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    arm_cell_motion::MotionObservation observation;
    observation.motion_status = motion_status_;
    return observation;
  }

  void set_next_behavior(NextBehavior behavior)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    behavior_ = behavior;
    started_ = false;
    canceled_ = false;
    motion_status_.execution_state = arm_cell_interfaces::msg::MotionStatus::MOTION_STATE_IDLE;
  }

  bool wait_started(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return started_;});
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  NextBehavior behavior_{NextBehavior::NORMAL};
  bool started_{false};
  bool canceled_{false};
  arm_cell_interfaces::msg::MotionStatus motion_status_;
  unique_identifier_msgs::msg::UUID cancel_execution_id_;
  std::string cancel_movement_id_;
};

arm_cell_motion::MotionGeometry action_lifecycle_geometry()
{
  arm_cell_motion::MotionGeometry geometry;
  geometry.configured = true;
  geometry.observation_reference = "profile_reference";
  geometry.observation_to_object.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.rotation.w = 1.0;
  geometry.object_to_grasp_tcp.translation.z = 0.1;
  geometry.insertion_axis_tcp.z = 1.0;
  geometry.pick_retract_distance_m = 0.4;
  return geometry;
}

PnpAction::Goal action_lifecycle_pnp_goal()
{
  PnpAction::Goal goal;
  goal.vision_acquisition_id = "action-lifecycle-acquisition";
  arm_cell_interfaces::msg::PickCandidate candidate;
  candidate.acquisition_id = goal.vision_acquisition_id;
  candidate.candidate_id = "action-lifecycle-candidate";
  candidate.target_pose.header.frame_id = "base_link";
  candidate.target_pose.pose.orientation.w = 1.0;
  candidate.target_pose.pose.position.z = 1.0;
  candidate.orientation_symmetry_order = Action::Goal::ORIENTATION_FOURFOLD;
  candidate.estimated_height_m = 0.1F;
  candidate.has_estimated_height = true;
  candidate.has_observed_rectangle = true;
  candidate.rectangle_long_side_m = 0.07F;
  candidate.rectangle_short_side_m = 0.07F;
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
  goal.place_orientation_constraint = PnpAction::Goal::PLACE_ORIENTATION_BOUNDED;
  goal.place_approach_direction_object.z = 1.0;
  goal.place_approach_distance_m = 0.2;
  goal.place_retract_distance_m = 0.4;
  return goal;
}
}  // namespace

TEST(LegacyExecuteTaskActionServer, KeepsExistingEndpointAndResultContract)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp::Node>("legacy_execute_task_contract_test");
  auto motion = std::make_shared<ActionMotionPort>();
  auto gripper = std::make_shared<arm_cell_motion::FakeMotionBackendGripperPort>();
  auto task_executor = std::make_shared<arm_cell_task_execution::TaskExecutor>(
    motion, gripper, arm_cell_motion::MotionGeometry{},
    arm_cell_task_execution::TaskExecutor::InterruptionChecker{}, std::chrono::milliseconds(10));
  arm_cell_task_execution::LegacyExecuteTaskActionServer server(
    node.get(), motion, task_executor);
  auto client = rclcpp_action::create_client<Action>(node, "/motion/execute_task");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));

  Action::Goal goal;
  goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;
  const auto goal_future = client->async_send_goal(goal);
  ASSERT_EQ(goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);
  const auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto wrapped = result_future.get();
  ASSERT_NE(wrapped.result, nullptr);
  EXPECT_EQ(wrapped.result->result_code.value, Code::TASK_RESULT_SUCCESS);

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
}

TEST(LegacyExecuteTaskActionServer, SerializesGoalsAndRoutesCancellationFeedbackAndTerminalState)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp::Node>("legacy_execute_task_cancel_test");
  auto motion = std::make_shared<CancellableActionMotionPort>();
  auto gripper = std::make_shared<arm_cell_motion::FakeMotionBackendGripperPort>();
  auto task_executor = std::make_shared<arm_cell_task_execution::TaskExecutor>(
    motion, gripper, arm_cell_motion::MotionGeometry{},
    arm_cell_task_execution::TaskExecutor::InterruptionChecker{}, std::chrono::milliseconds(10));
  arm_cell_task_execution::LegacyExecuteTaskActionServer server(
    node.get(), motion, task_executor);
  auto client = rclcpp_action::create_client<Action>(node, "/motion/execute_task");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));

  std::atomic_size_t feedback_count{0};
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback = [&feedback_count](
      rclcpp_action::ClientGoalHandle<Action>::SharedPtr,
      const std::shared_ptr<const Action::Feedback>) {++feedback_count;};
  Action::Goal goal;
  goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;
  const auto goal_future = client->async_send_goal(goal, options);
  ASSERT_EQ(goal_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto goal_handle = goal_future.get();
  ASSERT_NE(goal_handle, nullptr);
  ASSERT_TRUE(motion->wait_started(std::chrono::seconds(2)));

  const auto competing_future = client->async_send_goal(goal, options);
  ASSERT_EQ(competing_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto competing_handle = competing_future.get();
  ASSERT_NE(competing_handle, nullptr);
  const auto competing_result_future = client->async_get_result(competing_handle);
  ASSERT_EQ(competing_result_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto competing_result = competing_result_future.get();
  ASSERT_NE(competing_result.result, nullptr);
  EXPECT_EQ(competing_result.result->result_code.value, Code::TASK_RESULT_BUSY);

  const auto cancel_future = client->async_cancel_goal(goal_handle);
  ASSERT_EQ(cancel_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(cancel_future.get()->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  const auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto wrapped = result_future.get();
  ASSERT_NE(wrapped.result, nullptr);
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(wrapped.result->result_code.value, Code::TASK_RESULT_CANCELED);
  EXPECT_GT(feedback_count.load(), 0U);
  EXPECT_TRUE(motion->cancel_called());
  EXPECT_FALSE(motion->cancel_movement_id().empty());
  EXPECT_TRUE(motion->has_cancel_execution_id());

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
}

TEST(ExecutePnpActionServer, ReusesSharedExecutorSafelyAcrossTerminalCancelExceptionAndLegacyGoals)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp::Node>("execute_pnp_lifecycle_test");
  auto motion = std::make_shared<LifecycleActionMotionPort>();
  auto gripper = std::make_shared<arm_cell_motion::FakeMotionBackendGripperPort>();
  auto support = std::make_shared<arm_cell_task_execution::TaskExecutionSupport>(
    motion, gripper, action_lifecycle_geometry(), std::chrono::milliseconds(10));
  auto task_executor = std::make_shared<arm_cell_task_execution::TaskExecutor>(support);
  arm_cell_task_execution::ExecutePnpActionServer pnp_server(
    node.get(), motion, task_executor);
  arm_cell_task_execution::LegacyExecuteTaskActionServer legacy_server(
    node.get(), motion, task_executor);
  auto pnp_client = rclcpp_action::create_client<PnpAction>(
    node, "/task_execution/execute_pnp");
  auto legacy_client = rclcpp_action::create_client<Action>(node, "/motion/execute_task");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct SpinCleanup
  {
    rclcpp::executors::MultiThreadedExecutor & executor;
    std::thread & spin_thread;
    ~SpinCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {spin_thread.join();}
      rclcpp::shutdown();
    }
  } spin_cleanup{executor, spin_thread};

  ASSERT_TRUE(pnp_client->wait_for_action_server(std::chrono::seconds(2)));
  ASSERT_TRUE(legacy_client->wait_for_action_server(std::chrono::seconds(2)));
  using PnpClient = rclcpp_action::Client<PnpAction>;
  using LegacyClient = rclcpp_action::Client<Action>;
  const auto run_pnp = [&pnp_client](PnpAction::Goal goal)
      -> std::optional<PnpClient::WrappedResult> {
      auto goal_future = pnp_client->async_send_goal(goal);
      if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        ADD_FAILURE() << "ExecutePnp goal acceptance timed out";
        return std::nullopt;
      }
      const auto goal_handle = goal_future.get();
      if (!goal_handle) {
        ADD_FAILURE() << "ExecutePnp goal was rejected";
        return std::nullopt;
      }
      auto result_future = pnp_client->async_get_result(goal_handle);
      if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        ADD_FAILURE() << "ExecutePnp terminal result timed out";
        return std::nullopt;
      }
      return result_future.get();
    };
  const auto run_legacy = [&legacy_client]()
      -> std::optional<LegacyClient::WrappedResult> {
      Action::Goal goal;
      goal.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;
      auto goal_future = legacy_client->async_send_goal(goal);
      if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        ADD_FAILURE() << "Legacy goal acceptance timed out";
        return std::nullopt;
      }
      const auto goal_handle = goal_future.get();
      if (!goal_handle) {
        ADD_FAILURE() << "Legacy goal was rejected";
        return std::nullopt;
      }
      auto result_future = legacy_client->async_get_result(goal_handle);
      if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        ADD_FAILURE() << "Legacy terminal result timed out";
        return std::nullopt;
      }
      return result_future.get();
    };
  const auto expect_callbacks_cleared = [&support]() {
      EXPECT_FALSE(support->interruption_checker());
      EXPECT_FALSE(support->phase_callback());
    };

  auto first = run_pnp(action_lifecycle_pnp_goal());
  ASSERT_TRUE(first.has_value());
  ASSERT_NE(first->result, nullptr);
  EXPECT_EQ(first->code, rclcpp_action::ResultCode::SUCCEEDED) << first->result->diagnostic_detail;
  EXPECT_EQ(first->result->pnp_outcome, PnpAction::Result::PNP_OUTCOME_COMPLETED);
  EXPECT_TRUE(first->result->material_placed);
  expect_callbacks_cleared();

  auto after_pnp = run_legacy();
  ASSERT_TRUE(after_pnp.has_value());
  ASSERT_NE(after_pnp->result, nullptr);
  EXPECT_EQ(after_pnp->result->result_code.value, Code::TASK_RESULT_SUCCESS);
  expect_callbacks_cleared();

  auto after_legacy = run_pnp(action_lifecycle_pnp_goal());
  ASSERT_TRUE(after_legacy.has_value());
  ASSERT_NE(after_legacy->result, nullptr);
  EXPECT_EQ(after_legacy->code, rclcpp_action::ResultCode::SUCCEEDED)
    << after_legacy->result->diagnostic_detail;
  EXPECT_EQ(after_legacy->result->pnp_outcome, PnpAction::Result::PNP_OUTCOME_COMPLETED);
  expect_callbacks_cleared();

  motion->set_next_behavior(LifecycleActionMotionPort::NextBehavior::BLOCK_PICK_APPROACH);
  auto cancel_goal_future = pnp_client->async_send_goal(action_lifecycle_pnp_goal());
  ASSERT_EQ(cancel_goal_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto cancel_goal_handle = cancel_goal_future.get();
  ASSERT_NE(cancel_goal_handle, nullptr);
  ASSERT_TRUE(motion->wait_started(std::chrono::seconds(3)));

  auto competing_pnp_future = pnp_client->async_send_goal(action_lifecycle_pnp_goal());
  ASSERT_EQ(competing_pnp_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto competing_pnp_handle = competing_pnp_future.get();
  ASSERT_NE(competing_pnp_handle, nullptr);
  auto competing_pnp_result_future = pnp_client->async_get_result(competing_pnp_handle);
  ASSERT_EQ(competing_pnp_result_future.wait_for(std::chrono::seconds(3)),
    std::future_status::ready);
  const auto competing_pnp_result = competing_pnp_result_future.get();
  ASSERT_NE(competing_pnp_result.result, nullptr);
  EXPECT_EQ(competing_pnp_result.result->result_code.value, Code::TASK_RESULT_BUSY);

  Action::Goal competing_legacy_goal;
  competing_legacy_goal.task_type.value =
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;
  auto competing_legacy_future = legacy_client->async_send_goal(competing_legacy_goal);
  ASSERT_EQ(competing_legacy_future.wait_for(std::chrono::seconds(3)),
    std::future_status::ready);
  const auto competing_legacy_handle = competing_legacy_future.get();
  ASSERT_NE(competing_legacy_handle, nullptr);
  auto competing_legacy_result_future = legacy_client->async_get_result(competing_legacy_handle);
  ASSERT_EQ(competing_legacy_result_future.wait_for(std::chrono::seconds(3)),
    std::future_status::ready);
  const auto competing_legacy_result = competing_legacy_result_future.get();
  ASSERT_NE(competing_legacy_result.result, nullptr);
  EXPECT_EQ(competing_legacy_result.result->result_code.value, Code::TASK_RESULT_BUSY);

  auto cancel_future = pnp_client->async_cancel_goal(cancel_goal_handle);
  ASSERT_EQ(cancel_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_EQ(cancel_future.get()->return_code, action_msgs::srv::CancelGoal::Response::ERROR_NONE);
  auto canceled_result_future = pnp_client->async_get_result(cancel_goal_handle);
  ASSERT_EQ(canceled_result_future.wait_for(std::chrono::seconds(5)),
    std::future_status::ready);
  const auto canceled_result = canceled_result_future.get();
  ASSERT_NE(canceled_result.result, nullptr);
  EXPECT_EQ(canceled_result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(canceled_result.result->result_code.value, Code::TASK_RESULT_CANCELED);
  expect_callbacks_cleared();

  auto after_cancel = run_pnp(action_lifecycle_pnp_goal());
  ASSERT_TRUE(after_cancel.has_value());
  ASSERT_NE(after_cancel->result, nullptr);
  EXPECT_EQ(after_cancel->code, rclcpp_action::ResultCode::SUCCEEDED);
  expect_callbacks_cleared();

  motion->set_next_behavior(LifecycleActionMotionPort::NextBehavior::THROW_PICK_APPROACH);
  auto exception_result = run_pnp(action_lifecycle_pnp_goal());
  ASSERT_TRUE(exception_result.has_value());
  ASSERT_NE(exception_result->result, nullptr);
  EXPECT_EQ(exception_result->code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(exception_result->result->pnp_outcome, PnpAction::Result::PNP_OUTCOME_FAILED);
  EXPECT_EQ(exception_result->result->result_code.value, Code::TASK_RESULT_BACKEND_ERROR);
  EXPECT_NE(exception_result->result->diagnostic_detail.find("lifecycle motion exception"),
    std::string::npos);
  EXPECT_FALSE(exception_result->result->material_placed);
  expect_callbacks_cleared();

  auto after_exception = run_pnp(action_lifecycle_pnp_goal());
  ASSERT_TRUE(after_exception.has_value());
  ASSERT_NE(after_exception->result, nullptr);
  EXPECT_EQ(after_exception->code, rclcpp_action::ResultCode::SUCCEEDED);
  expect_callbacks_cleared();

  motion->set_next_behavior(
    LifecycleActionMotionPort::NextBehavior::SAFETY_STOP_THEN_THROW_PICK_APPROACH);
  auto safety_exception = run_pnp(action_lifecycle_pnp_goal());
  ASSERT_TRUE(safety_exception.has_value());
  ASSERT_NE(safety_exception->result, nullptr);
  EXPECT_EQ(safety_exception->code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(safety_exception->result->result_code.value, Code::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(safety_exception->result->pnp_outcome,
    PnpAction::Result::PNP_OUTCOME_SAFETY_PREEMPTED);
  EXPECT_NE(safety_exception->result->diagnostic_detail.find("exception after Safety Stop"),
    std::string::npos);
  expect_callbacks_cleared();

  // Reset restores permission without rewriting the previous STOPPED observation.
  auto home_after_safety = run_legacy();
  ASSERT_TRUE(home_after_safety.has_value());
  ASSERT_NE(home_after_safety->result, nullptr);
  EXPECT_EQ(home_after_safety->result->result_code.value, Code::TASK_RESULT_SUCCESS);
  expect_callbacks_cleared();

  auto after_safety_exception = run_pnp(action_lifecycle_pnp_goal());
  ASSERT_TRUE(after_safety_exception.has_value());
  ASSERT_NE(after_safety_exception->result, nullptr);
  EXPECT_EQ(after_safety_exception->code, rclcpp_action::ResultCode::SUCCEEDED);
  expect_callbacks_cleared();

  // A later task must not rewrite the interrupted PICK's terminal result.
  EXPECT_EQ(safety_exception->result->result_code.value, Code::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(safety_exception->result->pnp_outcome,
    PnpAction::Result::PNP_OUTCOME_SAFETY_PREEMPTED);

  motion->set_next_behavior(LifecycleActionMotionPort::NextBehavior::THROW_PLACE_RETRACT);
  auto post_release_exception = run_pnp(action_lifecycle_pnp_goal());
  ASSERT_TRUE(post_release_exception.has_value());
  ASSERT_NE(post_release_exception->result, nullptr);
  EXPECT_EQ(post_release_exception->code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(post_release_exception->result->pnp_outcome, PnpAction::Result::PNP_OUTCOME_FAILED);
  EXPECT_EQ(post_release_exception->result->result_code.value, Code::TASK_RESULT_BACKEND_ERROR);
  EXPECT_TRUE(post_release_exception->result->material_placed);
  EXPECT_EQ(post_release_exception->result->object_disposition,
    Action::Result::OBJECT_DISPOSITION_PLACED);
  EXPECT_NE(post_release_exception->result->diagnostic_detail.find("post-release motion exception"),
    std::string::npos);
  expect_callbacks_cleared();

  auto legacy_after_exception = run_legacy();
  ASSERT_TRUE(legacy_after_exception.has_value());
  ASSERT_NE(legacy_after_exception->result, nullptr);
  EXPECT_EQ(legacy_after_exception->result->result_code.value, Code::TASK_RESULT_SUCCESS);
  expect_callbacks_cleared();

  motion->set_next_behavior(LifecycleActionMotionPort::NextBehavior::THROW_LEGACY_HOME);
  auto legacy_exception = run_legacy();
  ASSERT_TRUE(legacy_exception.has_value());
  ASSERT_NE(legacy_exception->result, nullptr);
  EXPECT_EQ(legacy_exception->code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(legacy_exception->result->result_code.value, Code::TASK_RESULT_BACKEND_ERROR);
  EXPECT_NE(legacy_exception->result->diagnostic_detail.find("legacy lifecycle motion exception"),
    std::string::npos);
  expect_callbacks_cleared();

  auto after_legacy_exception = run_pnp(action_lifecycle_pnp_goal());
  ASSERT_TRUE(after_legacy_exception.has_value());
  ASSERT_NE(after_legacy_exception->result, nullptr);
  EXPECT_EQ(after_legacy_exception->code, rclcpp_action::ResultCode::SUCCEEDED);
  expect_callbacks_cleared();
}

TEST(LegacyExecuteTaskActionServer, RecoveryHomeUsesMotionCoreAdmissionAfterPreviousStop)
{
  using Status = arm_cell_interfaces::msg::MotionStatus;
  using Capability = arm_cell_interfaces::msg::MotionCapability;
  using Safety = arm_cell_interfaces::msg::SafetyState;
  using Type = arm_cell_interfaces::msg::MotionTaskType;
  struct Scenario
  {
    const char * name;
    bool refresh_stop;
    bool backend_inactive;
    std::uint8_t capability;
    std::uint8_t expected;
  };
  for (const auto scenario : {
      Scenario{"stopping", false, true, Capability::MOTION_NORMAL, Code::TASK_RESULT_BUSY},
      Scenario{"stopped_unconfirmed", true, false, Capability::MOTION_NORMAL,
        Code::TASK_RESULT_BUSY},
      Scenario{"reset_complete", true, true, Capability::MOTION_NORMAL,
        Code::TASK_RESULT_SUCCESS},
      Scenario{"no_permission", true, true, Capability::MOTION_NONE,
        Code::TASK_RESULT_PERMISSION_DENIED},
      Scenario{"recovery_only", true, true, Capability::MOTION_RECOVERY_ONLY,
        Code::TASK_RESULT_PERMISSION_DENIED}})
  {
    SCOPED_TRACE(scenario.name);
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    auto node = std::make_shared<rclcpp::Node>("recovery_home_admission_test");
    auto backend = std::make_shared<arm_cell_motion::FakeMotionBackend>();
    auto motion = std::make_shared<arm_cell_motion::MotionCore>(
      backend, std::chrono::seconds(30));
    Safety safety;
    safety.valid = true;
    safety.required_inputs_fresh = true;
    safety.safety_state = Safety::SAFETY_STATE_SAFE;
    safety.motion_capability.value = Capability::MOTION_NORMAL;
    safety.motion_envelope_valid = true;
    safety.max_velocity_scale = 1.0F;
    safety.max_acceleration_scale = 1.0F;
    motion->update_safety_state(safety, arm_cell_motion::MotionCore::Clock::now());
    ASSERT_EQ(motion->executeLegacyMovement(Type::TASK_TYPE_GO_HOME,
      "01000000000000000000000000000000", "previous-movement"), Code::TASK_RESULT_SUCCESS);
    unique_identifier_msgs::msg::UUID stop_id;
    stop_id.uuid[0] = 2;
    ASSERT_TRUE(motion->request_stop(stop_id,
      arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY));
    backend->set_execution_active(false);
    backend->set_inactivity_confirmed(true);
    if (scenario.refresh_stop) {motion->refresh_stop_state();}
    ASSERT_EQ(motion->status().execution_state,
      scenario.refresh_stop ? Status::MOTION_STATE_STOPPED : Status::MOTION_STATE_STOPPING);
    // Backend evidence may be lost after the last STOPPED publication.
    backend->set_inactivity_confirmed(scenario.backend_inactive);
    safety.motion_capability.value = scenario.capability;
    motion->update_safety_state(safety, arm_cell_motion::MotionCore::Clock::now());
    auto gripper = backend->gripper_port();
    ASSERT_EQ(gripper->open(), Code::TASK_RESULT_SUCCESS);
    arm_cell_motion::MotionGeometry geometry;
    auto task_executor = std::make_shared<arm_cell_task_execution::TaskExecutor>(
      motion, gripper, geometry,
      arm_cell_task_execution::TaskExecutor::InterruptionChecker{}, std::chrono::milliseconds(10));
    arm_cell_task_execution::LegacyExecuteTaskActionServer server(node.get(), motion, task_executor);
    auto client = rclcpp_action::create_client<Action>(node, "/motion/execute_task");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() {executor.spin();});
    struct SpinCleanup
    {
      rclcpp::executors::MultiThreadedExecutor & executor;
      std::thread & thread;
      ~SpinCleanup() {executor.cancel(); thread.join(); rclcpp::shutdown();}
    } cleanup{executor, spin_thread};
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
    const auto backend_calls = backend->execute_task_calls();
    Action::Goal home;
    home.task_type.value = Type::TASK_TYPE_GO_HOME;
    auto goal_future = client->async_send_goal(home);
    ASSERT_EQ(goal_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
    auto handle = goal_future.get();
    ASSERT_NE(handle, nullptr);
    auto result_future = client->async_get_result(handle);
    ASSERT_EQ(result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
    const auto result = result_future.get();
    ASSERT_NE(result.result, nullptr);
    EXPECT_EQ(result.result->result_code.value, scenario.expected);
    EXPECT_EQ(backend->execute_task_calls() - backend_calls,
      scenario.expected == Code::TASK_RESULT_SUCCESS ? 1 : 0);
  }
}

TEST(ExecutePnpActionServer, ResetAndNewHomePreserveInterruptedPickResult)
{
  using Safety = arm_cell_interfaces::msg::SafetyState;
  using Capability = arm_cell_interfaces::msg::MotionCapability;
  using Status = arm_cell_interfaces::msg::MotionStatus;
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp::Node>("pnp_reset_home_test");
  auto backend = std::make_shared<arm_cell_motion::FakeMotionBackend>();
  backend->set_block_execution(true);
  backend->set_defer_terminal_return(true);
  const auto geometry = action_lifecycle_geometry();
  auto motion = std::make_shared<arm_cell_motion::MotionCore>(
    backend, std::chrono::seconds(30), geometry);
  Safety safety;
  safety.valid = true;
  safety.required_inputs_fresh = true;
  safety.safety_state = Safety::SAFETY_STATE_SAFE;
  safety.motion_capability.value = Capability::MOTION_NORMAL;
  safety.motion_envelope_valid = true;
  safety.max_velocity_scale = 1.0F;
  safety.max_acceleration_scale = 1.0F;
  motion->update_safety_state(safety, arm_cell_motion::MotionCore::Clock::now());
  auto task_executor = std::make_shared<arm_cell_task_execution::TaskExecutor>(
    motion, backend->gripper_port(), geometry);
  arm_cell_task_execution::ExecutePnpActionServer pnp_server(node.get(), motion, task_executor);
  arm_cell_task_execution::LegacyExecuteTaskActionServer home_server(node.get(), motion, task_executor);
  auto pnp_client = rclcpp_action::create_client<PnpAction>(node, "/task_execution/execute_pnp");
  auto home_client = rclcpp_action::create_client<Action>(node, "/motion/execute_task");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct SpinCleanup
  {
    rclcpp::executors::MultiThreadedExecutor & executor;
    std::thread & thread;
    std::shared_ptr<arm_cell_motion::FakeMotionBackend> backend;
    ~SpinCleanup()
    {
      backend->release_execution();
      backend->release_terminal_return();
      executor.cancel();
      thread.join();
      rclcpp::shutdown();
    }
  } cleanup{executor, spin_thread, backend};
  ASSERT_TRUE(pnp_client->wait_for_action_server(std::chrono::seconds(2)));
  ASSERT_TRUE(home_client->wait_for_action_server(std::chrono::seconds(2)));
  auto pick_future = pnp_client->async_send_goal(action_lifecycle_pnp_goal());
  ASSERT_EQ(pick_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto pick_handle = pick_future.get();
  ASSERT_NE(pick_handle, nullptr);
  ASSERT_TRUE(backend->wait_until_execution_started());
  safety.motion_capability.value = Capability::MOTION_NONE;
  safety.safety_state = Safety::SAFETY_STATE_EMERGENCY_STOPPED;
  motion->update_safety_state(safety, arm_cell_motion::MotionCore::Clock::now());
  unique_identifier_msgs::msg::UUID stop_id;
  stop_id.uuid[0] = 3;
  ASSERT_TRUE(motion->request_stop(stop_id,
    arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY));
  ASSERT_TRUE(backend->wait_until_interruption_ready());
  backend->set_execution_active(false);
  backend->set_inactivity_confirmed(true);
  backend->release_terminal_return();
  auto pick_result_future = pnp_client->async_get_result(pick_handle);
  ASSERT_EQ(pick_result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto pick_result = pick_result_future.get();
  ASSERT_NE(pick_result.result, nullptr);
  EXPECT_EQ(pick_result.result->result_code.value, Code::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(pick_result.result->pnp_outcome, PnpAction::Result::PNP_OUTCOME_SAFETY_PREEMPTED);
  motion->refresh_stop_state();
  ASSERT_EQ(motion->status().execution_state, Status::MOTION_STATE_STOPPED);

  // Model the restored Safety authority after operator Reset; do not rewrite Motion state.
  safety.safety_state = Safety::SAFETY_STATE_SAFE;
  safety.motion_capability.value = Capability::MOTION_NORMAL;
  motion->update_safety_state(safety, arm_cell_motion::MotionCore::Clock::now());
  ASSERT_EQ(backend->gripper_port()->open(), Code::TASK_RESULT_SUCCESS);
  ASSERT_EQ(motion->status().holding_state, Status::HOLDING_RELEASED);
  Action::Goal home;
  home.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;
  auto home_future = home_client->async_send_goal(home);
  ASSERT_EQ(home_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  auto home_handle = home_future.get();
  ASSERT_NE(home_handle, nullptr);
  auto home_result_future = home_client->async_get_result(home_handle);
  ASSERT_EQ(home_result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto home_result = home_result_future.get();
  ASSERT_NE(home_result.result, nullptr);
  EXPECT_EQ(home_result.result->result_code.value, Code::TASK_RESULT_SUCCESS);

  // Read the original Action result from the server again after the new HOME.
  using GetResult = PnpAction::Impl::GetResultService;
  auto result_client = node->create_client<GetResult>(
    "/task_execution/execute_pnp/_action/get_result");
  ASSERT_TRUE(result_client->wait_for_service(std::chrono::seconds(2)));
  auto request = std::make_shared<GetResult::Request>();
  request->goal_id.uuid = pick_handle->get_goal_id();
  auto old_result_future = result_client->async_send_request(request);
  ASSERT_EQ(old_result_future.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto old_result = old_result_future.get();
  EXPECT_EQ(old_result->result.result_code.value, Code::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_EQ(old_result->result.pnp_outcome, PnpAction::Result::PNP_OUTCOME_SAFETY_PREEMPTED);
  EXPECT_FALSE(old_result->result.material_placed);
}
