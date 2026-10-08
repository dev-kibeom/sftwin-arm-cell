#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <future>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <rcutils/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arm_cell_interfaces/action/execute_task.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#define private public
#include "arm_cell_motion_moveit2/isaac_ros_motion_transport.hpp"
#undef private

namespace arm_cell_motion_moveit2
{
namespace
{
using JointState = sensor_msgs::msg::JointState;
using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
using MotionTaskType = arm_cell_interfaces::msg::MotionTaskType;
using String = std_msgs::msg::String;
using MoveGroupAction = moveit_msgs::action::MoveGroup;

struct CapturedLog
{
  int severity;
  std::string format;
};

std::vector<CapturedLog> captured_logs;

void capture_log(
  const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
  const char * format, va_list * arguments)
{
  std::array<char, 1024> message{};
  va_list copied_arguments;
  va_copy(copied_arguments, *arguments);
  std::vsnprintf(message.data(), message.size(), format ? format : "", copied_arguments);
  va_end(copied_arguments);
  captured_logs.push_back(CapturedLog{severity, message.data()});
}

class IsaacRosMotionTransportTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>(
      "isaac_ros_motion_transport_test", rclcpp::NodeOptions().use_global_arguments(false));
    observer_ = std::make_shared<rclcpp::Node>(
      "isaac_ros_motion_transport_observer", rclcpp::NodeOptions().use_global_arguments(false));
    feedback_publisher_ = node_->create_publisher<JointState>(
      "/isaac/joint_states", rclcpp::SensorDataQoS());
    status_publisher_ = observer_->create_publisher<String>("/gripper/status", 10);
    command_subscription_ = observer_->create_subscription<JointState>(
      "/joint_commands", rclcpp::QoS(10).reliable().durability_volatile(),
      [this](const JointState::SharedPtr message) {
        std::size_t command_count = 0;
        std::lock_guard<std::mutex> lock(command_mutex_);
        commands_.push_back(*message);
        command_count = commands_.size();
        if (simulate_feedback_after_command_count_ > 0 &&
        command_count >= simulate_feedback_after_command_count_)
        {
          feedback_publisher_->publish(*message);
        }
      });
    executor_.add_node(node_);
    executor_.add_node(observer_);
    transport_ = std::make_unique<IsaacRosMotionTransport>(
      node_, arm_joint_names(), std::vector<double>(6, 0.1), std::vector<double>(6, -0.1), false);
  }

  void TearDown() override
  {
    transport_.reset();
    executor_.remove_node(observer_);
    executor_.remove_node(node_);
    observer_.reset();
    node_.reset();
  }

  static std::vector<std::string> arm_joint_names()
  {
    return {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  }

  void publish_feedback(const std::vector<double> & positions, bool complete = true)
  {
    JointState message;
    message.name = complete ? arm_joint_names() : std::vector<std::string>{"joint_1"};
    message.position = complete ? positions : std::vector<double>{positions.front()};
    feedback_publisher_->publish(message);
    if (complete) {
      pump_until([this]() {return transport_->available();});
    } else {
      for (int index = 0; index < 20; ++index) {
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }

  void pump_until(const std::function<bool()> & predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    executor_.spin_some();
    ASSERT_TRUE(predicate());
  }

  void publish_status(
    bool ready, bool grasp, bool attached, bool released, std::uint64_t sequence)
  {
    publish_status_payload(
      "ready=" + std::to_string(ready ? 1 : 0) +
      ";grasp=" + std::to_string(grasp ? 1 : 0) +
      ";attached=" + std::to_string(attached ? 1 : 0) +
      ";released=" + std::to_string(released ? 1 : 0) +
      ";width_mm=12.500;seq=" + std::to_string(sequence));
  }

  void publish_status_payload(const std::string & payload)
  {
    String message;
    message.data = payload;
    status_publisher_->publish(message);
    executor_.spin_some();
  }

  std::size_t command_count() const
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return commands_.size();
  }

  std::vector<double> last_command() const
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return commands_.back().position;
  }

  JointState last_command_message() const
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return commands_.back();
  }

  std::vector<JointState> command_messages() const
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return commands_;
  }

  rosgraph_msgs::msg::Clock::SharedPtr clock_message(double simulation_time) const
  {
    auto message = std::make_shared<rosgraph_msgs::msg::Clock>();
    message->clock.sec = static_cast<std::int32_t>(simulation_time);
    message->clock.nanosec = static_cast<std::uint32_t>(
      (simulation_time - static_cast<double>(message->clock.sec)) * 1e9);
    return message;
  }

  void start_approach_final_hold(
    const std::vector<double> & final_target,
    const std::vector<double> & actual_positions)
  {
    publish_feedback(std::vector<double>(6, 0.0));

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = arm_joint_names();
    trajectory_msgs::msg::JointTrajectoryPoint start;
    start.positions = std::vector<double>(6, 0.0);
    start.time_from_start.sec = 0;
    trajectory.points.push_back(start);
    trajectory_msgs::msg::JointTrajectoryPoint finish;
    finish.positions = final_target;
    finish.time_from_start.sec = 1;
    trajectory.points.push_back(finish);

    transport_->timed_trajectory_active_ = true;
    transport_->active_phase_ =
      arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
    transport_->execution_active_ = true;
    transport_->stop_requested_ = false;
    transport_->approach_reference_completion_observed_ = false;
    transport_->has_simulation_time_ = true;
    transport_->latest_simulation_time_ = 10.0;
    transport_->trajectory_watchdog_.start(
      10.0, 1.0, std::chrono::steady_clock::now());
    transport_->trajectory_worker_->set_velocity_limits(std::vector<double>(6, 1.0));
    ASSERT_TRUE(transport_->trajectory_worker_->start(trajectory, 1, 10.0).valid);

    transport_->on_simulation_clock(clock_message(10.0));
    executor_.spin_some();
    transport_->on_simulation_clock(clock_message(11.0));
    executor_.spin_some();
    publish_feedback(actual_positions);
    ASSERT_TRUE(transport_->final_target_hold_.active());
  }

  void clear_commands()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    commands_.clear();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::Publisher<JointState>::SharedPtr feedback_publisher_;
  rclcpp::Publisher<String>::SharedPtr status_publisher_;
  rclcpp::Subscription<JointState>::SharedPtr command_subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<IsaacRosMotionTransport> transport_;
  mutable std::mutex command_mutex_;
  std::vector<JointState> commands_;
  std::size_t simulate_feedback_after_command_count_{0};
};

TEST_F(IsaacRosMotionTransportTest, HoldRequiresMeasuredSettlingAfterHoldCommand)
{
  const std::vector<double> measured(6, 0.0);
  publish_feedback(measured);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  clear_commands();

  transport_->hold();
  ASSERT_FALSE(transport_->inactivity_confirmed());
  pump_until([this]() {return command_count() > 0;});
  EXPECT_EQ(last_command(), measured);

  publish_feedback(measured, false);
  EXPECT_FALSE(transport_->inactivity_confirmed());
  publish_feedback(measured);
  EXPECT_FALSE(transport_->inactivity_confirmed());
  publish_feedback(measured);
  EXPECT_TRUE(transport_->inactivity_confirmed());
}

TEST_F(IsaacRosMotionTransportTest, JointCommandPublisherUsesReliableVolatileQoS)
{
  const auto publisher_info = node_->get_publishers_info_by_topic("/joint_commands");
  ASSERT_FALSE(publisher_info.empty());
  const auto matching_publisher = std::find_if(
    publisher_info.begin(), publisher_info.end(), [](const auto & info) {
      return info.qos_profile().get_rmw_qos_profile().reliability ==
      RMW_QOS_POLICY_RELIABILITY_RELIABLE &&
      info.qos_profile().get_rmw_qos_profile().durability ==
      RMW_QOS_POLICY_DURABILITY_VOLATILE;
    });
  ASSERT_NE(matching_publisher, publisher_info.end());
}

TEST_F(IsaacRosMotionTransportTest, AvailabilityReturnsPromptlyBeforeInitialization)
{
  auto availability = std::async(
    std::launch::async, [this]() {return transport_->available();});
  ASSERT_EQ(
    availability.wait_for(std::chrono::milliseconds(100)), std::future_status::ready);
  EXPECT_FALSE(availability.get());
  const auto diagnostic = transport_->availability_diagnostic();
  EXPECT_NE(diagnostic.find("move_group_ready=true"), std::string::npos);
  EXPECT_NE(diagnostic.find("init_in_progress=false"), std::string::npos);
  EXPECT_NE(diagnostic.find("transport_ready=false"), std::string::npos);
}

TEST_F(IsaacRosMotionTransportTest, MoveGroupInitializationReachesTerminalState)
{
  node_->declare_parameter<std::string>(
    "robot_description",
    "<robot name=\"test\"><link name=\"base_link\"/><link name=\"link_1\"/>"
    "<joint name=\"joint_1\" type=\"revolute\"><parent link=\"base_link\"/>"
    "<child link=\"link_1\"/><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
    "<axis xyz=\"0 0 1\"/><limit lower=\"-3.14\" upper=\"3.14\""
    " effort=\"1\" velocity=\"1\"/></joint></robot>");
  node_->declare_parameter<std::string>(
    "robot_description_semantic",
    "<robot name=\"test\"><group name=\"arm\"><joint name=\"joint_1\"/>"
    "</group></robot>");
  const auto started = std::chrono::steady_clock::now();
  auto transport = std::make_unique<IsaacRosMotionTransport>(
    node_, arm_joint_names(), std::vector<double>(6, 0.1), std::vector<double>(6, -0.1), true);

  bool terminal = false;
  const auto deadline = started + std::chrono::seconds(8);
  while (std::chrono::steady_clock::now() < deadline) {
    executor_.spin_some();
    const auto diagnostic = transport->availability_diagnostic();
    if (diagnostic.find("init_in_progress=false") != std::string::npos) {
      terminal = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(terminal) << transport->availability_diagnostic();
  const auto destruction_started = std::chrono::steady_clock::now();
  transport.reset();
  EXPECT_LT(
    std::chrono::steady_clock::now() - destruction_started,
    std::chrono::seconds(1));
}

TEST_F(IsaacRosMotionTransportTest, PlanningOnlyReadinessDoesNotRequireExecuteTrajectory)
{
  node_->declare_parameter<std::string>(
    "robot_description",
    "<robot name=\"test\"><link name=\"base_link\"/><link name=\"link_1\"/>"
    "<joint name=\"joint_1\" type=\"revolute\"><parent link=\"base_link\"/>"
    "<child link=\"link_1\"/><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
    "<axis xyz=\"0 0 1\"/><limit lower=\"-3.14\" upper=\"3.14\""
    " effort=\"1\" velocity=\"1\"/></joint></robot>");
  node_->declare_parameter<std::string>(
    "robot_description_semantic",
    "<robot name=\"test\"><group name=\"arm\"><joint name=\"joint_1\"/>"
    "</group></robot>");
  auto planning_server = rclcpp_action::create_server<MoveGroupAction>(
    node_, "move_action",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveGroupAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroupAction>>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroupAction>>) {});

  auto transport = std::make_unique<IsaacRosMotionTransport>(
    node_, arm_joint_names(), std::vector<double>(6, 0.1), std::vector<double>(6, -0.1), true);
  bool ready = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (std::chrono::steady_clock::now() < deadline) {
    executor_.spin_some();
    const auto diagnostic = transport->availability_diagnostic();
    if (diagnostic.find("move_group_ready=true") != std::string::npos &&
      diagnostic.find("init_in_progress=false") != std::string::npos)
    {
      ready = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  ASSERT_TRUE(ready) << transport->availability_diagnostic();
  transport.reset();
  EXPECT_TRUE(planning_server);
}

TEST_F(IsaacRosMotionTransportTest, CancelUsesIsaacCommandPathAndNeedsFeedback)
{
  const std::vector<double> measured(6, 0.0);
  publish_feedback(measured);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  clear_commands();

  transport_->cancel();
  EXPECT_FALSE(transport_->inactivity_confirmed());
  pump_until([this]() {return command_count() > 0;});
}

TEST_F(IsaacRosMotionTransportTest, DirectMotionWaitsForMeasuredCompletion)
{
  publish_feedback(std::vector<double>(6, 0.0));
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  pump_until([this]() {return command_count() > 0;});
  EXPECT_TRUE(last_command_message().velocity.empty());

  auto completion = std::async(
    std::launch::async, [this]() {return transport_->wait_for_motion_completion();});
  EXPECT_EQ(
    completion.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);

  publish_feedback(std::vector<double>(6, 0.1));
  EXPECT_EQ(
    completion.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  publish_feedback(std::vector<double>(6, 0.1));
  ASSERT_EQ(
    completion.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::COMPLETED);
}

TEST_F(IsaacRosMotionTransportTest, DirectMotionProgressingBeyondOldDeadlineCanComplete)
{
  transport_.reset();
  transport_ = std::make_unique<IsaacRosMotionTransport>(
    node_, arm_joint_names(), std::vector<double>(6, 0.1), std::vector<double>(6, -0.1),
    false, false, "", std::vector<std::string>{}, 0.0, "linear", 2.0,
    0.005, 0.08726646259971647, true, 0.001, 3, 0.25);
  publish_feedback(std::vector<double>(6, 0.0));

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));

  auto completion = std::async(
    std::launch::async, [this]() {return transport_->wait_for_motion_completion();});
  for (int index = 1; index <= 6; ++index) {
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    publish_feedback(std::vector<double>(6, index / 60.0));
  }
  publish_feedback(std::vector<double>(6, 0.1));
  publish_feedback(std::vector<double>(6, 0.1));

  ASSERT_EQ(completion.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::COMPLETED);
}

TEST_F(IsaacRosMotionTransportTest, DirectMotionFailsWhenFreshFeedbackStopsProgressing)
{
  transport_.reset();
  transport_ = std::make_unique<IsaacRosMotionTransport>(
    node_, arm_joint_names(), std::vector<double>(6, 0.1), std::vector<double>(6, -0.1),
    false, false, "", std::vector<std::string>{}, 0.0, "linear", 2.0,
    0.005, 0.08726646259971647, true, 0.001, 3, 0.25);
  publish_feedback(std::vector<double>(6, 0.0));

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  const auto initial_command_count = command_count();

  auto completion = std::async(
    std::launch::async, [this]() {return transport_->wait_for_motion_completion();});
  for (int index = 0; index < 8 &&
    completion.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready; ++index)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    publish_feedback(std::vector<double>(6, 0.0));
  }

  ASSERT_EQ(completion.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::TIMEOUT);
  pump_until([this, initial_command_count]() {
    return command_count() > initial_command_count &&
           last_command() == std::vector<double>(6, 0.0);
  });
  EXPECT_EQ(last_command(), std::vector<double>(6, 0.0));
}

TEST_F(IsaacRosMotionTransportTest, RepeatsJointTargetUntilIsaacFeedbackArrives)
{
  publish_feedback(std::vector<double>(6, 0.0));
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));

  simulate_feedback_after_command_count_ = 2;
  std::atomic_bool spinning{true};
  std::thread spinner([this, &spinning]() {
      while (spinning.load()) {
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });

  EXPECT_EQ(
    transport_->wait_for_motion_completion(), MotionCompletionOutcome::COMPLETED);
  spinning.store(false);
  spinner.join();
  EXPECT_GE(command_count(), 2u);
}

TEST_F(IsaacRosMotionTransportTest, StopInterruptsMeasuredMotionWait)
{
  publish_feedback(std::vector<double>(6, 0.0));
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));

  auto completion = std::async(
    std::launch::async, [this]() {return transport_->wait_for_motion_completion();});
  EXPECT_EQ(
    completion.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);

  transport_->stop();
  ASSERT_EQ(
    completion.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::SAFETY_PREEMPTED);
}

TEST_F(IsaacRosMotionTransportTest, MissingMeasuredCompletionFailsClosed)
{
  publish_feedback(std::vector<double>(6, 0.0));
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));

  auto completion = std::async(
    std::launch::async, [this]() {return transport_->wait_for_motion_completion();});
  ASSERT_EQ(
    completion.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::STALE_FEEDBACK);
}

TEST_F(IsaacRosMotionTransportTest, EmergencyStopDoesNotConfirmWithoutFreshFeedback)
{
  const std::vector<double> measured(6, 0.0);
  publish_feedback(measured);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  clear_commands();

  transport_->stop();
  EXPECT_FALSE(transport_->inactivity_confirmed());
  pump_until([this]() {return command_count() > 0;});
}

TEST_F(IsaacRosMotionTransportTest, HoldingObservationUsesFreshHeldStatus)
{
  publish_feedback(std::vector<double>(6, 0.0));
  publish_status(true, true, true, false, 1);

  const auto observation = transport_->holding_observation();
  EXPECT_EQ(observation.state, HoldingState::HELD);
  EXPECT_TRUE(observation.fresh());
  EXPECT_EQ(observation.sequence, 1u);
}

TEST_F(IsaacRosMotionTransportTest, HoldingObservationUsesFreshReleasedStatus)
{
  publish_feedback(std::vector<double>(6, 0.0));
  publish_status(true, false, false, true, 1);

  const auto observation = transport_->holding_observation();
  EXPECT_EQ(observation.state, HoldingState::RELEASED);
  EXPECT_TRUE(observation.fresh());
}

TEST_F(IsaacRosMotionTransportTest, SameSequenceRefreshesWidthAndRejectsSemanticReplay)
{
  publish_feedback(std::vector<double>(6, 0.0));
  publish_status(true, false, false, true, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(325));
  publish_status_payload(
    "ready=1;grasp=0;attached=0;released=1;width_mm=13.500;seq=1");
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  auto observation = transport_->holding_observation();
  EXPECT_EQ(observation.state, HoldingState::RELEASED);
  EXPECT_EQ(observation.sequence, 1u);
  EXPECT_TRUE(observation.fresh());
  EXPECT_DOUBLE_EQ(transport_->gripper_width_mm_, 13.5);

  publish_status_payload(
    "ready=1;grasp=1;attached=1;released=0;width_mm=13.500;seq=1");
  observation = transport_->holding_observation();
  EXPECT_EQ(observation.state, HoldingState::RELEASED);
  EXPECT_EQ(observation.sequence, 1u);

  publish_status(true, true, true, false, 2);
  observation = transport_->holding_observation();
  EXPECT_EQ(observation.state, HoldingState::HELD);
  EXPECT_EQ(observation.sequence, 2u);
  EXPECT_TRUE(observation.fresh());
}

TEST_F(IsaacRosMotionTransportTest, GripperStatusLogsOnlySemanticChangesOrRejectedReceipts)
{
  EXPECT_EQ(
    rcutils_logging_set_logger_level(
      node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG), RCUTILS_RET_OK);
  const auto previous_handler = rcutils_logging_get_output_handler();
  captured_logs.clear();
  rcutils_logging_set_output_handler(capture_log);

  publish_status(true, true, true, false, 7);
  publish_status_payload(
    "ready=1;grasp=1;attached=1;released=0;width_mm=12.510;seq=8");
  publish_status_payload(
    "ready=1;grasp=1;attached=1;released=0;width_mm=12.520;seq=9");
  publish_status(true, false, false, true, 10);
  publish_status_payload(
    "ready=1;grasp=0;attached=0;released=1;width_mm=12.490;seq=11");
  publish_status(true, false, false, true, 9);
  publish_status(true, true, true, false, 11);
  publish_status_payload("malformed");
  publish_status_payload("ready=1;grasp=0;attached=0;released=1;width_mm=12.5");
  publish_status_payload(
    "ready=1;grasp=0;attached=0;released=1;width_mm=nan;seq=12");

  rcutils_logging_set_output_handler(previous_handler);
  EXPECT_EQ(
    rcutils_logging_set_logger_level(
      node_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO), RCUTILS_RET_OK);
  EXPECT_EQ(
    std::count_if(
      captured_logs.begin(), captured_logs.end(), [](const CapturedLog & log) {
        return log.severity == RCUTILS_LOG_SEVERITY_DEBUG;
      }), 2);
  EXPECT_EQ(
    std::count_if(
      captured_logs.begin(), captured_logs.end(), [](const CapturedLog & log) {
        return log.severity == RCUTILS_LOG_SEVERITY_WARN;
      }), 5);
  const auto has_diagnostic = [](const std::string & text) {
      return std::any_of(
        captured_logs.begin(), captured_logs.end(), [&text](const CapturedLog & log) {
          return log.format.find(text) != std::string::npos;
        });
    };
  EXPECT_TRUE(has_diagnostic("reason=stale_sequence"));
  EXPECT_TRUE(has_diagnostic("reason=sequence_reused_with_different_state"));
  EXPECT_TRUE(has_diagnostic("reason=malformed_payload"));
  EXPECT_TRUE(has_diagnostic("reason=missing_field"));
  EXPECT_TRUE(has_diagnostic("reason=non_finite_width"));
  EXPECT_TRUE(std::all_of(
      captured_logs.begin(), captured_logs.end(), [](const CapturedLog & log) {
        return log.severity != RCUTILS_LOG_SEVERITY_INFO;
      }));
  EXPECT_EQ(transport_->holding_observation().state, HoldingState::RELEASED);
  EXPECT_EQ(transport_->holding_observation().sequence, 11u);
  EXPECT_TRUE(transport_->holding_observation().fresh());
}

TEST_F(IsaacRosMotionTransportTest, GraspRelationRequiresFreshHeldAndExpiresOnRelease)
{
  transport_->begin_pick_grasp_relation();

  NormalizedMotionRequest place_request;
  place_request.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  place_request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
  EXPECT_FALSE(transport_->submit_normalized(place_request));

  geometry_msgs::msg::Transform selected_relation;
  selected_relation.rotation.w = 1.0;
  transport_->stage_selected_pick_grasp_relation(selected_relation);
  EXPECT_FALSE(transport_->submit_normalized(place_request));
  EXPECT_TRUE(transport_->confirm_fresh_held_grasp_relation());
  EXPECT_FALSE(transport_->confirm_fresh_held_grasp_relation());

  transport_->confirm_fresh_released_grasp_relation();
  EXPECT_FALSE(transport_->confirm_fresh_held_grasp_relation());
  EXPECT_FALSE(transport_->submit_normalized(place_request));
}

TEST_F(IsaacRosMotionTransportTest, GripperCommandDoesNotInventHoldingState)
{
  publish_feedback(std::vector<double>(6, 0.0));
  publish_status(true, false, false, false, 1);
  ASSERT_TRUE(transport_->command_normalized_gripper(0.0));

  const auto observation = transport_->holding_observation();
  EXPECT_EQ(observation.state, HoldingState::UNKNOWN);
  EXPECT_FALSE(observation.fresh());
}

TEST_F(IsaacRosMotionTransportTest, UnreadyRuntimeRejectsGripperCommand)
{
  publish_feedback(std::vector<double>(6, 0.0));
  publish_status(false, false, false, false, 1);

  EXPECT_FALSE(transport_->command_normalized_gripper(0.0));
  EXPECT_EQ(transport_->holding_observation().state, HoldingState::UNKNOWN);
}

TEST_F(IsaacRosMotionTransportTest, RejectedGlobalProfileStartDoesNotBlockLaterCapture)
{
  trajectory_msgs::msg::JointTrajectory rejected;
  const auto failed = transport_->start_global_profiled_trajectory(
    rejected, 1, 0.0, "test_profile");
  EXPECT_FALSE(failed.valid);
  EXPECT_FALSE(transport_->global_profile_capture_complete_);
  EXPECT_FALSE(transport_->global_profile_trace_active_);

  trajectory_msgs::msg::JointTrajectory valid;
  valid.joint_names = arm_joint_names();
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = std::vector<double>(6, 0.0);
  valid.points.push_back(start);
  trajectory_msgs::msg::JointTrajectoryPoint finish;
  finish.positions = std::vector<double>(6, 0.1);
  finish.time_from_start.sec = 1;
  valid.points.push_back(finish);

  const auto started = transport_->start_global_profiled_trajectory(
    valid, 2, 0.0, "test_profile");
  ASSERT_TRUE(started.valid) << started.reason;
  EXPECT_TRUE(transport_->global_profile_trace_active_);
  EXPECT_FALSE(transport_->global_profile_capture_complete_);

  transport_->finish_global_profile_capture("trajectory_execution_complete", true);
  EXPECT_FALSE(transport_->global_profile_trace_active_);
  EXPECT_TRUE(transport_->global_profile_capture_complete_);
}

TEST_F(IsaacRosMotionTransportTest, CancelledGlobalProfileCaptureCanBeRetried)
{
  publish_feedback(std::vector<double>(6, 0.0));
  trajectory_msgs::msg::JointTrajectory valid;
  valid.joint_names = arm_joint_names();
  trajectory_msgs::msg::JointTrajectoryPoint start;
  start.positions = std::vector<double>(6, 0.0);
  valid.points.push_back(start);
  trajectory_msgs::msg::JointTrajectoryPoint finish;
  finish.positions = std::vector<double>(6, 0.1);
  finish.time_from_start.sec = 1;
  valid.points.push_back(finish);

  ASSERT_TRUE(
    transport_->start_global_profiled_trajectory(
      valid, 1, 0.0, "test_profile").valid);
  transport_->cancel();
  EXPECT_FALSE(transport_->global_profile_trace_active_);
  EXPECT_FALSE(transport_->global_profile_capture_complete_);

  ASSERT_TRUE(
    transport_->start_global_profiled_trajectory(
      valid, 2, 0.0, "test_profile").valid);
  EXPECT_TRUE(transport_->global_profile_trace_active_);
  transport_->finish_global_profile_capture("trajectory_execution_complete", true);
  EXPECT_TRUE(transport_->global_profile_capture_complete_);
}

TEST_F(IsaacRosMotionTransportTest, MissingOrStaleGripperStatusFailsClosed)
{
  publish_feedback(std::vector<double>(6, 0.0));
  EXPECT_FALSE(transport_->command_normalized_gripper(0.0));
  EXPECT_EQ(transport_->holding_observation().state, HoldingState::UNKNOWN);

  publish_status(true, true, true, false, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(550));
  EXPECT_EQ(transport_->holding_observation().state, HoldingState::UNKNOWN);
}

TEST_F(IsaacRosMotionTransportTest, TimedApproachRepeatsFinalTargetWithZeroVelocity)
{
  const std::vector<double> final_target(6, 0.2);
  start_approach_final_hold(final_target, std::vector<double>(6, 0.1));
  const auto entered = last_command_message();
  ASSERT_EQ(entered.position, final_target);
  ASSERT_EQ(entered.velocity, std::vector<double>(6, 0.0));

  const auto count_before = command_count();
  transport_->on_simulation_clock(clock_message(11.25));
  executor_.spin_some();
  transport_->on_simulation_clock(clock_message(11.5));
  executor_.spin_some();
  EXPECT_EQ(command_count(), count_before + 2);
  const auto messages = command_messages();
  ASSERT_GE(messages.size(), count_before + 2);
  EXPECT_EQ(messages[count_before].position, final_target);
  EXPECT_EQ(messages[count_before].velocity, std::vector<double>(6, 0.0));
  EXPECT_EQ(messages[count_before + 1].position, final_target);
  EXPECT_EQ(messages[count_before + 1].velocity, std::vector<double>(6, 0.0));
  EXPECT_TRUE(transport_->final_target_hold_.active());
}

TEST_F(IsaacRosMotionTransportTest, TimedApproachCanSettleAfterPlannedDuration)
{
  const std::vector<double> final_target(6, 0.2);
  const std::vector<double> actual(6, 0.1);
  start_approach_final_hold(final_target, actual);

  auto completion = std::async(
    std::launch::async, [this]() {return transport_->wait_for_motion_completion();});
  for (int index = 1; index <= 6; ++index) {
    transport_->on_simulation_clock(clock_message(11.0 + index));
    executor_.spin_some();
    publish_feedback(std::vector<double>(6, 0.1 + index / 60.0));
  }
  EXPECT_EQ(
    completion.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  EXPECT_TRUE(transport_->final_target_hold_.active());

  transport_->cancel();
  ASSERT_EQ(completion.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::CANCELED);
  EXPECT_FALSE(transport_->final_target_hold_.active());
  EXPECT_EQ(last_command(), std::vector<double>(6, 0.2));
}

TEST_F(IsaacRosMotionTransportTest, CancelRevokesTimedFinalTargetHold)
{
  const std::vector<double> final_target(6, 0.2);
  const std::vector<double> actual(6, 0.1);
  start_approach_final_hold(final_target, actual);
  transport_->cancel();
  pump_until([this]() {return command_count() > 3;});

  const auto count_after_cancel = command_count();
  EXPECT_FALSE(transport_->final_target_hold_.active());
  EXPECT_EQ(last_command(), actual);
  EXPECT_TRUE(last_command_message().velocity.empty());
  transport_->on_simulation_clock(clock_message(11.5));
  EXPECT_EQ(command_count(), count_after_cancel);
  EXPECT_EQ(last_command(), actual);
}

TEST_F(IsaacRosMotionTransportTest, StopRevokesTimedFinalTargetHold)
{
  const std::vector<double> final_target(6, 0.2);
  const std::vector<double> actual(6, 0.1);
  start_approach_final_hold(final_target, actual);
  transport_->stop();
  pump_until([this]() {return command_count() > 3;});

  const auto count_after_stop = command_count();
  EXPECT_FALSE(transport_->final_target_hold_.active());
  EXPECT_EQ(last_command(), actual);
  EXPECT_TRUE(last_command_message().velocity.empty());
  transport_->on_simulation_clock(clock_message(11.5));
  EXPECT_EQ(command_count(), count_after_stop);
  EXPECT_EQ(last_command(), actual);
}

TEST_F(IsaacRosMotionTransportTest, SafetyPreemptRevokesTimedFinalTargetHold)
{
  const std::vector<double> final_target(6, 0.2);
  const std::vector<double> actual(6, 0.1);
  start_approach_final_hold(final_target, actual);
  transport_->safety_preempt(arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED);
  pump_until([this]() {return command_count() > 3;});

  const auto count_after_preempt = command_count();
  EXPECT_FALSE(transport_->final_target_hold_.active());
  EXPECT_EQ(last_command(), actual);
  EXPECT_TRUE(last_command_message().velocity.empty());
  transport_->on_simulation_clock(clock_message(11.5));
  EXPECT_EQ(command_count(), count_after_preempt);
  EXPECT_EQ(last_command(), actual);
}

}  // namespace
}  // namespace arm_cell_motion_moveit2
