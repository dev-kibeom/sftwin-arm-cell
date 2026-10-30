#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <future>
#include <functional>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
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
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#define private public
#include "arm_cell_motion/isaac_ros_motion_transport.hpp"
#include "arm_cell_motion/pick_capture_reference.hpp"
#include "arm_cell_motion/pick_grasp_pose_candidates.hpp"
#include "arm_cell_motion/pick_rectangle_geometry.hpp"
#undef private

namespace arm_cell_motion
{
namespace
{
using JointState = sensor_msgs::msg::JointState;
using ExecuteTask = arm_cell_interfaces::action::ExecuteTask;
using MotionTaskType = arm_cell_interfaces::msg::MotionTaskType;
using String = std_msgs::msg::String;
using MoveGroupAction = moveit_msgs::action::MoveGroup;

// Analytic IK for the test-only Cartesian slide chain. Collision checking,
// scene snapshots, trajectory validation and transport submission stay real.
class CartesianSlideIK : public kinematics::KinematicsBase
{
public:
  CartesianSlideIK()
  {
    setValues("robot_description", "arm", "base_link", std::vector<std::string>{"sf_grasp_tcp"}, 0.01);
  }
  bool getPositionIK(const geometry_msgs::msg::Pose & pose, const std::vector<double> &,
    std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error,
    const kinematics::KinematicsQueryOptions &) const override
  {
    solution = {pose.position.x, pose.position.y, pose.position.z, 0.0, 0.0, 0.0};
    error.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }
  bool searchPositionIK(const geometry_msgs::msg::Pose & pose,
    const std::vector<double> & seed, double, std::vector<double> & solution,
    moveit_msgs::msg::MoveItErrorCodes & error,
    const kinematics::KinematicsQueryOptions & options) const override
  {return getPositionIK(pose, seed, solution, error, options);}
  bool searchPositionIK(const geometry_msgs::msg::Pose & pose,
    const std::vector<double> & seed, double timeout, const std::vector<double> &,
    std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error,
    const kinematics::KinematicsQueryOptions & options) const override
  {return searchPositionIK(pose, seed, timeout, solution, error, options);}
  bool searchPositionIK(const geometry_msgs::msg::Pose & pose,
    const std::vector<double> & seed, double timeout, std::vector<double> & solution,
    const IKCallbackFn & callback, moveit_msgs::msg::MoveItErrorCodes & error,
    const kinematics::KinematicsQueryOptions & options) const override
  {
    if (!searchPositionIK(pose, seed, timeout, solution, error, options)) {return false;}
    if (callback) {callback(pose, solution, error);}
    return error.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
  bool searchPositionIK(const geometry_msgs::msg::Pose & pose,
    const std::vector<double> & seed, double timeout, const std::vector<double> &,
    std::vector<double> & solution, const IKCallbackFn & callback,
    moveit_msgs::msg::MoveItErrorCodes & error,
    const kinematics::KinematicsQueryOptions & options) const override
  {return searchPositionIK(pose, seed, timeout, solution, callback, error, options);}
  bool getPositionFK(const std::vector<std::string> &, const std::vector<double> & joints,
    std::vector<geometry_msgs::msg::Pose> & poses) const override
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = joints[0] + joints[3];
    pose.position.y = joints[1] + joints[4];
    pose.position.z = joints[2] + joints[5];
    pose.orientation.w = 1.0;
    poses = {pose};
    return true;
  }
  const std::vector<std::string> & getJointNames() const override {return joints_;}
  const std::vector<std::string> & getLinkNames() const override {return links_;}
private:
  const std::vector<std::string> joints_{
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  const std::vector<std::string> links_{"sf_grasp_tcp"};
};

// Script only the solver boundary; Motion still performs FK, bounds, contact,
// state validity and the real MoveGroup client request.
class ScriptedPickIK : public CartesianSlideIK
{
public:
  int grasp_error{moveit_msgs::msg::MoveItErrorCodes::SUCCESS};
  int approach_error{moveit_msgs::msg::MoveItErrorCodes::SUCCESS};
  bool unset_error_after_first{false};
  double grasp_offset{0.0};
  double approach_offset{0.0};
  mutable std::vector<std::vector<double>> grasp_seeds;
  std::function<void()> on_grasp;

  bool getPositionIK(
    const geometry_msgs::msg::Pose & pose, const std::vector<double> & seed,
    std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error,
    const kinematics::KinematicsQueryOptions & options) const override
  {
    if (approach_error != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      error.val = approach_error;
      return false;
    }
    CartesianSlideIK::getPositionIK(pose, seed, solution, error, options);
    solution[0] += approach_offset;
    return true;
  }

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & pose,
    const std::vector<double> & seed, double, std::vector<double> & solution,
    const IKCallbackFn & callback, moveit_msgs::msg::MoveItErrorCodes & error,
    const kinematics::KinematicsQueryOptions & options) const override
  {
    grasp_seeds.push_back(seed);
    if (on_grasp) {on_grasp();}
    if (unset_error_after_first && grasp_seeds.size() > 1U) {return false;}
    if (grasp_error != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      error.val = grasp_error;
      return false;
    }
    CartesianSlideIK::getPositionIK(pose, seed, solution, error, options);
    solution[0] += grasp_offset;
    if (grasp_seeds.size() % 2U == 0U) {
      solution[0] += 0.05;
      solution[3] -= 0.05;  // Another joint configuration with identical TCP FK.
    }
    if (callback) {callback(pose, solution, error);}
    return error.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
};

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
    close_subscription_ = observer_->create_subscription<String>(
      "/gripper/close", 10, [this](const String::SharedPtr message) {
        std::lock_guard<std::mutex> lock(close_mutex_);
        close_ids_.push_back(message->data);
      });
    capture_reference_subscription_ = observer_->create_subscription<String>(
      "/gripper/capture_reference", 10, [this](const String::SharedPtr message) {
        std::lock_guard<std::mutex> lock(capture_reference_mutex_);
        capture_reference_payloads_.push_back(message->data);
      });
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

  void publish_feedback(
    const std::vector<double> & positions, bool complete = true,
    std::int64_t stamp_increment_ns = 16'666'667,
    const std::vector<double> & velocities = {})
  {
    std::uint64_t previous_generation = 0U;
    {
      std::lock_guard<std::mutex> lock(transport_->mutex_);
      previous_generation = transport_->joint_state_generation_;
    }
    JointState message;
    feedback_stamp_ns_ += stamp_increment_ns;
    message.header.stamp.sec = static_cast<std::int32_t>(feedback_stamp_ns_ / 1'000'000'000);
    message.header.stamp.nanosec = static_cast<std::uint32_t>(
      feedback_stamp_ns_ % 1'000'000'000);
    message.name = complete ? arm_joint_names() : std::vector<std::string>{"joint_1"};
    message.position = complete ? positions : std::vector<double>{positions.front()};
    message.velocity = velocities;
    feedback_publisher_->publish(message);
    if (complete) {
      if (stamp_increment_ns == 0) {
        for (int index = 0; index < 20; ++index) {
          executor_.spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      } else {
        pump_until([this, previous_generation]() {
          std::lock_guard<std::mutex> lock(transport_->mutex_);
          return transport_->joint_state_generation_ > previous_generation;
        });
      }
    } else {
      for (int index = 0; index < 20; ++index) {
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }

  void enable_profile_capture_for_test()
  {
    configure_diagnostics_for_test(true);
    transport_->configure_waypoint_mode(true, true);
  }

  void configure_diagnostics_for_test(
    bool enabled, const std::string & output_directory =
    "/tmp/arm_cell_motion_profile_transport_test")
  {
    transport_.reset();
    const auto snapshot_directory = output_directory + "_snapshots";
    transport_ = std::make_unique<IsaacRosMotionTransport>(
      node_, arm_joint_names(), std::vector<double>(6, 0.1), std::vector<double>(
        6,
        -0.1),
      false, false, "", std::vector<std::string>{}, 0.0, "linear", 2.0, 0.08726646259971647, 60.0, 0.01, 0.1, 0.1, 5.0, 1.0, 5.0,
      enabled, output_directory, snapshot_directory);
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

  std::vector<std::string> close_ids() const
  {
    std::lock_guard<std::mutex> lock(close_mutex_);
    return close_ids_;
  }

  std::vector<std::string> capture_reference_payloads() const
  {
    std::lock_guard<std::mutex> lock(capture_reference_mutex_);
    return capture_reference_payloads_;
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

  void start_timed_motion_for_test()
  {
    publish_feedback(std::vector<double>(6, 0.0));
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = arm_joint_names();
    trajectory_msgs::msg::JointTrajectoryPoint first, last;
    first.positions = std::vector<double>(6, 0.0);
    last.positions = std::vector<double>(6, 0.1);
    last.time_from_start.sec = 1;
    trajectory.points = {first, last};
    ASSERT_TRUE(transport_->trajectory_worker_->start(trajectory, 41, 0.0).valid);
    transport_->timed_trajectory_active_ = true;
    transport_->execution_active_ = true;
    transport_->inactivity_confirmed_ = false;
    transport_->active_motion_task_type_ = MotionTaskType::TASK_TYPE_PICK;
    transport_->trajectory_watchdog_.start(0.0, 1.0, std::chrono::steady_clock::now());
  }

  rosgraph_msgs::msg::Clock::SharedPtr clock_message(double simulation_time) const
  {
    auto message = std::make_shared<rosgraph_msgs::msg::Clock>();
    message->clock.sec = static_cast<std::int32_t>(simulation_time);
    message->clock.nanosec = static_cast<std::uint32_t>(
      (simulation_time - static_cast<double>(message->clock.sec)) * 1e9);
    return message;
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
  rclcpp::Subscription<String>::SharedPtr close_subscription_;
  mutable std::mutex close_mutex_;
  std::vector<std::string> close_ids_;
  rclcpp::Subscription<String>::SharedPtr capture_reference_subscription_;
  mutable std::mutex capture_reference_mutex_;
  std::vector<std::string> capture_reference_payloads_;
  rclcpp::Subscription<JointState>::SharedPtr command_subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<IsaacRosMotionTransport> transport_;
  mutable std::mutex command_mutex_;
  std::vector<JointState> commands_;
  std::size_t simulate_feedback_after_command_count_{0};
  std::int64_t feedback_stamp_ns_{0};
};

TEST_F(IsaacRosMotionTransportTest, StoredPlaceWaypointTrajectoryLoadsOnceAndPlansWithoutMoveGroup)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("place_waypoint_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(directory);
  const auto path = directory / "trajectory.json";
  const std::vector<std::string> joints = arm_joint_names();
  const std::vector<double> home(6, 0.1);
  nlohmann::json artifact{
    {"schema_version", 1}, {"robot_model", "m0609_robotiq_2f85"},
    {"planning_group", "arm"}, {"joint_names", joints},
    {"planning_config", {{"planner_id", "RRTConnect"},
      {"velocity_scaling", 0.1}, {"acceleration_scaling", 0.1},
      {"planned_velocity_scale", 0.1}, {"planned_acceleration_scale", 0.1}}},
    {"home_joint_positions", home},
    {"object_target_pose", {0.5, 0.25, 0.3675, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476}},
    {"approach_tcp_pose", {0.5, 0.25, 0.3825, 0.7071067811865476, 0.0, 0.0, 0.7071067811865476}},
    {"points", nlohmann::json::array({
      {{"positions", home}, {"velocities", std::vector<double>(6, 0.0)},
       {"accelerations", std::vector<double>(6, 0.0)}, {"effort", nlohmann::json::array()},
       {"time_sec", 0}, {"time_nanosec", 0}},
      {{"positions", std::vector<double>(6, 0.2)},
       {"velocities", std::vector<double>(6, 0.1)},
       {"accelerations", std::vector<double>(6, 0.0)}, {"effort", nlohmann::json::array()},
       {"time_sec", 2}, {"time_nanosec", 0}}})}};
  std::ofstream(path) << artifact.dump();
  auto loaded = std::make_unique<IsaacRosMotionTransport>(
    node_, joints, home, std::vector<double>(6, -0.1), false, false, "",
    std::vector<std::string>{}, 0.0, "linear", 2.0, 0.08726646259971647, 60.0,
    0.03, 0.1, 0.1, 5.0, 1.0, 5.0, false, "", "", "RRTConnect", path.string());
  EXPECT_FALSE(loaded->cached_place_waypoint_approach_.has_value());
  loaded->configure_waypoint_mode(false, false);
  EXPECT_FALSE(loaded->place_waypoint_trajectory_load_attempted_);
  loaded->configure_waypoint_mode(false, true);
  ASSERT_TRUE(loaded->cached_place_waypoint_approach_.has_value());
  NormalizedMotionRequest request;
  request.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  request.place_waypoint_approach = true;
  request.object_pose.header.frame_id = "base_link";
  request.object_pose.pose.position.x = 0.5;
  request.object_pose.pose.position.y = 0.25;
  request.object_pose.pose.position.z = 0.3675;
  request.object_pose.pose.orientation.x = 0.7071067811865476;
  request.object_pose.pose.orientation.w = 0.7071067811865476;
  request.approach_target = request.object_pose;
  request.approach_target.pose.position.z = 0.3825;
  for (int cycle = 0; cycle < 2; ++cycle) {
    NormalizedTrajectoryPlan plan;
    EXPECT_EQ(loaded->plan_normalized(request, plan),
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
    ASSERT_EQ(plan.trajectory.points.size(), 2U);
    EXPECT_DOUBLE_EQ(plan.trajectory.points.back().time_from_start.sec, 2);
  }
  EXPECT_EQ(loaded->generate_place_waypoint_approach(request, path.string()),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL);
  EXPECT_EQ(nlohmann::json::parse(std::ifstream(path)), artifact);
  std::ofstream(path, std::ios::trunc) << "{broken";
  auto invalid = std::make_unique<IsaacRosMotionTransport>(
    node_, joints, home, std::vector<double>(6, -0.1), false, false, "",
    std::vector<std::string>{}, 0.0, "linear", 2.0, 0.08726646259971647, 60.0,
    0.03, 0.1, 0.1, 5.0, 1.0, 5.0, false, "", "", "RRTConnect", path.string());
  EXPECT_FALSE(invalid->cached_place_waypoint_approach_.has_value());
  NormalizedTrajectoryPlan rejected;
  EXPECT_EQ(invalid->plan_normalized(request, rejected),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED);
  std::filesystem::remove_all(directory);
}

TEST_F(IsaacRosMotionTransportTest, InterruptedRequestIsDiscardedBeforePlanning)
{
  // Match TaskExecutor admission: initialize search before submitting a PICK.
  transport_->begin_pick_search();
  NormalizedMotionRequest request;
  request.task_type.value = MotionTaskType::TASK_TYPE_PICK;
  request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
  request.motion_goal_uuid = "stopped-pick";
  request.interruption_result = []() {
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
    };

  EXPECT_FALSE(transport_->submit_normalized(request));
  EXPECT_EQ(transport_->normalized_failure_result(),
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
  EXPECT_NE(transport_->last_pick_candidate_evaluation().find(
      "\"interrupted_result_discarded\":true"), std::string::npos);
}

TEST_F(IsaacRosMotionTransportTest, HoldCompletesWhenCommandProductionAndHoldDeliveryFinish)
{
  const std::vector<double> initial(6, 0.0);
  const std::vector<double> stopped(6, 0.2);
  publish_feedback(initial);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  clear_commands();

  transport_->hold();
  ASSERT_TRUE(transport_->inactivity_confirmed());
  EXPECT_FALSE(transport_->execution_active());
  pump_until([this]() {return command_count() > 0;});
  EXPECT_EQ(last_command(), initial);

  publish_feedback(stopped, false);
  EXPECT_TRUE(transport_->inactivity_confirmed());
  publish_feedback(stopped, true, 250'000'000);
  EXPECT_TRUE(transport_->inactivity_confirmed());
  publish_feedback(stopped, true, 100'000'000);
  EXPECT_TRUE(transport_->inactivity_confirmed());
  publish_feedback(stopped, true, 250'000'000);
  EXPECT_TRUE(transport_->inactivity_confirmed());
  publish_feedback(stopped, true, 250'000'000);
  EXPECT_TRUE(transport_->inactivity_confirmed());
}

TEST_F(IsaacRosMotionTransportTest, StaleJointFeedbackAndFailedHoldStayUnconfirmed)
{
  const std::vector<double> initial(6, 0.0);
  const std::vector<double> stopped(6, 0.2);
  publish_feedback(initial);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  std::this_thread::sleep_for(std::chrono::milliseconds(510));

  transport_->safety_preempt(arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
  EXPECT_FALSE(transport_->inactivity_confirmed());
  EXPECT_TRUE(transport_->execution_active());
  EXPECT_EQ(
    transport_->inactivity_diagnostic(), "stop_hold_not_published:joint_state_stale");
  publish_feedback(stopped);
  publish_feedback(stopped);
  publish_feedback(stopped);
  EXPECT_FALSE(transport_->inactivity_confirmed());
  EXPECT_TRUE(transport_->execution_active());
}

TEST_F(IsaacRosMotionTransportTest, StopCompletesAfterHoldDeliveryWithoutStableJointSamples)
{
  const std::vector<double> initial(6, 0.0);
  publish_feedback(initial);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  transport_->safety_preempt(arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);

  const auto base_stamp = feedback_stamp_ns_;
  publish_feedback(std::vector<double>(6, 0.2), true, 1'000'000);
  EXPECT_TRUE(transport_->inactivity_confirmed());
  EXPECT_FALSE(transport_->execution_active());
  EXPECT_GT(feedback_stamp_ns_, base_stamp);
  EXPECT_NE(transport_->inactivity_diagnostic().find("stop_hold_delivered"), std::string::npos);
}

TEST_F(IsaacRosMotionTransportTest, StopDoesNotWaitForNonzeroJointVelocity)
{
  const std::vector<double> initial(6, 0.0);
  publish_feedback(initial);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  transport_->safety_preempt(arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);

  const std::vector<double> stopped(6, 0.0);
  const std::vector<double> moving_velocity(6, 0.01);
  publish_feedback(stopped, true, 100'000'000, moving_velocity);
  EXPECT_TRUE(transport_->inactivity_confirmed());
  EXPECT_FALSE(transport_->execution_active());
}

TEST_F(IsaacRosMotionTransportTest, StopCompletesWhileJointPositionContinuesToChange)
{
  const std::vector<double> initial(6, 0.0);
  publish_feedback(initial);

  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  transport_->safety_preempt(arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);

  for (int index = 1; index <= 20; ++index) {
    publish_feedback(
      std::vector<double>(6, index * 0.0004), true, 100'000'000);
  }
  EXPECT_TRUE(transport_->inactivity_confirmed());
  EXPECT_FALSE(transport_->execution_active());
  EXPECT_EQ(transport_->inactivity_diagnostic(), "stop_hold_delivered");
}

TEST_F(IsaacRosMotionTransportTest, JointTargetRejectsBeforePublishingWithoutMeasuredState)
{
  bool published = true;
  {
    std::lock_guard<std::mutex> lock(transport_->mutex_);
    published = transport_->publish_joint_target_locked(std::vector<double>(6, 0.2));
  }

  EXPECT_FALSE(published);
  EXPECT_EQ(command_count(), 0U);
}

TEST_F(IsaacRosMotionTransportTest, TrajectoryWorkerStartFailurePrecedesCommandSink)
{
  std::size_t command_sink_calls = 0U;
  TrajectoryExecutionWorker worker(
    nullptr,
    [&command_sink_calls](const TimedJointSample &) {
      ++command_sink_calls;
      return true;
    });
  trajectory_msgs::msg::JointTrajectory trajectory;

  EXPECT_FALSE(worker.start(trajectory, 1U, 0.0).valid);
  EXPECT_EQ(command_sink_calls, 0U);
}

TEST_F(IsaacRosMotionTransportTest, TrajectoryWorkerSinkFailureOccursAfterAcceptedStart)
{
  std::size_t command_sink_calls = 0U;
  TrajectoryExecutionWorker worker(
    std::make_shared<LinearTrajectorySampler>(),
    [&command_sink_calls](const TimedJointSample &) {
      ++command_sink_calls;
      return false;
    });
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = {"joint"};
  trajectory_msgs::msg::JointTrajectoryPoint initial;
  initial.positions = {0.0};
  initial.time_from_start.sec = 0;
  trajectory.points.push_back(initial);
  trajectory_msgs::msg::JointTrajectoryPoint final;
  final.positions = {0.1};
  final.time_from_start.sec = 1;
  trajectory.points.push_back(final);

  EXPECT_TRUE(worker.start(trajectory, 1U, 0.0).valid);
  EXPECT_EQ(command_sink_calls, 0U);
  EXPECT_FALSE(worker.on_simulation_time(0.0).valid);
  EXPECT_EQ(command_sink_calls, 1U);
  EXPECT_TRUE(worker.failed());
}

TEST_F(IsaacRosMotionTransportTest, RacedSubmissionDoesNotRepeatAnAlreadyAppliedHold)
{
  const std::vector<double> initial(6, 0.0);
  publish_feedback(initial);
  transport_->safety_preempt(arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
  ASSERT_TRUE(transport_->stop_requested_);
  ASSERT_TRUE(transport_->stop_command_published_);

  {
    std::lock_guard<std::mutex> lock(transport_->mutex_);
    ASSERT_TRUE(transport_->publish_joint_target_locked(std::vector<double>(6, 0.2)));
  }
  EXPECT_FALSE(transport_->stop_requested_);
  transport_->reassert_hold_after_raced_motion_command();
  pump_until([this]() {return command_count() >= 3U;});
  EXPECT_EQ(command_count(), 3U);
  EXPECT_TRUE(transport_->stop_requested_);
}

TEST_F(IsaacRosMotionTransportTest, StopBeforeLateSubmissionReusesThePublishedHold)
{
  const std::vector<double> initial(6, 0.0);
  publish_feedback(initial);
  {
    std::lock_guard<std::mutex> lock(transport_->mutex_);
    ASSERT_TRUE(transport_->publish_joint_target_locked(std::vector<double>(6, 0.2)));
  }
  transport_->safety_preempt(arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
  pump_until([this]() {return command_count() >= 2U;});
  const auto stop_command_count = command_count();

  transport_->reassert_hold_after_raced_motion_command();
  EXPECT_EQ(command_count(), stop_command_count);
  EXPECT_TRUE(transport_->stop_requested_);
  EXPECT_TRUE(transport_->stop_command_published_);
}

TEST_F(IsaacRosMotionTransportTest, StopPreemptRejectsSubsequentTimedTrajectoryCommand)
{
  const std::vector<double> initial(6, 0.0);
  publish_feedback(initial);
  transport_->safety_preempt(arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
  pump_until([this]() {return command_count() > 0U;});
  const auto stop_command_count = command_count();

  TimedJointSample sample;
  sample.positions = std::vector<double>(6, 0.1);
  sample.velocities = std::vector<double>(6, 0.0);
  EXPECT_FALSE(transport_->publish_timed_joint_target(sample));
  executor_.spin_some();
  EXPECT_EQ(command_count(), stop_command_count);
  EXPECT_TRUE(transport_->stop_requested_);
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
    "<link name=\"sf_grasp_tcp\"/><joint name=\"tcp_joint\" type=\"fixed\">"
    "<parent link=\"link_1\"/><child link=\"sf_grasp_tcp\"/></joint>"
    "<joint name=\"joint_1\" type=\"revolute\"><parent link=\"base_link\"/>"
    "<child link=\"link_1\"/><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
    "<axis xyz=\"0 0 1\"/><limit lower=\"-3.14\" upper=\"3.14\""
    " effort=\"1\" velocity=\"1\"/></joint></robot>");
  node_->declare_parameter<std::string>(
    "robot_description_semantic",
    "<robot name=\"test\"><group name=\"arm\"><chain base_link=\"base_link\" tip_link=\"sf_grasp_tcp\"/>"
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
    "<link name=\"sf_grasp_tcp\"/><joint name=\"tcp_joint\" type=\"fixed\">"
    "<parent link=\"link_1\"/><child link=\"sf_grasp_tcp\"/></joint>"
    "<joint name=\"joint_1\" type=\"revolute\"><parent link=\"base_link\"/>"
    "<child link=\"link_1\"/><origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>"
    "<axis xyz=\"0 0 1\"/><limit lower=\"-3.14\" upper=\"3.14\""
    " effort=\"1\" velocity=\"1\"/></joint></robot>");
  node_->declare_parameter<std::string>(
    "robot_description_semantic",
    "<robot name=\"test\"><group name=\"arm\"><chain base_link=\"base_link\" tip_link=\"sf_grasp_tcp\"/>"
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

TEST_F(
  IsaacRosMotionTransportTest,
  PickApproachOccupancyIsSentOnlyInItsMoveGroupPlanningRequest)
{
  const auto diagnostic_id = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path diagnostic_directory = "/tmp/arm_cell_motion_occupancy_snapshot_" +
    std::to_string(diagnostic_id);
  std::filesystem::remove_all(diagnostic_directory);
  configure_diagnostics_for_test(false, diagnostic_directory.string());
  std::string urdf = "<robot name='slide'><link name='base_link'/>";
  const std::array<std::string, 3> axes{"1 0 0", "0 1 0", "0 0 1"};
  for (std::size_t index = 0; index < 6; ++index) {
    const auto parent = index == 0 ? "base_link" : "link_" + std::to_string(index);
    const auto child = "link_" + std::to_string(index + 1);
    urdf += "<link name='" + child + "'/>";
    urdf += "<joint name='joint_" + std::to_string(index + 1) + "' type='prismatic'>" +
      "<parent link='" + parent + "'/><child link='" + child + "'/><axis xyz='" +
      axes[index % 3] + "'/><limit lower='-2' upper='2' effort='1' velocity='1'/></joint>";
  }
  urdf += "<link name='sf_grasp_tcp'><collision><geometry><sphere radius='0.01'/>";
  urdf += "</geometry></collision></link><joint name='tcp_fixed' type='fixed'>";
  urdf += "<parent link='link_6'/><child link='sf_grasp_tcp'/></joint></robot>";
  node_->declare_parameter<std::string>("robot_description", urdf);
  node_->declare_parameter<std::string>("robot_description_semantic",
    "<robot name='slide'><group name='arm'><chain base_link='base_link' "
    "tip_link='sf_grasp_tcp'/></group></robot>");

  std::vector<MoveGroupAction::Goal> planning_requests;
  int32_t default_planner_code = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  auto planning_server = rclcpp_action::create_server<MoveGroupAction>(
    observer_, "move_action",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveGroupAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroupAction>>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroupAction>> goal) {
      planning_requests.push_back(*goal->get_goal());
      auto result = std::make_shared<MoveGroupAction::Result>();
      result->error_code.val = goal->get_goal()->planning_options.planning_scene_diff.
        world.collision_objects.empty() ? default_planner_code :
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      result->planned_trajectory.joint_trajectory.joint_names = arm_joint_names();
      for (const auto z : {0.2, 0.1}) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.0, 0.0, z, 0.0, 0.0, 0.0};
        point.time_from_start = rclcpp::Duration::from_seconds(z == 0.2 ? 0.0 : 1.0);
        result->planned_trajectory.joint_trajectory.points.push_back(point);
      }
      goal->succeed(result);
    });
  auto group_future = std::async(std::launch::async, [&]() {
      return std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        node_, "arm", std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration::from_seconds(3.0));
    });
  const auto initialization_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (group_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
    std::chrono::steady_clock::now() < initialization_deadline)
  {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(group_future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  auto group = group_future.get();
  auto model = std::const_pointer_cast<moveit::core::RobotModel>(group->getRobotModel());
  const auto solver = std::make_shared<CartesianSlideIK>();
  model->setKinematicsAllocators({{"arm", [solver](const auto *) {return solver;}}});
  transport_->move_group_state_->pick_ik_solver = solver;
  transport_->move_group_state_->move_group = std::move(group);
  transport_->move_group_state_->ready = true;

  planning_scene::PlanningScene scene(model);
  const std::vector<std::string> occupancy_ids{
    "vision_candidate_occupancy_probe_observed",
    "vision_candidate_occupancy_probe_unknown"};
  const auto add_box = [&](const std::string & id, double x) {
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = "base_link";
      object.id = id;
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      shape_msgs::msg::SolidPrimitive shape;
      shape.type = shape_msgs::msg::SolidPrimitive::BOX;
      shape.dimensions = {0.04, 0.04, 0.04};
      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.z = 1.0;
      pose.orientation.w = 1.0;
      object.primitives = {shape};
      object.primitive_poses = {pose};
      EXPECT_TRUE(scene.processCollisionObjectMsg(object));
    };
  add_box(occupancy_ids[0], 1.0);
  add_box(occupancy_ids[1], 1.5);
  add_box("fixed_equipment", 1.8);
  moveit_msgs::msg::PlanningScene original_scene;
  scene.getPlanningSceneMsg(original_scene);
  std::size_t snapshot_count = 0;
  auto snapshot_service = observer_->create_service<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene", [&](
      const std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request>,
      std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response> response) {
      ++snapshot_count;
      response->scene = original_scene;
    });
  std::size_t validity_call_count = 0U;
  auto validity_service = observer_->create_service<moveit_msgs::srv::GetStateValidity>(
    "/check_state_validity", [&](
      const std::shared_ptr<moveit_msgs::srv::GetStateValidity::Request> request,
      std::shared_ptr<moveit_msgs::srv::GetStateValidity::Response> response) {
      ++validity_call_count;
      moveit::core::RobotState state(model);
      moveit::core::robotStateMsgToRobotState(request->robot_state, state);
      state.update();
      response->valid = !scene.isStateColliding(state, "arm");
      if (!response->valid) {
        moveit_msgs::msg::ContactInformation contact;
        contact.contact_body_1 = "sf_grasp_tcp";
        contact.contact_body_2 = occupancy_ids[0];
        response->contacts.push_back(contact);
      }
    });
  auto state_publisher = observer_->create_publisher<JointState>("/joint_states", 10);
  NormalizedMotionRequest request;
  request.task_type.value = MotionTaskType::TASK_TYPE_PICK;
  request.motion_goal_uuid = "withheld-relaxed-pick";
  request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
  request.has_estimated_object_height = true;
  request.has_observed_rectangle = true;
  request.rectangle_long_side_m = 0.04F;
  request.rectangle_short_side_m = 0.02F;
  request.estimated_object_height_m = 0.04;
  request.grasp_width_min_mm = 10.0F;
  request.grasp_width_max_mm = 80.0F;
  request.has_non_target_occupancy = true;
  request.vision_acquisition_id = "acquisition-occupancy-snapshot";
  request.vision_candidate_id = "selected-candidate";
  arm_cell_interfaces::msg::PickCandidate selected_candidate;
  selected_candidate.acquisition_id = request.vision_acquisition_id;
  selected_candidate.candidate_id = request.vision_candidate_id;
  request.material_candidates.push_back(selected_candidate);
  request.observation_pose.header.frame_id = "base_link";
  request.observation_pose.header.stamp = node_->now();
  for (std::size_t index = 0; index < 1U; ++index) {
    arm_cell_interfaces::msg::CollisionPrimitive primitive;
    primitive.occupancy_class = index == 0 ?
      arm_cell_interfaces::msg::CollisionPrimitive::OBSERVED_OCCUPANCY :
      arm_cell_interfaces::msg::CollisionPrimitive::UNKNOWN_OCCUPANCY;
    const auto object = std::find_if(
      original_scene.world.collision_objects.begin(), original_scene.world.collision_objects.end(),
      [&](const auto & value) {return value.id == occupancy_ids[index];});
    ASSERT_NE(object, original_scene.world.collision_objects.end());
    primitive.shape = object->primitives[0];
    primitive.pose = object->primitive_poses[0];
    request.non_target_occupancy.push_back(primitive);
  }
  request.tcp_target.header.frame_id = "base_link";
  request.tcp_target.pose.position.z = 0.1;
  request.tcp_target.pose.orientation.w = 1.0;
  request.target_pose = request.tcp_target;
  request.object_pose = request.target_pose;
  request.insertion_axis_tcp.z = -1.0;
  const auto submit = [&]() {
      auto submitted = std::async(std::launch::async, [&]() {
          return transport_->submit_normalized(request);
        });
      while (submitted.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        JointState state;
        state.header.stamp = node_->now();
        state.name = arm_joint_names();
        state.position = {0.0, 0.0, 0.2, 0.0, 0.0, 0.0};
        feedback_publisher_->publish(state);
        state_publisher->publish(state);
        executor_.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      executor_.spin_some();
      return submitted.get();
    };
  // The actual MoveGroup plan-only action receives this request's Vision
  // geometry; the legacy snapshot and shared scene remain untouched.
  publish_feedback({0.0, 0.0, 0.2, 0.0, 0.0, 0.0});
  transport_->begin_pick_search();
  const bool submitted = submit();
  ASSERT_FALSE(planning_requests.empty()) << transport_->last_pick_candidate_evaluation();
  EXPECT_TRUE(submitted);
  pump_until([&]() {return command_count() > 0U;});
  EXPECT_EQ(snapshot_count, 1U);
  EXPECT_EQ(validity_call_count, 0U);
  ASSERT_FALSE(planning_requests.empty());
  const auto & request_diff = planning_requests.back().planning_options.planning_scene_diff;
  EXPECT_TRUE(request_diff.is_diff);
  EXPECT_EQ(
    std::count_if(
      request_diff.world.collision_objects.begin(), request_diff.world.collision_objects.end(),
      [](const auto & object) {
        return object.operation == moveit_msgs::msg::CollisionObject::ADD;
      }), 1);
  EXPECT_EQ(
    std::count_if(
      request_diff.world.collision_objects.begin(), request_diff.world.collision_objects.end(),
      [](const auto & object) {
        return object.operation == moveit_msgs::msg::CollisionObject::REMOVE;
      }), 2);

  moveit_msgs::msg::PlanningScene after_scene;
  scene.getPlanningSceneMsg(after_scene);
  EXPECT_EQ(after_scene.world, original_scene.world);

  const auto snapshots_directory = std::filesystem::path(
    diagnostic_directory.string() + "_snapshots");
  const auto png_count = [&]() {
      std::size_t count = 0U;
      if (std::filesystem::exists(snapshots_directory)) {
        for (const auto & entry : std::filesystem::recursive_directory_iterator(snapshots_directory)) {
          if (entry.is_regular_file() && entry.path().extension() == ".png") {
            ++count;
            EXPECT_EQ(entry.path().parent_path(), snapshots_directory / "acquisition" /
              request.vision_acquisition_id);
            EXPECT_FALSE(cv::imread(entry.path().string(), cv::IMREAD_COLOR).empty());
          }
        }
      }
      return count;
    };
  pump_until([&]() {return png_count() == 1U;});
  EXPECT_FALSE(std::filesystem::exists(diagnostic_directory / "motion_planning_records"));

  // Repeated Pose/Seed planning for the same candidate reuses the existing
  // attempt snapshot. A candidate transition creates exactly one new image.
  EXPECT_TRUE(submit());
  EXPECT_EQ(png_count(), 1U);
  request.vision_candidate_id = "next-candidate";
  request.material_candidates.front().candidate_id = request.vision_candidate_id;
  EXPECT_TRUE(submit());
  pump_until([&]() {return png_count() == 2U;});
  EXPECT_FALSE(std::filesystem::exists(diagnostic_directory / "motion_planning_records"));
  if (std::filesystem::exists(diagnostic_directory)) {
    for (const auto & entry : std::filesystem::recursive_directory_iterator(diagnostic_directory)) {
      EXPECT_NE(entry.path().extension(), ".png");
    }
  }
  transport_.reset();
  std::filesystem::remove_all(diagnostic_directory.string() + "_snapshots");
  std::filesystem::remove_all(diagnostic_directory);
}

TEST_F(IsaacRosMotionTransportTest, PickEvaluationConsumesSolverAndValidationFacts)
{
  enable_profile_capture_for_test();
  std::string urdf = "<robot name='slide'><link name='base_link'/>";
  const std::array<std::string, 3> axes{"1 0 0", "0 1 0", "0 0 1"};
  for (std::size_t index = 0; index < 6; ++index) {
    const auto parent = index == 0 ? "base_link" : "link_" + std::to_string(index);
    const auto child = "link_" + std::to_string(index + 1);
    urdf += "<link name='" + child + "'/>";
    urdf += "<joint name='joint_" + std::to_string(index + 1) + "' type='prismatic'>" +
      "<parent link='" + parent + "'/><child link='" + child + "'/><axis xyz='" +
      axes[index % 3] + "'/><limit lower='-2' upper='2' effort='1' velocity='1'/></joint>";
  }
  urdf += "<link name='sf_grasp_tcp'><collision><geometry><sphere radius='0.01'/>";
  urdf += "</geometry></collision></link><joint name='tcp_fixed' type='fixed'>";
  urdf += "<parent link='link_6'/><child link='sf_grasp_tcp'/></joint></robot>";
  node_->declare_parameter<std::string>("robot_description", urdf);
  node_->declare_parameter<std::string>(
    "robot_description_semantic",
    "<robot name='slide'><group name='arm'><chain base_link='base_link' "
    "tip_link='sf_grasp_tcp'/></group></robot>");

  std::vector<MoveGroupAction::Goal> planning_requests;
  int32_t default_planner_code = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  auto planning_server = rclcpp_action::create_server<MoveGroupAction>(
    observer_, "move_action",
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveGroupAction::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroupAction>>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveGroupAction>> goal) {
      planning_requests.push_back(*goal->get_goal());
      auto result = std::make_shared<MoveGroupAction::Result>();
      result->error_code.val = default_planner_code;
      result->planned_trajectory.joint_trajectory.joint_names = arm_joint_names();
      for (const auto z : {0.2, 0.1}) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.0, 0.0, z, 0.0, 0.0, 0.0};
        point.time_from_start = rclcpp::Duration::from_seconds(z == 0.2 ? 0.0 : 1.0);
        result->planned_trajectory.joint_trajectory.points.push_back(point);
      }
      goal->succeed(result);
    });
  auto group_future = std::async(
    std::launch::async, [&]() {
      return std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        node_, "arm", std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration::from_seconds(3.0));
    });
  const auto initialization_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (group_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
    std::chrono::steady_clock::now() < initialization_deadline)
  {
    executor_.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(group_future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  auto group = group_future.get();
  auto model = std::const_pointer_cast<moveit::core::RobotModel>(group->getRobotModel());
  const auto solver = std::make_shared<ScriptedPickIK>();
  model->setKinematicsAllocators({{"arm", [solver](const auto *) {return solver;}}});
  transport_->move_group_state_->pick_ik_solver = solver;
  transport_->move_group_state_->move_group = std::move(group);
  transport_->move_group_state_->ready = true;

  using Codes = moveit_msgs::msg::MoveItErrorCodes;
  int validity_mode = 0;
  planning_scene::PlanningScene scene(model);
  moveit_msgs::msg::PlanningScene scene_message;
  scene.getPlanningSceneMsg(scene_message);
  auto snapshot_service = observer_->create_service<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene", [&](
      const std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request>,
      std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response> response) {
      response->scene = scene_message;
    });
  auto state_publisher = observer_->create_publisher<JointState>("/joint_states", 10);
  const std::vector<double> measured{0.0, 0.0, 0.2, 0.0, 0.0, 0.0};
  struct Case
  {
    const char * name;
    int grasp_error;
    int approach_error;
    double grasp_offset;
    double approach_offset;
    int validity_mode;
    int planner_code;
    const char * evaluation;
    std::size_t attempts;
    bool backend;
  };
  const std::vector<Case> cases{
    {"selected_capture_unavailable", Codes::SUCCESS, Codes::SUCCESS, 0, 0, 0, Codes::SUCCESS,
      "EXECUTABLE_APPROACH_SELECTED", 1U, false},
    {"no_solution", Codes::NO_IK_SOLUTION, Codes::SUCCESS, 0, 0, 0, Codes::SUCCESS,
      "GRASP_IK_NO_SOLUTION", 6U, false},
    {"stale_solver_error", Codes::NO_IK_SOLUTION, Codes::SUCCESS, 0, 0, 0, Codes::SUCCESS,
      "GRASP_IK_NO_SOLUTION", 6U, false},
    {"budget", Codes::NO_IK_SOLUTION, Codes::SUCCESS, 0, 0, 0, Codes::SUCCESS,
      "SEARCH_BUDGET_EXHAUSTED", 2U, false},
    {"single_pose_no_solution", Codes::NO_IK_SOLUTION, Codes::SUCCESS, 0, 0, 0, Codes::SUCCESS,
      "GRASP_IK_NO_SOLUTION", 2U, false},
    {"timeout", Codes::TIMED_OUT, Codes::SUCCESS, 0, 0, 0, Codes::SUCCESS,
      "GRASP_IK_FAILED", 2U, false},
    {"unknown_solver_failure", Codes::FAILURE, Codes::SUCCESS, 0, 0, 0, Codes::SUCCESS,
      "GRASP_IK_FAILED", 2U, false},
    {"solver_backend", Codes::FRAME_TRANSFORM_FAILURE, Codes::SUCCESS, 0, 0, 0, Codes::SUCCESS,
      "BACKEND_ERROR", 1U, true},
    {"grasp_bounds", Codes::SUCCESS, Codes::SUCCESS, 3.0, 0, 0, Codes::SUCCESS,
      "GRASP_VALIDATION_FAILED", 2U, false},
    {"grasp_fk", Codes::SUCCESS, Codes::SUCCESS, 0.1, 0, 0, Codes::SUCCESS,
      "GRASP_VALIDATION_FAILED", 2U, false},
    {"grasp_collision", Codes::SUCCESS, Codes::SUCCESS, 0, 0, 1, Codes::SUCCESS,
      "GRASP_COLLISION", 2U, false},
      {"grasp_collision_vision", Codes::SUCCESS, Codes::SUCCESS, 0, 0, 0, Codes::SUCCESS,
        "GRASP_COLLISION", 2U, false},
    {"approach_ik", Codes::SUCCESS, Codes::NO_IK_SOLUTION, 0, 0, 0, Codes::SUCCESS,
      "APPROACH_IK_FAILED", 2U, false},
    {"approach_fk", Codes::SUCCESS, Codes::SUCCESS, 0, 0.1, 0, Codes::SUCCESS,
      "APPROACH_VALIDATION_FAILED", 2U, false},
    {"approach_collision", Codes::SUCCESS, Codes::SUCCESS, 0, 0, 2, Codes::SUCCESS,
      "APPROACH_VALIDATION_FAILED", 2U, false},
    {"planning", Codes::SUCCESS, Codes::SUCCESS, 0, 0, 0, Codes::PLANNING_FAILED,
      "GLOBAL_PLANNING_FAILED", 1U, false},
    {"planning_backend", Codes::SUCCESS, Codes::SUCCESS, 0, 0, 0, Codes::COMMUNICATION_FAILURE,
      "BACKEND_ERROR", 1U, true},
    {"interrupted", Codes::SUCCESS, Codes::SUCCESS, 0, 0, 0, Codes::SUCCESS,
      "INTERRUPTED", 1U, false},
  };
  for (const auto & test : cases) {
    SCOPED_TRACE(test.name);
    solver->grasp_error = test.grasp_error;
    solver->unset_error_after_first = std::string(test.name) == "stale_solver_error";
    solver->approach_error = test.approach_error;
    solver->grasp_offset = test.grasp_offset;
    solver->approach_offset = test.approach_offset;
    solver->grasp_seeds.clear();
    validity_mode = test.validity_mode;
    scene.getPlanningSceneMsg(scene_message);
    if (validity_mode != 0) {
      moveit_msgs::msg::CollisionObject obstacle;
      obstacle.header.frame_id = "base_link";
      obstacle.id = "fixture";
      obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;
      shape_msgs::msg::SolidPrimitive shape;
      shape.type = shape_msgs::msg::SolidPrimitive::BOX;
      shape.dimensions = {0.04, 0.04, 0.04};
      geometry_msgs::msg::Pose pose;
      pose.position.z = validity_mode == 1 ? 0.1 : 0.2;
      pose.orientation.w = 1.0;
      obstacle.primitives = {shape};
      obstacle.primitive_poses = {pose};
      scene_message.world.collision_objects.push_back(obstacle);
    }
    planning_requests.clear();
    default_planner_code = test.planner_code;
    NormalizedMotionRequest request;
    request.task_type.value = MotionTaskType::TASK_TYPE_PICK;
    request.motion_goal_uuid = test.name;
    request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
    request.tcp_target.header.frame_id = "base_link";
    request.tcp_target.pose.position.z = 0.1;
    request.tcp_target.pose.orientation.w = 1.0;
    request.target_pose = request.tcp_target;
    request.object_pose = request.target_pose;
    request.insertion_axis_tcp.z = -1.0;
    request.approach_distance_m = 0.1;
    if (std::string(test.name) == "grasp_collision_vision") {
      request.has_non_target_occupancy = true;
      request.observation_pose.header.frame_id = "base_link";
      arm_cell_interfaces::msg::CollisionPrimitive occupancy;
      occupancy.occupancy_class =
        arm_cell_interfaces::msg::CollisionPrimitive::OBSERVED_OCCUPANCY;
      occupancy.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
      occupancy.shape.dimensions = {0.04, 0.04, 0.04};
      occupancy.pose.position.z = 0.1;
      occupancy.pose.orientation.w = 1.0;
      request.non_target_occupancy.push_back(occupancy);
    }
    if (test.grasp_error == Codes::NO_IK_SOLUTION) {
      request.has_estimated_object_height = true;
      request.has_observed_rectangle = true;
      request.rectangle_long_side_m = 0.04F;
      request.rectangle_short_side_m = 0.02F;
      request.estimated_object_height_m = 0.04;
      request.grasp_width_min_mm = 10.0F;
      request.grasp_width_max_mm = 80.0F;
    }
    bool interrupted = false;
    nlohmann::json previous_snapshot;
    solver->on_grasp = [&]() {
        interrupted = std::string(test.name) == "interrupted";
        if (solver->grasp_seeds.size() > 1U) {
          previous_snapshot = nlohmann::json::parse(transport_->last_pick_candidate_evaluation());
        }
        if (std::string(test.name) == "budget" && solver->grasp_seeds.size() == 2U) {
          std::this_thread::sleep_for(std::chrono::milliseconds(600));
        }
      };
    request.interruption_result = [&]() {
        return interrupted ?
               arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED :
               arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
      };
    publish_feedback(measured);
    transport_->begin_pick_search();
    transport_->pick_grasp_pose_candidate_limit_ =
      std::string(test.name) == "single_pose_no_solution" ? 1U : 3U;
    transport_->pick_search_budget_s_ = std::string(test.name) == "budget" ? 0.5 : 100.0;
    // Old same-action collision evidence must not influence an unrelated Pose.
    transport_->pick_collision_feedback_goal_uuid_ = request.motion_goal_uuid;
    transport_->pick_collision_feedback_ = {{"link_4", "old_fixture", "old_stage",
      false, 0U, 99U, 1U}};
    auto submitted = std::async(
      std::launch::async, [&]() {
        auto shared_transport = std::shared_ptr<IsaacMotionTransport>(transport_.get(), [](auto *) {});
        IsaacMotionBackend backend(shared_transport);
        NormalizedTrajectoryPlan plan;
        return backend.plan_normalized(request, plan);
      });
    while (submitted.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      JointState state;
      state.header.stamp = node_->now();
      state.name = arm_joint_names();
      state.position = measured;
      feedback_publisher_->publish(state);
      state_publisher->publish(state);
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const bool selected = std::string(test.name) == "selected_capture_unavailable";
    EXPECT_EQ(submitted.get(), transport_->normalized_failure_result());
    const auto record = nlohmann::json::parse(transport_->last_pick_candidate_evaluation());
    EXPECT_EQ(record.at("pick_grasp_pose_order_policy"), "distance_then_tilt");
    const auto & ordered_pose_records = record.at("grasp_pose_candidates");
    for (std::size_t index = 0U; index < ordered_pose_records.size(); ++index) {
      EXPECT_EQ(ordered_pose_records[index].at("evaluation_rank"), index + 1U);
      EXPECT_EQ(ordered_pose_records[index].at("generation_index"), index);
      EXPECT_TRUE(ordered_pose_records[index].contains("withdrawal_tilt_rad"));
    }
    const auto & attempts = record.at("pick_attempts");
    ASSERT_EQ(attempts.size(), solver->grasp_seeds.size());
    for (std::size_t i = 0; i < attempts.size(); ++i) {
      EXPECT_EQ(attempts[i].at("attempt"), i + 1U);
      EXPECT_TRUE(attempts[i].contains("grasp_pose_candidate"));
      EXPECT_TRUE(attempts[i].contains("ik_seed_attempt"));
      EXPECT_TRUE(attempts[i].contains("grasp_ik"));
      EXPECT_TRUE(attempts[i].contains("approach_ik"));
      EXPECT_TRUE(attempts[i].contains("validation_elapsed_s"));
      const auto & bt_steps = attempts[i].at("executed_stages");
      EXPECT_NE(std::find(bt_steps.begin(), bt_steps.end(), "PrepareAttempt"), bt_steps.end());
      EXPECT_NE(std::find(bt_steps.begin(), bt_steps.end(), "SolveGraspIK"), bt_steps.end());
    }
    if (interrupted) {
      EXPECT_EQ(record.at("candidate_search_termination_reason"), "INTERRUPTED");
      EXPECT_EQ(record.at("grasp_pose_candidates")[0].at("result"), "DISCARDED_INTERRUPTED");
      EXPECT_EQ(solver->grasp_seeds.size(), 1U);
      EXPECT_EQ(
        transport_->normalized_failure_result(),
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
      solver->on_grasp = {};
      continue;
    }
    ASSERT_EQ(attempts.size(), test.attempts) << record.dump();
    if (std::string(test.name) == "budget") {
      EXPECT_EQ(record.at("candidate_search_termination_reason"), "SEARCH_BUDGET_EXHAUSTED");
      EXPECT_EQ(record.at("ik_termination_reason"), "SEARCH_BUDGET_EXHAUSTED");
      EXPECT_EQ(attempts.back().at("search_evaluation"), test.evaluation);
      EXPECT_EQ(record.at("grasp_pose_candidates")[0].at("result"), "SEED_DEFERRED");
      EXPECT_EQ(record.at("grasp_pose_candidates")[0].at("transition_reason"),
        "SEARCH_BUDGET_EXHAUSTED");
      EXPECT_EQ(transport_->normalized_failure_result(),
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED);
      continue;
    }
    EXPECT_EQ(attempts.front().at("search_evaluation"), test.evaluation);
    if (selected) {
      EXPECT_EQ(transport_->normalized_failure_result(),
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
      EXPECT_EQ(record.at("candidate_search_termination_reason"), "EXECUTABLE_APPROACH_PLAN_SELECTED");
      EXPECT_EQ(record.at("grasp_pose_candidates")[0].at("result"), "PLANNED_SELECTED");
      continue;
    }
    if (!test.backend) {
      EXPECT_EQ(record.at("candidate_search_termination_reason"), "ALL_CANDIDATES_EXHAUSTED");
      for (const auto & candidate : record.at("grasp_pose_candidates")) {
        EXPECT_NE(candidate.at("result"), "SEED_DEFERRED");
      }
    }
    if (std::string(test.name) == "single_pose_no_solution") {
      ASSERT_EQ(previous_snapshot.at("pick_attempts").size(), 1U);
      EXPECT_EQ(previous_snapshot.at("grasp_pose_candidates")[0].at("result"), "SEED_DEFERRED");
      EXPECT_EQ(attempts[1].at("ik_seed_attempt"), 2);
      EXPECT_EQ(attempts[1].at("grasp_pose_candidate"), 1);
      continue;
    }
    if (test.backend) {
      EXPECT_EQ(
        transport_->normalized_failure_result(),
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
    }
    if (solver->unset_error_after_first) {
      EXPECT_EQ(attempts[1].at("search_evaluation"), "GRASP_IK_FAILED");
      EXPECT_EQ(attempts[1].at("grasp_ik").at("moveit_error_code"), Codes::FAILURE);
      const std::vector<int> expected_poses{1, 2, 2, 3, 3, 1};
      const std::vector<int> expected_seeds{1, 1, 2, 1, 2, 2};
      for (std::size_t i = 0; i < attempts.size(); ++i) {
        EXPECT_EQ(attempts[i].at("grasp_pose_candidate"), expected_poses[i]);
        EXPECT_EQ(attempts[i].at("ik_seed_attempt"), expected_seeds[i]);
      }
    } else if (test.grasp_error == Codes::NO_IK_SOLUTION) {
      for (std::size_t i = 0; i < 6; ++i) {
        EXPECT_EQ(attempts[i].at("grasp_pose_candidate"), i % 3 + 1);
        EXPECT_EQ(attempts[i].at("ik_seed_attempt"), i / 3 + 1);
        EXPECT_FALSE(attempts[i].at("collision_seed_diversification_used").get<bool>());
        EXPECT_TRUE(attempts[i].at("collision_feedback_source_contacts").empty());
        if (i < 3) {EXPECT_EQ(solver->grasp_seeds[i], measured);}
      }
    } else if (test.attempts == 2U) {
      EXPECT_EQ(attempts[1].at("search_evaluation"), test.evaluation);
      EXPECT_EQ(attempts[1].at("grasp_pose_candidate"), 1);
      EXPECT_EQ(attempts[1].at("ik_seed_attempt"), 2);
      const bool collision_hint = test.validity_mode == 1 || test.validity_mode == 2 ||
        std::string(test.name) == "grasp_collision_vision";
      EXPECT_EQ(attempts[1].at("collision_seed_diversification_used"), collision_hint);
      EXPECT_EQ(
        attempts[1].at("collision_feedback_source_contacts").size(),
        collision_hint ? 1U : 0U);
    }
    solver->on_grasp = {};
  }
}

TEST_F(IsaacRosMotionTransportTest, CancelUsesIsaacCommandPathAndCompletesAfterHoldDelivery)
{
  const std::vector<double> measured(6, 0.0);
  publish_feedback(measured);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  clear_commands();

  transport_->cancel();
  EXPECT_TRUE(transport_->inactivity_confirmed());
  pump_until([this]() {return command_count() > 0;});
}

TEST_F(IsaacRosMotionTransportTest, DirectMotionWaitsForMeasuredCompletion)
{
  publish_feedback(std::vector<double>(6, 0.0));
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  pump_until([this]() {return command_count() > 0;});
  EXPECT_EQ(last_command_message().velocity, std::vector<double>(6, 0.0));

  auto completion = std::async(
    std::launch::async, [this]() {return transport_->wait_for_motion_completion();});
  EXPECT_EQ(
    completion.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);

  publish_feedback(std::vector<double>(6, 0.08));
  EXPECT_EQ(
    completion.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  publish_feedback(std::vector<double>(6, 0.08));
  ASSERT_EQ(
    completion.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::COMPLETED);
}

TEST_F(IsaacRosMotionTransportTest, PositionOnlyTargetClearsPreviousVelocityTarget)
{
  publish_feedback(std::vector<double>(6, 0.0));
  const std::vector<double> positions(6, 0.2);
  const std::vector<double> velocities(6, 0.4);
  {
    std::lock_guard<std::mutex> lock(transport_->mutex_);
    ASSERT_TRUE(transport_->publish_joint_target_locked(positions, &velocities));
  }
  pump_until([this]() {return command_count() == 1U;});

  {
    std::lock_guard<std::mutex> lock(transport_->mutex_);
    ASSERT_TRUE(transport_->publish_joint_target_locked(std::vector<double>(6, 0.0)));
  }
  pump_until([this]() {return command_count() == 2U;});

  const auto commands = command_messages();
  ASSERT_EQ(commands.size(), 2U);
  EXPECT_EQ(commands[0].velocity, velocities);
  EXPECT_EQ(commands[1].velocity, std::vector<double>(6, 0.0));
}

TEST_F(IsaacRosMotionTransportTest, SafetyHoldClearsPreviousVelocityTarget)
{
  publish_feedback(std::vector<double>(6, 0.0));
  const std::vector<double> positions(6, 0.2);
  const std::vector<double> velocities(6, 0.4);
  {
    std::lock_guard<std::mutex> lock(transport_->mutex_);
    ASSERT_TRUE(transport_->publish_joint_target_locked(positions, &velocities));
  }
  pump_until([this]() {return command_count() == 1U;});

  transport_->safety_preempt(arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
  pump_until([this]() {return command_count() == 2U;});

  const auto commands = command_messages();
  ASSERT_EQ(commands.size(), 2U);
  EXPECT_EQ(commands[0].velocity, velocities);
  EXPECT_EQ(commands[1].velocity, std::vector<double>(6, 0.0));
}

TEST_F(IsaacRosMotionTransportTest, PickRetractCompletesWithoutFeedbackAndWithoutInactivityEvidence)
{
  const std::vector<double> settled_residual(6, 0.030001);
  publish_feedback(settled_residual);

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = arm_joint_names();
  trajectory_msgs::msg::JointTrajectoryPoint initial;
  initial.positions = std::vector<double>(6, 0.0);
  initial.velocities = std::vector<double>(6, 0.1);
  initial.time_from_start.sec = 0;
  trajectory.points.push_back(initial);
  trajectory_msgs::msg::JointTrajectoryPoint final;
  final.positions = std::vector<double>(6, 0.1);
  final.velocities = std::vector<double>(6, 0.0);
  final.time_from_start.sec = 1;
  trajectory.points.push_back(final);

  auto & worker = *transport_->trajectory_worker_;
  worker.set_joint_target_accuracy_tolerance(0.03);
  ASSERT_TRUE(worker.start(trajectory, 1, 0.0).valid);

  {
    std::lock_guard<std::mutex> lock(transport_->mutex_);
    transport_->active_motion_task_type_ = MotionTaskType::TASK_TYPE_PICK;
    transport_->target_positions_ = std::vector<double>(6, 0.1);
    transport_->motion_command_generation_ = transport_->joint_state_generation_;
    transport_->execution_active_ = true;
    transport_->inactivity_confirmed_ = false;
    transport_->timed_trajectory_active_ = true;
    transport_->motion_progress_watchdog_ = MotionProgressWatchdog(0.01);
    transport_->motion_progress_watchdog_.start(
      settled_residual, std::chrono::steady_clock::now());
    transport_->motion_progress_watchdog_active_ = true;
    transport_->trajectory_watchdog_.start(
      0.0, 1.0, std::chrono::steady_clock::now());
  }
  transport_->on_simulation_clock(clock_message(1.0));

  EXPECT_TRUE(worker.completion_eligible());
  EXPECT_FALSE(worker.diagnostics().final_joint_target_within_tolerance);
  EXPECT_FALSE(transport_->inactivity_confirmed());
  EXPECT_FALSE(transport_->execution_active());
  transport_->on_simulation_clock(clock_message(5.0));
  EXPECT_EQ(
    transport_->wait_for_motion_completion(), MotionCompletionOutcome::COMPLETED);
}

TEST_F(IsaacRosMotionTransportTest, FinalSinkCancelPreservesInterruption)
{
  start_timed_motion_for_test();
  auto sink = transport_->trajectory_worker_->command_sink_;
  transport_->trajectory_worker_->command_sink_ = [this, sink](const TimedJointSample & sample) {
      const auto accepted = sink(sample);
      transport_->cancel();
      return accepted;
    };
  transport_->on_simulation_clock(clock_message(1.0));
  EXPECT_FALSE(transport_->trajectory_worker_->completion_eligible());
  EXPECT_EQ(transport_->wait_for_motion_completion(), MotionCompletionOutcome::CANCELED);
}

TEST_F(IsaacRosMotionTransportTest, FinalSinkSafetyStopPreservesPreemption)
{
  start_timed_motion_for_test();
  auto sink = transport_->trajectory_worker_->command_sink_;
  transport_->trajectory_worker_->command_sink_ = [this, sink](const TimedJointSample & sample) {
      const auto accepted = sink(sample);
      transport_->stop();
      return accepted;
    };
  transport_->on_simulation_clock(clock_message(1.0));
  EXPECT_FALSE(transport_->trajectory_worker_->completion_eligible());
  EXPECT_EQ(transport_->wait_for_motion_completion(), MotionCompletionOutcome::SAFETY_PREEMPTED);
  EXPECT_TRUE(transport_->inactivity_confirmed());
}

TEST_F(IsaacRosMotionTransportTest, ClockStallCancelsCommandsBeforeLaterFinalClock)
{
  start_timed_motion_for_test();
  transport_->trajectory_watchdog_.start(
    0.0, 1.0,
    std::chrono::steady_clock::now() - std::chrono::seconds(3));
  EXPECT_EQ(transport_->wait_for_motion_completion(), MotionCompletionOutcome::TIMEOUT);
  transport_->on_simulation_clock(clock_message(1.0));
  EXPECT_FALSE(transport_->trajectory_worker_->completion_eligible());
  EXPECT_FALSE(transport_->trajectory_worker_->active());
}

TEST_F(IsaacRosMotionTransportTest, BackwardClockCannotBeOverwrittenByFinalClock)
{
  start_timed_motion_for_test();
  transport_->on_simulation_clock(clock_message(0.5));
  transport_->on_simulation_clock(clock_message(0.25));
  transport_->on_simulation_clock(clock_message(1.0));
  EXPECT_EQ(
    transport_->wait_for_motion_completion(),
    MotionCompletionOutcome::TRANSPORT_UNAVAILABLE);
  EXPECT_FALSE(transport_->trajectory_worker_->completion_eligible());
}

TEST_F(IsaacRosMotionTransportTest, DirectMotionProgressingBeyondOldDeadlineCanComplete)
{
  transport_.reset();
  transport_ = std::make_unique<IsaacRosMotionTransport>(
    node_, arm_joint_names(), std::vector<double>(6, 0.1), std::vector<double>(6, -0.1),
    false, false, "", std::vector<std::string>{}, 0.0, "linear", 2.0,
    0.08726646259971647, 0.25);
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

TEST_F(IsaacRosMotionTransportTest, TimedPickTrajectoryStallCancelsWorkerAndPublishesHold)
{
  const std::vector<double> measured(6, 0.0);
  publish_feedback(measured);
  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = arm_joint_names();
  trajectory_msgs::msg::JointTrajectoryPoint initial;
  initial.positions = measured;
  initial.velocities = std::vector<double>(6, 0.1);
  trajectory.points.push_back(initial);
  trajectory_msgs::msg::JointTrajectoryPoint final;
  final.positions = std::vector<double>(6, 0.1);
  final.velocities = std::vector<double>(6, 0.0);
  final.time_from_start.sec = 1;
  trajectory.points.push_back(final);

  auto & worker = *transport_->trajectory_worker_;
  ASSERT_TRUE(worker.start(trajectory, 1, 0.0).valid);
  {
    std::lock_guard<std::mutex> lock(transport_->mutex_);
    transport_->active_motion_task_type_ = MotionTaskType::TASK_TYPE_PICK;
    transport_->target_positions_ = std::vector<double>(6, 0.1);
    transport_->motion_command_generation_ = transport_->joint_state_generation_;
    transport_->execution_active_ = true;
    transport_->inactivity_confirmed_ = false;
    transport_->timed_trajectory_active_ = true;
    transport_->motion_progress_watchdog_ = MotionProgressWatchdog(0.05);
    transport_->motion_progress_watchdog_.start(measured, std::chrono::steady_clock::now());
    transport_->motion_progress_watchdog_active_ = true;
    transport_->trajectory_watchdog_.start(
      0.0, 1.0, std::chrono::steady_clock::now());
  }
  transport_->on_simulation_clock(clock_message(0.01));
  const auto initial_command_count = command_count();
  auto completion = std::async(
    std::launch::async, [this]() {return transport_->wait_for_motion_completion();});
  for (int index = 1; index <= 10 &&
    completion.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready; ++index)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    transport_->on_simulation_clock(clock_message(index * 0.01));
    publish_feedback(measured);
  }

  EXPECT_EQ(completion.wait_for(std::chrono::milliseconds(50)), std::future_status::ready);
  pump_until([this, initial_command_count]() {
    return command_count() > initial_command_count && last_command() == std::vector<double>(6, 0.0);
  });
  EXPECT_EQ(last_command(), measured);

  for (int index = 0; index < 35 &&
    completion.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready; ++index)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    publish_feedback(measured);
  }
  ASSERT_EQ(completion.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::JOINT_PROGRESS_STALLED);
  EXPECT_FALSE(transport_->execution_active());
  EXPECT_TRUE(transport_->inactivity_confirmed());
  const auto stop_diagnostic = transport_->inactivity_diagnostic();
  EXPECT_NE(stop_diagnostic.find("trajectory_worker_cancelled=true"), std::string::npos);
  EXPECT_NE(stop_diagnostic.find("hold_command_published=true"), std::string::npos);
  EXPECT_NE(stop_diagnostic.find("stop_hold_delivered"), std::string::npos);
}

TEST_F(IsaacRosMotionTransportTest, DirectMotionFailsWhenFreshFeedbackStopsProgressing)
{
  transport_.reset();
  transport_ = std::make_unique<IsaacRosMotionTransport>(
    node_, arm_joint_names(), std::vector<double>(6, 0.1), std::vector<double>(6, -0.1),
    false, false, "", std::vector<std::string>{}, 0.0, "linear", 2.0,
    0.08726646259971647, 0.25);
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
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::JOINT_PROGRESS_STALLED);
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

TEST_F(IsaacRosMotionTransportTest, FeedbackGapWarnsAndResumesMeasuredCompletion)
{
  publish_feedback(std::vector<double>(6, 0.0));
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));

  auto completion = std::async(
    std::launch::async, [this]() {return transport_->wait_for_motion_completion();});
  const auto previous_handler = rcutils_logging_get_output_handler();
  captured_logs.clear();
  rcutils_logging_set_output_handler(capture_log);
  const auto gap_wait_status = completion.wait_for(std::chrono::milliseconds(650));
  rcutils_logging_set_output_handler(previous_handler);
  EXPECT_EQ(gap_wait_status, std::future_status::timeout);
  EXPECT_EQ(
    std::count_if(
      captured_logs.begin(), captured_logs.end(), [](const CapturedLog & log) {
        return log.severity == RCUTILS_LOG_SEVERITY_WARN &&
               log.format.find("Motion joint feedback observation gap") != std::string::npos;
      }), 1);
  const auto gap_warning = std::find_if(
    captured_logs.begin(), captured_logs.end(), [](const CapturedLog & log) {
      return log.severity == RCUTILS_LOG_SEVERITY_WARN &&
             log.format.find("Motion joint feedback observation gap") != std::string::npos;
    });
  ASSERT_NE(gap_warning, captured_logs.end());
  const auto elapsed_offset = gap_warning->format.find("elapsed_ms=");
  ASSERT_NE(elapsed_offset, std::string::npos);
  EXPECT_GE(std::stoll(gap_warning->format.substr(elapsed_offset + 11)), 500LL);
  EXPECT_NE(
    gap_warning->format.find(
      "task_type=TASK_TYPE_GO_HOME phase=TASK_TYPE_GO_HOME"), std::string::npos);

  publish_feedback(std::vector<double>(6, 0.1));
  publish_feedback(std::vector<double>(6, 0.1));
  ASSERT_EQ(completion.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::COMPLETED);
}

TEST_F(IsaacRosMotionTransportTest, MissingInitialFeedbackWarnsOnceAfterExecutionWaitStarts)
{
  pump_until([this]() {
    return transport_->joint_command_publisher_->get_subscription_count() > 0U;
  });
  {
    std::lock_guard<std::mutex> lock(transport_->mutex_);
    transport_->active_motion_task_type_ = MotionTaskType::TASK_TYPE_GO_HOME;
  }
  auto completion = std::async(
    std::launch::async, [this]() {return transport_->wait_for_motion_completion();});

  const auto previous_handler = rcutils_logging_get_output_handler();
  captured_logs.clear();
  rcutils_logging_set_output_handler(capture_log);
  const auto gap_wait_status = completion.wait_for(std::chrono::milliseconds(650));
  rcutils_logging_set_output_handler(previous_handler);
  EXPECT_EQ(gap_wait_status, std::future_status::timeout);
  EXPECT_EQ(
    std::count_if(
      captured_logs.begin(), captured_logs.end(), [](const CapturedLog & log) {
        return log.severity == RCUTILS_LOG_SEVERITY_WARN &&
               log.format.find("Motion joint feedback observation gap") != std::string::npos;
      }), 1);
  const auto gap_warning = std::find_if(
    captured_logs.begin(), captured_logs.end(), [](const CapturedLog & log) {
      return log.severity == RCUTILS_LOG_SEVERITY_WARN &&
             log.format.find("Motion joint feedback observation gap") != std::string::npos;
    });
  ASSERT_NE(gap_warning, captured_logs.end());
  const auto elapsed_offset = gap_warning->format.find("elapsed_ms=");
  ASSERT_NE(elapsed_offset, std::string::npos);
  EXPECT_GE(std::stoll(gap_warning->format.substr(elapsed_offset + 11)), 500LL);

  transport_->cancel();
  ASSERT_EQ(completion.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::CANCELED);
}

TEST_F(IsaacRosMotionTransportTest, FeedbackGapDoesNotCountAsMotionProgressStall)
{
  const std::vector<double> measured(6, 0.0);
  publish_feedback(measured);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  {
    std::lock_guard<std::mutex> lock(transport_->mutex_);
    transport_->motion_progress_watchdog_ = MotionProgressWatchdog(0.55);
    transport_->motion_progress_watchdog_.start(
      measured, std::chrono::steady_clock::now());
    transport_->motion_progress_watchdog_active_ = true;
  }

  auto completion = std::async(
    std::launch::async, [this]() {return transport_->wait_for_motion_completion();});
  EXPECT_EQ(
    completion.wait_for(std::chrono::milliseconds(650)), std::future_status::timeout);

  publish_feedback(measured);
  EXPECT_EQ(
    completion.wait_for(std::chrono::milliseconds(20)), std::future_status::timeout);
  transport_->cancel();
  ASSERT_EQ(completion.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(completion.get(), MotionCompletionOutcome::CANCELED);
}

TEST_F(IsaacRosMotionTransportTest, EmergencyStopCompletesAfterHoldDelivery)
{
  const std::vector<double> measured(6, 0.0);
  publish_feedback(measured);
  ExecuteTask::Goal goal;
  goal.task_type.value = MotionTaskType::TASK_TYPE_GO_HOME;
  ASSERT_TRUE(transport_->submit(goal));
  clear_commands();

  transport_->stop();
  EXPECT_TRUE(transport_->inactivity_confirmed());
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

TEST_F(IsaacRosMotionTransportTest, TaskAccuracyLogsOnlySuccessfulOutOfToleranceWarning)
{
  const auto previous_handler = rcutils_logging_get_output_handler();
  captured_logs.clear();
  rcutils_logging_set_output_handler(capture_log);
  NormalizedMotionRequest request;
  request.task_type.value = MotionTaskType::TASK_TYPE_PICK;
  request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING;
  request.pick_grasp_position_tolerance_m = 0.01;
  request.pick_grasp_orientation_tolerance_rad = 0.1;

  transport_->log_task_accuracy_summary(
    request, arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS,
    4.8, 0.02094);
  transport_->log_task_accuracy_summary(
    request, arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS,
    15.1, 0.02100);
  request.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  request.place_position_tolerance_m = 0.01;
  request.place_orientation_tolerance_rad = 0.1;
  transport_->log_task_accuracy_summary(
    request, arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS,
    6.1, 0.01571);
  request.task_type.value = MotionTaskType::TASK_TYPE_PICK;
  transport_->log_task_accuracy_summary(
    request, arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR,
    15.1, 0.02100);
  transport_->log_task_accuracy_summary(
    request, arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED,
    5.2, 0.02200);
  transport_->log_task_accuracy_summary(
    request, arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED,
    5.3, 0.02300);

  rcutils_logging_set_output_handler(previous_handler);
  ASSERT_EQ(captured_logs.size(), 1U);
  EXPECT_EQ(captured_logs.front().severity, RCUTILS_LOG_SEVERITY_WARN);
  EXPECT_NE(
    captured_logs.front().format.find("PICK grasp completed outside Recipe accuracy tolerance"),
    std::string::npos);
  EXPECT_NE(captured_logs.front().format.find("tcp_position_error_mm=15.100"), std::string::npos);
}

TEST_F(IsaacRosMotionTransportTest, AccuracyPathStrategyTracksCurrentGoalAndPhase)
{
  using Phase = arm_cell_interfaces::msg::MotionTaskPhase;
  const std::string goal_uuid = "current-goal";
  IsaacRosMotionTransport::ExecutedPathSelection selected{
    goal_uuid, Phase::TASK_PHASE_APPROACHING, "GLOBAL"};
  NormalizedMotionRequest request;
  request.motion_goal_uuid = goal_uuid;
  request.phase = Phase::TASK_PHASE_APPROACHING;
  EXPECT_EQ(IsaacRosMotionTransport::diagnostic_path_strategy(selected, request), "GLOBAL");

  // The latest candidate's executed path replaces the earlier candidate's value.
  selected.strategy = "CARTESIAN";
  request.phase = Phase::TASK_PHASE_EXECUTING;
  selected.phase = Phase::TASK_PHASE_EXECUTING;
  EXPECT_EQ(IsaacRosMotionTransport::diagnostic_path_strategy(selected, request), "CARTESIAN");
  selected.strategy = "GLOBAL_FALLBACK";
  EXPECT_EQ(
    IsaacRosMotionTransport::diagnostic_path_strategy(selected, request), "GLOBAL_FALLBACK");

  request.phase = Phase::TASK_PHASE_APPROACHING;
  EXPECT_EQ(
    IsaacRosMotionTransport::diagnostic_path_strategy(selected, request), "UNAVAILABLE");
  request.phase = Phase::TASK_PHASE_EXECUTING;
  request.motion_goal_uuid = "previous-goal";
  EXPECT_EQ(
    IsaacRosMotionTransport::diagnostic_path_strategy(selected, request), "UNAVAILABLE");
  selected.strategy.clear();
  request.motion_goal_uuid = goal_uuid;
  EXPECT_EQ(
    IsaacRosMotionTransport::diagnostic_path_strategy(selected, request), "UNAVAILABLE");
}

TEST_F(IsaacRosMotionTransportTest, MissingAccuracyDoesNotChangeCompletedTrajectory)
{
  start_timed_motion_for_test();
  transport_->on_simulation_clock(clock_message(1.0));
  ASSERT_TRUE(transport_->trajectory_worker_->completion_eligible());
  transport_->diagnostics_capture_enabled_ = true;
  transport_->planning_max_acceleration_rad_s2_ = 1.0;
  transport_->latest_state_receipt_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);
  NormalizedMotionRequest request;
  request.task_type.value = MotionTaskType::TASK_TYPE_PICK;
  request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING;
  transport_->record_task_accuracy(
    request,
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS);
  ASSERT_EQ(transport_->task_accuracy_history_.size(), 1u);
  const auto accuracy = nlohmann::json::parse(transport_->task_accuracy_history_.back());
  EXPECT_EQ(accuracy.at("segment"), "RETRACT");
  EXPECT_TRUE(accuracy.at("tcp_position_error_mm").is_null());
  EXPECT_TRUE(accuracy.at("tcp_orientation_error_rad").is_null());
  const auto & execution = accuracy.at("trajectory_execution");
  EXPECT_TRUE(execution.at("final_joint_position_error_max_rad").is_null());
  EXPECT_TRUE(execution.at("actual_joint_positions_rad").is_null());
  EXPECT_EQ(execution.at("goal_accuracy"), "UNAVAILABLE");
  EXPECT_TRUE(execution.at("trajectory_execution_complete").get<bool>());
  EXPECT_LE(
    execution.at("final_command_attempt_steady_time_s").get<double>(),
    execution.at("final_command_accepted_steady_time_s").get<double>());
  EXPECT_EQ(transport_->wait_for_motion_completion(), MotionCompletionOutcome::COMPLETED);
}

TEST_F(IsaacRosMotionTransportTest, FailedExecutionAccuracySurvivesTerminalJsonSerialization)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_accuracy_failure_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  struct DirectoryCleanup
  {
    std::filesystem::path path;
    ~DirectoryCleanup() {std::filesystem::remove_all(path);}
  } cleanup{directory};
  configure_diagnostics_for_test(true, directory.string());
  const std::string goal_uuid = "failed-accuracy-goal";
  transport_->set_pending_motion_planning_evidence(
    goal_uuid, MotionTaskType::TASK_TYPE_PICK, "{\"selected_strategy\":\"CARTESIAN\"}");
  transport_->task_accuracy_goal_uuid_ = goal_uuid;
  transport_->task_accuracy_task_type_ = MotionTaskType::TASK_TYPE_PICK;
  transport_->task_accuracy_history_.push_back(
    R"({"segment":"GRASP","motion_goal_uuid":"failed-accuracy-goal","execution_result":"TASK_RESULT_BACKEND_ERROR","measurement_status":"AVAILABLE","tcp_position_error_mm":4.8,"tcp_orientation_error_rad":0.02,"tcp_orientation_error_unit":"rad","path_strategy":"CARTESIAN"})");

  transport_->finalize_motion_planning_diagnostic(
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR);
  transport_.reset();

  const auto path = directory / "motion_planning_records" / (goal_uuid + ".json");
  ASSERT_TRUE(std::filesystem::exists(path));
  std::ifstream input(path, std::ios::binary);
  const std::string serialized(
    (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  const auto record = nlohmann::json::parse(serialized);
  EXPECT_EQ(record.at("task_executor_result_code"), "TASK_RESULT_BACKEND_ERROR");
  const auto & accuracy = record.at("planning_evidence").at("task_accuracy").at(0);
  EXPECT_EQ(accuracy.at("execution_result"), "TASK_RESULT_BACKEND_ERROR");
  EXPECT_EQ(accuracy.at("measurement_status"), "AVAILABLE");
  EXPECT_EQ(accuracy.at("tcp_position_error_mm"), 4.8);
  EXPECT_EQ(accuracy.at("path_strategy"), "CARTESIAN");
}

TEST_F(IsaacRosMotionTransportTest, BackendFailureLinksExistingPlaceEvidenceByGoalUuid)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_backend_failure_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  configure_diagnostics_for_test(true, directory.string());
  const std::string goal_uuid = "place-backend-failure";
  transport_->set_pending_motion_planning_evidence(goal_uuid, MotionTaskType::TASK_TYPE_PLACE,
    R"({"planning_stage":"place_approach","final_failure_stage":"GLOBAL_PLANNING","moveit_error_code":-1,"failure_reason":"PLANNING_FAILED"})");

  transport_->finalize_motion_planning_diagnostic(
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR, 1.25,
    "place_global_planning_or_execution", "");
  transport_.reset();

  const auto path = directory / "motion_planning_records" / (goal_uuid + ".json");
  ASSERT_TRUE(std::filesystem::exists(path));
  std::ifstream input(path, std::ios::binary);
  const auto record = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>()));
  const auto & failure = record.at("planning_evidence").at("backend_failure");
  EXPECT_EQ(record.at("motion_goal_uuid"), goal_uuid);
  EXPECT_EQ(record.at("task_type"), "TASK_TYPE_PLACE");
  EXPECT_EQ(failure.at("stage"), "GLOBAL_PLANNING");
  EXPECT_EQ(failure.at("stage_source"), "planning_evidence.final_failure_stage");
  EXPECT_EQ(failure.at("reason"), "PLANNING_FAILED");
  EXPECT_EQ(failure.at("reason_source"), "planning_evidence.failure_reason");
  EXPECT_EQ(failure.at("backend_result_code"), "TASK_RESULT_BACKEND_ERROR");
  EXPECT_EQ(failure.at("detail_reference"),
    "motion_planning_records/place-backend-failure.json#/planning_evidence");
  std::filesystem::remove_all(directory);
}

TEST_F(IsaacRosMotionTransportTest, BackendFailureWithoutEvidenceReportsUnavailableReason)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_backend_failure_unavailable_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  configure_diagnostics_for_test(true, directory.string());
  transport_->set_pending_motion_planning_evidence(
    "place-no-evidence", MotionTaskType::TASK_TYPE_PLACE, "{}");

  transport_->finalize_motion_planning_diagnostic(
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR, 1.25,
    "place_global_planning_or_execution", "");
  transport_.reset();

  const auto path = directory / "motion_planning_records" / "place-no-evidence.json";
  ASSERT_TRUE(std::filesystem::exists(path));
  std::ifstream input(path, std::ios::binary);
  const auto record = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>()));
  const auto & failure = record.at("planning_evidence").at("backend_failure");
  EXPECT_EQ(failure.at("reason"), "UNAVAILABLE");
  EXPECT_EQ(failure.at("reason_unavailable_reason"), "backend_evidence_has_no_failure_reason");
  EXPECT_TRUE(failure.contains("detail_reference"));
  std::filesystem::remove_all(directory);
}

TEST_F(IsaacRosMotionTransportTest, ExecutionExceptionIsLinkedToMotionGoalAndEvidence)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_backend_exception_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  configure_diagnostics_for_test(true, directory.string());
  const std::string goal_uuid = "place-exception-goal";
  transport_->set_pending_motion_planning_evidence(goal_uuid, MotionTaskType::TASK_TYPE_PLACE,
    R"({"execution_result":"TASK_RESULT_BACKEND_ERROR","execution_stage":"trajectory_worker"})");

  transport_->finalize_motion_planning_diagnostic(
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR, 1.25,
    "execution_worker:exception", "trajectory controller rejected execution");
  transport_.reset();

  const auto path = directory / "motion_planning_records" / (goal_uuid + ".json");
  ASSERT_TRUE(std::filesystem::exists(path));
  std::ifstream input(path, std::ios::binary);
  const auto record = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>()));
  const auto & failure = record.at("planning_evidence").at("backend_failure");
  EXPECT_EQ(record.at("motion_goal_uuid"), goal_uuid);
  EXPECT_EQ(failure.at("stage"), "execution_worker:exception");
  EXPECT_EQ(failure.at("reason"), "trajectory controller rejected execution");
  EXPECT_EQ(failure.at("reason_source"), "task_executor_exception");
  EXPECT_EQ(failure.at("detail_reference"),
    "motion_planning_records/place-exception-goal.json#/planning_evidence");
  std::filesystem::remove_all(directory);
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
  EXPECT_TRUE(transport_->commit_pending_grasp_relation());
  EXPECT_FALSE(transport_->commit_pending_grasp_relation());

  transport_->clear_grasp_relation_after_release();
  EXPECT_FALSE(transport_->commit_pending_grasp_relation());
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

class SelectedCaptureReferenceTest : public IsaacRosMotionTransportTest,
  public testing::WithParamInterface<uint8_t> {};

TEST_P(SelectedCaptureReferenceTest, CloseWaitsForMatchingAppliedCaptureReference)
{
  publish_feedback(std::vector<double>(6, 0.0));
  publish_status(true, false, false, false, 1);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.orientation.w = 1.0;
  ASSERT_TRUE(transport_->stage_pick_capture_reference(pose, 70.0, GetParam(), "pick-current"));
  pump_until([this]() {return !capture_reference_payloads().empty();});
  EXPECT_NE(
    capture_reference_payloads().front().find("\"frame_id\":\"base_link\""),
    std::string::npos);

  EXPECT_FALSE(transport_->command_gripper_close());
  EXPECT_TRUE(close_ids().empty());

  publish_status_payload(
    "ready=1;grasp=0;attached=0;released=0;width_mm=12.500;seq=1;"
    "capture_reference_id=pick-current");
  publish_feedback(std::vector<double>(6, 0.0));
  publish_status_payload(
    "ready=1;grasp=0;attached=0;released=0;width_mm=12.500;seq=2;"
    "capture_reference_id=pick-current");
  ASSERT_TRUE(transport_->command_gripper_close());
  pump_until([this]() {return !close_ids().empty();});
  EXPECT_EQ(close_ids(), (std::vector<std::string>{"pick-current"}));
}

INSTANTIATE_TEST_SUITE_P(
  ProductionAndFixedVision, SelectedCaptureReferenceTest,
  testing::Values(uint8_t{0}, uint8_t{1}, uint8_t{2}, uint8_t{4}));

TEST_F(IsaacRosMotionTransportTest, FixedVisionSelectedReferencePreservesPoseWidthAndOrder)
{
  geometry_msgs::msg::PoseStamped selected;
  selected.header.frame_id = "base_link";
  selected.pose.position.x = 0.31;
  selected.pose.orientation.y = std::sqrt(0.5);
  selected.pose.orientation.w = std::sqrt(0.5);
  for (const uint8_t order : {0U, 1U}) {
    ASSERT_TRUE(transport_->stage_pick_capture_reference(selected, 70.0, order, "validation"));
  }
  pump_until([this]() {return capture_reference_payloads().size() == 2U;});
  const auto payloads = capture_reference_payloads();
  ASSERT_EQ(payloads.size(), 2U);
  for (std::size_t order = 0; order < payloads.size(); ++order) {
    const auto packet = nlohmann::json::parse(payloads[order]);
    EXPECT_EQ(packet["frame_id"], selected.header.frame_id);
    EXPECT_DOUBLE_EQ(packet["position"][0], selected.pose.position.x);
    EXPECT_DOUBLE_EQ(packet["orientation_xyzw"][1], selected.pose.orientation.y);
    EXPECT_DOUBLE_EQ(packet["expected_contact_width_mm"], 70.0);
    EXPECT_EQ(packet["orientation_symmetry_order"], order);
  }
  EXPECT_FALSE(transport_->stage_pick_capture_reference(selected, 0.0, 1, "validation"));
  EXPECT_FALSE(transport_->stage_pick_capture_reference(selected, 70.0, 3, "validation"));
}

TEST_F(IsaacRosMotionTransportTest, EstimatedRectangleCandidatePublishesReferenceWithoutFixtureLookup)
{
  const auto geometry_status = classify_rectangle_geometry(true, true, 0.08, 0.07, 0.07);
  ASSERT_EQ(geometry_status, RectangleGeometryStatus::AVAILABLE);
  const Eigen::Quaterniond object_orientation = Eigen::Quaterniond::Identity();
  const auto candidates = generate_pick_grasp_search_candidates(
    object_orientation, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
    Eigen::Vector3d(0.2, 0.1, 0.7), Eigen::Vector3d(0.0, 0.0, -1.0),
    0.08, 0.07, 0.07, 65.0, 75.0, 85.0,
    PickGraspSearchSampling{80.0, 0.5, 3U, 8U, 48U, 0.005});
  ASSERT_GT(candidates.size(), 1U);
  const auto selected = std::find_if(candidates.begin(), candidates.end(), [](const auto & item) {
      return item.grasp_position_offset_object.norm() < 1e-12 && item.contact && item.contact->width_mm == 70.0;
    });
  ASSERT_NE(selected, candidates.end());

  std::size_t fixture_lookup_count = 0U;
  const auto resolution = resolve_capture_reference_contact_width(
    geometry_status, selected->contact->width_mm, true,
    [&fixture_lookup_count](std::string & reason) -> std::optional<double> {
      ++fixture_lookup_count;
      reason = "fixture_id_unavailable";
      return std::nullopt;
    });
  EXPECT_EQ(resolution.source, "vision_candidate_geometry");
  EXPECT_EQ(resolution.status, "ready");
  EXPECT_DOUBLE_EQ(resolution.width_mm, 70.0);
  EXPECT_TRUE(resolution.failure_reason.empty());
  EXPECT_EQ(fixture_lookup_count, 0U);
  const auto diagnostic = capture_reference_diagnostic_json(
    resolution, geometry_status, "", false, true, 0.08, 0.07, 0.07, 65.0, 75.0);
  EXPECT_EQ(diagnostic["source"], "vision_candidate_geometry");
  EXPECT_EQ(diagnostic["rectangle_geometry_status"], "available");
  EXPECT_FALSE(diagnostic["has_observed_rectangle"]);
  EXPECT_DOUBLE_EQ(diagnostic["rectangle_dimensions_m"][0], 0.08);
  EXPECT_DOUBLE_EQ(diagnostic["rectangle_dimensions_m"][1], 0.07);
  EXPECT_DOUBLE_EQ(diagnostic["estimated_object_height_m"], 0.07);
  EXPECT_DOUBLE_EQ(diagnostic["recipe_contact_width_policy_mm"][0], 65.0);
  EXPECT_DOUBLE_EQ(diagnostic["recipe_contact_width_policy_mm"][1], 75.0);
  EXPECT_DOUBLE_EQ(diagnostic["contact_width_input_mm"], 70.0);

  geometry_msgs::msg::PoseStamped selected_pose;
  selected_pose.header.frame_id = "base_link";
  selected_pose.pose.position.x = 0.2;
  selected_pose.pose.position.y = 0.1;
  selected_pose.pose.position.z = 0.7;
  selected_pose.pose.orientation.x = selected->tcp_orientation.x();
  selected_pose.pose.orientation.y = selected->tcp_orientation.y();
  selected_pose.pose.orientation.z = selected->tcp_orientation.z();
  selected_pose.pose.orientation.w = selected->tcp_orientation.w();
  ASSERT_TRUE(transport_->stage_pick_capture_reference(
      selected_pose, resolution.width_mm, 2U, "partial-acq3-transport"));
  pump_until([this]() {return !capture_reference_payloads().empty();});
  const auto reference = nlohmann::json::parse(capture_reference_payloads().front());
  EXPECT_EQ(reference["motion_goal_uuid"], "partial-acq3-transport");
  EXPECT_EQ(reference["frame_id"], "base_link");
  EXPECT_DOUBLE_EQ(reference["position"][2], 0.7);
  EXPECT_DOUBLE_EQ(reference["expected_contact_width_mm"], 70.0);
  EXPECT_EQ(reference["orientation_symmetry_order"], 2U);
}

TEST_F(IsaacRosMotionTransportTest, GeometryMissingValidationPathStillUsesFixtureWidth)
{
  std::size_t fixture_lookup_count = 0U;
  const auto resolution = resolve_capture_reference_contact_width(
    RectangleGeometryStatus::UNAVAILABLE, 0.0, true,
    [&fixture_lookup_count](std::string &) -> std::optional<double> {
      ++fixture_lookup_count;
      return 70.0;
    });
  EXPECT_EQ(resolution.source, "validation_fixture");
  EXPECT_EQ(resolution.status, "ready");
  EXPECT_DOUBLE_EQ(resolution.width_mm, 70.0);
  EXPECT_EQ(fixture_lookup_count, 1U);
  const auto diagnostic = capture_reference_diagnostic_json(
    resolution, RectangleGeometryStatus::UNAVAILABLE, "fixture-70", false, false,
    0.0, 0.0, 0.0, 65.0, 75.0);
  EXPECT_EQ(diagnostic["source"], "validation_fixture");
  EXPECT_EQ(diagnostic["fixture_id"], "fixture-70");
}

TEST_F(IsaacRosMotionTransportTest, CaptureReferenceRequiresExplicitFrameId)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.orientation.w = 1.0;
  EXPECT_FALSE(transport_->stage_pick_capture_reference(pose, 70.0, 4, "pick-current"));
  EXPECT_TRUE(capture_reference_payloads().empty());
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
  enable_profile_capture_for_test();
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

TEST_F(IsaacRosMotionTransportTest, MotionProfileCaptureRequiresWaypointMode)
{
  configure_diagnostics_for_test(true);
  EXPECT_FALSE(transport_->profile_trace_);

  transport_->configure_waypoint_mode(false, false);
  EXPECT_FALSE(transport_->profile_trace_);

  transport_->configure_waypoint_mode(true, false);
  EXPECT_TRUE(transport_->profile_trace_);
}

TEST_F(IsaacRosMotionTransportTest, CancelledGlobalProfileCaptureCanBeRetried)
{
  enable_profile_capture_for_test();
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


}  // namespace

}  // namespace arm_cell_motion

namespace arm_cell_motion
{
namespace
{

TEST_F(IsaacRosMotionTransportTest, JointTargetPublisherUsesConfiguredTrayReturnValues)
{
  const std::vector<double> tray_return{
    -1.5707963267948966, 0.17453292519943295, 1.7453292519943295,
    0.0, 0.0, 1.5707963267948966};
  node_->set_parameter(rclcpp::Parameter("place_tray_return_joint_positions", tray_return));
  transport_.reset();
  transport_ = std::make_unique<IsaacRosMotionTransport>(
    node_, arm_joint_names(), std::vector<double>(6, 0.1), std::vector<double>(6, -0.1), false);
  simulate_feedback_after_command_count_ = 1U;
  publish_feedback(std::vector<double>(6, 0.1));
  ASSERT_TRUE(transport_->publish_joint_target(transport_->place_tray_return_joint_positions_));
  pump_until([this]() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    return !commands_.empty();
  });
  std::lock_guard<std::mutex> lock(command_mutex_);
  ASSERT_FALSE(commands_.empty());
  EXPECT_EQ(commands_.back().name, arm_joint_names());
  EXPECT_EQ(commands_.back().position, tray_return);
}

TEST_F(IsaacRosMotionTransportTest, RetreatReplanningUsesPlaceFinalBudgetPerAction)
{
  transport_->place_approach_global_max_requests_ = 1U;
  transport_->place_final_global_fallback_max_requests_ = 2U;
  NormalizedMotionRequest request;
  request.task_type.value = MotionTaskType::TASK_TYPE_PLACE;
  request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING;
  request.motion_goal_uuid = "place-retreat-one";

  EXPECT_FALSE(transport_->submit_normalized(request));
  EXPECT_EQ(transport_->place_retreat_planning_requests_, 1U);
  EXPECT_TRUE(transport_->retry_place_retreat());
  EXPECT_FALSE(transport_->retry_place_retreat());

  request.motion_goal_uuid = "place-retreat-two";
  EXPECT_FALSE(transport_->submit_normalized(request));
  EXPECT_EQ(transport_->place_retreat_planning_requests_, 1U);
  EXPECT_TRUE(transport_->retry_place_retreat());
}

}  // namespace
}  // namespace arm_cell_motion
