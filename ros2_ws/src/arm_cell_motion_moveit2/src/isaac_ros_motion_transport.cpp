#include "arm_cell_motion_moveit2/isaac_ros_motion_transport.hpp"
#include "motion_primitive_policy.hpp"

#include "arm_cell_motion_moveit2/cartesian_manipulation.hpp"
#include "arm_cell_motion_moveit2/grasp_contact_policy.hpp"
#include "arm_cell_motion_moveit2/gripper_port.hpp"
#include "arm_cell_motion_moveit2/place_approach_ik_policy.hpp"
#include "arm_cell_motion_moveit2/place_approach_trajectory.hpp"
#include "arm_cell_motion_moveit2/executable_trajectory_finalize.hpp"
#include "arm_cell_motion_moveit2/trajectory_continuity.hpp"
#include "arm_cell_motion_moveit2/trajectory_settling_diagnostics.hpp"
#include "arm_cell_motion_moveit2/trajectory_validation.hpp"
#include "arm_cell_motion_moveit2/motion_profile_trace.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <Eigen/Geometry>
#include <exception>
#include <future>
#include <iomanip>
#include <sstream>
#include <string>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include <arm_cell_interfaces/msg/motion_task_type.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp_action/rclcpp_action.hpp>

namespace arm_cell_motion_moveit2
{

namespace
{
constexpr auto kStateFreshness = std::chrono::milliseconds(500);
constexpr auto kMotionCommandRepublishPeriod = std::chrono::milliseconds(100);
constexpr auto kGripperStatusFreshness = std::chrono::milliseconds(500);
constexpr auto kMoveGroupServerWait = std::chrono::seconds(5);
constexpr double kPositionTolerance = 0.01;
constexpr double kSettlingDiagnosticPeriodS = 0.25;
constexpr double kFullyOpenWidthMm = 85.0;
constexpr double kManipulationCartesianStepM = 0.005;

geometry_msgs::msg::Quaternion quaternion_message(const Eigen::Quaterniond & value)
{
  geometry_msgs::msg::Quaternion result;
  result.x = value.x();
  result.y = value.y();
  result.z = value.z();
  result.w = value.w();
  return result;
}

void offset_tcp_insertion_axis(
  geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Vector3 & axis_tcp,
  double distance)
{
  const Eigen::Quaterniond rotation(
    pose.pose.orientation.w, pose.pose.orientation.x,
    pose.pose.orientation.y, pose.pose.orientation.z);
  const Eigen::Vector3d axis = rotation.normalized() *
    Eigen::Vector3d(axis_tcp.x, axis_tcp.y, axis_tcp.z);
  pose.pose.position.x -= distance * axis.x();
  pose.pose.position.y -= distance * axis.y();
  pose.pose.position.z -= distance * axis.z();
}

geometry_msgs::msg::PoseStamped offset_object_pose(
  const geometry_msgs::msg::PoseStamped & object_pose,
  const geometry_msgs::msg::Vector3 & direction_object, double distance)
{
  auto result = object_pose;
  const Eigen::Quaterniond rotation(
    object_pose.pose.orientation.w, object_pose.pose.orientation.x,
    object_pose.pose.orientation.y, object_pose.pose.orientation.z);
  const Eigen::Vector3d direction = rotation.normalized() * Eigen::Vector3d(
    direction_object.x, direction_object.y, direction_object.z);
  result.pose.position.x += distance * direction.x();
  result.pose.position.y += distance * direction.y();
  result.pose.position.z += distance * direction.z();
  return result;
}

geometry_msgs::msg::PoseStamped compose_object_tcp_pose(
  const geometry_msgs::msg::PoseStamped & object_pose,
  const geometry_msgs::msg::Transform & object_to_tcp)
{
  auto result = object_pose;
  const Eigen::Quaterniond object_rotation(
    object_pose.pose.orientation.w, object_pose.pose.orientation.x,
    object_pose.pose.orientation.y, object_pose.pose.orientation.z);
  const Eigen::Quaterniond relation_rotation(
    object_to_tcp.rotation.w, object_to_tcp.rotation.x,
    object_to_tcp.rotation.y, object_to_tcp.rotation.z);
  const auto offset = object_rotation.normalized() * Eigen::Vector3d(
    object_to_tcp.translation.x, object_to_tcp.translation.y,
    object_to_tcp.translation.z);
  result.pose.position.x += offset.x();
  result.pose.position.y += offset.y();
  result.pose.position.z += offset.z();
  const auto tcp_rotation = (object_rotation.normalized() * relation_rotation.normalized()).normalized();
  result.pose.orientation = quaternion_message(tcp_rotation);
  return result;
}

geometry_msgs::msg::Transform object_to_tcp_relation(
  const geometry_msgs::msg::PoseStamped & object_pose,
  const geometry_msgs::msg::PoseStamped & tcp_pose)
{
  geometry_msgs::msg::Transform relation;
  const Eigen::Quaterniond object_rotation(
    object_pose.pose.orientation.w, object_pose.pose.orientation.x,
    object_pose.pose.orientation.y, object_pose.pose.orientation.z);
  const Eigen::Quaterniond tcp_rotation(
    tcp_pose.pose.orientation.w, tcp_pose.pose.orientation.x,
    tcp_pose.pose.orientation.y, tcp_pose.pose.orientation.z);
  const auto object_inverse = object_rotation.normalized().conjugate();
  const Eigen::Vector3d offset(
    tcp_pose.pose.position.x - object_pose.pose.position.x,
    tcp_pose.pose.position.y - object_pose.pose.position.y,
    tcp_pose.pose.position.z - object_pose.pose.position.z);
  const auto local_offset = object_inverse * offset;
  relation.translation.x = local_offset.x();
  relation.translation.y = local_offset.y();
  relation.translation.z = local_offset.z();
  const auto relative_rotation = (object_inverse * tcp_rotation.normalized()).normalized();
  relation.rotation.x = relative_rotation.x();
  relation.rotation.y = relative_rotation.y();
  relation.rotation.z = relative_rotation.z();
  relation.rotation.w = relative_rotation.w();
  return relation;
}

std::vector<double> bounded_samples(double bound, double step)
{
  if (bound <= 0.0 || step <= 0.0) {
    return {0.0};
  }
  const auto count = static_cast<int>(std::floor(bound / step + 1e-9));
  std::vector<double> result;
  result.reserve(static_cast<std::size_t>(2 * count + 1));
  for (int index = -count; index <= count; ++index) {
    result.push_back(static_cast<double>(index) * step);
  }
  if (result.empty() || result.front() > -bound + 1e-9) {
    result.insert(result.begin(), -bound);
  }
  if (result.back() < bound - 1e-9) {
    result.push_back(bound);
  }
  return result;
}

std::vector<Eigen::Quaterniond> target_orientation_candidates(
  const Eigen::Quaterniond & nominal, double roll_tolerance, double pitch_tolerance,
  double roll_step, double pitch_step)
{
  const auto roll_samples = bounded_samples(roll_tolerance, roll_step);
  const auto pitch_samples = bounded_samples(pitch_tolerance, pitch_step);
  const auto nominal_zyx = nominal.normalized().toRotationMatrix().eulerAngles(2, 1, 0);
  const double nominal_yaw = nominal_zyx[0];
  const double nominal_pitch = nominal_zyx[1];
  const double nominal_roll = nominal_zyx[2];
  std::vector<Eigen::Quaterniond> candidates;
  candidates.reserve(roll_samples.size() * pitch_samples.size());
  for (const auto roll : roll_samples) {
    for (const auto pitch : pitch_samples) {
      if (roll == 0.0 && pitch == 0.0) {
        candidates.push_back(nominal.normalized());
        continue;
      }
      const Eigen::Quaterniond candidate(
        Eigen::AngleAxisd(nominal_yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(nominal_pitch + pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(nominal_roll + roll, Eigen::Vector3d::UnitX()));
      const auto normalized_candidate = candidate.normalized();
      if (std::none_of(candidates.begin(), candidates.end(), [&](const auto & existing) {
          return std::abs(existing.dot(normalized_candidate)) > 1.0 - 1e-12;
        }))
      {
        candidates.push_back(normalized_candidate);
      }
    }
  }
  return candidates;
}

struct PickOrientationCandidate
{
  geometry_msgs::msg::PoseStamped target_pose;
  geometry_msgs::msg::PoseStamped approach_pose;
  std::vector<double> joint_goal;
  double joint_distance{0.0};
  double nominal_deviation{0.0};
  std::size_t sample_index{0U};
};

std::string environment_or(const char * name, const std::string & fallback)
{
  const auto * value = std::getenv(name);
  return value && *value ? value : fallback;
}

std::vector<double> first_profile_joints(const std::vector<double> & values)
{
  return {values.begin(), values.begin() + std::min<std::size_t>(2, values.size())};
}

void log_move_group_request_scaling(
  const rclcpp::Logger & logger,
  moveit::planning_interface::MoveGroupInterface & move_group)
{
  moveit_msgs::msg::MotionPlanRequest request;
  move_group.constructMotionPlanRequest(request);
  RCLCPP_DEBUG(
    logger,
    "MoveIt planning profile diagnostics max_velocity_scaling_factor=%.6f "
    "max_acceleration_scaling_factor=%.6f",
    request.max_velocity_scaling_factor, request.max_acceleration_scaling_factor);
}

std::string build_commit_sha()
{
#ifdef ARM_CELL_BUILD_COMMIT_SHA
  return environment_or("GIT_COMMIT_SHA", ARM_CELL_BUILD_COMMIT_SHA);
#else
  return environment_or("GIT_COMMIT_SHA", "commit-sha-unreported");
#endif
}

std::string format_joint_values(
  const moveit::core::JointModelGroup * group, const moveit::core::RobotState & state)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(12);
  stream << "[";
  const auto & names = group->getVariableNames();
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (index != 0) {
      stream << ",";
    }
    stream << names[index] << "=" << state.getVariablePosition(names[index]);
  }
  stream << "]";
  return stream.str();
}

std::string format_double_values(const std::vector<double> & values)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(12);
  stream << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << ",";
    }
    stream << values[index];
  }
  stream << "]";
  return stream.str();
}

class MoveGroupPlanningTimeRestore final
{
public:
  explicit MoveGroupPlanningTimeRestore(
    moveit::planning_interface::MoveGroupInterface & move_group)
  : move_group_(move_group), original_time_s_(move_group.getPlanningTime()) {}

  ~MoveGroupPlanningTimeRestore()
  {
    move_group_.setPlanningTime(original_time_s_);
  }

  double original_time_s() const {return original_time_s_;}

private:
  moveit::planning_interface::MoveGroupInterface & move_group_;
  double original_time_s_;
};

std::string format_bool_values(const std::vector<bool> & values)
{
  std::ostringstream stream;
  stream << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << ",";
    }
    stream << (values[index] ? "true" : "false");
  }
  stream << "]";
  return stream.str();
}

std::string format_string_values(const std::vector<std::string> & values)
{
  std::ostringstream stream;
  stream << "[";
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << ",";
    }
    stream << values[index];
  }
  stream << "]";
  return stream.str();
}

bool wrap_equivalent_joint_policy(const std::string & joint_name)
{
  // Explicit manipulation policy: joints with the hardware range [-2pi, 2pi]
  // may use an equivalent representation. joint_3 is intentionally excluded.
  static constexpr std::array<const char *, 5> kWrapEquivalentJoints = {
    "joint_1", "joint_2", "joint_4", "joint_5", "joint_6"};
  return std::find(
    kWrapEquivalentJoints.begin(), kWrapEquivalentJoints.end(), joint_name) !=
         kWrapEquivalentJoints.end();
}

void log_ik_diagnostic(
  const rclcpp::Logger & logger,
  const std::string & label,
  const geometry_msgs::msg::PoseStamped & pose,
  const moveit::core::RobotState & seed,
  const moveit::core::JointModelGroup * group,
  const std::string & tip)
{
  moveit::core::RobotState solution(seed);
  const bool ik_success = solution.setFromIK(group, pose.pose, tip, 0.2);
  const bool bounds_ok = ik_success && solution.satisfiesBounds(group);
  RCLCPP_DEBUG(
    logger,
    "Motion IK diagnostic label=%s frame=%s xyz=(%.9f,%.9f,%.9f) quat=(%.9f,%.9f,%.9f,%.9f) "
    "ik_success=%s bounds_ok=%s solution=%s",
    label.c_str(), pose.header.frame_id.c_str(), pose.pose.position.x, pose.pose.position.y,
    pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y,
    pose.pose.orientation.z, pose.pose.orientation.w, ik_success ? "true" : "false",
    bounds_ok ? "true" : "false",
    ik_success ? format_joint_values(group, solution).c_str() : "none");
}
}

IsaacRosMotionTransport::IsaacRosMotionTransport(
  rclcpp::Node::SharedPtr node,
  std::vector<std::string> arm_joint_names,
  std::vector<double> home_joint_positions,
  std::vector<double> retract_joint_positions,
  bool initialize_move_group,
  bool validation_grasp_contact_policy_enabled,
  std::string validation_grasp_contact_object_id_prefix,
  std::vector<std::string> validation_grasp_contact_links,
  double planning_max_acceleration_rad_s2,
  std::string trajectory_sampler_name,
  double trajectory_clock_liveness_timeout_s,
  double approach_entry_position_tolerance_m,
  double approach_entry_orientation_tolerance_rad,
  bool approach_entry_require_settled,
  double approach_entry_settled_joint_delta_rad,
  std::size_t approach_entry_settled_samples,
  double motion_progress_stall_timeout_s,
  double place_tracking_tolerance_rad,
  double place_approach_entry_position_tolerance_m,
  double planning_velocity_scaling_factor,
  double planning_acceleration_scaling_factor,
  double planning_time_s,
  double place_candidate_planning_time_s,
  double place_total_planning_time_s)
: node_(std::move(node)),
  arm_joint_names_(std::move(arm_joint_names)),
  home_joint_positions_(std::move(home_joint_positions)),
  retract_joint_positions_(std::move(retract_joint_positions)),
  motion_progress_watchdog_(motion_progress_stall_timeout_s),
  validation_grasp_contact_policy_enabled_(validation_grasp_contact_policy_enabled),
  validation_grasp_contact_object_id_prefix_(
    std::move(validation_grasp_contact_object_id_prefix)),
  validation_grasp_contact_links_(std::move(validation_grasp_contact_links)),
  planning_max_acceleration_rad_s2_(planning_max_acceleration_rad_s2),
  planning_velocity_scaling_factor_(planning_velocity_scaling_factor),
  planning_acceleration_scaling_factor_(planning_acceleration_scaling_factor),
  planning_time_s_(planning_time_s),
  place_candidate_planning_time_s_(place_candidate_planning_time_s),
  place_total_planning_time_s_(place_total_planning_time_s),
  trajectory_sampler_name_(trajectory_sampler_name),
  place_tracking_tolerance_rad_(place_tracking_tolerance_rad),
  place_approach_entry_position_tolerance_m_(
    place_approach_entry_position_tolerance_m == -1.0 ?
    approach_entry_position_tolerance_m : place_approach_entry_position_tolerance_m),
  approach_entry_policy_{
    approach_entry_position_tolerance_m, approach_entry_orientation_tolerance_rad,
    approach_entry_require_settled},
  approach_settling_tracker_(
    approach_entry_settled_joint_delta_rad, approach_entry_settled_samples),
  trajectory_watchdog_(trajectory_clock_liveness_timeout_s)
{
  if (!std::isfinite(place_tracking_tolerance_rad_) || place_tracking_tolerance_rad_ <= 0.0) {
    throw std::invalid_argument("place tracking tolerance must be finite and positive");
  }
  if (!std::isfinite(place_approach_entry_position_tolerance_m_) ||
    place_approach_entry_position_tolerance_m_ <= 0.0)
  {
    throw std::invalid_argument(
            "place approach entry position tolerance must be finite and positive");
  }
  if (!std::isfinite(planning_velocity_scaling_factor_) ||
    planning_velocity_scaling_factor_ <= 0.0 || planning_velocity_scaling_factor_ > 1.0)
  {
    throw std::invalid_argument("planning velocity scaling factor must be in (0, 1]");
  }
  if (!std::isfinite(planning_acceleration_scaling_factor_) ||
    planning_acceleration_scaling_factor_ <= 0.0 ||
    planning_acceleration_scaling_factor_ > 1.0)
  {
    throw std::invalid_argument("planning acceleration scaling factor must be in (0, 1]");
  }
  if (!std::isfinite(planning_time_s_) || planning_time_s_ <= 0.0) {
    throw std::invalid_argument("planning time must be finite and positive");
  }
  if (initialize_move_group) {
    this->initialize_move_group();
  }
  joint_command_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_commands", rclcpp::QoS(10).reliable().durability_volatile());
  gripper_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
    "/gripper/command", 10);
  joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/isaac/joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr message) {
      on_joint_state(message);
    });
  current_fixture_object_id_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/pnp/current_fixture_object_id", rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::String::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(current_fixture_object_id_mutex_);
      current_fixture_object_id_ = message->data;
    });
  auto trajectory_sampler = make_trajectory_sampler(trajectory_sampler_name);
  if (!trajectory_sampler) {
    trajectory_sampler_configuration_error_ =
      "unsupported trajectory sampler: " + trajectory_sampler_name;
  }
  trajectory_worker_ = std::make_shared<TrajectoryExecutionWorker>(
    std::move(trajectory_sampler),
    [this](const TimedJointSample & sample) {
      return publish_timed_joint_target(sample);
    });
  profile_trace_ = std::make_shared<MotionProfileTrace>(
    environment_or("ARM_CELL_PROFILE_EVIDENCE_DIR", "/tmp/arm_cell_motion_profile"), 200000);
  trajectory_worker_->set_profile_trace(profile_trace_);
  simulation_clock_subscription_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::QoS(10),
    [this](const rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
      on_simulation_clock(message);
    });
  gripper_status_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/gripper/status", 10,
    [this](const std_msgs::msg::String::ConstSharedPtr message) {
      on_gripper_status(message);
    });
  planning_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene");
  apply_planning_scene_client_ = node_->create_client<moveit_msgs::srv::ApplyPlanningScene>(
    "/apply_planning_scene");
  state_validity_client_ = node_->create_client<moveit_msgs::srv::GetStateValidity>(
    "/check_state_validity");
}

IsaacRosMotionTransport::~IsaacRosMotionTransport()
{
  if (move_group_worker_.joinable()) {
    move_group_worker_.join();
  }
}

void IsaacRosMotionTransport::begin_pick_grasp_relation()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_object_to_tcp_valid_ = false;
  accepted_object_to_tcp_valid_ = false;
}

void IsaacRosMotionTransport::stage_selected_pick_grasp_relation(
  const geometry_msgs::msg::Transform & relation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_object_to_tcp_ = relation;
  pending_object_to_tcp_valid_ = true;
}

void IsaacRosMotionTransport::discard_pending_grasp_relation()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_object_to_tcp_valid_ = false;
}

bool IsaacRosMotionTransport::confirm_fresh_held_grasp_relation()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pending_object_to_tcp_valid_) {
    return false;
  }
  accepted_object_to_tcp_ = pending_object_to_tcp_;
  accepted_object_to_tcp_valid_ = true;
  pending_object_to_tcp_valid_ = false;
  RCLCPP_DEBUG(
    node_->get_logger(),
    "grasp relation diagnostic state=ACCEPTED object_to_tcp_translation=(%.6f,%.6f,%.6f) "
    "object_to_tcp_quaternion_xyzw=(%.6f,%.6f,%.6f,%.6f)",
    accepted_object_to_tcp_.translation.x,
    accepted_object_to_tcp_.translation.y,
    accepted_object_to_tcp_.translation.z,
    accepted_object_to_tcp_.rotation.x,
    accepted_object_to_tcp_.rotation.y,
    accepted_object_to_tcp_.rotation.z,
    accepted_object_to_tcp_.rotation.w);
  return true;
}

void IsaacRosMotionTransport::confirm_fresh_released_grasp_relation()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_object_to_tcp_valid_ = false;
  accepted_object_to_tcp_valid_ = false;
}

void IsaacRosMotionTransport::initialize_move_group()
{
  const auto state = move_group_state_;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    state->required = true;
    if (state->ready || state->in_progress) {
      return;
    }
    state->failure_reason.clear();
    state->in_progress = true;
  }

  if (move_group_worker_.joinable()) {
    move_group_worker_.join();
  }

  const auto node = node_;
  const auto velocity_scaling_factor = planning_velocity_scaling_factor_;
  const auto acceleration_scaling_factor = planning_acceleration_scaling_factor_;
  const auto planning_time_s = planning_time_s_;
  RCLCPP_DEBUG(node_->get_logger(), "MoveGroup initialization start");
  move_group_worker_ = std::thread(
    [state, node, velocity_scaling_factor, acceleration_scaling_factor, planning_time_s]() {
      try {
        using MoveGroupAction = moveit_msgs::action::MoveGroup;
        const auto discovery_deadline =
        std::chrono::steady_clock::now() + kMoveGroupServerWait;
        RCLCPP_DEBUG(node->get_logger(), "MoveGroup action discovery before");
        auto move_action_client = rclcpp_action::create_client<MoveGroupAction>(
          node, "move_action");
        const auto remaining = discovery_deadline - std::chrono::steady_clock::now();
        if (remaining <= std::chrono::steady_clock::duration::zero() ||
        !move_action_client->wait_for_action_server(
          std::chrono::duration_cast<std::chrono::nanoseconds>(remaining)))
        {
          throw std::runtime_error("MoveGroup action server /move_action unavailable");
        }
        RCLCPP_DEBUG(node->get_logger(), "MoveGroup action discovery complete");
        RCLCPP_DEBUG(
          node->get_logger(),
          "MoveGroup construction before wait_for_servers=%ld seconds",
          kMoveGroupServerWait.count());
        auto move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          node, "arm", std::shared_ptr<tf2_ros::Buffer>(),
          rclcpp::Duration::from_seconds(kMoveGroupServerWait.count()));
        RCLCPP_DEBUG(node->get_logger(), "MoveGroup construction returned");
        if (!move_group->getRobotModel()) {
          throw std::runtime_error("MoveGroup construction returned without a robot model");
        }
        move_group->setMaxVelocityScalingFactor(velocity_scaling_factor);
        move_group->setMaxAccelerationScalingFactor(acceleration_scaling_factor);
        move_group->setPlanningTime(planning_time_s);
        RCLCPP_DEBUG(
          node->get_logger(),
          "MoveGroup effective request settings: max_velocity_scaling_factor=%.6f "
          "max_acceleration_scaling_factor=%.6f planning_time_s=%.6f "
          "source=motion_node_parameter",
          velocity_scaling_factor, acceleration_scaling_factor, planning_time_s);
        RCLCPP_DEBUG(node->get_logger(), "MoveGroup post-construction robot model check passed");
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->in_progress) {
          state->move_group = std::move(move_group);
          state->ready = true;
          state->in_progress = false;
          state->failure_reason.clear();
          RCLCPP_INFO(node->get_logger(), "MoveGroup initialization ready");
        }
      } catch (const std::exception & exception) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->in_progress) {
          state->ready = false;
          state->in_progress = false;
          state->failure_reason = exception.what();
          RCLCPP_ERROR(
            node->get_logger(), "MoveGroup initialization failed: %s", exception.what());
        }
      } catch (...) {
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->in_progress) {
          state->ready = false;
          state->in_progress = false;
          state->failure_reason = "unknown MoveGroupInterface initialization exception";
          RCLCPP_ERROR(
            node->get_logger(), "MoveGroup initialization failed: unknown exception");
        }
      }
    });
}

bool IsaacRosMotionTransport::available() const
{
  std::unique_lock<std::mutex> lock(move_group_state_->mutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    return false;
  }
  return (!move_group_state_->required ||
         (move_group_state_->ready && !move_group_state_->in_progress &&
         move_group_state_->move_group)) && current_state_fresh();
}

bool IsaacRosMotionTransport::apply_runtime_tuning(const MotionTuning & tuning)
{
  const auto finite_positive = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
  if (!finite_positive(tuning.planning_time_s) ||
    !finite_positive(tuning.place_candidate_planning_time_s) ||
    !finite_positive(tuning.place_total_planning_time_s) ||
    !finite_positive(tuning.motion_progress_stall_timeout_s) ||
    !finite_positive(tuning.approach_entry_position_tolerance_m) ||
    !finite_positive(tuning.approach_entry_orientation_tolerance_rad) ||
    !finite_positive(tuning.place_approach_entry_position_tolerance_m) ||
    !finite_positive(tuning.place_tracking_tolerance_rad) ||
    !std::isfinite(tuning.planning_velocity_scaling_factor) ||
    tuning.planning_velocity_scaling_factor <= 0.0 ||
    tuning.planning_velocity_scaling_factor > 1.0 ||
    !std::isfinite(tuning.planning_acceleration_scaling_factor) ||
    tuning.planning_acceleration_scaling_factor <= 0.0 ||
    tuning.planning_acceleration_scaling_factor > 1.0)
  {
    return false;
  }
  planning_time_s_ = tuning.planning_time_s;
  place_candidate_planning_time_s_ = tuning.place_candidate_planning_time_s;
  place_total_planning_time_s_ = tuning.place_total_planning_time_s;
  planning_velocity_scaling_factor_ = tuning.planning_velocity_scaling_factor;
  planning_acceleration_scaling_factor_ = tuning.planning_acceleration_scaling_factor;
  motion_progress_watchdog_ = MotionProgressWatchdog(tuning.motion_progress_stall_timeout_s);
  approach_entry_policy_.position_tolerance_m = tuning.approach_entry_position_tolerance_m;
  approach_entry_policy_.orientation_tolerance_rad =
    tuning.approach_entry_orientation_tolerance_rad;
  place_approach_entry_position_tolerance_m_ =
    tuning.place_approach_entry_position_tolerance_m;
  place_tracking_tolerance_rad_ = tuning.place_tracking_tolerance_rad;
  return true;
}

bool IsaacRosMotionTransport::set_motion_envelope(const MotionEnvelope & envelope)
{
  if (!envelope.valid || !std::isfinite(envelope.max_velocity_scale) ||
    !std::isfinite(envelope.max_acceleration_scale) ||
    envelope.max_velocity_scale <= 0.0F || envelope.max_velocity_scale > 1.0F ||
    envelope.max_acceleration_scale <= 0.0F || envelope.max_acceleration_scale > 1.0F)
  {
    return false;
  }
  std::lock_guard<std::mutex> lock(motion_envelope_mutex_);
  safety_motion_envelope_ = envelope;
  return true;
}

MotionEnvelope IsaacRosMotionTransport::applied_motion_envelope() const
{
  std::lock_guard<std::mutex> lock(motion_envelope_mutex_);
  return safety_motion_envelope_;
}

std::string IsaacRosMotionTransport::availability_diagnostic() const
{
  const auto state = move_group_state_;
  const bool backend_node_initialized = static_cast<bool>(node_);
  bool move_group_ready = false;
  bool move_group_mutex_busy = false;
  bool joint_state_received = false;
  bool joint_state_fresh = false;
  bool required_arm_joints_present = false;
  bool gripper_status_received = false;
  bool gripper_status_fresh = false;
  bool gripper_ready = false;
  bool state_mutex_busy = false;
  std::string initialization_in_progress = "unknown";
  std::string initialization_failure = "unknown";

  std::unique_lock<std::mutex> move_group_lock(state->mutex, std::try_to_lock);
  if (!move_group_lock.owns_lock()) {
    move_group_mutex_busy = true;
  } else {
    move_group_ready = (!state->required ||
      (state->ready && !state->in_progress && state->move_group));
    initialization_in_progress = state->in_progress ? "true" : "false";
    initialization_failure = state->failure_reason.empty() ? "none" : state->failure_reason;
    std::unique_lock<std::mutex> state_lock(mutex_, std::try_to_lock);
    if (!state_lock.owns_lock()) {
      state_mutex_busy = true;
    } else {
      joint_state_received = latest_joint_state_ != nullptr;
      if (joint_state_received) {
        std::vector<double> measured_positions;
        required_arm_joints_present = extract_arm_positions(
          *latest_joint_state_, measured_positions);
        joint_state_fresh =
          std::chrono::steady_clock::now() - latest_state_receipt_ <= kStateFreshness;
      }
      gripper_status_received = gripper_status_sequence_ > 0;
      gripper_status_fresh = gripper_status_received &&
        std::chrono::steady_clock::now() - latest_gripper_status_receipt_ <=
        kGripperStatusFreshness;
      gripper_ready = gripper_status_fresh && gripper_runtime_ready_;
    }
  }

  bool ros_time_valid = false;
  if (node_) {
    ros_time_valid = node_->now().nanoseconds() > 0;
  }
  const bool transport_ready = move_group_ready && joint_state_received &&
    joint_state_fresh && required_arm_joints_present;
  std::ostringstream stream;
  stream << "move_group_ready=" << (move_group_ready ? "true" : "false")
         << " backend_node_initialized=" << (backend_node_initialized ? "true" : "false")
         << " joint_state_received=" << (joint_state_received ? "true" : "false")
         << " joint_state_fresh=" << (joint_state_fresh ? "true" : "false")
         << " required_arm_joints_present=" <<
    (required_arm_joints_present ? "true" : "false")
         << " gripper_status_received=" << (gripper_status_received ? "true" : "false")
         << " gripper_status_fresh=" << (gripper_status_fresh ? "true" : "false")
         << " gripper_ready=" << (gripper_ready ? "true" : "false")
         << " ros_time_valid=" << (ros_time_valid ? "true" : "false")
         << " init_in_progress=" << initialization_in_progress
         << " move_group_mutex_busy=" << (move_group_mutex_busy ? "true" : "false")
         << " state_mutex_busy=" << (state_mutex_busy ? "true" : "false")
         << " init_failure_reason=" << initialization_failure
         << " transport_ready=" << (transport_ready ? "true" : "false");
  return stream.str();
}

bool IsaacRosMotionTransport::submit(
  const arm_cell_interfaces::action::ExecuteTask::Goal & goal)
{
  using MotionTaskType = arm_cell_interfaces::msg::MotionTaskType;
  if (!available()) {
    return false;
  }
  const auto envelope = applied_motion_envelope();
  // These direct position-target paths do not produce a time-parameterized
  // trajectory, so they cannot attest reduced MoveIt scaling limits.
  if (envelope.max_velocity_scale < 1.0F || envelope.max_acceleration_scale < 1.0F) {
    return false;
  }
  if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME ||
    goal.task_type.value == MotionTaskType::TASK_TYPE_RETRACT)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    approach_acceptance_active_ = false;
    approach_expected_joint_positions_.clear();
    validated_approach_start_joint_positions_.clear();
    approach_start_state_validated_ = false;
    approach_entry_target_ = geometry_msgs::msg::PoseStamped{};
    approach_reference_completion_observed_ = false;
    approach_settling_tracker_.reset();
    active_phase_ = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_UNKNOWN;
    active_trajectory_is_place_ = false;
  }
  if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!publish_joint_target_locked(home_joint_positions_)) {
      return false;
    }
    begin_profile_trace(
      "GO_HOME", "command=single_position_target;completion=2_feedbacks;"
      "isaac_position_controller_profile=runtime_owned_unreported");
    go_home_profile_trace_active_ = true;
    if (has_simulation_time_) {
      profile_trace_->add_row(
        {
          latest_simulation_time_, "GO_HOME_COMMAND", home_joint_positions_, {}, {}, {}, {}});
    }
    return true;
  }
  if (goal.task_type.value == MotionTaskType::TASK_TYPE_RETRACT) {
    return publish_joint_target(retract_joint_positions_);
  }
  return false;
}

void IsaacRosMotionTransport::begin_profile_trace(
  const std::string & kind, const std::string & config)
{
  std::ostringstream run_id;
  const auto wall_tag = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  run_id << kind << '-' << wall_tag << '-' << ++next_profile_trace_id_;
  profile_trace_->begin(
    run_id.str(), environment_or("ISAAC_SIM_VERSION", "Isaac-runtime-version-unreported"),
    environment_or("ARM_CELL_PROFILE_ID", node_->get_name()), "ROS /clock simulation time",
    build_commit_sha());
  active_profile_configuration_ = config;
  RCLCPP_DEBUG(
    node_->get_logger(), "[motion profile] event=START run_id=%s kind=%s time_basis=sim_clock",
    run_id.str().c_str(), kind.c_str());
}

void IsaacRosMotionTransport::finish_profile_trace(const std::string & outcome)
{
  std::vector<double> actual;
  if ((outcome == "trajectory_execution_complete" ||
    outcome == "settled_by_existing_two_feedback_completion") && latest_joint_state_)
  {
    extract_arm_positions(*latest_joint_state_, actual);
  }
  std::vector<double> commanded;
  std::vector<double> commanded_velocity;
  if (outcome == "trajectory_execution_complete") {
    const auto diagnostics = trajectory_worker_->diagnostics();
    commanded = diagnostics.commanded_positions;
    commanded_velocity = diagnostics.commanded_velocity;
  } else if (outcome == "settled_by_existing_two_feedback_completion") {
    commanded = home_joint_positions_;
  }
  if (trajectory_worker_) {
    trajectory_worker_->set_profile_trace(nullptr);
  }
  auto snapshot = profile_trace_->take_snapshot();
  const auto joint_names = arm_joint_names_;
  const auto configuration = active_profile_configuration_ +
    ";planning_max_acceleration_rad_s2=" + std::to_string(planning_max_acceleration_rad_s2_) +
    ";moveit_velocity_scaling_factor=" +
    std::to_string(planning_velocity_scaling_factor_) +
    ";moveit_acceleration_scaling_factor=" +
    std::to_string(planning_acceleration_scaling_factor_) +
    ";place_tracking_tolerance_rad=" + std::to_string(place_tracking_tolerance_rad_);
  const auto logger = node_->get_logger();
  std::thread(
    [
      snapshot, joint_names, configuration, outcome, actual, commanded, commanded_velocity,
      logger]() {
      const auto result = snapshot->finish(outcome, joint_names, configuration);
      if (!result.success) {
        RCLCPP_ERROR(logger, "[motion profile] artifact write failed: %s", result.error.c_str());
        return;
      }
      RCLCPP_DEBUG(
        logger, "[motion profile] event=FINISH outcome=%s rows=%zu dropped=%zu "
        "simulation_span=%.6f command_joint_1_2=%s command_velocity_joint_1_2=%s "
        "actual_joint_1_2=%s csv=%s manifest=%s",
        outcome.c_str(), result.sample_count, result.dropped_sample_count,
        result.simulation_end - result.simulation_start,
        format_double_values(first_profile_joints(commanded)).c_str(),
        format_double_values(first_profile_joints(commanded_velocity)).c_str(),
        format_double_values(first_profile_joints(actual)).c_str(), result.samples_path.c_str(),
        result.manifest_path.c_str());
    }).detach();
}

TrajectorySampleResult IsaacRosMotionTransport::start_global_profiled_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory, std::uint64_t execution_id,
  double start_simulation_time, const std::string & config)
{
  bool capture_profile = false;
  std::lock_guard<std::mutex> lock(profile_capture_mutex_);
  if (global_profile_trace_active_) {
    return {false, "a GLOBAL profile capture is already active"};
  }
  capture_profile = !global_profile_capture_complete_;
  if (capture_profile) {
    begin_profile_trace("GLOBAL_MOVEIT", config);
    global_profile_trace_active_ = true;
  }
  trajectory_worker_->set_profile_trace(capture_profile ? profile_trace_ : nullptr);
  const auto started = trajectory_worker_->start(
    trajectory, execution_id, start_simulation_time);
  if (!started.valid && capture_profile) {
    global_profile_trace_active_ = false;
    trajectory_worker_->set_profile_trace(nullptr);
    finish_profile_trace("trajectory_worker_rejected");
  }
  return started;
}

void IsaacRosMotionTransport::finish_global_profile_capture(
  const std::string & outcome, bool completed)
{
  std::lock_guard<std::mutex> lock(profile_capture_mutex_);
  if (!global_profile_trace_active_) {
    return;
  }
  global_profile_trace_active_ = false;
  if (completed) {
    global_profile_capture_complete_ = true;
  }
  finish_profile_trace(outcome);
}

std::vector<double> IsaacRosMotionTransport::extract_arm_velocities(
  const sensor_msgs::msg::JointState & message) const
{
  std::vector<double> velocities;
  velocities.reserve(arm_joint_names_.size());
  for (const auto & name : arm_joint_names_) {
    const auto joint = std::find(message.name.begin(), message.name.end(), name);
    if (joint == message.name.end()) {return {};}
    const auto index = static_cast<std::size_t>(std::distance(message.name.begin(), joint));
    if (index >= message.velocity.size() || !std::isfinite(message.velocity[index])) {return {};}
    velocities.push_back(message.velocity[index]);
  }
  return velocities;
}

bool IsaacRosMotionTransport::apply_validation_grasp_contact_policy(
  moveit_msgs::msg::AllowedCollisionMatrix & original_acm)
{
  const auto reason_name = [](GraspContactPolicyReason reason) {
      switch (reason) {
        case GraspContactPolicyReason::ACCEPTED: return "ACCEPTED";
        case GraspContactPolicyReason::TARGET_NOT_FOUND: return "TARGET_NOT_FOUND";
        case GraspContactPolicyReason::TARGET_AMBIGUOUS: return "TARGET_AMBIGUOUS";
        case GraspContactPolicyReason::INVALID_CONFIG: return "INVALID_CONFIG";
        case GraspContactPolicyReason::SCENE_SERVICE_UNAVAILABLE:
          return "SCENE_SERVICE_UNAVAILABLE";
        case GraspContactPolicyReason::SCENE_READ_TIMEOUT: return "SCENE_READ_TIMEOUT";
        case GraspContactPolicyReason::ACM_APPLY_FAILED: return "ACM_APPLY_FAILED";
      }
      return "INVALID_CONFIG";
    };
  const auto join = [](const std::vector<std::string> & values) {
      std::ostringstream stream;
      stream << "[";
      for (std::size_t index = 0; index < values.size(); ++index) {
        if (index != 0U) {
          stream << ",";
        }
        stream << values[index];
      }
      stream << "]";
      return stream.str();
    };
  const auto emit = [&](const char * result, GraspContactPolicyReason reason,
      const GraspContactPolicyResolution & resolution, const char * read_status,
      const char * apply_status) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "grasp contact policy diagnostic result=%s reason=%s target_identity=%s "
        "resolved_collision_object_id=%s prefix_matches=%s authorized_links=%s "
        "planning_scene_read=%s planning_scene_apply=%s",
        result, reason_name(reason), resolution.target_identity.c_str(),
        resolution.resolved_collision_object_id.c_str(), join(resolution.prefix_matches).c_str(),
        join(resolution.authorized_links).c_str(), read_status, apply_status);
    };

  GraspContactPolicyResolution resolution;
  resolution.authorized_links = validation_grasp_contact_links_;
  std::string current_fixture_object_id;
  {
    std::lock_guard<std::mutex> lock(current_fixture_object_id_mutex_);
    current_fixture_object_id = current_fixture_object_id_;
  }
  resolution.target_identity = current_fixture_object_id;
  if (!validation_grasp_contact_policy_enabled_) {
    resolution.reason = GraspContactPolicyReason::INVALID_CONFIG;
    emit("REJECTED", resolution.reason, resolution, "NOT_REQUESTED", "NOT_REQUESTED");
    return false;
  }
  if (!valid_grasp_contact_config(
      validation_grasp_contact_object_id_prefix_, current_fixture_object_id,
      validation_grasp_contact_links_))
  {
    resolution.reason = GraspContactPolicyReason::INVALID_CONFIG;
    emit("REJECTED", resolution.reason, resolution, "NOT_READ", "NOT_APPLIED");
    return false;
  }
  if (!planning_scene_client_ || !apply_planning_scene_client_ ||
    !planning_scene_client_->wait_for_service(std::chrono::milliseconds(500)) ||
    !apply_planning_scene_client_->wait_for_service(std::chrono::milliseconds(500)))
  {
    resolution.reason = planning_scene_failure_reason(false, false, false);
    emit("REJECTED", resolution.reason, resolution, "UNAVAILABLE", "NOT_APPLIED");
    return false;
  }

  auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  scene_request->components.components = grasp_contact_policy_planning_scene_components();
  auto scene_future = planning_scene_client_->async_send_request(scene_request);
  if (scene_future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
    resolution.reason = planning_scene_failure_reason(true, false, false);
    emit("REJECTED", resolution.reason, resolution, "TIMEOUT", "NOT_APPLIED");
    return false;
  }
  const auto scene = scene_future.get();
  if (!scene) {
    resolution.reason = planning_scene_failure_reason(true, false, false);
    emit("REJECTED", resolution.reason, resolution, "FAILED", "NOT_APPLIED");
    return false;
  }
  original_acm = scene->scene.allowed_collision_matrix;
  resolution = resolve_current_target(
    scene->scene.world.collision_objects, validation_grasp_contact_object_id_prefix_,
    current_fixture_object_id, validation_grasp_contact_links_);
  if (resolution.reason != GraspContactPolicyReason::ACCEPTED) {
    emit("REJECTED", resolution.reason, resolution, "ACCEPTED", "NOT_APPLIED");
    return false;
  }
  const auto policy = allow_current_target_contacts(
    original_acm, resolution.resolved_collision_object_id, validation_grasp_contact_links_);
  if (!policy.has_value()) {
    resolution.reason = GraspContactPolicyReason::INVALID_CONFIG;
    emit("REJECTED", resolution.reason, resolution, "ACCEPTED", "NOT_APPLIED");
    return false;
  }

  auto apply_request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  apply_request->scene.is_diff = true;
  apply_request->scene.allowed_collision_matrix = *policy;
  auto apply_future = apply_planning_scene_client_->async_send_request(apply_request);
  if (apply_future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
    resolution.reason = planning_scene_failure_reason(true, true, false);
    emit("REJECTED", resolution.reason, resolution, "ACCEPTED", "TIMEOUT");
    return false;
  }
  const auto apply_response = apply_future.get();
  if (!apply_response || !apply_response->success) {
    resolution.reason = planning_scene_failure_reason(true, true, false);
    emit("REJECTED", resolution.reason, resolution, "ACCEPTED", "FAILED");
    return false;
  }
  resolution.reason = GraspContactPolicyReason::ACCEPTED;
  emit("ACCEPTED", resolution.reason, resolution, "ACCEPTED", "ACCEPTED");
  return true;
}

bool IsaacRosMotionTransport::restore_planning_scene_acm(
  const moveit_msgs::msg::AllowedCollisionMatrix & original_acm)
{
  if (!apply_planning_scene_client_ ||
    !apply_planning_scene_client_->wait_for_service(std::chrono::milliseconds(500)))
  {
    return false;
  }
  auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  request->scene.is_diff = true;
  request->scene.allowed_collision_matrix = original_acm;
  auto future = apply_planning_scene_client_->async_send_request(request);
  return future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready &&
         future.get()->success;
}

bool IsaacRosMotionTransport::submit_normalized(const NormalizedMotionRequest & input_request)
{
  auto request = input_request;
  if (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
    request.phase != arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING)
  {
    std::lock_guard<std::mutex> orientation_lock(mutex_);
    if (selected_pick_target_orientation_valid_) {
      request.target_pose.pose.orientation = selected_pick_target_orientation_;
      request.approach_target = request.target_pose;
      offset_tcp_insertion_axis(
        request.approach_target, request.insertion_axis_tcp, request.approach_distance_m);
      request.retract_target = request.target_pose;
      offset_tcp_insertion_axis(
        request.retract_target, request.insertion_axis_tcp,
        request.retract_distance_m);
      if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING) {
        request.tcp_target = request.target_pose;
      } else if (request.phase ==
        arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING)
      {
        request.tcp_target = request.retract_target;
      }
    }
  }
  if (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE) {
    std::lock_guard<std::mutex> place_pose_lock(mutex_);
    if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING) {
      selected_place_pose_valid_ = false;
      if (!accepted_object_to_tcp_valid_) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "Motion internal cause: no grasp relation was established by an accepted PICK target");
        return false;
      }
      request.object_to_grasp_tcp = accepted_object_to_tcp_;
    } else if (selected_place_pose_valid_) {
      request.object_pose = selected_place_object_pose_;
      request.target_pose = selected_place_target_pose_;
      request.approach_target = selected_place_approach_pose_;
      request.retract_target = selected_place_retract_pose_;
      if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING) {
        request.tcp_target = request.target_pose;
      } else if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING) {
        request.tcp_target = request.retract_target;
        // RELEASED has already ended the rigid grasp relation; the selected
        // absolute release pose is retained only for this final retreat.
        selected_place_pose_valid_ = false;
        pending_object_to_tcp_valid_ = false;
        accepted_object_to_tcp_valid_ = false;
      }
    }
  }
  std::lock_guard<std::mutex> lock(move_group_state_->mutex);
  const auto input_semantics = request.task_type.value ==
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ? "PICK_OBSERVATION_POSE" :
    (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE ?
    "PLACE_DESIRED_OBJECT_POSE" : "TASK_POSE");
  RCLCPP_DEBUG(
    node_->get_logger(),
    "motion pose diagnostic task_type=%u input_semantics=%s phase=%u observation_reference=%s observation_frame=%s "
    "observation=(%.9f,%.9f,%.9f;%.9f,%.9f,%.9f,%.9f) "
    "object_frame=%s object=(%.9f,%.9f,%.9f;%.9f,%.9f,%.9f,%.9f) "
    "target=(%.9f,%.9f,%.9f;%.9f,%.9f,%.9f,%.9f) "
    "approach=(%.9f,%.9f,%.9f;%.9f,%.9f,%.9f,%.9f) "
    "motion diagnostic submit phase=%u tcp=(%.9f,%.9f,%.9f) grasp=(%.9f,%.9f,%.9f) "
    "approach=(%.9f,%.9f,%.9f) tcp_q=(%.9f,%.9f,%.9f,%.9f) grasp_q=(%.9f,%.9f,%.9f,%.9f) "
    "approach_q=(%.9f,%.9f,%.9f,%.9f)",
    static_cast<unsigned>(request.task_type.value), input_semantics,
    static_cast<unsigned>(request.phase), request.observation_reference.c_str(),
    request.observation_pose.header.frame_id.c_str(),
    request.observation_pose.pose.position.x, request.observation_pose.pose.position.y,
    request.observation_pose.pose.position.z, request.observation_pose.pose.orientation.x,
    request.observation_pose.pose.orientation.y, request.observation_pose.pose.orientation.z,
    request.observation_pose.pose.orientation.w, request.object_pose.header.frame_id.c_str(),
    request.object_pose.pose.position.x, request.object_pose.pose.position.y,
    request.object_pose.pose.position.z, request.object_pose.pose.orientation.x,
    request.object_pose.pose.orientation.y, request.object_pose.pose.orientation.z,
    request.object_pose.pose.orientation.w, request.target_pose.pose.position.x,
    request.target_pose.pose.position.y, request.target_pose.pose.position.z,
    request.target_pose.pose.orientation.x, request.target_pose.pose.orientation.y,
    request.target_pose.pose.orientation.z, request.target_pose.pose.orientation.w,
    request.approach_target.pose.position.x, request.approach_target.pose.position.y,
    request.approach_target.pose.position.z, request.approach_target.pose.orientation.x,
    request.approach_target.pose.orientation.y, request.approach_target.pose.orientation.z,
    request.approach_target.pose.orientation.w,
    static_cast<unsigned>(request.phase), request.tcp_target.pose.position.x,
    request.tcp_target.pose.position.y, request.tcp_target.pose.position.z,
    request.target_pose.pose.position.x, request.target_pose.pose.position.y,
    request.target_pose.pose.position.z, request.approach_target.pose.position.x,
    request.approach_target.pose.position.y, request.approach_target.pose.position.z,
    request.tcp_target.pose.orientation.x, request.tcp_target.pose.orientation.y,
    request.tcp_target.pose.orientation.z, request.tcp_target.pose.orientation.w,
    request.target_pose.pose.orientation.x, request.target_pose.pose.orientation.y,
    request.target_pose.pose.orientation.z, request.target_pose.pose.orientation.w,
    request.approach_target.pose.orientation.x, request.approach_target.pose.orientation.y,
    request.approach_target.pose.orientation.z, request.approach_target.pose.orientation.w);
  if (move_group_state_->required && !move_group_state_->ready) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion diagnostic normalized submit rejected: MoveGroup not ready");
    return false;
  }
  if (!current_state_fresh()) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion diagnostic normalized submit rejected: current joint state is missing or stale");
    return false;
  }
  if (!move_group_state_->move_group) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion diagnostic normalized submit rejected: MoveGroupInterface is null");
    return false;
  }
  const auto envelope = applied_motion_envelope();
  const auto planning_scales = apply_motion_envelope_to_planning_scales(
    planning_velocity_scaling_factor_, planning_acceleration_scaling_factor_, envelope);
  move_group_state_->move_group->setMaxVelocityScalingFactor(planning_scales.max_velocity);
  move_group_state_->move_group->setMaxAccelerationScalingFactor(
    planning_scales.max_acceleration);
  {
    std::lock_guard<std::mutex> state_lock(mutex_);
    active_phase_ = request.phase;
    approach_acceptance_active_ = requires_approach_entry_acceptance(
      request.task_type.value, request.phase);
    active_trajectory_is_place_ =
      request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE;
    const bool insertion_phase = uses_validated_approach_start_state(
      request.task_type.value, request.phase);
    if (!approach_acceptance_active_ && !insertion_phase) {
      approach_expected_joint_positions_.clear();
      validated_approach_start_joint_positions_.clear();
      approach_start_state_validated_ = false;
      approach_reference_completion_observed_ = false;
      approach_entry_target_ = geometry_msgs::msg::PoseStamped{};
      approach_acceptance_candidate_valid_ = false;
      approach_settling_tracker_.reset();
    }
    if (approach_acceptance_active_) {
      final_target_hold_diagnostic_started_ = false;
      approach_entry_target_ = request.approach_target;
      approach_acceptance_candidate_valid_ = false;
      approach_acceptance_log_initialized_ = false;
      last_approach_acceptance_reason_.clear();
      last_approach_rejection_reason_.clear();
      last_approach_rejection_position_error_m_ = 0.0;
      last_approach_rejection_orientation_error_rad_ = 0.0;
      approach_expected_joint_positions_.clear();
      validated_approach_start_joint_positions_.clear();
      approach_start_state_validated_ = false;
      approach_settling_tracker_.reset();
      approach_reference_completion_observed_ = false;
    }
  }

  // Diagnostic-only B1-B5 probe. It observes the exact MoveGroup state and pose
  // variants; it must not change the request or planning behavior.
  const auto current_state = move_group_state_->move_group->getCurrentState(1.0);
  const auto * joint_group = move_group_state_->move_group->getRobotModel()->getJointModelGroup(
    "arm");
  if (!current_state || !joint_group) {
    RCLCPP_WARN(
      node_->get_logger(),
      "planning diagnostic current state or arm joint group unavailable");
    // The diagnostic probes below dereference both values. Fail this task
    // cleanly so a missing MoveIt state becomes a canonical backend result,
    // rather than crashing the motion node.
    return false;
  } else {
    moveit::core::RobotState exact_solution(*current_state);
    const bool exact_ik = exact_solution.setFromIK(
      joint_group, request.approach_target.pose, "sf_grasp_tcp", 0.2);
    const bool exact_bounds = exact_ik && exact_solution.satisfiesBounds(joint_group);
    RCLCPP_DEBUG(
      node_->get_logger(),
      "planning diagnostic approach IK=%s bounds=%s current=%s solution=%s",
      exact_ik ? "success" : "failure", exact_bounds ? "valid" : "invalid",
      format_joint_values(joint_group, *current_state).c_str(),
      exact_ik ? format_joint_values(joint_group, exact_solution).c_str() : "none");
    if (exact_ik) {
      std::ostringstream deltas;
      deltas << "[";
      const auto & names = joint_group->getVariableNames();
      for (std::size_t index = 0; index < names.size(); ++index) {
        if (index != 0) {
          deltas << ",";
        }
        const double delta = exact_solution.getVariablePosition(names[index]) -
          current_state->getVariablePosition(names[index]);
        deltas << names[index] << "=" << delta;
      }
      deltas << "]";
      RCLCPP_DEBUG(
        node_->get_logger(), "planning diagnostic approach current_to_solution_delta=%s",
        deltas.str().c_str());
    }
    log_ik_diagnostic(
      node_->get_logger(), "approach_yaw_zero", [&]() {
        auto pose = request.approach_target;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        return pose;
      }(), *current_state, joint_group, "sf_grasp_tcp");
    const Eigen::Quaterniond current_tcp_q(
      current_state->getGlobalLinkTransform("sf_grasp_tcp").rotation());
    auto current_orientation_pose = request.approach_target;
    current_orientation_pose.pose.orientation.x = current_tcp_q.x();
    current_orientation_pose.pose.orientation.y = current_tcp_q.y();
    current_orientation_pose.pose.orientation.z = current_tcp_q.z();
    current_orientation_pose.pose.orientation.w = current_tcp_q.w();
    log_ik_diagnostic(
      node_->get_logger(), "approach_current_tcp_orientation", current_orientation_pose,
      *current_state, joint_group, "sf_grasp_tcp");
    log_ik_diagnostic(
      node_->get_logger(), "grasp_exact", request.target_pose, *current_state,
      joint_group, "sf_grasp_tcp");
    log_ik_diagnostic(
      node_->get_logger(), "grasp_yaw_zero", [&]() {
        auto pose = request.target_pose;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        return pose;
      }(), *current_state, joint_group, "sf_grasp_tcp");
    auto grasp_current_orientation_pose = request.target_pose;
    grasp_current_orientation_pose.pose.orientation.x = current_tcp_q.x();
    grasp_current_orientation_pose.pose.orientation.y = current_tcp_q.y();
    grasp_current_orientation_pose.pose.orientation.z = current_tcp_q.z();
    grasp_current_orientation_pose.pose.orientation.w = current_tcp_q.w();
    log_ik_diagnostic(
      node_->get_logger(), "grasp_current_tcp_orientation", grasp_current_orientation_pose,
      *current_state, joint_group, "sf_grasp_tcp");

    if (planning_scene_client_->wait_for_service(std::chrono::milliseconds(200))) {
      auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
      scene_request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
      auto scene_future = planning_scene_client_->async_send_request(scene_request);
      if (scene_future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
        const auto scene = scene_future.get();
        RCLCPP_DEBUG(
          node_->get_logger(),
          "planning diagnostic scene summary world_object_count=%zu attached_object_count=%zu",
          scene->scene.world.collision_objects.size(),
          scene->scene.robot_state.attached_collision_objects.size());
      } else {
        RCLCPP_WARN(
          node_->get_logger(), "planning diagnostic get_planning_scene response timeout");
      }
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "planning diagnostic get_planning_scene unavailable");
    }

    const auto check_state_validity = [&](const std::string & label,
        const moveit::core::RobotState & state) {
        if (!state_validity_client_->wait_for_service(std::chrono::milliseconds(200))) {
          RCLCPP_WARN(
            node_->get_logger(), "planning diagnostic check_state_validity unavailable label=%s",
            label.c_str());
          return;
        }
        auto validity_request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        moveit::core::robotStateToRobotStateMsg(state, validity_request->robot_state, true);
        validity_request->group_name = "arm";
        auto validity_future = state_validity_client_->async_send_request(validity_request);
        if (validity_future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
          RCLCPP_WARN(
            node_->get_logger(),
            "planning diagnostic check_state_validity response timeout label=%s",
            label.c_str());
          return;
        }
        const auto validity = validity_future.get();
        RCLCPP_DEBUG(
          node_->get_logger(),
          "planning diagnostic state_validity label=%s valid=%s contacts=%zu",
          label.c_str(), validity->valid ? "true" : "false", validity->contacts.size());
        const auto contact_class_name = [](GraspContactClass classification) {
            switch (classification) {
              case GraspContactClass::EXPECTED_TARGET_FINGERTIP:
                return "EXPECTED_TARGET_FINGERTIP";
              case GraspContactClass::TARGET_UNAUTHORIZED_ROBOT_LINK:
                return "TARGET_UNAUTHORIZED_ROBOT_LINK";
              case GraspContactClass::ROBOT_SELF_CONTACT:
                return "ROBOT_SELF_CONTACT";
              case GraspContactClass::UNRELATED_WORLD_OBJECT:
                return "UNRELATED_WORLD_OBJECT";
              case GraspContactClass::UNRELATED_FIXTURE:
                return "UNRELATED_FIXTURE";
              case GraspContactClass::STALE_OR_DUPLICATE_COLLISION_OBJECT:
                return "STALE_OR_DUPLICATE_COLLISION_OBJECT";
            }
            return "STALE_OR_DUPLICATE_COLLISION_OBJECT";
          };
        for (const auto & contact : validity->contacts) {
          std::string target_object_id;
          for (const auto & body : {contact.contact_body_1, contact.contact_body_2}) {
            if (body.rfind(validation_grasp_contact_object_id_prefix_, 0) != 0) {
              continue;
            }
            std::lock_guard<std::mutex> lock(current_fixture_object_id_mutex_);
            if (body == current_fixture_object_id_) {
              target_object_id = body;
            }
          }
          const auto classification = classify_grasp_contact(
            contact.contact_body_1, contact.body_type_1, contact.contact_body_2,
            contact.body_type_2, target_object_id, validation_grasp_contact_links_);
          RCLCPP_DEBUG(
            node_->get_logger(),
            "planning diagnostic contact label=%s class=%s body1=%s type1=%u body2=%s type2=%u depth=%.9f "
            "point=(%.9f,%.9f,%.9f)",
            label.c_str(), contact_class_name(classification), contact.contact_body_1.c_str(),
            contact.body_type_1,
            contact.contact_body_2.c_str(), contact.body_type_2, contact.depth,
            contact.position.x, contact.position.y, contact.position.z);
        }
      };
    check_state_validity("current_measured", *current_state);
    auto home_state = *current_state;
    home_state.setJointGroupPositions("arm", home_joint_positions_);
    home_state.update();
    check_state_validity("go_home", home_state);
    if (exact_ik) {
      check_state_validity("approach_exact", exact_solution);
    }
    moveit::core::RobotState grasp_solution(*current_state);
    if (grasp_solution.setFromIK(
        joint_group, request.target_pose.pose, "sf_grasp_tcp", 0.2))
    {
      check_state_validity("grasp_exact", grasp_solution);
    }
    auto top_reference = request.target_pose;
    top_reference.pose.position.z = request.tcp_target.pose.position.z - 0.13;
    moveit::core::RobotState top_solution(*current_state);
    if (top_solution.setFromIK(joint_group, top_reference.pose, "sf_grasp_tcp", 0.2)) {
      log_ik_diagnostic(
        node_->get_logger(), "registered_top_reference", top_reference, *current_state,
        joint_group, "sf_grasp_tcp");
      check_state_validity("registered_top_reference", top_solution);
    } else {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "planning diagnostic registered_top_reference IK=failure");
    }
  }
  const bool grasp_contact_policy_requested =
    validation_grasp_contact_policy_enabled_ &&
    request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
    request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING;
  moveit_msgs::msg::AllowedCollisionMatrix original_acm;
  bool grasp_contact_policy_active = false;
  if (grasp_contact_policy_requested) {
    grasp_contact_policy_active = apply_validation_grasp_contact_policy(original_acm);
    if (!grasp_contact_policy_active) {
      RCLCPP_DEBUG(node_->get_logger(), "[Motion validation] grasp contact policy rejected closed");
      return false;
    }
  }
  auto restore_grasp_contact_policy = [&]() {
      if (!grasp_contact_policy_active) {
        return true;
      }
      const bool restored = restore_planning_scene_acm(original_acm);
      grasp_contact_policy_active = false;
      return restored;
    };

  const bool cartesian_primitive = uses_cartesian_manipulation(
    request.task_type.value, request.phase);
  const bool validated_approach_insertion = uses_validated_approach_start_state(
    request.task_type.value, request.phase);
  const char * primitive_type = cartesian_primitive ? "CARTESIAN" : "GLOBAL";
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  int planning_result = moveit::core::MoveItErrorCode::FAILURE;
  bool planning_succeeded = false;
  bool planning_precomputed = false;
  double path_fraction = 1.0;
  const auto robot_model = move_group_state_->move_group->getRobotModel();
  moveit::core::RobotState planning_start_state(*current_state);
  std::vector<double> measured_start;
  std::vector<JointContinuityLimit> continuity_limits;
  measured_start.reserve(arm_joint_names_.size());
  continuity_limits.reserve(arm_joint_names_.size());
  std::vector<double> expected_insertion_start;
  if (validated_approach_insertion) {
    std::lock_guard<std::mutex> state_lock(mutex_);
    if (!approach_start_state_validated_) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "insertion diagnostic rejected: APPROACH start state was not validated");
      restore_grasp_contact_policy();
      return false;
    }
    measured_start = validated_approach_start_joint_positions_;
    expected_insertion_start = approach_expected_joint_positions_;
    planning_start_state.setJointGroupPositions(joint_group, measured_start);
    planning_start_state.update();
    RCLCPP_DEBUG(
      node_->get_logger(),
      "insertion diagnostic start_state_sync=ACCEPTED expected_joints=%s measured_joints=%s",
      format_double_values(expected_insertion_start).c_str(),
      format_double_values(measured_start).c_str());
  }
  for (const auto & joint_name : arm_joint_names_) {
    const auto * joint_model = robot_model->getJointModel(joint_name);
    if (!joint_model || joint_model->getVariableBounds().empty()) {
      RCLCPP_DEBUG(
        node_->get_logger(), "motion diagnostic missing continuity bounds for %s", joint_name.c_str());
      return false;
    }
    const auto & bounds = joint_model->getVariableBounds().front();
    if (!validated_approach_insertion) {
      measured_start.push_back(current_state->getVariablePosition(joint_name));
    }
    continuity_limits.push_back(
      {
        bounds.min_position_, bounds.max_position_,
        wrap_equivalent_joint_policy(joint_name)});
  }
  std::vector<double> trajectory_velocity_limits;
  trajectory_velocity_limits.reserve(arm_joint_names_.size());
  for (const auto & joint_name : arm_joint_names_) {
    const auto * joint_model = robot_model->getJointModel(joint_name);
    if (!joint_model || joint_model->getVariableBounds().empty()) {
      RCLCPP_DEBUG(
        node_->get_logger(), "motion diagnostic missing executable velocity bounds for %s",
        joint_name.c_str());
      return false;
    }
    trajectory_velocity_limits.push_back(joint_model->getVariableBounds().front().max_velocity_);
  }
  const double trajectory_acceleration_limit = planning_max_acceleration_rad_s2_ > 0.0 ?
    planning_max_acceleration_rad_s2_ : std::numeric_limits<double>::max();
  const auto finalize_raw_trajectory = [&](moveit_msgs::msg::RobotTrajectory & trajectory) {
      return finalize_executable_trajectory(
        trajectory, arm_joint_names_, trajectory_velocity_limits,
        trajectory_acceleration_limit,
        [&](moveit_msgs::msg::RobotTrajectory & raw_trajectory) {
          robot_trajectory::RobotTrajectory timed_trajectory(robot_model, "arm");
          timed_trajectory.setRobotTrajectoryMsg(planning_start_state, raw_trajectory);
          trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
          if (!time_parameterization.computeTimeStamps(timed_trajectory, 1.0, 1.0)) {
            return false;
          }
          timed_trajectory.getRobotTrajectoryMsg(raw_trajectory);
          return true;
        });
    };
  JointTrajectoryContinuityResult continuity;
  std::vector<double> selected_global_goal;
  bool selected_global_goal_valid = false;
  move_group_state_->move_group->setStartState(planning_start_state);
  if (cartesian_primitive) {
    const auto start_transform = planning_start_state.getGlobalLinkTransform("sf_grasp_tcp");
    geometry_msgs::msg::Pose start_tcp;
    start_tcp.position.x = start_transform.translation().x();
    start_tcp.position.y = start_transform.translation().y();
    start_tcp.position.z = start_transform.translation().z();
    const Eigen::Quaterniond start_orientation(start_transform.rotation());
    start_tcp.orientation.x = start_orientation.x();
    start_tcp.orientation.y = start_orientation.y();
    start_tcp.orientation.z = start_orientation.z();
    start_tcp.orientation.w = start_orientation.w();
    start_tcp = cartesian_start_pose(
      start_tcp, request.tcp_target.pose, request.task_type.value);

    const auto validate_executable_state = [&](std::size_t sample_index,
        const std::vector<double> & sample_positions, std::string & reason,
        std::vector<std::string> & contact_bodies) {
        moveit::core::RobotState sample_state(planning_start_state);
        sample_state.setJointGroupPositions(joint_group, sample_positions);
        sample_state.update();
        if (!state_validity_client_->wait_for_service(std::chrono::milliseconds(200))) {
          reason = "collision/contact (state validity service unavailable)";
          return false;
        }
        auto validity_request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        moveit::core::robotStateToRobotStateMsg(sample_state, validity_request->robot_state, true);
        validity_request->group_name = "arm";
        auto validity_future = state_validity_client_->async_send_request(validity_request);
        if (validity_future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
          reason = "collision/contact (state validity timeout)";
          return false;
        }
        const auto validity = validity_future.get();
        if (!validity->valid) {
          for (const auto & contact : validity->contacts) {
            contact_bodies.push_back(contact.contact_body_1);
            contact_bodies.push_back(contact.contact_body_2);
          }
          RCLCPP_DEBUG(
            node_->get_logger(),
            "motion diagnostic Cartesian sample collision/contact sample=%zu contacts=%s",
            sample_index, format_string_values(contact_bodies).c_str());
          reason = "collision/contact";
          return false;
        }
        reason.clear();
        contact_bodies.clear();
        return true;
      };

    const auto solve_ik = [&](const geometry_msgs::msg::Pose & sample_tcp,
        const std::vector<double> & seed_positions,
        std::vector<double> & solution_positions, std::string & reason) {
        moveit::core::RobotState seed_state(planning_start_state);
        seed_state.setJointGroupPositions(joint_group, seed_positions);
        seed_state.update();
        moveit::core::RobotState solution_state(seed_state);
        if (!solution_state.setFromIK(joint_group, sample_tcp, "sf_grasp_tcp", 0.2)) {
          reason = "IK";
          return false;
        }
        if (!solution_state.satisfiesBounds(joint_group)) {
          reason = "bounds";
          return false;
        }
        solution_state.copyJointGroupPositions(joint_group, solution_positions);
        reason.clear();
        return true;
      };

    const auto check_collision = [&](std::size_t sample_index,
        const geometry_msgs::msg::Pose &, const std::vector<double> & sample_positions,
        std::string & reason) {
        std::vector<std::string> contact_bodies;
        return validate_executable_state(
          sample_index, sample_positions, reason, contact_bodies);
      };

    const auto generated = generate_cartesian_linear_trajectory(
      start_tcp, request.tcp_target.pose, measured_start, arm_joint_names_, continuity_limits,
      kManipulationCartesianStepM, solve_ik, check_collision);
    path_fraction = generated.path_fraction;
    planning_result = generated.valid ? moveit::core::MoveItErrorCode::SUCCESS :
      moveit::core::MoveItErrorCode::FAILURE;
    planning_succeeded = generated.valid;
    if (generated.valid) {
      plan.trajectory_.joint_trajectory = generated.trajectory;
      continuity = generated.continuity;
    } else {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "motion diagnostic Cartesian generation rejected sample=%zu tcp=(%.9f,%.9f,%.9f) reason=%s",
        generated.failure.sample_index, generated.failure.tcp.position.x,
        generated.failure.tcp.position.y, generated.failure.tcp.position.z,
        generated.failure.reason.c_str());
    }
    if (planning_succeeded) {
      const auto finalized = finalize_raw_trajectory(plan.trajectory_);
      planning_succeeded = finalized.success;
      if (planning_succeeded) {
        const auto final_validation = validate_cartesian_joint_trajectory(
          plan.trajectory_.joint_trajectory, measured_start, continuity_limits,
          validate_executable_state);
        if (!final_validation.valid) {
          RCLCPP_DEBUG(
            node_->get_logger(),
            "motion diagnostic post-TOTG Cartesian validation rejected sample=%zu reason=%s contacts=%s",
            final_validation.failure.sample_index, final_validation.failure.reason.c_str(),
            format_string_values(final_validation.failure.contact_bodies).c_str());
          restore_grasp_contact_policy();
          return false;
        }
        continuity = final_validation.continuity;
      } else {
        RCLCPP_DEBUG(
          node_->get_logger(), "motion diagnostic Cartesian finalize rejected: %s",
          finalized.reason.c_str());
      }
    }
  } else {
    const bool place_global_approach =
      request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
    if (place_global_approach) {
      PlaceApproachIkPolicy ik_policy;
      ik_policy.per_candidate_planning_budget_s = place_candidate_planning_time_s_;
      ik_policy.total_planning_budget_s = place_total_planning_time_s_;
      const auto seeds = generate_place_approach_ik_seeds(
        measured_start, arm_joint_names_, continuity_limits, ik_policy);
      const auto object_pose_candidates = generate_place_object_pose_candidates(
        request.object_pose, request.place_position_tolerance_m,
        request.place_orientation_tolerance_rad, request.place_orientation_constraint);
      const auto canonical_seed = canonicalize_ik_seed(
        measured_start, ik_policy.ik_seed_quantum_rad);
      const auto & desired_object_orientation = request.object_pose.pose.orientation;
      const auto & preferred_tool_orientation = request.place_tool_orientation_preference;
      RCLCPP_DEBUG(
        node_->get_logger(),
        "PLACE APPROACH diagnostic IK] measured_current_joints=%s canonical_ik_seed=%s "
        "ik_seed_quantum_rad=%.12f seed_count=%zu max_candidates=%zu "
        "ik_timeout_per_seed_s=%.3f desired_object=(%.9f,%.9f,%.9f) "
        "desired_object_orientation_xyzw=(%.9f,%.9f,%.9f,%.9f) "
        "tool_orientation_preference_present=%s preferred_tool_orientation_xyzw=(%.9f,%.9f,%.9f,%.9f) "
        "position_tolerance_m=%.9f orientation_tolerance_rad=%.9f pose_candidate_count=%zu",
        format_double_values(measured_start).c_str(),
        format_double_values(canonical_seed).c_str(), ik_policy.ik_seed_quantum_rad,
        seeds.size(), ik_policy.max_candidates, ik_policy.ik_timeout_s,
        request.object_pose.pose.position.x, request.object_pose.pose.position.y,
        request.object_pose.pose.position.z, desired_object_orientation.x,
        desired_object_orientation.y, desired_object_orientation.z, desired_object_orientation.w,
        request.has_place_tool_orientation_preference ? "true" : "false",
        preferred_tool_orientation.x, preferred_tool_orientation.y,
        preferred_tool_orientation.z, preferred_tool_orientation.w,
        request.place_position_tolerance_m,
        request.place_orientation_tolerance_rad, object_pose_candidates.size());
      if (seeds.empty() || object_pose_candidates.empty()) {
        restore_grasp_contact_policy();
        RCLCPP_WARN(
          node_->get_logger(),
          "PLACE planning exhausted reason=no_valid_pose_or_seed pose_candidates=%zu "
          "ik_candidates=0 planning_candidates=0 planned_candidates=0 "
          "ik_rejected=0 collision_rejected=0 other_rejected=0 planning_failed=0 "
          "trajectory_finalize_failed=0 selected=none",
          object_pose_candidates.size());
        return false;
      }

      struct PlaceGeometryCandidate
      {
        geometry_msgs::msg::PoseStamped target_pose;
        geometry_msgs::msg::PoseStamped approach_pose;
      };
      std::vector<PlaceGeometryCandidate> place_geometry;
      place_geometry.reserve(object_pose_candidates.size());
      for (const auto & object_candidate : object_pose_candidates) {
        PlaceGeometryCandidate geometry_candidate;
        geometry_candidate.target_pose = compose_object_tcp_pose(
          object_candidate.object_pose, request.object_to_grasp_tcp);
        geometry_candidate.approach_pose = compose_object_tcp_pose(
          offset_object_pose(
            object_candidate.object_pose, request.place_approach_direction_object,
            request.approach_distance_m), request.object_to_grasp_tcp);
        place_geometry.push_back(std::move(geometry_candidate));
      }

      std::vector<PlaceApproachIkCandidate> candidates;
      candidates.reserve(seeds.size() * place_geometry.size());
      for (std::size_t pose_index = 0; pose_index < place_geometry.size(); ++pose_index) {
        const auto & object_candidate = object_pose_candidates[pose_index];
        for (const auto & seed : seeds) {
          PlaceApproachIkCandidate candidate;
          candidate.object_pose_candidate_index = pose_index;
          candidate.nominal_pose_deviation =
            request.place_orientation_constraint == PlaceOrientationConstraint::FREE ? 0.0 :
            object_candidate.nominal_deviation;
          if (request.place_orientation_constraint != PlaceOrientationConstraint::FREE &&
            request.has_place_tool_orientation_preference)
          {
            candidate.tool_orientation_preference_error_rad =
              orientation_angular_distance_rad(
              place_geometry[pose_index].target_pose.pose.orientation,
              request.place_tool_orientation_preference);
            if (!candidate.tool_orientation_preference_error_rad) {
              candidate.rejection_reason = "TOOL_ORIENTATION_PREFERENCE_INVALID";
              candidates.push_back(std::move(candidate));
              continue;
            }
          }
          candidate.seed_index = seed.index;
          candidate.seed_source = seed.source;
          candidate.seed_positions = seed.joints;
          moveit::core::RobotState ik_seed_state(*current_state);
          ik_seed_state.setJointGroupPositions(joint_group, seed.joints);
          ik_seed_state.update();
          moveit::core::RobotState ik_solution(ik_seed_state);
          candidate.ik_success = ik_solution.setFromIK(
            joint_group, place_geometry[pose_index].approach_pose.pose,
            "sf_grasp_tcp", ik_policy.ik_timeout_s);
          if (!candidate.ik_success) {
            candidate.rejection_reason = "IK_FAILED";
            candidates.push_back(std::move(candidate));
            continue;
          }

          ik_solution.copyJointGroupPositions(joint_group, candidate.raw_solution);
          if (!prepare_place_approach_candidate(candidate, measured_start, continuity_limits)) {
            candidates.push_back(std::move(candidate));
            continue;
          }

          moveit::core::RobotState endpoint_state(*current_state);
          endpoint_state.setJointGroupPositions(joint_group, candidate.normalized_goal);
          endpoint_state.update();
          if (!state_validity_client_ ||
            !state_validity_client_->wait_for_service(std::chrono::milliseconds(200)))
          {
            candidate.rejection_reason = "state validity service unavailable";
            candidates.push_back(std::move(candidate));
            continue;
          }
          auto validity_request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
          moveit::core::robotStateToRobotStateMsg(
            endpoint_state, validity_request->robot_state, true);
          validity_request->group_name = "arm";
          auto validity_future = state_validity_client_->async_send_request(validity_request);
          if (validity_future.wait_for(std::chrono::milliseconds(500)) !=
            std::future_status::ready)
          {
            candidate.rejection_reason = "state validity response timeout";
            candidates.push_back(std::move(candidate));
            continue;
          }
          const auto validity = validity_future.get();
          candidate.endpoint_state_valid = validity->valid;
          for (const auto & contact : validity->contacts) {
            candidate.contact_bodies.push_back(contact.contact_body_1);
            candidate.contact_bodies.push_back(contact.contact_body_2);
          }
          if (!candidate.endpoint_state_valid) {
            candidate.rejection_reason = "collision/contact";
          }
          candidates.push_back(std::move(candidate));
        }
      }

      const auto ranked_candidates = rank_place_approach_candidates(
        candidates, measured_start, continuity_limits, {},
        ik_policy.duplicate_tolerance_rad);
      const bool bounded_place_orientation =
        request.place_orientation_constraint == PlaceOrientationConstraint::BOUNDED;
      const auto bounded_pose_groups = bounded_place_orientation ?
        group_bounded_place_pose_candidates(object_pose_candidates, request.object_pose) :
        BoundedPlacePoseCandidateGroups{};
      const auto partition_ranked_candidates = [&](
        const std::vector<std::size_t> & pose_indices) {
          std::vector<std::size_t> pose_ranked;
          for (const auto index : ranked_candidates) {
            if (std::find(pose_indices.begin(), pose_indices.end(),
              candidates[index].object_pose_candidate_index) != pose_indices.end())
            {
              pose_ranked.push_back(index);
            }
          }
          return interleave_place_candidates_by_pose(
            candidates, pose_ranked, object_pose_candidates.size());
      };
      const auto primary_bounded_candidates = bounded_place_orientation ?
        partition_ranked_candidates(bounded_pose_groups.primary_orientation_indices) :
        std::vector<std::size_t>{};
      const auto fallback_position_candidates = bounded_place_orientation ?
        partition_ranked_candidates(bounded_pose_groups.position_fallback_indices) :
        std::vector<std::size_t>{};
      const auto planning_candidate_indices = bounded_place_orientation ?
        primary_bounded_candidates : ranked_candidates;
      for (std::size_t candidate_index = 0;
        candidate_index < candidates.size(); ++candidate_index)
      {
        const auto & candidate = candidates[candidate_index];
        RCLCPP_DEBUG(
          node_->get_logger(),
          "PLACE APPROACH diagnostic candidate] candidate_index=%zu seed_index=%zu "
          "pose_candidate=%zu nominal_pose_deviation=%.9f seed_source=%s seed_joints=%s "
          "object=(%.9f,%.9f,%.9f;%.9f,%.9f,%.9f,%.9f) "
          "computed_tcp_orientation_xyzw=(%.9f,%.9f,%.9f,%.9f) "
          "tool_orientation_preference_error_rad=%.12f "
          "target_position=(%.9f,%.9f,%.9f) "
          "approach=(%.9f,%.9f,%.9f) ik_success=%s raw_ik_solution=%s "
          "normalized_goal=%s raw_delta=%s wrap_aware_delta=%s ranking_cost=%.12f "
          "endpoint_state_valid=%s contacts=%s duplicate_rejected=%s valid=%s reason=%s",
          candidate_index, candidate.seed_index, candidate.object_pose_candidate_index,
          candidate.nominal_pose_deviation, candidate.seed_source.c_str(),
          format_double_values(candidate.seed_positions).c_str(),
          object_pose_candidates[candidate.object_pose_candidate_index].object_pose.pose.position.x,
          object_pose_candidates[candidate.object_pose_candidate_index].object_pose.pose.position.y,
          object_pose_candidates[candidate.object_pose_candidate_index].object_pose.pose.position.z,
          object_pose_candidates[candidate.object_pose_candidate_index].object_pose.pose.orientation.x,
          object_pose_candidates[candidate.object_pose_candidate_index].object_pose.pose.orientation.y,
          object_pose_candidates[candidate.object_pose_candidate_index].object_pose.pose.orientation.z,
          object_pose_candidates[candidate.object_pose_candidate_index].object_pose.pose.orientation.w,
          place_geometry[candidate.object_pose_candidate_index].target_pose.pose.orientation.x,
          place_geometry[candidate.object_pose_candidate_index].target_pose.pose.orientation.y,
          place_geometry[candidate.object_pose_candidate_index].target_pose.pose.orientation.z,
          place_geometry[candidate.object_pose_candidate_index].target_pose.pose.orientation.w,
          candidate.tool_orientation_preference_error_rad.value_or(-1.0),
          place_geometry[candidate.object_pose_candidate_index].target_pose.pose.position.x,
          place_geometry[candidate.object_pose_candidate_index].target_pose.pose.position.y,
          place_geometry[candidate.object_pose_candidate_index].target_pose.pose.position.z,
          place_geometry[candidate.object_pose_candidate_index].approach_pose.pose.position.x,
          place_geometry[candidate.object_pose_candidate_index].approach_pose.pose.position.y,
          place_geometry[candidate.object_pose_candidate_index].approach_pose.pose.position.z,
          candidate.ik_success ? "true" : "false",
          format_double_values(candidate.raw_solution).c_str(),
          format_double_values(candidate.normalized_goal).c_str(),
          format_double_values(candidate.raw_delta).c_str(),
          format_double_values(candidate.wrap_aware_delta).c_str(), candidate.ranking_cost,
          candidate.endpoint_state_valid ? "true" : "false",
          format_string_values(candidate.contact_bodies).c_str(),
          candidate.duplicate_rejected ? "true" : "false",
          candidate.valid ? "true" : "false", candidate.rejection_reason.c_str());
      }
      if (ranked_candidates.empty()) {
        restore_grasp_contact_policy();
        const auto ik_rejected = std::count_if(
          candidates.begin(), candidates.end(), [](const auto & candidate) {
            return candidate.rejection_reason == "IK_FAILED" ||
                   candidate.rejection_reason == "IK_GOAL_INVALID";
          });
        const auto collision_rejected = std::count_if(
          candidates.begin(), candidates.end(), [](const auto & candidate) {
            return candidate.rejection_reason.find("collision/contact") != std::string::npos ||
                   candidate.rejection_reason == "ENDPOINT_STATE_INVALID";
          });
        const auto other_rejected = std::count_if(
          candidates.begin(), candidates.end(), [](const auto & candidate) {
            return !candidate.valid && candidate.rejection_reason != "IK_FAILED" &&
                   candidate.rejection_reason != "IK_GOAL_INVALID" &&
                   candidate.rejection_reason.find("collision/contact") == std::string::npos &&
                   candidate.rejection_reason != "ENDPOINT_STATE_INVALID";
          });
        RCLCPP_WARN(
          node_->get_logger(),
          "PLACE planning exhausted pose_candidates=%zu ik_candidates=%zu "
          "ik_rejected=%zu collision_rejected=%zu other_rejected=%zu "
          "planning_candidates=0 planned_candidates=0 planning_failed=0 "
          "trajectory_finalize_failed=0 selected=none",
          object_pose_candidates.size(), candidates.size(),
          static_cast<std::size_t>(ik_rejected),
          static_cast<std::size_t>(collision_rejected),
          static_cast<std::size_t>(other_rejected));
        return false;
      }

      const std::string planner_id = move_group_state_->move_group->getPlannerId();
      MoveGroupPlanningTimeRestore planning_time_restore(*move_group_state_->move_group);
      moveit::planning_interface::MoveGroupInterface::Plan selected_plan;
      std::unordered_map<std::size_t, moveit::planning_interface::MoveGroupInterface::Plan>
        candidate_plans;
      const auto plan_candidate =
        [&](const PlaceApproachIkCandidate & candidate, double budget_s) {
          moveit::planning_interface::MoveGroupInterface::Plan candidate_plan;
          move_group_state_->move_group->setPlanningTime(budget_s);
          moveit::core::RobotState selected_goal_state(*current_state);
          selected_goal_state.setJointGroupPositions(joint_group, candidate.normalized_goal);
          selected_goal_state.update();
          if (!move_group_state_->move_group->setJointValueTarget(selected_goal_state)) {
            return PlaceApproachPlanObservation{
              false, moveit::core::MoveItErrorCode::FAILURE, 0U, 0.0, 0.0, false,
              "planning_target_rejected"};
          }
          log_move_group_request_scaling(
            node_->get_logger(), *move_group_state_->move_group);
          const auto planning_start = std::chrono::steady_clock::now();
          const auto result = move_group_state_->move_group->plan(candidate_plan);
          const auto planning_duration = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - planning_start).count();
          auto trajectory_points = candidate_plan.trajectory_.joint_trajectory.points.size();
          const bool planning_success =
          result == moveit::core::MoveItErrorCode::SUCCESS && trajectory_points > 0U;
          double trajectory_duration_s = 0.0;
          bool time_parameterized = false;
          std::string time_parameterization_reason =
            planning_success ? "not_attempted" : "planning_failed";
          if (planning_success) {
            const auto finalized = finalize_raw_trajectory(candidate_plan.trajectory_);
            time_parameterized = finalized.success;
            trajectory_duration_s = finalized.duration_s;
            time_parameterization_reason = finalized.reason;
            if (time_parameterized) {
              trajectory_points = candidate_plan.trajectory_.joint_trajectory.points.size();
              const auto candidate_index = static_cast<std::size_t>(
                &candidate - candidates.data());
              candidate_plans.emplace(candidate_index, std::move(candidate_plan));
            }
          }
          return PlaceApproachPlanObservation{
            planning_success && time_parameterized, result.val, trajectory_points,
            planning_duration, trajectory_duration_s, time_parameterized,
            time_parameterization_reason};
        };
      const auto place_plan_result = bounded_place_orientation ?
        plan_place_candidates_with_fallback(
        candidates, primary_bounded_candidates, fallback_position_candidates, ik_policy,
        plan_candidate, PlacePlanSelectionPolicy::SHORTEST_PLANNING_DURATION,
        bounded_pose_groups.primary_orientation_indices.size(),
        bounded_pose_groups.position_fallback_indices.size()) :
        plan_ranked_place_approach_candidates(
        candidates, planning_candidate_indices, ik_policy, plan_candidate,
        PlacePlanSelectionPolicy::EXISTING_RANKING,
        0U);
      for (const auto & attempt : place_plan_result.attempts) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "PLACE APPROACH diagnostic planning] candidate_index=%zu seed_index=%zu "
          "planner_id=%s planning_budget_s=%.12f planning_duration_s=%.12f "
          "trajectory_duration_s=%.9f result=%d trajectory_points=%zu "
          "time_parameterized=%s time_parameterization_reason=%s success=%s",
          attempt.candidate_index, attempt.seed_index, planner_id.c_str(),
          attempt.planning_budget_s, attempt.planning_duration_s,
          attempt.trajectory_duration_s, attempt.result_code,
          attempt.trajectory_points, attempt.time_parameterized ? "true" : "false",
          attempt.time_parameterization_reason.c_str(), attempt.success ? "true" : "false");
      }
      planning_result = place_plan_result.attempts.empty() ?
        moveit::core::MoveItErrorCode::FAILURE :
        place_plan_result.attempts.back().result_code;
      planning_succeeded = place_plan_result.success;
      const auto ik_rejected = std::count_if(
        candidates.begin(), candidates.end(), [](const auto & candidate) {
          return candidate.rejection_reason == "IK_FAILED" ||
                 candidate.rejection_reason == "IK_GOAL_INVALID";
        });
      const auto collision_rejected = std::count_if(
        candidates.begin(), candidates.end(), [](const auto & candidate) {
          return candidate.rejection_reason.find("collision/contact") != std::string::npos ||
                 candidate.rejection_reason == "ENDPOINT_STATE_INVALID";
        });
      const auto other_rejected = std::count_if(
        candidates.begin(), candidates.end(), [](const auto & candidate) {
          return !candidate.valid && candidate.rejection_reason != "IK_FAILED" &&
                 candidate.rejection_reason != "IK_GOAL_INVALID" &&
                 candidate.rejection_reason.find("collision/contact") == std::string::npos &&
                 candidate.rejection_reason != "ENDPOINT_STATE_INVALID";
        });
      const auto planning_failed = std::count_if(
        place_plan_result.attempts.begin(), place_plan_result.attempts.end(),
        [](const auto & attempt) {
          return attempt.result_code != moveit::core::MoveItErrorCode::SUCCESS ||
                 attempt.trajectory_points == 0U;
        });
      const auto trajectory_finalize_failed = std::count_if(
        place_plan_result.attempts.begin(), place_plan_result.attempts.end(),
        [](const auto & attempt) {
          return attempt.result_code == moveit::core::MoveItErrorCode::SUCCESS &&
                 attempt.trajectory_points > 0U &&
                 !attempt.time_parameterized;
        });
      if (planning_succeeded) {
        const auto selected_candidate_index = place_plan_result.selected_candidate_index;
        auto selected_plan_iterator = candidate_plans.find(selected_candidate_index);
        if (selected_plan_iterator == candidate_plans.end()) {
          restore_grasp_contact_policy();
          RCLCPP_WARN(
            node_->get_logger(),
            "PLACE planning exhausted reason=selected_plan_not_retained "
            "pose_candidates=%zu ik_candidates=%zu ik_rejected=%zu "
            "collision_rejected=%zu other_rejected=%zu planning_candidates=%zu "
            "planned_candidates=%zu planning_failed=%zu trajectory_finalize_failed=%zu "
            "selected=none",
            object_pose_candidates.size(), candidates.size(),
            static_cast<std::size_t>(ik_rejected),
            static_cast<std::size_t>(collision_rejected),
            static_cast<std::size_t>(other_rejected),
            ranked_candidates.size(), place_plan_result.attempts.size(),
            static_cast<std::size_t>(planning_failed),
            static_cast<std::size_t>(trajectory_finalize_failed));
          return false;
        }
        plan = std::move(selected_plan_iterator->second);
        const auto & selected_candidate = candidates[selected_candidate_index];
        const auto selected_pose_index = selected_candidate.object_pose_candidate_index;
        double selected_trajectory_duration_s = std::numeric_limits<double>::infinity();
        for (const auto & attempt : place_plan_result.attempts) {
          if (attempt.candidate_index == selected_candidate_index) {
            selected_trajectory_duration_s = attempt.trajectory_duration_s;
            break;
          }
        }
        request.object_pose = object_pose_candidates[selected_pose_index].object_pose;
        request.target_pose = place_geometry[selected_pose_index].target_pose;
        request.approach_target = place_geometry[selected_pose_index].approach_pose;
        request.retract_target = compose_object_tcp_pose(
          offset_object_pose(
            request.object_pose, request.place_approach_direction_object,
            request.retract_distance_m), request.object_to_grasp_tcp);
        request.tcp_target = request.approach_target;
        selected_global_goal = selected_candidate.normalized_goal;
        selected_global_goal_valid = true;
        {
          std::lock_guard<std::mutex> place_pose_lock(mutex_);
          selected_place_object_pose_ = request.object_pose;
          selected_place_target_pose_ = request.target_pose;
          selected_place_approach_pose_ = request.approach_target;
          selected_place_retract_pose_ = request.retract_target;
          selected_place_pose_valid_ = true;
        }
        planning_result = moveit::core::MoveItErrorCode::SUCCESS;
        {
          std::lock_guard<std::mutex> state_lock(mutex_);
          // Use the planner-selected pose, not the recipe-derived nominal pose.
          approach_entry_target_ = selected_place_approach_pose_;
          approach_acceptance_candidate_index_ = selected_candidate_index;
          approach_acceptance_candidate_valid_ = true;
          approach_expected_joint_positions_ = selected_global_goal;
        }
        RCLCPP_DEBUG(
          node_->get_logger(),
          "PLACE planning selected candidate_index=%zu seed_index=%zu "
          "pose_candidate=%zu nominal_pose_deviation=%.9f "
          "tool_orientation_preference_error_rad=%.12f joint_distance_cost=%.12f "
          "trajectory_duration_s=%.9f object=(%.9f,%.9f,%.9f) "
          "computed_tcp_orientation_xyzw=(%.9f,%.9f,%.9f,%.9f) "
          "approach=(%.9f,%.9f,%.9f) seed_source=%s normalized_goal=%s "
          "pose_candidates=%zu ik_candidates=%zu ik_rejected=%zu collision_rejected=%zu "
          "other_rejected=%zu planning_candidates=%zu planned_candidates=%zu "
          "planning_failed=%zu trajectory_finalize_failed=%zu "
          "selection=%s "
          "elapsed_planning_budget_s=%.12f",
          place_plan_result.selected_candidate_index,
          selected_candidate.seed_index, selected_pose_index,
          selected_candidate.nominal_pose_deviation,
          selected_candidate.tool_orientation_preference_error_rad.value_or(-1.0),
          selected_candidate.ranking_cost, selected_trajectory_duration_s,
          request.object_pose.pose.position.x, request.object_pose.pose.position.y,
          request.object_pose.pose.position.z,
          request.target_pose.pose.orientation.x, request.target_pose.pose.orientation.y,
          request.target_pose.pose.orientation.z, request.target_pose.pose.orientation.w,
          request.approach_target.pose.position.x, request.approach_target.pose.position.y,
          request.approach_target.pose.position.z, selected_candidate.seed_source.c_str(),
          format_double_values(selected_global_goal).c_str(),
          object_pose_candidates.size(), candidates.size(),
          static_cast<std::size_t>(ik_rejected),
          static_cast<std::size_t>(collision_rejected),
          static_cast<std::size_t>(other_rejected), ranked_candidates.size(),
          place_plan_result.attempts.size(),
          static_cast<std::size_t>(planning_failed),
          static_cast<std::size_t>(trajectory_finalize_failed),
          bounded_place_orientation ? "shortest_planning_duration" :
          "tool_orientation_error_then_pose_deviation_then_joint_cost_then_trajectory_duration",
          place_plan_result.elapsed_planning_s);
      } else {
        RCLCPP_WARN(
          node_->get_logger(),
          "PLACE planning exhausted pose_candidates=%zu ik_candidates=%zu "
          "ik_rejected=%zu collision_rejected=%zu other_rejected=%zu "
          "planning_candidates=%zu planned_candidates=%zu planning_failed=%zu "
          "trajectory_finalize_failed=%zu "
          "selected=none elapsed_planning_budget_s=%.12f total_budget_s=%.12f",
          object_pose_candidates.size(), candidates.size(),
          static_cast<std::size_t>(ik_rejected),
          static_cast<std::size_t>(collision_rejected),
          static_cast<std::size_t>(other_rejected), ranked_candidates.size(),
          place_plan_result.attempts.size(),
          static_cast<std::size_t>(planning_failed),
          static_cast<std::size_t>(trajectory_finalize_failed),
          place_plan_result.elapsed_planning_s, ik_policy.total_planning_budget_s);
      }
    } else if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING) {
      const auto validate_goal_state = [&](const std::vector<double> & positions,
          std::string & reason) {
          moveit::core::RobotState goal_state(*current_state);
          goal_state.setJointGroupPositions(joint_group, positions);
          goal_state.update();
          if (!state_validity_client_->wait_for_service(std::chrono::milliseconds(200))) {
            reason = "collision/contact (state validity service unavailable)";
            return false;
          }
          auto validity_request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
          moveit::core::robotStateToRobotStateMsg(goal_state, validity_request->robot_state, true);
          validity_request->group_name = "arm";
          auto validity_future = state_validity_client_->async_send_request(validity_request);
          if (validity_future.wait_for(std::chrono::milliseconds(500)) !=
            std::future_status::ready)
          {
            reason = "collision/contact (state validity timeout)";
            return false;
          }
          const auto validity = validity_future.get();
          if (!validity->valid) {
            std::vector<std::string> contact_bodies;
            for (const auto & contact : validity->contacts) {
              contact_bodies.push_back(contact.contact_body_1);
              contact_bodies.push_back(contact.contact_body_2);
            }
            RCLCPP_DEBUG(
              node_->get_logger(),
              "motion diagnostic GLOBAL APPROACH selected goal collision/contact contacts=%s",
              format_string_values(contact_bodies).c_str());
            reason = "collision/contact";
            return false;
          }
          reason.clear();
          return true;
        };
      const bool pick_approach = request.task_type.value ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK;
      const Eigen::Quaterniond nominal_orientation(
        request.target_pose.pose.orientation.w, request.target_pose.pose.orientation.x,
        request.target_pose.pose.orientation.y, request.target_pose.pose.orientation.z);
      const auto orientation_candidates = pick_approach ?
        target_orientation_candidates(
        nominal_orientation, request.target_roll_tolerance_rad,
        request.target_pitch_tolerance_rad, request.target_roll_sample_step_rad,
        request.target_pitch_sample_step_rad) :
        std::vector<Eigen::Quaterniond>{nominal_orientation.normalized()};
      std::vector<PickOrientationCandidate> feasible_candidates;
      std::size_t ik_rejected = 0U;
      std::size_t collision_rejected = 0U;
      std::size_t other_rejected = 0U;
      for (std::size_t candidate_index = 0;
        candidate_index < orientation_candidates.size(); ++candidate_index)
      {
        auto candidate_target = request.target_pose;
        candidate_target.pose.orientation = quaternion_message(orientation_candidates[candidate_index]);
        moveit::core::RobotState target_solution(*current_state);
        if (!target_solution.setFromIK(
            joint_group, candidate_target.pose, "sf_grasp_tcp", 0.2) ||
          !target_solution.satisfiesBounds(joint_group))
        {
          ++ik_rejected;
          continue;
        }
        std::vector<double> target_positions;
        target_solution.copyJointGroupPositions(joint_group, target_positions);
        std::string validity_reason;
        if (!validate_goal_state(target_positions, validity_reason)) {
          if (validity_reason.find("collision/contact") != std::string::npos) {
            ++collision_rejected;
          } else {
            ++other_rejected;
          }
          continue;
        }
        auto candidate_approach = candidate_target;
        offset_tcp_insertion_axis(
          candidate_approach, request.insertion_axis_tcp, request.approach_distance_m);
        moveit::core::RobotState approach_solution(*current_state);
        if (!approach_solution.setFromIK(
            joint_group, candidate_approach.pose, "sf_grasp_tcp", 0.2) ||
          !approach_solution.satisfiesBounds(joint_group))
        {
          ++ik_rejected;
          continue;
        }
        std::vector<double> raw_approach_positions;
        approach_solution.copyJointGroupPositions(joint_group, raw_approach_positions);
        const auto selected_goal = select_nearest_joint_goal(
          raw_approach_positions, measured_start, continuity_limits, validate_goal_state);
        if (!selected_goal.valid) {
          if (selected_goal.reason.find("collision/contact") != std::string::npos) {
            ++collision_rejected;
          } else {
            ++other_rejected;
          }
          continue;
        }
        double joint_distance_squared = 0.0;
        for (std::size_t joint_index = 0; joint_index < measured_start.size(); ++joint_index) {
          const auto delta = std::remainder(
            selected_goal.selected_goal[joint_index] - measured_start[joint_index],
            6.28318530717958647692);
          joint_distance_squared += delta * delta;
        }
        const auto nominal_deviation = 2.0 * std::acos(std::clamp(
          std::abs(nominal_orientation.normalized().dot(orientation_candidates[candidate_index])),
          0.0, 1.0));
        feasible_candidates.push_back(PickOrientationCandidate{
          candidate_target, candidate_approach, selected_goal.selected_goal,
          std::sqrt(joint_distance_squared), nominal_deviation, candidate_index});
      }
      if (feasible_candidates.empty()) {
        restore_grasp_contact_policy();
        RCLCPP_WARN(
          node_->get_logger(),
          "%s planning exhausted pose_candidates=%zu ik_rejected=%zu "
          "collision_rejected=%zu other_rejected=%zu planning_failed=0 "
          "trajectory_finalize_failed=0 selected=none",
          pick_approach ? "PICK" : "PLACE", orientation_candidates.size(),
          ik_rejected, collision_rejected, other_rejected);
        return false;
      }
      std::sort(
        feasible_candidates.begin(), feasible_candidates.end(),
        [](const auto & lhs, const auto & rhs) {
          if (std::abs(lhs.joint_distance - rhs.joint_distance) > 1e-9) {
            return lhs.joint_distance < rhs.joint_distance;
          }
          return lhs.nominal_deviation < rhs.nominal_deviation;
        });
      const std::size_t planning_candidate_count = std::min<std::size_t>(
        feasible_candidates.size(), pick_approach ? 3U : 1U);
      bool selected_candidate_found = false;
      double selected_joint_distance = std::numeric_limits<double>::infinity();
      double selected_duration_s = std::numeric_limits<double>::infinity();
      double selected_nominal_deviation = std::numeric_limits<double>::infinity();
      std::size_t selected_candidate_index = 0U;
      moveit::planning_interface::MoveGroupInterface::Plan selected_plan;
      std::vector<double> selected_goal;
      std::size_t planning_failed = 0U;
      std::size_t trajectory_finalize_failed = 0U;
      for (std::size_t rank = 0; rank < planning_candidate_count; ++rank) {
        const auto & candidate = feasible_candidates[rank];
        moveit::core::RobotState candidate_goal_state(*current_state);
        candidate_goal_state.setJointGroupPositions(joint_group, candidate.joint_goal);
        candidate_goal_state.update();
        moveit::planning_interface::MoveGroupInterface::Plan candidate_plan;
        const auto planning_started = std::chrono::steady_clock::now();
        const bool target_set = move_group_state_->move_group->setJointValueTarget(
          candidate_goal_state);
        const auto result = target_set ? move_group_state_->move_group->plan(candidate_plan) :
          moveit::core::MoveItErrorCode::FAILURE;
        const auto planning_duration_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - planning_started).count();
        bool plan_valid = result == moveit::core::MoveItErrorCode::SUCCESS &&
          !candidate_plan.trajectory_.joint_trajectory.points.empty();
        double trajectory_duration_s = std::numeric_limits<double>::infinity();
        std::string finalize_reason = plan_valid ? "not_attempted" : "planning_failed";
        if (plan_valid) {
          const auto finalized = finalize_raw_trajectory(candidate_plan.trajectory_);
          plan_valid = finalized.success;
          trajectory_duration_s = finalized.duration_s;
          finalize_reason = finalized.reason;
        }
        if (!plan_valid) {
          if (result == moveit::core::MoveItErrorCode::SUCCESS &&
            !candidate_plan.trajectory_.joint_trajectory.points.empty())
          {
            ++trajectory_finalize_failed;
          } else {
            ++planning_failed;
          }
        }
        const bool better = plan_valid && (!selected_candidate_found ||
          candidate.joint_distance < selected_joint_distance - 1e-9 ||
          (std::abs(candidate.joint_distance - selected_joint_distance) <= 1e-9 &&
          (trajectory_duration_s < selected_duration_s - 1e-9 ||
          (std::abs(trajectory_duration_s - selected_duration_s) <= 1e-9 &&
          candidate.nominal_deviation < selected_nominal_deviation))));
        RCLCPP_DEBUG(
          node_->get_logger(),
          "orientation planning diagnostic sample=%zu rank=%zu target_xyz=(%.9f,%.9f,%.9f) "
          "quaternion=(%.9f,%.9f,%.9f,%.9f) joint_distance=%.9f "
          "planning_duration_s=%.6f trajectory_duration_s=%.6f result=%d time_parameterized=%s finalize_reason=%s selected=%s",
          candidate.sample_index, rank, candidate.target_pose.pose.position.x,
          candidate.target_pose.pose.position.y, candidate.target_pose.pose.position.z,
          candidate.target_pose.pose.orientation.x, candidate.target_pose.pose.orientation.y,
          candidate.target_pose.pose.orientation.z, candidate.target_pose.pose.orientation.w,
          candidate.joint_distance, planning_duration_s, trajectory_duration_s, result.val,
          plan_valid ? "true" : "false", finalize_reason.c_str(), better ? "true" : "false");
        if (better) {
          selected_candidate_found = true;
          selected_candidate_index = rank;
          selected_joint_distance = candidate.joint_distance;
          selected_duration_s = trajectory_duration_s;
          selected_nominal_deviation = candidate.nominal_deviation;
          selected_plan = std::move(candidate_plan);
          selected_goal = candidate.joint_goal;
        }
      }
      if (!selected_candidate_found) {
        restore_grasp_contact_policy();
        RCLCPP_WARN(
          node_->get_logger(),
          "%s planning exhausted pose_candidates=%zu ik_rejected=%zu "
          "collision_rejected=%zu other_rejected=%zu planning_failed=%zu "
          "trajectory_finalize_failed=%zu selected=none planning_candidates=%zu",
          pick_approach ? "PICK" : "PLACE", orientation_candidates.size(),
          ik_rejected, collision_rejected, other_rejected, planning_failed,
          trajectory_finalize_failed, planning_candidate_count);
        return false;
      }
      const auto & selected_candidate = feasible_candidates[selected_candidate_index];
      request.target_pose = selected_candidate.target_pose;
      request.approach_target = selected_candidate.approach_pose;
      request.tcp_target = selected_candidate.approach_pose;
      selected_global_goal = selected_goal;
      selected_global_goal_valid = true;
      plan = std::move(selected_plan);
      planning_result = moveit::core::MoveItErrorCode::SUCCESS;
      planning_succeeded = true;
      planning_precomputed = true;
      if (pick_approach) {
        {
          std::lock_guard<std::mutex> orientation_lock(mutex_);
          selected_pick_target_orientation_ = selected_candidate.target_pose.pose.orientation;
          selected_pick_target_orientation_valid_ = true;
        }
        stage_selected_pick_grasp_relation(object_to_tcp_relation(
            request.object_pose, selected_candidate.target_pose));
      }
      {
        std::lock_guard<std::mutex> state_lock(mutex_);
        approach_entry_target_ = selected_candidate.approach_pose;
        if (request.task_type.value ==
          arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE)
        {
          approach_acceptance_candidate_index_ = selected_candidate.sample_index;
          approach_acceptance_candidate_valid_ = true;
        }
        approach_expected_joint_positions_ = selected_goal;
      }
      RCLCPP_DEBUG(
        node_->get_logger(),
        "%s planning selected candidate=%zu target_xyz=(%.9f,%.9f,%.9f) "
        "target_quaternion=(%.9f,%.9f,%.9f,%.9f) approach_xyz=(%.9f,%.9f,%.9f) "
        "insertion_axis_base=(%.9f,%.9f,%.9f) approach_to_target_distance_m=%.6f "
        "joint_distance=%.9f trajectory_duration_s=%.6f selected_rank=%zu of %zu "
        "selection=joint_distance_then_trajectory_duration_then_nominal_deviation "
        "pose_candidates=%zu ik_rejected=%zu collision_rejected=%zu other_rejected=%zu "
        "planning_failed=%zu trajectory_finalize_failed=%zu",
        pick_approach ? "PICK" : "PLACE", selected_candidate.sample_index,
        selected_candidate.target_pose.pose.position.x,
        selected_candidate.target_pose.pose.position.y, selected_candidate.target_pose.pose.position.z,
        selected_candidate.target_pose.pose.orientation.x,
        selected_candidate.target_pose.pose.orientation.y,
        selected_candidate.target_pose.pose.orientation.z,
        selected_candidate.target_pose.pose.orientation.w,
        selected_candidate.approach_pose.pose.position.x,
        selected_candidate.approach_pose.pose.position.y,
        selected_candidate.approach_pose.pose.position.z,
        (Eigen::Quaterniond(
          selected_candidate.target_pose.pose.orientation.w,
          selected_candidate.target_pose.pose.orientation.x,
          selected_candidate.target_pose.pose.orientation.y,
          selected_candidate.target_pose.pose.orientation.z).normalized() *
        Eigen::Vector3d(
          request.insertion_axis_tcp.x, request.insertion_axis_tcp.y,
          request.insertion_axis_tcp.z)).x(),
        (Eigen::Quaterniond(
          selected_candidate.target_pose.pose.orientation.w,
          selected_candidate.target_pose.pose.orientation.x,
          selected_candidate.target_pose.pose.orientation.y,
          selected_candidate.target_pose.pose.orientation.z).normalized() *
        Eigen::Vector3d(
          request.insertion_axis_tcp.x, request.insertion_axis_tcp.y,
          request.insertion_axis_tcp.z)).y(),
        (Eigen::Quaterniond(
          selected_candidate.target_pose.pose.orientation.w,
          selected_candidate.target_pose.pose.orientation.x,
          selected_candidate.target_pose.pose.orientation.y,
          selected_candidate.target_pose.pose.orientation.z).normalized() *
        Eigen::Vector3d(
          request.insertion_axis_tcp.x, request.insertion_axis_tcp.y,
          request.insertion_axis_tcp.z)).z(),
        request.approach_distance_m,
        selected_joint_distance, selected_duration_s, selected_candidate_index + 1U,
        planning_candidate_count, orientation_candidates.size(), ik_rejected,
        collision_rejected, other_rejected, planning_failed, trajectory_finalize_failed);
    } else if (!move_group_state_->move_group->setPoseTarget(request.tcp_target, "sf_grasp_tcp")) {
      restore_grasp_contact_policy();
      RCLCPP_DEBUG(
        node_->get_logger(),
        "motion diagnostic normalized submit rejected: MoveGroup setPoseTarget failed");
      return false;
    }
    if (!place_global_approach && !planning_precomputed) {
      log_move_group_request_scaling(
        node_->get_logger(), *move_group_state_->move_group);
      const auto result = move_group_state_->move_group->plan(plan);
      planning_result = result.val;
      planning_succeeded = result == moveit::core::MoveItErrorCode::SUCCESS;
      move_group_state_->move_group->clearPoseTargets();
      if (planning_succeeded) {
        const auto finalized = finalize_raw_trajectory(plan.trajectory_);
        planning_succeeded = finalized.success;
        if (!finalized.success) {
          RCLCPP_DEBUG(
            node_->get_logger(), "motion diagnostic MoveGroup finalize rejected: %s",
            finalized.reason.c_str());
        }
      }
    }
  }
  if (!restore_grasp_contact_policy()) {
    RCLCPP_DEBUG(node_->get_logger(), "[Motion validation] grasp contact policy restore failed");
    return false;
  }
  RCLCPP_DEBUG(
    node_->get_logger(),
    "motion diagnostic planning result=%d primitive=%s path_fraction=%.9f trajectory_points=%zu "
    "joint_names=%zu",
    planning_result, primitive_type, path_fraction, plan.trajectory_.joint_trajectory.points.size(),
    plan.trajectory_.joint_trajectory.joint_names.size());
  std::ostringstream trajectory_names;
  trajectory_names << "[";
  for (std::size_t index = 0;
    index < plan.trajectory_.joint_trajectory.joint_names.size(); ++index)
  {
    if (index != 0) {
      trajectory_names << ",";
    }
    trajectory_names << plan.trajectory_.joint_trajectory.joint_names[index];
  }
  trajectory_names << "]";
  std::ostringstream configured_names;
  configured_names << "[";
  for (std::size_t index = 0; index < arm_joint_names_.size(); ++index) {
    if (index != 0) {
      configured_names << ",";
    }
    configured_names << arm_joint_names_[index];
  }
  configured_names << "]";
  RCLCPP_DEBUG(
    node_->get_logger(), "motion diagnostic trajectory_joint_names=%s configured_joint_names=%s",
    trajectory_names.str().c_str(), configured_names.str().c_str());
  if (plan.trajectory_.joint_trajectory.joint_names != arm_joint_names_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion diagnostic JOINT_NAME_ORDER_MISMATCH trajectory vs configured arm order");
    if (planning_max_acceleration_rad_s2_ > 0.0) {
      return false;
    }
  }
  if (!planning_succeeded || plan.trajectory_.joint_trajectory.points.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "motion diagnostic normalized submit failed at planning");
    return false;
  }
  if (selected_global_goal_valid) {
    continuity = normalize_joint_trajectory(
      plan.trajectory_.joint_trajectory, measured_start, continuity_limits, false);
    if (!continuity.valid) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "motion diagnostic GLOBAL APPROACH trajectory continuity rejected: %s",
        continuity.reason.c_str());
      return false;
    }
    const auto & planned_endpoint = plan.trajectory_.joint_trajectory.points.back().positions;
    for (std::size_t index = 0; index < selected_global_goal.size(); ++index) {
      const auto delta = planned_endpoint[index] - selected_global_goal[index];
      if (std::abs(delta) > 1e-3) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "motion diagnostic GLOBAL APPROACH endpoint mismatch joint=%zu delta=%.9f",
          index, delta);
        return false;
      }
    }
  } else if (!continuity.valid) {
    continuity = normalize_joint_trajectory(
      plan.trajectory_.joint_trajectory, measured_start, continuity_limits);
    if (!continuity.valid) {
      RCLCPP_DEBUG(
        node_->get_logger(), "motion diagnostic joint continuity validation rejected: %s",
        continuity.reason.c_str());
      return false;
    }
  }
  const auto & diagnostic_start = planning_start_state.getGlobalLinkTransform("sf_grasp_tcp");
  const auto & diagnostic_end = request.tcp_target.pose;
  const auto & points = plan.trajectory_.joint_trajectory.points;
  const auto & final_time = points.back().time_from_start;
  const double final_duration = static_cast<double>(final_time.sec) + final_time.nanosec / 1e9;
  RCLCPP_DEBUG(
    node_->get_logger(),
    "[Motion motion diagnostic] phase=%d primitive=%s start_tcp=(%.9f,%.9f,%.9f) "
    "end_tcp=(%.9f,%.9f,%.9f) path_fraction=%.9f trajectory_points=%zu "
    "raw_endpoint_delta=%s wrap_aware_endpoint_delta=%s normalized=%s "
    "normalized_joints=%s accumulated_motion=%s final_duration=%.9f "
    "expected_start_joints=%s measured_start_joints=%s",
    request.phase, primitive_type, diagnostic_start.translation().x(),
    diagnostic_start.translation().y(), diagnostic_start.translation().z(),
    diagnostic_end.position.x, diagnostic_end.position.y, diagnostic_end.position.z,
    path_fraction, points.size(), format_double_values(continuity.raw_endpoint_delta).c_str(),
    format_double_values(continuity.wrap_aware_endpoint_delta).c_str(),
    continuity.normalized ? "true" : "false",
    format_bool_values(continuity.normalized_joints).c_str(),
    format_double_values(continuity.raw_cumulative_motion).c_str(), final_duration,
    format_double_values(expected_insertion_start).c_str(),
    format_double_values(measured_start).c_str());
  if (planning_max_acceleration_rad_s2_ > 0.0) {
    if (!trajectory_sampler_configuration_error_.empty()) {
      RCLCPP_DEBUG(
        node_->get_logger(), "motion diagnostic trajectory execution rejected: %s",
        trajectory_sampler_configuration_error_.c_str());
      return false;
    }
    double start_simulation_time = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_simulation_time_) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "motion diagnostic trajectory execution rejected: Isaac simulation clock unavailable");
        return false;
      }
      start_simulation_time = latest_simulation_time_;
    }
    std::vector<double> velocity_limits;
    velocity_limits.reserve(arm_joint_names_.size());
    for (const auto & joint_name : arm_joint_names_) {
      const auto * joint_model = robot_model->getJointModel(joint_name);
      if (!joint_model || joint_model->getVariableBounds().empty()) {
        RCLCPP_DEBUG(
          node_->get_logger(), "motion diagnostic missing velocity bounds for %s",
          joint_name.c_str());
        return false;
      }
      velocity_limits.push_back(joint_model->getVariableBounds().front().max_velocity_);
    }
    const auto validation = validate_time_parameterized_trajectory(
      plan.trajectory_.joint_trajectory, velocity_limits, planning_max_acceleration_rad_s2_);
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion diagnostic trajectory validation points=%zu final_time_from_start=%.9f "
      "velocities=%s accelerations=%s valid=%s reason=%s",
      points.size(), static_cast<double>(final_time.sec) + final_time.nanosec / 1e9,
      points.front().velocities.empty() ? "false" : "true",
      points.front().accelerations.empty() ? "false" : "true",
      validation.valid ? "true" : "false", validation.reason.c_str());
    if (!validation.valid) {
      RCLCPP_DEBUG(node_->get_logger(), "motion diagnostic timed trajectory validation rejected");
      return false;
    }
    const auto expected_duration =
      static_cast<double>(final_time.sec) + final_time.nanosec / 1e9;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      timed_trajectory_active_ = true;
      approach_settling_tracker_.reset();
      approach_reference_completion_observed_ = false;
      settling_diagnostics_enabled_ =
        request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
      settling_diagnostic_logged_ = false;
      last_settling_diagnostic_simulation_time_ = start_simulation_time;
      final_target_hold_.cancel();
      final_target_hold_publish_count_ = 0;
      timed_clock_callback_count_ = 0;
      timed_command_publish_attempt_count_ = 0;
      timed_command_publish_success_count_ = 0;
      last_timed_telemetry_simulation_time_ = start_simulation_time;
      trajectory_watchdog_.start(
        start_simulation_time, expected_duration, std::chrono::steady_clock::now());
    }
    const auto tracking_tolerance = select_trajectory_tracking_tolerance(
      request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE,
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING,
      place_tracking_tolerance_rad_);
    trajectory_worker_->set_convergence_tolerance(tracking_tolerance);
    trajectory_worker_->set_velocity_limits(velocity_limits);
    std::ostringstream active_limits;
    active_limits << "sampler=" << trajectory_sampler_name_
                  << ";planning_max_acceleration_rad_s2="
                  << planning_max_acceleration_rad_s2_ << ";velocity_limits="
                  << format_double_values(velocity_limits)
                  << ";moveit_velocity_scaling_factor=" << planning_scales.max_velocity
                  << ";moveit_acceleration_scaling_factor="
                  << planning_scales.max_acceleration
                  << ";duration_s=" << expected_duration;
    TrajectorySampleResult worker_start;
    if (cartesian_primitive) {
      trajectory_worker_->set_profile_trace(nullptr);
      worker_start = trajectory_worker_->start(
        plan.trajectory_.joint_trajectory, ++next_trajectory_execution_id_, start_simulation_time);
    } else {
      worker_start = start_global_profiled_trajectory(
        plan.trajectory_.joint_trajectory, ++next_trajectory_execution_id_,
        start_simulation_time, active_limits.str());
    }
    if (!worker_start.valid) {
      std::lock_guard<std::mutex> lock(mutex_);
      timed_trajectory_active_ = false;
      settling_diagnostics_enabled_ = false;
      RCLCPP_DEBUG(
        node_->get_logger(), "motion diagnostic trajectory worker rejected plan: %s",
        worker_start.reason.c_str());
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      execution_active_ = true;
      stop_requested_ = false;
      inactivity_confirmed_ = false;
      completion_outcome_ = MotionCompletionOutcome::COMPLETED;
      settled_feedback_count_ = 0;
    }
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion diagnostic full trajectory execution started execution_id=%llu points=%zu "
      "start_simulation_time=%.9f",
      static_cast<unsigned long long>(trajectory_worker_->diagnostics().execution_id),
      trajectory_worker_->diagnostics().point_count, start_simulation_time);
    return true;
  }
  const auto & final_positions = plan.trajectory_.joint_trajectory.points.back().positions;
  std::ostringstream target_stream;
  target_stream << "[";
  for (std::size_t index = 0; index < final_positions.size(); ++index) {
    if (index != 0) {
      target_stream << ",";
    }
    target_stream << final_positions[index];
  }
  target_stream << "]";
  RCLCPP_DEBUG(
    node_->get_logger(), "motion diagnostic planned final joint target=%s",
    target_stream.str().c_str());
  return publish_joint_target(
    final_positions);
}

MotionCompletionOutcome IsaacRosMotionTransport::wait_for_motion_completion()
{
  std::unique_lock<std::mutex> lock(mutex_);
  bool bounded_hold_started = false;
  std::chrono::steady_clock::time_point bounded_hold_start;
  const auto bounded_hold_terminal = [this, &bounded_hold_started, &bounded_hold_start](
    MotionCompletionOutcome outcome)
    {
      if (!bounded_hold_started) {
        return;
      }
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - bounded_hold_start).count();
      const char * final_outcome = "TRANSPORT_UNAVAILABLE";
      switch (outcome) {
        case MotionCompletionOutcome::COMPLETED: final_outcome = "COMPLETED"; break;
        case MotionCompletionOutcome::TIMEOUT: final_outcome = "TIMEOUT"; break;
        case MotionCompletionOutcome::STALE_FEEDBACK: final_outcome = "STALE_FEEDBACK"; break;
        case MotionCompletionOutcome::CANCELED: final_outcome = "CANCELED"; break;
        case MotionCompletionOutcome::SAFETY_PREEMPTED: final_outcome = "SAFETY_PREEMPTED"; break;
        case MotionCompletionOutcome::TRANSPORT_UNAVAILABLE: break;
      }
      const char * final_acceptance = outcome == MotionCompletionOutcome::COMPLETED ?
        "ACCEPTED" : "REJECTED";
      RCLCPP_DEBUG(
        node_->get_logger(),
        "final target hold diagnostic event=TERMINAL final_acceptance=%s "
        "final_outcome=%s "
        "last_rejection_reason=%s last_position_error_m=%.9f "
        "last_orientation_error_rad=%.9f hold_duration_s=%.6f",
        final_acceptance, final_outcome, last_approach_rejection_reason_.c_str(),
        last_approach_rejection_position_error_m_,
        last_approach_rejection_orientation_error_rad_, elapsed);
      final_target_hold_diagnostic_started_ = false;
      final_target_hold_diagnostic_start_ = std::chrono::steady_clock::time_point{};
      approach_acceptance_log_initialized_ = false;
      last_approach_acceptance_reason_.clear();
      last_approach_rejection_reason_.clear();
      last_approach_acceptance_accepted_ = false;
      last_approach_acceptance_settled_ = false;
      last_approach_acceptance_fresh_feedback_ = false;
      last_approach_acceptance_joint_start_sync_ = false;
      last_approach_acceptance_state_valid_ = false;
      last_approach_acceptance_position_error_m_ = 0.0;
      last_approach_acceptance_orientation_error_rad_ = 0.0;
      last_approach_rejection_position_error_m_ = 0.0;
      last_approach_rejection_orientation_error_rad_ = 0.0;
      bounded_hold_started = false;
      bounded_hold_start = std::chrono::steady_clock::time_point{};
    };
  auto next_command_republish = std::chrono::steady_clock::now() +
    kMotionCommandRepublishPeriod;
  if (final_target_hold_diagnostic_started_) {
    bounded_hold_started = true;
    bounded_hold_start = final_target_hold_diagnostic_start_;
  }
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (final_target_hold_diagnostic_started_ && !bounded_hold_started) {
      bounded_hold_started = true;
      bounded_hold_start = final_target_hold_diagnostic_start_;
    }
    if (stop_requested_) {
      RCLCPP_WARN(node_->get_logger(), "motion diagnostic completion interrupted: stop_requested");
      finish_global_profile_capture("trajectory_interrupted", false);
      motion_progress_watchdog_active_ = false;
      bounded_hold_terminal(completion_outcome_);
      return completion_outcome_;
    }
    if (trajectory_worker_ && trajectory_worker_->failed()) {
      RCLCPP_DEBUG(
        node_->get_logger(), "motion diagnostic trajectory execution failed: %s",
        trajectory_worker_->failure_reason().c_str());
      finish_global_profile_capture("trajectory_execution_failed", false);
      motion_progress_watchdog_active_ = false;
      bounded_hold_terminal(MotionCompletionOutcome::TRANSPORT_UNAVAILABLE);
      return MotionCompletionOutcome::TRANSPORT_UNAVAILABLE;
    }
    const bool approach_phase =
      (timed_trajectory_active_ || approach_acceptance_active_) && active_phase_ ==
      arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
    if (final_target_hold_.active() && !bounded_hold_started) {
      bounded_hold_started = true;
      bounded_hold_start = std::chrono::steady_clock::now();
      final_target_hold_diagnostic_started_ = true;
      final_target_hold_diagnostic_start_ = bounded_hold_start;
    }
    if (timed_trajectory_active_) {
      const auto trajectory_status = trajectory_watchdog_.evaluate(
        latest_simulation_time_, trajectory_worker_->diagnostics().execution_complete, now);
      const auto watchdog_diagnostics = trajectory_watchdog_.diagnostics();
      if (trajectory_status == TrajectoryWatchdogStatus::SIMULATION_CLOCK_STALL)
      {
        const auto diagnostics = trajectory_worker_->diagnostics();
        if (settling_diagnostics_enabled_)
        {
          const auto settling_elapsed =
            watchdog_diagnostics.current_simulation_time -
            watchdog_diagnostics.expected_simulation_duration;
          RCLCPP_DEBUG(
            node_->get_logger(),
            "motion settling diagnostic event=FINAL_SNAPSHOT execution_id=%llu "
            "trajectory_time=%.9f segment=%zu %s",
            static_cast<unsigned long long>(diagnostics.execution_id), diagnostics.trajectory_time,
            diagnostics.current_segment,
            format_trajectory_settling_snapshot(
              settling_elapsed, diagnostics.commanded_positions, diagnostics.actual_positions,
              arm_joint_names_).c_str());
        }
        RCLCPP_DEBUG(
          node_->get_logger(),
          "motion diagnostic timed trajectory stopped: simulation clock made no progress "
          "expected_simulation_duration=%.9f current_simulation_time=%.9f "
          "elapsed_wall=%.9f observed_rtf=%.9f wall_since_progress=%.9f "
          "segment=%zu max_tracking_error=%.9f final_converged=%s timeout_reason=%s",
          watchdog_diagnostics.expected_simulation_duration,
          watchdog_diagnostics.current_simulation_time,
          watchdog_diagnostics.elapsed_wall_s, watchdog_diagnostics.observed_rtf,
          watchdog_diagnostics.wall_since_progress_s, diagnostics.current_segment,
          diagnostics.max_tracking_error, diagnostics.final_converged ? "true" : "false",
          "SIMULATION_CLOCK_STALL");
        trajectory_worker_->cancel();
        final_target_hold_.cancel();
        timed_trajectory_active_ = false;
        settling_diagnostics_enabled_ = false;
        finish_global_profile_capture("trajectory_watchdog_timeout", false);
        motion_progress_watchdog_active_ = false;
        bounded_hold_terminal(MotionCompletionOutcome::TIMEOUT);
        lock.unlock();
        publish_current_hold_target(MotionCompletionOutcome::TIMEOUT);
        return MotionCompletionOutcome::TIMEOUT;
      }
    }
    if (!joint_command_publisher_ || joint_command_publisher_->get_subscription_count() == 0) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "motion diagnostic completion failed: no /joint_commands subscriber");
      finish_global_profile_capture("trajectory_command_transport_unavailable", false);
      motion_progress_watchdog_active_ = false;
      bounded_hold_terminal(MotionCompletionOutcome::TRANSPORT_UNAVAILABLE);
      return MotionCompletionOutcome::TRANSPORT_UNAVAILABLE;
    }
    const bool timed_execution_complete = timed_trajectory_active_ && trajectory_worker_ &&
      trajectory_worker_->completion_eligible();
    const bool approach_hold_ready = !approach_phase || !timed_trajectory_active_ ||
      final_target_hold_.active();
    const bool approach_settled = !approach_phase || actual_approach_settled_locked();
    const bool motion_target_reached = timed_trajectory_active_ ?
      (timed_execution_complete && approach_hold_ready && approach_settled) :
      (measured_target_reached_locked() && approach_settled);
    if (latest_joint_state_ && motion_target_reached &&
      joint_state_generation_ > motion_command_generation_ &&
      !execution_active_ && inactivity_confirmed_ &&
      (!timed_trajectory_active_ || trajectory_worker_->completion_eligible()))
    {
      if (approach_phase) {
        lock.unlock();
        const bool accepted = validate_approach_entry();
        lock.lock();
        if (!accepted) {
          if (!final_target_hold_.active()) {
            timed_trajectory_active_ = false;
            settling_diagnostics_enabled_ = false;
            finish_global_profile_capture("trajectory_approach_rejected", false);
            motion_progress_watchdog_active_ = false;
            bounded_hold_terminal(MotionCompletionOutcome::TRANSPORT_UNAVAILABLE);
            lock.unlock();
            publish_current_hold_target(MotionCompletionOutcome::TRANSPORT_UNAVAILABLE);
            return MotionCompletionOutcome::TRANSPORT_UNAVAILABLE;
          }
        } else {
          bounded_hold_terminal(MotionCompletionOutcome::COMPLETED);
          log_motion_state_locked("COMPLETED");
          final_target_hold_.cancel();
          timed_trajectory_active_ = false;
          settling_diagnostics_enabled_ = false;
          motion_progress_watchdog_active_ = false;
          return MotionCompletionOutcome::COMPLETED;
        }
      } else {
        log_motion_state_locked("COMPLETED");
        timed_trajectory_active_ = false;
        settling_diagnostics_enabled_ = false;
        motion_progress_watchdog_active_ = false;
        bounded_hold_terminal(MotionCompletionOutcome::COMPLETED);
        return MotionCompletionOutcome::COMPLETED;
      }
    }
    if (execution_active_ && (!trajectory_worker_ || !trajectory_worker_->active()) &&
      now >= next_command_republish)
    {
      republish_joint_target_locked();
      next_command_republish = now + kMotionCommandRepublishPeriod;
    }
    const auto freshness_deadline = latest_joint_state_ ?
      latest_state_receipt_ + kStateFreshness : now + kStateFreshness;
    if (std::chrono::steady_clock::now() >= freshness_deadline) {
      log_motion_state_locked("STALE_FEEDBACK");
      finish_global_profile_capture("trajectory_feedback_stale", false);
      motion_progress_watchdog_active_ = false;
      bounded_hold_terminal(MotionCompletionOutcome::STALE_FEEDBACK);
      return MotionCompletionOutcome::STALE_FEEDBACK;
    }
    const bool motion_progress_expected = execution_active_ || approach_phase ||
      final_target_hold_.active();
    if (motion_progress_watchdog_active_ && motion_progress_expected &&
      motion_progress_watchdog_.stalled(now))
    {
      log_motion_state_locked("MOTION_PROGRESS_STALLED");
      finish_global_profile_capture("motion_progress_stalled", false);
      motion_progress_watchdog_active_ = false;
      final_target_hold_.cancel();
      timed_trajectory_active_ = false;
      settling_diagnostics_enabled_ = false;
      bounded_hold_terminal(MotionCompletionOutcome::TIMEOUT);
      lock.unlock();
      publish_current_hold_target(MotionCompletionOutcome::TIMEOUT);
      return MotionCompletionOutcome::TIMEOUT;
    }
    const auto progress_deadline = timed_trajectory_active_ ?
      trajectory_watchdog_.next_liveness_deadline() :
      (motion_progress_watchdog_active_ ? motion_progress_watchdog_.next_deadline() :
      now + kMotionCommandRepublishPeriod);
    joint_state_condition_.wait_until(
      lock, std::min({progress_deadline, freshness_deadline, next_command_republish}));
  }
}

void IsaacRosMotionTransport::republish_joint_target_locked()
{
  if (!joint_command_publisher_ || target_positions_.size() != arm_joint_names_.size()) {
    return;
  }
  sensor_msgs::msg::JointState command;
  command.header.stamp = node_->now();
  command.name = arm_joint_names_;
  command.position = target_positions_;
  joint_command_publisher_->publish(command);
  motion_command_generation_ = joint_state_generation_;
}

bool IsaacRosMotionTransport::publish_joint_target(const std::vector<double> & positions)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return publish_joint_target_locked(positions);
}

bool IsaacRosMotionTransport::publish_timed_joint_target(const TimedJointSample & sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++timed_command_publish_attempt_count_;
  if (!timed_trajectory_active_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "motion diagnostic timed trajectory command rejected: timed execution is inactive");
    return false;
  }
  const auto published = publish_joint_target_locked(sample.positions, &sample.velocities);
  if (published) {
    ++timed_command_publish_success_count_;
  }
  return published;
}

bool IsaacRosMotionTransport::publish_final_target_hold_locked()
{
  if (!final_target_hold_.active() || !joint_command_publisher_ ||
    joint_command_publisher_->get_subscription_count() == 0)
  {
    return false;
  }
  const auto command = final_target_hold_.command(latest_simulation_time_);
  if (command.positions.size() != arm_joint_names_.size() ||
    command.velocities.size() != arm_joint_names_.size())
  {
    return false;
  }
  sensor_msgs::msg::JointState message;
  message.header.stamp = node_->now();
  message.name = arm_joint_names_;
  message.position = command.positions;
  message.velocity = command.velocities;
  joint_command_publisher_->publish(message);
  ++final_target_hold_publish_count_;
  return true;
}

bool IsaacRosMotionTransport::publish_joint_target_locked(
  const std::vector<double> & positions,
  const std::vector<double> * velocities)
{
  if (!latest_joint_state_) {
    RCLCPP_DEBUG(node_->get_logger(), "motion diagnostic publish rejected: no latest joint state");
    return false;
  }
  if (positions.size() != arm_joint_names_.size()) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion diagnostic publish rejected: target joint count=%zu expected=%zu",
      positions.size(), arm_joint_names_.size());
    return false;
  }
  if (positions.size() != home_joint_positions_.size()) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion diagnostic publish rejected: target/home joint count mismatch target=%zu home=%zu",
      positions.size(), home_joint_positions_.size());
    return false;
  }
  if (std::any_of(
      positions.begin(), positions.end(), [](double value) {return !std::isfinite(value);}))
  {
    RCLCPP_DEBUG(node_->get_logger(), "motion diagnostic publish rejected: non-finite joint target");
    return false;
  }
  if (velocities != nullptr) {
    if (velocities->size() != arm_joint_names_.size() || std::any_of(
        velocities->begin(), velocities->end(), [](double value) {return !std::isfinite(value);}))
    {
      RCLCPP_DEBUG(node_->get_logger(), "motion diagnostic publish rejected: invalid velocity target");
      return false;
    }
  }
  if (joint_command_publisher_->get_subscription_count() == 0) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Cannot publish /joint_commands: no compatible Isaac command subscriber is connected");
    return false;
  }
  if (!motion_progress_watchdog_active_) {
    std::vector<double> measured_positions;
    if (!extract_arm_positions(*latest_joint_state_, measured_positions)) {
      RCLCPP_DEBUG(node_->get_logger(), "motion diagnostic cannot initialize progress watchdog");
      return false;
    }
    motion_progress_watchdog_.start(measured_positions, std::chrono::steady_clock::now());
    motion_progress_watchdog_active_ = true;
  }
  sensor_msgs::msg::JointState command;
  command.header.stamp = node_->now();
  command.name = arm_joint_names_;
  command.position = positions;
  if (velocities != nullptr) {
    command.velocity = *velocities;
  }
  joint_command_publisher_->publish(command);
  target_positions_ = positions;
  motion_command_generation_ = joint_state_generation_;
  completion_outcome_ = MotionCompletionOutcome::COMPLETED;
  execution_active_ = true;
  stop_requested_ = false;
  inactivity_confirmed_ = false;
  settled_feedback_count_ = 0;
  return true;
}

bool IsaacRosMotionTransport::publish_current_hold_target(
  MotionCompletionOutcome interruption)
{
  std::lock_guard<std::mutex> lock(mutex_);
  completion_outcome_ = interruption;
  stop_requested_ = true;
  joint_state_condition_.notify_all();
  std::vector<double> measured_positions;
  if (!latest_joint_state_ ||
    std::chrono::steady_clock::now() - latest_state_receipt_ > kStateFreshness ||
    !extract_arm_positions(*latest_joint_state_, measured_positions))
  {
    inactivity_confirmed_ = false;
    return false;
  }
  if (joint_command_publisher_->get_subscription_count() == 0) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Cannot publish /joint_commands: no compatible Isaac command subscriber is connected");
    inactivity_confirmed_ = false;
    return false;
  }
  sensor_msgs::msg::JointState command;
  command.header.stamp = node_->now();
  command.name = arm_joint_names_;
  command.position = measured_positions;
  joint_command_publisher_->publish(command);
  target_positions_ = std::move(measured_positions);
  execution_active_ = true;
  inactivity_confirmed_ = false;
  settled_feedback_count_ = 0;
  stop_command_generation_ = joint_state_generation_;
  joint_state_condition_.notify_all();
  return true;
}

bool IsaacRosMotionTransport::extract_arm_positions(
  const sensor_msgs::msg::JointState & message,
  std::vector<double> & positions) const
{
  positions.clear();
  positions.reserve(arm_joint_names_.size());
  for (const auto & name : arm_joint_names_) {
    const auto joint = std::find(message.name.begin(), message.name.end(), name);
    if (joint == message.name.end()) {
      return false;
    }
    const auto index = static_cast<std::size_t>(std::distance(message.name.begin(), joint));
    if (index >= message.position.size() || !std::isfinite(message.position[index])) {
      return false;
    }
    positions.push_back(message.position[index]);
  }
  return true;
}

void IsaacRosMotionTransport::on_joint_state(
  const sensor_msgs::msg::JointState::ConstSharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_joint_state_ = std::make_shared<sensor_msgs::msg::JointState>(*message);
  latest_state_receipt_ = std::chrono::steady_clock::now();
  ++joint_state_generation_;
  std::vector<double> measured_positions;
  if (!extract_arm_positions(
      *message,
      measured_positions) || target_positions_.size() != measured_positions.size())
  {
    return;
  }
  if (motion_progress_watchdog_active_) {
    motion_progress_watchdog_.observe(measured_positions, latest_state_receipt_);
  }
  if (go_home_profile_trace_active_ && has_simulation_time_) {
    profile_trace_->add_row(
      {
        latest_simulation_time_, "ISAAC_ARTICULATION", home_joint_positions_, {}, {},
        measured_positions, extract_arm_velocities(*message)});
  }
  if (trajectory_worker_) {
    trajectory_worker_->update_actual_positions(
      measured_positions, has_simulation_time_ ? latest_simulation_time_ : -1.0);
    const auto telemetry = trajectory_worker_->diagnostics();
    if (timed_trajectory_active_ && active_phase_ ==
      arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING &&
      telemetry.execution_complete && !final_target_hold_.active())
    {
      if (!final_target_hold_.begin(telemetry.final_target, latest_simulation_time_)) {
        RCLCPP_DEBUG(
          node_->get_logger(),
          "final target hold diagnostic result=REJECTED reason=invalid final target");
        timed_trajectory_active_ = false;
        stop_requested_ = true;
        completion_outcome_ = MotionCompletionOutcome::TRANSPORT_UNAVAILABLE;
      } else {
        final_target_hold_publish_count_ = 0;
        final_target_hold_diagnostic_started_ = true;
        final_target_hold_diagnostic_start_ = std::chrono::steady_clock::now();
        RCLCPP_DEBUG(
          node_->get_logger(),
          "final target hold diagnostic event=ENTER execution_id=%llu "
          "simulation_time=%.9f final_target=%s zero_velocity=%s",
          static_cast<unsigned long long>(telemetry.execution_id), latest_simulation_time_,
          format_double_values(telemetry.final_target).c_str(),
          format_double_values(std::vector<double>(telemetry.final_target.size(), 0.0)).c_str());
        if (!publish_final_target_hold_locked()) {
          RCLCPP_DEBUG(
            node_->get_logger(),
            "final target hold diagnostic result=REJECTED reason=final target publish failed");
          timed_trajectory_active_ = false;
          stop_requested_ = true;
          completion_outcome_ = MotionCompletionOutcome::TRANSPORT_UNAVAILABLE;
          final_target_hold_.cancel();
        }
      }
    }
    if (telemetry.execution_complete &&
      telemetry.execution_id != final_lifecycle_logged_execution_id_)
    {
      final_lifecycle_logged_execution_id_ = telemetry.execution_id;
      finish_global_profile_capture("trajectory_execution_complete", true);
      RCLCPP_DEBUG(
        node_->get_logger(),
        "motion telemetry diagnostic final_target_lifecycle execution_id=%llu "
        "final_target=%s final_sample_first_simulation_time=%.9f "
        "final_sample_publish_count=%llu final_target_changed_after_publish=%s "
        "final_target_active=%s final_target_active_duration=%.9f "
        "actual=%s commanded=%s tracking_error=%s "
        "clock_callbacks=%llu samples=%llu publish_attempts=%llu publish_successes=%llu",
        static_cast<unsigned long long>(telemetry.execution_id),
        format_double_values(telemetry.final_target).c_str(),
        telemetry.final_sample_first_published_simulation_time,
        static_cast<unsigned long long>(telemetry.final_sample_publish_count),
        telemetry.final_target_changed_after_publish ? "true" : "false",
        telemetry.final_target_active ? "true" : "false",
        telemetry.final_target_active_duration,
        format_double_values(telemetry.actual_positions).c_str(),
        format_double_values(telemetry.commanded_positions).c_str(),
        format_double_values(telemetry.tracking_error).c_str(),
        static_cast<unsigned long long>(timed_clock_callback_count_),
        static_cast<unsigned long long>(telemetry.sample_count),
        static_cast<unsigned long long>(timed_command_publish_attempt_count_),
        static_cast<unsigned long long>(timed_command_publish_success_count_));
    }
    if (timed_trajectory_active_ && active_phase_ ==
      arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING)
    {
      const auto execution_complete = trajectory_worker_->diagnostics().execution_complete;
      if (execution_complete && !approach_reference_completion_observed_) {
        approach_settling_tracker_.reset();
        approach_reference_completion_observed_ = true;
      }
      if (approach_reference_completion_observed_) {
        approach_settling_tracker_.observe(measured_positions);
      }
    }
  }
  const bool at_target = std::equal(
    target_positions_.begin(), target_positions_.end(), measured_positions.begin(),
    [](double target, double actual) {
      return std::abs(target - actual) <= kPositionTolerance;
    });
  if (approach_acceptance_active_ && !timed_trajectory_active_ && active_phase_ ==
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING)
  {
    if (at_target && joint_state_generation_ > motion_command_generation_) {
      if (!approach_reference_completion_observed_) {
        approach_settling_tracker_.reset();
        approach_reference_completion_observed_ = true;
      }
      approach_settling_tracker_.observe(measured_positions);
    } else if (!at_target && approach_reference_completion_observed_) {
      approach_settling_tracker_.reset();
      approach_reference_completion_observed_ = false;
    }
  }
  if (stop_requested_) {
    if (joint_state_generation_ <= stop_command_generation_) {
      return;
    }
    settled_feedback_count_ = at_target ? settled_feedback_count_ + 1 : 0;
    if (settled_feedback_count_ >= 2) {
      stop_requested_ = false;
      execution_active_ = false;
      inactivity_confirmed_ = true;
    }
  } else if (execution_active_ && !timed_trajectory_active_) {
    if (at_target) {
      ++settled_feedback_count_;
    } else {
      settled_feedback_count_ = 0;
    }
    if (settled_feedback_count_ >= 2) {
      execution_active_ = false;
      inactivity_confirmed_ = true;
      if (go_home_profile_trace_active_) {
        go_home_profile_trace_active_ = false;
        finish_profile_trace("settled_by_existing_two_feedback_completion");
      }
    }
  } else if (execution_active_ && trajectory_worker_->completion_eligible()) {
    execution_active_ = false;
    inactivity_confirmed_ = true;
  }
  joint_state_condition_.notify_all();
}

void IsaacRosMotionTransport::on_simulation_clock(
  const rosgraph_msgs::msg::Clock::ConstSharedPtr message)
{
  const auto simulation_time = static_cast<double>(message->clock.sec) +
    static_cast<double>(message->clock.nanosec) / 1e9;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_simulation_time_ = simulation_time;
    has_simulation_time_ = true;
    if (timed_trajectory_active_) {
      ++timed_clock_callback_count_;
      trajectory_watchdog_.observe(simulation_time, std::chrono::steady_clock::now());
    }
  }
  bool final_hold_publish_failed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (timed_trajectory_active_ && final_target_hold_.active()) {
      final_hold_publish_failed = !publish_final_target_hold_locked();
    }
  }
  if (final_hold_publish_failed) {
    std::lock_guard<std::mutex> lock(mutex_);
    final_target_hold_.cancel();
    timed_trajectory_active_ = false;
    stop_requested_ = true;
    completion_outcome_ = MotionCompletionOutcome::TRANSPORT_UNAVAILABLE;
    joint_state_condition_.notify_all();
    RCLCPP_DEBUG(
      node_->get_logger(),
      "final target hold diagnostic result=REJECTED reason=hold publish failed");
    return;
  }
  if (!trajectory_worker_ || !trajectory_worker_->active()) {
    joint_state_condition_.notify_all();
    return;
  }
  const auto result = trajectory_worker_->on_simulation_time(simulation_time);
  if (!result.valid) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!timed_trajectory_active_ || stop_requested_) {
      finish_global_profile_capture("trajectory_execution_aborted", false);
      return;
    }
    execution_active_ = false;
    stop_requested_ = true;
    inactivity_confirmed_ = false;
    completion_outcome_ = MotionCompletionOutcome::TRANSPORT_UNAVAILABLE;
    finish_global_profile_capture("trajectory_execution_failed", false);
    joint_state_condition_.notify_all();
    RCLCPP_DEBUG(
      node_->get_logger(), "motion diagnostic full trajectory execution failed: %s",
      result.reason.c_str());
    return;
  }
  const auto diagnostics = trajectory_worker_->diagnostics();
  bool log_settling_snapshot = false;
  bool log_timed_telemetry = false;
  double settling_elapsed = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto & watchdog_diagnostics = trajectory_watchdog_.diagnostics();
    settling_elapsed =
      watchdog_diagnostics.current_simulation_time -
      watchdog_diagnostics.expected_simulation_duration;
    if (settling_diagnostics_enabled_ && settling_elapsed >= 0.0 &&
      (!settling_diagnostic_logged_ || should_log_settling_snapshot(
        last_settling_diagnostic_simulation_time_, simulation_time,
        kSettlingDiagnosticPeriodS)))
    {
      settling_diagnostic_logged_ = true;
      last_settling_diagnostic_simulation_time_ = simulation_time;
      log_settling_snapshot = true;
    }
    if (timed_trajectory_active_ &&
      (diagnostics.command_simulation_time - last_timed_telemetry_simulation_time_ >=
      kSettlingDiagnosticPeriodS || diagnostics.sample_count == 1))
    {
      last_timed_telemetry_simulation_time_ = diagnostics.command_simulation_time;
      log_timed_telemetry = true;
    }
  }
  if (log_settling_snapshot) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion settling diagnostic event=SNAPSHOT execution_id=%llu "
      "trajectory_time=%.9f segment=%zu %s",
      static_cast<unsigned long long>(diagnostics.execution_id), diagnostics.trajectory_time,
      diagnostics.current_segment,
      format_trajectory_settling_snapshot(
        settling_elapsed, diagnostics.commanded_positions, diagnostics.actual_positions,
        arm_joint_names_).c_str());
  }
  if (log_timed_telemetry) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion telemetry diagnostic execution_id=%llu simulation_time=%.9f "
      "trajectory_time=%.9f segment=%zu command=%s command_delta=%s "
      "command_velocity=%s actual=%s actual_delta=%s actual_velocity=%s "
      "tracking_error=%s clock_callbacks=%llu samples=%llu publish_attempts=%llu "
      "publish_successes=%llu final_sample_publish_count=%llu",
      static_cast<unsigned long long>(diagnostics.execution_id), simulation_time,
      diagnostics.trajectory_time, diagnostics.current_segment,
      format_double_values(diagnostics.commanded_positions).c_str(),
      format_double_values(diagnostics.commanded_delta).c_str(),
      format_double_values(diagnostics.commanded_velocity).c_str(),
      format_double_values(diagnostics.actual_positions).c_str(),
      format_double_values(diagnostics.actual_delta).c_str(),
      format_double_values(diagnostics.actual_velocity).c_str(),
      format_double_values(diagnostics.tracking_error).c_str(),
      static_cast<unsigned long long>(timed_clock_callback_count_),
      static_cast<unsigned long long>(diagnostics.sample_count),
      static_cast<unsigned long long>(timed_command_publish_attempt_count_),
      static_cast<unsigned long long>(timed_command_publish_success_count_),
      static_cast<unsigned long long>(diagnostics.final_sample_publish_count));
  }
  joint_state_condition_.notify_all();
}

void IsaacRosMotionTransport::on_gripper_status(
  const std_msgs::msg::String::ConstSharedPtr message)
{
  const auto log_rejection = [this](
    const char * reason, std::uint64_t sequence = 0,
    std::uint64_t current_sequence = 0)
  {
    bool should_log = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_gripper_status_rejection_reason_ != reason) {
        last_gripper_status_rejection_reason_ = reason;
        should_log = true;
      }
    }
    if (!should_log) {
      return;
    }
    RCLCPP_WARN(
      node_->get_logger(),
      "gripper status receipt rejected reason=%s seq=%llu current_seq=%llu",
      reason, static_cast<unsigned long long>(sequence),
      static_cast<unsigned long long>(current_sequence));
  };
  std::unordered_map<std::string, std::string> fields;
  std::stringstream stream(message->data);
  std::string field;
  while (std::getline(stream, field, ';')) {
    const auto separator = field.find('=');
    if (separator == std::string::npos) {
      log_rejection("malformed_payload");
      return;
    }
    fields[field.substr(0, separator)] = field.substr(separator + 1);
  }
  const auto required = {"ready", "grasp", "attached", "released", "width_mm", "seq"};
  for (const auto & key : required) {
    if (fields.find(key) == fields.end()) {
      log_rejection("missing_field");
      return;
    }
  }
  try {
    const bool ready = std::stoi(fields.at("ready")) == 1;
    const bool grasp = std::stoi(fields.at("grasp")) == 1;
    const bool attached = std::stoi(fields.at("attached")) == 1;
    const bool released = std::stoi(fields.at("released")) == 1;
    const double width = std::stod(fields.at("width_mm"));
    const auto sequence = static_cast<std::uint64_t>(std::stoull(fields.at("seq")));
    if (!std::isfinite(width)) {
      log_rejection("non_finite_width", sequence);
      return;
    }
    bool semantic_state_changed = false;
    const char * rejection_reason = nullptr;
    std::uint64_t current_sequence = 0;
    const auto receive_time = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_sequence = gripper_status_sequence_;
      if (sequence > gripper_status_sequence_) {
        semantic_state_changed =
          ready != gripper_runtime_ready_ || grasp != grasp_confirmed_ ||
          attached != object_attached_ || released != release_confirmed_;
        gripper_runtime_ready_ = ready;
        grasp_confirmed_ = grasp;
        object_attached_ = attached;
        release_confirmed_ = released;
        gripper_width_mm_ = width;
        gripper_status_sequence_ = sequence;
        latest_gripper_status_receipt_ = std::chrono::steady_clock::now();
      } else if (sequence == 0) {
        rejection_reason = "zero_sequence";
      } else if (sequence < gripper_status_sequence_) {
        rejection_reason = "stale_sequence";
      } else if (
        ready != gripper_runtime_ready_ || grasp != grasp_confirmed_ ||
        attached != object_attached_ || released != release_confirmed_)
      {
        rejection_reason = "sequence_reused_with_different_state";
      } else {
        // Width is live actuator feedback, not part of the semantic holding
        // observation identity. Accept it with the same sequence and refresh
        // receipt time while preserving strict semantic replay rejection.
        gripper_width_mm_ = width;
        latest_gripper_status_receipt_ = receive_time;
      }
      if (!rejection_reason) {
        last_gripper_status_rejection_reason_.clear();
      }
    }
    if (semantic_state_changed) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "gripper status transition diagnostic receive_ms=%lld seq=%llu "
        "ready=%d grasp=%d attached=%d released=%d width_mm=%.3f prior_seq=%llu",
        static_cast<long long>(
          std::chrono::duration_cast<std::chrono::milliseconds>(
            receive_time.time_since_epoch()).count()),
        static_cast<unsigned long long>(sequence), ready ? 1 : 0, grasp ? 1 : 0,
        attached ? 1 : 0, released ? 1 : 0, width,
        static_cast<unsigned long long>(current_sequence));
    } else if (rejection_reason) {
      log_rejection(rejection_reason, sequence, current_sequence);
    }
  } catch (const std::exception &) {
    log_rejection("malformed_payload");
    return;
  }
}

bool IsaacRosMotionTransport::validate_approach_entry()
{
  const auto should_log_acceptance = [this](
      bool accepted, const std::string & reason, bool settled, bool fresh_feedback,
    bool joint_start_sync, bool state_valid)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool first = !approach_acceptance_log_initialized_;
      const bool changed = first ||
        last_approach_acceptance_accepted_ != accepted ||
        last_approach_acceptance_reason_ != reason ||
        last_approach_acceptance_settled_ != settled ||
        last_approach_acceptance_fresh_feedback_ != fresh_feedback ||
        last_approach_acceptance_joint_start_sync_ != joint_start_sync ||
        last_approach_acceptance_state_valid_ != state_valid;
      if (changed) {
        approach_acceptance_log_initialized_ = true;
        last_approach_acceptance_accepted_ = accepted;
        last_approach_acceptance_reason_ = reason;
        last_approach_acceptance_settled_ = settled;
        last_approach_acceptance_fresh_feedback_ = fresh_feedback;
        last_approach_acceptance_joint_start_sync_ = joint_start_sync;
        last_approach_acceptance_state_valid_ = state_valid;
      }
      if (!accepted) {
        last_approach_rejection_reason_ = reason;
      }
      return changed ? (first ? std::string("FIRST") : std::string("CHANGE")) :
             std::string{};
    };
  std::vector<double> actual_positions;
  std::vector<double> expected_positions;
  geometry_msgs::msg::Pose requested_pose;
  geometry_msgs::msg::Pose selected_approach_pose;
  std::size_t selected_candidate_index = 0;
  bool selected_candidate_valid = false;
  std::chrono::steady_clock::time_point receipt;
  bool actual_settled = false;
  bool is_place = false;
  uint8_t phase = 0;
  std::string early_rejection;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_joint_state_ ||
      std::chrono::steady_clock::now() - latest_state_receipt_ > kStateFreshness ||
      !extract_arm_positions(*latest_joint_state_, actual_positions))
    {
      early_rejection = "fresh actual joint state unavailable";
    } else {
      requested_pose = approach_entry_target_.pose;
      selected_approach_pose = approach_entry_target_.pose;
      selected_candidate_index = approach_acceptance_candidate_index_;
      selected_candidate_valid = approach_acceptance_candidate_valid_;
      receipt = latest_state_receipt_;
      actual_settled = approach_reference_completion_observed_ &&
        approach_settling_tracker_.settled();
      is_place = active_trajectory_is_place_;
      phase = active_phase_;
      expected_positions = approach_expected_joint_positions_;
    }
  }
  if (!early_rejection.empty()) {
    const auto event = should_log_acceptance(false, early_rejection, false, false, false, false);
    if (!event.empty()) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "approach acceptance diagnostic event=%s result=REJECTED requested_tcp=unavailable "
        "actual_tcp=unavailable position_error_m=unavailable orientation_error_rad=unavailable "
        "settled=false fresh_feedback=false joint_start_sync=false state_valid=false "
        "contact_bodies=[] reason=%s", event.c_str(), early_rejection.c_str());
    }
    return false;
  }

  std::shared_ptr<moveit::core::RobotState> actual_state;
  const moveit::core::JointModelGroup * joint_group = nullptr;
  early_rejection.clear();
  {
    std::lock_guard<std::mutex> lock(move_group_state_->mutex);
    if (!move_group_state_->move_group) {
      early_rejection = "MoveGroup unavailable";
    } else {
      const auto current_state = move_group_state_->move_group->getCurrentState(1.0);
      joint_group = move_group_state_->move_group->getRobotModel()->getJointModelGroup("arm");
      if (!current_state || !joint_group) {
        early_rejection = "RobotState unavailable";
      } else {
        actual_state = std::make_shared<moveit::core::RobotState>(*current_state);
      }
    }
  }
  if (!early_rejection.empty()) {
    const auto event = should_log_acceptance(false, early_rejection, actual_settled, true,
        false, false);
    if (!event.empty()) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "approach acceptance diagnostic event=%s result=REJECTED requested_tcp=(%.9f,%.9f,%.9f) "
        "actual_tcp=unavailable position_error_m=unavailable orientation_error_rad=unavailable "
        "settled=%s fresh_feedback=true joint_start_sync=false state_valid=false "
        "contact_bodies=[] reason=%s",
        event.c_str(), requested_pose.position.x, requested_pose.position.y,
        requested_pose.position.z, actual_settled ? "true" : "false", early_rejection.c_str());
    }
    return false;
  }
  actual_state->setJointGroupPositions(joint_group, actual_positions);
  actual_state->update();
  const auto actual_transform = actual_state->getGlobalLinkTransform("sf_grasp_tcp");
  geometry_msgs::msg::Pose actual_pose;
  actual_pose.position.x = actual_transform.translation().x();
  actual_pose.position.y = actual_transform.translation().y();
  actual_pose.position.z = actual_transform.translation().z();
  const Eigen::Quaterniond actual_orientation(actual_transform.rotation());
  actual_pose.orientation.x = actual_orientation.x();
  actual_pose.orientation.y = actual_orientation.y();
  actual_pose.orientation.z = actual_orientation.z();
  actual_pose.orientation.w = actual_orientation.w();

  bool state_valid = false;
  std::string state_validity_diagnostic;
  std::vector<std::string> contact_bodies;
  if (!state_validity_client_ ||
    !state_validity_client_->wait_for_service(std::chrono::milliseconds(200)))
  {
    state_validity_diagnostic = "state validity service unavailable";
  } else {
    auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    moveit::core::robotStateToRobotStateMsg(*actual_state, request->robot_state, true);
    request->group_name = "arm";
    auto future = state_validity_client_->async_send_request(request);
    if (future.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
      const auto response = future.get();
      state_valid = response->valid;
      for (const auto & contact : response->contacts) {
        contact_bodies.push_back(contact.contact_body_1);
        contact_bodies.push_back(contact.contact_body_2);
      }
    } else {
      state_validity_diagnostic = "state validity response timeout";
    }
  }

  const bool fresh_feedback = std::chrono::steady_clock::now() - receipt <= kStateFreshness;
  const bool is_approaching =
    phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
  const auto tracking_tolerance = select_trajectory_tracking_tolerance(
    is_place, is_approaching, place_tracking_tolerance_rad_);
  const bool joint_start_sync = fresh_feedback && actual_settled &&
    joint_positions_within_tolerance(
    expected_positions, actual_positions, tracking_tolerance);
  auto approach_entry_policy = approach_entry_policy_;
  approach_entry_policy.position_tolerance_m = select_approach_entry_position_tolerance(
    is_place, is_approaching, approach_entry_policy_.position_tolerance_m,
    place_approach_entry_position_tolerance_m_);
  const auto result = evaluate_approach_entry(
    requested_pose, actual_pose, actual_settled, fresh_feedback, state_valid,
    approach_entry_policy);
  const bool accepted = result.accepted && joint_start_sync;
  const std::string reason = !joint_start_sync ? "APPROACH_START_STATE_UNSYNCED" :
    (state_validity_diagnostic.empty() ? result.reason :
    result.reason + " (" + state_validity_diagnostic + ")");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_approach_acceptance_position_error_m_ = result.position_error_m;
    last_approach_acceptance_orientation_error_rad_ = result.orientation_error_rad;
    if (!accepted) {
      last_approach_rejection_position_error_m_ = result.position_error_m;
      last_approach_rejection_orientation_error_rad_ = result.orientation_error_rad;
    }
  }
  const auto acceptance_event = should_log_acceptance(
      accepted, reason, result.settled, result.fresh_feedback, joint_start_sync,
      result.state_valid);
  if (!acceptance_event.empty())
  {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "approach acceptance diagnostic event=%s result=%s selected_candidate_index=%s "
      "selected_approach_tcp=(%.9f,%.9f,%.9f;%.9f,%.9f,%.9f,%.9f) "
      "acceptance_target_tcp=(%.9f,%.9f,%.9f;%.9f,%.9f,%.9f,%.9f) "
      "actual_tcp=(%.9f,%.9f,%.9f) position_error_m=%.9f orientation_error_rad=%.9f "
      "settled=%s fresh_feedback=%s joint_start_sync=%s expected_joints=%s "
      "measured_joints=%s joint_tracking_tolerance_rad=%.6f state_valid=%s "
      "contact_bodies=%s reason=%s",
      acceptance_event.c_str(), accepted ? "ACCEPTED" : "REJECTED",
      selected_candidate_valid ? std::to_string(selected_candidate_index).c_str() : "unavailable",
      selected_approach_pose.position.x, selected_approach_pose.position.y,
      selected_approach_pose.position.z, selected_approach_pose.orientation.x,
      selected_approach_pose.orientation.y, selected_approach_pose.orientation.z,
      selected_approach_pose.orientation.w, requested_pose.position.x,
      requested_pose.position.y, requested_pose.position.z,
      requested_pose.orientation.x, requested_pose.orientation.y,
      requested_pose.orientation.z, requested_pose.orientation.w,
      actual_pose.position.x, actual_pose.position.y,
      actual_pose.position.z, result.position_error_m, result.orientation_error_rad,
      result.settled ? "true" : "false", result.fresh_feedback ? "true" : "false",
      joint_start_sync ? "true" : "false", format_double_values(expected_positions).c_str(),
      format_double_values(actual_positions).c_str(), tracking_tolerance,
      result.state_valid ? "true" : "false",
      format_string_values(contact_bodies).c_str(), reason.c_str());
  }
  if (!joint_start_sync) {
    return false;
  }
  if (result.accepted) {
    std::lock_guard<std::mutex> state_lock(mutex_);
    validated_approach_start_joint_positions_ = actual_positions;
    approach_start_state_validated_ = true;
  }
  return accepted;
}

bool IsaacRosMotionTransport::current_state_fresh() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_joint_state_) {
    return false;
  }
  std::vector<double> measured_positions;
  return std::chrono::steady_clock::now() - latest_state_receipt_ <= kStateFreshness &&
         extract_arm_positions(*latest_joint_state_, measured_positions);
}

bool IsaacRosMotionTransport::actual_approach_settled_locked() const
{
  return approach_reference_completion_observed_ && approach_settling_tracker_.settled() &&
         latest_joint_state_ &&
         std::chrono::steady_clock::now() - latest_state_receipt_ <= kStateFreshness;
}

bool IsaacRosMotionTransport::measured_target_reached_locked() const
{
  if (!latest_joint_state_ ||
    std::chrono::steady_clock::now() - latest_state_receipt_ > kStateFreshness)
  {
    return false;
  }
  std::vector<double> measured_positions;
  if (!extract_arm_positions(*latest_joint_state_, measured_positions) ||
    target_positions_.size() != measured_positions.size())
  {
    return false;
  }
  return std::equal(
    target_positions_.begin(), target_positions_.end(), measured_positions.begin(),
    [](double target, double actual) {
      return std::abs(target - actual) <= kPositionTolerance;
    });
}

void IsaacRosMotionTransport::log_motion_state_locked(const std::string & event) const
{
  std::ostringstream stream;
  stream << "motion diagnostic completion=" << event
         << " execution_active=" << (execution_active_ ? "true" : "false")
         << " inactivity_confirmed=" << (inactivity_confirmed_ ? "true" : "false")
         << " command_generation=" << motion_command_generation_
         << " state_generation=" << joint_state_generation_;
  if (trajectory_worker_) {
    const auto & diagnostics = trajectory_worker_->diagnostics();
    stream << " trajectory_execution_id=" << diagnostics.execution_id
           << " trajectory_segment=" << diagnostics.current_segment
           << " trajectory_time=" << diagnostics.trajectory_time
           << " max_tracking_error=" << diagnostics.max_tracking_error
           << " final_converged=" << (diagnostics.final_converged ? "true" : "false")
           << " execution_complete=" <<
      (diagnostics.execution_complete ? "true" : "false");
  }
  if (latest_joint_state_) {
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - latest_state_receipt_).count();
    stream << " latest_state_age_ms=" << age;
    stream << " measured_name_order=[";
    for (std::size_t index = 0; index < latest_joint_state_->name.size(); ++index) {
      if (index != 0) {
        stream << ",";
      }
      stream << latest_joint_state_->name[index];
    }
    stream << "]";
  } else {
    stream << " latest_state_age_ms=none";
  }
  stream << " name_join_target_measured_error=[";
  std::vector<double> measured_positions;
  if (latest_joint_state_ && extract_arm_positions(*latest_joint_state_, measured_positions) &&
    measured_positions.size() == target_positions_.size())
  {
    for (std::size_t index = 0; index < target_positions_.size(); ++index) {
      if (index != 0) {
        stream << ",";
      }
      stream << arm_joint_names_[index] << ":(" << target_positions_[index] << "," <<
        measured_positions[index] << "," <<
        std::abs(target_positions_[index] - measured_positions[index]) << ")";
    }
  }
  const auto tracking_tolerance = timed_trajectory_active_ ?
    select_trajectory_tracking_tolerance(
    active_trajectory_is_place_,
    active_phase_ == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING,
    place_tracking_tolerance_rad_) :
    kPositionTolerance;
  stream << "] tolerance=" << tracking_tolerance
         << " final_target_hold_active=" << (final_target_hold_.active() ? "true" : "false")
         << " final_target_hold_elapsed=" <<
    final_target_hold_.elapsed(latest_simulation_time_)
         << " final_target_hold_publish_count=" << final_target_hold_publish_count_;
  RCLCPP_DEBUG(node_->get_logger(), "%s", stream.str().c_str());
}

void IsaacRosMotionTransport::cancel()
{
  if (trajectory_worker_) {
    trajectory_worker_->cancel();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    final_target_hold_.cancel();
    timed_trajectory_active_ = false;
    finish_global_profile_capture("trajectory_cancelled", false);
  }
  publish_current_hold_target(MotionCompletionOutcome::CANCELED);
}

void IsaacRosMotionTransport::safety_preempt(uint8_t stop_mode)
{
  stop_gripper();
  if (trajectory_worker_) {
    trajectory_worker_->cancel();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    final_target_hold_.cancel();
    timed_trajectory_active_ = false;
    finish_global_profile_capture("trajectory_safety_preempted", false);
  }
  if (stop_mode == arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED) {
    publish_current_hold_target(MotionCompletionOutcome::SAFETY_PREEMPTED);
  } else if (stop_mode == arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE) {
    publish_current_hold_target(MotionCompletionOutcome::SAFETY_PREEMPTED);
  } else if (stop_mode == arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY) {
    publish_current_hold_target(MotionCompletionOutcome::SAFETY_PREEMPTED);
  }
}

void IsaacRosMotionTransport::hold()
{
  safety_preempt(arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED);
}

void IsaacRosMotionTransport::stop()
{
  safety_preempt(arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY);
}
bool IsaacRosMotionTransport::execution_active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return execution_active_;
}
bool IsaacRosMotionTransport::inactivity_confirmed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return inactivity_confirmed_;
}

bool IsaacRosMotionTransport::command_normalized_gripper(double opening)
{
  if (!std::isfinite(opening) || opening < 0.0 || opening > 1.0 || !available()) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!gripper_runtime_ready_ ||
      std::chrono::steady_clock::now() - latest_gripper_status_receipt_ >
      kGripperStatusFreshness)
    {
      return false;
    }
  }
  std_msgs::msg::Float64 command;
  command.data = opening * kFullyOpenWidthMm;
  RCLCPP_DEBUG(
    node_->get_logger(), "gripper command publish width_mm=%.3f",
    command.data);
  gripper_publisher_->publish(command);
  std::lock_guard<std::mutex> lock(mutex_);
  gripper_active_ = true;
  return true;
}

HoldingObservation IsaacRosMotionTransport::holding_observation() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  HoldingObservation observation;
  const auto now = HoldingObservation::Clock::now();
  if (!gripper_runtime_ready_ ||
    now < latest_gripper_status_receipt_ ||
    now - latest_gripper_status_receipt_ > std::chrono::milliseconds(500) ||
    gripper_status_sequence_ == 0)
  {
    return observation;
  }
  observation.observed_at = latest_gripper_status_receipt_;
  observation.sequence = gripper_status_sequence_;
  if (object_attached_ && grasp_confirmed_) {
    observation.state = HoldingState::HELD;
  } else if (!object_attached_ && release_confirmed_) {
    observation.state = HoldingState::RELEASED;
  }
  return observation;
}

bool IsaacRosMotionTransport::gripper_active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return gripper_active_;
}
void IsaacRosMotionTransport::stop_gripper()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!gripper_active_ || !gripper_runtime_ready_ ||
    std::chrono::steady_clock::now() - latest_gripper_status_receipt_ >
    kGripperStatusFreshness)
  {
    gripper_active_ = false;
    return;
  }
  std_msgs::msg::Float64 command;
  command.data = gripper_width_mm_;
  RCLCPP_DEBUG(
    node_->get_logger(), "gripper command publish width_mm=%.3f",
    command.data);
  gripper_publisher_->publish(command);
  gripper_active_ = false;
}
}  // namespace arm_cell_motion_moveit2
