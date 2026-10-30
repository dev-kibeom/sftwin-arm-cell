#include "arm_cell_motion/candidate_occupancy.hpp"
#include "arm_cell_motion/isaac_ros_motion_transport.hpp"
#include "arm_cell_motion/planning_start_state.hpp"
#include "motion_primitive_policy.hpp"
#include "pick_cartesian_diagnostics.hpp"

#include "arm_cell_motion/cartesian_manipulation.hpp"
#include "arm_cell_motion/grasp_contact_policy.hpp"
#include "arm_cell_motion/pick_retract_action_wait.hpp"
#include "arm_cell_motion/pick_retract_collision_policy.hpp"
#include "arm_cell_motion/gripper_port.hpp"
#include "arm_cell_motion/place_approach_ik_policy.hpp"
#include "arm_cell_motion/place_approach_trajectory.hpp"
#include "arm_cell_motion/executable_trajectory_finalize.hpp"
#include "arm_cell_motion/trajectory_continuity.hpp"
#include "arm_cell_motion/trajectory_validation.hpp"
#include "arm_cell_motion/motion_profile_trace.hpp"
#include "arm_cell_motion/motion_result_code_name.hpp"
#include "arm_cell_motion/pick_symmetry.hpp"
#include "arm_cell_motion/pick_approach_geometry.hpp"
#include "arm_cell_motion/pick_capture_reference.hpp"
#include "arm_cell_motion/pick_collision_seed_policy.hpp"
#include "arm_cell_motion/pick_search_budget.hpp"
#include "arm_cell_motion/pick_search_controller.hpp"
#include "arm_cell_motion/pick_approach_planning_bt.hpp"
#include "arm_cell_motion/pick_planning_diagnostics.hpp"
#include "arm_cell_motion/motion_path_planning.hpp"
#include "arm_cell_motion/pick_approach_ik.hpp"
#include "arm_cell_motion/pick_grasp_contact.hpp"
#include "arm_cell_motion/pick_grasp_ik.hpp"
#include "arm_cell_motion/pick_grasp_kinematics.hpp"
#include "arm_cell_motion/pick_state_collision.hpp"
#include "arm_cell_motion/planning_scene_builder.hpp"
#include "arm_cell_motion/pick_rectangle_geometry.hpp"
#include "arm_cell_motion/reversed_trajectory.hpp"
#include "arm_cell_motion/pick_grasp_pose_candidates.hpp"
#include "arm_cell_motion/box_grasp_geometry.hpp"
#include "arm_cell_motion/rectangle_grasp_geometry.hpp"
#include "arm_cell_motion/task_accuracy.hpp"
#include "arm_cell_motion/occupancy_snapshot_renderer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <Eigen/Geometry>
#include <exception>
#include <future>
#include <functional>
#include <iomanip>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <unistd.h>

#include <arm_cell_interfaces/msg/motion_task_type.hpp>
#include <arm_cell_interfaces/msg/collision_primitive.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <nlohmann/json.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <nlohmann/json.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace arm_cell_motion
{

namespace
{
const char * occupancy_class_name(std::uint8_t value)
{
  using Primitive = arm_cell_interfaces::msg::CollisionPrimitive;
  switch (value) {
    case Primitive::OBSERVED_OCCUPANCY: return "OBSERVED_OCCUPANCY";
    case Primitive::UNKNOWN_OCCUPANCY: return "UNKNOWN_OCCUPANCY";
    default: return "UNKNOWN_CLASS";
  }
}

std::string json_escape(const std::string & value)
{
  std::ostringstream escaped;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\b': escaped << "\\b"; break;
      case '\f': escaped << "\\f"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (character < 0x20) {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<unsigned>(character) << std::dec;
        } else {
          escaped << static_cast<char>(character);
        }
    }
  }
  return escaped.str();
}

std::string diagnostic_goal_id(std::string value)
{
  std::replace_if(value.begin(), value.end(), [](unsigned char character) {
    return !std::isalnum(character) && character != '-' && character != '_';
  }, '_');
  return value.empty() ? "unknown-goal" : value;
}

std::pair<std::string, std::string> backend_failure_reason(const nlohmann::json & evidence)
{
  for (const auto * key : {"final_failure_reason", "failure_reason", "scene_failure_reason"}) {
    if (evidence.contains(key) && evidence[key].is_string()) {
      const auto value = evidence[key].get<std::string>();
      if (!value.empty() && value != "unavailable" && value != "UNAVAILABLE") {
        return {value, std::string("planning_evidence.") + key};
      }
    }
  }
  const auto final_segment = evidence.find("final_segment_planning");
  if (final_segment != evidence.end() && final_segment->is_object()) {
    for (const auto * key : {"cartesian_failure_reason"}) {
      if (final_segment->contains(key) && (*final_segment)[key].is_string()) {
        const auto value = (*final_segment)[key].get<std::string>();
        if (!value.empty() && value != "unavailable" && value != "UNAVAILABLE") {
          return {value, std::string("planning_evidence.final_segment_planning.") + key};
        }
      }
    }
    const auto attempts = final_segment->find("global_fallback_attempts");
    if (attempts != final_segment->end() && attempts->is_array() && !attempts->empty()) {
      const auto & last = attempts->back();
      if (last.is_object() && last.contains("result") && last["result"].is_string()) {
        const auto value = last["result"].get<std::string>();
        if (!value.empty() && value != "SUCCESS") {
          return {value, "planning_evidence.final_segment_planning.global_fallback_attempts"};
        }
      }
    }
  }
  return {};
}

void append_json_number(std::ostream & output, double value)
{
  if (std::isfinite(value)) {output << value;} else {output << "null";}
}

void append_json_numbers(std::ostream & output, const std::vector<double> & values)
{
  output << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {output << ',';}
    append_json_number(output, values[index]);
  }
  output << ']';
}

void append_json_pose(std::ostream & output, const geometry_msgs::msg::PoseStamped & pose)
{
  output << "{\"frame_id\":\"" << json_escape(pose.header.frame_id)
         << "\",\"position_xyz\":[";
  append_json_number(output, pose.pose.position.x);
  output << ',';
  append_json_number(output, pose.pose.position.y);
  output << ',';
  append_json_number(output, pose.pose.position.z);
  output << "],\"orientation_xyzw\":[";
  append_json_number(output, pose.pose.orientation.x);
  output << ',';
  append_json_number(output, pose.pose.orientation.y);
  output << ',';
  append_json_number(output, pose.pose.orientation.z);
  output << ',';
  append_json_number(output, pose.pose.orientation.w);
  output << "]}";
}

template<typename PlanFunction>
auto plan_with_num_attempts(
  moveit::planning_interface::MoveGroupInterface & move_group,
  unsigned int num_attempts, PlanFunction && plan_function)
{
  moveit_msgs::msg::MotionPlanRequest original_request;
  move_group.constructMotionPlanRequest(original_request);
  struct RestoreAttempts
  {
    moveit::planning_interface::MoveGroupInterface & move_group;
    unsigned int attempts;
    ~RestoreAttempts() {move_group.setNumPlanningAttempts(attempts);}
  } restore{move_group, static_cast<unsigned int>(std::max(1, original_request.num_planning_attempts))};
  move_group.setNumPlanningAttempts(num_attempts);
  return std::forward<PlanFunction>(plan_function)();
}

// Require stability over a full feedback freshness window, independent of callback cadence.
constexpr auto kStopObservationWindow = std::chrono::milliseconds(500);
constexpr auto kMotionCommandRepublishPeriod = std::chrono::milliseconds(100);
constexpr auto kGripperStatusFreshness = std::chrono::milliseconds(500);
constexpr auto kMoveGroupServerWait = std::chrono::seconds(5);
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

void offset_pick_approach(
  geometry_msgs::msg::PoseStamped & pose, PickApproachAxis approach_axis,
  const geometry_msgs::msg::Vector3 & insertion_axis_tcp, double distance)
{
  pose = pick_approach_pose(pose, approach_axis, insertion_axis_tcp, distance);
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
  const auto tcp_rotation = compose_object_tcp_orientation(object_rotation, relation_rotation);
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

using PickOrientationCandidate = PickPlanningCandidate;

std::string environment_or(const char * name, const std::string & fallback)
{
  const auto * value = std::getenv(name);
  return value && *value ? value : fallback;
}

std::string build_commit_sha()
{
#ifdef ARM_CELL_BUILD_COMMIT_SHA
  return environment_or("GIT_COMMIT_SHA", ARM_CELL_BUILD_COMMIT_SHA);
#else
  return environment_or("GIT_COMMIT_SHA", "commit-sha-unreported");
#endif
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

class MoveGroupPositionToleranceRestore final
{
public:
  explicit MoveGroupPositionToleranceRestore(
    moveit::planning_interface::MoveGroupInterface & move_group)
  : move_group_(move_group), original_tolerance_(move_group.getGoalPositionTolerance()) {}

  ~MoveGroupPositionToleranceRestore()
  {
    move_group_.setGoalPositionTolerance(original_tolerance_);
  }

private:
  moveit::planning_interface::MoveGroupInterface & move_group_;
  double original_tolerance_;
};

class MoveGroupPoseReferenceFrameRestore final
{
public:
  explicit MoveGroupPoseReferenceFrameRestore(
    moveit::planning_interface::MoveGroupInterface & move_group)
  : move_group_(move_group), original_frame_(move_group.getPoseReferenceFrame()) {}

  ~MoveGroupPoseReferenceFrameRestore()
  {
    move_group_.setPoseReferenceFrame(original_frame_);
  }

private:
  moveit::planning_interface::MoveGroupInterface & move_group_;
  std::string original_frame_;
};

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

nlohmann::json merge_pick_candidate_history(const nlohmann::json & snapshot)
{
  auto merged = snapshot;
  if (!snapshot.is_object()) {return merged;}
  std::vector<const nlohmann::json *> sources;
  if (snapshot.contains("candidate_attempt_history") &&
    snapshot["candidate_attempt_history"].is_array())
  {
    for (const auto & previous : snapshot["candidate_attempt_history"]) {
      if (previous.is_object()) {sources.push_back(&previous);}
    }
  }
  sources.push_back(&snapshot);
  std::map<std::pair<std::string, std::size_t>, nlohmann::json> candidates_by_number;
  std::vector<nlohmann::json> attempts;
  std::size_t approach_planning_calls = 0U;
  std::size_t fk_rejections = 0U;
  std::size_t collision_rejections = 0U;
  for (std::size_t source_index = 0; source_index < sources.size(); ++source_index) {
    const auto & source = *sources[source_index];
    const bool duplicate_snapshot = std::any_of(
      sources.begin(), sources.begin() + static_cast<std::ptrdiff_t>(source_index),
      [&](const nlohmann::json * previous) {return *previous == source;});
    if (duplicate_snapshot) {continue;}
    if (source.contains("grasp_pose_candidates") &&
      source["grasp_pose_candidates"].is_array())
    {
      const auto & source_candidates = source["grasp_pose_candidates"];
      for (std::size_t index = 0; index < source_candidates.size(); ++index) {
        const auto & candidate = source_candidates[index];
        const auto number = candidate.value("candidate", index + 1U);
        const auto material_id = source.value("material_candidate_id", std::string{});
        auto candidate_with_material_id = candidate;
        if (!material_id.empty()) {
          candidate_with_material_id["material_candidate_id"] = material_id;
        }
        auto existing = candidates_by_number.find({material_id, number});
        if (existing == candidates_by_number.end() ||
          (existing->second.value("result", "NOT_EVALUATED") == "NOT_EVALUATED" &&
          candidate.value("result", "NOT_EVALUATED") != "NOT_EVALUATED"))
        {
          candidates_by_number[{material_id, number}] = std::move(candidate_with_material_id);
        }
      }
    }
    if (source.contains("pick_attempts") && source["pick_attempts"].is_array()) {
      for (const auto & attempt : source["pick_attempts"]) {
        auto material_attempt = attempt;
        const auto material_id = source.value("material_candidate_id", std::string{});
        if (!material_id.empty()) {material_attempt["material_candidate_id"] = material_id;}
        attempts.push_back(std::move(material_attempt));
      }
    }
    approach_planning_calls += source.value("approach_global_planning_call_count", 0U);
    if (source.contains("rejections") && source["rejections"].is_object()) {
      fk_rejections += source["rejections"].value("fk", 0U);
      collision_rejections += source["rejections"].value("collision_or_validity", 0U);
    }
  }
  auto candidate_array = nlohmann::json::array();
  for (auto & entry : candidates_by_number) {candidate_array.push_back(std::move(entry.second));}
  merged["grasp_pose_candidates"] = std::move(candidate_array);
  merged["pick_attempts"] = std::move(attempts);
  merged["approach_global_planning_call_count"] = approach_planning_calls;
  merged["aggregated_rejections"] = {
    {"fk", fk_rejections}, {"collision_or_validity", collision_rejections}};
  return merged;
}

}  // namespace

MotionPlanningDiagnosticCapture::MotionPlanningDiagnosticCapture(
  std::string directory, rclcpp::Logger logger, std::string snapshot_directory)
: directory_(std::move(directory)), snapshot_directory_(std::move(snapshot_directory)),
  logger_(std::move(logger)),
  worker_([this]() {write_loop();}) {}

MotionPlanningDiagnosticCapture::~MotionPlanningDiagnosticCapture()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  condition_.notify_one();
  if (worker_.joinable()) {worker_.join();}
}

bool MotionPlanningDiagnosticCapture::enqueue(
  const std::string & goal_uuid, const char * task_type, uint8_t result_code,
  const std::string & evidence, const std::string & vision_acquisition_id)
{
  std::ostringstream record;
  record << std::setprecision(17)
         << "{\"schema_version\":1,\"motion_goal_uuid\":\""
         << json_escape(goal_uuid) << "\",\"vision_acquisition_id\":";
  if (vision_acquisition_id.empty()) {
    record << "null";
  } else {
    record << "\"" << json_escape(vision_acquisition_id) << "\"";
  }
  record << ",\"task_type\":\"" << task_type
         << "\",\"record_status\":\"terminal\",\"task_executor_result_code\":\""
         << motion_result_code_name(result_code)
         << "\",\"task_executor_result_code_value\":" <<
    static_cast<unsigned>(result_code)
         << ",\"planning_evidence\":" << evidence << '}';
  return enqueue_serialized(goal_uuid, record.str());
}

bool MotionPlanningDiagnosticCapture::enqueue_progress(
  const std::string & goal_uuid, const char * task_type, const std::string & evidence,
  const std::string & vision_acquisition_id)
{
  std::ostringstream record;
  record << std::setprecision(17)
         << "{\"schema_version\":1,\"motion_goal_uuid\":\""
         << json_escape(goal_uuid) << "\",\"vision_acquisition_id\":";
  if (vision_acquisition_id.empty()) {
    record << "null";
  } else {
    record << "\"" << json_escape(vision_acquisition_id) << "\"";
  }
  record << ",\"task_type\":\"" << task_type
         << "\",\"record_status\":\"planning_in_progress\","
            "\"task_executor_result_code\":null,\"task_executor_result_code_value\":null,"
            "\"planning_evidence\":" << evidence << '}';
  if (std::string(task_type) == "TASK_TYPE_PICK") {
    try {
      (void)enqueue_pick_details(goal_uuid, vision_acquisition_id, evidence);
    } catch (...) {
      RCLCPP_WARN(logger_, "Motion PICK detail capture unavailable");
    }
  }
  return enqueue_serialized(goal_uuid, record.str());
}

bool MotionPlanningDiagnosticCapture::enqueue_occupancy_snapshot(
  const std::string & goal_uuid, const std::string & vision_acquisition_id,
  const std::string & candidate_id,
  const std::vector<arm_cell_interfaces::msg::CollisionPrimitive> & primitives,
  std::string & relative_path)
{
  const auto file_stem = "occupancy_" + safe_goal_id(goal_uuid) + "_candidate_" +
    safe_goal_id(candidate_id);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto attempt_key = goal_uuid + '\0' + vision_acquisition_id + '\0' + candidate_id;
    const auto existing = occupancy_snapshot_paths_.find(attempt_key);
    if (existing != occupancy_snapshot_paths_.end()) {
      relative_path = existing->second;
      return true;
    }
    if (stopping_ || occupancy_snapshot_queue_.size() >= kOccupancySnapshotQueueCapacity) {
      return false;
    }
    const auto sequence = ++occupancy_snapshot_sequence_;
    const auto file_name = (std::filesystem::path("acquisition") /
      safe_goal_id(vision_acquisition_id) /
      (file_stem + "_attempt_" + std::to_string(sequence) + ".png")).string();
    occupancy_snapshot_queue_.push_back(
      {file_name, vision_acquisition_id, candidate_id, primitives});
    relative_path = std::filesystem::absolute(
      std::filesystem::path(snapshot_directory_) / file_name).lexically_relative(
      std::filesystem::absolute(std::filesystem::path(directory_).parent_path())).string();
    occupancy_snapshot_paths_.emplace(attempt_key, relative_path);
  }
  condition_.notify_one();
  return true;
}

std::size_t MotionPlanningDiagnosticCapture::enqueue_pick_details(
  const std::string & goal_uuid, const std::string & vision_acquisition_id,
  const std::string & evidence)
{
  const auto snapshot = nlohmann::json::parse(evidence, nullptr, false);
  if (!snapshot.is_object()) {
    return 0U;
  }
  auto candidate_snapshot = merge_pick_candidate_history(snapshot);
  if (!candidate_snapshot["grasp_pose_candidates"].is_array()) {return 0U;}
  const auto & candidates = candidate_snapshot["grasp_pose_candidates"];
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_) {return false;}
  std::size_t accepted = 0U;
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    auto attempts = nlohmann::json::array();
    if (candidate_snapshot.contains("pick_attempts") &&
      candidate_snapshot["pick_attempts"].is_array())
    {
      for (const auto & attempt : candidate_snapshot["pick_attempts"]) {
        if (attempt.value("grasp_pose_candidate", 0U) == candidates[index].value("candidate", index + 1U) &&
          attempt.value("material_candidate_id", std::string{}) ==
          candidates[index].value("material_candidate_id", std::string{})) {
          attempts.push_back(attempt);
        }
      }
    }
    nlohmann::json detail = {
      {"schema_version", 1}, {"motion_goal_uuid", goal_uuid},
      {"vision_acquisition_id", vision_acquisition_id.empty() ?
        nlohmann::json(nullptr) : nlohmann::json(vision_acquisition_id)},
      {"candidate_number", index + 1U}, {"candidate", candidates[index]},
      {"attempts", std::move(attempts)}};
    if (candidates[index].contains("material_candidate_id")) {
      detail["material_candidate_id"] = candidates[index]["material_candidate_id"];
      detail["grasp_pose_candidate_number"] = candidates[index].value("candidate", index + 1U);
    }
    if (index == 0U && snapshot.contains("vision_occupancy_evidence")) {
      detail["vision_occupancy_evidence"] = snapshot["vision_occupancy_evidence"];
    }
    std::ostringstream file_name;
    file_name << "motion_planning_details/" << safe_goal_id(goal_uuid) << "_candidate_" <<
      std::setw(3)
              << std::setfill('0') << index + 1U << ".json";
    const auto detail_path = file_name.str();
    const auto logical_serialized = detail.dump();
    const bool oversized = logical_serialized.size() + 32U > kMaxDetailBytes;
    const auto cached = detail_snapshots_.find(detail_path);
    uint64_t generation = 1U;
    bool same_snapshot = false;
    if (cached != detail_snapshots_.end()) {
      same_snapshot = !oversized && !cached->second.value.is_null() &&
        cached->second.value == detail;
      generation = cached->second.generation + (same_snapshot ? 0U : 1U);
      if (same_snapshot && (cached->second.write_pending || cached->second.persisted)) {
        ++accepted;
        continue;
      }
    }
    auto & tracked = detail_snapshots_[detail_path];
    tracked.value = oversized ? nlohmann::json(nullptr) : detail;
    tracked.generation = generation;
    tracked.write_pending = false;
    if (!same_snapshot) {tracked.persisted = false;}
    while (detail_snapshots_.size() > kDetailSnapshotCacheCapacity) {
      detail_snapshots_.erase(detail_snapshots_.begin());
    }
    auto persisted_detail = detail;
    persisted_detail["detail_revision"] = generation;
    const auto serialized = persisted_detail.dump();
    if (oversized || serialized.size() > kMaxDetailBytes) {
      continue;
    }
    const auto queued = std::find_if(
      detail_queue_.begin(), detail_queue_.end(), [&](const PendingRecord & pending) {
        return pending.file_name == detail_path;
      });
    if (queued != detail_queue_.end()) {
      queued->serialized = serialized;
      queued->goal_uuid = goal_uuid;
      queued->detail_generation = generation;
      tracked.write_pending = true;
      ++accepted;
    } else if (detail_queue_.size() < kDetailQueueCapacity) {
      detail_queue_.push_back({goal_uuid, detail_path, serialized, false, generation});
      tracked.write_pending = true;
      ++accepted;
    } else {
      continue;
    }
  }
  condition_.notify_one();
  return accepted;
}

bool MotionPlanningDiagnosticCapture::enqueue_pick_summary(
  const std::string & goal_uuid, const std::string & vision_acquisition_id,
  uint8_t result_code, bool insertion_started, const std::string & evidence,
  std::size_t accepted_detail_count)
{
  const auto snapshot = nlohmann::json::parse(evidence, nullptr, false);
  auto candidate_snapshot = merge_pick_candidate_history(snapshot);
  const auto raw_material_attempts = snapshot.is_object() ?
    snapshot.value("material_candidate_attempts", nlohmann::json::array()) :
    nlohmann::json::array();
  auto material_attempts = nlohmann::json::array();
  if (raw_material_attempts.is_array()) {
    for (std::size_t index = 0; index < raw_material_attempts.size(); ++index) {
      const auto & attempt = raw_material_attempts[index];
      if (!attempt.is_object()) {continue;}
      nlohmann::json compact_attempt = {
        {"attempt_order", attempt.value("attempt_order", index + 1U)},
        {"candidate_id", attempt.value("candidate_id", std::string{})},
        {"result", attempt.value("result", std::string("unavailable"))},
        {"failure_stage", attempt.value("failure_stage", std::string("unavailable"))},
        {"pick_grasp_insertion_started", attempt.value(
          "pick_grasp_insertion_started", false)}};
      if (attempt.contains("result_code")) {
        compact_attempt["result_code"] = attempt["result_code"];
      }
      material_attempts.push_back(std::move(compact_attempt));
    }
  }
  nlohmann::json summary = {
    {"schema_version", 1}, {"record_type", "motion_pick_terminal_summary"},
    {"motion_goal_uuid", goal_uuid},
    {"vision_acquisition_id", vision_acquisition_id.empty() ?
      nlohmann::json(nullptr) : nlohmann::json(vision_acquisition_id)},
    {"task_executor_result_code", motion_result_code_name(result_code)},
    {"task_executor_result_code_value", result_code},
    {"pick_grasp_insertion_started", insertion_started},
    {"material_candidate_search", {
      {"attempt_count", raw_material_attempts.is_array() ? raw_material_attempts.size() : 0U},
      {"attempts", material_attempts},
      {"candidate_switch_occurred", material_attempts.size() > 1U},
      {"final_target_candidate_id", snapshot.is_object() ?
        snapshot.value("outcome_material_candidate_id", std::string{}) : std::string{}},
      {"grasp_pose_candidates_reference", "motion_planning_details"}}},
    {"candidate_counts", {{"scope", "GRASP_POSE"}, {"generated", 0},
      {"evaluated", 0}, {"unevaluated", 0}}},
    {"candidate_dispositions", nlohmann::json::object()},
    {"failure_stage_counts", nlohmann::json::object()},
    {"stage_failure_totals", {{"ik", 0}, {"fk", 0}, {"collision", 0},
      {"approach", 0}, {"cartesian", 0}, {"fallback", 0}}},
    {"approach_execution_count", 0}, {"insertion_execution_count", insertion_started ? 1 : 0},
    {"detail_coverage", {{"status", "unavailable"}, {"detail_references", nlohmann::json::array()},
      {"missing_count", 0}, {"missing_reason", "candidate_evidence_unavailable"}}}};
  if (snapshot.is_object()) {
    summary["material_candidate_search"]["acquisition_id"] =
      snapshot.value("vision_acquisition_id", vision_acquisition_id);
    const auto candidates = candidate_snapshot.value(
      "grasp_pose_candidates", nlohmann::json::array());
    summary["candidate_counts"]["generated"] = candidates.size();
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      const auto result = candidates[index].value("result", "NOT_EVALUATED");
      if (result == "NOT_EVALUATED") {
        summary["candidate_counts"]["unevaluated"] =
          summary["candidate_counts"]["unevaluated"].get<std::size_t>() + 1U;
      } else {
        summary["candidate_counts"]["evaluated"] =
          summary["candidate_counts"]["evaluated"].get<std::size_t>() + 1U;
      }
      summary["candidate_dispositions"][result] =
        summary["candidate_dispositions"].value(result, 0U) + 1U;
      if (result.find("IK") != std::string::npos) {
        summary["stage_failure_totals"]["ik"] =
          summary["stage_failure_totals"]["ik"].get<std::size_t>() + 1U;
      }
      if (result.find("APPROACH") != std::string::npos) {
        summary["stage_failure_totals"]["approach"] =
          summary["stage_failure_totals"]["approach"].get<std::size_t>() + 1U;
      }
      std::ostringstream ref;
      ref << "motion_planning_details/" << safe_goal_id(goal_uuid) << "_candidate_"
          << std::setw(3) << std::setfill('0') << index + 1U << ".json";
      summary["detail_coverage"]["detail_references"].push_back(ref.str());
    }
    summary["selected_candidate"] = candidate_snapshot.value("selected_candidate", -1);
    summary["selected_candidate_scope"] = "GRASP_POSE";
    summary["last_evaluated_candidate"] = candidate_snapshot.value("active_candidate", -1);
    const auto selected_index = summary["selected_candidate"].get<int>();
    summary["selected_candidate_number"] = selected_index >= 0 ?
      nlohmann::json(selected_index + 1) : nlohmann::json(nullptr);
    nlohmann::json last_evaluated_number = nullptr;
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      if (candidates[index].value("result", "NOT_EVALUATED") != "NOT_EVALUATED") {
        last_evaluated_number = index + 1U;
      }
    }
    summary["last_evaluated_candidate_number"] = last_evaluated_number;
    summary["final_failure_stage"] = snapshot.value("final_failure_stage", "unavailable");
    summary["final_failure_reason"] = snapshot.value("final_failure_reason", "unavailable");
    if (snapshot.contains("vision_occupancy_evidence")) {
      summary["vision_occupancy_evidence"] = snapshot["vision_occupancy_evidence"];
    }
    if (snapshot.contains("task_executor_elapsed_s")) {
      summary["task_executor_elapsed_s"] = snapshot["task_executor_elapsed_s"];
    }
    if (snapshot.contains("final_segment_planning")) {
      summary["final_segment_planning"] = snapshot["final_segment_planning"];
      const auto & final_segment = snapshot["final_segment_planning"];
      if (final_segment.is_object()) {
        if (final_segment.value("cartesian_result", "") != "SUCCESS") {
          summary["stage_failure_totals"]["cartesian"] = 1;
        }
        if (final_segment.contains("global_fallback_attempts") &&
          final_segment["global_fallback_attempts"].is_array())
        {
          for (const auto & attempt : final_segment["global_fallback_attempts"]) {
            if (attempt.value("result", "") != "SUCCESS") {
              summary["stage_failure_totals"]["fallback"] =
                summary["stage_failure_totals"]["fallback"].get<std::size_t>() + 1U;
            }
          }
        } else if (!final_segment.value("goal_ik_valid_and_collision_free", false) &&
          final_segment.value("cartesian_result", "") != "SUCCESS")
        {
          summary["stage_failure_totals"]["fallback"] = 1;
        }
        if (final_segment.value("final_planning_result", "") != "SUCCESS") {
          const bool fallback_attempted = final_segment.contains("global_fallback_attempts") &&
            final_segment["global_fallback_attempts"].is_array() &&
            !final_segment["global_fallback_attempts"].empty();
          summary["final_failure_stage"] = fallback_attempted ?
            "GLOBAL_FALLBACK" : "CARTESIAN_GRASP";
          summary["final_failure_reason"] = fallback_attempted ?
            final_segment["global_fallback_attempts"].back().value("result", "FAILED") :
            final_segment.value("cartesian_failure_reason", "FAILED");
        }
      }
    }
    summary["approach_attempt_count"] = candidate_snapshot.value(
      "approach_global_planning_call_count", 0U);
    if (candidate_snapshot.contains("aggregated_rejections") &&
      candidate_snapshot["aggregated_rejections"].is_object())
    {
      summary["stage_failure_totals"]["fk"] =
        candidate_snapshot["aggregated_rejections"].value("fk", 0U);
      summary["stage_failure_totals"]["collision"] =
        candidate_snapshot["aggregated_rejections"].value("collision_or_validity", 0U);
    }
    const auto count_attempts = [&](const nlohmann::json & source) {
        if (!source.is_object() || !source.contains("pick_attempts") ||
          !source["pick_attempts"].is_array()) {return;}
        for (const auto & attempt : source["pick_attempts"]) {
          const auto stage = attempt.value("stage", "unavailable");
          const auto reason = attempt.value("reason", "unavailable");
          auto & stage_counts = summary["failure_stage_counts"][stage];
          if (!stage_counts.is_object()) {stage_counts = nlohmann::json::object();}
          stage_counts[reason] = stage_counts.value(reason, 0U) + 1U;
        }
      };
    count_attempts(candidate_snapshot);
    if (snapshot.contains("task_accuracy") && snapshot["task_accuracy"].is_array()) {
      auto approach_results = nlohmann::json::array();
      for (const auto & accuracy : snapshot["task_accuracy"]) {
        if (accuracy.value("segment", "") == "APPROACH") {
          summary["approach_execution_count"] =
            summary["approach_execution_count"].get<std::size_t>() + 1U;
          approach_results.push_back({
            {"execution_result", accuracy.value("execution_result", "unavailable")},
            {"measurement_status", accuracy.value("measurement_status", "unavailable")},
            {"path_strategy", accuracy.value("path_strategy", "unavailable")},
            {"tcp_position_error_mm", accuracy.contains("tcp_position_error_mm") ?
              accuracy["tcp_position_error_mm"] : nlohmann::json(nullptr)},
            {"tcp_orientation_error_rad", accuracy.contains("tcp_orientation_error_rad") ?
              accuracy["tcp_orientation_error_rad"] : nlohmann::json(nullptr)}});
        }
      }
      summary["approach_execution_results"] = std::move(approach_results);
    }
    if (candidates.empty()) {
      summary["detail_coverage"]["status"] = "unavailable";
      summary["detail_coverage"]["missing_reason"] = "candidate_list_missing";
    } else {
      summary["detail_coverage"]["status"] = "pending";
      summary["detail_coverage"]["expected_count"] = candidates.size();
      summary["detail_coverage"]["accepted_count"] = accepted_detail_count;
      summary["detail_coverage"]["not_accepted_count"] =
        candidates.size() > accepted_detail_count ?
        candidates.size() - accepted_detail_count : 0U;
      summary["detail_coverage"]["missing_reason"] = "awaiting_writer_confirmation";
    }
  }
  if (!summary["detail_coverage"]["detail_references"].empty()) {
    auto revisions = nlohmann::json::object();
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & reference : summary["detail_coverage"]["detail_references"]) {
      const auto key = reference.get<std::string>();
      const auto tracked = detail_snapshots_.find(key);
      revisions[key] = tracked == detail_snapshots_.end() ? nlohmann::json(nullptr) :
        nlohmann::json(tracked->second.generation);
    }
    summary["detail_coverage"]["expected_detail_revisions"] = std::move(revisions);
  }
  auto serialized = summary.dump();
  if (serialized.size() > kMaxSummaryBytes) {
    summary["detail_coverage"]["detail_references"] = nlohmann::json::array();
    summary["detail_coverage"].erase("expected_detail_revisions");
    summary["detail_coverage"]["status"] = "unavailable";
    summary["detail_coverage"]["references_omitted"] = true;
    summary["detail_coverage"]["missing_reason"] = "summary_byte_limit";
    if (summary.contains("final_segment_planning") &&
      summary["final_segment_planning"].is_object())
    {
      const auto & final_segment = summary["final_segment_planning"];
      summary["final_segment_planning"] = {
        {"cartesian_result", final_segment.value("cartesian_result", "unavailable")},
        {"cartesian_failure_reason", final_segment.value("cartesian_failure_reason", "unavailable")},
        {"final_planning_result", final_segment.value("final_planning_result", "unavailable")},
        {"selected_strategy", final_segment.value("selected_strategy", "unavailable")},
        {"global_fallback_attempt_count",
          final_segment.contains("global_fallback_attempts") &&
          final_segment["global_fallback_attempts"].is_array() ?
          final_segment["global_fallback_attempts"].size() : 0U}};
    }
    if (summary.contains("vision_occupancy_evidence")) {
      summary["vision_occupancy_evidence"]["primitives"] = "OMITTED_SUMMARY_BYTE_LIMIT";
    }
    serialized = summary.dump();
    if (serialized.size() > kMaxSummaryBytes) {
      summary["failure_stage_counts"] = nlohmann::json::object();
      summary["failure_stage_counts_omitted"] = true;
      serialized = summary.dump();
    }
  }
  if (serialized.size() > kMaxSummaryBytes) {
    const auto full_attempts = summary["material_candidate_search"].value(
      "attempts", nlohmann::json::array());
    const std::size_t retained_final_count = full_attempts.empty() ? 0U : 1U;
    const std::size_t max_prefix_count = full_attempts.size() - retained_final_count;
    std::vector<nlohmann::json> omitted_failure_counts(
      full_attempts.size() + 1U, nlohmann::json::object());
    const auto omitted_attempt_limit = full_attempts.size() - retained_final_count;
    for (std::size_t index = omitted_attempt_limit; index > 0U; --index) {
      omitted_failure_counts[index - 1U] = omitted_failure_counts[index];
      const auto & attempt = full_attempts[index - 1U];
      const auto key = attempt.value("result", std::string("unavailable")) + ":" +
        attempt.value("failure_stage", std::string("unavailable"));
      omitted_failure_counts[index - 1U][key] =
        omitted_failure_counts[index - 1U].value(key, 0U) + 1U;
    }
    const auto summary_with_material_prefix = [&](std::size_t prefix_count) {
        auto bounded = summary;
        auto & material = bounded["material_candidate_search"];
        auto retained_attempts = nlohmann::json::array();
        for (std::size_t index = 0; index < prefix_count; ++index) {
          retained_attempts.push_back(full_attempts[index]);
        }
        if (retained_final_count > 0U) {
          retained_attempts.push_back(full_attempts.back());
        }
        material["attempts"] = std::move(retained_attempts);
        const std::size_t omitted_count = full_attempts.size() - prefix_count - retained_final_count;
        const auto omitted_begin = prefix_count;
        const auto omitted_end = full_attempts.size() - retained_final_count;
        if (omitted_count > 0U) {
          material["omitted_attempts"] = {
            {"count", omitted_count},
            {"attempt_order_range", {
              full_attempts[omitted_begin].value("attempt_order", omitted_begin + 1U),
              full_attempts[omitted_end - 1U].value("attempt_order", omitted_end)}},
            {"reason", "summary_byte_limit"},
            {"failure_counts_by_result_and_stage", omitted_failure_counts[omitted_begin]}};
        }
        return bounded;
      };
    std::size_t lower = 0U;
    std::size_t upper = max_prefix_count + 1U;
    while (lower < upper) {
      const auto middle = lower + (upper - lower) / 2U;
      const auto candidate = summary_with_material_prefix(middle);
      if (candidate.dump().size() <= kMaxSummaryBytes) {
        lower = middle + 1U;
      } else {
        upper = middle;
      }
    }
    const auto prefix_count = lower == 0U ? 0U : lower - 1U;
    summary = summary_with_material_prefix(prefix_count);
    serialized = summary.dump();
  }
  if (serialized.size() > kMaxSummaryBytes) {
    const auto material = summary["material_candidate_search"];
    const auto attempts = material.value("attempts", nlohmann::json::array());
    const auto retained_final_count = attempts.empty() ? 0U : 1U;
    const auto failure_counts = material.value(
      "omitted_attempts", nlohmann::json::object()).value(
      "failure_counts_by_result_and_stage", nlohmann::json::object());
    summary = {
      {"schema_version", 1}, {"record_type", "motion_pick_terminal_summary"},
      {"motion_goal_uuid", goal_uuid},
      {"vision_acquisition_id", vision_acquisition_id.empty() ?
        nlohmann::json(nullptr) : nlohmann::json(vision_acquisition_id)},
      {"task_executor_result_code", motion_result_code_name(result_code)},
      {"task_executor_result_code_value", result_code},
      {"pick_grasp_insertion_started", insertion_started},
      {"final_failure_stage", summary.value("final_failure_stage", "unavailable")},
      {"final_failure_reason", summary.value("final_failure_reason", "unavailable")},
      {"material_candidate_search", {
        {"attempt_count", raw_material_attempts.is_array() ? raw_material_attempts.size() : 0U},
        {"attempts", retained_final_count > 0U ? nlohmann::json::array({attempts.back()}) :
          nlohmann::json::array()},
        {"candidate_switch_occurred", material.value("candidate_switch_occurred", false)},
        {"final_target_candidate_id", material.value("final_target_candidate_id", std::string{})},
        {"acquisition_id", material.value("acquisition_id", vision_acquisition_id)},
        {"grasp_pose_candidates_reference", "motion_planning_details"},
        {"omitted_attempts", {
          {"count", raw_material_attempts.is_array() ?
            raw_material_attempts.size() - retained_final_count : 0U},
          {"reason", "summary_byte_limit"},
          {"failure_counts_by_result_and_stage", failure_counts}}}}},
      {"detail_coverage", {
        {"status", "unavailable"}, {"detail_references", nlohmann::json::array()},
        {"missing_reason", "summary_byte_limit"}, {"references_omitted", true}}}};
    serialized = summary.dump();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {return false;}
    const auto file_name = "motion_planning_summaries/" + safe_goal_id(goal_uuid) + ".json";
    const auto existing = std::find_if(
      summary_queue_.begin(), summary_queue_.end(), [&](const PendingRecord & pending) {
        return pending.file_name == file_name;
      });
    if (existing != summary_queue_.end()) {
      existing->serialized = std::move(serialized);
    } else if (summary_queue_.size() < kSummaryQueueCapacity) {
      summary_queue_.push_back({goal_uuid, file_name, std::move(serialized), false});
    } else {
      return false;
    }
  }
  condition_.notify_one();
  return true;
}

bool MotionPlanningDiagnosticCapture::enqueue_serialized(
  const std::string & goal_uuid, std::string serialized)
{
  if (serialized.size() > kMaxRecordBytes) {
    RCLCPP_WARN(
        logger_,
        "Motion planning diagnostic unavailable reason=record_byte_limit");
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {
      RCLCPP_WARN(
          logger_,
          "Motion planning diagnostic unavailable reason=writer_stopping");
      return false;
    }
    const auto queued = std::find_if(
      queue_.begin(), queue_.end(), [&goal_uuid](const PendingRecord & pending) {
        return pending.goal_uuid == goal_uuid;
      });
    if (queued != queue_.end()) {
      queued->serialized = std::move(serialized);
      return true;
    }
    if (queue_.size() >= kQueueCapacity) {
      RCLCPP_WARN(
          logger_,
          "Motion planning diagnostic unavailable reason=writer_queue_full");
      return false;
    }
    queue_.push_back({goal_uuid, {}, std::move(serialized)});
  }
  condition_.notify_one();
  return true;
}

void MotionPlanningDiagnosticCapture::queue_ready_summary_finalizations()
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto iterator = pending_summary_coverage_.begin();
    iterator != pending_summary_coverage_.end() &&
    summary_queue_.size() < kSummaryQueueCapacity;)
  {
    const auto & goal_uuid = iterator->first;
    const bool details_pending = std::any_of(
      detail_queue_.begin(), detail_queue_.end(), [&](const PendingRecord & detail) {
        return detail.goal_uuid == goal_uuid;
      }) || std::any_of(detail_snapshots_.begin(), detail_snapshots_.end(),
      [&](const auto & item) {
        return item.first.rfind("motion_planning_details/" + safe_goal_id(goal_uuid) +
          "_candidate_", 0U) == 0U && item.second.write_pending;
      });
    if (details_pending) {
      ++iterator;
      continue;
    }
    auto record = std::move(iterator->second);
    record.finalize_coverage = true;
    summary_queue_.push_back(std::move(record));
    iterator = pending_summary_coverage_.erase(iterator);
  }
}

void MotionPlanningDiagnosticCapture::clear_detail_snapshots_for_goal(
  const std::string & goal_uuid)
{
  const auto prefix = "motion_planning_details/" + safe_goal_id(goal_uuid) + "_candidate_";
  const auto occupancy_prefix = goal_uuid + '\0';
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto iterator = detail_snapshots_.begin(); iterator != detail_snapshots_.end();) {
    if (iterator->first.rfind(prefix, 0U) == 0U) {
      iterator = detail_snapshots_.erase(iterator);
    } else {
      ++iterator;
    }
  }
  for (auto iterator = occupancy_snapshot_paths_.begin();
    iterator != occupancy_snapshot_paths_.end();)
  {
    if (iterator->first.rfind(occupancy_prefix, 0U) == 0U) {
      iterator = occupancy_snapshot_paths_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

void MotionPlanningDiagnosticCapture::write_loop() noexcept
{
  for (;; ) {
    PendingRecord record;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]() {
          return stopping_ || !queue_.empty() || !detail_queue_.empty() || !summary_queue_.empty() ||
                 !occupancy_snapshot_queue_.empty();
        });
      if (queue_.empty() && detail_queue_.empty() && summary_queue_.empty() &&
        occupancy_snapshot_queue_.empty() && stopping_) {return;}
      if (!summary_queue_.empty()) {
        record = std::move(summary_queue_.front());
        summary_queue_.pop_front();
      } else if (!detail_queue_.empty()) {
        record = std::move(detail_queue_.front());
        detail_queue_.pop_front();
      } else if (!queue_.empty()) {
        record = std::move(queue_.front());
        queue_.pop_front();
      } else if (!occupancy_snapshot_queue_.empty()) {
        auto snapshot = std::move(occupancy_snapshot_queue_.front());
        occupancy_snapshot_queue_.pop_front();
        lock.unlock();
        try {
          const auto rendered = render_occupancy_snapshot(
            snapshot.primitives, snapshot.acquisition_id, snapshot.candidate_id);
          if (rendered.png.empty()) {
            throw std::runtime_error("PNG encoding failed");
          }
          save_occupancy_snapshot_png(
            std::filesystem::path(snapshot_directory_) / snapshot.file_name, rendered);
        } catch (const std::exception & error) {
          RCLCPP_WARN(logger_, "Motion occupancy snapshot save failed: %s", error.what());
        } catch (...) {
          RCLCPP_WARN(logger_, "Motion occupancy snapshot save failed");
        }
        continue;
      }
    }
    bool finalize_detail_coverage = false;
    bool persisted_record = false;
    try {
      const std::filesystem::path root(directory_);
      const auto destination = record.file_name.empty() ?
        root / (safe_goal_id(record.goal_uuid) + ".json") : root / record.file_name;
      if (record.file_name.rfind("motion_planning_summaries/", 0U) == 0U) {
        auto summary = nlohmann::json::parse(record.serialized, nullptr, false);
        if (summary.is_object() && summary["detail_coverage"]["detail_references"].is_array()) {
          std::size_t pending_count = 0U;
          if (!record.finalize_coverage) {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto queued_count = static_cast<std::size_t>(std::count_if(
                detail_queue_.begin(), detail_queue_.end(), [&](const PendingRecord & detail) {
                  return detail.goal_uuid == record.goal_uuid;
                }));
            const auto in_flight_count = static_cast<std::size_t>(std::count_if(
                detail_snapshots_.begin(), detail_snapshots_.end(), [&](const auto & item) {
                  return item.first.rfind("motion_planning_details/" +
                    safe_goal_id(record.goal_uuid) + "_candidate_", 0U) == 0U &&
                    item.second.write_pending && std::none_of(
                    detail_queue_.begin(), detail_queue_.end(), [&](const PendingRecord & detail) {
                      return detail.file_name == item.first;
                    });
                }));
            pending_count = queued_count + in_flight_count;
          }
          if (pending_count > 0U) {
            summary["detail_coverage"]["status"] = "pending";
            summary["detail_coverage"]["pending_count"] = pending_count;
            summary["detail_coverage"]["missing_reason"] = "detail_writes_pending";
            record.serialized = summary.dump();
            std::lock_guard<std::mutex> lock(mutex_);
            pending_summary_coverage_.insert_or_assign(record.goal_uuid, record);
          } else if (summary["detail_coverage"].value("references_omitted", false)) {
            summary["detail_coverage"]["status"] = "unavailable";
            summary["detail_coverage"]["missing_reason"] = "summary_byte_limit";
            finalize_detail_coverage = true;
            record.serialized = summary.dump();
          } else if (summary["detail_coverage"].value("status", "") == "unavailable") {
            // No candidate evidence means there is no basis for a coverage claim.
            finalize_detail_coverage = true;
            record.serialized = summary.dump();
          } else {
            auto missing = nlohmann::json::array();
            auto unverified = nlohmann::json::array();
            std::size_t persisted_count = 0U;
            std::size_t unverified_count = 0U;
            const auto & references = summary["detail_coverage"]["detail_references"];
            const auto expected_revisions = summary["detail_coverage"].value(
              "expected_detail_revisions", nlohmann::json::object());
            const auto record_root = destination.parent_path().parent_path();
            for (const auto & reference : references) {
              std::error_code reference_error;
              bool persisted = false;
              const auto reference_text = reference.is_string() ?
                reference.get<std::string>() : std::string();
              const auto expected = expected_revisions.value(reference_text, nlohmann::json());
              const auto name = std::filesystem::path(reference_text).stem().string();
              const auto marker = name.rfind("_candidate_");
              std::size_t candidate_number = 0U;
              if (marker != std::string::npos) {
                try {candidate_number = static_cast<std::size_t>(std::stoul(name.substr(marker + 11U)));}
                catch (...) {candidate_number = 0U;}
              }
              const bool known_revision = expected.is_number_unsigned() || expected.is_number_integer();
              if (known_revision && reference.is_string() &&
                std::filesystem::is_regular_file(record_root / reference_text, reference_error) &&
                !reference_error)
              {
                nlohmann::json detail_record;
                std::ifstream input(record_root / reference_text, std::ios::binary);
                try {
                  input >> detail_record;
                  persisted = detail_record.is_object() &&
                    detail_record.value("detail_revision", uint64_t{0U}) ==
                    expected.get<std::uint64_t>();
                } catch (...) {persisted = false;}
              }
              if (persisted) {
                ++persisted_count;
              } else if (known_revision) {
                if (candidate_number > 0U) {missing.push_back(candidate_number);}
              } else {
                ++unverified_count;
                if (candidate_number > 0U) {unverified.push_back(candidate_number);}
              }
            }
            const auto missing_count = missing.size();
            summary["detail_coverage"]["persisted_count"] = persisted_count;
            summary["detail_coverage"]["missing_count"] = missing_count;
            summary["detail_coverage"]["missing_candidate_numbers"] = std::move(missing);
            summary["detail_coverage"]["unverified_count"] = unverified_count;
            summary["detail_coverage"]["unverified_candidate_numbers"] = std::move(unverified);
            summary["detail_coverage"]["pending_count"] = 0U;
            summary["detail_coverage"]["status"] = missing_count > 0U ? "missing" :
              (unverified_count > 0U ? "unavailable" : "persisted");
            summary["detail_coverage"]["missing_reason"] = missing_count > 0U ?
              nlohmann::json("detail_revision_missing_or_mismatch") :
              (unverified_count > 0U ? nlohmann::json("detail_revision_unavailable") :
              nlohmann::json(nullptr));
            finalize_detail_coverage = true;
            record.serialized = summary.dump();
          }
        }
      }
      const auto directory = destination.parent_path();
      std::filesystem::create_directories(directory);
      std::uintmax_t retained_bytes = 0;
      for (const auto & entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
          retained_bytes += entry.file_size();
        }
      }
      if (std::filesystem::exists(destination)) {
        retained_bytes -= std::min<std::uintmax_t>(
          retained_bytes, std::filesystem::file_size(destination));
      }
      if (retained_bytes + record.serialized.size() > kMaxDirectoryBytes) {
        RCLCPP_WARN(logger_, "Motion planning diagnostic unavailable "
                             "reason=directory_byte_limit");
      } else {
        const auto temporary = destination.string() + ".tmp";
        {
          std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
          if (!output) {throw std::runtime_error("open failed");}
          output << record.serialized << '\n';
          output.flush();
          if (!output) {throw std::runtime_error("write failed");}
        }
        std::filesystem::rename(temporary, destination);
        persisted_record = true;
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(logger_, "Motion planning diagnostic save failed cause=%s",
                  error.what());
    } catch (...) {
      RCLCPP_WARN(logger_, "Motion planning diagnostic save failed");
    }
    if (record.file_name.rfind("motion_planning_details/", 0U) == 0U) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto tracked = detail_snapshots_.find(record.file_name);
        if (tracked != detail_snapshots_.end() &&
          tracked->second.generation == record.detail_generation)
        {
          tracked->second.write_pending = false;
          tracked->second.persisted = persisted_record;
        }
      }
      queue_ready_summary_finalizations();
    } else if (finalize_detail_coverage) {
      clear_detail_snapshots_for_goal(record.goal_uuid);
    }
    queue_ready_summary_finalizations();
  }
}

std::string MotionPlanningDiagnosticCapture::safe_goal_id(std::string value)
{
  std::replace_if(
    value.begin(), value.end(), [](unsigned char character) {
      return !std::isalnum(character) && character != '-' && character != '_';
    }, '_');
  return value.empty() ? "unknown-goal" : value;
}

void IsaacRosMotionTransport::load_place_waypoint_trajectory()
{
  if (place_waypoint_trajectory_load_attempted_) {return;}
  place_waypoint_trajectory_load_attempted_ = true;
  if (!place_waypoint_trajectory_file_.empty()) {
    try {
      std::ifstream input(place_waypoint_trajectory_file_);
      if (!input) {throw std::runtime_error("trajectory file is missing");}
      const auto artifact = nlohmann::json::parse(input);
      if (artifact.at("schema_version") != 1 ||
        artifact.at("robot_model") != "m0609_robotiq_2f85" ||
        artifact.at("planning_group") != "arm" ||
        artifact.at("planning_config").at("planner_id") != planning_planner_id_ ||
        artifact.at("planning_config").at("velocity_scaling") !=
        planning_velocity_scaling_factor_ ||
        artifact.at("planning_config").at("acceleration_scaling") !=
        planning_acceleration_scaling_factor_ ||
        artifact.at("joint_names").get<std::vector<std::string>>() != arm_joint_names_ ||
        artifact.at("home_joint_positions").get<std::vector<double>>() != home_joint_positions_)
      {throw std::runtime_error("trajectory identity does not match the active profile");}
      place_waypoint_object_target_signature_ =
        artifact.at("object_target_pose").get<std::vector<double>>();
      place_waypoint_tcp_target_signature_ =
        artifact.at("approach_tcp_pose").get<std::vector<double>>();
      if (place_waypoint_object_target_signature_.size() != 7U ||
        place_waypoint_tcp_target_signature_.size() != 7U)
      {throw std::runtime_error("trajectory target identity is malformed");}
      NormalizedTrajectoryPlan loaded;
      loaded.trajectory.joint_names = arm_joint_names_;
      loaded.planning_scales.max_velocity =
        artifact.at("planning_config").at("planned_velocity_scale").get<double>();
      loaded.planning_scales.max_acceleration =
        artifact.at("planning_config").at("planned_acceleration_scale").get<double>();
      if (!std::isfinite(loaded.planning_scales.max_velocity) ||
        !std::isfinite(loaded.planning_scales.max_acceleration) ||
        loaded.planning_scales.max_velocity <= 0.0 ||
        loaded.planning_scales.max_velocity > 1.0 ||
        loaded.planning_scales.max_acceleration <= 0.0 ||
        loaded.planning_scales.max_acceleration > 1.0)
      {throw std::runtime_error("trajectory planning scales are invalid");}
      std::int64_t previous_time_ns = -1;
      for (const auto & item : artifact.at("points")) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = item.at("positions").get<std::vector<double>>();
        point.velocities = item.at("velocities").get<std::vector<double>>();
        point.accelerations = item.at("accelerations").get<std::vector<double>>();
        point.effort = item.at("effort").get<std::vector<double>>();
        point.time_from_start.sec = item.at("time_sec").get<int32_t>();
        point.time_from_start.nanosec = item.at("time_nanosec").get<uint32_t>();
        if (point.positions.size() != arm_joint_names_.size()) {
          throw std::runtime_error("trajectory point joint count mismatch");
        }
        if (point.velocities.size() != arm_joint_names_.size() ||
          point.accelerations.size() != arm_joint_names_.size() ||
          point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000U)
        {throw std::runtime_error("trajectory point timing or derivatives are incomplete");}
        const auto finite_values = [](const std::vector<double> & values) {
            return std::all_of(values.begin(), values.end(),
              [](double value) {return std::isfinite(value);});
          };
        const auto time_ns = static_cast<std::int64_t>(point.time_from_start.sec) * 1000000000LL +
          point.time_from_start.nanosec;
        if (!finite_values(point.positions) || !finite_values(point.velocities) ||
          !finite_values(point.accelerations) || time_ns <= previous_time_ns)
        {throw std::runtime_error("trajectory contains non-finite or unordered points");}
        previous_time_ns = time_ns;
        loaded.trajectory.points.push_back(std::move(point));
      }
      if (loaded.trajectory.points.size() < 2U) {
        throw std::runtime_error("trajectory has fewer than two points");
      }
      for (std::size_t joint = 0; joint < home_joint_positions_.size(); ++joint) {
        if (std::abs(loaded.trajectory.points.front().positions[joint] -
          home_joint_positions_[joint]) > 1.0e-6)
        {throw std::runtime_error("trajectory start state does not match HOME");}
      }
      std::lock_guard<std::mutex> lock(place_waypoint_cache_mutex_);
      cached_place_waypoint_approach_ = std::move(loaded);
      RCLCPP_DEBUG(node_->get_logger(), "PLACE WAYPOINT trajectory loaded");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_->get_logger(), "PLACE WAYPOINT trajectory unavailable: %s",
        error.what());
    }
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
  double cartesian_tcp_orientation_tolerance_rad,
  double motion_progress_stall_timeout_s,
  double trajectory_goal_tolerance_rad,
  double planning_velocity_scaling_factor,
  double planning_acceleration_scaling_factor,
  double planning_time_s,
  double place_candidate_planning_time_s,
  double place_total_planning_time_s,
  bool diagnostics_capture_enabled,
  std::string diagnostics_output_directory,
  std::string snapshot_run_directory,
  std::string planning_planner_id,
  std::string place_waypoint_trajectory_file)
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
  planning_planner_id_(std::move(planning_planner_id)),
  place_candidate_planning_time_s_(place_candidate_planning_time_s),
  place_total_planning_time_s_(place_total_planning_time_s),
  trajectory_sampler_name_(trajectory_sampler_name),
  cartesian_tcp_orientation_tolerance_rad_(cartesian_tcp_orientation_tolerance_rad),
  trajectory_goal_tolerance_rad_(trajectory_goal_tolerance_rad),
  diagnostics_capture_enabled_(diagnostics_capture_enabled),
  diagnostics_output_directory_(std::move(diagnostics_output_directory)),
  snapshot_run_directory_(std::move(snapshot_run_directory)),
  place_waypoint_trajectory_file_(std::move(place_waypoint_trajectory_file)),
  trajectory_watchdog_(trajectory_clock_liveness_timeout_s)
{
  if (!node_->has_parameter("pick_ik_solver")) {
    node_->declare_parameter<std::string>(
      "pick_ik_solver", "kdl_kinematics_plugin/KDLKinematicsPlugin");
  }
  if (!node_->has_parameter("pick_ik_solve_type")) {
    node_->declare_parameter<std::string>("pick_ik_solve_type", "");
  }
  if (!node_->has_parameter("pick_ik_timeout_s")) {
    node_->declare_parameter<double>("pick_ik_timeout_s", 1.0);
  }
  if (!node_->has_parameter("pick_grasp_ik_timeout_s")) {
    node_->declare_parameter<double>("pick_grasp_ik_timeout_s", 1.0);
  }
  if (!node_->has_parameter("pick_approach_ik_timeout_s")) {
    node_->declare_parameter<double>("pick_approach_ik_timeout_s", 1.0);
  }
  if (!node_->has_parameter("pick_search_budget_s")) {
    node_->declare_parameter<double>("pick_search_budget_s", 100.0);
  }
  if (!node_->has_parameter("pick_max_withdrawal_tilt_deg")) {
    node_->declare_parameter<double>("pick_max_withdrawal_tilt_deg", 90.0);
  }
  if (!node_->has_parameter("grasp_occupancy_mode")) {
    node_->declare_parameter<std::string>("grasp_occupancy_mode", "FULL_COLLISION");
  }
  grasp_occupancy_mode_ = parse_grasp_occupancy_mode(
    node_->get_parameter("grasp_occupancy_mode").as_string());
  const auto read_finger_dimension = [this](const char * name, double default_value) {
      if (!node_->has_parameter(name)) {node_->declare_parameter<double>(name, default_value);}
      return node_->get_parameter(name).as_double();
    };
  finger_clearance_.radius_m = read_finger_dimension("finger_clearance_radius_m", 0.010);
  finger_clearance_.contact_span_m = read_finger_dimension("finger_contact_span_m", 0.020);
  finger_clearance_.depth_m = read_finger_dimension("finger_clearance_depth_m", 0.025);
  validate_finger_clearance_parameters(finger_clearance_);
  if (!node_->has_parameter("pick_grasp_pose_order_policy")) {
    node_->declare_parameter<std::string>(
      "pick_grasp_pose_order_policy", "distance_then_tilt");
  }
  pick_grasp_pose_order_policy_ =
    node_->get_parameter("pick_grasp_pose_order_policy").as_string();
  (void)parse_pick_grasp_pose_order_policy(pick_grasp_pose_order_policy_);
  if (!node_->has_parameter("pick_position_range_half_length_fraction")) {
    node_->declare_parameter<double>("pick_position_range_half_length_fraction", 0.5);
  }
  if (!node_->has_parameter("pick_max_downward_position_offset_m")) {
    node_->declare_parameter<double>("pick_max_downward_position_offset_m", 0.005);
  }
  if (!node_->has_parameter("pick_position_offset_samples_per_tangent_axis")) {
    node_->declare_parameter<int64_t>("pick_position_offset_samples_per_tangent_axis", 3);
  }
  if (!node_->has_parameter("pick_orientation_samples_per_closing_axis")) {
    node_->declare_parameter<int64_t>("pick_orientation_samples_per_closing_axis", 8);
  }
  if (!node_->has_parameter("pick_grasp_pose_candidate_limit")) {
    node_->declare_parameter<int64_t>("pick_grasp_pose_candidate_limit", 48);
  }
  if (!node_->has_parameter("pick_ik_seed_attempts_per_grasp_pose")) {
    node_->declare_parameter<int64_t>("pick_ik_seed_attempts_per_grasp_pose", 2);
  }
  if (!node_->has_parameter("planning_num_attempts")) {
    node_->declare_parameter<int64_t>("planning_num_attempts", 3);
  }
  const auto declare_positive_limit = [&](const char * name, int64_t fallback) {
      if (!node_->has_parameter(name)) {node_->declare_parameter<int64_t>(name, fallback);}
      const auto value = node_->get_parameter(name).as_int();
      if (value < 1 || value > 100) {
        throw std::invalid_argument(std::string(name) + " must be in [1, 100]");
      }
      return static_cast<std::size_t>(value);
    };
  const auto declare_stage_limit = [&](const char * name, int64_t fallback) {
      if (!node_->has_parameter(name)) {node_->declare_parameter<int64_t>(name, fallback);}
      const auto value = node_->get_parameter(name).as_int();
      if (value < 1 || value > 20) {
        throw std::invalid_argument(std::string(name) + " must be in [1, 20]");
      }
      return static_cast<std::size_t>(value);
    };
  if (!node_->has_parameter("place_tray_return_joint_positions")) {
    node_->declare_parameter<std::vector<double>>(
      "place_tray_return_joint_positions", std::vector<double>{});
  }
  place_tray_return_joint_positions_ =
    node_->get_parameter("place_tray_return_joint_positions").as_double_array();
  if (!place_tray_return_joint_positions_.empty() &&
    (place_tray_return_joint_positions_.size() != arm_joint_names_.size() ||
    !std::all_of(place_tray_return_joint_positions_.begin(),
      place_tray_return_joint_positions_.end(), [](double v) {return std::isfinite(v);})))
  {
    throw std::invalid_argument(
      "place_tray_return_joint_positions must be finite and follow arm_joint_names");
  }
  place_pose_candidate_limit_ = declare_positive_limit("place_max_pose_candidates", 13);
  pick_approach_global_max_requests_ = declare_stage_limit(
    "pick_approach_global_max_requests", 3);
  pick_final_cartesian_max_requests_ = declare_stage_limit(
    "pick_final_cartesian_max_requests", 1);
  pick_final_global_fallback_max_requests_ = declare_stage_limit(
    "pick_final_global_fallback_max_requests", 3);
  place_approach_global_max_requests_ = declare_stage_limit(
    "place_approach_global_max_requests", 3);
  place_final_cartesian_max_requests_ = declare_stage_limit(
    "place_final_cartesian_max_requests", 1);
  place_final_global_fallback_max_requests_ = declare_stage_limit(
    "place_final_global_fallback_max_requests", 3);
  pick_ik_solver_name_ = node_->get_parameter("pick_ik_solver").as_string();
  pick_ik_solve_type_ = node_->get_parameter("pick_ik_solve_type").as_string();
  pick_ik_timeout_s_ = node_->get_parameter("pick_ik_timeout_s").as_double();
  pick_grasp_ik_timeout_s_ = node_->get_parameter("pick_grasp_ik_timeout_s").as_double();
  pick_approach_ik_timeout_s_ = node_->get_parameter("pick_approach_ik_timeout_s").as_double();
  if (!node_->has_parameter("pick_candidate_search_budget_s")) {
    node_->declare_parameter<double>("pick_candidate_search_budget_s", 15.0);
  }
  pick_candidate_search_budget_s_ =
    node_->get_parameter("pick_candidate_search_budget_s").as_double();
  pick_search_budget_s_ = node_->get_parameter("pick_search_budget_s").as_double();
  pick_max_withdrawal_tilt_deg_ =
    node_->get_parameter("pick_max_withdrawal_tilt_deg").as_double();
  pick_position_range_half_length_fraction_ =
    node_->get_parameter("pick_position_range_half_length_fraction").as_double();
  pick_max_downward_position_offset_m_ =
    node_->get_parameter("pick_max_downward_position_offset_m").as_double();
  const auto position_samples =
    node_->get_parameter("pick_position_offset_samples_per_tangent_axis").as_int();
  const auto orientation_samples =
    node_->get_parameter("pick_orientation_samples_per_closing_axis").as_int();
  if (!std::isfinite(pick_max_withdrawal_tilt_deg_) || pick_max_withdrawal_tilt_deg_ < 0.0 ||
    pick_max_withdrawal_tilt_deg_ > 180.0 ||
    !std::isfinite(pick_position_range_half_length_fraction_) ||
    pick_position_range_half_length_fraction_ < 0.0 ||
    pick_position_range_half_length_fraction_ >= 1.0 ||
    !std::isfinite(pick_max_downward_position_offset_m_) ||
    pick_max_downward_position_offset_m_ < 0.0 ||
    position_samples < 1 || position_samples > 20 ||
    orientation_samples < 1 || orientation_samples > 72)
  {
    throw std::invalid_argument("invalid PICK grasp search sampling configuration");
  }
  pick_position_offset_samples_per_tangent_axis_ = static_cast<std::size_t>(position_samples);
  pick_orientation_samples_per_closing_axis_ = static_cast<std::size_t>(orientation_samples);
  const auto configured_pick_grasp_pose_candidate_limit =
    node_->get_parameter("pick_grasp_pose_candidate_limit").as_int();
  if (configured_pick_grasp_pose_candidate_limit < 3 ||
    configured_pick_grasp_pose_candidate_limit > 1000)
  {
    throw std::invalid_argument("pick_grasp_pose_candidate_limit must be in [3, 1000]");
  }
  pick_grasp_pose_candidate_limit_ =
    static_cast<std::size_t>(configured_pick_grasp_pose_candidate_limit);
  const auto configured_pick_ik_seed_attempts_per_grasp_pose =
    node_->get_parameter("pick_ik_seed_attempts_per_grasp_pose").as_int();
  if (configured_pick_ik_seed_attempts_per_grasp_pose < 1) {
    throw std::invalid_argument("pick_ik_seed_attempts_per_grasp_pose must be at least 1");
  }
  pick_ik_seed_attempts_per_grasp_pose_ =
    static_cast<std::size_t>(configured_pick_ik_seed_attempts_per_grasp_pose);
  const auto configured_planning_num_attempts =
    node_->get_parameter("planning_num_attempts").as_int();
  if (configured_planning_num_attempts < 1 ||
    configured_planning_num_attempts > std::numeric_limits<unsigned int>::max())
  {
    throw std::invalid_argument("planning_num_attempts must be a positive unsigned integer");
  }
  planning_num_attempts_ = static_cast<unsigned int>(configured_planning_num_attempts);
  const bool trac = pick_ik_solver_name_ ==
    "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin";
  const bool kdl = pick_ik_solver_name_ ==
    "kdl_kinematics_plugin/KDLKinematicsPlugin";
  if (!trac && !kdl) {
    throw std::invalid_argument("unsupported PICK IK solver: " + pick_ik_solver_name_);
  }
  if (!std::isfinite(pick_ik_timeout_s_) || pick_ik_timeout_s_ <= 0.0) {
    throw std::invalid_argument("pick_ik_timeout_s must be finite and positive");
  }
  if (!std::isfinite(pick_grasp_ik_timeout_s_) || pick_grasp_ik_timeout_s_ <= 0.0 ||
    pick_grasp_ik_timeout_s_ > 30.0 ||
    !std::isfinite(pick_approach_ik_timeout_s_) || pick_approach_ik_timeout_s_ <= 0.0 ||
    pick_approach_ik_timeout_s_ > 30.0 || !std::isfinite(pick_search_budget_s_) ||
    pick_search_budget_s_ <= 0.0 || pick_search_budget_s_ > 600.0 ||
    !std::isfinite(pick_candidate_search_budget_s_) ||
    pick_candidate_search_budget_s_ <= 0.0 || pick_candidate_search_budget_s_ > 600.0)
  {
    throw std::invalid_argument("invalid PICK search budget or 6D IK timeout");
  }
  if (kdl && !pick_ik_solve_type_.empty()) {
    throw std::invalid_argument("KDL does not support pick_ik_solve_type");
  }
  if (trac && pick_ik_solve_type_ != "Speed" && pick_ik_solve_type_ != "Distance" &&
    pick_ik_solve_type_ != "Manipulation1" && pick_ik_solve_type_ != "Manipulation2" &&
    pick_ik_solve_type_ != "Manipulation3")
  {
    throw std::invalid_argument("unsupported TRAC-IK pick_ik_solve_type");
  }
  planning_diagnostic_capture_ = std::make_shared<MotionPlanningDiagnosticCapture>(
    diagnostics_output_directory_ + "/motion_planning_records", node_->get_logger(),
    snapshot_run_directory_);
  if (diagnostics_capture_enabled) {
    const char * requested_capture_id = std::getenv("ARM_CELL_OBSERVABILITY_CAPTURE_ID");
    clock_diagnostic_capture_ = std::make_unique<ClockDiagnosticCapture>(
      diagnostics_output_directory_, std::string(node_->get_name()) + "-pid-" +
      std::to_string(static_cast<unsigned long long>(::getpid())),
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(trajectory_watchdog_.liveness_timeout_s())),
      requested_capture_id == nullptr ? "" : requested_capture_id);
  }
  if (!std::isfinite(trajectory_goal_tolerance_rad_) || trajectory_goal_tolerance_rad_ <= 0.0) {
    throw std::invalid_argument("trajectory goal tolerance must be finite and positive");
  }
  if (!std::isfinite(cartesian_tcp_orientation_tolerance_rad_) ||
    cartesian_tcp_orientation_tolerance_rad_ <= 0.0)
  {
    throw std::invalid_argument("Cartesian TCP orientation tolerance must be finite and positive");
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
  gripper_close_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "/gripper/close", 10);
  capture_reference_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "/gripper/capture_reference", rclcpp::QoS(1).reliable().durability_volatile());
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
  std::string execution_id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_object_to_tcp_valid_ = false;
    accepted_object_to_tcp_valid_ = false;
    execution_id = capture_reference_id_;
    capture_reference_id_.clear();
    applied_capture_reference_id_.clear();
  }
  if (capture_reference_publisher_ && !execution_id.empty()) {
    std_msgs::msg::String invalidation;
    invalidation.data = "{\"motion_goal_uuid\":\"" + execution_id + "\",\"invalidate\":true}";
    capture_reference_publisher_->publish(invalidation);
  }
}

void IsaacRosMotionTransport::stage_selected_pick_grasp_relation(
  const geometry_msgs::msg::Transform & relation)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_object_to_tcp_ = relation;
  pending_object_to_tcp_valid_ = true;
}

bool IsaacRosMotionTransport::stage_pick_capture_reference(
  const geometry_msgs::msg::PoseStamped & pose, double width_mm, uint8_t symmetry_order,
  const std::string & execution_id)
{
  if (!capture_reference_publisher_ || execution_id.empty() || pose.header.frame_id.empty() ||
    !std::isfinite(width_mm) || width_mm <= 0.0 ||
    !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
    !std::isfinite(pose.pose.position.z) || !std::isfinite(pose.pose.orientation.x) ||
    !std::isfinite(pose.pose.orientation.y) || !std::isfinite(pose.pose.orientation.z) ||
    !std::isfinite(pose.pose.orientation.w) ||
    (symmetry_order > 4U || symmetry_order == 3U))
  {
    return false;
  }
  std_msgs::msg::String message;
  std::ostringstream payload;
  payload.precision(17);
  payload << "{\"motion_goal_uuid\":\"" << json_escape(execution_id)
          << "\",\"frame_id\":\"" << json_escape(pose.header.frame_id) << "\",\"position\":["
          << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z
          << "],\"orientation_xyzw\":[" << pose.pose.orientation.x << ","
          << pose.pose.orientation.y << "," << pose.pose.orientation.z << ","
          << pose.pose.orientation.w << "],\"expected_contact_width_mm\":" << width_mm
          << ",\"orientation_symmetry_order\":" << static_cast<unsigned>(symmetry_order) << "}";
  message.data = payload.str();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    capture_reference_id_ = execution_id;
    applied_capture_reference_id_.clear();
  }
  capture_reference_publisher_->publish(message);
  return true;
}

void IsaacRosMotionTransport::discard_pending_grasp_relation()
{
  std::string execution_id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_object_to_tcp_valid_ = false;
    execution_id = capture_reference_id_;
    capture_reference_id_.clear();
    applied_capture_reference_id_.clear();
  }
  if (capture_reference_publisher_ && !execution_id.empty()) {
    std_msgs::msg::String invalidation;
    invalidation.data = "{\"motion_goal_uuid\":\"" + execution_id + "\",\"invalidate\":true}";
    capture_reference_publisher_->publish(invalidation);
  }
}

bool IsaacRosMotionTransport::commit_pending_grasp_relation()
{
  std::string execution_id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_object_to_tcp_valid_) {
      return false;
    }
    accepted_object_to_tcp_ = pending_object_to_tcp_;
    accepted_object_to_tcp_valid_ = true;
    pending_object_to_tcp_valid_ = false;
    execution_id = capture_reference_id_;
    capture_reference_id_.clear();
    applied_capture_reference_id_.clear();
  }
  if (capture_reference_publisher_ && !execution_id.empty()) {
    std_msgs::msg::String invalidation;
    invalidation.data = "{\"motion_goal_uuid\":\"" + execution_id + "\",\"invalidate\":true}";
    capture_reference_publisher_->publish(invalidation);
  }
  RCLCPP_DEBUG(
    node_->get_logger(),
    "PICK accepted the object-to-gripper transform for this execution");
  return true;
}

void IsaacRosMotionTransport::clear_grasp_relation_after_release()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_object_to_tcp_valid_ = false;
  accepted_object_to_tcp_valid_ = false;
}

void IsaacRosMotionTransport::configure_waypoint_mode(
  bool pick_waypoint, bool place_waypoint)
{
  if (place_waypoint) {load_place_waypoint_trajectory();}
  if (diagnostics_capture_enabled_ && (pick_waypoint || place_waypoint)) {
    profile_trace_ = std::make_shared<MotionProfileTrace>(
      diagnostics_output_directory_ + "/motion_profile_trace", 200000);
    trajectory_worker_->set_profile_trace(profile_trace_);
  } else {
    profile_trace_.reset();
    trajectory_worker_->set_profile_trace(nullptr);
  }
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
  const auto planning_planner_id = planning_planner_id_;
  const auto pick_ik_solver_name = pick_ik_solver_name_;
  const auto pick_ik_solve_type = pick_ik_solve_type_;
  const auto pick_ik_timeout_s = pick_ik_timeout_s_;
  move_group_worker_ = std::thread(
    [state, node, velocity_scaling_factor, acceleration_scaling_factor, planning_time_s,
      planning_planner_id, pick_ik_solver_name, pick_ik_solve_type,
      pick_ik_timeout_s]() {
      try {
        using MoveGroupAction = moveit_msgs::action::MoveGroup;
        const auto discovery_deadline =
        std::chrono::steady_clock::now() + kMoveGroupServerWait;
        auto move_action_client = rclcpp_action::create_client<MoveGroupAction>(
          node, "move_action");
        const auto remaining = discovery_deadline - std::chrono::steady_clock::now();
        if (remaining <= std::chrono::steady_clock::duration::zero() ||
        !move_action_client->wait_for_action_server(
          std::chrono::duration_cast<std::chrono::nanoseconds>(remaining)))
        {
          throw std::runtime_error("MoveGroup action server /move_action unavailable");
        }

        auto move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          node, "arm", std::shared_ptr<tf2_ros::Buffer>(),
          rclcpp::Duration::from_seconds(kMoveGroupServerWait.count()));

        if (!move_group->getRobotModel()) {
          throw std::runtime_error("MoveGroup construction returned without a robot model");
        }
        move_group->setMaxVelocityScalingFactor(velocity_scaling_factor);
        move_group->setMaxAccelerationScalingFactor(acceleration_scaling_factor);
        move_group->setPlanningTime(planning_time_s);
        if (!planning_planner_id.empty()) {
          move_group->setPlannerId(planning_planner_id);
        }
        move_group->setWorkspace(-1.0, -1.0, -0.2, 1.0, 1.0, 1.5);

        std::vector<rclcpp::Parameter> pick_parameters{
          rclcpp::Parameter("robot_description_kinematics.arm.position_only_ik", false)};
        if (!pick_ik_solve_type.empty()) {
          pick_parameters.emplace_back(
            "robot_description_kinematics.arm.solve_type", pick_ik_solve_type);
        }
        rclcpp::NodeOptions pick_options;
        pick_options.use_global_arguments(false);
        pick_options.parameter_overrides(pick_parameters);
        auto pick_node = std::make_shared<rclcpp::Node>("pick_ik_backend", pick_options);
        // This private backend's parameter library emits raw parameter dumps at DEBUG.
        // Keep meaningful initialization summaries on the Motion logger without
        // flooding debug logs with configuration internals.
        pick_node->get_logger().set_level(rclcpp::Logger::Level::Info);
        auto pick_loader = std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>(
          "moveit_core", "kinematics::KinematicsBase");
        auto pick_solver = pick_loader->createSharedInstance(pick_ik_solver_name);
        if (!pick_solver || !pick_solver->initialize(
            pick_node, *move_group->getRobotModel(), "arm", "base_link", {"sf_grasp_tcp"}, 0.005))
        {
          throw std::runtime_error("PICK IK plugin initialization failed: " +
                  pick_ik_solver_name);
        }
        pick_solver->setDefaultTimeout(pick_ik_timeout_s);
        RCLCPP_DEBUG(
          node->get_logger(), "PICK IK initialized solver=%s mode=%s",
          pick_ik_solver_name.c_str(), pick_ik_solve_type.empty() ? "KDL" :
          pick_ik_solve_type.c_str());


        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->in_progress) {
          state->move_group = std::move(move_group);
          state->pick_ik_loader = std::move(pick_loader);
          state->pick_ik_solver = std::move(pick_solver);
          state->pick_ik_node = std::move(pick_node);
          state->ready = true;
          state->in_progress = false;
          state->failure_reason.clear();
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

bool IsaacRosMotionTransport::measured_tcp_pose(geometry_msgs::msg::PoseStamped & pose) const
{
  if (!current_state_fresh()) {return false;}
  const auto state = move_group_state_;
  if (!state) {return false;}
  std::lock_guard<std::mutex> lock(state->mutex);
  if (!state->ready || !state->move_group) {return false;}
  const auto measured = state->move_group->getCurrentPose("sf_grasp_tcp");
  if (measured.header.frame_id.empty() ||
    !std::isfinite(measured.pose.position.x) || !std::isfinite(measured.pose.position.y) ||
    !std::isfinite(measured.pose.position.z) ||
    !std::isfinite(measured.pose.orientation.x) ||
    !std::isfinite(measured.pose.orientation.y) ||
    !std::isfinite(measured.pose.orientation.z) ||
    !std::isfinite(measured.pose.orientation.w))
  {
    return false;
  }
  pose = measured;
  return true;
}

bool IsaacRosMotionTransport::available() const
{
  std::unique_lock<std::mutex> lock(move_group_state_->mutex, std::try_to_lock);
  if (!lock.owns_lock()) {
    return false;
  }
  return (!move_group_state_->required ||
         (move_group_state_->ready && !move_group_state_->in_progress &&
         move_group_state_->move_group && move_group_state_->pick_ik_solver)) &&
         current_state_fresh();
}

void IsaacRosMotionTransport::begin_pick_search()
{
  std::lock_guard<std::mutex> lock(pick_search_budget_mutex_);
  pick_search_started_at_ = std::chrono::steady_clock::now();
  pick_candidate_search_started_at_ = pick_search_started_at_;
  pick_search_goal_uuid_.clear();
  pick_candidate_grasp_pose_candidates_remaining_ = pick_grasp_pose_candidate_limit_;
  std::lock_guard<std::mutex> state_lock(mutex_);
  pending_pick_descent_trajectory_.points.clear();
  reusable_pick_descent_trajectory_.points.clear();
  pending_pick_descent_for_current_execution_ = false;
}

void IsaacRosMotionTransport::begin_pick_search(const std::string & execution_id)
{
  {
    std::lock_guard<std::mutex> lock(pick_search_budget_mutex_);
    if (!execution_id.empty() && pick_search_started_at_ !=
      std::chrono::steady_clock::time_point{} && pick_search_goal_uuid_ == execution_id)
    {
      return;
    }
  }
  begin_pick_search();
  std::lock_guard<std::mutex> lock(pick_search_budget_mutex_);
  pick_search_goal_uuid_ = execution_id;
}

void IsaacRosMotionTransport::begin_pick_candidate_search()
{
  std::lock_guard<std::mutex> lock(pick_search_budget_mutex_);
  pick_candidate_search_started_at_ = std::chrono::steady_clock::now();
  pick_candidate_grasp_pose_candidates_remaining_ = pick_grasp_pose_candidate_limit_;
  {
    std::lock_guard<std::mutex> state_lock(mutex_);
    pick_grasp_pose_candidate_floor_ = 0U;
    pick_active_grasp_pose_candidate_count_ = 0U;
  }
}

void IsaacRosMotionTransport::record_material_candidate_search(
  const std::vector<MaterialCandidateSearchAttempt> & attempts,
  const std::string & outcome)
{
  if (!diagnostics_capture_enabled_) {return;}
  std::lock_guard<std::mutex> lock(planning_diagnostic_mutex_);
  auto evidence = nlohmann::json::parse(pending_motion_planning_evidence_, nullptr, false);
  if (!evidence.is_object()) {evidence = nlohmann::json::object();}
  auto & attempt_records = evidence["material_candidate_attempts"];
  attempt_records = nlohmann::json::array();
  std::vector<std::string> attempted;
  attempted.reserve(attempts.size());
  for (std::size_t index = 0; index < attempts.size(); ++index) {
    const auto & attempt = attempts[index];
    if (attempt.search_began) {attempted.push_back(attempt.candidate_id);}
    attempt_records.push_back({
      {"attempt_order", index + 1U},
      {"candidate_id", attempt.candidate_id},
      {"result_code", attempt.result_code},
      {"result", motion_result_code_name(attempt.result_code)},
      {"failure_stage", attempt.failure_stage},
      {"pick_grasp_insertion_started", attempt.grasp_insertion_started},
      {"precise_search_began", attempt.search_began}});
  }
  evidence["attempted_material_candidate_ids"] = attempted;
  evidence["outcome_material_candidate_id"] = outcome;
  pending_motion_planning_evidence_ = evidence.dump();
}

std::string IsaacRosMotionTransport::last_pick_candidate_evaluation() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pick_candidate_evaluation_;
}

bool IsaacRosMotionTransport::advance_to_next_planning_candidate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!next_candidate_available_) {return false;}
  next_candidate_available_ = false;
  if (selected_planning_candidate_task_ ==
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK)
  {
    pick_grasp_pose_candidate_floor_ = selected_planning_candidate_index_ + 1U;
  } else if (selected_planning_candidate_task_ ==
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE)
  {
    place_candidate_pose_floor_ = selected_planning_candidate_index_ + 1U;
  } else {
    return false;
  }
  return true;
}

bool IsaacRosMotionTransport::retry_place_retreat()
{
  std::lock_guard<std::mutex> lock(mutex_);
  // The PLACE final-path fallback budget bounds retreat replans within the
  // current Action UUID; the Approach request limit is a separate search.
  if (place_retreat_planning_requests_ >= place_final_global_fallback_max_requests_) {
    return false;
  }
  ++place_retreat_planning_requests_;
  return true;
}

void IsaacRosMotionTransport::set_pending_motion_planning_evidence(
  const std::string & goal_uuid, uint8_t task_type, std::string evidence)
{
  if (!diagnostics_capture_enabled_ || evidence.empty()) {return;}
  std::lock_guard<std::mutex> lock(planning_diagnostic_mutex_);
  pending_motion_planning_goal_uuid_ = goal_uuid;
  pending_motion_planning_task_type_ = task_type;
  pending_motion_planning_evidence_ = std::move(evidence);
}

void IsaacRosMotionTransport::finalize_motion_planning_diagnostic(uint8_t result_code)
{
  if (!planning_diagnostic_capture_) {return;}
  std::string goal_uuid;
  std::string vision_acquisition_id;
  std::string evidence;
  std::vector<std::string> task_accuracy_history;
  uint8_t task_type = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_UNSPECIFIED;
  {
    std::lock_guard<std::mutex> lock(planning_diagnostic_mutex_);
    goal_uuid = std::move(pending_motion_planning_goal_uuid_);
    vision_acquisition_id = std::move(pending_motion_planning_vision_acquisition_id_);
    evidence = std::move(pending_motion_planning_evidence_);
    task_type = pending_motion_planning_task_type_;
    task_accuracy_history = std::move(task_accuracy_history_);
    if (goal_uuid.empty()) {goal_uuid = task_accuracy_goal_uuid_;}
    if (task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_UNSPECIFIED) {
      task_type = task_accuracy_task_type_;
    }
    pending_motion_planning_vision_acquisition_id_.clear();
    task_accuracy_goal_uuid_.clear();
    task_accuracy_task_type_ =
      arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_UNSPECIFIED;
    pending_motion_planning_task_type_ =
      arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_UNSPECIFIED;
  }
  const bool is_pick = task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK;
  if (evidence.empty() && task_accuracy_history.empty() && !is_pick && result_code !=
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR) {return;}
  auto terminal_evidence = nlohmann::json::parse(evidence, nullptr, false);
  if (!terminal_evidence.is_object()) {terminal_evidence = nlohmann::json::object();}
  if (!planning_candidate_history_.empty()) {
    auto history = nlohmann::json::array();
    for (const auto & snapshot : planning_candidate_history_) {
      auto item = nlohmann::json::parse(snapshot, nullptr, false);
      if (!item.is_discarded()) {history.push_back(std::move(item));}
    }
    terminal_evidence["candidate_attempt_history"] = std::move(history);
  }
  if (task_type == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE &&
    !last_place_failure_stage_.empty())
  {
    terminal_evidence["final_result"] = motion_result_code_name(result_code);
    terminal_evidence["final_failure_stage"] = last_place_failure_stage_;
  }
  if (!task_accuracy_history.empty()) {
    auto accuracy = nlohmann::json::array();
    for (const auto & entry : task_accuracy_history) {
      auto item = nlohmann::json::parse(entry, nullptr, false);
      if (!item.is_discarded()) {accuracy.push_back(std::move(item));}
    }
    terminal_evidence["task_accuracy"] = std::move(accuracy);
  }
  evidence = terminal_evidence.dump();
  if (is_pick) {
    bool insertion_started = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      insertion_started = pick_grasp_insertion_started_;
    }
    std::size_t accepted_detail_count = 0U;
    try {
      accepted_detail_count = planning_diagnostic_capture_->enqueue_pick_details(
        goal_uuid, vision_acquisition_id, evidence);
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "Motion PICK detail capture unavailable");
    }
    try {
      if (!planning_diagnostic_capture_->enqueue_pick_summary(
          goal_uuid, vision_acquisition_id, result_code, insertion_started, evidence,
          accepted_detail_count))
      {
        RCLCPP_WARN(node_->get_logger(), "Motion PICK terminal summary unavailable");
      }
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "Motion PICK terminal summary unavailable");
    }
    nlohmann::json compact = {
      {"final_failure_stage", terminal_evidence.value("final_failure_stage", "unavailable")},
      {"final_failure_reason", terminal_evidence.value("final_failure_reason", "unavailable")},
      {"pick_grasp_insertion_started", insertion_started},
      {"detail_summary_reference", "motion_planning_summaries/" + goal_uuid + ".json"}};
    if (terminal_evidence.contains("backend_failure")) {
      compact["backend_failure"] = terminal_evidence["backend_failure"];
    }
    auto candidate_evidence = terminal_evidence;
    if (terminal_evidence.contains("candidate_attempt_history") &&
      terminal_evidence["candidate_attempt_history"].is_array() &&
      !terminal_evidence["candidate_attempt_history"].empty())
    {
      candidate_evidence = terminal_evidence["candidate_attempt_history"].back();
    }
    compact["generated_candidate_count"] = candidate_evidence.value(
      "generated_candidate_count", candidate_evidence.value("candidate_count", 0U));
    compact["evaluated_candidate_count"] = candidate_evidence.value(
      "evaluated_candidate_count", 0U);
    if (candidate_evidence.contains("selected_candidate")) {
      compact["selected_candidate"] = candidate_evidence["selected_candidate"];
    }
    if (terminal_evidence.contains("final_segment_planning")) {
      compact["final_segment_planning"] = terminal_evidence["final_segment_planning"];
    }
    if (terminal_evidence.contains("task_accuracy")) {
      compact["task_accuracy"] = terminal_evidence["task_accuracy"];
    }
    evidence = compact.dump();
  }
  if (!planning_diagnostic_capture_->enqueue(
      goal_uuid, motion_task_type_name(task_type), result_code, evidence,
      vision_acquisition_id))
  {
    RCLCPP_WARN(node_->get_logger(),
                "Motion planning diagnostic was not persisted");
  }
}

void IsaacRosMotionTransport::finalize_motion_planning_diagnostic(
  uint8_t result_code, double elapsed_s)
{
  finalize_motion_planning_diagnostic(result_code, elapsed_s, {}, {});
}

void IsaacRosMotionTransport::finalize_motion_planning_diagnostic(
  uint8_t result_code, double elapsed_s, const std::string & failure_stage,
  const std::string & failure_detail)
{
  if (!planning_diagnostic_capture_) {return;}
  {
    std::lock_guard<std::mutex> lock(planning_diagnostic_mutex_);
    auto evidence = nlohmann::json::parse(pending_motion_planning_evidence_, nullptr, false);
    if (!evidence.is_object() && result_code ==
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR)
    {
      evidence = nlohmann::json::object();
    }
    if (evidence.is_object()) {
      evidence["task_executor_elapsed_s"] = elapsed_s;
      if (pending_motion_planning_task_type_ ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE &&
        !last_place_failure_stage_.empty())
      {
        evidence["final_failure_stage"] = last_place_failure_stage_;
      }
      if (pending_motion_planning_task_type_ ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE &&
        !failure_stage.empty() && !evidence.contains("final_failure_stage") && result_code !=
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS)
      {
        evidence["final_failure_stage"] = failure_stage;
      }
      if (result_code ==
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR)
      {
        const auto [evidence_reason, evidence_reason_source] = backend_failure_reason(evidence);
        std::string recorded_stage = failure_stage;
        std::string stage_source = failure_stage.empty() ? "UNAVAILABLE" : "task_executor";
        for (const auto * key : {"final_failure_stage", "planning_stage", "stage"}) {
          if (evidence.contains(key) && evidence[key].is_string()) {
            const auto value = evidence[key].get<std::string>();
            if (!value.empty() && value != "unavailable" && value != "UNAVAILABLE") {
              recorded_stage = value;
              stage_source = std::string("planning_evidence.") + key;
              break;
            }
          }
        }
        const auto reason = !failure_detail.empty() ? failure_detail : evidence_reason;
        const auto reason_source = !failure_detail.empty() ? "task_executor_exception" :
          evidence_reason_source;
        nlohmann::json backend_failure = {
          {"stage", recorded_stage.empty() ? "UNAVAILABLE" : recorded_stage},
          {"stage_source", stage_source},
          {"stage_unavailable_reason", recorded_stage.empty() ?
            "task_executor_and_backend_evidence_have_no_failure_stage" : "NONE"},
          {"reason", reason.empty() ? "UNAVAILABLE" : reason},
          {"reason_source", reason.empty() ? "UNAVAILABLE" : reason_source},
          {"reason_unavailable_reason", reason.empty() ?
            "backend_evidence_has_no_failure_reason" : "NONE"},
          {"backend_result_code", motion_result_code_name(result_code)}};
        for (const auto * key : {"moveit_error_code", "planner_result_code", "result_code"}) {
          if (evidence.contains(key)) {backend_failure["backend_result_evidence"][key] = evidence[key];}
        }
        for (const auto * key : {"execution_active", "backend_inactivity_confirmed",
          "backend_execution_state"})
        {
          if (evidence.contains(key)) {backend_failure["backend_state"][key] = evidence[key];}
        }
        const auto & goal_uuid = pending_motion_planning_goal_uuid_;
        if (!goal_uuid.empty()) {
          backend_failure["detail_reference"] = "motion_planning_records/" +
            diagnostic_goal_id(goal_uuid) + ".json#/planning_evidence";
        } else {
          backend_failure["detail_reference_unavailable_reason"] =
            "motion_goal_uuid_unavailable";
        }
        evidence["backend_failure"] = std::move(backend_failure);
      }
      pending_motion_planning_evidence_ = evidence.dump();
    }
  }
  finalize_motion_planning_diagnostic(result_code);
}

uint8_t IsaacRosMotionTransport::normalized_failure_result() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return normalized_failure_result_;
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
    !finite_positive(tuning.cartesian_tcp_orientation_tolerance_rad) ||
    !finite_positive(tuning.trajectory_goal_tolerance_rad) ||
    !std::isfinite(tuning.planning_velocity_scaling_factor) ||
    tuning.planning_velocity_scaling_factor <= 0.0 ||
    tuning.planning_velocity_scaling_factor > 1.0 ||
    !std::isfinite(tuning.planning_acceleration_scaling_factor) ||
    tuning.planning_acceleration_scaling_factor <= 0.0 ||
    tuning.planning_acceleration_scaling_factor > 1.0 ||
    !std::isfinite(tuning.disturbance_push_distance_m) ||
    tuning.disturbance_push_distance_m < 0.001 || tuning.disturbance_push_distance_m > 0.1 ||
    !std::isfinite(tuning.disturbance_approach_clearance_m) ||
    tuning.disturbance_approach_clearance_m < 0.001 ||
    tuning.disturbance_approach_clearance_m > 0.1 ||
    !std::isfinite(tuning.disturbance_speed_fraction) ||
    tuning.disturbance_speed_fraction <= 0.0 || tuning.disturbance_speed_fraction > 1.0)
  {
    return false;
  }
  planning_time_s_ = tuning.planning_time_s;
  place_candidate_planning_time_s_ = tuning.place_candidate_planning_time_s;
  place_total_planning_time_s_ = tuning.place_total_planning_time_s;
  planning_velocity_scaling_factor_ = tuning.planning_velocity_scaling_factor;
  planning_acceleration_scaling_factor_ = tuning.planning_acceleration_scaling_factor;
  disturbance_push_distance_m_ = tuning.disturbance_push_distance_m;
  disturbance_approach_clearance_m_ = tuning.disturbance_approach_clearance_m;
  disturbance_speed_fraction_ = tuning.disturbance_speed_fraction;
  motion_progress_watchdog_ = MotionProgressWatchdog(tuning.motion_progress_stall_timeout_s);
  cartesian_tcp_orientation_tolerance_rad_ =
    tuning.cartesian_tcp_orientation_tolerance_rad;
  trajectory_goal_tolerance_rad_ = tuning.trajectory_goal_tolerance_rad;
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
  return submit(goal, {});
}

bool IsaacRosMotionTransport::submit(
  const arm_cell_interfaces::action::ExecuteTask::Goal & goal,
  const std::string & motion_goal_uuid)
{
  using MotionTaskType = arm_cell_interfaces::msg::MotionTaskType;
  const bool go_home = goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME;
  if (go_home && diagnostics_capture_enabled_) {
    std::lock_guard<std::mutex> lock(planning_diagnostic_mutex_);
    pending_motion_planning_goal_uuid_.clear();
    pending_motion_planning_evidence_.clear();
    pending_motion_planning_task_type_ = MotionTaskType::TASK_TYPE_GO_HOME;
  }
  if (!available()) {
    return false;
  }
  const auto envelope = applied_motion_envelope();
  const auto record_home_locked = [&](const std::string & reason, bool accepted) {
      if (!diagnostics_capture_enabled_) {return;}
      const auto now_steady = std::chrono::steady_clock::now();
      const bool have_start = latest_joint_state_ != nullptr;
      const auto start_age_ns = have_start && now_steady >= latest_state_receipt_ ?
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        now_steady - latest_state_receipt_).count() : -1;
      std::vector<double> start_positions;
      const bool start_positions_available = have_start &&
        extract_arm_positions(*latest_joint_state_, start_positions);
      std::ostringstream evidence;
      evidence << std::setprecision(17)
               << "{\"decision\":\"direct_home_joint_target\","
               << "\"planner\":\"not_used_by_direct_joint_target_path\","
               << "\"submission_accepted\":" << (accepted ? "true" : "false")
               << ",\"submission_reason\":\"" << json_escape(reason) << '\"'
               << ",\"safety_motion_envelope\":{\"valid\":"
               << (envelope.valid ? "true" : "false")
               << ",\"max_velocity_scale\":" << envelope.max_velocity_scale
               << ",\"max_acceleration_scale\":" << envelope.max_acceleration_scale << '}'
               << ",\"joint_names\":[";
      for (std::size_t index = 0; index < arm_joint_names_.size(); ++index) {
        if (index != 0U) {evidence << ',';}
        evidence << '\"' << json_escape(arm_joint_names_[index]) << '\"';
      }
      evidence << "],\"home_joint_target\":";
      append_json_numbers(evidence, home_joint_positions_);
      evidence << ",\"start_state\":{\"available\":"
               << (start_positions_available ? "true" : "false")
               << ",\"source\":\"latest_joint_state_at_submission\""
               << ",\"source_stamp_ns\":" <<
        (have_start ? rclcpp::Time(latest_joint_state_->header.stamp).nanoseconds() : -1)
               << ",\"receipt_steady_age_ns\":" << start_age_ns
               << ",\"receipt_freshness_threshold_ns\":"
               << std::chrono::duration_cast<std::chrono::nanoseconds>(kStateFreshness).count()
               << ",\"joint_positions\":";
      append_json_numbers(evidence, start_positions);
      evidence << "}}";
      set_pending_motion_planning_evidence(
        motion_goal_uuid, MotionTaskType::TASK_TYPE_GO_HOME, evidence.str());
    };
  // These direct position-target paths do not produce a time-parameterized
  // trajectory, so they cannot attest reduced MoveIt scaling limits.
  if (envelope.max_velocity_scale < 1.0F || envelope.max_acceleration_scale < 1.0F) {
    if (go_home && diagnostics_capture_enabled_) {
      std::lock_guard<std::mutex> lock(mutex_);
      record_home_locked("safety_envelope_rejects_direct_target_scaling", false);
    }
    return false;
  }
  if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME ||
    goal.task_type.value == MotionTaskType::TASK_TYPE_RETRACT)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_motion_task_type_ = goal.task_type.value;
  }
  if (goal.task_type.value == MotionTaskType::TASK_TYPE_GO_HOME) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string rejection_reason;
    const bool published = publish_joint_target_locked(
      home_joint_positions_, nullptr,
      diagnostics_capture_enabled_ ? &rejection_reason : nullptr);
    record_home_locked(published ? "joint_command_published" : rejection_reason, published);
    if (!published) {
      return false;
    }
    if (profile_trace_) {
      begin_profile_trace(
        "GO_HOME", "command=single_position_target;completion=2_feedbacks;"
        "isaac_position_controller_profile=runtime_owned_unreported", motion_goal_uuid);
      go_home_profile_trace_active_ = true;
      if (has_simulation_time_) {
        profile_trace_->add_row(
          {
            latest_simulation_time_, "GO_HOME_COMMAND", home_joint_positions_, {}, {}, {}, {}});
      }
    }
    return true;
  }
  if (goal.task_type.value == MotionTaskType::TASK_TYPE_RETRACT) {
    return publish_joint_target(retract_joint_positions_);
  }
  return false;
}

void IsaacRosMotionTransport::begin_profile_trace(
  const std::string & kind, const std::string & config,
  const std::string & motion_goal_uuid, std::uint64_t execution_id)
{
  std::ostringstream run_id;
  const auto wall_tag = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  run_id << kind << '-' << wall_tag << '-' << ++next_profile_trace_id_;
  profile_trace_->begin(
    run_id.str(), environment_or("ISAAC_SIM_VERSION", "Isaac-runtime-version-unreported"),
    environment_or("ARM_CELL_PROFILE_ID", node_->get_name()), "ROS /clock simulation time",
    build_commit_sha(), motion_goal_uuid, execution_id);
  active_profile_configuration_ = config;
  RCLCPP_DEBUG(node_->get_logger(),
               "[motion profile] event=START kind=%s time_basis=sim_clock",
               kind.c_str());
}

void IsaacRosMotionTransport::finish_profile_trace(const std::string & outcome)
{
  try {
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
      ";trajectory_goal_tolerance_rad=" + std::to_string(trajectory_goal_tolerance_rad_);
    const auto logger = node_->get_logger();
    std::thread(
      [
        snapshot, joint_names, configuration, outcome, logger]() {
        const auto result = snapshot->finish(outcome, joint_names, configuration);
        if (!result.success) {
          RCLCPP_WARN(
            logger, "Motion profile trace could not be saved; motion result is unaffected cause=%s",
            result.error.c_str());
          return;
        }
        RCLCPP_DEBUG(
          logger, "Motion profile trace completed outcome=%s", outcome.c_str());
      }).detach();
  } catch (...) {
    RCLCPP_WARN(
      node_->get_logger(), "Motion profile capture unavailable; motion result is unaffected");
  }
}

TrajectorySampleResult IsaacRosMotionTransport::start_global_profiled_trajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory, std::uint64_t execution_id,
  double start_simulation_time, const std::string & config,
  const std::string & motion_goal_uuid)
{
  bool capture_profile = false;
  std::lock_guard<std::mutex> lock(profile_capture_mutex_);
  if (global_profile_trace_active_) {
    return {false, "a GLOBAL profile capture is already active"};
  }
  capture_profile = profile_trace_ && !global_profile_capture_complete_;
  if (capture_profile) {
    begin_profile_trace("GLOBAL_MOVEIT", config, motion_goal_uuid, execution_id);
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
  const auto emit = [&](const char * result, GraspContactPolicyReason reason,
      const GraspContactPolicyResolution & resolution, const char * read_status,
      const char * apply_status) {
      RCLCPP_DEBUG(
        node_->get_logger(), "Grasp contact policy result=%s reason=%s target_identity=%s resolved_collision_object_id=%s planning_scene_read=%s planning_scene_apply=%s",
        result, reason_name(reason), resolution.target_identity.c_str(),
        resolution.resolved_collision_object_id.c_str(), read_status, apply_status);
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

void IsaacRosMotionTransport::reset_pick_grasp_insertion_started()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pick_grasp_insertion_started_ = false;
}

MotionDisturbanceParameters IsaacRosMotionTransport::disturbance_parameters() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {disturbance_push_distance_m_, disturbance_approach_clearance_m_,
    disturbance_speed_fraction_};
}

bool IsaacRosMotionTransport::pick_grasp_insertion_started() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return pick_grasp_insertion_started_;
}

bool IsaacRosMotionTransport::submit_normalized_once(
  const NormalizedMotionRequest & input_request, const std::vector<double> * waypoint,
  double budget_s, bool qualify_place_target, NormalizedTrajectoryPlan * plan_output)
{

  auto request = input_request;
  if (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
    request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING)
  {
    std::lock_guard<std::mutex> budget_lock(pick_search_budget_mutex_);
    if (pick_search_started_at_ == std::chrono::steady_clock::time_point{} ||
      (!request.motion_goal_uuid.empty() &&
      request.motion_goal_uuid != pick_search_goal_uuid_))
    {
      pick_search_started_at_ = std::chrono::steady_clock::now();
      pick_candidate_search_started_at_ = pick_search_started_at_;
      pick_search_goal_uuid_ = request.motion_goal_uuid;
    }
    {
      std::lock_guard<std::mutex> state_lock(mutex_);
      reusable_pick_descent_trajectory_.points.clear();
      pending_pick_descent_trajectory_.points.clear();
      pending_pick_descent_for_current_execution_ = false;
    }
    if (pick_candidate_grasp_pose_candidates_remaining_ == 0U) {
      std::lock_guard<std::mutex> result_lock(mutex_);
      normalized_failure_result_ =
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED;
      return false;
    }
  }
  if (diagnostics_capture_enabled_ && request.task_type.value ==
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK)
  {
    std::lock_guard<std::mutex> lock(planning_diagnostic_mutex_);
    pending_motion_planning_goal_uuid_ = request.motion_goal_uuid;
    pending_motion_planning_vision_acquisition_id_ = request.vision_acquisition_id;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_executed_path_selection_ = {};
    active_motion_task_type_ = request.task_type.value;
  }
  const bool place_approach_request =
    request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE &&
    request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING && !waypoint;
  if (place_approach_request && diagnostics_capture_enabled_) {
    std::lock_guard<std::mutex> lock(planning_diagnostic_mutex_);
    pending_motion_planning_goal_uuid_ = request.motion_goal_uuid;
    pending_motion_planning_evidence_ =
      "{\"planning_stage\":\"place_approach\",\"evidence_status\":\"planning_record_unavailable\","
      "\"reason\":\"no_later_planning_snapshot_was_captured\"}";
    pending_motion_planning_task_type_ =
      arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE;
  }
  const auto record_place_preplanning_failure = [&](const char * stage, const char * reason) {
      if (!place_approach_request || !diagnostics_capture_enabled_) {return;}
      std::ostringstream evidence;
      evidence << "{\"planning_stage\":\"" << stage << "\",\"failure_reason\":\""
               << json_escape(reason) << "\",\"requested_object_pose\":";
      append_json_pose(evidence, request.object_pose);
      evidence << ",\"requested_tcp_target\":";
      append_json_pose(evidence, request.target_pose);
      evidence << ",\"requested_approach_target\":";
      append_json_pose(evidence, request.approach_target);
      const auto envelope = applied_motion_envelope();
      evidence << ",\"safety_motion_envelope\":{\"valid\":"
               << (envelope.valid ? "true" : "false")
               << ",\"max_velocity_scale\":" << envelope.max_velocity_scale
               << ",\"max_acceleration_scale\":" << envelope.max_acceleration_scale << "}}";
      set_pending_motion_planning_evidence(
        request.motion_goal_uuid, request.task_type.value, evidence.str());
    };
  {
    std::lock_guard<std::mutex> result_lock(mutex_);
    normalized_failure_result_ =
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  const auto discard_if_interrupted = [&]() {
      const auto result = request.interruption_result ? request.interruption_result() :
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
      if (result == arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS) {
        return false;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      normalized_failure_result_ = result;
      if (request.task_type.value ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK)
      {
        pick_collision_feedback_.clear();
        pick_collision_feedback_goal_uuid_.clear();
      }
      if (request.task_type.value ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
        request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING)
      {
        // The diagnostic snapshot is replaced at each planning milestone.
        // Preserve the last snapshot and mark the discarded result explicitly.
        if (pick_candidate_evaluation_.empty()) {
          pick_candidate_evaluation_ =
            "{\"motion_goal_uuid\":\"" + json_escape(request.motion_goal_uuid) +
            "\",\"interrupted_result_discarded\":true}";
        } else if (pick_candidate_evaluation_.back() == '}' &&
          pick_candidate_evaluation_.find("\"interrupted_result_discarded\"") ==
          std::string::npos)
        {
          pick_candidate_evaluation_.pop_back();
          pick_candidate_evaluation_ += ",\"interrupted_result_discarded\":true}";
        }
        if (diagnostics_capture_enabled_) {
          set_pending_motion_planning_evidence(
            request.motion_goal_uuid, request.task_type.value, pick_candidate_evaluation_);
        }
      }
      return true;
    };
  const auto record_pick_collision_feedback = [&](const auto & contacts) {
      if (request.task_type.value !=
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK || contacts.empty())
      {
        return;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (planning_candidate_goal_uuid_ != request.motion_goal_uuid) {return;}
      pick_collision_feedback_ = contacts;
      pick_collision_feedback_goal_uuid_ = request.motion_goal_uuid;
    };
  if (discard_if_interrupted()) {return false;}
  if (!waypoint && !qualify_place_target &&
    request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING) {
    bool new_goal = false;
    bool preserve_previous = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      new_goal = planning_candidate_goal_uuid_ != request.motion_goal_uuid;
      if (new_goal) {
        planning_candidate_goal_uuid_ = request.motion_goal_uuid;
        pick_grasp_pose_candidate_floor_ = 0U;
        pick_active_grasp_pose_candidate_count_ = 0U;
        pick_collision_feedback_.clear();
        pick_collision_feedback_goal_uuid_.clear();
        selected_pick_ik_seed_attempt_ = 0U;
        place_candidate_pose_floor_ = 0U;
      } else {
        preserve_previous = pick_grasp_pose_candidate_floor_ > 0U ||
          place_candidate_pose_floor_ > 0U;
      }
      next_candidate_available_ = false;
    }
    {
      std::lock_guard<std::mutex> lock(planning_diagnostic_mutex_);
      if (new_goal) {
        planning_candidate_history_.clear();
      } else if (preserve_previous &&
        pending_motion_planning_goal_uuid_ == request.motion_goal_uuid &&
        !pending_motion_planning_evidence_.empty())
      {
        planning_candidate_history_.push_back(pending_motion_planning_evidence_);
      }
    }
    if (request.task_type.value ==
      arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK)
    {
      std::lock_guard<std::mutex> result_lock(mutex_);
      pick_candidate_evaluation_.clear();
      selected_pick_pose_valid_ = false;
    }
  }
  if (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
    request.phase != arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING)
  {
    std::lock_guard<std::mutex> orientation_lock(mutex_);
    if (selected_pick_pose_valid_) {
      request.object_pose = selected_pick_object_pose_;
      request.target_pose = selected_pick_target_pose_;
      request.approach_target = selected_pick_approach_pose_;
      request.retract_target = selected_pick_retract_pose_;
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
      if (!request.place_waypoint_approach && !accepted_object_to_tcp_valid_) {
        record_place_preplanning_failure(
          "preconditions_not_met", "accepted_pick_grasp_relation_unavailable");
        RCLCPP_WARN(node_->get_logger(),
                    "Motion internal cause: no grasp relation was established "
                    "by an accepted PICK target"
                    "");
        return false;
      }
      if (!request.place_waypoint_approach) {
        request.object_to_grasp_tcp = accepted_object_to_tcp_;
      }
    } else if (selected_place_pose_valid_) {
      request.object_pose = selected_place_object_pose_;
      request.target_pose = selected_place_target_pose_;
      request.approach_target = selected_place_approach_pose_;
      request.retract_target = selected_place_retract_pose_;
      if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING) {
        request.tcp_target = request.target_pose;
      } else if (request.phase ==
        arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING)
      {
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
  RCLCPP_DEBUG(
      node_->get_logger(),
      "%s motion is preparing its %s phase from the %s observation reference",
      motion_task_type_name(request.task_type.value),
      motion_task_phase_name(request.phase),
      request.observation_reference.c_str());
  if (move_group_state_->required && !move_group_state_->ready) {
    record_place_preplanning_failure("preconditions_not_met", "move_group_not_ready");
    RCLCPP_WARN(
        node_->get_logger(),
        "motion diagnostic normalized submit rejected: MoveGroup not ready"
        "");
    return false;
  }
  bool cached_state_received = false;
  bool cached_arm_positions_available = false;
  std::int64_t cached_state_source_stamp_ns = -1;
  std::int64_t cached_state_receipt_age_ns = -1;
  std::vector<double> cached_start_positions;
  {
    std::lock_guard<std::mutex> state_lock(mutex_);
    const auto decision_steady = std::chrono::steady_clock::now();
    cached_state_received = static_cast<bool>(latest_joint_state_);
    if (cached_state_received) {
      cached_state_source_stamp_ns =
        rclcpp::Time(latest_joint_state_->header.stamp).nanoseconds();
      cached_state_receipt_age_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        decision_steady - latest_state_receipt_).count();
      cached_arm_positions_available =
        extract_arm_positions(*latest_joint_state_, cached_start_positions);
    }
  }
  const auto state_freshness_threshold_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(kStateFreshness).count();
  const bool cached_state_fresh = cached_state_received &&
    cached_state_receipt_age_ns <= state_freshness_threshold_ns &&
    cached_arm_positions_available;
  if (!cached_state_fresh) {
    const char * freshness_failure_reason = !cached_state_received ?
      "joint_state_not_received" :
      cached_state_receipt_age_ns > state_freshness_threshold_ns ?
      "joint_state_receipt_stale" : "arm_joint_positions_unavailable";
    if (place_approach_request && diagnostics_capture_enabled_) {
      std::ostringstream evidence;
      evidence << std::setprecision(17)
               << "{\"planning_stage\":\"preconditions_not_met\","
               << "\"failure_reason\":\"" << freshness_failure_reason << '"'
               << ",\"requested_object_pose\":";
      append_json_pose(evidence, request.object_pose);
      evidence << ",\"requested_tcp_target\":";
      append_json_pose(evidence, request.target_pose);
      evidence << ",\"requested_approach_target\":";
      append_json_pose(evidence, request.approach_target);
      evidence << ",\"cached_joint_state\":{\"received\":"
               << (cached_state_received ? "true" : "false")
               << ",\"source_stamp_ns\":" << cached_state_source_stamp_ns
               << ",\"receipt_steady_age_ns\":" << cached_state_receipt_age_ns
               << ",\"receipt_freshness_threshold_ns\":"
               << state_freshness_threshold_ns
               << ",\"arm_joint_positions_available\":"
               << (cached_arm_positions_available ? "true" : "false")
               << ",\"joint_positions\":";
      append_json_numbers(evidence, cached_start_positions);
      const auto envelope = applied_motion_envelope();
      evidence << "},\"safety_motion_envelope\":{\"valid\":"
               << (envelope.valid ? "true" : "false")
               << ",\"max_velocity_scale\":" << envelope.max_velocity_scale
               << ",\"max_acceleration_scale\":"
               << envelope.max_acceleration_scale << "}}";
      set_pending_motion_planning_evidence(
        request.motion_goal_uuid, request.task_type.value, evidence.str());
    }
    RCLCPP_WARN(node_->get_logger(),
                "motion diagnostic normalized submit rejected: current joint "
                "state is missing or stale"
                "");
    return false;
  }
  if (!move_group_state_->move_group) {
    record_place_preplanning_failure("preconditions_not_met", "move_group_interface_unavailable");
    RCLCPP_WARN(node_->get_logger(), "motion diagnostic normalized submit "
                                     "rejected: MoveGroupInterface is null"
                                     "");
    return false;
  }
  const auto envelope = applied_motion_envelope();
  auto planning_scales = apply_motion_envelope_to_planning_scales(
    planning_velocity_scaling_factor_, planning_acceleration_scaling_factor_, envelope);
  if (request.is_disturbance) {
    planning_scales.max_velocity *= request.motion_speed_fraction;
    planning_scales.max_acceleration *= request.motion_speed_fraction;
  }
  move_group_state_->move_group->setMaxVelocityScalingFactor(planning_scales.max_velocity);
  move_group_state_->move_group->setMaxAccelerationScalingFactor(
    planning_scales.max_acceleration);
  {
    std::lock_guard<std::mutex> state_lock(mutex_);
  }

  // This current state is the actual start state used by the plan below.
  const auto current_state = move_group_state_->move_group->getCurrentState(1.0);
  const auto * joint_group = move_group_state_->move_group->getRobotModel()->getJointModelGroup(
    "arm");
  if (!current_state || !joint_group) {
    record_place_preplanning_failure(
      "preconditions_not_met", "planning_start_state_or_arm_group_unavailable");
    RCLCPP_WARN(node_->get_logger(),
                "Motion planning cannot continue because the current robot "
                "state or arm planning group is unavailable");
    return false;
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
      RCLCPP_WARN(node_->get_logger(),
                  "[Motion validation] grasp contact policy rejected closed"
                  "");
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

  const bool cartesian_primitive = !(request.segment == MotionSegmentKind::PLACE_CLEARANCE &&
    request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE) &&
    uses_cartesian_manipulation(request.task_type.value, request.phase);
  const char * primitive_type = cartesian_primitive ? "CARTESIAN" : "GLOBAL";
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool planning_succeeded = false;
  bool planning_precomputed = false;
  std::string selected_final_strategy = cartesian_primitive ? "CARTESIAN" : "GLOBAL";
  const auto robot_model = move_group_state_->move_group->getRobotModel();
  const bool pick_retract_collision_exception =
    is_pick_retract_collision_exception_scope(
    request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK,
    request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING);
  const bool vision_derived_pick =
    request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
    !request.material_candidates.empty() && !request.vision_candidate_id.empty();
  const bool vision_collision_scope_required = vision_derived_pick ||
    (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
    request.has_non_target_occupancy && !request.non_target_occupancy.empty());
  const bool planning_scene_context_required =
    request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ||
    request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE;
  if (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
    !valid_candidate_occupancy(
      request.has_non_target_occupancy, request.non_target_occupancy, request.observation_pose))
  {
    std::lock_guard<std::mutex> result_lock(mutex_);
    normalized_failure_result_ =
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    return false;
  }
  planning_scene::PlanningScenePtr retract_validation_scene;
  planning_scene::PlanningScenePtr request_validation_scene;
  planning_scene::PlanningScenePtr grasp_validation_scene;
  moveit_msgs::msg::PlanningScene manipulation_scene_diff;
  manipulation_scene_diff.is_diff = true;
  manipulation_scene_diff.robot_state.is_diff = true;
  moveit_msgs::msg::PlanningScene planning_request_scene_diff = manipulation_scene_diff;
  std::vector<std::string> request_candidate_ids;
  std::vector<std::string> previous_candidate_ids;
  moveit_msgs::msg::PlanningScene retract_scene_diff;
  std::string retract_exception_context_result = "NOT_APPLICABLE";
  std::string scene_snapshot_source = "NOT_APPLICABLE";
  moveit_msgs::msg::PlanningScene scene_snapshot;
  if (planning_scene_context_required) {
    bool has_scene_snapshot = false;
    if (planning_scene_client_ &&
      planning_scene_client_->wait_for_service(std::chrono::milliseconds(500)))
    {
      auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
      scene_request->components.components = pick_retract_planning_scene_components();
      auto scene_future = planning_scene_client_->async_send_request(scene_request);
      if (scene_future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready) {
        const auto response = scene_future.get();
        if (response) {
          scene_snapshot = response->scene;
          has_scene_snapshot = true;
          scene_snapshot_source = "GetPlanningScene";
        } else {
          retract_exception_context_result = "SCENE_RESPONSE_EMPTY";
        }
      } else {
        retract_exception_context_result = "SCENE_READ_TIMEOUT";
      }
    } else {
      retract_exception_context_result = "SCENE_SERVICE_UNAVAILABLE";
    }
    if (has_scene_snapshot) {
      request_validation_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
      if (!request_validation_scene->setPlanningSceneMsg(scene_snapshot)) {
        request_validation_scene.reset();
        retract_exception_context_result = "SCENE_SNAPSHOT_INVALID";
      } else {
        bool valid = true;
        previous_candidate_ids = candidate_occupancy_ids(scene_snapshot);
        if (previous_candidate_ids.size() != std::set<std::string>(
            previous_candidate_ids.begin(), previous_candidate_ids.end()).size())
        {
          valid = false;
          retract_exception_context_result = "VISION_OCCUPANCY_IDENTITY_AMBIGUOUS";
        }
        PlanningSceneBuilder scene_builder(
          scene_snapshot, request.non_target_occupancy, request.observation_pose.header);
        const auto manipulation_scene = scene_builder.build(PlanningScenePolicy{});
        if (!manipulation_scene.valid) {
          valid = false;
          retract_exception_context_result = manipulation_scene.failure_reason;
        } else {
          manipulation_scene_diff = manipulation_scene.diff;
        }
        const bool add_candidate_occupancy = vision_collision_scope_required &&
          request.phase ==
          arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
        PlanningScenePolicy path_policy;
        path_policy.vision_occupancy = add_candidate_occupancy ?
          VisionOccupancyPolicy::INCLUDE : VisionOccupancyPolicy::EXCLUDE;
        path_policy.apply_pick_retract_tray_exception = pick_retract_collision_exception;
        const auto path_scene = scene_builder.build(path_policy);
        if (!path_scene.valid) {
          valid = false;
          retract_exception_context_result = path_scene.failure_reason;
        } else {
          planning_request_scene_diff = path_scene.diff;
          request_candidate_ids = path_scene.candidate_ids;
        }
        if (!add_candidate_occupancy && vision_collision_scope_required) {
          PlanningScenePolicy occupancy_identity_policy;
          occupancy_identity_policy.vision_occupancy = VisionOccupancyPolicy::INCLUDE;
          const auto occupancy_identity_scene = scene_builder.build(occupancy_identity_policy);
          if (!occupancy_identity_scene.valid) {
            valid = false;
            retract_exception_context_result = occupancy_identity_scene.failure_reason;
          } else {
            request_candidate_ids = occupancy_identity_scene.candidate_ids;
          }
        }
        if (valid && !request_validation_scene->setPlanningSceneDiffMsg(
            manipulation_scene_diff))
        {
          valid = false;
          retract_exception_context_result = "SCENE_DIFF_INVALID";
        }
        if (valid && pick_retract_collision_exception) {
          const bool tray_present = std::any_of(
            scene_snapshot.world.collision_objects.begin(),
            scene_snapshot.world.collision_objects.end(), [](const auto & object) {
              return object.id == kPickRetractTrayCollisionObject;
            });
          const auto allowed_acm = make_pick_retract_allowed_collision_matrix(
            scene_snapshot.allowed_collision_matrix);
          if (!tray_present || !allowed_acm) {
            valid = false;
            retract_exception_context_result = tray_present ? "ACM_INVALID" : "TRAY_NOT_FOUND";
          } else {
            request_validation_scene->getAllowedCollisionMatrixNonConst() =
              collision_detection::AllowedCollisionMatrix(*allowed_acm);
            retract_scene_diff = make_pick_retract_planning_scene_diff(
              scene_snapshot.allowed_collision_matrix);
            manipulation_scene_diff.allowed_collision_matrix =
              retract_scene_diff.allowed_collision_matrix;
            planning_request_scene_diff.allowed_collision_matrix =
              retract_scene_diff.allowed_collision_matrix;
            retract_exception_context_result = "APPLIED_REQUEST_SCOPED";
          }
        }
        if (valid && request.task_type.value ==
          arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
          request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING)
        {
          PlanningScenePolicy grasp_policy;
          grasp_policy.vision_occupancy = VisionOccupancyPolicy::INCLUDE;
          const auto grasp_scene = scene_builder.build(grasp_policy);
          if (!grasp_scene.valid) {
            valid = false;
            retract_exception_context_result = grasp_scene.failure_reason;
          } else {
            grasp_validation_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
            if (!grasp_validation_scene->setPlanningSceneMsg(scene_snapshot) ||
              !grasp_validation_scene->setPlanningSceneDiffMsg(grasp_scene.diff))
            {
              grasp_validation_scene.reset();
              valid = false;
              retract_exception_context_result = "GRASP_SCENE_DIFF_INVALID";
            }
          }
        }
        if (!valid) {
          request_validation_scene.reset();
        }
      }
    }
  }
  if (planning_scene_context_required && !request_validation_scene) {
    RCLCPP_ERROR(
      node_->get_logger(), "Motion planning scene context unavailable: %s",
      retract_exception_context_result.c_str());
    {
      std::lock_guard<std::mutex> result_lock(mutex_);
      normalized_failure_result_ =
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    }
    const auto failure_stage = pick_retract_collision_exception ? "RETRACT_SCENE_VALIDATION" :
      request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE ?
      "PLACE_SCENE_CONTEXT" : "PICK_SCENE_CONTEXT";
    const nlohmann::json evidence = {
      {"motion_goal_uuid", request.motion_goal_uuid},
      {"task_type", motion_task_type_name(request.task_type.value)},
      {"failure_stage", failure_stage},
      {"failure_reason", retract_exception_context_result},
      {"final_failure_stage", failure_stage},
      {"final_failure_reason", retract_exception_context_result},
      {"vision_acquisition_id", request.vision_acquisition_id},
      {"vision_candidate_id", request.vision_candidate_id},
      {"vision_occupancy_scene_valid", false},
      {"vision_occupancy_evidence", {
        {"computed", true},
        {"scene_snapshot", {
          {"source", scene_snapshot_source},
          {"status", retract_exception_context_result},
          {"candidate_identity_match", false}}},
        {"actual_validation_policy", {
          {"vision_occupancy_exclusion_applied", false},
          {"requested_phase", motion_task_phase_name(request.phase)},
          {"failure_before_collision_check", true}}},
        {"primitives", [&request]() {
          auto primitives = nlohmann::json::array();
          for (const auto & primitive : request.non_target_occupancy) {
            primitives.push_back({{"occupancy_class", occupancy_class_name(primitive.occupancy_class)}});
          }
          return primitives;
        }()}}}};
    set_pending_motion_planning_evidence(
      request.motion_goal_uuid, request.task_type.value, evidence.dump());
    return false;
  }
  std::vector<double> latest_measured_start;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_joint_state_ ||
      std::chrono::steady_clock::now() - latest_state_receipt_ > kStateFreshness ||
      !extract_arm_positions(*latest_joint_state_, latest_measured_start))
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Motion planning cannot continue because the latest measured "
                  "joint state is missing or stale");
      return false;
    }
  }
  moveit::core::RobotState planning_start_state(*current_state);
  apply_measured_joint_positions(planning_start_state, arm_joint_names_, latest_measured_start);
  if (request.has_start_joint_state) {
    std::vector<double> requested_start;
    requested_start.reserve(arm_joint_names_.size());
    for (const auto & joint_name : arm_joint_names_) {
      const auto name = std::find(
        request.start_joint_state.name.begin(), request.start_joint_state.name.end(), joint_name);
      if (name == request.start_joint_state.name.end()) {
        std::lock_guard<std::mutex> result_lock(mutex_);
        normalized_failure_result_ =
          arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
        return false;
      }
      const auto index = static_cast<std::size_t>(
        std::distance(request.start_joint_state.name.begin(), name));
      if (index >= request.start_joint_state.position.size() ||
        !std::isfinite(request.start_joint_state.position[index]))
      {
        std::lock_guard<std::mutex> result_lock(mutex_);
        normalized_failure_result_ =
          arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
        return false;
      }
      requested_start.push_back(request.start_joint_state.position[index]);
    }
    apply_measured_joint_positions(planning_start_state, arm_joint_names_, requested_start);
    planning_start_state.update();
    if (!planning_start_state.satisfiesBounds(joint_group)) {
      std::lock_guard<std::mutex> result_lock(mutex_);
      normalized_failure_result_ =
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
      return false;
    }
  }
  std::vector<double> measured_start;
  std::vector<JointContinuityLimit> continuity_limits;
  measured_start.reserve(arm_joint_names_.size());
  continuity_limits.reserve(arm_joint_names_.size());
  for (const auto & joint_name : arm_joint_names_) {
    const auto * joint_model = robot_model->getJointModel(joint_name);
    if (!joint_model || joint_model->getVariableBounds().empty()) {
      RCLCPP_WARN(node_->get_logger(),
                  "motion diagnostic missing continuity bounds for %s"
                  "",
                  joint_name.c_str());
      return false;
    }
    const auto & bounds = joint_model->getVariableBounds().front();
    measured_start.push_back(planning_start_state.getVariablePosition(joint_name));
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
      RCLCPP_WARN(node_->get_logger(),
                  "motion diagnostic missing executable velocity bounds for %s"
                  "",
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
  const bool candidate_attempt_snapshot = vision_derived_pick &&
    request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
  std::string occupancy_snapshot_path;
  bool occupancy_snapshot_queued = false;
  if (candidate_attempt_snapshot) {
    try {
      occupancy_snapshot_queued = planning_diagnostic_capture_ &&
        planning_diagnostic_capture_->enqueue_occupancy_snapshot(
        request.motion_goal_uuid, request.vision_acquisition_id,
        request.vision_candidate_id, request.non_target_occupancy, occupancy_snapshot_path);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        node_->get_logger(), "Motion occupancy snapshot enqueue failed: %s", error.what());
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "Motion occupancy snapshot enqueue failed");
    }
  }
  nlohmann::json occupancy_evidence = {
    {"computed", request.has_non_target_occupancy},
    {"vision_acquisition_id", request.vision_acquisition_id},
    {"vision_candidate_id", request.vision_candidate_id},
    {"selected_candidate_id", vision_derived_pick ? request.vision_candidate_id : ""},
    {"frame", request.observation_pose.header.frame_id},
    {"stamp_sec", request.observation_pose.header.stamp.sec},
    {"stamp_nanosec", request.observation_pose.header.stamp.nanosec},
    {"primitives", nlohmann::json::array()}};
  if (diagnostics_capture_enabled_) {
    for (const auto & primitive : request.non_target_occupancy) {
      nlohmann::json evidence = {
        {"occupancy_class", occupancy_class_name(primitive.occupancy_class)},
        {"shape_type", primitive.shape_type},
        {"position", {primitive.pose.position.x, primitive.pose.position.y,
            primitive.pose.position.z}},
        {"orientation", {primitive.pose.orientation.x, primitive.pose.orientation.y,
            primitive.pose.orientation.z, primitive.pose.orientation.w}}};
      using Primitive = arm_cell_interfaces::msg::CollisionPrimitive;
      if (primitive.shape_type == Primitive::SOLID_PRIMITIVE) {
        evidence["solid_primitive_type"] = primitive.shape.type;
        evidence["dimensions"] = primitive.shape.dimensions;
      } else if (primitive.shape_type == Primitive::CONVEX_TOP) {
        auto vertices = nlohmann::json::array();
        for (const auto & v : primitive.top_vertices) {vertices.push_back({v.x, v.y, v.z});}
        evidence["top_vertices"] = std::move(vertices);
        evidence["bottom_height_m"] = primitive.bottom_height_m;
      }
      occupancy_evidence["primitives"].push_back(std::move(evidence));
    }
    if (vision_derived_pick ||
      (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
      request.has_non_target_occupancy && !request.non_target_occupancy.empty()))
    {
      auto scene_candidate_objects = nlohmann::json::array();
      for (std::size_t index = 0; index < request_candidate_ids.size(); ++index) {
        scene_candidate_objects.push_back(
          {
            {"object_id", request_candidate_ids[index]},
            {"occupancy_class", index < request.non_target_occupancy.size() ?
              occupancy_class_name(request.non_target_occupancy[index].occupancy_class) :
              "UNAVAILABLE"}});
      }
      occupancy_evidence["scene_snapshot"] = {
        {"source", scene_snapshot_source},
        {"status", request_validation_scene ?
          "FIXED_SCENE_VALID_REQUEST_GEOMETRY_IDENTIFIED" : retract_exception_context_result},
        {"candidate_identity_match", static_cast<bool>(request_validation_scene)},
        {"candidate_objects", std::move(scene_candidate_objects)}};
      occupancy_evidence["actual_validation_policy"] = {
        {"vision_occupancy_exclusion_applied", static_cast<bool>(request_validation_scene)},
        {"approach_global_planning", {
            {"vision_occupancy_collision_check",
              request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING ?
              "ADDED_TO_GLOBAL_PLANNING_REQUEST" : "NOT_APPLICABLE"},
            {"goal_state_validation",
              request.phase != arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING ?
              "NOT_APPLICABLE" : request_validation_scene ?
              "VISION_OCCUPANCY_EXCLUDED" : "NOT_RUN_SCENE_UNAVAILABLE"}}},
        {"cartesian_manipulation", {
            {"phase", motion_task_phase_name(request.phase)},
            {"vision_occupancy_collision_check", cartesian_primitive ?
              (request_validation_scene ? "EXCLUDED_DURING_PATH_SAMPLE_VALIDATION" :
              "NOT_RUN_SCENE_UNAVAILABLE") : "NOT_APPLICABLE"}}}};
    }
    if (candidate_attempt_snapshot) {
      nlohmann::json snapshot_record = {
        {"status", occupancy_snapshot_queued ? "queued" : "enqueue_failed"},
        {"candidate_id", request.vision_candidate_id},
        {"acquisition_id", request.vision_acquisition_id},
        {"primitive_count", request.non_target_occupancy.size()},
        {"planning_object_ids", request_candidate_ids}};
      if (occupancy_snapshot_queued) {
        snapshot_record["path"] = occupancy_snapshot_path;
      }
      occupancy_evidence["snapshot"] = std::move(snapshot_record);
    }
  }
  JointTrajectoryContinuityResult continuity;
  std::vector<double> selected_global_goal;
  bool selected_global_goal_valid = false;
  move_group_state_->move_group->setStartState(planning_start_state);
  if (waypoint) {
    MoveGroupPlanningTimeRestore restore_time(*move_group_state_->move_group);
    if (waypoint->size() != arm_joint_names_.size()) {return false;}
    moveit::core::RobotState endpoint(planning_start_state);
    endpoint.setJointGroupPositions(joint_group, *waypoint);
    endpoint.update();
    if (!endpoint.satisfiesBounds(joint_group) ||
      !move_group_state_->move_group->setJointValueTarget(*waypoint))
    {
      place_recovery_candidate_fatal_ = false;
      place_recovery_waypoint_reason_ = "joint_target_invalid";
      return false;
    }
    move_group_state_->move_group->setPlanningTime(budget_s);
    const auto result = plan_with_num_attempts(
      *move_group_state_->move_group, planning_num_attempts_, [&]() {
        if (planning_scene_context_required) {
          return plan_with_request_scene_diff(
            *move_group_state_->move_group, plan, planning_request_scene_diff,
            [&]() {return discard_if_interrupted();});
        }
        return move_group_state_->move_group->plan(plan);
      });
    place_recovery_last_plan_code_ = result.val;
    planning_succeeded = result == moveit::core::MoveItErrorCode::SUCCESS;
    place_recovery_waypoint_reason_ = planning_succeeded ? "planned" : "planner_failed";
    if (planning_succeeded) {
      const auto finalized = finalize_raw_trajectory(plan.trajectory_);
      planning_succeeded = finalized.success;
      if (!finalized.success) {place_recovery_waypoint_reason_ = finalized.reason;}
    }
    if (result.val == moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED ||
      result.val == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT)
    {
      place_recovery_candidate_fatal_ = false;
      std::lock_guard<std::mutex> result_lock(mutex_);
      normalized_failure_result_ =
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
    }
    if (planning_succeeded) {
      planning_scene::PlanningScene validation_scene(robot_model);
      moveit_msgs::msg::RobotState start_message;
      moveit::core::robotStateToRobotStateMsg(planning_start_state, start_message, true);
      moveit_msgs::msg::PlanningScene request_scene_message;
      if (request_validation_scene) {
        request_validation_scene->getPlanningSceneMsg(request_scene_message);
      }
      if (!request_validation_scene) {
        planning_succeeded = false;
        place_recovery_waypoint_reason_ = "scene_context_unavailable";
      } else if (!validation_scene.setPlanningSceneMsg(
          request_scene_message))
      {
        planning_succeeded = false;
        place_recovery_waypoint_reason_ = "scene_context_invalid";
      } else if (!validation_scene.isPathValid(start_message, plan.trajectory_, "arm")) {
        planning_succeeded = false;
        place_recovery_candidate_fatal_ = false;
        place_recovery_waypoint_reason_ = "waypoint_path_invalid";
      } else {place_recovery_waypoint_reason_ = "trajectory_validation_or_submission_failed";}
    }
    selected_global_goal = *waypoint;
    selected_global_goal_valid = true;
    selected_final_strategy = "PLACE_WAYPOINT";
  } else if (cartesian_primitive) {
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
    auto cartesian_target = request.tcp_target.pose;
    if (requires_world_vertical_pick_descent_guard(
        request.task_type.value, request.phase, request.pick_approach_axis) &&
      !make_world_vertical_pick_descent_target(
        start_tcp, request.tcp_target.pose, cartesian_target))
    {
      {
        std::lock_guard<std::mutex> result_lock(mutex_);
        normalized_failure_result_ =
          arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
      }
      if (diagnostics_capture_enabled_) {
        std::string selection;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          selection = pick_candidate_evaluation_;
        }
        if (selection.empty() || selection.back() != '}') {
          selection = "{}";
        }
        std::ostringstream record;
        record << std::setprecision(17)
               << "{\"motion_goal_uuid\":\"" << json_escape(request.motion_goal_uuid)
               << "\",\"phase\":\"EXECUTING\",\"stage\":\"cartesian_descent\","
               << "\"status\":\"REJECTED\","
               << "\"direction\":\"planning_frame_world_negative_z\","
               << "\"failure_reason\":\"approach_not_above_grasp_in_planning_frame\","
               << "\"path_fraction\":0.0,\"sample_count\":0,"
               << "\"validated_sample_count\":0,"
               << "\"failure_sample\":{\"available\":false,"
               << "\"unavailable_reason\":\"path_generation_not_started\","
               << "\"pose_available\":false,"
               << "\"pose_unavailable_reason\":\"path_generation_not_started\","
               << "\"contact_bodies\":{\"available\":false,"
               << "\"unavailable_reason\":\"state_validity_not_run\",\"pairs\":[]}}";
        append_cartesian_planning_elapsed(record, std::nullopt);
        record << ",\"start\":";
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header.frame_id = request.target_pose.header.frame_id;
        start_pose.pose = start_tcp;
        append_json_pose(record, start_pose);
        record << ",\"target\":";
        append_json_pose(record, request.tcp_target);
        record << ",\"requested_grasp_pose\":";
        append_json_pose(record, request.tcp_target);
        record << '}';
        const auto combined = upsert_cartesian_diagnostic(
          std::move(selection), "cartesian_descent", record.str());
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pick_candidate_evaluation_ = combined;
        }
        set_pending_motion_planning_evidence(
          request.motion_goal_uuid, request.task_type.value, combined);
      }
      RCLCPP_WARN(node_->get_logger(),
                  "PICK Cartesian descent rejected because measured Approach "
                  "is not above Grasp");
      restore_grasp_contact_policy();
      return false;
    }

    constexpr std::size_t kNoPathSampleIndex =
      std::numeric_limits<std::size_t>::max();
    std::size_t cartesian_collision_source_candidate = 0U;
    std::size_t cartesian_collision_source_seed_attempt = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (request.task_type.value ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
        selected_planning_candidate_task_ ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
        planning_candidate_goal_uuid_ == request.motion_goal_uuid)
      {
        cartesian_collision_source_candidate = selected_planning_candidate_index_ + 1U;
        cartesian_collision_source_seed_attempt = selected_pick_ik_seed_attempt_;
      }
    }
    const auto validate_executable_state = [&](std::size_t sample_index,
        const std::vector<double> & sample_positions,
        std::string & reason,
        std::vector<std::string> & contact_bodies) {
        moveit::core::RobotState sample_state(planning_start_state);
        sample_state.setJointGroupPositions(joint_group, sample_positions);
        sample_state.update();
        if (!sample_state.satisfiesBounds(joint_group)) {
          reason = "joint bounds";
          return false;
        }
        const Eigen::Quaterniond actual_orientation(
          sample_state.getGlobalLinkTransform("sf_grasp_tcp").rotation());
        const Eigen::Quaterniond required_orientation(
          cartesian_target.orientation.w, cartesian_target.orientation.x,
          cartesian_target.orientation.y, cartesian_target.orientation.z);
        if (actual_orientation.angularDistance(required_orientation) >
          cartesian_tcp_orientation_tolerance_rad_)
        {
          reason = "Cartesian TCP orientation";
          return false;
        }
        if (request_validation_scene || pick_retract_collision_exception) {
          if (!request_validation_scene) {
            reason = "PICK request-scoped validation scene unavailable";
            return false;
          }
          collision_detection::CollisionRequest collision_request;
          collision_request.group_name = "arm";
          collision_request.contacts = true;
          collision_request.max_contacts = 32;
          collision_request.max_contacts_per_pair = 4;
          collision_detection::CollisionResult collision_result;
          request_validation_scene->checkCollision(
            collision_request, collision_result, sample_state);
          if (collision_result.collision) {
            for (const auto & contact_pair : collision_result.contacts) {
              contact_bodies.push_back(contact_pair.first.first);
              contact_bodies.push_back(contact_pair.first.second);
            }
          }
          if (collision_result.collision) {
            reason = "collision/contact";
            return false;
          }
          reason.clear();
          contact_bodies.clear();
          return true;
        }
        if (!state_validity_client_ ||
          !state_validity_client_->wait_for_service(std::chrono::milliseconds(200))) {
          reason = "state validity service unavailable";
          return false;
        }
        auto validity_request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        moveit::core::robotStateToRobotStateMsg(sample_state, validity_request->robot_state, true);
        validity_request->group_name = "arm";
        auto validity_future = state_validity_client_->async_send_request(validity_request);
        if (validity_future.wait_for(std::chrono::milliseconds(500)) != std::future_status::ready) {
          reason = "state validity request timed out";
          return false;
        }
        const auto validity = validity_future.get();
        if (!validity->valid) {
          std::vector<PickCollisionContact> observed_contacts;
          for (const auto & contact : validity->contacts) {
            contact_bodies.push_back(contact.contact_body_1);
            contact_bodies.push_back(contact.contact_body_2);
            if (request.task_type.value ==
              arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK)
            {
              observed_contacts.push_back(PickCollisionContact{
                  contact.contact_body_1, contact.contact_body_2,
                  sample_index == kNoPathSampleIndex ?
                  "final_grasp_target_state_validity" :
                  "final_cartesian_path_validation",
                  sample_index != kNoPathSampleIndex,
                  sample_index == kNoPathSampleIndex ? 0U : sample_index,
                  cartesian_collision_source_candidate,
                  cartesian_collision_source_seed_attempt});
            }
          }
          record_pick_collision_feedback(observed_contacts);
          reason = contact_bodies.empty() ?
            "state validity check failed" : "collision/contact";
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
        const Eigen::Quaterniond actual_orientation(
          solution_state.getGlobalLinkTransform("sf_grasp_tcp").rotation());
        const Eigen::Quaterniond required_orientation(
          sample_tcp.orientation.w, sample_tcp.orientation.x,
          sample_tcp.orientation.y, sample_tcp.orientation.z);
        if (actual_orientation.angularDistance(required_orientation) >
          cartesian_tcp_orientation_tolerance_rad_)
        {
          reason = "orientation";
          return false;
        }
        solution_state.copyJointGroupPositions(joint_group, solution_positions);
        reason.clear();
        return true;
      };

    const auto check_collision = [&](std::size_t sample_index,
        const geometry_msgs::msg::Pose &, const std::vector<double> & sample_positions,
        std::string & reason, std::vector<std::string> & contact_bodies) {
        return validate_executable_state(
          sample_index, sample_positions, reason, contact_bodies);
      };

    const auto record_cartesian_diagnostic = [&](const std::string & status,
        const CartesianLinearGenerationResult & result, const std::string & failure_reason,
        std::optional<double> planning_elapsed_s) {
        if (!diagnostics_capture_enabled_) {return;}
        std::string evidence;
        {
          std::lock_guard<std::mutex> lock(planning_diagnostic_mutex_);
          evidence = pending_motion_planning_goal_uuid_ == request.motion_goal_uuid ?
            pending_motion_planning_evidence_ : std::string{};
        }
        if (request.task_type.value ==
          arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!pick_candidate_evaluation_.empty()) {evidence = pick_candidate_evaluation_;}
        }
        const bool retract_phase = request.phase ==
          arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING;
        const std::string diagnostic_key = retract_phase ? "cartesian_retract" :
          request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ?
          "cartesian_descent" : "place_cartesian_release";
        const std::string direction = request.pick_approach_axis ==
          PickApproachAxis::TCP_INSERTION_AXIS ?
          (retract_phase ? "tcp_insertion_axis_away_from_grasp" :
          "tcp_insertion_axis_toward_grasp") :
          (retract_phase ? "tcp_insertion_axis_away_from_grasp" :
          "planning_frame_world_negative_z");
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header.frame_id = request.target_pose.header.frame_id;
        start_pose.pose = start_tcp;
        geometry_msgs::msg::PoseStamped descent_pose = start_pose;
        descent_pose.pose = cartesian_target;
        auto serialized = make_pick_cartesian_diagnostic(
          request.motion_goal_uuid, diagnostic_key, status, failure_reason, direction,
          start_pose, descent_pose, request.tcp_target, result, planning_elapsed_s);
        if (result.reused_executed_trajectory) {
          auto record = nlohmann::json::parse(serialized, nullptr, false);
          if (record.is_object()) {
            record["reused_executed_descent"] = true;
            record["reuse_direction"] = "REVERSE";
            serialized = record.dump();
          }
        }
        if (pick_retract_collision_exception) {
          auto record = nlohmann::json::parse(serialized, nullptr, false);
          if (record.is_object()) {
            record["collision_exception"] = {
              {"scope", "PICK_RETRACT_PLANNING_AND_VALIDATION"},
              {"applied", static_cast<bool>(retract_validation_scene)},
              {"context_result", retract_exception_context_result},
              {"allowed_pairs", {
                {kPickRetractTrayAllowedLinks[0], kPickRetractTrayCollisionObject},
                {kPickRetractTrayAllowedLinks[1], kPickRetractTrayCollisionObject}}}};
            serialized = record.dump();
          }
        }
        evidence = status == "READY_FOR_EXECUTION" ?
          upsert_cartesian_ready_diagnostic(
          std::move(evidence), diagnostic_key, serialized, selected_final_strategy) :
          upsert_cartesian_diagnostic(std::move(evidence), diagnostic_key, serialized);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pick_candidate_evaluation_ = evidence;
        }
        set_pending_motion_planning_evidence(request.motion_goal_uuid, request.task_type.value, evidence);
        if (planning_diagnostic_capture_) {
          planning_diagnostic_capture_->enqueue_progress(
            request.motion_goal_uuid, motion_task_type_name(request.task_type.value), evidence,
            request.vision_acquisition_id);
        }
      };

    const CartesianPlanningTimer cartesian_planning_timer;
    bool cartesian_accepted = false;
    std::string cartesian_validation_failure;
    std::vector<nlohmann::json> final_global_fallback_attempts;
    std::vector<nlohmann::json> cartesian_attempts;
    const auto cartesian_request_limit = request.task_type.value ==
      arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ?
      pick_final_cartesian_max_requests_ : place_final_cartesian_max_requests_;
    CartesianLinearGenerationResult generated;
    bool reused_pick_descent = false;
    if (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING)
    {
      trajectory_msgs::msg::JointTrajectory descent;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        descent = reusable_pick_descent_trajectory_;
      }
      generated.trajectory = reverse_joint_trajectory(descent);
      if (generated.trajectory.points.empty())
      {
        {
          std::lock_guard<std::mutex> result_lock(mutex_);
          normalized_failure_result_ =
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
        }
        set_pending_motion_planning_evidence(request.motion_goal_uuid,
          request.task_type.value, nlohmann::json{
            {"motion_goal_uuid", request.motion_goal_uuid},
            {"stage", "cartesian_retract_path_reuse"},
            {"reused_executed_descent", false},
            {"reuse_failure_reason", "NO_EXECUTED_DESCENT_AVAILABLE"}}.dump());
        return false;
      }
      generated.continuity = normalize_joint_trajectory(
        generated.trajectory, measured_start, continuity_limits, false);
      if (!generated.continuity.valid) {
        {
          std::lock_guard<std::mutex> result_lock(mutex_);
          normalized_failure_result_ =
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
        }
        set_pending_motion_planning_evidence(request.motion_goal_uuid,
          request.task_type.value, nlohmann::json{
            {"motion_goal_uuid", request.motion_goal_uuid},
            {"stage", "cartesian_retract_path_reuse"},
            {"reused_executed_descent", true},
            {"reuse_failure_reason", "TRAJECTORY_CONTINUITY_REJECTED"},
            {"trajectory_continuity_reason", generated.continuity.reason}}.dump());
        return false;
      }
      generated.valid = true;
      generated.reused_executed_trajectory = true;
      generated.path_fraction = 1.0;
      generated.sample_count = generated.trajectory.points.size();
      reused_pick_descent = true;
    }
    for (std::size_t request_index = 0;
      !reused_pick_descent && request_index < cartesian_request_limit; ++request_index)
    {
      if (discard_if_interrupted()) {
        restore_grasp_contact_policy();
        return false;
      }
      // Refine the same straight TCP segment on each request. This changes
      // Cartesian sampling resolution without changing its target or path.
      const double spatial_step = kManipulationCartesianStepM /
        static_cast<double>(request_index + 1U);
      generated = generate_cartesian_linear_trajectory(
        start_tcp, cartesian_target, measured_start, arm_joint_names_, continuity_limits,
        spatial_step, solve_ik, check_collision);
      nlohmann::json cartesian_attempt;
      cartesian_attempt["request_index"] = request_index + 1U;
      cartesian_attempt["spatial_step_m"] = spatial_step;
      cartesian_attempt["result"] = generated.valid ? "SUCCESS" : "FAILED";
      cartesian_attempt["path_fraction"] = generated.path_fraction;
      cartesian_attempt["sample_count"] = generated.sample_count;
      cartesian_attempt["validated_sample_count"] = generated.validated_sample_count;
      cartesian_attempt["failure_reason"] = generated.failure.reason;
      cartesian_attempt["failure_sample_index"] = generated.failure.sample_index;
      cartesian_attempt["failure_contact_bodies"] = generated.failure.contact_bodies;
      cartesian_attempts.push_back(std::move(cartesian_attempt));
      if (generated.valid) {break;}
    }
    planning_succeeded = generated.valid;
    if (generated.valid) {
      plan.trajectory_.joint_trajectory = generated.trajectory;
      continuity = generated.continuity;
      const auto finalized = finalize_raw_trajectory(plan.trajectory_);
      if (finalized.success) {
        const auto final_validation = validate_cartesian_joint_trajectory(
          plan.trajectory_.joint_trajectory, measured_start, continuity_limits,
          validate_executable_state);
        if (final_validation.valid) {
          continuity = final_validation.continuity;
          generated.validated_sample_count = final_validation.validated_sample_count;
          cartesian_accepted = true;
        } else {
          cartesian_validation_failure = final_validation.reason;
        }
      } else {
        cartesian_validation_failure = finalized.reason;
      }
      planning_succeeded = cartesian_accepted;
    }
    if (!cartesian_accepted) {
      record_cartesian_diagnostic(
        generated.valid ? "EXECUTABLE_VALIDATION_FAILED" : "FAILED", generated,
        generated.valid ? cartesian_validation_failure : generated.reason,
        cartesian_planning_timer.elapsed_s());
      if (discard_if_interrupted()) {
        restore_grasp_contact_policy();
        return false;
      }

      if (reused_pick_descent) {
        std::lock_guard<std::mutex> result_lock(mutex_);
        normalized_failure_result_ =
          arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
        return false;
      }

      // A global final-segment route is eligible only after independently
      // proving the exact requested TCP goal has a collision-free IK state.
      // This keeps Cartesian endpoint collision and state-validity service
      // failures fail-closed while permitting a planner route around an
      // obstructed straight segment.
      std::vector<double> fallback_goal;
      std::string fallback_goal_reason;
      bool fallback_goal_valid = (!pick_retract_collision_exception ||
        static_cast<bool>(retract_validation_scene)) && solve_ik(
        request.tcp_target.pose, measured_start, fallback_goal, fallback_goal_reason);
      if (fallback_goal_valid) {
        std::vector<std::string> goal_contacts;
        fallback_goal_valid = validate_executable_state(
          kNoPathSampleIndex, fallback_goal, fallback_goal_reason, goal_contacts);
      }
      const auto unavailable_validity_evidence = [](const std::string & reason) {
          return reason.find("state validity service unavailable") != std::string::npos ||
                 reason.find("state validity request timed out") != std::string::npos;
        };
      const bool path_validity_unavailable =
        unavailable_validity_evidence(generated.valid ? cartesian_validation_failure :
        generated.failure.reason);
      const auto fallback_limit = request.task_type.value ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ?
        pick_final_global_fallback_max_requests_ : place_final_global_fallback_max_requests_;
      if (fallback_goal_valid && !path_validity_unavailable && fallback_limit > 0U &&
        move_group_state_->move_group->setPoseTarget(request.tcp_target, "sf_grasp_tcp"))
      {
        moveit::planning_interface::MoveGroupInterface::Plan fallback_plan;
        const auto fallback_start = std::chrono::steady_clock::now();
        for (std::size_t request_index = 0; request_index < fallback_limit; ++request_index) {
          if (discard_if_interrupted()) {
            move_group_state_->move_group->clearPoseTargets();
            restore_grasp_contact_policy();
            return false;
          }
          const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - fallback_start).count();
          const double remaining = planning_time_s_ - elapsed;
          if (remaining <= 0.0) {break;}
          move_group_state_->move_group->setPlanningTime(remaining);
          const auto global_result = plan_with_num_attempts(
            *move_group_state_->move_group, planning_num_attempts_, [&]() {
              if (pick_retract_collision_exception || vision_collision_scope_required) {
                return plan_with_request_scene_diff(
                  *move_group_state_->move_group, fallback_plan, manipulation_scene_diff,
                  [&]() {return discard_if_interrupted();});
              }
              return move_group_state_->move_group->plan(fallback_plan);
            });
          nlohmann::json attempt_record;
          attempt_record["request_index"] = request_index + 1U;
          attempt_record["moveit_error_code"] = global_result.val;
          attempt_record["trajectory_points"] =
            fallback_plan.trajectory_.joint_trajectory.points.size();
          attempt_record["planning_elapsed_s"] = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - fallback_start).count();
          if (discard_if_interrupted()) {
            move_group_state_->move_group->clearPoseTargets();
            restore_grasp_contact_policy();
            return false;
          }
          if (global_result != moveit::core::MoveItErrorCode::SUCCESS ||
            fallback_plan.trajectory_.joint_trajectory.points.empty())
          {
            attempt_record["result"] = "PLANNING_FAILED";
            attempt_record["finalization_reason"] = "planner_failed_or_empty_trajectory";
            final_global_fallback_attempts.push_back(std::move(attempt_record));
            continue;
          }
          const auto finalized = finalize_raw_trajectory(fallback_plan.trajectory_);
          attempt_record["trajectory_duration_s"] = finalized.duration_s;
          attempt_record["finalization_reason"] = finalized.reason;
          if (!finalized.success) {
            attempt_record["result"] = "TRAJECTORY_INVALID";
            final_global_fallback_attempts.push_back(std::move(attempt_record));
            continue;
          }
          auto fallback_continuity = normalize_joint_trajectory(
            fallback_plan.trajectory_.joint_trajectory, measured_start,
            continuity_limits, false);
          if (!fallback_continuity.valid) {
            attempt_record["result"] = "CONTINUITY_INVALID";
            attempt_record["continuity_reason"] = fallback_continuity.reason;
            final_global_fallback_attempts.push_back(std::move(attempt_record));
            continue;
          }
          attempt_record["result"] = "SUCCESS";
          final_global_fallback_attempts.push_back(std::move(attempt_record));
          plan = std::move(fallback_plan);
          continuity = std::move(fallback_continuity);
          selected_final_strategy = "GLOBAL_FALLBACK";
          planning_succeeded = true;
          break;
        }
        move_group_state_->move_group->clearPoseTargets();
      }
      RCLCPP_DEBUG(
        node_->get_logger(), "%s final-segment Cartesian planning %s; global fallback %s",
        request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ?
        "PICK" : "PLACE", generated.valid ? "succeeded" : "failed",
        planning_succeeded ? "selected" :
        fallback_goal_valid ? "exhausted" : "not eligible");
      const bool state_validity_unavailable =
        path_validity_unavailable ||
        unavailable_validity_evidence(fallback_goal_reason);
      const bool default_planner_error = std::any_of(
        final_global_fallback_attempts.begin(), final_global_fallback_attempts.end(),
        [](const auto & attempt) {
          const int32_t code = attempt.value("moveit_error_code", 0);
          using Code = moveit_msgs::msg::MoveItErrorCodes;
          return code != Code::SUCCESS && code != Code::PLANNING_FAILED &&
                 code != Code::TIMED_OUT && code != Code::GOAL_IN_COLLISION &&
                 code != Code::START_STATE_IN_COLLISION;
        });
      const bool default_route_failure = fallback_goal_reason == "collision/contact" ||
        generated.failure.reason == "collision/contact" ||
        (!final_global_fallback_attempts.empty() && !default_planner_error);
      if (!planning_succeeded && !state_validity_unavailable && !default_planner_error &&
        !discard_if_interrupted())
      {
        std::lock_guard<std::mutex> candidate_lock(mutex_);
        if (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
          default_route_failure && !default_planner_error)
        {
          // Classify the actual collision/no-route outcome; backend/scene errors
          // keep their failure class and interruption retains precedence.
          normalized_failure_result_ =
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
        }
        if (selected_planning_candidate_task_ ==
          arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK)
        {
          next_candidate_available_ = selected_planning_candidate_index_ + 1U <
            pick_active_grasp_pose_candidate_count_;
        } else if (selected_planning_candidate_task_ ==
          arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE)
        {
          next_candidate_available_ = selected_planning_candidate_index_ + 1U <
            place_pose_candidate_count_;
        }
      }
      if (diagnostics_capture_enabled_) {
        nlohmann::json stage;
        stage["motion_goal_uuid"] = request.motion_goal_uuid;
        stage["task_type"] = motion_task_type_name(request.task_type.value);
        stage["stage"] = request.phase ==
          arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING ?
          "final_grasp_or_place" : "cartesian_manipulation";
        stage["cartesian_max_requests"] = request.task_type.value ==
          arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ?
          pick_final_cartesian_max_requests_ : place_final_cartesian_max_requests_;
        stage["cartesian_attempts"] = cartesian_attempts;
        stage["global_fallback_max_requests"] = fallback_limit;
        stage["cartesian_result"] = cartesian_accepted ? "SUCCESS" :
          generated.valid ? "EXECUTABLE_VALIDATION_FAILED" : "FAILED";
        stage["cartesian_path_fraction"] = generated.path_fraction;
        stage["cartesian_sample_count"] = generated.sample_count;
        stage["cartesian_validated_sample_count"] = generated.validated_sample_count;
        stage["cartesian_failure_reason"] = generated.valid ?
          cartesian_validation_failure : generated.failure.reason;
        stage["cartesian_failure_sample_index"] = generated.failure.sample_index;
        stage["cartesian_failure_contact_bodies"] = generated.failure.contact_bodies;
        stage["start_joint_positions"] = measured_start;
        std::ostringstream target_record;
        append_json_pose(target_record, request.tcp_target);
        stage["target"] = nlohmann::json::parse(target_record.str(), nullptr, false);
        stage["goal_ik_valid_and_collision_free"] = fallback_goal_valid;
        stage["goal_validation_failure"] = fallback_goal_reason;
        stage["global_fallback_attempts"] = final_global_fallback_attempts;
        if (pick_retract_collision_exception) {
          stage["collision_exception"] = {
            {"scope", "PICK_RETRACT_PLANNING_AND_VALIDATION"},
            {"applied", static_cast<bool>(retract_validation_scene)},
            {"context_result", retract_exception_context_result},
            {"allowed_pairs", {
              {kPickRetractTrayAllowedLinks[0], kPickRetractTrayCollisionObject},
              {kPickRetractTrayAllowedLinks[1], kPickRetractTrayCollisionObject}}}};
        }
        stage["final_planning_result"] = planning_succeeded ? "SUCCESS" : "FAILED";
        stage["selected_strategy"] = planning_succeeded ?
          "GLOBAL_FALLBACK" : "NO_EXECUTABLE_PATH";
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stage["candidate_transition_available"] = next_candidate_available_;
        }
        auto evidence = request.task_type.value ==
          arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ?
          pick_candidate_evaluation_ : std::string{};
        if (evidence.empty()) {
          std::lock_guard<std::mutex> lock(planning_diagnostic_mutex_);
          evidence = pending_motion_planning_evidence_;
        }
        auto final_evidence = nlohmann::json::parse(evidence.empty() ? "{}" : evidence,
          nullptr, false);
        if (!final_evidence.is_object()) {final_evidence = nlohmann::json::object();}
        final_evidence["vision_occupancy_evidence"] = occupancy_evidence;
        const auto combined = upsert_cartesian_diagnostic(
          final_evidence.dump(), "final_segment_planning", stage.dump());
        if (!combined.empty()) {
          if (request.task_type.value ==
            arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK)
          {
            std::lock_guard<std::mutex> lock(mutex_);
            pick_candidate_evaluation_ = combined;
          }
          set_pending_motion_planning_evidence(
            request.motion_goal_uuid, request.task_type.value, combined);
          if (planning_diagnostic_capture_) {
            planning_diagnostic_capture_->enqueue_progress(
              request.motion_goal_uuid, motion_task_type_name(request.task_type.value), combined,
              request.vision_acquisition_id);
          }
        }
      }
    }
    if (planning_succeeded) {
      // Both Cartesian and global trajectories have passed their own execution
      // validity checks before reaching this point.
    }
    if (planning_succeeded) {
      if (cartesian_result_selected_for_execution(selected_final_strategy)) {
        record_cartesian_diagnostic(
          "READY_FOR_EXECUTION", generated, "",
          cartesian_planning_timer.elapsed_s());
      }
    }
  } else {
    const bool place_global_approach =
      request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
    const bool place_clearance = request.segment == MotionSegmentKind::PLACE_CLEARANCE &&
      request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE;
    if (place_global_approach) {
      PlaceApproachIkPolicy ik_policy;
      ik_policy.per_candidate_planning_budget_s = place_candidate_planning_time_s_;
      ik_policy.total_planning_budget_s = budget_s > 0.0 ?
        std::min(place_total_planning_time_s_, budget_s) : place_total_planning_time_s_;
      ik_policy.planning_requests_per_candidate = place_approach_global_max_requests_;
      const auto seeds = generate_place_approach_ik_seeds(
        measured_start, arm_joint_names_, continuity_limits, ik_policy);
      auto object_pose_candidates = generate_place_object_pose_candidates(
        request.object_pose, request.place_position_tolerance_m,
        request.place_orientation_tolerance_rad, request.place_orientation_constraint);
      if (object_pose_candidates.size() > place_pose_candidate_limit_) {
        object_pose_candidates.resize(place_pose_candidate_limit_);
      }
      {
        std::lock_guard<std::mutex> candidate_lock(mutex_);
        place_pose_candidate_count_ = object_pose_candidates.size();
      }
      std::size_t place_pose_floor = 0U;
      {
        std::lock_guard<std::mutex> candidate_lock(mutex_);
        place_pose_floor = qualify_place_target ? 0U : place_candidate_pose_floor_;
      }
      if (seeds.empty() || object_pose_candidates.empty()) {
        if (diagnostics_capture_enabled_) {
          std::ostringstream evidence;
          evidence << std::setprecision(17)
                   << "{\"planning_stage\":\"candidate_generation\","
                   << "\"failure_reason\":\""
                   << (seeds.empty() ? "no_valid_ik_seed" : "no_valid_pose_candidate")
                   << "\",\"requested_object_pose\":";
          append_json_pose(evidence, request.object_pose);
          evidence << ",\"requested_tcp_target\":";
          append_json_pose(evidence, request.target_pose);
          evidence << ",\"requested_approach_target\":";
          append_json_pose(evidence, request.approach_target);
          evidence << ",\"planning_start_joint_positions\":";
          append_json_numbers(evidence, measured_start);
          evidence << ",\"pose_candidate_count\":" << object_pose_candidates.size()
                   << ",\"ik_seed_count\":" << seeds.size()
                   << ",\"safety_motion_envelope\":{\"valid\":"
                   << (envelope.valid ? "true" : "false")
                   << ",\"max_velocity_scale\":" << envelope.max_velocity_scale
                   << ",\"max_acceleration_scale\":"
                   << envelope.max_acceleration_scale << "}}";
          set_pending_motion_planning_evidence(
            request.motion_goal_uuid, request.task_type.value, evidence.str());
        }
        restore_grasp_contact_policy();
        RCLCPP_WARN(node_->get_logger(),
                    "PLACE planning failed because no valid object pose or IK "
                    "seed was available"
                    "");
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
          candidate.seed_index = seed.index;
          candidate.seed_source = seed.source;
          candidate.seed_positions = seed.joints;
          if (pose_index < place_pose_floor) {
            candidate.rejection_reason = "PREVIOUS_POSE_CANDIDATE_EXHAUSTED";
          }
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
          candidates.push_back(std::move(candidate));
        }
      }

      auto priority_candidate_indices = order_place_approach_candidate_specs(candidates);
      const bool bounded_place_orientation =
        request.place_orientation_constraint == PlaceOrientationConstraint::BOUNDED;
      const auto bounded_pose_groups = bounded_place_orientation ?
        group_bounded_place_pose_candidates(object_pose_candidates, request.object_pose) :
        BoundedPlacePoseCandidateGroups{};
      const auto partition_priority_candidates = [&](
        const std::vector<std::size_t> & pose_indices) {
          std::vector<std::size_t> pose_ranked;
          for (const auto index : priority_candidate_indices) {
            if (std::find(
                pose_indices.begin(), pose_indices.end(),
                candidates[index].object_pose_candidate_index) != pose_indices.end())
            {
              pose_ranked.push_back(index);
            }
          }
          return pose_ranked;
        };
      const auto primary_bounded_candidates = bounded_place_orientation ?
        partition_priority_candidates(bounded_pose_groups.primary_orientation_indices) :
        std::vector<std::size_t>{};
      const auto fallback_position_candidates = bounded_place_orientation ?
        partition_priority_candidates(bounded_pose_groups.position_fallback_indices) :
        std::vector<std::size_t>{};
      const auto planning_candidate_indices = bounded_place_orientation ?
        primary_bounded_candidates : priority_candidate_indices;
      const auto capture_place_plan_evaluation = [&](
        const PlaceApproachPlanResult & result, const char * failure_reason,
        const std::string & recovery_eligibility = "not_evaluated") {
          if (!diagnostics_capture_enabled_) {return;}
          std::ostringstream evidence;
          evidence << std::setprecision(17)
                   << "{\"planning_stage\":\""
                   << (qualify_place_target ? "place_target_qualification" : "place_global_approach")
                   << "\","
                   << "\"planning_group\":\"" << json_escape(joint_group->getName()) << '\"'
                   << ",\"planner_id\":\"" << json_escape(
            move_group_state_->move_group->getPlannerId()) << '\"'
                   << ",\"scene_identity\":{\"status\":\"unavailable\","
                   << "\"reason\":\"MoveIt scene revision is not exposed at candidate evaluation\"}"
                   << ",\"requested_object_pose\":";
          append_json_pose(evidence, request.object_pose);
          evidence << ",\"requested_tcp_target\":";
          append_json_pose(evidence, request.target_pose);
          evidence << ",\"requested_approach_target\":";
          append_json_pose(evidence, request.approach_target);
          evidence << ",\"constraints\":{\"position_tolerance_m\":";
          append_json_number(evidence, request.place_position_tolerance_m);
          evidence << ",\"orientation_tolerance_rad\":";
          append_json_number(evidence, request.place_orientation_tolerance_rad);
          evidence << ",\"orientation_constraint\":\""
                   << (request.place_orientation_constraint == PlaceOrientationConstraint::FIXED ?
          "FIXED" : request.place_orientation_constraint == PlaceOrientationConstraint::BOUNDED ?
          "BOUNDED" : "FREE")
                   << "\",\"has_tool_orientation_preference\":"
                   << (request.has_place_tool_orientation_preference ? "true" : "false")
                   << ",\"tool_orientation_preference_xyzw\":[";
          append_json_number(evidence, request.place_tool_orientation_preference.x);
          evidence << ',';
          append_json_number(evidence, request.place_tool_orientation_preference.y);
          evidence << ',';
          append_json_number(evidence, request.place_tool_orientation_preference.z);
          evidence << ',';
          append_json_number(evidence, request.place_tool_orientation_preference.w);
          evidence << "],\"approach_direction_object_xyz\":[";
          append_json_number(evidence, request.place_approach_direction_object.x);
          evidence << ',';
          append_json_number(evidence, request.place_approach_direction_object.y);
          evidence << ',';
          append_json_number(evidence, request.place_approach_direction_object.z);
          evidence << "],\"approach_distance_m\":";
          append_json_number(evidence, request.approach_distance_m);
          evidence << "},\"safety_motion_envelope\":{\"valid\":"
                   << (envelope.valid ? "true" : "false")
                   << ",\"max_velocity_scale\":" << envelope.max_velocity_scale
                   << ",\"max_acceleration_scale\":"
                   << envelope.max_acceleration_scale << '}'
                   << ",\"planning_settings\":{\"moveit_velocity_scaling\":";
          append_json_number(evidence, planning_scales.max_velocity);
          evidence << ",\"moveit_acceleration_scaling\":";
          append_json_number(evidence, planning_scales.max_acceleration);
          evidence << ",\"moveit_planning_time_s\":";
          append_json_number(evidence, planning_time_s_);
          evidence << ",\"moveit_num_planning_attempts\":" << planning_num_attempts_;
          evidence << ",\"per_candidate_budget_s\":";
          append_json_number(evidence, ik_policy.per_candidate_planning_budget_s);
          evidence << ",\"total_candidate_budget_s\":";
          append_json_number(evidence, ik_policy.total_planning_budget_s);
          evidence << ",\"ik_timeout_s\":";
          append_json_number(evidence, ik_policy.ik_timeout_s);
          evidence <<
            ",\"candidate_ranking\":\"tool_orientation_preference_then_pose_deviation_then_existing_ik_seed_order; joint_delta_diagnostic_only\""
                   << ",\"plan_selection_policy\":\""
                   << "FIRST_EXECUTABLE_TRAJECTORY_IN_CANDIDATE_RANKING\"";
          evidence <<
            "},\"start_state\":{\"basis\":\"MoveIt current state used to build the plan\","
                   << "\"joint_names\":[";
          for (std::size_t index = 0; index < arm_joint_names_.size(); ++index) {
            if (index != 0U) {evidence << ',';}
            evidence << '\"' << json_escape(arm_joint_names_[index]) << '\"';
          }
          evidence << "],\"joint_positions\":";
          append_json_numbers(evidence, measured_start);
          evidence << "},\"pose_candidates\":[";
          for (std::size_t index = 0; index < object_pose_candidates.size(); ++index) {
            if (index != 0U) {evidence << ',';}
            evidence << "{\"index\":" << index << ",\"nominal_deviation\":";
            append_json_number(evidence, object_pose_candidates[index].nominal_deviation);
            evidence << ",\"object_pose\":";
            append_json_pose(evidence, object_pose_candidates[index].object_pose);
            evidence << ",\"tcp_target\":";
            append_json_pose(evidence, place_geometry[index].target_pose);
            evidence << ",\"tcp_approach\":";
            append_json_pose(evidence, place_geometry[index].approach_pose);
            evidence << '}';
          }
          evidence << "],\"candidates\":[";
          for (std::size_t index = 0; index < candidates.size(); ++index) {
            if (index != 0U) {evidence << ',';}
            const auto & candidate = candidates[index];
            evidence << "{\"generation_index\":" << index
                     << ",\"pose_candidate_index\":"
                     << candidate.object_pose_candidate_index
                     << ",\"nominal_pose_deviation\":";
            append_json_number(evidence, candidate.nominal_pose_deviation);
            evidence << ",\"tool_orientation_preference_error_rad\":";
            if (candidate.tool_orientation_preference_error_rad) {
              append_json_number(evidence, *candidate.tool_orientation_preference_error_rad);
            } else {
              evidence << "null";
            }
            evidence << ",\"seed_index\":" << candidate.seed_index
                     << ",\"seed_source\":\"" << json_escape(candidate.seed_source)
                     << "\",\"seed_joint_positions\":";
            append_json_numbers(evidence, candidate.seed_positions);
            evidence << ",\"ik_attempted\":" << (candidate.ik_attempted ? "true" : "false")
                     << ",\"ik_success\":" << (candidate.ik_success ? "true" : "false")
                     << ",\"ik_elapsed_s\":";
            append_json_number(evidence, candidate.ik_elapsed_s);
            evidence << ",\"raw_ik_solution\":";
            append_json_numbers(evidence, candidate.raw_solution);
            evidence << ",\"goal_valid\":" << (candidate.goal_valid ? "true" : "false")
                     << ",\"normalized_goal\":";
            append_json_numbers(evidence, candidate.normalized_goal);
            evidence << ",\"raw_delta\":";
            append_json_numbers(evidence, candidate.raw_delta);
            evidence << ",\"wrap_aware_delta\":";
            append_json_numbers(evidence, candidate.wrap_aware_delta);
            evidence << ",\"endpoint_state_valid\":"
                     << (candidate.endpoint_state_valid ? "true" : "false")
                     << ",\"contact_bodies\":[";
            for (std::size_t contact = 0; contact < candidate.contact_bodies.size(); ++contact) {
              if (contact != 0U) {evidence << ',';}
              evidence << '\"' << json_escape(candidate.contact_bodies[contact]) << '\"';
            }
            evidence << "],\"contact_pairs\":[";
            for (std::size_t contact = 0; contact < candidate.contact_pairs.size(); ++contact) {
              if (contact != 0U) {evidence << ',';}
              evidence << "{\"body_1\":\""
                       << json_escape(candidate.contact_pairs[contact].first)
                       << "\",\"body_2\":\""
                       << json_escape(candidate.contact_pairs[contact].second) << "\"}";
            }
            evidence << "],\"valid_for_ranking\":" << (candidate.valid ? "true" : "false")
                     << ",\"duplicate_rejected\":"
                     << (candidate.duplicate_rejected ? "true" : "false")
                     << ",\"ranking_cost\":";
            append_json_number(evidence, candidate.ranking_cost);
            evidence << ",\"rejection_reason\":\""
                     << json_escape(candidate.rejection_reason) << "\"}";
          }
          evidence << "],\"candidate_priority_indices\":[";
          for (std::size_t index = 0; index < priority_candidate_indices.size(); ++index) {
            if (index != 0U) {evidence << ',';}
            evidence << priority_candidate_indices[index];
          }
          std::vector<PlaceApproachIkCandidate> evaluated_candidates;
          std::vector<std::size_t> evaluated_candidate_indices;
          for (const auto index : priority_candidate_indices) {
            if (candidates[index].ik_attempted) {
              evaluated_candidates.push_back(candidates[index]);
              evaluated_candidate_indices.push_back(index);
            }
          }
          const auto evaluated_rank = rank_place_approach_candidates(
            evaluated_candidates, measured_start, continuity_limits, {},
            ik_policy.duplicate_tolerance_rad);
          evidence << "],\"ranked_candidate_indices\":[";
          for (std::size_t index = 0; index < evaluated_rank.size(); ++index) {
            if (index != 0U) {evidence << ',';}
            evidence << evaluated_candidate_indices[evaluated_rank[index]];
          }
          evidence << "],\"planner_submission_order\":[";
          for (std::size_t index = 0; !qualify_place_target && index < result.attempts.size(); ++index) {
            if (index != 0U) {evidence << ',';}
            evidence << result.attempts[index].candidate_index;
          }
          evidence << "],\"planning_attempts\":[";
          for (std::size_t index = 0; index < result.attempts.size(); ++index) {
            if (index != 0U) {evidence << ',';}
            const auto & attempt = result.attempts[index];
            evidence << "{\"candidate_index\":" << attempt.candidate_index
                     << ",\"request_index\":" << attempt.request_index
                     << ",\"seed_index\":" << attempt.seed_index
                     << ",\"ik_attempted\":" << (attempt.ik_attempted ? "true" : "false")
                     << ",\"ik_succeeded\":" << (attempt.ik_succeeded ? "true" : "false")
                     << ",\"ik_elapsed_s\":";
            append_json_number(evidence, attempt.ik_elapsed_s);
            evidence << ",\"validation_stage\":\""
                     << json_escape(attempt.validation_stage) << "\",\"terminal\":"
                     << (attempt.terminal ? "true" : "false")
                     << ",\"planning_budget_s\":";
            append_json_number(evidence, attempt.planning_budget_s);
            evidence << ",\"planning_duration_s\":";
            append_json_number(evidence, attempt.planning_duration_s);
            evidence << ",\"planner_result_code\":";
            if (attempt.planner_invoked) {evidence << attempt.result_code;} else {evidence << "null";}
            evidence << ",\"planner_result_name\":\""
                     << (attempt.planner_invoked ? moveit_error_code_name(attempt.result_code) :
              "NOT_INVOKED") << '\"'
                     << ",\"waypoint_retryable\":"
                     << (attempt.waypoint_retryable ? "true" : "false")
                     << ",\"infrastructure_failure\":"
                     << (attempt.infrastructure_failure ? "true" : "false");
            evidence << ",\"trajectory_points\":" << attempt.trajectory_points
                     << ",\"trajectory_duration_s\":";
            append_json_number(evidence, attempt.trajectory_duration_s);
            evidence << ",\"time_parameterized\":"
                     << (attempt.time_parameterized ? "true" : "false")
                     << ",\"time_parameterization_reason\":\""
                     << json_escape(attempt.time_parameterization_reason)
                     << "\",\"target_setter_attempted\":"
                     << (attempt.target_setter_attempted ? "true" : "false")
                     << ",\"target_setter_accepted\":"
                     << (attempt.target_setter_accepted ? "true" : "false")
                     << ",\"planner_invoked\":"
                     << (attempt.planner_invoked ? "true" : "false");
            if (diagnostics_capture_enabled_ &&
              !attempt.target_state_diagnostic.group_name.empty())
            {
              const auto & target_state = attempt.target_state_diagnostic;
              evidence << ",\"target_group_validation\":{\"group_name\":\""
                       << json_escape(target_state.group_name)
                       << "\",\"group_variable_names\":[";
              for (std::size_t name_index = 0;
                name_index < target_state.group_variable_names.size(); ++name_index)
              {
                if (name_index != 0U) {evidence << ',';}
                evidence << '"' << json_escape(target_state.group_variable_names[name_index]) << '"';
              }
              evidence << "],\"group_contract_matches\":"
                       << (target_state.group_contract_matches ? "true" : "false")
                       << ",\"goal_size_matches_group\":"
                       << (target_state.goal_size_matches_group ? "true" : "false")
                       << ",\"group_bounds_satisfied\":"
                       << (target_state.group_bounds_satisfied ? "true" : "false")
                       << ",\"variables\":[";
              for (std::size_t variable_index = 0;
                variable_index < target_state.variables.size(); ++variable_index)
              {
                if (variable_index != 0U) {evidence << ',';}
                const auto & variable = target_state.variables[variable_index];
                evidence << "{\"name\":\"" << json_escape(variable.name)
                         << "\",\"value\":";
                append_json_number(evidence, variable.value);
                evidence << ",\"position_bounded\":"
                         << (variable.position_bounded ? "true" : "false")
                         << ",\"lower_bound\":";
                append_json_number(evidence, variable.lower_bound);
                evidence << ",\"upper_bound\":";
                append_json_number(evidence, variable.upper_bound);
                evidence << ",\"within_bounds\":"
                         << (variable.within_bounds ? "true" : "false") << '}';
              }
              evidence << "]}";
            }
            evidence << ",\"accepted\":" << (attempt.success ? "true" : "false") << '}';
          }
          evidence << "],\""
                   << (qualify_place_target ? "target_qualification_success" : "planning_success")
                   << "\":" << (result.success ? "true" : "false")
                   << ",\"terminal\":" << (result.terminal ? "true" : "false")
                   << ",\"elapsed_planning_s\":";
          append_json_number(evidence, result.elapsed_planning_s);
          evidence << ",\"selected_candidate_index\":";
          if (result.success) {evidence << result.selected_candidate_index;} else {
            evidence << "null";
          }
          evidence << ",\"failure_reason\":\"" << json_escape(failure_reason)
                   << "\",\"place_recovery_eligibility\":\""
                   << json_escape(recovery_eligibility) << "\"}";
          set_pending_motion_planning_evidence(
            request.motion_goal_uuid, request.task_type.value, evidence.str());
        };
      if (priority_candidate_indices.empty()) {
        {
          std::lock_guard<std::mutex> result_lock(mutex_);
          normalized_failure_result_ =
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
        }
        capture_place_plan_evaluation(
          PlaceApproachPlanResult{},
          "no_recipe_compliant_pose_and_seed_candidates_generated");
        restore_grasp_contact_policy();
        RCLCPP_WARN(node_->get_logger(),
                    "PLACE planning failed because no pose candidate passed "
                    "reachability and state validity checks");
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
          const auto candidate_index = static_cast<std::size_t>(&candidate - candidates.data());
          auto & evaluated = candidates[candidate_index];
          PlaceApproachPlanObservation observation;
          observation.validation_stage = "IK";
          const auto interrupted = [&]() {
              if (!discard_if_interrupted()) {return false;}
              evaluated.rejection_reason = motion_result_code_name(normalized_failure_result());
              observation.validation_stage = "INTERRUPTED";
              observation.terminal = true;
              return true;
            };
          const auto reject = [&](const std::string & stage, const std::string & reason) {
              evaluated.rejection_reason = reason;
              observation.validation_stage = stage;
            };
          if (interrupted()) {return observation;}
          if (!candidate.rejection_reason.empty()) {
            reject("CANDIDATE_VALIDATION", candidate.rejection_reason);
            return observation;
          }

          moveit::core::RobotState ik_seed_state(
            qualify_place_target ? planning_start_state : *current_state);
          ik_seed_state.setJointGroupPositions(joint_group, candidate.seed_positions);
          ik_seed_state.update();
          moveit::core::RobotState ik_solution(ik_seed_state);
          const auto ik_start = std::chrono::steady_clock::now();
          evaluated.ik_attempted = true;
          observation.ik_attempted = true;
          evaluated.ik_success = ik_solution.setFromIK(
            joint_group, qualify_place_target ?
            place_geometry[candidate.object_pose_candidate_index].target_pose.pose :
            place_geometry[candidate.object_pose_candidate_index].approach_pose.pose,
            "sf_grasp_tcp", ik_policy.ik_timeout_s);
          evaluated.ik_elapsed_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - ik_start).count();
          observation.ik_elapsed_s = evaluated.ik_elapsed_s;
          observation.ik_succeeded = evaluated.ik_success;
          if (interrupted()) {return observation;}
          if (!evaluated.ik_success) {
            observation.waypoint_retryable = true;
            reject("IK", "IK_FAILED");
            return observation;
          }
          ik_solution.copyJointGroupPositions(joint_group, evaluated.raw_solution);
          if (!prepare_place_approach_candidate(evaluated, measured_start, continuity_limits)) {
            reject("GRASP_VALIDATION", evaluated.rejection_reason);
            return observation;
          }

          moveit::core::RobotState endpoint_state(
            qualify_place_target ? planning_start_state : *current_state);
          endpoint_state.setJointGroupPositions(joint_group, evaluated.normalized_goal);
          endpoint_state.update();
          if (!endpoint_state.satisfiesBounds(joint_group)) {
            evaluated.endpoint_state_valid = false;
            reject("JOINT_BOUNDS", "JOINT_BOUNDS_INVALID");
            return observation;
          }
          if (interrupted()) {return observation;}
          if (!state_validity_client_ ||
            !state_validity_client_->wait_for_service(std::chrono::milliseconds(200)))
          {
            if (interrupted()) {return observation;}
            observation.infrastructure_failure = true;
            observation.terminal = true;
            reject("STATE_VALIDITY", "STATE_VALIDITY_SERVICE_UNAVAILABLE");
            return observation;
          }
          if (interrupted()) {return observation;}
          auto validity_request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
          moveit::core::robotStateToRobotStateMsg(
            endpoint_state, validity_request->robot_state, true);
          validity_request->group_name = "arm";
          auto validity_future = state_validity_client_->async_send_request(validity_request);
          const auto validity_start = std::chrono::steady_clock::now();
          while (validity_future.wait_for(std::chrono::milliseconds(50)) !=
            std::future_status::ready)
          {
            if (interrupted()) {return observation;}
            if (std::chrono::steady_clock::now() - validity_start >=
              std::chrono::milliseconds(500))
            {
              observation.infrastructure_failure = true;
              observation.terminal = true;
              reject("STATE_VALIDITY", "STATE_VALIDITY_RESPONSE_TIMEOUT");
              return observation;
            }
          }
          if (interrupted()) {return observation;}
          const auto validity = validity_future.get();
          if (!validity) {
            observation.infrastructure_failure = true;
            observation.terminal = true;
            reject("STATE_VALIDITY", "STATE_VALIDITY_RESPONSE_EMPTY");
            return observation;
          }
          evaluated.endpoint_state_valid = validity->valid;
          for (const auto & contact : validity->contacts) {
            evaluated.contact_bodies.push_back(contact.contact_body_1);
            evaluated.contact_bodies.push_back(contact.contact_body_2);
            evaluated.contact_pairs.emplace_back(
              contact.contact_body_1, contact.contact_body_2);
          }
          if (!evaluated.endpoint_state_valid) {
            observation.waypoint_retryable = true;
            reject("STATE_VALIDITY", evaluated.contact_bodies.empty() ?
              "STATE_VALIDITY_FAILED" : "COLLISION_CONTACT");
            return observation;
          }

          if (qualify_place_target) {
            observation.validation_stage = "TARGET_STATE_VALID";
            observation.success = true;
            return observation;
          }

          std::vector<PlaceApproachIkCandidate> ranked_so_far;
          for (const auto index : priority_candidate_indices) {
            if (candidates[index].ik_attempted && index != candidate_index) {
              ranked_so_far.push_back(candidates[index]);
            }
          }
          ranked_so_far.push_back(evaluated);
          (void)rank_place_approach_candidates(
            ranked_so_far, measured_start, continuity_limits, {},
            ik_policy.duplicate_tolerance_rad);
          evaluated = std::move(ranked_so_far.back());
          if (!evaluated.valid) {
            observation.waypoint_retryable = evaluated.duplicate_rejected;
            reject("GRASP_VALIDATION", evaluated.rejection_reason);
            return observation;
          }

          PlaceApproachPlanObservation::TargetStateDiagnostic target_state_diagnostic;
          const auto & group_variable_names = joint_group->getVariableNames();
          const bool group_contract_matches =
            move_group_state_->move_group->getName() == joint_group->getName() &&
            joint_group->getName() == "arm" && group_variable_names == arm_joint_names_ &&
            evaluated.normalized_goal.size() == group_variable_names.size();
          if (diagnostics_capture_enabled_) {
            target_state_diagnostic.group_name = joint_group->getName();
            target_state_diagnostic.group_variable_names = group_variable_names;
            target_state_diagnostic.group_contract_matches = group_contract_matches;
            target_state_diagnostic.goal_size_matches_group =
              evaluated.normalized_goal.size() == group_variable_names.size();
          }
          if (!group_contract_matches) {
            observation.infrastructure_failure = true;
            observation.terminal = true;
            reject("TARGET_VALIDATION", "PLANNING_TARGET_GROUP_CONTRACT_MISMATCH");
            observation.target_state_diagnostic = std::move(target_state_diagnostic);
            return observation;
          }
          moveit::core::RobotState selected_goal_state(*current_state);
          selected_goal_state.setJointGroupPositions(joint_group, evaluated.normalized_goal);
          selected_goal_state.update();
          if (diagnostics_capture_enabled_) {
            target_state_diagnostic.group_bounds_satisfied =
              selected_goal_state.satisfiesBounds(joint_group);
            for (const auto & variable_name : joint_group->getVariableNames()) {
              const auto & bounds = robot_model->getVariableBounds(variable_name);
              const auto value = selected_goal_state.getVariablePosition(variable_name);
              const bool within_bounds = !bounds.position_bounded_ ||
                (std::isfinite(value) && value >= bounds.min_position_ &&
                value <= bounds.max_position_);
              target_state_diagnostic.variables.push_back(
                {variable_name, value, bounds.position_bounded_, bounds.min_position_,
                  bounds.max_position_, within_bounds});
            }
          }
          const bool target_setter_accepted =
            move_group_state_->move_group->setJointValueTarget(evaluated.normalized_goal);
          if (!target_setter_accepted) {
            observation.infrastructure_failure = true;
            observation.terminal = true;
            reject("TARGET_VALIDATION", "PLANNING_TARGET_REJECTED");
            observation.target_setter_attempted = true;
            observation.target_setter_accepted = false;
            observation.target_state_diagnostic = std::move(target_state_diagnostic);
            return observation;
          }
          if (interrupted()) {return observation;}
          observation.validation_stage = "GLOBAL_PLANNING";
          move_group_state_->move_group->setPlanningTime(budget_s);
          const auto planning_start = std::chrono::steady_clock::now();
          const auto result = plan_with_num_attempts(
            *move_group_state_->move_group, planning_num_attempts_, [&]() {
              if (planning_scene_context_required) {
                return plan_with_request_scene_diff(
                  *move_group_state_->move_group, candidate_plan,
                  planning_request_scene_diff, [&]() {return discard_if_interrupted();});
              }
              return move_group_state_->move_group->plan(candidate_plan);
            });
          const auto planning_duration = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - planning_start).count();
          if (interrupted()) {
            observation.result_code = result.val;
            observation.planning_duration_s = planning_duration;
            observation.planner_invoked = true;
            observation.target_setter_attempted = true;
            observation.target_setter_accepted = true;
            observation.validation_stage = "SAFETY_PREEMPTED";
            return observation;
          }
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
              candidate_plans.emplace(candidate_index, std::move(candidate_plan));
            }
          }
          observation.waypoint_retryable = place_planning_failure_is_retryable(
            result.val, planning_duration, budget_s);
          observation.infrastructure_failure =
            (!planning_success && !observation.waypoint_retryable) ||
            (planning_success && !time_parameterized);
          observation.success = planning_success && time_parameterized;
          observation.result_code = result.val;
          observation.trajectory_points = trajectory_points;
          observation.planning_duration_s = planning_duration;
          observation.trajectory_duration_s = trajectory_duration_s;
          observation.time_parameterized = time_parameterized;
          observation.time_parameterization_reason = time_parameterization_reason;
          observation.target_setter_attempted = true;
          observation.target_setter_accepted = true;
          observation.planner_invoked = true;
          if (interrupted()) {
            observation.success = false;
            return observation;
          }
          observation.target_state_diagnostic = std::move(target_state_diagnostic);
          return observation;
        };
      if (qualify_place_target) {
        auto target_indices = planning_candidate_indices;
        if (bounded_place_orientation) {
          target_indices.insert(target_indices.end(),
            fallback_position_candidates.begin(), fallback_position_candidates.end());
        } else if (target_indices.size() > ik_policy.max_candidates) {
          target_indices.resize(ik_policy.max_candidates);
        }
        PlaceApproachPlanResult qualification;
        bool infrastructure_failure = false;
        bool ik_seen = false;
        for (const auto index : target_indices) {
          const auto observed = plan_candidate(candidates[index], 0.0);
          PlaceApproachPlanAttempt attempt;
          attempt.candidate_index = index;
          attempt.seed_index = candidates[index].seed_index;
          attempt.ik_attempted = observed.ik_attempted;
          attempt.ik_succeeded = observed.ik_succeeded;
          attempt.ik_elapsed_s = observed.ik_elapsed_s;
          attempt.validation_stage = observed.validation_stage;
          attempt.terminal = observed.terminal;
          attempt.infrastructure_failure = observed.infrastructure_failure;
          attempt.success = observed.success;
          attempt.time_parameterization_reason = "not_applicable_target_qualification";
          qualification.attempts.push_back(std::move(attempt));
          ik_seen = ik_seen || observed.ik_succeeded;
          infrastructure_failure = infrastructure_failure || observed.infrastructure_failure;
          if (observed.success) {
            qualification.success = true;
            qualification.selected_candidate_index = index;
            break;
          }
          if (observed.terminal) {qualification.terminal = true; break;}
        }
        capture_place_plan_evaluation(qualification, qualification.success ?
          "valid_recipe_target_candidate_found" : "no_valid_target_within_search_budget");
        if (discard_if_interrupted()) {return false;}
        if (!qualification.success) {
          std::lock_guard<std::mutex> result_lock(mutex_);
          normalized_failure_result_ = infrastructure_failure ?
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR :
            ik_seen ? arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED :
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE;
        }
        return qualification.success;
      }
      const auto place_plan_result = bounded_place_orientation ?
        plan_place_candidates_with_fallback(
        candidates, primary_bounded_candidates, fallback_position_candidates, ik_policy,
        plan_candidate,
        primary_bounded_candidates.size(),
        fallback_position_candidates.size()) :
        plan_ranked_place_approach_candidates(
        candidates, planning_candidate_indices, ik_policy, plan_candidate,
        0U);
      if (discard_if_interrupted()) {
        capture_place_plan_evaluation(place_plan_result, "safety_preempted_results_discarded");
        restore_grasp_contact_policy();
        return false;
      }
      // Only exhausted candidate-search failures before trajectory submission are eligible.
      // A candidate-local IK or endpoint collision rejection does not prove that HOME cannot
      // provide a better measured start for a fresh PLACE search.
      place_recovery_eligible_ = !place_plan_result.success && !place_plan_result.terminal &&
        !place_plan_result.attempts.empty() &&
        std::any_of(place_plan_result.attempts.begin(), place_plan_result.attempts.end(),
          [](const PlaceApproachPlanAttempt & attempt) {return attempt.waypoint_retryable;}) &&
        std::none_of(place_plan_result.attempts.begin(), place_plan_result.attempts.end(),
          [](const PlaceApproachPlanAttempt & attempt) {return attempt.infrastructure_failure;});
      const bool planner_invoked = std::any_of(
        place_plan_result.attempts.begin(), place_plan_result.attempts.end(),
        [](const auto & attempt) {return attempt.planner_invoked;});
      const bool has_infrastructure_failure = std::any_of(
        place_plan_result.attempts.begin(), place_plan_result.attempts.end(),
        [](const auto & attempt) {return attempt.infrastructure_failure;});
      const std::string recovery_eligibility = place_recovery_eligible_ ?
        (planner_invoked ? "planner_path_search_failed_or_timed_out" :
        "ik_or_endpoint_collision_search_exhausted") :
        place_plan_result.terminal ? "terminal_interruption_or_failure" :
        has_infrastructure_failure ? "infrastructure_failure" :
        place_plan_result.attempts.empty() ? "no_candidate_attempts" :
        "no_retryable_candidate_failure";
      capture_place_plan_evaluation(
        place_plan_result, place_plan_result.success ? "none" :
        "no_candidate_produced_a_valid_planned_trajectory", recovery_eligibility);
      if (!place_plan_result.success) {
        const bool planner_called = std::any_of(
          place_plan_result.attempts.begin(), place_plan_result.attempts.end(),
          [](const auto & attempt) {return attempt.planner_invoked;});
        std::lock_guard<std::mutex> result_lock(mutex_);
        const bool infrastructure_failure = std::any_of(
          place_plan_result.attempts.begin(), place_plan_result.attempts.end(),
          [](const auto & attempt) {return attempt.infrastructure_failure;});
        const bool valid_ik_seen = std::any_of(
          place_plan_result.attempts.begin(), place_plan_result.attempts.end(),
          [](const auto & attempt) {return attempt.ik_succeeded;});
        normalized_failure_result_ = infrastructure_failure ?
          arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR :
          (planner_called || valid_ik_seen) ?
          arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED :
          arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE;
      }
      planning_succeeded = place_plan_result.success;
      if (planning_succeeded) {
        place_recovery_eligible_ = false;
        const auto selected_candidate_index = place_plan_result.selected_candidate_index;
        auto selected_plan_iterator = candidate_plans.find(selected_candidate_index);
        if (selected_plan_iterator == candidate_plans.end()) {
          restore_grasp_contact_policy();
          RCLCPP_WARN(node_->get_logger(),
                      "PLACE planning failed because the selected trajectory "
                      "was not retained"
                      "");
          return false;
        }
        plan = std::move(selected_plan_iterator->second);
        const auto & selected_candidate = candidates[selected_candidate_index];
        const auto selected_pose_index = selected_candidate.object_pose_candidate_index;
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
          selected_planning_candidate_task_ = request.task_type.value;
          selected_planning_candidate_index_ = selected_pose_index;
          selected_place_object_pose_ = request.object_pose;
          selected_place_target_pose_ = request.target_pose;
          selected_place_approach_pose_ = request.approach_target;
          selected_place_retract_pose_ = request.retract_target;
          selected_place_pose_valid_ = true;
        }
        RCLCPP_DEBUG(
          node_->get_logger(),
          "PLACE approach planning found a valid trajectory among the available candidates");
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "PLACE approach planning failed because no candidate "
                    "produced a valid trajectory");
      }
    } else if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING) {
      PickPlanningDiagnostics pick_diagnostics;
      std::vector<PickCollisionContact> latest_pick_collision_contacts;
      auto & all_pick_collision_contacts = pick_diagnostics.legacy_output().all_pick_collision_contacts;
      std::size_t collision_source_candidate = 0U;
      std::size_t collision_source_seed_attempt = 0U;
      std::string state_validation_stage = "approach_candidate_state";
      if (request.task_type.value ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK)
      {
        std::lock_guard<std::mutex> collision_lock(mutex_);
        if (pick_collision_feedback_goal_uuid_ == request.motion_goal_uuid) {
          latest_pick_collision_contacts = pick_collision_feedback_;
        }
        all_pick_collision_contacts = latest_pick_collision_contacts;
      }
      PickSearchEvaluation pick_evaluation = PickSearchEvaluation::GRASP_VALIDATION_FAILED;
      enum class StateValidation {VALID, COLLISION, REJECTED, BACKEND_ERROR};
      StateValidation state_validation = StateValidation::VALID;
      const auto validate_goal_state = [&](const std::vector<double> & positions,
          std::string & reason, VisionOccupancyPolicy occupancy_policy) {
          state_validation = StateValidation::VALID;
          moveit::core::RobotState goal_state(*current_state);
          goal_state.setJointGroupPositions(joint_group, positions);
          goal_state.update();
          const auto & validation_scene = occupancy_policy == VisionOccupancyPolicy::INCLUDE ?
            grasp_validation_scene : request_validation_scene;
          if (validation_scene) {
            const auto collision = validate_pick_state_collision(
              *validation_scene, goal_state);
            if (collision.collision) {
              std::vector<PickCollisionContact> observed_contacts;
              for (const auto & contact_pair : collision.contacts) {
                observed_contacts.push_back(PickCollisionContact{
                    contact_pair.first, contact_pair.second,
                    state_validation_stage, false, 0U,
                    collision_source_candidate, collision_source_seed_attempt});
              }
              if (!observed_contacts.empty()) {
                latest_pick_collision_contacts = observed_contacts;
                all_pick_collision_contacts.insert(
                  all_pick_collision_contacts.end(), observed_contacts.begin(),
                  observed_contacts.end());
                record_pick_collision_feedback(observed_contacts);
              }
              state_validation = StateValidation::COLLISION;
              reason = "collision/contact";
              return false;
            }
            reason.clear();
            return true;
          }
          if (!state_validity_client_ ||
            !state_validity_client_->wait_for_service(std::chrono::milliseconds(200)))
          {
            state_validation = StateValidation::BACKEND_ERROR;
            reason = "state validity service unavailable";
            return false;
          }
          auto validity_request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
          moveit::core::robotStateToRobotStateMsg(goal_state, validity_request->robot_state, true);
          validity_request->group_name = "arm";
          auto validity_future = state_validity_client_->async_send_request(validity_request);
          if (validity_future.wait_for(std::chrono::milliseconds(500)) !=
            std::future_status::ready)
          {
            state_validation = StateValidation::BACKEND_ERROR;
            reason = "state validity request timed out";
            return false;
          }
          const auto validity = validity_future.get();
          if (!validity) {
            state_validation = StateValidation::BACKEND_ERROR;
            reason = "state validity response empty";
            return false;
          }
          if (!validity->valid) {
            std::vector<PickCollisionContact> observed_contacts;
            for (const auto & contact : validity->contacts) {
              observed_contacts.push_back(PickCollisionContact{
                  contact.contact_body_1, contact.contact_body_2,
                  state_validation_stage, false, 0U,
                  collision_source_candidate, collision_source_seed_attempt});
            }
            if (!observed_contacts.empty()) {
              latest_pick_collision_contacts = observed_contacts;
              all_pick_collision_contacts.insert(
                all_pick_collision_contacts.end(), observed_contacts.begin(),
                observed_contacts.end());
              record_pick_collision_feedback(observed_contacts);
            }
            state_validation = observed_contacts.empty() ?
              StateValidation::REJECTED : StateValidation::COLLISION;
            reason = observed_contacts.empty() ?
              "state validity check failed" : "collision/contact";
            return false;
          }
          reason.clear();
          return true;
        };
      const bool pick_approach = request.task_type.value ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK;
      const bool rectangle_geometry_required = request.has_observed_rectangle ||
        request.rectangle_long_side_m != 0.0F || request.rectangle_short_side_m != 0.0F ||
        request.orientation_symmetry_order ==
        arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_TWOFOLD ||
        request.orientation_symmetry_order ==
        arm_cell_interfaces::action::ExecuteTask::Goal::ORIENTATION_FOURFOLD;
      const auto rectangle_geometry_status = classify_rectangle_geometry(
        rectangle_geometry_required, request.has_estimated_object_height,
        request.rectangle_long_side_m, request.rectangle_short_side_m,
        request.estimated_object_height_m);
      const bool rectangle_geometry_available =
        rectangle_geometry_status == RectangleGeometryStatus::AVAILABLE;
      std::chrono::steady_clock::time_point pick_search_start;
      std::chrono::steady_clock::time_point candidate_search_start;
      {
        std::lock_guard<std::mutex> budget_lock(pick_search_budget_mutex_);
        candidate_search_start = pick_candidate_search_started_at_;
        pick_search_start = pick_search_started_at_ == std::chrono::steady_clock::time_point{} ?
          std::chrono::steady_clock::now() : pick_search_started_at_;
      }
      const PickSearchBudget pick_search_budget(
        pick_search_budget_s_, pick_search_start, pick_candidate_search_budget_s_,
        candidate_search_start == std::chrono::steady_clock::time_point{} ?
        pick_search_start : candidate_search_start);
      auto & pick_search_budget_exhausted = pick_diagnostics.legacy_output().pick_search_budget_exhausted;
      pick_search_budget_exhausted = false;
      auto & pick_search_termination_reason = pick_diagnostics.legacy_output().pick_search_termination_reason;
      pick_search_termination_reason = "SEARCHING";
      std::size_t starting_grasp_pose_candidate = 0U;
      if (pick_approach) {
        std::lock_guard<std::mutex> candidate_lock(mutex_);
        starting_grasp_pose_candidate = pick_grasp_pose_candidate_floor_;
      }
      auto candidate_target = request.target_pose;
      auto candidate_approach = candidate_target;
      offset_pick_approach(
        candidate_approach, request.pick_approach_axis,
        request.insertion_axis_tcp, request.approach_distance_m);
      const Eigen::Quaterniond object_orientation(
        request.object_pose.pose.orientation.w, request.object_pose.pose.orientation.x,
        request.object_pose.pose.orientation.y, request.object_pose.pose.orientation.z);
      std::vector<PickGraspPoseCandidate> grasp_pose_candidates;
      std::size_t action_grasp_pose_budget_remaining = pick_grasp_pose_candidate_limit_;
      if (pick_approach) {
        std::lock_guard<std::mutex> budget_lock(pick_search_budget_mutex_);
        action_grasp_pose_budget_remaining = pick_candidate_grasp_pose_candidates_remaining_;
      }
      if (pick_approach && rectangle_geometry_available) {
        const Eigen::Quaterniond reference_grasp_tcp_orientation(
          candidate_target.pose.orientation.w, candidate_target.pose.orientation.x,
          candidate_target.pose.orientation.y, candidate_target.pose.orientation.z);
        const Eigen::Vector3d reference_grasp_position_base(
          candidate_target.pose.position.x, candidate_target.pose.position.y,
          candidate_target.pose.position.z);
        const Eigen::Vector3d object_position_base(
          request.object_pose.pose.position.x, request.object_pose.pose.position.y,
          request.object_pose.pose.position.z);
        const Eigen::Vector3d reference_grasp_position_object =
          object_orientation.normalized().conjugate() *
          (reference_grasp_position_base - object_position_base);
        const Eigen::Vector3d insertion_axis_tcp(
          request.insertion_axis_tcp.x, request.insertion_axis_tcp.y,
          request.insertion_axis_tcp.z);
        grasp_pose_candidates = generate_pick_grasp_search_candidates(
          object_orientation, reference_grasp_tcp_orientation,
          reference_grasp_position_object, object_position_base,
          insertion_axis_tcp, request.rectangle_long_side_m,
          request.rectangle_short_side_m, request.estimated_object_height_m,
          request.grasp_width_min_mm, request.grasp_width_max_mm,
          kFullyOpenWidthMm,
          PickGraspSearchSampling{
            pick_max_withdrawal_tilt_deg_, pick_position_range_half_length_fraction_,
            pick_position_offset_samples_per_tangent_axis_, pick_orientation_samples_per_closing_axis_,
            std::min(pick_grasp_pose_candidate_limit_, action_grasp_pose_budget_remaining),
            pick_max_downward_position_offset_m_});
      } else if (pick_approach &&
        rectangle_geometry_status == RectangleGeometryStatus::UNAVAILABLE)
      {
        const Eigen::Quaterniond reference_grasp_tcp_orientation(
          candidate_target.pose.orientation.w, candidate_target.pose.orientation.x,
          candidate_target.pose.orientation.y, candidate_target.pose.orientation.z);
        if (action_grasp_pose_budget_remaining > 0U) {
          grasp_pose_candidates.push_back(
            {reference_grasp_tcp_orientation.normalized(), std::nullopt});
        }
      }
      const auto pick_order_policy =
        parse_pick_grasp_pose_order_policy(pick_grasp_pose_order_policy_);
      order_pick_grasp_search_candidates(grasp_pose_candidates, pick_order_policy);
      auto planning_context = std::make_shared<PlanningContext>();
      planning_context->operation_identity = request.motion_goal_uuid;
      planning_context->robot_state = current_state;
      planning_context->robot_model = robot_model;
      planning_context->joint_group = joint_group;
      planning_context->grasp_ik_solver = move_group_state_->pick_ik_solver;
      planning_context->move_group = move_group_state_->move_group.get();
      planning_context->scene_snapshot = scene_snapshot;
      if (planning_scene_context_required) {
        planning_context->scene_builder = std::make_unique<PlanningSceneBuilder>(
          scene_snapshot, request.non_target_occupancy, request.observation_pose.header);
      }
      planning_context->planning_budget = std::make_unique<PickSearchBudget>(pick_search_budget);
      planning_context->grasp_candidates = grasp_pose_candidates;
      planning_context->search_controller = std::make_unique<PickSearchController>(
        grasp_pose_candidates.size(), pick_ik_seed_attempts_per_grasp_pose_,
        starting_grasp_pose_candidate);
      auto & pick_search = *planning_context->search_controller;
      planning_context->expected_joint_count = measured_start.size();
      planning_context->grasp_occupancy_mode = grasp_occupancy_mode_;
      planning_context->finger_clearance = finger_clearance_;
      planning_context->non_target_occupancy = request.non_target_occupancy;
      planning_context->grasp_ik_timeout_s = pick_grasp_ik_timeout_s_;
      planning_context->approach_ik_timeout_s = pick_approach_ik_timeout_s_;
      planning_context->approach_planning_timeout_s = planning_time_s_;
      planning_context->object_orientation = object_orientation;
      planning_context->rectangle_long_side_m = request.rectangle_long_side_m;
      planning_context->rectangle_short_side_m = request.rectangle_short_side_m;
      planning_context->object_height_m = request.estimated_object_height_m;
      planning_context->grasp_width_min_mm = request.grasp_width_min_mm;
      planning_context->grasp_width_max_mm = request.grasp_width_max_mm;
      planning_context->gripper_max_width_mm = kFullyOpenWidthMm;
      planning_context->interruption_result = request.interruption_result;
      const std::size_t candidate_count = pick_approach ?
        std::max<std::size_t>(1U, grasp_pose_candidates.size()) : 1U;
      pick_diagnostics.resize(candidate_count, !pick_approach || !grasp_pose_candidates.empty());
      auto & evaluated_candidate_count = pick_diagnostics.legacy_output().evaluated_candidate_count;
      evaluated_candidate_count = 0U;
      std::vector<PickOrientationCandidate> feasible_candidates;
      auto & candidate_results = pick_diagnostics.legacy_output().candidate_results;
      auto & candidate_transition_reasons = pick_diagnostics.legacy_output().candidate_transition_reasons;
      auto & candidate_plan_status = pick_diagnostics.legacy_output().candidate_plan_status;
      auto & candidate_plan_elapsed_s = pick_diagnostics.legacy_output().candidate_plan_elapsed_s;
      auto & candidate_joint_distance = pick_diagnostics.legacy_output().candidate_joint_distance;
      auto & candidate_moveit_error_code = pick_diagnostics.legacy_output().candidate_moveit_error_code;
      auto & candidate_trajectory_points = pick_diagnostics.legacy_output().candidate_trajectory_points;
      auto & candidate_finalize_reason = pick_diagnostics.legacy_output().candidate_finalize_reason;
      auto & tcp_orientation_candidates = pick_diagnostics.legacy_output().tcp_orientation_candidates;
      auto & expected_width_candidates = pick_diagnostics.legacy_output().expected_width_candidates;
      std::string capture_reference_source{"not_selected"};
      std::string capture_reference_status{"not_attempted"};
      std::string capture_reference_failure_reason;
      std::string capture_reference_fixture_id;
      double capture_reference_contact_width_mm{0.0};
      auto & final_pick_failure_stage = pick_diagnostics.legacy_output().final_pick_failure_stage;
      final_pick_failure_stage = "IK";
      auto & final_pick_failure_reason = pick_diagnostics.legacy_output().final_pick_failure_reason;
      final_pick_failure_reason = "NO_IK_SOLUTION";
      auto & collision_rejected = pick_diagnostics.legacy_output().collision_rejected;
      collision_rejected = 0U;
      auto & ik_attempts = pick_diagnostics.legacy_output().ik_attempts;
      ik_attempts = 0U;
      auto & task_validation_attempts = pick_diagnostics.legacy_output().task_validation_attempts;
      task_validation_attempts = 0U;
      auto & geometry_rejected = pick_diagnostics.legacy_output().geometry_rejected;
      geometry_rejected = 0U;
      auto & bounds_rejected = pick_diagnostics.legacy_output().bounds_rejected;
      bounds_rejected = 0U;
      auto & fk_rejected = pick_diagnostics.legacy_output().fk_rejected;
      fk_rejected = 0U;
      auto & valid_ik_solutions = pick_diagnostics.legacy_output().valid_ik_solutions;
      valid_ik_solutions = 0U;
      std::string last_ik_rejection = "none";
      std::vector<std::vector<double>> raw_ik_solutions;
      auto & ik_callback_elapsed_s = pick_diagnostics.legacy_output().ik_callback_elapsed_s;
      ik_callback_elapsed_s = 0.0;
      auto & approach_ik_elapsed_s = pick_diagnostics.legacy_output().approach_ik_elapsed_s;
      approach_ik_elapsed_s = 0.0;
      auto & approach_ik_calls = pick_diagnostics.legacy_output().approach_ik_calls;
      approach_ik_calls = 0U;
      auto & pick_global_planning_calls = pick_diagnostics.legacy_output().pick_global_planning_calls;
      pick_global_planning_calls = 0U;
      auto & pick_global_planning_elapsed_s = pick_diagnostics.legacy_output().pick_global_planning_elapsed_s;
      pick_global_planning_elapsed_s = 0.0;
      auto & ik_validation_elapsed_s = pick_diagnostics.legacy_output().ik_validation_elapsed_s;
      ik_validation_elapsed_s = 0.0;
      auto & ik_api_elapsed_s = pick_diagnostics.legacy_output().ik_api_elapsed_s;
      ik_api_elapsed_s = 0.0;
      auto & diagnostic_selection_path = pick_diagnostics.legacy_output().diagnostic_selection_path;
      diagnostic_selection_path = "none";
      const auto ik_callback = [&](const geometry_msgs::msg::Pose &,
          const std::vector<double> & positions,
          moveit_msgs::msg::MoveItErrorCodes & error) {
          const auto callback_start = std::chrono::steady_clock::now();
          ++ik_attempts;
          if (request.interruption_result && request.interruption_result() !=
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS)
          {
            error.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
          } else if (positions.size() != measured_start.size() ||
            !std::all_of(positions.begin(), positions.end(),
            [](double joint) {return std::isfinite(joint);}))
          {
            ++bounds_rejected;
            last_ik_rejection = "nonfinite_or_joint_count";
            error.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
          } else {
            const bool duplicate = std::any_of(
              raw_ik_solutions.begin(), raw_ik_solutions.end(),
              [&](const auto & existing) {
                return existing == positions;
              });
            if (!duplicate) {raw_ik_solutions.push_back(positions);}
            error.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
          }
          ik_callback_elapsed_s += std::chrono::duration<double>(
            std::chrono::steady_clock::now() - callback_start).count();
        };
      const auto validate_ik_solution = [&](const std::vector<double> & positions) {
          ++task_validation_attempts;
          const auto expected_axis = rectangle_geometry_available &&
            pick_search.pose_index() < grasp_pose_candidates.size() &&
            grasp_pose_candidates[pick_search.pose_index()].contact ?
            std::optional<std::size_t>(grasp_pose_candidates[
              pick_search.pose_index()].contact->closing_axis) : std::nullopt;
          if (rectangle_geometry_available && !expected_axis) {
            ++geometry_rejected;
            last_ik_rejection = "grasp_pose_candidate_unavailable";
            return;
          }
          const auto & grasp_goal = candidate_approach;
          const Eigen::Quaterniond expected_grasp_orientation(
            grasp_goal.pose.orientation.w, grasp_goal.pose.orientation.x,
            grasp_goal.pose.orientation.y, grasp_goal.pose.orientation.z);
          auto grasp = validate_pick_grasp_kinematics(
            *current_state, joint_group, positions, measured_start.size(), grasp_goal,
            false);
          if (grasp.check != PickGraspKinematicCheck::VALID) {
            if (grasp.check == PickGraspKinematicCheck::JOINT_BOUNDS) {++bounds_rejected;}
            if (grasp.check == PickGraspKinematicCheck::FK_TARGET_MISMATCH) {++fk_rejected;}
            last_ik_rejection = grasp.reason;
            return;
          }
          const auto contact_validation = validate_pick_grasp_contact(
            grasp.actual_orientation, expected_axis, object_orientation,
            request.rectangle_long_side_m, request.rectangle_short_side_m,
            request.estimated_object_height_m, request.grasp_width_min_mm,
            request.grasp_width_max_mm, kFullyOpenWidthMm);
          if (contact_validation.check == PickGraspContactCheck::MISMATCH) {
            ++geometry_rejected;
            pick_evaluation = PickSearchEvaluation::GRASP_CONTACT_MISMATCH;
            last_ik_rejection = contact_validation.reason;
            return;
          }
          const auto contact = contact_validation.contact;
          std::vector<double> effective_positions = positions;

          state_validation_stage = "approach_candidate_state";

          const auto validate_approach_goal_state = [&](const std::vector<double> & positions,
              std::string & reason) {
              return validate_goal_state(
                positions, reason, VisionOccupancyPolicy::EXCLUDE);
            };
          const auto selected = select_nearest_joint_goal(
            effective_positions, measured_start, continuity_limits,
            validate_approach_goal_state);
          if (!selected.valid) {
            if (selected.reason == "collision/contact" ||
              selected.reason == "state validity check failed") {++collision_rejected;}

            last_ik_rejection = selected.reason;
            return;
          }

          ++valid_ik_solutions;
          double new_distance = 0.0;
          for (std::size_t index = 0; index < measured_start.size(); ++index) {
            const double delta = std::remainder(
              selected.selected_goal[index] - measured_start[index], 6.28318530717958647692);
            new_distance += delta * delta;
          }
          const bool duplicate = std::any_of(
            feasible_candidates.begin(), feasible_candidates.end(),
            [&](const PickOrientationCandidate & existing) {

              double difference = 0.0;
              for (std::size_t index = 0; index < selected.selected_goal.size(); ++index) {
                const double delta = std::remainder(
                  existing.joint_goal[index] - selected.selected_goal[index],
                  6.28318530717958647692);
                difference += delta * delta;
              }
              return difference < 1e-4;
            });
          if (!duplicate) {
            auto target = candidate_target;
            auto approach_pose = candidate_approach;
            if (!pick_approach) {
              target.pose.orientation = quaternion_message(grasp.actual_orientation.normalized());
              approach_pose = pick_approach_pose(
                target, request.pick_approach_axis, request.insertion_axis_tcp,
                request.approach_distance_m);
            }
            feasible_candidates.push_back(PickOrientationCandidate{
              request.object_pose, target, approach_pose, selected.selected_goal, positions,
              std::sqrt(new_distance), contact ? contact->width_mm : 0.0,
              contact ? contact->closing_axis : 0U,
              rectangle_geometry_available ? pick_search.pose_index() : 0U});
          } else {
            last_ik_rejection = "DUPLICATE_IK_RESULT";
          }
        };
      std::vector<double> raw_solution;
      moveit_msgs::msg::MoveItErrorCodes ik_error;
      auto & ik_solver_calls = pick_diagnostics.legacy_output().ik_solver_calls;
      ik_solver_calls = 0U;
      auto & pick_attempt_records = pick_diagnostics.legacy_output().pick_attempt_records;
      const auto record_interrupted_ik = [&]() {

          if (!diagnostics_capture_enabled_) {return;}
          if (pick_approach) {
            pick_search_budget_exhausted = false;
            pick_search_termination_reason = "INTERRUPTED";
          }
          std::lock_guard<std::mutex> lock(mutex_);
          if (pick_candidate_evaluation_.empty() || pick_candidate_evaluation_.back() != '}') {
            return;
          }
          pick_candidate_evaluation_.pop_back();
          std::ostringstream elapsed;
          elapsed << std::setprecision(17)
                  << ",\"ik_solver\":\"" << json_escape(pick_ik_solver_name_)
                  << "\",\"ik_solve_type\":\"" << json_escape(pick_ik_solve_type_)
                  << "\",\"ik_timeout_s\":" << pick_ik_timeout_s_
                  << ",\"grasp_ik_timeout_s\":" << pick_grasp_ik_timeout_s_
                  << ",\"approach_ik_timeout_s\":" << pick_approach_ik_timeout_s_
                  << ",\"pick_candidate_search_budget_s\":" << pick_candidate_search_budget_s_
                  << ",\"pick_search_budget_s\":" << pick_search_budget_s_
                  << ",\"ik_search_elapsed_s\":" << ik_api_elapsed_s
                  << ",\"ik_api_elapsed_s\":" << ik_api_elapsed_s
                  << ",\"ik_callback_elapsed_s\":" << ik_callback_elapsed_s
                  << ",\"ik_solver_calls\":" << ik_solver_calls
                  << ",\"approach_ik_calls\":" << approach_ik_calls
                  << ",\"approach_ik_elapsed_s\":" << approach_ik_elapsed_s
                  << ",\"candidate_search_elapsed_s\":" << pick_search_budget.elapsed_s()
                  << ",\"candidate_search_remaining_s\":" << pick_search_budget.remaining_s()
                  << ",\"candidate_search_budget_exhausted\":"
                  << (pick_search_budget_exhausted ? "true" : "false")
                  << ",\"candidate_search_termination_reason\":\""
                  << json_escape(pick_search_termination_reason) << "\""
                  << ",\"ik_attempts\":" << ik_attempts
                  << ",\"raw_ik_solutions\":" << raw_ik_solutions.size()
                  << ",\"pick_grasp_pose_candidate_limit\":" << pick_grasp_pose_candidate_limit_
                  << ",\"pick_ik_seed_attempts_per_grasp_pose\":" <<
            pick_ik_seed_attempts_per_grasp_pose_
                  << ",\"pick_attempts\":[";
          for (std::size_t index = 0; index < pick_attempt_records.size(); ++index) {
            if (index > 0U) {elapsed << ',';}
            elapsed << pick_attempt_records[index];
          }
          elapsed << "],\"ik_seed_joint_positions\":";
          append_json_numbers(elapsed, measured_start);
          elapsed << ",\"ik_goal_approach\":";
          append_json_pose(elapsed, candidate_approach);
          elapsed << '}';
          pick_candidate_evaluation_ += elapsed.str();
          set_pending_motion_planning_evidence(
            request.motion_goal_uuid, request.task_type.value, pick_candidate_evaluation_);
        };
      if (move_group_state_->pick_ik_solver && !pick_approach) {
        const auto ik_api_start = std::chrono::steady_clock::now();
        ++ik_solver_calls;
        move_group_state_->pick_ik_solver->searchPositionIK(
          candidate_approach.pose, measured_start, pick_ik_timeout_s_, raw_solution,
          ik_callback, ik_error);
        ik_api_elapsed_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - ik_api_start).count();
      }
      if (discard_if_interrupted()) {
        record_interrupted_ik();
        restore_grasp_contact_policy();
        return false;
      }
      // Distance has already spent its solver budget comparing joint distances.
      // Keep solutions from that solve and defer expensive task validity checks.
      const auto raw_distance = [&](const std::vector<double> & positions) {
          double distance = 0.0;
          for (std::size_t index = 0; index < measured_start.size(); ++index) {
            const double delta = std::remainder(
              positions[index] - measured_start[index], 6.28318530717958647692);
            distance += delta * delta;
          }
          return distance;
        };
      std::sort(raw_ik_solutions.begin(), raw_ik_solutions.end(),
        [&](const auto & lhs, const auto & rhs) {
          return raw_distance(lhs) < raw_distance(rhs);
        });
      std::size_t next_raw_solution = 0U;
      auto & ik_search_elapsed_s = pick_diagnostics.legacy_output().ik_search_elapsed_s;
      ik_search_elapsed_s = ik_api_elapsed_s;
      std::size_t validation_batches = 0U;
      auto & ik_termination_reason = pick_diagnostics.legacy_output().ik_termination_reason;
      ik_termination_reason = "searching";
      const auto refresh_candidate_diagnostics = [&]() {
          if (pick_approach) {return;}
          candidate_results.resize(candidate_count, "FEASIBLE");
          candidate_plan_status.resize(candidate_count, "NOT_ATTEMPTED");
          candidate_plan_elapsed_s.resize(candidate_count, 0.0);
          candidate_joint_distance.resize(candidate_count, std::numeric_limits<double>::infinity());
          candidate_moveit_error_code.resize(candidate_count, 0);
          candidate_trajectory_points.resize(candidate_count, 0U);
          candidate_finalize_reason.resize(candidate_count, "not_attempted");
          tcp_orientation_candidates.resize(candidate_count, Eigen::Quaterniond::Identity());
          expected_width_candidates.resize(candidate_count, 0.0);
          for (const auto & candidate : feasible_candidates) {
            const auto index = candidate.sample_index;
            if (candidate_plan_status[index] == "NOT_ATTEMPTED") {
              candidate_results[index] = "FEASIBLE";
            }
            candidate_joint_distance[index] = candidate.joint_distance;
            tcp_orientation_candidates[index] = Eigen::Quaterniond(
              candidate.target_pose.pose.orientation.w,
              candidate.target_pose.pose.orientation.x,
              candidate.target_pose.pose.orientation.y,
              candidate.target_pose.pose.orientation.z);
            expected_width_candidates[index] = candidate.expected_contact_width_mm;
          }
          if (feasible_candidates.empty() && grasp_pose_candidates.empty()) {
            candidate_results[0] = "NO_RECIPE_VALID_GRASP_POSE";
          }
        };
      const auto save_candidate_evaluation = [&](int active, int selected) {
          if (!diagnostics_capture_enabled_) {return;}
          const auto diagnostic_index = static_cast<std::size_t>(
            selected >= 0 ? selected : active >= 0 ? active : 0);
          const auto diagnostic_candidate = std::find_if(
            feasible_candidates.begin(), feasible_candidates.end(),
            [&](const PickOrientationCandidate & candidate) {
              return candidate.sample_index == diagnostic_index;
            });
          std::ostringstream record;
          record << std::setprecision(17)
                 << "{\"motion_goal_uuid\":\"" << json_escape(request.motion_goal_uuid)
                 << "\",\"planning_group\":\"arm\",\"ik_solver\":\""
                 << json_escape(pick_ik_solver_name_)
                 << "\",\"planner_id\":\"" << json_escape(
            move_group_state_->move_group->getPlannerId())
                 << "\",\"rawpart_pose\":";
          append_json_pose(record, request.object_pose);
          record << ",\"moveit_num_planning_attempts\":" << planning_num_attempts_
                 << ",\"ik_solve_type\":\"" << json_escape(pick_ik_solve_type_)
                 << "\",\"position_only_ik\":"
                 << "false"
                 << ",\"ik_timeout_s\":" << pick_ik_timeout_s_
                 << ",\"grasp_ik_timeout_s\":" << pick_grasp_ik_timeout_s_
                 << ",\"approach_ik_timeout_s\":" << pick_approach_ik_timeout_s_
                 << ",\"pick_candidate_search_budget_s\":" << pick_candidate_search_budget_s_
                 << ",\"pick_search_budget_s\":" << pick_search_budget_s_
                 << ",\"grasp_pose_candidate_limit\":" <<
            pick_grasp_pose_candidate_limit_
                 << ",\"ik_seed_attempts_per_candidate\":" <<
            pick_ik_seed_attempts_per_grasp_pose_
                 << ",\"ik_search_elapsed_s\":" << ik_search_elapsed_s
                 << ",\"candidate_search_elapsed_s\":" << std::chrono::duration<double>(
            std::chrono::steady_clock::now() - pick_search_start).count()
                 << ",\"candidate_search_remaining_s\":" << pick_search_budget.remaining_s()
                 << ",\"candidate_search_budget_exhausted\":"
                 << (pick_search_budget_exhausted ? "true" : "false")
                 << ",\"candidate_search_termination_reason\":\""
                 << json_escape(pick_search_termination_reason) << "\""
                 << ",\"pick_grasp_pose_order_policy\":\""
                 << pick_grasp_pose_order_policy_ << "\""
                 << ",\"generated_candidate_count\":" << grasp_pose_candidates.size()
                 << ",\"evaluated_candidate_count\":" << evaluated_candidate_count
                 << ",\"ik_api_elapsed_s\":" << ik_api_elapsed_s
                 << ",\"ik_callback_elapsed_s\":" << ik_callback_elapsed_s
                 << ",\"ik_validation_elapsed_s\":" << ik_validation_elapsed_s
                 << ",\"approach_ik_elapsed_s\":" << approach_ik_elapsed_s
                 << ",\"grasp_ik_call_count\":" << ik_solver_calls
                 << ",\"grasp_ik_elapsed_s\":" << ik_api_elapsed_s
                 << ",\"approach_ik_call_count\":" << approach_ik_calls
                 << ",\"approach_global_planning_call_count\":"
                 << pick_global_planning_calls
                 << ",\"approach_global_planning_elapsed_s\":"
                 << pick_global_planning_elapsed_s
                 << ",\"raw_ik_solutions\":" << raw_ik_solutions.size()
                 << ",\"raw_solutions_checked\":" << next_raw_solution
                 << ",\"ik_solver_calls\":" << ik_solver_calls
                 << ",\"validation_batches\":" << validation_batches
                 << ",\"alternate_search_used\":"
                 << (ik_solver_calls > 1U || validation_batches > 1U ? "true" : "false")
                 << ",\"ik_termination_reason\":\"" << ik_termination_reason << "\""
                 << ",\"final_failure_stage\":\""
                 << json_escape(final_pick_failure_stage) << "\",\"final_failure_reason\":\""
                 << json_escape(final_pick_failure_reason) << "\""
                 << ",\"pick_attempts\":[";
          for (std::size_t index = 0; index < pick_attempt_records.size(); ++index) {
            if (index > 0U) {record << ',';}
            record << pick_attempt_records[index];
          }
          record << "],\"ik_seed_joint_positions\":";
          append_json_numbers(record, measured_start);
          record << ",\"ik_goal_approach\":";
          append_json_pose(record,
            pick_approach && !planning_context->attempt.executed_stages.empty() ?
            pick_diagnostics.summary().candidate_approach : candidate_approach);
          record << ",\"ik_attempts\":" << ik_attempts
                 << ",\"task_validation_attempts\":" << task_validation_attempts
                 << ",\"valid_ik_solutions\":" << valid_ik_solutions
                 << ",\"rejections\":{\"geometry\":" << geometry_rejected
                 << ",\"bounds\":" << bounds_rejected
                 << ",\"fk\":" << fk_rejected
                 << ",\"collision_or_validity\":" << collision_rejected
                 << ",\"last_reason\":\"" << json_escape(last_ik_rejection)
                 << "\"},\"candidate_count\":" << grasp_pose_candidates.size()
                 << ",\"candidate_result\":\"" << json_escape(candidate_results[diagnostic_index])
                 << "\",\"joint_distance_rad\":";
          append_json_number(record, candidate_joint_distance[diagnostic_index]);
          record << ",\"selected_joint_goal\":";
          if (selected >= 0 && diagnostic_candidate != feasible_candidates.end()) {
            append_json_numbers(record, diagnostic_candidate->joint_goal);
          }
          else {record << "null";}
          record << ",\"selected_grasp_pose\":";
          if (selected >= 0 && diagnostic_candidate != feasible_candidates.end()) {
            append_json_pose(record, diagnostic_candidate->target_pose);
          } else {record << "null";}
          record << ",\"selected_grasp_joint_goal\":";
          if (selected >= 0 && diagnostic_candidate != feasible_candidates.end()) {
            append_json_numbers(record, diagnostic_candidate->grasp_joint_goal);
          } else {record << "null";}
          record << ",\"selected_approach_pose\":";
          if (selected >= 0 && diagnostic_candidate != feasible_candidates.end()) {
            append_json_pose(record, diagnostic_candidate->approach_pose);
          } else {record << "null";}
          record << ",\"selection_path\":\""
                 << (selected >= 0 ? json_escape(diagnostic_selection_path) : "none")
                 << "\"";
          record << ",\"selected_object_relative_offset_xyz\":";
          if (selected >= 0 && diagnostic_candidate != feasible_candidates.end() &&
            diagnostic_candidate->sample_index < grasp_pose_candidates.size())
          {
            const auto & offset =
              grasp_pose_candidates[diagnostic_candidate->sample_index].grasp_position_offset_object;
            record << '[' << offset.x() << ',' << offset.y() << ',' << offset.z() << ']';
          } else {record << "null";}
          record << ",\"selected_tcp_orientation_xyzw\":";
          if (selected >= 0) {
            record << '[' << tcp_orientation_candidates[diagnostic_index].x() << ','
                   << tcp_orientation_candidates[diagnostic_index].y() << ','
                   << tcp_orientation_candidates[diagnostic_index].z() << ','
                   << tcp_orientation_candidates[diagnostic_index].w() << ']';
          } else {record << "null";}
          record << ",\"face_evidence\":\""
                 << (diagnostic_candidate != feasible_candidates.end() &&
            diagnostic_candidate->closing_axis_index == 2U ?
            "observed_top_inferred_bottom" : "inferred_side_pair")
                 << "\",\"capture_reference\":"
                 << capture_reference_diagnostic_json(
            CaptureReferenceContactWidth{capture_reference_source, capture_reference_status,
              capture_reference_failure_reason, capture_reference_contact_width_mm},
            rectangle_geometry_status, capture_reference_fixture_id,
            request.has_observed_rectangle, request.has_estimated_object_height,
            request.rectangle_long_side_m, request.rectangle_short_side_m,
            request.estimated_object_height_m, request.grasp_width_min_mm,
            request.grasp_width_max_mm).dump()
                 << ",\"expected_contact_width_mm\":"
                 << expected_width_candidates[diagnostic_index]
                 << ",\"global_planning\":{\"status\":\""
                 << json_escape(candidate_plan_status[diagnostic_index])
                 << "\",\"elapsed_s\":" << candidate_plan_elapsed_s[diagnostic_index]
                 << ",\"moveit_error_code\":" << candidate_moveit_error_code[diagnostic_index]
                 << ",\"trajectory_points\":" << candidate_trajectory_points[diagnostic_index]
                 << ",\"finalization_reason\":\""
                 << json_escape(candidate_finalize_reason[diagnostic_index])
                 << "\"},\"grasp_pose_candidates\":[";
          for (std::size_t index = 0U; index < grasp_pose_candidates.size(); ++index) {
            if (index > 0U) {record << ',';}
            geometry_msgs::msg::PoseStamped grasp_pose = request.target_pose;
            grasp_pose.pose.orientation = quaternion_message(
              grasp_pose_candidates[index].tcp_orientation);
            const Eigen::Vector3d offset_base = object_orientation.normalized() *
              grasp_pose_candidates[index].grasp_position_offset_object;
            grasp_pose.pose.position.x += offset_base.x();
            grasp_pose.pose.position.y += offset_base.y();
            grasp_pose.pose.position.z += offset_base.z();
            const auto approach_pose = pick_approach_pose(
              grasp_pose, request.pick_approach_axis, request.insertion_axis_tcp,
              request.approach_distance_m);
            record << "{\"candidate\":" << index + 1U
                   << ",\"evaluation_rank\":" << index + 1U
                   << ",\"generation_index\":"
                   << grasp_pose_candidates[index].generation_index << ",\"tcp_pose\":";
            append_json_pose(record, grasp_pose);
            record << ",\"approach_pose\":";
            append_json_pose(record, approach_pose);
            record << ",\"reference_grasp_tcp_distance_m\":"
                   << grasp_pose_candidates[index].grasp_position_offset_object.norm()
                   << ",\"withdrawal_tilt_rad\":";
            if (grasp_pose_candidates[index].contact) {
              record << grasp_pose_candidates[index].withdrawal_tilt_rad;
            } else {record << "null";}
            record << ",\"grasp_position_offset_object_xyz\":["
                   << grasp_pose_candidates[index].grasp_position_offset_object.x() << ','
                   << grasp_pose_candidates[index].grasp_position_offset_object.y() << ','
                   << grasp_pose_candidates[index].grasp_position_offset_object.z() << ']';
            record << ",\"closing_axis_index\":";
            if (grasp_pose_candidates[index].contact) {
              record << grasp_pose_candidates[index].contact->closing_axis;
            } else {
              record << "null";
            }
            record << ",\"expected_contact_width_mm\":";
            if (grasp_pose_candidates[index].contact) {
              record << grasp_pose_candidates[index].contact->width_mm;
            } else {
              record << "null";
            }
            record << ",\"contact_surface\":\"";
            if (grasp_pose_candidates[index].contact) {
              record << (grasp_pose_candidates[index].contact->face_evidence ==
                BoxFaceEvidence::OBSERVED_TOP_INFERRED_BOTTOM ?
                "observed_top_inferred_bottom" : "inferred_side_pair");
            } else {
              record << "unavailable";
            }
            record << "\",\"result\":\"" << json_escape(candidate_results[index])
                   << "\",\"planning_status\":\""
                   << json_escape(candidate_plan_status[index])
                   << "\",\"transition_reason\":\""
                   << json_escape(candidate_transition_reasons[index]) << "\"}";
          }
          record << "],\"collision_feedback\":[";
          for (std::size_t index = 0U; index < all_pick_collision_contacts.size(); ++index) {
            if (index > 0U) {record << ',';}
            const auto & contact = all_pick_collision_contacts[index];
            record << "{\"body_1\":\"" << json_escape(contact.body_1)
                   << "\",\"body_2\":\"" << json_escape(contact.body_2)
                   << "\",\"stage\":\"" << json_escape(contact.stage)
                   << "\",\"source_pose_candidate\":" << contact.pose_candidate
                   << ",\"source_ik_seed_attempt\":" << contact.seed_attempt
                   << ",\"sample_index\":";
            if (contact.sample_index_available) {record << contact.sample_index;}
            else {record << "null";}
            record << '}';
          }
          record << "],\"active_candidate\":" << active
                 << ",\"selected_candidate\":" << selected << '}';
          std::lock_guard<std::mutex> result_lock(mutex_);
          auto candidate_record = nlohmann::json::parse(record.str());
          candidate_record["material_candidate_id"] = request.vision_candidate_id;
          candidate_record["vision_acquisition_id"] = request.vision_acquisition_id;
          candidate_record["vision_occupancy_evidence"] = occupancy_evidence;
          pick_candidate_evaluation_ = candidate_record.dump();
          set_pending_motion_planning_evidence(
            request.motion_goal_uuid, request.task_type.value, pick_candidate_evaluation_);
          if (planning_diagnostic_capture_) {
            planning_diagnostic_capture_->enqueue_progress(
              request.motion_goal_uuid,
              motion_task_type_name(request.task_type.value), pick_candidate_evaluation_,
              request.vision_acquisition_id);
          }
        };
      bool selected_candidate_found = false;
      double selected_joint_distance = std::numeric_limits<double>::infinity();
      double selected_duration_s = std::numeric_limits<double>::infinity();
      std::size_t selected_candidate_index = 0U;
      moveit::planning_interface::MoveGroupInterface::Plan selected_plan;
      std::vector<double> selected_goal;
      auto & planning_failed = pick_diagnostics.legacy_output().planning_failed;
      planning_failed = 0U;
      auto & trajectory_finalize_failed = pick_diagnostics.legacy_output().trajectory_finalize_failed;
      trajectory_finalize_failed = 0U;
      auto & planned_candidate_count = pick_diagnostics.legacy_output().planned_candidate_count;
      planned_candidate_count = 0U;
      auto & planning_attempted = pick_diagnostics.legacy_output().planning_attempted;
      planning_attempted = 0U;
      const auto interrupted_ik = [&]() {
          if (!request.interruption_result || request.interruption_result() ==
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS)
          {
            return false;
          }
          if (pick_approach) {

            pick_search_budget_exhausted = false;
            pick_search_termination_reason = "INTERRUPTED";
          }
          ik_search_elapsed_s = ik_api_elapsed_s + ik_validation_elapsed_s;
          refresh_candidate_diagnostics();
          save_candidate_evaluation(-1, -1);
          return discard_if_interrupted();
        };

      if (pick_approach) {
        planning_context->planning_max_requests = pick_approach_global_max_requests_;
        planning_context->planning_num_attempts = planning_num_attempts_;
        planning_context->finalize_trajectory = finalize_raw_trajectory;
        planning_context->object_pose = request.object_pose;
        planning_context->reference_grasp_goal = request.target_pose;
        planning_context->insertion_axis_tcp = request.insertion_axis_tcp;
        planning_context->approach_axis = request.pick_approach_axis;
        planning_context->approach_distance_m = request.approach_distance_m;
        planning_context->measured_start = measured_start;
        planning_context->continuity_limits = continuity_limits;
        planning_context->latest_collision_contacts = latest_pick_collision_contacts;
        planning_context->attempted_solutions.resize(grasp_pose_candidates.size());
        planning_context->record_attempt_outcome = [&pick_diagnostics, &planning_context,
            &feasible_candidates, &refresh_candidate_diagnostics, &save_candidate_evaluation,
            &record_pick_collision_feedback, this](std::size_t, std::size_t, PickSearchEvaluation, const std::string &, const std::string &) {
            pick_diagnostics.collect(planning_context->attempt, planning_context->result,
              PickPlanningDiagnosticMetadata{planning_context->search_controller->attempt_count(),
                pick_ik_seed_attempts_per_grasp_pose_, planning_context->diagnostic_evidence,
                planning_context->latest_collision_contacts});
            feasible_candidates = planning_context->valid_candidates;
            if (!planning_context->latest_collision_contacts.empty()) {
              record_pick_collision_feedback(planning_context->latest_collision_contacts);
            }
            refresh_candidate_diagnostics();
            save_candidate_evaluation(-1, -1);
          };
        BT::BehaviorTreeFactory factory;
        register_pick_approach_planning_nodes(factory);
        auto blackboard = BT::Blackboard::create();
        blackboard->set("context", planning_context);
        auto tree = factory.createTreeFromText(pick_approach_planning_tree_xml(), blackboard);
        const auto status = tree.tickRoot();
        // Motion result selection consumes execution state even if diagnostics are disabled.
        feasible_candidates = planning_context->valid_candidates;
        {
          std::lock_guard<std::mutex> lock(pick_search_budget_mutex_);
          pick_candidate_grasp_pose_candidates_remaining_ -= std::min(
            pick_candidate_grasp_pose_candidates_remaining_, planning_context->visited_poses.size());
        }
        if (discard_if_interrupted()) {restore_grasp_contact_policy(); return false;}
        pick_diagnostics.finish(planning_context->result);
        if (status == BT::NodeStatus::SUCCESS && planning_context->result.selected) {
          const auto selected = std::find_if(feasible_candidates.begin(), feasible_candidates.end(),
            [&](const auto & value) {return value.sample_index == planning_context->attempt.current_pose_index &&
              value.joint_goal == planning_context->attempt.selected_joint_goal;});
          if (selected != feasible_candidates.end()) {
            selected_candidate_found = true;
            selected_candidate_index = std::distance(feasible_candidates.begin(), selected);
            selected_joint_distance = selected->joint_distance;
            selected_goal = selected->joint_goal;
            selected_plan = std::move(planning_context->result.selected_plan);
          }
        }
        if (pick_search.status() == PickSearchStatus::BACKEND_ERROR) {
          std::lock_guard<std::mutex> lock(mutex_);
          normalized_failure_result_ = arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
          return false;
        }
        refresh_candidate_diagnostics();
        save_candidate_evaluation(-1, selected_candidate_found ?
          static_cast<int>(planning_context->attempt.current_pose_index) : -1);
      }
      while (!pick_approach && !selected_candidate_found) {
        if (interrupted_ik()) {
          restore_grasp_contact_policy();
          return false;
        }
        if (next_raw_solution == raw_ik_solutions.size()) {
          // A single Distance budget is an optimization interval, not the
          // complete PICK deadline. Permit one bounded alternate solve after
          // all ranked solutions from the first call have been exhausted.
          if (ik_solver_calls >= 2U || !move_group_state_->pick_ik_solver) {break;}
          const auto previous_count = raw_ik_solutions.size();
          const auto ik_api_start = std::chrono::steady_clock::now();
          ++ik_solver_calls;
          move_group_state_->pick_ik_solver->searchPositionIK(
            candidate_approach.pose, measured_start, pick_ik_timeout_s_, raw_solution,
            ik_callback, ik_error);
          ik_api_elapsed_s += std::chrono::duration<double>(
            std::chrono::steady_clock::now() - ik_api_start).count();
          ik_search_elapsed_s = ik_api_elapsed_s + ik_validation_elapsed_s;
          if (interrupted_ik()) {
            restore_grasp_contact_policy();
            return false;
          }
          std::sort(raw_ik_solutions.begin() + previous_count, raw_ik_solutions.end(),
            [&](const auto & lhs, const auto & rhs) {
              return raw_distance(lhs) < raw_distance(rhs);
            });
          if (raw_ik_solutions.size() == previous_count) {break;}
        }
        ++validation_batches;
        const std::size_t batch_goal = feasible_candidates.size() + 4U;
        while (next_raw_solution < raw_ik_solutions.size() &&
          feasible_candidates.size() < batch_goal)
        {
          if (interrupted_ik()) {
            restore_grasp_contact_policy();
            return false;
          }
          const auto validation_start = std::chrono::steady_clock::now();
          validate_ik_solution(raw_ik_solutions[next_raw_solution++]);
          ik_validation_elapsed_s += std::chrono::duration<double>(
            std::chrono::steady_clock::now() - validation_start).count();
        }
        ik_search_elapsed_s = ik_api_elapsed_s + ik_validation_elapsed_s;
        if (interrupted_ik()) {
          restore_grasp_contact_policy();
          return false;
        }
        refresh_candidate_diagnostics();
        std::sort(feasible_candidates.begin(), feasible_candidates.end(),
          [](const auto & lhs, const auto & rhs) {
            return lhs.joint_distance < rhs.joint_distance;
          });
        std::vector<std::size_t> planning_order;
        for (std::size_t index = 0; index < feasible_candidates.size(); ++index) {
          if (candidate_plan_status[feasible_candidates[index].sample_index] == "NOT_ATTEMPTED") {
            planning_order.push_back(index);
          }
        }
        if (planning_order.empty()) {continue;}
        const auto first_selected_rank = first_successful_candidate(
          planning_order, [&](std::size_t rank) {
            if (discard_if_interrupted()) {return true;}
            const auto & candidate = feasible_candidates[rank];
            moveit::core::RobotState candidate_goal_state(*current_state);
            candidate_goal_state.setJointGroupPositions(joint_group, candidate.joint_goal);
            candidate_goal_state.update();
            moveit::planning_interface::MoveGroupInterface::Plan candidate_plan;
            const bool target_set = move_group_state_->move_group->setJointValueTarget(
              candidate_goal_state);
            const auto candidate_index = candidate.sample_index;
            candidate_plan_status[candidate_index] = target_set ? "IN_PROGRESS" : "TARGET_REJECTED";
            candidate_finalize_reason[candidate_index] = target_set ? "not_attempted" : "target_rejected";
            if (diagnostics_capture_enabled_) {
              save_candidate_evaluation(
                target_set ? static_cast<int>(candidate_index) : -1,
                selected_candidate_found ? static_cast<int>(
                  feasible_candidates[selected_candidate_index].sample_index) : -1);
            }
            if (target_set) {
              ++planning_attempted;
            }
            const auto planning_start = std::chrono::steady_clock::now();
            const auto result = target_set ?
            (planning_scene_context_required ? plan_with_request_scene_diff(
              *move_group_state_->move_group, candidate_plan, planning_request_scene_diff,
              [&]() {return discard_if_interrupted();}) :
            move_group_state_->move_group->plan(candidate_plan)) :
            moveit::core::MoveItErrorCode::FAILURE;
            candidate_plan_elapsed_s[candidate_index] = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - planning_start).count();
            if (request.interruption_result && request.interruption_result() !=
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS)
            {
              candidate_plan_status[candidate_index] = "DISCARDED_INTERRUPTED";
              candidate_results[candidate_index] = "DISCARDED_INTERRUPTED";
              save_candidate_evaluation(static_cast<int>(candidate_index), -1);
              discard_if_interrupted();
              return true;
            }
            candidate_moveit_error_code[candidate_index] = static_cast<int>(result.val);
            candidate_trajectory_points[candidate_index] =
            candidate_plan.trajectory_.joint_trajectory.points.size();
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
            candidate_finalize_reason[candidate_index] = finalize_reason;
            candidate_plan_status[candidate_index] = !target_set ? "TARGET_REJECTED" :
            (plan_valid ? "PLANNED_FEASIBLE" :
            (result == moveit::core::MoveItErrorCode::SUCCESS ?
            "TRAJECTORY_FINALIZATION_FAILED" : "PLANNING_FAILED"));
            if (plan_valid) {
              candidate_results[candidate.sample_index] = "PLANNED_FEASIBLE";
              ++planned_candidate_count;
            }
            if (!plan_valid) {
              candidate_results[candidate.sample_index] = result ==
              moveit::core::MoveItErrorCode::SUCCESS ? "TRAJECTORY_INVALID" : "PLANNING_FAILED";
              if (result == moveit::core::MoveItErrorCode::SUCCESS &&
              !candidate_plan.trajectory_.joint_trajectory.points.empty())
              {
                ++trajectory_finalize_failed;
              } else {
                ++planning_failed;
              }
            }
            const bool better = plan_valid && (pick_approach || !selected_candidate_found ||
            candidate.joint_distance < selected_joint_distance - 1e-9 ||
            (std::abs(candidate.joint_distance - selected_joint_distance) <= 1e-9 &&
            trajectory_duration_s < selected_duration_s - 1e-9));
            if (better) {
              if (selected_candidate_found) {
                candidate_results[feasible_candidates[selected_candidate_index].sample_index] =
                "PLANNED_FEASIBLE";
              }
              candidate_results[candidate.sample_index] = "PLANNED_SELECTED";
              selected_candidate_found = true;
              selected_candidate_index = rank;
              selected_joint_distance = candidate.joint_distance;
              selected_duration_s = trajectory_duration_s;
              selected_plan = std::move(candidate_plan);
              selected_goal = candidate.joint_goal;
            }
            if (diagnostics_capture_enabled_) {
              save_candidate_evaluation(
                -1, selected_candidate_found ?
                static_cast<int>(feasible_candidates[selected_candidate_index].sample_index) : -1);
            }
            return pick_approach && plan_valid;
          });
        (void)first_selected_rank;
        if (discard_if_interrupted()) {
          restore_grasp_contact_policy();
          return false;
        }
      }
      if (!selected_candidate_found) {
        if (!pick_approach) {
          ik_termination_reason = raw_ik_solutions.empty() ? "solver_no_solution" :
            "all_ranked_solutions_rejected_or_unplannable";
        }
        if (rectangle_geometry_status == RectangleGeometryStatus::INVALID) {
          capture_reference_source = "vision_candidate_geometry";
          capture_reference_status = "failed";
          capture_reference_failure_reason = "invalid_rectangle_geometry";
        } else if (rectangle_geometry_available) {
          capture_reference_source = "vision_candidate_geometry";
          capture_reference_status = "failed";
          const bool has_policy_valid_contact = std::any_of(
            grasp_pose_candidates.begin(), grasp_pose_candidates.end(),
            [](const PickGraspPoseCandidate & candidate) {
              return candidate.contact && candidate.contact->width_mm > 0.0;
            });
          capture_reference_failure_reason = grasp_pose_candidates.empty() ?
            "no_grasp_pose_candidates" : has_policy_valid_contact ?
            "no_executable_candidate_after_robot_feasibility_checks" :
            "contact_width_policy_mismatch";
        }
        {
          std::lock_guard<std::mutex> result_lock(mutex_);
          normalized_failure_result_ = rectangle_geometry_status ==
            RectangleGeometryStatus::INVALID ?
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL :
            pick_approach &&
            !pick_action_trajectory_execution_started_ ?
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED :
            planning_attempted > 0U || collision_rejected > 0U ?
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED :
            arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE;
        }
        refresh_candidate_diagnostics();
        save_candidate_evaluation(-1, -1);
        restore_grasp_contact_policy();
        if (pick_approach) {
          RCLCPP_ERROR(
              node_->get_logger(),
              "PICK approach failed after grasp_pose_candidates=%zu/%zu "
              "final_stage=%s final_reason=%s",
              pick_search.pose_index(), grasp_pose_candidates.size(),
              final_pick_failure_stage.c_str(),
              final_pick_failure_reason.c_str());
        } else {
          RCLCPP_WARN(
              node_->get_logger(),
              "PLACE approach planning failed: no executable grasp candidate");
        }
        return false;
      }
      ik_termination_reason = "executable_candidate_selected";
      auto & selected_candidate = feasible_candidates[selected_candidate_index];
      if (pick_approach) {
        const FixtureContactWidthLookup fixture_lookup = [&](std::string & failure_reason) {
            {
              std::lock_guard<std::mutex> lock(current_fixture_object_id_mutex_);
              capture_reference_fixture_id = current_fixture_object_id_;
            }
            if (capture_reference_fixture_id.empty()) {
              failure_reason = "fixture_id_unavailable";
              return std::optional<double>{};
            }
            if (!planning_scene_client_ ||
              !planning_scene_client_->wait_for_service(std::chrono::milliseconds(500)))
            {
              failure_reason = "fixture_scene_service_unavailable";
              return std::optional<double>{};
            }
            auto query = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
            query->components.components =
              moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
            auto future = planning_scene_client_->async_send_request(query);
            if (future.wait_for(std::chrono::milliseconds(1000)) !=
              std::future_status::ready)
            {
              failure_reason = "fixture_scene_query_timeout";
              return std::optional<double>{};
            }
            const auto scene = future.get();
            if (!scene) {
              failure_reason = "fixture_scene_query_empty_response";
              return std::optional<double>{};
            }
            const auto width = fixture_contact_width_mm(
              scene->scene.world.collision_objects, capture_reference_fixture_id,
              selected_candidate.target_pose, request.grasp_width_min_mm,
              request.grasp_width_max_mm, kFullyOpenWidthMm);
            if (!width) {
              failure_reason = "fixture_geometry_or_width_policy_invalid";
            }
            return width;
          };
        const auto resolution = resolve_capture_reference_contact_width(
          rectangle_geometry_status, selected_candidate.expected_contact_width_mm,
          validation_grasp_contact_policy_enabled_, fixture_lookup);
        capture_reference_source = resolution.source;
        capture_reference_status = resolution.status;
        capture_reference_failure_reason = resolution.failure_reason;
        capture_reference_contact_width_mm = resolution.width_mm;
        selected_candidate.expected_contact_width_mm = resolution.width_mm;
        expected_width_candidates[selected_candidate.sample_index] = resolution.width_mm;
        if (resolution.source == "validation_fixture" && resolution.status == "failed") {
          RCLCPP_WARN(node_->get_logger(),
            "PICK Capture Reference fixture resolution failed: %s",
            resolution.failure_reason.c_str());
        }
      }
      save_candidate_evaluation(-1, static_cast<int>(selected_candidate.sample_index));
      request.target_pose = selected_candidate.target_pose;
      request.approach_target = selected_candidate.approach_pose;
      request.tcp_target = selected_candidate.approach_pose;
      selected_global_goal = selected_goal;
      selected_global_goal_valid = true;
      plan = std::move(selected_plan);
      planning_succeeded = true;
      planning_precomputed = true;
      if (pick_approach) {
        {
          std::lock_guard<std::mutex> orientation_lock(mutex_);
          selected_planning_candidate_task_ = request.task_type.value;
          selected_planning_candidate_index_ = selected_candidate.sample_index;
          selected_pick_ik_seed_attempt_ = pick_search.seed_attempt();
          pick_active_grasp_pose_candidate_count_ = grasp_pose_candidates.size();
          selected_pick_target_orientation_ = selected_candidate.target_pose.pose.orientation;
          selected_pick_target_orientation_valid_ = true;
          selected_pick_object_pose_ = selected_candidate.object_pose;
          selected_pick_target_pose_ = selected_candidate.target_pose;
          selected_pick_approach_pose_ = selected_candidate.approach_pose;
          selected_pick_retract_pose_ = selected_candidate.target_pose;
          offset_tcp_insertion_axis(
            selected_pick_retract_pose_, request.insertion_axis_tcp, request.retract_distance_m);
          selected_pick_pose_valid_ = true;
        }
        stage_selected_pick_grasp_relation(
          object_to_tcp_relation(
            selected_candidate.object_pose, selected_candidate.target_pose));
        if (!stage_pick_capture_reference(
            selected_candidate.target_pose, selected_candidate.expected_contact_width_mm,
            request.orientation_symmetry_order, request.motion_goal_uuid))
        {
          capture_reference_status = "failed";
          if (capture_reference_failure_reason.empty()) {
            capture_reference_failure_reason = "capture_reference_publish_failed";
          }
          save_candidate_evaluation(-1, static_cast<int>(selected_candidate.sample_index));
          {
            std::lock_guard<std::mutex> result_lock(mutex_);
            normalized_failure_result_ =
              arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
          }
          restore_grasp_contact_policy();
          return false;
        }
      }
      RCLCPP_DEBUG(
        node_->get_logger(),
        "%s approach planning selected a collision-free trajectory using joint distance, "
        "trajectory duration, and orientation deviation",
        pick_approach ? "PICK" : "PLACE");
    } else if (place_clearance) {
      const double margin = request.place_waypoint_clearance_y_margin_m;
      if (!std::isfinite(margin) || margin <= 1.0e-6) {
        restore_grasp_contact_policy();
        RCLCPP_WARN(node_->get_logger(), "PLACE WAYPOINT clearance Y margin is invalid");
        return false;
      }
    } else if (!move_group_state_->move_group->setPoseTarget(request.tcp_target, "sf_grasp_tcp")) {
      restore_grasp_contact_policy();
      RCLCPP_WARN(node_->get_logger(),
                  "motion diagnostic normalized submit rejected: MoveGroup "
                  "setPoseTarget failed"
                  "");
      return false;
    }
    if (place_clearance) {
      MoveGroupPlanningTimeRestore restore_time(*move_group_state_->move_group);
      MoveGroupPositionToleranceRestore restore_position_tolerance(
        *move_group_state_->move_group);
      MoveGroupPoseReferenceFrameRestore restore_pose_reference_frame(
        *move_group_state_->move_group);
      move_group_state_->move_group->setPoseReferenceFrame("base_link");
      // PositionTarget carries no orientation constraint. Its spherical goal
      // tolerance is bounded by the Y margin, so the goal region cannot cross
      // back inside the configured Tray boundary.
      const auto planning_start = std::chrono::steady_clock::now();
      const auto & center = request.tcp_target.pose.position;
      const double lateral_step = std::min(
        request.place_waypoint_clearance_y_margin_m, 0.10);
      const std::array<std::pair<double, double>, 5> offsets{{
        {0.0, 0.0}, {lateral_step, 0.0}, {-lateral_step, 0.0},
        {0.0, lateral_step}, {0.0, -lateral_step}}};
      for (const auto & [dx, dz] : offsets) {
        if (discard_if_interrupted()) {break;}
        const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - planning_start).count();
        const double remaining = (budget_s > 0.0 ? budget_s : planning_time_s_) - elapsed;
        if (remaining <= 0.0) {break;}
        move_group_state_->move_group->clearPoseTargets();
        if (!move_group_state_->move_group->setPositionTarget(
            center.x + dx, center.y, center.z + dz, "sf_grasp_tcp"))
        {
          continue;
        }
        move_group_state_->move_group->setGoalPositionTolerance(
          std::max(0.0, request.place_waypoint_clearance_y_margin_m - 1.0e-6));
        move_group_state_->move_group->setPlanningTime(remaining);
        plan.trajectory_.joint_trajectory.joint_names.clear();
        plan.trajectory_.joint_trajectory.points.clear();
        const auto result = plan_with_request_scene_diff(
          *move_group_state_->move_group, plan, planning_request_scene_diff,
          [&]() {return discard_if_interrupted();});
        if (result == moveit::core::MoveItErrorCode::SUCCESS &&
          !plan.trajectory_.joint_trajectory.points.empty())
        {
          const auto finalized = finalize_raw_trajectory(plan.trajectory_);
          if (finalized.success) {
            planning_succeeded = true;
            selected_final_strategy = "PLACE_WAYPOINT_CLEARANCE_GLOBAL_POSITION";
            break;
          }
        }
      }
      move_group_state_->move_group->clearPoseTargets();
    } else if (!place_global_approach && !planning_precomputed) {
      const auto result = planning_scene_context_required ?
        plan_with_request_scene_diff(
        *move_group_state_->move_group, plan, planning_request_scene_diff,
        [&]() {return discard_if_interrupted();}) :
        move_group_state_->move_group->plan(plan);
      planning_succeeded = result == moveit::core::MoveItErrorCode::SUCCESS;
      move_group_state_->move_group->clearPoseTargets();
      if (planning_succeeded) {
        const auto finalized = finalize_raw_trajectory(plan.trajectory_);
        planning_succeeded = finalized.success;
        if (!finalized.success) {
          RCLCPP_WARN(node_->get_logger(),
                      "motion diagnostic MoveGroup finalize rejected: %s"
                      "",
                      finalized.reason.c_str());
        }
      }
    }
  }
  if (!restore_grasp_contact_policy()) {
    RCLCPP_WARN(node_->get_logger(),
                "[Motion validation] grasp contact policy restore failed"
                "");
    return false;
  }
  if (discard_if_interrupted()) {return false;}
  RCLCPP_DEBUG(
    node_->get_logger(), "Motion planning %s for the %s phase",
    planning_succeeded ? "produced a valid trajectory" : "did not produce a valid trajectory",
    motion_task_phase_name(request.phase));
  if (!plan.trajectory_.joint_trajectory.points.empty() &&
    !plan.trajectory_.joint_trajectory.joint_names.empty() &&
    plan.trajectory_.joint_trajectory.joint_names != arm_joint_names_)
  {
    if (waypoint) {place_recovery_waypoint_reason_ = "trajectory_joint_name_order_mismatch";}
    RCLCPP_WARN(node_->get_logger(),
                "motion diagnostic JOINT_NAME_ORDER_MISMATCH trajectory vs "
                "configured arm order"
                "");
    if (planning_max_acceleration_rad_s2_ > 0.0) {
      return false;
    }
  }
  if (!planning_succeeded || plan.trajectory_.joint_trajectory.points.empty()) {

    return false;
  }
  if (discard_if_interrupted()) {return false;}
  if (selected_global_goal_valid) {
    continuity = normalize_joint_trajectory(
      plan.trajectory_.joint_trajectory, measured_start, continuity_limits, false);
    if (!continuity.valid) {
      if (waypoint) {place_recovery_waypoint_reason_ = continuity.reason;}
      RCLCPP_WARN(
          node_->get_logger(),
          "motion diagnostic GLOBAL APPROACH trajectory continuity rejected: %s"
          "",
          continuity.reason.c_str());
      return false;
    }
    const auto & planned_endpoint = plan.trajectory_.joint_trajectory.points.back().positions;
    for (std::size_t index = 0; index < selected_global_goal.size(); ++index) {
      const auto delta = planned_endpoint[index] - selected_global_goal[index];
      if (std::abs(delta) > 1e-3) {
        RCLCPP_WARN(node_->get_logger(),
                    "Motion rejected the approach trajectory because its "
                    "endpoint differs from the selected joint goal joint=%s"
                    "",
                    arm_joint_names_[index].c_str());
        return false;
      }
    }
  } else if (!continuity.valid) {
    continuity = normalize_joint_trajectory(
      plan.trajectory_.joint_trajectory, measured_start, continuity_limits);
    if (!continuity.valid) {
      if (waypoint) {place_recovery_waypoint_reason_ = continuity.reason;}
      RCLCPP_WARN(node_->get_logger(),
                  "motion diagnostic joint continuity validation rejected: %s"
                  "",
                  continuity.reason.c_str());
      return false;
    }
  }
  if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING &&
    (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ||
    request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE))
  {
    RCLCPP_DEBUG(
      node_->get_logger(), "%s final path selected strategy=%s",
      request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ?
      "PICK" : "PLACE", selected_final_strategy.c_str());
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_executed_path_selection_ = {
      request.motion_goal_uuid, request.phase, selected_final_strategy};
  }
  RCLCPP_DEBUG(
    node_->get_logger(),
    "Motion generated a %s trajectory for %s with %s endpoint continuity",
    primitive_type, motion_task_phase_name(request.phase),
    continuity.valid ? "valid" : "invalid");
  if (plan_output) {
    if (discard_if_interrupted()) {return false;}
    plan_output->request = request;
    plan_output->trajectory = plan.trajectory_.joint_trajectory;
    plan_output->planning_scales = planning_scales;
    plan_output->cartesian_primitive = cartesian_primitive;
    plan_output->place_recovery_waypoint = waypoint != nullptr;
    plan_output->start_state.name = arm_joint_names_;
    plan_output->start_state.position = measured_start;
    plan_output->end_state.name = arm_joint_names_;
    const auto & final_point = plan_output->trajectory.points.back();
    for (const auto & joint_name : arm_joint_names_) {
      const auto trajectory_joint = std::find(
        plan_output->trajectory.joint_names.begin(),
        plan_output->trajectory.joint_names.end(), joint_name);
      if (trajectory_joint == plan_output->trajectory.joint_names.end()) {return false;}
      const auto index = static_cast<std::size_t>(std::distance(
        plan_output->trajectory.joint_names.begin(), trajectory_joint));
      if (index >= final_point.positions.size()) {return false;}
      plan_output->end_state.position.push_back(final_point.positions[index]);
    }
    return true;
  }
  return execute_planned_trajectory(
    request, plan.trajectory_.joint_trajectory, cartesian_primitive,
    waypoint != nullptr, planning_scales, discard_if_interrupted);
}

bool IsaacRosMotionTransport::execute_planned_trajectory(
  const NormalizedMotionRequest & request,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  bool cartesian_primitive, bool place_recovery_waypoint,
  const MotionPlanningScales & planning_scales,
  const std::function<bool()> & discard_if_interrupted)
{
  const auto robot_model = move_group_state_->move_group->getRobotModel();
  if (planning_max_acceleration_rad_s2_ > 0.0) {
    if (!trajectory_sampler_configuration_error_.empty()) {
      if (place_recovery_waypoint) {
        place_recovery_waypoint_reason_ = trajectory_sampler_configuration_error_;
      }
      RCLCPP_WARN(node_->get_logger(),
                  "motion diagnostic trajectory execution rejected: %s"
                  "",
                  trajectory_sampler_configuration_error_.c_str());
      return false;
    }
    double start_simulation_time = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_simulation_time_) {
        if (place_recovery_waypoint) {
          place_recovery_waypoint_reason_ = "simulation_clock_unavailable";
        }
        RCLCPP_WARN(node_->get_logger(),
                    "motion diagnostic trajectory execution rejected: Isaac "
                    "simulation clock unavailable"
                    "");
        return false;
      }
      start_simulation_time = latest_simulation_time_;
    }
    std::vector<double> velocity_limits;
    velocity_limits.reserve(arm_joint_names_.size());
    for (const auto & joint_name : arm_joint_names_) {
      const auto * joint_model = robot_model->getJointModel(joint_name);
      if (!joint_model || joint_model->getVariableBounds().empty()) {
        RCLCPP_WARN(node_->get_logger(),
                    "motion diagnostic missing velocity bounds for %s"
                    "",
                    joint_name.c_str());
        return false;
      }
      velocity_limits.push_back(joint_model->getVariableBounds().front().max_velocity_);
    }
    const auto & points = trajectory.points;
    const auto & final_time = points.back().time_from_start;
    const auto validation = validate_time_parameterized_trajectory(
      trajectory, velocity_limits, planning_max_acceleration_rad_s2_);
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Motion trajectory validation %s: %s",
      validation.valid ? "passed" : "failed", validation.reason.c_str());
    if (!validation.valid) {
      if (place_recovery_waypoint) {place_recovery_waypoint_reason_ = validation.reason;}
      RCLCPP_WARN(node_->get_logger(),
                  "Motion rejected the executable trajectory cause=%s"
                  "",
                  validation.reason.c_str());
      return false;
    }
    const auto expected_duration =
      static_cast<double>(final_time.sec) + final_time.nanosec / 1e9;
    if (discard_if_interrupted()) {return false;}
    {
      std::lock_guard<std::mutex> lock(mutex_);
      timed_trajectory_active_ = true;
      timed_clock_callback_count_ = 0;
      timed_command_publish_attempt_count_ = 0;
      timed_command_publish_success_count_ = 0;
      trajectory_watchdog_.start(
        start_simulation_time, expected_duration, std::chrono::steady_clock::now());
    }
    trajectory_worker_->set_joint_target_accuracy_tolerance(trajectory_goal_tolerance_rad_);
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
    if (discard_if_interrupted()) {
      if (normalized_failure_result() !=
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED)
      {
        cancel();
      }
      return false;
    }
    TrajectorySampleResult worker_start;
    const bool tracks_pick_insertion = request.task_type.value ==
      arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING;
    if (tracks_pick_insertion) {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_pick_descent_trajectory_ = trajectory;
      pending_pick_descent_for_current_execution_ = true;
    }
    if (tracks_pick_insertion) {
      std::lock_guard<std::mutex> lock(mutex_);
      // A worker exception leaves submission uncertain; the Action result must
      // conservatively preserve the possibility that insertion began.
      pick_grasp_insertion_started_ = true;
    }
    if (cartesian_primitive) {
      trajectory_worker_->set_profile_trace(nullptr);
      const auto execution_id = ++next_trajectory_execution_id_;
      worker_start = trajectory_worker_->start(
        trajectory, execution_id, start_simulation_time);
    } else {
      const auto execution_id = ++next_trajectory_execution_id_;
      worker_start = start_global_profiled_trajectory(
        trajectory, execution_id,
        start_simulation_time, active_limits.str(), request.motion_goal_uuid);
    }
    if (!worker_start.valid) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (tracks_pick_insertion) {pick_grasp_insertion_started_ = false;}
      timed_trajectory_active_ = false;
      if (place_recovery_waypoint) {place_recovery_waypoint_reason_ = worker_start.reason;}
      RCLCPP_WARN(node_->get_logger(),
                  "motion diagnostic trajectory worker rejected plan: %s"
                  "",
                  worker_start.reason.c_str());
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (request.task_type.value ==
        arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK)
      {
        pick_action_trajectory_execution_started_ = true;
      }
      execution_active_ = true;
      stop_requested_ = false;
      inactivity_confirmed_ = false;
      completion_outcome_ = MotionCompletionOutcome::COMPLETED;
      settled_feedback_count_ = 0;
    }
    // StopMotion may race with worker start after the pre-start check. The
    // owning Action's interruption remains latched even after Safety is SAFE.
    if (discard_if_interrupted()) {
      if (normalized_failure_result() ==
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
        // StopMotion can race trajectory start; this hold overrides only the
        // command submitted after the first stop dispatch.
        reassert_hold_after_raced_motion_command();
      } else {
        cancel();
      }
      return false;
    }
    RCLCPP_DEBUG(node_->get_logger(), "Motion trajectory execution began");
    return true;
  }
  const auto & final_positions = trajectory.points.back().positions;
  if (discard_if_interrupted()) {return false;}
  const bool tracks_pick_insertion = request.task_type.value ==
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
    request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING;
  if (tracks_pick_insertion) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_pick_descent_trajectory_ = trajectory;
    pending_pick_descent_for_current_execution_ = true;
  }
  if (tracks_pick_insertion) {
    std::lock_guard<std::mutex> lock(mutex_);
    pick_grasp_insertion_started_ = true;
  }
  const bool published = publish_joint_target(final_positions);
  if (published && request.task_type.value ==
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pick_action_trajectory_execution_started_ = true;
  }
  if (!published && tracks_pick_insertion) {
    std::lock_guard<std::mutex> lock(mutex_);
    pick_grasp_insertion_started_ = false;
  }
  if (discard_if_interrupted()) {
    if (normalized_failure_result() ==
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
      // StopMotion can race the direct command; this hold overrides only that
      // later command and preserves the original stop confirmation contract.
      reassert_hold_after_raced_motion_command();
    } else {
      cancel();
    }
    return false;
  }
  return published;
}

void IsaacRosMotionTransport::record_task_accuracy(
  const NormalizedMotionRequest & request, uint8_t execution_result)
{
  try {
    using TaskType = arm_cell_interfaces::msg::MotionTaskType;
    using Phase = arm_cell_interfaces::msg::MotionTaskPhase;
    const bool pick = request.task_type.value == TaskType::TASK_TYPE_PICK;
    const bool approach = request.phase == Phase::TASK_PHASE_APPROACHING;
    const bool retract = request.phase == Phase::TASK_PHASE_RETRACTING;
    const bool process_target = !approach && !retract;
    const char * segment =
      retract ? "RETRACT" : (approach ? "APPROACH" : (pick ? "GRASP" : "PLACE"));
    geometry_msgs::msg::PoseStamped expected = request.tcp_target;
    std::size_t selected_candidate = 0U;
    std::string selected_strategy;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      selected_candidate = selected_planning_candidate_index_ + (pick ? 0U : 1U);
      selected_strategy = diagnostic_path_strategy(last_executed_path_selection_, request);
      if (pick && selected_pick_pose_valid_) {
        expected = retract ? selected_pick_retract_pose_ :
          (approach ? selected_pick_approach_pose_ : selected_pick_target_pose_);
      } else if (!pick && selected_place_pose_valid_) {
        expected = retract ? selected_place_retract_pose_ :
          (approach ? selected_place_approach_pose_ : selected_place_target_pose_);
      }
    }
    std::shared_ptr<moveit::core::RobotState> measured;
    std::string unavailable_reason;
    std::vector<double> fresh_positions;
    std::vector<double> fresh_velocities;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool state_fresh = latest_joint_state_ &&
        std::chrono::steady_clock::now() - latest_state_receipt_ <= kStateFreshness &&
        extract_arm_positions(*latest_joint_state_, fresh_positions);
      if (!state_fresh) {unavailable_reason = "measured_joint_feedback_stale_or_missing";} else {
        fresh_velocities = extract_arm_velocities(*latest_joint_state_);
      }
    }
    {
      if (unavailable_reason.empty()) {
        std::lock_guard<std::mutex> lock(move_group_state_->mutex);
        if (!move_group_state_->move_group) {
          unavailable_reason = "move_group_unavailable";
        } else {
          const auto robot_model = move_group_state_->move_group->getRobotModel();
          if (!robot_model) {
            unavailable_reason = "fresh_robot_state_unavailable";
          } else {
            measured = std::make_shared<moveit::core::RobotState>(robot_model);
            measured->setToDefaultValues();
            measured->setVariablePositions(arm_joint_names_, fresh_positions);
            measured->update();
          }
        }
      }
    }
    nlohmann::json accuracy = {
      {"segment", segment}, {"motion_goal_uuid", request.motion_goal_uuid},
      {"execution_result", motion_result_code_name(execution_result)},
      {"target_tcp_pose", nlohmann::json::object()},
      {"actual_tcp_pose", nlohmann::json::object()}, {"tcp_position_error_mm", nullptr},
      {"tcp_orientation_error_rad", nullptr}, {"tcp_orientation_error_unit", "rad"},
      {"recipe_tolerance", {
          {"position_m", !process_target ? nlohmann::json(nullptr) : nlohmann::json(
              pick ? request.pick_grasp_position_tolerance_m : request.place_position_tolerance_m)},
          {"orientation_rad", (!process_target || (!pick && request.place_orientation_constraint ==
            PlaceOrientationConstraint::FREE)) ? nlohmann::json(nullptr) : nlohmann::json(
              pick ? request.pick_grasp_orientation_tolerance_rad :
              request.place_orientation_tolerance_rad)}}},
      {"within_recipe_tolerance", nullptr},
      {"selected_planning_candidate_index", selected_candidate},
      {"path_strategy", selected_strategy.empty() ? "UNAVAILABLE" : selected_strategy},
      {"measurement_status", "UNAVAILABLE"}};
    if (planning_max_acceleration_rad_s2_ > 0.0) {
      const auto trajectory_diagnostics = trajectory_worker_->diagnostics();
      nlohmann::json final_joint_position_errors = nullptr;
      nlohmann::json final_joint_position_error_max_rad = nullptr;
      if (fresh_positions.size() == trajectory_diagnostics.final_target.size() &&
        fresh_positions.size() == arm_joint_names_.size())
      {
        const auto joint_accuracy = measure_joint_position_accuracy(
          arm_joint_names_, trajectory_diagnostics.final_target, fresh_positions);
        if (joint_accuracy) {
          final_joint_position_errors = nlohmann::json::array();
          for (const auto & joint : joint_accuracy->joints) {
            final_joint_position_errors.push_back({
              {"joint", joint.name},
              {"target_position_rad", joint.target_position_rad},
              {"actual_position_rad", joint.actual_position_rad},
              {"position_error_rad", joint.position_error_rad}});
          }
          final_joint_position_error_max_rad = joint_accuracy->maximum_position_error_rad;
        }
      }
      const auto optional_number = [](const std::optional<double> & value) {
          return value ? nlohmann::json(*value) : nlohmann::json(nullptr);
        };
      const bool joint_accuracy_available = !final_joint_position_error_max_rad.is_null();
      accuracy["trajectory_execution"] = {
        {"final_joint_position_errors", final_joint_position_errors},
        {"final_joint_position_error_max_rad", final_joint_position_error_max_rad},
        {"goal_accuracy_tolerance_rad", trajectory_goal_tolerance_rad_},
        {"goal_accuracy", !joint_accuracy_available ? "UNAVAILABLE" :
          (final_joint_position_error_max_rad.get<double>() <= trajectory_goal_tolerance_rad_ ?
          "WITHIN_TOLERANCE" : "EXCEEDED_TOLERANCE")},
        {"actual_joint_positions_rad", fresh_positions.empty() ? nlohmann::json(nullptr) :
          nlohmann::json(fresh_positions)},
        {"actual_joint_velocities_rad_s", fresh_velocities.empty() ? nlohmann::json(nullptr) :
          nlohmann::json(fresh_velocities)},
        {"actual_joint_delta_rad",
          (fresh_positions.empty() ||
          trajectory_diagnostics.actual_delta.empty()) ? nlohmann::json(nullptr) :
          nlohmann::json(trajectory_diagnostics.actual_delta)},
        {"planned_duration_s", trajectory_diagnostics.planned_duration_s},
        {"final_command_attempt_simulation_time", optional_number(
            trajectory_diagnostics.final_command_attempt_simulation_time)},
        {"final_command_attempt_steady_time_s", optional_number(
            trajectory_diagnostics.final_command_attempt_steady_time_s)},
        {"final_command_accepted_steady_time_s", optional_number(
            trajectory_diagnostics.final_command_accepted_steady_time_s)},
        {"trajectory_execution_complete", trajectory_diagnostics.trajectory_execution_complete},
        {"terminal_reason", trajectory_diagnostics.terminal_reason}};
    }
    if (!process_target) {
      accuracy["recipe_tolerance_unavailable_reason"] =
        "recipe_does_not_define_segment_tcp_accuracy_tolerance";
    } else if (!pick && request.place_orientation_constraint == PlaceOrientationConstraint::FREE) {
      accuracy["recipe_orientation_tolerance_status"] = "NOT_CONSTRAINED_BY_RECIPE";
    }
    accuracy["target_tcp_pose"] = {
      {"frame_id", expected.header.frame_id},
      {"position_m", {expected.pose.position.x, expected.pose.position.y,
          expected.pose.position.z}},
      {"orientation_xyzw", {expected.pose.orientation.x, expected.pose.orientation.y,
          expected.pose.orientation.z, expected.pose.orientation.w}}};
    if (!pick && process_target) {
      accuracy["recipe_nominal_object_pose"] = {
        {"frame_id", request.object_pose.header.frame_id},
        {"position_m", {request.object_pose.pose.position.x,
            request.object_pose.pose.position.y, request.object_pose.pose.position.z}},
        {"orientation_xyzw", {request.object_pose.pose.orientation.x,
            request.object_pose.pose.orientation.y, request.object_pose.pose.orientation.z,
            request.object_pose.pose.orientation.w}}};
      accuracy["recipe_nominal_tcp_pose"] = {
        {"frame_id", request.tcp_target.header.frame_id},
        {"position_m", {request.tcp_target.pose.position.x,
            request.tcp_target.pose.position.y, request.tcp_target.pose.position.z}},
        {"orientation_xyzw", {request.tcp_target.pose.orientation.x,
            request.tcp_target.pose.orientation.y, request.tcp_target.pose.orientation.z,
            request.tcp_target.pose.orientation.w}}};
      accuracy["accuracy_basis"] = "recipe_nominal_tcp_pose";
    } else if (pick && !approach) {
      accuracy["accuracy_basis"] = "selected_grasp_tcp_pose";
    } else {
      accuracy["accuracy_basis"] = "approach_tcp_target";
    }
    if (expected.header.frame_id.empty()) {
      unavailable_reason = "selected_target_frame_unavailable";
    }
    if (measured && unavailable_reason.empty()) {
      const auto target_to_model = measured->getFrameTransform(expected.header.frame_id);
      const auto transform = target_to_model.inverse() *
        measured->getGlobalLinkTransform("sf_grasp_tcp");
      const auto position = transform.translation();
      const Eigen::Quaterniond orientation(transform.rotation());
      if (position.allFinite() && orientation.coeffs().allFinite()) {
        const Eigen::Vector3d target_position(
          expected.pose.position.x, expected.pose.position.y, expected.pose.position.z);
        Eigen::Quaterniond target_orientation(
          expected.pose.orientation.w, expected.pose.orientation.x,
          expected.pose.orientation.y, expected.pose.orientation.z);
        if (target_orientation.coeffs().allFinite() && target_orientation.norm() > 1e-12) {
          target_orientation.normalize();
          Eigen::Isometry3d selected_target = Eigen::Isometry3d::Identity();
          selected_target.translation() = target_position;
          selected_target.linear() = target_orientation.toRotationMatrix();
          const auto selected_error = measure_tcp_pose_accuracy(selected_target, transform);
          const double position_error_m = selected_error.position_error_m;
          const double orientation_error_rad = selected_error.orientation_error_rad;
          double recipe_position_error_m = position_error_m;
          double recipe_orientation_error_rad = orientation_error_rad;
          bool recipe_goal_valid = true;
          if (!pick && process_target) {
            const Eigen::Vector3d recipe_position(
              request.tcp_target.pose.position.x, request.tcp_target.pose.position.y,
              request.tcp_target.pose.position.z);
            Eigen::Quaterniond recipe_orientation(
              request.tcp_target.pose.orientation.w, request.tcp_target.pose.orientation.x,
              request.tcp_target.pose.orientation.y, request.tcp_target.pose.orientation.z);
            if (!recipe_position.allFinite() || !recipe_orientation.coeffs().allFinite() ||
              recipe_orientation.norm() <= 1e-12)
            {
              recipe_goal_valid = false;
              unavailable_reason = "recipe_nominal_tcp_pose_invalid";
            } else {
              recipe_orientation.normalize();
              Eigen::Isometry3d nominal_target = Eigen::Isometry3d::Identity();
              nominal_target.translation() = recipe_position;
              nominal_target.linear() = recipe_orientation.toRotationMatrix();
              const auto nominal_error = measure_tcp_pose_accuracy(nominal_target, transform);
              recipe_position_error_m = nominal_error.position_error_m;
              recipe_orientation_error_rad = nominal_error.orientation_error_rad;
              if ((recipe_position - target_position).norm() > 1e-12 ||
                std::abs(recipe_orientation.dot(target_orientation)) < 1.0 - 1e-12)
              {
                accuracy["selected_goal_error"] = {
                  {"position_mm", position_error_m * 1000.0},
                  {"orientation_rad", orientation_error_rad}};
              }
            }
          }
          const double position_tolerance = pick ? request.pick_grasp_position_tolerance_m :
            request.place_position_tolerance_m;
          const double orientation_tolerance = pick ? request.pick_grasp_orientation_tolerance_rad :
            request.place_orientation_tolerance_rad;
          const bool orientation_constrained = pick || request.place_orientation_constraint !=
            PlaceOrientationConstraint::FREE;
          const bool tolerance_defined = process_target && recipe_goal_valid &&
            std::isfinite(position_tolerance) && position_tolerance >= 0.0 &&
            (!orientation_constrained || (std::isfinite(orientation_tolerance) &&
            orientation_tolerance >= 0.0));
          accuracy["actual_tcp_pose"] = {
            {"frame_id", expected.header.frame_id},
            {"source", "moveit_robot_model_fk_from_measured_joint_state"},
            {"position_m", {position.x(), position.y(), position.z()}},
            {"orientation_xyzw", {orientation.x(), orientation.y(), orientation.z(),
                orientation.w()}}};
          accuracy["tcp_position_error_mm"] = recipe_position_error_m * 1000.0;
          accuracy["tcp_orientation_error_rad"] = recipe_orientation_error_rad;
          accuracy["within_recipe_tolerance"] = tolerance_defined ?
            nlohmann::json(recipe_position_error_m <= position_tolerance &&
              (!orientation_constrained ||
              recipe_orientation_error_rad <= orientation_tolerance)) : nlohmann::json(nullptr);
          accuracy["measurement_status"] = "AVAILABLE";
          if (!tolerance_defined) {
            accuracy["tolerance_status"] = "UNAVAILABLE";
            accuracy["tolerance_unavailable_reason"] = approach ?
              "recipe_tolerance_not_defined_for_approach_goal" :
              (recipe_goal_valid ? "recipe_tolerance_not_defined_for_segment" : unavailable_reason);
          }
          if (process_target) {
            log_task_accuracy_summary(
              request, execution_result, recipe_position_error_m * 1000.0,
              recipe_orientation_error_rad);
          }
        } else {
          unavailable_reason = "target_orientation_invalid";
        }
      } else {
        unavailable_reason = "fk_returned_non_finite_tcp_pose";
      }
    }
    if (accuracy["measurement_status"] == "UNAVAILABLE") {
      accuracy["unavailable_reason"] = unavailable_reason.empty() ?
        "robot_model_or_actual_joint_feedback_unavailable" : unavailable_reason;
      RCLCPP_DEBUG(
        node_->get_logger(), "Motion TCP accuracy measurement unavailable segment=%s",
        segment);
    }
    if (diagnostics_capture_enabled_) {
      const auto serialized_accuracy = accuracy.dump();
      {
        std::lock_guard<std::mutex> lock(planning_diagnostic_mutex_);
        if (task_accuracy_goal_uuid_ != request.motion_goal_uuid) {
          task_accuracy_goal_uuid_ = request.motion_goal_uuid;
          task_accuracy_history_.clear();
        }
        task_accuracy_history_.push_back(serialized_accuracy);
        task_accuracy_task_type_ = request.task_type.value;
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_DEBUG(
      node_->get_logger(), "Motion TCP accuracy diagnostics dropped after %s",
      error.what());
  } catch (...) {
    RCLCPP_DEBUG(node_->get_logger(), "Motion TCP accuracy diagnostics dropped");
  }
}

std::string IsaacRosMotionTransport::diagnostic_path_strategy(
  const ExecutedPathSelection & selection, const NormalizedMotionRequest & request)
{
  if (selection.motion_goal_uuid != request.motion_goal_uuid ||
    selection.phase != request.phase || selection.strategy.empty())
  {
    return "UNAVAILABLE";
  }
  return selection.strategy;
}

void IsaacRosMotionTransport::log_task_accuracy_summary(
  const NormalizedMotionRequest & request, uint8_t execution_result,
  double position_error_mm, double orientation_error_rad) const
{
  if (execution_result !=
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS)
  {
    return;
  }
  const bool pick = request.task_type.value ==
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK;
  const bool place = request.task_type.value ==
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE;
  if (!pick && !place) {
    return;
  }
  const double position_tolerance_m = pick ? request.pick_grasp_position_tolerance_m :
    request.place_position_tolerance_m;
  const bool orientation_constrained = pick || request.place_orientation_constraint !=
    PlaceOrientationConstraint::FREE;
  const double orientation_tolerance_rad = pick ? request.pick_grasp_orientation_tolerance_rad :
    request.place_orientation_tolerance_rad;
  if (!std::isfinite(position_tolerance_m) || position_tolerance_m < 0.0 ||
    (orientation_constrained && (!std::isfinite(orientation_tolerance_rad) ||
    orientation_tolerance_rad < 0.0)))
  {
    return;
  }
  const bool outside_position_tolerance = position_error_mm > position_tolerance_m * 1000.0;
  const bool outside_orientation_tolerance = orientation_constrained &&
    orientation_error_rad > orientation_tolerance_rad;
  if (!outside_position_tolerance && !outside_orientation_tolerance) {
    return;
  }
  if (pick) {
    RCLCPP_WARN(
      node_->get_logger(), "PICK grasp completed outside Recipe accuracy tolerance "
      "tcp_position_error_mm=%.3f"
      " tcp_orientation_error_deg=%.3f", position_error_mm,
      orientation_error_rad * 180.0 / 3.14159265358979323846);
  } else {
    RCLCPP_WARN(
      node_->get_logger(), "PLACE completed outside Recipe accuracy tolerance "
      "tcp_position_error_mm=%.3f"
      " tcp_orientation_error_deg=%.3f", position_error_mm,
      orientation_error_rad * 180.0 / 3.14159265358979323846);
  }
}

MotionCompletionOutcome IsaacRosMotionTransport::wait_for_motion_completion()
{
  return wait_for_motion_completion_until(std::chrono::steady_clock::time_point::max());
}

MotionCompletionOutcome IsaacRosMotionTransport::wait_for_motion_completion_until(
  std::chrono::steady_clock::time_point deadline)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const auto feedback_wait_started_at = std::chrono::steady_clock::now();
  std::vector<double> initial_feedback_positions;
  const bool feedback_available_at_start = latest_joint_state_ &&
    extract_arm_positions(*latest_joint_state_, initial_feedback_positions);
  auto feedback_gap_started_at = feedback_available_at_start ? latest_state_receipt_ :
    feedback_wait_started_at;
  auto next_command_republish = std::chrono::steady_clock::now() +
    kMotionCommandRepublishPeriod;
  bool feedback_gap_warned = false;
  while (true) {
    const auto now = std::chrono::steady_clock::now();
    if (stop_requested_) {
      finish_global_profile_capture("trajectory_interrupted", false);
      motion_progress_watchdog_active_ = false;
      return completion_outcome_;
    }
    if (timed_trajectory_active_ && trajectory_worker_ &&
      trajectory_worker_->completion_eligible())
    {
      if (pending_pick_descent_for_current_execution_) {
        reusable_pick_descent_trajectory_ = pending_pick_descent_trajectory_;
        pending_pick_descent_trajectory_.points.clear();
        pending_pick_descent_for_current_execution_ = false;
      }
      execution_active_ = false;
      timed_trajectory_active_ = false;
      motion_progress_watchdog_active_ = false;
      log_motion_state_locked("TRAJECTORY_COMPLETED");
      return MotionCompletionOutcome::COMPLETED;
    }
    if (now >= deadline) {
      if (trajectory_worker_) {trajectory_worker_->cancel("place_waypoint_execution_timeout");}
      timed_trajectory_active_ = false;
      motion_progress_watchdog_active_ = false;
      finish_global_profile_capture("place_waypoint_execution_timeout", false);
      lock.unlock();
      publish_current_hold_target(MotionCompletionOutcome::TIMEOUT);
      return MotionCompletionOutcome::TIMEOUT;
    }
    if (trajectory_worker_ && trajectory_worker_->failed()) {
      const auto diagnostics = trajectory_worker_->diagnostics();
      RCLCPP_WARN(node_->get_logger(),
                  "Motion trajectory execution failed before final target "
                  "completion reason=%s",
                  trajectory_worker_->failure_reason().c_str());
      finish_global_profile_capture("trajectory_execution_failed", false);
      motion_progress_watchdog_active_ = false;
      return MotionCompletionOutcome::TRANSPORT_UNAVAILABLE;
    }
    if (timed_trajectory_active_) {
      const auto trajectory_status = trajectory_watchdog_.evaluate(
        latest_simulation_time_, now);
      if (clock_diagnostic_capture_) {
        ClockDiagnosticCapture::Sample clock_sample;
        clock_sample.clock_time_ns = static_cast<int64_t>(latest_simulation_time_ * 1e9);
        const auto node_ros_now = node_->now();
        clock_sample.node_ros_time_ns = node_ros_now.nanoseconds();
        clock_sample.node_ros_time_available = true;
        clock_sample.node_ros_time_sim_active = node_->get_clock()->ros_time_is_active();
        clock_sample.receipt_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          latest_simulation_clock_receipt_.time_since_epoch()).count();
        clock_sample.decision_steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          now.time_since_epoch()).count();
        if (latest_simulation_clock_receipt_ != std::chrono::steady_clock::time_point{} &&
          now >= latest_simulation_clock_receipt_)
        {
          clock_sample.receipt_age_s = std::chrono::duration<double>(
            now - latest_simulation_clock_receipt_).count();
          clock_sample.receipt_age_available = true;
        }
        if (node_->get_clock()->ros_time_is_active()) {
          clock_sample.source_age_ns = node_ros_now.nanoseconds() - clock_sample.clock_time_ns;
          clock_sample.source_age_available = true;
        }
        clock_sample.watchdog_threshold_s = trajectory_watchdog_.liveness_timeout_s();
        clock_sample.existing_execution_id = std::to_string(
          trajectory_worker_->diagnostics().execution_id);
        clock_sample.watchdog_result = trajectory_status ==
          TrajectoryWatchdogStatus::SIMULATION_CLOCK_STALL ? "SIMULATION_CLOCK_STALL" : "RUNNING";
        clock_sample.observer = node_->get_name();
        clock_diagnostic_capture_->record_decision(clock_sample);
      }
      if (trajectory_status == TrajectoryWatchdogStatus::SIMULATION_CLOCK_STALL) {
        const auto diagnostics = trajectory_worker_->diagnostics();
        RCLCPP_WARN(
          node_->get_logger(), "Motion trajectory execution failed: simulation clock stalled");
        trajectory_worker_->cancel("simulation_clock_stall");
        timed_trajectory_active_ = false;
        finish_global_profile_capture("trajectory_watchdog_timeout", false);
        motion_progress_watchdog_active_ = false;
        lock.unlock();
        publish_current_hold_target(MotionCompletionOutcome::TIMEOUT);
        return MotionCompletionOutcome::TIMEOUT;
      }
    }
    if (!joint_command_publisher_ || joint_command_publisher_->get_subscription_count() == 0) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "Motion completion wait ended because command transport is "
        "unavailable: no /joint_commands subscriber");
      if (trajectory_worker_) {trajectory_worker_->cancel("command_transport_unavailable");}
      timed_trajectory_active_ = false;
      finish_global_profile_capture("trajectory_command_transport_unavailable", false);
      motion_progress_watchdog_active_ = false;
      lock.unlock();
      publish_current_hold_target(MotionCompletionOutcome::TRANSPORT_UNAVAILABLE);
      return MotionCompletionOutcome::TRANSPORT_UNAVAILABLE;
    }
    const bool motion_target_reached = !timed_trajectory_active_ &&
      measured_target_reached_locked();
    std::vector<double> current_feedback_positions;
    const bool feedback_fresh = latest_joint_state_ &&
      now - latest_state_receipt_ <= kStateFreshness &&
      extract_arm_positions(*latest_joint_state_, current_feedback_positions);
    const bool new_feedback_observed = feedback_fresh &&
      latest_state_receipt_ > feedback_gap_started_at;
    const auto feedback_gap_duration = new_feedback_observed ?
      latest_state_receipt_ - feedback_gap_started_at : now - feedback_gap_started_at;
    const bool feedback_gap_threshold_reached = feedback_gap_duration >= kStateFreshness;
    const bool feedback_gap_recovered = new_feedback_observed &&
      (feedback_gap_warned || feedback_gap_threshold_reached);
    if (feedback_gap_threshold_reached && !feedback_gap_warned) {
      const auto phase = last_executed_path_selection_.phase !=
        arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_UNKNOWN ?
        motion_task_phase_name(last_executed_path_selection_.phase) :
        motion_task_type_name(active_motion_task_type_);
      RCLCPP_WARN(
        node_->get_logger(),
        "Motion joint feedback observation gap elapsed_ms=%lld task_type=%s phase=%s",
        static_cast<long long>(
          std::chrono::duration_cast<std::chrono::milliseconds>(
          feedback_gap_duration).count()),
        motion_task_type_name(active_motion_task_type_), phase);
      feedback_gap_warned = true;
    }
    if (new_feedback_observed) {
      if (feedback_gap_recovered && motion_progress_watchdog_active_) {
        motion_progress_watchdog_.start(current_feedback_positions, latest_state_receipt_);
      }
      feedback_gap_started_at = latest_state_receipt_;
      feedback_gap_warned = false;
    }
    if (feedback_fresh && motion_target_reached &&
      joint_state_generation_ > motion_command_generation_ &&
      !execution_active_ && inactivity_confirmed_)
    {
      if (pending_pick_descent_for_current_execution_) {
        reusable_pick_descent_trajectory_ = pending_pick_descent_trajectory_;
        pending_pick_descent_trajectory_.points.clear();
        pending_pick_descent_for_current_execution_ = false;
      }
      log_motion_state_locked("COMPLETED");
      timed_trajectory_active_ = false;
      motion_progress_watchdog_active_ = false;
      return MotionCompletionOutcome::COMPLETED;
    }
    if (execution_active_ && (!trajectory_worker_ || !trajectory_worker_->active()) &&
      now >= next_command_republish)
    {
      republish_joint_target_locked();
      next_command_republish = now + kMotionCommandRepublishPeriod;
    }
    const auto freshness_deadline = feedback_fresh ?
      latest_state_receipt_ + kStateFreshness : feedback_gap_started_at + kStateFreshness;
    if (motion_progress_watchdog_active_ &&
      feedback_fresh &&
      (timed_trajectory_active_ || execution_active_) &&
      motion_progress_watchdog_.stalled(now))
    {
      std::string failure_reason = "JOINT_PROGRESS_STALLED";
      if (timed_trajectory_active_ && trajectory_worker_) {
        RCLCPP_WARN(
          node_->get_logger(), "Motion trajectory execution failed: joint progress stalled");
        trajectory_worker_->cancel("joint_progress_stalled");
      } else {
        RCLCPP_WARN(node_->get_logger(), "Motion execution timed out reason=%s", failure_reason.c_str());
      }
      log_motion_state_locked(failure_reason);
      finish_global_profile_capture("motion_progress_stalled", false);
      motion_progress_watchdog_active_ = false;
      timed_trajectory_active_ = false;
      lock.unlock();
      publish_current_hold_target(MotionCompletionOutcome::JOINT_PROGRESS_STALLED);
      std::unique_lock<std::mutex> stop_lock(mutex_);
      const auto stop_observation_deadline = std::chrono::steady_clock::now() +
        2 * kStopObservationWindow;
      joint_state_condition_.wait_until(
        stop_lock, stop_observation_deadline, [this]() {return inactivity_confirmed_;});
      // Deliver the terminal Action result after MotionStatus can report the
      // confirmed physical stop. If feedback cannot confirm it, retain the
      // stall result and let Orchestration fail its stop-confirmation check.
      return completion_outcome_;
    }
    const auto progress_deadline = timed_trajectory_active_ ?
      trajectory_watchdog_.next_liveness_deadline() :
      (motion_progress_watchdog_active_ && feedback_fresh ?
      motion_progress_watchdog_.next_deadline() : now + kMotionCommandRepublishPeriod);
    const auto feedback_check_deadline = now < freshness_deadline ?
      freshness_deadline : now + kMotionCommandRepublishPeriod;
    joint_state_condition_.wait_until(
      lock, std::min({progress_deadline, feedback_check_deadline, next_command_republish}));
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
  command.velocity.assign(arm_joint_names_.size(), 0.0);
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
  if (!timed_trajectory_active_ || stop_requested_ || !trajectory_worker_ ||
    !trajectory_worker_->active() ||
    sample.execution_id != trajectory_worker_->diagnostics().execution_id)
  {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Motion rejected a timed command because trajectory execution is inactive");
    return false;
  }
  const auto published = publish_joint_target_locked(sample.positions, &sample.velocities);
  if (published) {
    ++timed_command_publish_success_count_;
  }
  return published;
}

bool IsaacRosMotionTransport::publish_joint_target_locked(
  const std::vector<double> & positions,
  const std::vector<double> * velocities,
  std::string * diagnostic_rejection_reason)
{
  const auto reject = [diagnostic_rejection_reason](const char * reason) {
      if (diagnostic_rejection_reason) {*diagnostic_rejection_reason = reason;}
      return false;
    };
  if (!latest_joint_state_) {
    RCLCPP_DEBUG(node_->get_logger(), "motion diagnostic publish rejected: no latest joint state");
    return reject("joint_state_unavailable");
  }
  if (positions.size() != arm_joint_names_.size()) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion diagnostic publish rejected because target does not match the configured arm joint set");
    return reject("target_joint_count_mismatch");
  }
  if (positions.size() != home_joint_positions_.size()) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion diagnostic publish rejected because target does not match the HOME joint set");
    return reject("home_joint_count_mismatch");
  }
  if (std::any_of(
      positions.begin(), positions.end(), [](double value) {return !std::isfinite(value);}))
  {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "motion diagnostic publish rejected: non-finite joint target");
    return reject("non_finite_joint_target");
  }
  if (velocities != nullptr) {
    if (velocities->size() != arm_joint_names_.size() || std::any_of(
        velocities->begin(), velocities->end(), [](double value) {return !std::isfinite(value);}))
    {
      RCLCPP_DEBUG(
        node_->get_logger(), "motion diagnostic publish rejected: invalid velocity target");
      return reject("invalid_velocity_target");
    }
  }
  if (joint_command_publisher_->get_subscription_count() == 0) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Cannot publish /joint_commands: no compatible Isaac command subscriber is connected");
    return reject("joint_command_subscriber_unavailable");
  }
  if (!motion_progress_watchdog_active_) {
    std::vector<double> measured_positions;
    if (!extract_arm_positions(*latest_joint_state_, measured_positions)) {
      RCLCPP_DEBUG(node_->get_logger(), "motion diagnostic cannot initialize progress watchdog");
      return reject("measured_start_state_unavailable");
    }
    motion_progress_watchdog_.start(measured_positions, std::chrono::steady_clock::now());
    motion_progress_watchdog_active_ = true;
  }
  sensor_msgs::msg::JointState command;
  command.header.stamp = node_->now();
  command.name = arm_joint_names_;
  command.position = positions;
  command.velocity = velocities != nullptr ? *velocities :
    std::vector<double>(arm_joint_names_.size(), 0.0);
  joint_command_publisher_->publish(command);
  target_positions_ = positions;
  motion_command_generation_ = joint_state_generation_;
  completion_outcome_ = MotionCompletionOutcome::COMPLETED;
  execution_active_ = true;
  stop_requested_ = false;
  inactivity_confirmed_ = false;
  settled_feedback_count_ = 0;
  stop_command_published_ = false;
  return true;
}

bool IsaacRosMotionTransport::publish_current_hold_target(
  MotionCompletionOutcome interruption)
{
  std::lock_guard<std::mutex> lock(mutex_);
  completion_outcome_ = interruption;
  stop_requested_ = true;
  stop_command_published_ = false;
  joint_state_condition_.notify_all();
  gripper_status_condition_.notify_all();
  std::vector<double> measured_positions;
  if (!latest_joint_state_ ||
    std::chrono::steady_clock::now() - latest_state_receipt_ > kStateFreshness ||
    !extract_arm_positions(*latest_joint_state_, measured_positions))
  {
    inactivity_confirmed_ = false;
    const char * reason = !latest_joint_state_ ? "joint_state_missing" :
      std::chrono::steady_clock::now() - latest_state_receipt_ > kStateFreshness ?
      "joint_state_stale" : "arm_joint_positions_unavailable";
    RCLCPP_WARN(
      node_->get_logger(), "Motion cannot publish a stop hold command cause=%s", reason);
    return false;
  }
  if (timed_trajectory_active_ ||
    (trajectory_worker_ && trajectory_worker_->active()))
  {
    inactivity_confirmed_ = false;
    execution_active_ = true;
    RCLCPP_WARN(
      node_->get_logger(),
      "Motion cannot publish a stop hold while trajectory command production is active");
    return false;
  }
  if (joint_command_publisher_->get_subscription_count() == 0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Motion cannot publish a stop hold command cause=joint_command_subscriber_unavailable");
    inactivity_confirmed_ = false;
    return false;
  }
  sensor_msgs::msg::JointState command;
  command.header.stamp = node_->now();
  command.name = arm_joint_names_;
  command.position = measured_positions;
  command.velocity.assign(arm_joint_names_.size(), 0.0);
  joint_command_publisher_->publish(command);
  target_positions_ = measured_positions;
  // The stop caller has cancelled the trajectory worker and disabled timed
  // command publication before reaching this lock. Successful Hold publication
  // completes Motion's software stop; joint feedback does not gate this state.
  execution_active_ = false;
  inactivity_confirmed_ = true;
  settled_feedback_count_ = 0;
  stop_observation_diagnostic_ = "stop_hold_delivered";
  stop_command_published_ = true;
  stop_command_generation_ = joint_state_generation_;
  RCLCPP_DEBUG(
    node_->get_logger(),
    "Motion published the stop hold command after cancelling prior command production");
  joint_state_condition_.notify_all();
  return true;
}

void IsaacRosMotionTransport::reassert_hold_after_raced_motion_command()
{
  if (trajectory_worker_) {
    trajectory_worker_->cancel();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_requested_ && stop_command_published_) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "Motion kept the existing stop hold after the raced trajectory command was cancelled");
      return;
    }
    timed_trajectory_active_ = false;
    finish_global_profile_capture("trajectory_safety_preempted", false);
  }
  const bool hold_published =
    publish_current_hold_target(MotionCompletionOutcome::SAFETY_PREEMPTED);
  RCLCPP_DEBUG(
    node_->get_logger(),
    "Motion reasserted its accepted Safety hold after a raced command was cancelled "
    "hold_command_published=%s",
    hold_published ? "true" : "false");
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
  const auto message_stamp_ns =
    static_cast<std::int64_t>(message->header.stamp.sec) * 1'000'000'000LL +
    message->header.stamp.nanosec;
  std::int64_t previous_feedback_stamp_ns = 0;
  if (stop_requested_ && latest_joint_state_) {
    previous_feedback_stamp_ns =
      static_cast<std::int64_t>(latest_joint_state_->header.stamp.sec) * 1'000'000'000LL +
      latest_joint_state_->header.stamp.nanosec;
    if (message_stamp_ns <= previous_feedback_stamp_ns) {
      return;
    }
  }
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
  if (motion_progress_watchdog_active_ &&
    (!timed_trajectory_active_ || (trajectory_worker_ && trajectory_worker_->active())))
  {
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
  }

  const bool at_target = std::equal(
    target_positions_.begin(), target_positions_.end(), measured_positions.begin(),
    [this](double target, double actual) {
      return std::abs(target - actual) <= trajectory_goal_tolerance_rad_;
    });
  if (!stop_requested_ && execution_active_ && !timed_trajectory_active_) {
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
  }
  joint_state_condition_.notify_all();
}

void IsaacRosMotionTransport::on_simulation_clock(
  const rosgraph_msgs::msg::Clock::ConstSharedPtr message)
{
  auto receipt_time = std::chrono::steady_clock::time_point{};
  const auto simulation_time = static_cast<double>(message->clock.sec) +
    static_cast<double>(message->clock.nanosec) / 1e9;
  ClockDiagnosticCapture::Sample diagnostic_sample;
  bool diagnostic_sample_available = false;
  if (clock_diagnostic_capture_) {
    receipt_time = std::chrono::steady_clock::now();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_simulation_time_ = simulation_time;
    if (clock_diagnostic_capture_) {
      latest_simulation_clock_receipt_ = receipt_time;
    }
    has_simulation_time_ = true;
    if (timed_trajectory_active_ && trajectory_worker_ && trajectory_worker_->active()) {
      ++timed_clock_callback_count_;
      trajectory_watchdog_.observe(
        simulation_time, clock_diagnostic_capture_ ? receipt_time :
        std::chrono::steady_clock::now());
    }
  }
  if (clock_diagnostic_capture_) {
    auto & sample = diagnostic_sample;
    sample.clock_time_ns = rclcpp::Time(message->clock).nanoseconds();
    const auto node_ros_now = node_->now();
    sample.node_ros_time_ns = node_ros_now.nanoseconds();
    sample.node_ros_time_available = true;
    sample.node_ros_time_sim_active = node_->get_clock()->ros_time_is_active();
    sample.receipt_steady_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      receipt_time.time_since_epoch())
      .count();
    sample.observer = node_->get_name();
    if (sample.node_ros_time_sim_active) {
      sample.source_age_ns = sample.node_ros_time_ns - sample.clock_time_ns;
      sample.source_age_available = true;
    }
    diagnostic_sample_available = true;
  }
  if (diagnostic_sample_available) {
    clock_diagnostic_capture_->observe(diagnostic_sample);
  }
  if (!trajectory_worker_ || !trajectory_worker_->active()) {
    joint_state_condition_.notify_all();
    return;
  }
  const auto execution_id = trajectory_worker_->diagnostics().execution_id;
  const auto result = trajectory_worker_->on_simulation_time(simulation_time);
  if (!result.valid) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (trajectory_worker_->diagnostics().execution_id != execution_id) {return;}
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
    RCLCPP_WARN(
      node_->get_logger(), "motion diagnostic full trajectory execution failed: %s",
      result.reason.c_str());
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (timed_trajectory_active_ && !stop_requested_ &&
      trajectory_worker_->diagnostics().execution_id == execution_id &&
      trajectory_worker_->completion_eligible())
    {
      execution_active_ = false;
      // Normal command execution completion is not stop/inactivity evidence.
      inactivity_confirmed_ = false;
      motion_progress_watchdog_active_ = false;
      finish_global_profile_capture("trajectory_execution_complete", true);
    }
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
    std::string recovered_rejection_reason;
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
        const auto reference = fields.find("capture_reference_id");
        const std::string reference_id = reference == fields.end() ? "" : reference->second;
        if (!capture_reference_id_.empty() && reference_id == capture_reference_id_) {
          applied_capture_reference_id_ = reference_id;
          gripper_status_condition_.notify_all();
        }
        recovered_rejection_reason = last_gripper_status_rejection_reason_;
        last_gripper_status_rejection_reason_.clear();
      }
    }
    if (!recovered_rejection_reason.empty()) {
      RCLCPP_DEBUG(
        node_->get_logger(), "Gripper status observations recovered previous_rejection_reason=%s",
        recovered_rejection_reason.c_str());
    }
    if (semantic_state_changed) {
      RCLCPP_DEBUG(
        node_->get_logger(),
        "Gripper state changed ready=%s holding=%s object_attached=%s released=%s",
        ready ? "true" : "false", grasp ? "true" : "false",
        attached ? "true" : "false", released ? "true" : "false");
    } else if (rejection_reason) {
      log_rejection(rejection_reason, sequence, current_sequence);
    }
  } catch (const std::exception &) {
    log_rejection("malformed_payload");
    return;
  }
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
    [this](double target, double actual) {
      return std::abs(target - actual) <= trajectory_goal_tolerance_rad_;
    });
}

void IsaacRosMotionTransport::log_motion_state_locked(const std::string & event) const
{
  RCLCPP_DEBUG(node_->get_logger(),
               "Motion state changed event=%s active=%s backend_inactive=%s",
               event.c_str(), execution_active_ ? "true" : "false",
               inactivity_confirmed_ ? "true" : "false");
}

void IsaacRosMotionTransport::cancel()
{
  if (trajectory_worker_) {
    trajectory_worker_->cancel();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timed_trajectory_active_ = false;
    finish_global_profile_capture("trajectory_cancelled", false);
  }
  publish_current_hold_target(MotionCompletionOutcome::CANCELED);
}

void IsaacRosMotionTransport::safety_preempt(uint8_t stop_mode)
{
  stop_gripper();
  if (trajectory_worker_) {
    trajectory_worker_->cancel("safety_preempted");
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    timed_trajectory_active_ = false;
    finish_global_profile_capture("trajectory_safety_preempted", false);
  }
  if (stop_mode == arm_cell_interfaces::msg::StopMode::STOP_MODE_CONTROLLED ||
    stop_mode == arm_cell_interfaces::msg::StopMode::STOP_MODE_IMMEDIATE ||
    stop_mode == arm_cell_interfaces::msg::StopMode::STOP_MODE_EMERGENCY)
  {
    const bool hold_published =
      publish_current_hold_target(MotionCompletionOutcome::SAFETY_PREEMPTED);
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Motion Safety stop dispatched mode=%u trajectory_worker_cancelled=%s "
      "hold_command_published=%s",
      static_cast<unsigned>(stop_mode),
      !trajectory_worker_ || !trajectory_worker_->active() ? "true" : "false",
      hold_published ? "true" : "false");
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

std::string IsaacRosMotionTransport::inactivity_diagnostic() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool joint_progress_stall =
    completion_outcome_ == MotionCompletionOutcome::JOINT_PROGRESS_STALLED;
  const auto report = [this, joint_progress_stall](const std::string & reason) {
      if (!joint_progress_stall) {return reason;}
      const bool worker_cancelled = !trajectory_worker_ || !trajectory_worker_->active();
      return std::string("joint_progress_stalled;trajectory_worker_cancelled=") +
             (worker_cancelled ? "true" : "false") + ";hold_command_published=" +
             (stop_command_published_ || inactivity_confirmed_ ? "true" : "false") +
             ";" + reason;
    };
  if (!execution_active_ && inactivity_confirmed_) {
    return report(stop_observation_diagnostic_.empty() ?
      "stop_hold_delivered" : stop_observation_diagnostic_);
  }
  if (!stop_requested_) {
    return report("stop_dispatch_pending");
  }
  if (trajectory_worker_ && trajectory_worker_->active()) {
    return report("trajectory_worker_active");
  }
  if (!stop_command_published_) {
    if (!latest_joint_state_) {
      return report("stop_hold_not_published:joint_state_missing");
    }
    if (std::chrono::steady_clock::now() - latest_state_receipt_ > kStateFreshness) {
      return report("stop_hold_not_published:joint_state_stale");
    }
    std::vector<double> measured_positions;
    if (!extract_arm_positions(*latest_joint_state_, measured_positions)) {
      return report("stop_hold_not_published:arm_joint_positions_unavailable");
    }
    if (!joint_command_publisher_ || joint_command_publisher_->get_subscription_count() == 0) {
      return report("stop_hold_not_published:joint_command_subscriber_unavailable");
    }
    return report("stop_hold_not_published");
  }
  if (!latest_joint_state_ ||
    std::chrono::steady_clock::now() - latest_state_receipt_ > kStateFreshness)
  {
    return report(latest_joint_state_ ? "joint_state_stale" : "joint_state_missing");
  }
  if (joint_state_generation_ <= stop_command_generation_) {
    return report("awaiting_post_stop_joint_feedback");
  }
  return report(stop_observation_diagnostic_.empty() ?
         "stop_hold_not_confirmed" :
         stop_observation_diagnostic_);
}

bool IsaacRosMotionTransport::measured_joint_state(sensor_msgs::msg::JointState & state) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_joint_state_) {
    return false;
  }
  state = *latest_joint_state_;
  return true;
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
    node_->get_logger(), "Motion issued a logical gripper opening command");
  gripper_publisher_->publish(command);
  std::lock_guard<std::mutex> lock(mutex_);
  gripper_active_ = true;
  return true;
}

bool IsaacRosMotionTransport::command_gripper_close()
{
  if (!available()) {
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
  if (!gripper_close_publisher_) {
    return false;
  }
  std::string reference_id;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    reference_id = capture_reference_id_;
    if (!reference_id.empty()) {
      if (!gripper_status_condition_.wait_for(
          lock, std::chrono::seconds(2), [this, &reference_id]() {
            return stop_requested_ || applied_capture_reference_id_ == reference_id;
          }) || stop_requested_ || applied_capture_reference_id_ != reference_id)
      {
        return false;
      }
    }
  }
  std_msgs::msg::String command;
  command.data = reference_id;
  gripper_close_publisher_->publish(command);
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
  observation.width_mm = gripper_width_mm_;
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
    node_->get_logger(), "Motion stopped the gripper by holding its current opening");
  gripper_publisher_->publish(command);
  gripper_active_ = false;
}
}  // namespace arm_cell_motion
