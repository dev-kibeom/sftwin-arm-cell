#include "arm_cell_motion/isaac_ros_motion_transport.hpp"

#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>
#include <arm_cell_interfaces/msg/motion_task_phase.hpp>
#include <arm_cell_interfaces/msg/motion_task_type.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>
#include <array>
#include <cmath>

namespace arm_cell_motion
{

bool IsaacRosMotionTransport::submit_normalized(const NormalizedMotionRequest & request)
{
  NormalizedTrajectoryPlan plan;
  if (plan_normalized(request, plan) !=
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS)
  {
    return false;
  }
  return execute_planned(plan);
}

uint8_t IsaacRosMotionTransport::plan_normalized(
  const NormalizedMotionRequest & request, NormalizedTrajectoryPlan & plan)
{
  if (request.place_waypoint_approach) {
    std::optional<NormalizedTrajectoryPlan> cached;
    {
      std::lock_guard<std::mutex> lock(place_waypoint_cache_mutex_);
      cached = cached_place_waypoint_approach_;
    }
    if (!cached) {
      {
        std::lock_guard<std::mutex> state_lock(mutex_);
        last_place_failure_stage_ = "place_approach_planning";
      }
      RCLCPP_ERROR(node_->get_logger(),
        "PLACE WAYPOINT approach rejected: no valid pre-generated trajectory is loaded");
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED;
    }
    const auto matches_pose = [](const geometry_msgs::msg::PoseStamped & pose,
        const std::vector<double> & signature) {
        const std::array<double, 7> values{
          pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
          pose.pose.orientation.x, pose.pose.orientation.y,
          pose.pose.orientation.z, pose.pose.orientation.w};
        if (signature.size() != values.size()) {return false;}
        for (std::size_t index = 0; index < values.size(); ++index) {
          if (!std::isfinite(values[index]) || std::abs(values[index] - signature[index]) > 1.0e-6) {
            return false;
          }
        }
        return true;
      };
    if (!matches_pose(request.object_pose, place_waypoint_object_target_signature_) ||
      !matches_pose(request.approach_target, place_waypoint_tcp_target_signature_))
    {
      {
        std::lock_guard<std::mutex> state_lock(mutex_);
        last_place_failure_stage_ = "place_approach_planning";
      }
      RCLCPP_ERROR(node_->get_logger(),
        "PLACE WAYPOINT approach rejected: loaded trajectory target does not match profile");
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
    }
    plan = std::move(*cached);
    plan.request = request;
    {
      std::lock_guard<std::mutex> state_lock(mutex_);
      selected_place_object_pose_ = request.object_pose;
      selected_place_target_pose_ = request.target_pose;
      selected_place_approach_pose_ = request.approach_target;
      selected_place_retract_pose_ = request.retract_target;
      selected_place_pose_valid_ = true;
    }
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }
  const auto stage_prefix = [&request]() {
      if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING) {
        return std::string("place_approach");
      }
      if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING) {
        return std::string("place_retreat");
      }
      return std::string("place_insertion");
    };
  if (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_place_failure_stage_ = stage_prefix() + "_planning";
    if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING &&
      (place_retreat_planning_requests_ == 0U ||
      place_retreat_goal_uuid_ != request.motion_goal_uuid))
    {
      place_retreat_goal_uuid_ = request.motion_goal_uuid;
      place_retreat_planning_requests_ = 1U;
    }
  }
  if (!submit_normalized_once(request, nullptr, 0.0, false, &plan)) {
    return normalized_failure_result();
  }
  return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
}

uint8_t IsaacRosMotionTransport::generate_place_waypoint_approach(
  const NormalizedMotionRequest & input_request, const std::string & path)
{
  using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
  if (path.empty()) {return Code::TASK_RESULT_INVALID_GOAL;}
  if (std::filesystem::exists(path)) {return Code::TASK_RESULT_INVALID_GOAL;}
  if (home_joint_positions_.size() != arm_joint_names_.size()) {
    return Code::TASK_RESULT_INVALID_GOAL;
  }
  auto request = input_request;
  request.place_waypoint_approach = true;
  request.has_start_joint_state = true;
  request.start_joint_state.name = arm_joint_names_;
  request.start_joint_state.position = home_joint_positions_;

  const auto started = std::chrono::steady_clock::now();
  std::optional<NormalizedTrajectoryPlan> selected;
  double selected_duration = std::numeric_limits<double>::infinity();
  double selected_joint_distance = std::numeric_limits<double>::infinity();
  std::size_t candidate_count = 0U;
  std::size_t selected_candidate_index = 0U;
  constexpr std::size_t kCandidateBudget = 4U;
  for (std::size_t index = 0; index < kCandidateBudget; ++index) {
    NormalizedTrajectoryPlan candidate;
    if (!submit_normalized_once(
        request, nullptr, place_candidate_planning_time_s_, false, &candidate))
    {
      continue;
    }
    if (candidate.trajectory.points.size() < 2U) {continue;}
    const auto & last = candidate.trajectory.points.back();
    const double duration = static_cast<double>(last.time_from_start.sec) +
      static_cast<double>(last.time_from_start.nanosec) * 1e-9;
    if (!std::isfinite(duration) || duration <= 0.0) {continue;}
    double joint_distance = 0.0;
    for (std::size_t point = 1U; point < candidate.trajectory.points.size(); ++point) {
      const auto & before = candidate.trajectory.points[point - 1U].positions;
      const auto & after = candidate.trajectory.points[point].positions;
      if (before.size() != after.size()) {
        joint_distance = std::numeric_limits<double>::infinity(); break;
      }
      for (std::size_t joint = 0; joint < before.size(); ++joint) {
        if (!std::isfinite(before[joint]) || !std::isfinite(after[joint])) {
          joint_distance = std::numeric_limits<double>::infinity(); break;
        }
        joint_distance += std::abs(after[joint] - before[joint]);
      }
    }
    if (!std::isfinite(joint_distance)) {continue;}
    ++candidate_count;
    if (duration < selected_duration ||
      (duration == selected_duration && joint_distance < selected_joint_distance))
    {
      selected_duration = duration;
      selected_joint_distance = joint_distance;
      selected_candidate_index = index + 1U;
      selected = std::move(candidate);
    }
  }
  const double elapsed_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  {
    std::lock_guard<std::mutex> state_lock(mutex_);
    selected_place_pose_valid_ = false;
  }
  if (!selected) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "PLACE WAYPOINT HOME-to-PLACE precompute failed candidates=%zu elapsed_s=%.3f",
      candidate_count, elapsed_s);
    return normalized_failure_result();
  }
  nlohmann::json artifact;
  artifact["schema_version"] = 1;
  artifact["robot_model"] = "m0609_robotiq_2f85";
  artifact["planning_group"] = "arm";
  artifact["planning_config"] = {
    {"planner_id", planning_planner_id_},
    {"velocity_scaling", planning_velocity_scaling_factor_},
    {"acceleration_scaling", planning_acceleration_scaling_factor_},
    {"planned_velocity_scale", selected->planning_scales.max_velocity},
    {"planned_acceleration_scale", selected->planning_scales.max_acceleration}};
  artifact["joint_names"] = selected->trajectory.joint_names;
  artifact["home_joint_positions"] = home_joint_positions_;
  artifact["object_target_pose"] = {
    request.object_pose.pose.position.x, request.object_pose.pose.position.y,
    request.object_pose.pose.position.z, request.object_pose.pose.orientation.x,
    request.object_pose.pose.orientation.y, request.object_pose.pose.orientation.z,
    request.object_pose.pose.orientation.w};
  artifact["approach_tcp_pose"] = {
    request.approach_target.pose.position.x, request.approach_target.pose.position.y,
    request.approach_target.pose.position.z, request.approach_target.pose.orientation.x,
    request.approach_target.pose.orientation.y, request.approach_target.pose.orientation.z,
    request.approach_target.pose.orientation.w};
  artifact["candidate_budget"] = kCandidateBudget;
  artifact["valid_candidate_count"] = candidate_count;
  artifact["selected_candidate_index"] = selected_candidate_index;
  artifact["selection_criteria"] = "execution_duration_then_joint_path_l1";
  artifact["selected_duration_s"] = selected_duration;
  artifact["selected_joint_path_l1_rad"] = selected_joint_distance;
  artifact["generation_time_s"] = elapsed_s;
  artifact["points"] = nlohmann::json::array();
  for (const auto & point : selected->trajectory.points) {
    artifact["points"].push_back({
      {"positions", point.positions}, {"velocities", point.velocities},
      {"accelerations", point.accelerations}, {"effort", point.effort},
      {"time_sec", point.time_from_start.sec},
      {"time_nanosec", point.time_from_start.nanosec}});
  }
  const std::filesystem::path destination(path);
  std::error_code filesystem_error;
  const auto parent = destination.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, filesystem_error);
  }
  if (filesystem_error) {return Code::TASK_RESULT_BACKEND_ERROR;}
  const auto temporary_path = destination.string() + ".tmp";
  std::ofstream output(temporary_path, std::ios::out | std::ios::trunc);
  if (!output) {return Code::TASK_RESULT_BACKEND_ERROR;}
  output << artifact.dump(2) << '\n';
  output.close();
  if (!output) {return Code::TASK_RESULT_BACKEND_ERROR;}
  std::filesystem::create_hard_link(temporary_path, destination, filesystem_error);
  std::filesystem::remove(temporary_path);
  if (filesystem_error) {return Code::TASK_RESULT_BACKEND_ERROR;}
  set_pending_motion_planning_evidence(
    request.motion_goal_uuid, request.task_type.value, nlohmann::json{
      {"planning_stage", "place_waypoint_home_approach_precompute"},
      {"candidate_budget", kCandidateBudget},
      {"valid_candidate_count", candidate_count},
      {"selected_candidate_index", selected_candidate_index},
      {"selection_criteria", "execution_duration_then_joint_path_l1"},
      {"selected_trajectory_duration_s", selected_duration},
      {"selected_joint_path_l1_rad", selected_joint_distance},
      {"precompute_elapsed_s", elapsed_s},
      {"trajectory_file", path}}.dump());
  RCLCPP_INFO(
    node_->get_logger(),
    "PLACE WAYPOINT HOME-to-PLACE path saved candidate_count=%zu selected_duration_s=%.3f "
    "selected_joint_distance_rad=%.3f precompute_elapsed_s=%.3f",
    candidate_count, selected_duration, selected_joint_distance, elapsed_s);
  return Code::TASK_RESULT_SUCCESS;
}

bool IsaacRosMotionTransport::execute_planned(const NormalizedTrajectoryPlan & plan)
{
  const auto & request = plan.request;
  const auto stage_prefix = [&request]() {
      if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING) {
        return std::string("place_approach");
      }
      if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING) {
        return std::string("place_retreat");
      }
      return std::string("place_insertion");
    };
  if (request.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_place_failure_stage_ = stage_prefix() + "_execution";
  }
  const auto discard_if_interrupted = [&request]() {
      return request.interruption_result && request.interruption_result() !=
             arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
    };
  return execute_planned_trajectory(
    request, plan.trajectory, plan.cartesian_primitive, plan.place_recovery_waypoint,
    plan.planning_scales, discard_if_interrupted);
}

uint8_t IsaacRosMotionTransport::execute_joint_target()
{
  if (place_tray_return_joint_positions_.size() != arm_joint_names_.size() ||
    !publish_joint_target(place_tray_return_joint_positions_))
  {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  return wait_for_joint_target_result();
}

}  // namespace arm_cell_motion
