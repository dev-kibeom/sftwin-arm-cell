#include "arm_cell_motion_moveit2/task_executor.hpp"

#include <cmath>
#include <chrono>
#include <sstream>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <arm_cell_interfaces/msg/motion_task_phase.hpp>

namespace arm_cell_motion_moveit2
{

namespace
{
bool finite(double value)
{
  return std::isfinite(value);
}

bool valid_unit_vector(const geometry_msgs::msg::Vector3 & vector)
{
  const auto norm_squared = vector.x * vector.x + vector.y * vector.y + vector.z * vector.z;
  return finite(vector.x) && finite(vector.y) && finite(vector.z) &&
         finite(norm_squared) && std::abs(norm_squared - 1.0) <= 1e-6;
}

bool valid_pose(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  const auto q_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return pose.header.frame_id == "base_link" && finite(p.x) && finite(p.y) && finite(p.z) &&
         finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) && finite(q_norm) &&
         q_norm > 1e-12;
}

bool valid_unit_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const auto norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) && finite(norm_squared) &&
         std::abs(norm_squared - 1.0) <= 1e-6;
}

PlaceOrientationConstraint place_orientation_constraint(uint8_t value)
{
  using Goal = ExecuteTask::Goal;
  switch (value) {
    case Goal::PLACE_ORIENTATION_FIXED:
      return PlaceOrientationConstraint::FIXED;
    case Goal::PLACE_ORIENTATION_BOUNDED:
      return PlaceOrientationConstraint::BOUNDED;
    case Goal::PLACE_ORIENTATION_FREE:
      return PlaceOrientationConstraint::FREE;
    default:
      return PlaceOrientationConstraint::FIXED;
  }
}

bool valid_place_orientation_constraint(uint8_t value)
{
  using Goal = ExecuteTask::Goal;
  return value == Goal::PLACE_ORIENTATION_FIXED ||
         value == Goal::PLACE_ORIENTATION_BOUNDED ||
         value == Goal::PLACE_ORIENTATION_FREE;
}

tf2::Quaternion to_tf2(const geometry_msgs::msg::Quaternion & message)
{
  tf2::Quaternion result(message.x, message.y, message.z, message.w);
  result.normalize();
  return result;
}

geometry_msgs::msg::Quaternion from_tf2(const tf2::Quaternion & quaternion)
{
  geometry_msgs::msg::Quaternion result;
  result.x = quaternion.x();
  result.y = quaternion.y();
  result.z = quaternion.z();
  result.w = quaternion.w();
  return result;
}

geometry_msgs::msg::PoseStamped resolve_object_pose(
  const geometry_msgs::msg::PoseStamped & observation_pose, const MotionGeometry & geometry)
{
  const auto observation_rotation = to_tf2(observation_pose.pose.orientation);
  const auto observation_to_object_translation = geometry.observation_to_object.translation;
  const auto offset = tf2::quatRotate(
    observation_rotation,
    tf2::Vector3(
      observation_to_object_translation.x,
      observation_to_object_translation.y,
      observation_to_object_translation.z));
  auto object_pose = observation_pose;
  object_pose.pose.position.x += offset.x();
  object_pose.pose.position.y += offset.y();
  object_pose.pose.position.z += offset.z();
  const auto object_rotation = observation_rotation * to_tf2(geometry.observation_to_object.rotation);
  object_pose.pose.orientation = from_tf2(object_rotation);
  return object_pose;
}

geometry_msgs::msg::PoseStamped compose_target_pose(
  const geometry_msgs::msg::PoseStamped & object_pose, const MotionGeometry & geometry)
{
  const auto object_rotation = to_tf2(object_pose.pose.orientation);
  const auto tcp_offset = geometry.object_to_grasp_tcp.translation;
  const auto translated = tf2::quatRotate(
    object_rotation, tf2::Vector3(tcp_offset.x, tcp_offset.y, tcp_offset.z));
  geometry_msgs::msg::PoseStamped grasp = object_pose;
  grasp.pose.position.x += translated.x();
  grasp.pose.position.y += translated.y();
  grasp.pose.position.z += translated.z();
  const auto tcp_rotation = object_rotation * to_tf2(geometry.object_to_grasp_tcp.rotation);
  grasp.pose.orientation = from_tf2(tcp_rotation);
  return grasp;
}

geometry_msgs::msg::PoseStamped offset_along_insertion(
  const geometry_msgs::msg::PoseStamped & grasp,
  const MotionGeometry & geometry, double distance)
{
  const auto grasp_rotation = to_tf2(grasp.pose.orientation);
  auto axis = tf2::quatRotate(
    grasp_rotation,
    tf2::Vector3(
      geometry.insertion_axis_tcp.x, geometry.insertion_axis_tcp.y,
      geometry.insertion_axis_tcp.z));
  axis.normalize();
  auto result = grasp;
  result.pose.position.x -= distance * axis.x();
  result.pose.position.y -= distance * axis.y();
  result.pose.position.z -= distance * axis.z();
  return result;
}

geometry_msgs::msg::PoseStamped offset_object_pose(
  const geometry_msgs::msg::PoseStamped & object_pose,
  const geometry_msgs::msg::Vector3 & direction_object, double distance)
{
  auto result = object_pose;
  const auto direction_base = tf2::quatRotate(
    to_tf2(object_pose.pose.orientation),
    tf2::Vector3(direction_object.x, direction_object.y, direction_object.z));
  result.pose.position.x += distance * direction_base.x();
  result.pose.position.y += distance * direction_base.y();
  result.pose.position.z += distance * direction_base.z();
  return result;
}

std::string pose_summary(const geometry_msgs::msg::PoseStamped & pose)
{
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  std::ostringstream stream;
  stream << "frame=" << pose.header.frame_id
         << " xyz=(" << p.x << "," << p.y << "," << p.z << ")"
         << " quat=(" << q.x << "," << q.y << "," << q.z << "," << q.w << ")";
  return stream.str();
}

const char * holding_state_name(HoldingState state)
{
  switch (state) {
    case HoldingState::HELD:
      return "HELD";
    case HoldingState::RELEASED:
      return "RELEASED";
    case HoldingState::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

const char * holding_wait_result_name(HoldingWaitResult result)
{
  switch (result) {
    case HoldingWaitResult::CONFIRMED:
      return "CONFIRMED";
    case HoldingWaitResult::UNKNOWN:
      return "UNKNOWN";
    case HoldingWaitResult::TIMEOUT:
      return "TIMEOUT";
    case HoldingWaitResult::CANCELED:
      return "CANCELED";
    case HoldingWaitResult::SAFETY_PREEMPTED:
      return "SAFETY_PREEMPTED";
    case HoldingWaitResult::BACKEND_UNAVAILABLE:
    default:
      return "BACKEND_UNAVAILABLE";
  }
}

long long observation_age_ms(const HoldingObservation & observation)
{
  const auto now = HoldingObservation::Clock::now();
  if (observation.sequence == 0 || now < observation.observed_at) {
    return -1;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    now - observation.observed_at).count();
}


}  // namespace

bool MotionGeometry::valid() const
{
  const auto & translation = object_to_grasp_tcp.translation;
  const auto & rotation = object_to_grasp_tcp.rotation;
  const auto & observation_translation = observation_to_object.translation;
  const auto & observation_rotation = observation_to_object.rotation;
  const auto observation_rotation_norm = observation_rotation.x * observation_rotation.x +
    observation_rotation.y * observation_rotation.y + observation_rotation.z * observation_rotation.z +
    observation_rotation.w * observation_rotation.w;
  const auto rotation_norm = rotation.x * rotation.x + rotation.y * rotation.y +
    rotation.z * rotation.z + rotation.w * rotation.w;
  const auto axis_norm = std::sqrt(
    insertion_axis_tcp.x * insertion_axis_tcp.x +
    insertion_axis_tcp.y * insertion_axis_tcp.y + insertion_axis_tcp.z * insertion_axis_tcp.z);
  return configured && !observation_reference.empty() &&
         finite(observation_translation.x) && finite(observation_translation.y) &&
         finite(observation_translation.z) && finite(observation_rotation.x) &&
         finite(observation_rotation.y) && finite(observation_rotation.z) &&
         finite(observation_rotation.w) && finite(observation_rotation_norm) &&
         observation_rotation_norm > 1e-12 && finite(translation.x) && finite(translation.y) &&
         finite(translation.z) &&
         finite(rotation.x) && finite(rotation.y) && finite(rotation.z) && finite(rotation.w) &&
         finite(rotation_norm) && rotation_norm > 1e-12 && finite(insertion_axis_tcp.x) &&
         finite(insertion_axis_tcp.y) && finite(insertion_axis_tcp.z) && finite(axis_norm) &&
         axis_norm > 1e-9 &&
         finite(target_roll_tolerance_rad) && target_roll_tolerance_rad >= 0.0 &&
         finite(target_pitch_tolerance_rad) && target_pitch_tolerance_rad >= 0.0 &&
         finite(target_roll_sample_step_rad) && target_roll_sample_step_rad >= 0.0 &&
         finite(target_pitch_sample_step_rad) && target_pitch_sample_step_rad >= 0.0 &&
         (target_roll_tolerance_rad == 0.0 || target_roll_sample_step_rad > 0.0) &&
         (target_pitch_tolerance_rad == 0.0 || target_pitch_sample_step_rad > 0.0) &&
         finite(pick_approach_distance_m) && pick_approach_distance_m > 0.0 &&
         finite(pick_retract_distance_m) && pick_retract_distance_m > 0.0;
}

TaskExecutor::TaskExecutor(
  std::shared_ptr<MotionBackend> backend, MotionGeometry geometry,
  std::chrono::milliseconds holding_confirmation_timeout)
: TaskExecutor(
    std::move(backend), nullptr, geometry, {}, holding_confirmation_timeout)
{
}

TaskExecutor::TaskExecutor(
  std::shared_ptr<MotionBackend> backend,
  std::shared_ptr<GripperPort> gripper_port,
  MotionGeometry geometry,
  InterruptionChecker interruption_checker,
  std::chrono::milliseconds holding_confirmation_timeout)
: backend_(std::move(backend)), gripper_port_(std::move(gripper_port)),
  geometry_(geometry), holding_confirmation_timeout_(holding_confirmation_timeout),
  interruption_checker_(std::move(interruption_checker))
{
  if (!gripper_port_ && backend_) {
    gripper_port_ = backend_->gripper_port();
  }
}

void TaskExecutor::set_interruption_checker(InterruptionChecker interruption_checker)
{
  interruption_checker_ = std::move(interruption_checker);
}

void TaskExecutor::set_runtime_geometry(
  MotionGeometry geometry, std::chrono::milliseconds holding_confirmation_timeout)
{
  geometry_ = std::move(geometry);
  holding_confirmation_timeout_ = holding_confirmation_timeout;
}

void TaskExecutor::set_object_state(MotionObjectState object_state)
{
  object_state_tracker_.set_state(object_state);
}

MotionObjectState TaskExecutor::object_state() const
{
  return object_state_tracker_.state();
}

uint8_t TaskExecutor::execute(const ExecuteTask::Goal & goal)
{
  return execute(goal, object_state_tracker_.state());
}

uint8_t TaskExecutor::execute(
  const ExecuteTask::Goal & goal, MotionObjectState object_state,
  const std::string & motion_execution_id)
{
  motion_execution_id_ = motion_execution_id;
  set_object_state(object_state);
  switch (goal.task_type.value) {
    case MotionTaskType::TASK_TYPE_PICK:
      return execute_pick(goal);
    case MotionTaskType::TASK_TYPE_PLACE:
      return execute_place(goal);
    case MotionTaskType::TASK_TYPE_GO_HOME:
      return backend_->execute_task(goal);
    case MotionTaskType::TASK_TYPE_RETRACT:
      if (object_state == MotionObjectState::UNCERTAIN) {
        return MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
      }
      return backend_->execute_task(goal);
    default:
      return MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
  }
}

uint8_t TaskExecutor::execute_pick(const ExecuteTask::Goal & goal)
{
  if (!goal.has_target_pose || goal.target_pose.header.frame_id != "base_link" ||
    !goal.has_grasp_width || goal.grasp_width_mm < 0.0F || goal.grasp_width_mm > 85.0F ||
    object_state_tracker_.state() != MotionObjectState::NO_OBJECT || !geometry_.valid() ||
    !valid_pose(goal.target_pose))
  {
    return MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
  }
  backend_->begin_pick_grasp_relation();
  const auto fail_pick = [this](uint8_t failure) {
      backend_->discard_pending_grasp_relation();
      return failure;
    };
  std::uint64_t release_baseline = 0;
  auto result = execute_gripper_open(release_baseline);
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    return fail_pick(result);
  }
  const auto release_result = fresh_holding(HoldingState::RELEASED, release_baseline);
  if (release_result != HoldingWaitResult::CONFIRMED) {
    if (release_result == HoldingWaitResult::CANCELED) {
      return fail_pick(MotionTaskResultCode::TASK_RESULT_CANCELED);
    }
    if (release_result == HoldingWaitResult::SAFETY_PREEMPTED) {
      return fail_pick(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
    }
    if (release_result == HoldingWaitResult::BACKEND_UNAVAILABLE) {
      return fail_pick(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
    }
    return fail_pick(MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  }
  result = execute_motion(
    goal,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING);
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    return fail_pick(result);
  }
  result = execute_motion(goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING);
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    return fail_pick(result);
  }
  std::uint64_t holding_baseline = 0;
  result = execute_gripper_close(goal.grasp_width_mm, holding_baseline);
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    return fail_pick(result);
  }
  const auto holding_result = fresh_holding(HoldingState::HELD, holding_baseline);
  if (holding_result != HoldingWaitResult::CONFIRMED) {
    if (holding_result == HoldingWaitResult::CANCELED) {
      return fail_pick(MotionTaskResultCode::TASK_RESULT_CANCELED);
    }
    if (holding_result == HoldingWaitResult::SAFETY_PREEMPTED) {
      return fail_pick(MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED);
    }
    if (holding_result == HoldingWaitResult::BACKEND_UNAVAILABLE) {
      return fail_pick(MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE);
    }
    return fail_pick(MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE);
  }
  object_state_tracker_.mark_attached();
  if (!backend_->confirm_fresh_held_grasp_relation()) {
    backend_->discard_pending_grasp_relation();
    return MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
  }
  return execute_motion(goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING);
}

uint8_t TaskExecutor::execute_place(const ExecuteTask::Goal & goal)
{
  if (!goal.has_target_pose || goal.target_pose.header.frame_id != "base_link" ||
    object_state_tracker_.state() != MotionObjectState::ATTACHED || !geometry_.valid() ||
    !valid_pose(goal.target_pose) || !finite(goal.place_position_tolerance_m) ||
    goal.place_position_tolerance_m < 0.0 ||
    !finite(goal.place_orientation_tolerance_rad) ||
    goal.place_orientation_tolerance_rad < 0.0 ||
    !valid_place_orientation_constraint(goal.place_orientation_constraint) ||
    (goal.place_orientation_constraint == ExecuteTask::Goal::PLACE_ORIENTATION_FREE &&
    goal.has_place_tool_orientation_preference) ||
    (goal.has_place_tool_orientation_preference &&
    !valid_unit_quaternion(goal.place_tool_orientation_preference)) ||
    !valid_unit_vector(goal.place_approach_direction_object) ||
    !finite(goal.place_approach_distance_m) || goal.place_approach_distance_m <= 0.0 ||
    !finite(goal.place_retract_distance_m) || goal.place_retract_distance_m <= 0.0)
  {
    return MotionTaskResultCode::TASK_RESULT_INVALID_GOAL;
  }
  auto result = execute_motion(
    goal,
    arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING);
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    return result;
  }
  result = execute_motion(goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING);
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    return result;
  }
  std::uint64_t holding_baseline = 0;
  result = execute_gripper_open(holding_baseline);
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    return result;
  }
  const auto holding_result = fresh_holding(HoldingState::RELEASED, holding_baseline);
  if (holding_result != HoldingWaitResult::CONFIRMED) {
    if (holding_result == HoldingWaitResult::CANCELED) {
      return MotionTaskResultCode::TASK_RESULT_CANCELED;
    }
    if (holding_result == HoldingWaitResult::SAFETY_PREEMPTED) {
      return MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
    }
    if (holding_result == HoldingWaitResult::BACKEND_UNAVAILABLE) {
      return MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
    }
    return MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
  }
  backend_->confirm_fresh_released_grasp_relation();
  object_state_tracker_.mark_detached();
  return execute_motion(goal, arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING);
}

uint8_t TaskExecutor::execute_motion(const ExecuteTask::Goal & goal, uint8_t phase)
{
  const auto request = normalized_request(goal, phase);
  RCLCPP_DEBUG(
    rclcpp::get_logger("task_executor"),
    "motion_execution_id=%s phase=%u observation_reference=%s observation_pose{%s} "
    "object_pose{%s} "
    "target_pose{%s} approach_pose{%s} retract_pose{%s} "
    "approach_distance_m=%.6f retract_distance_m=%.6f grasp_width_mm=%.6f",
    motion_execution_id_.c_str(), static_cast<unsigned>(request.phase),
    request.observation_reference.c_str(),
    pose_summary(request.observation_pose).c_str(),
    pose_summary(request.object_pose).c_str(), pose_summary(request.target_pose).c_str(),
    pose_summary(request.approach_target).c_str(), pose_summary(request.retract_target).c_str(),
    request.approach_distance_m, request.retract_distance_m,
    static_cast<double>(request.grasp_width_mm));
  return backend_->execute_normalized(request);
}

uint8_t TaskExecutor::execute_gripper_close(
  float width_mm, std::uint64_t & baseline_sequence)
{
  if (!gripper_port_) {
    return MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
  }
  baseline_sequence = holding_sequence();
  const auto result = gripper_port_->close(width_mm);
  if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    RCLCPP_INFO(
      rclcpp::get_logger("task_executor"),
      "gripper close command issued motion_execution_id=%s",
      motion_execution_id_.c_str());
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("task_executor"),
      "gripper close command failed to issue motion_execution_id=%s result_code=%u",
      motion_execution_id_.c_str(), static_cast<unsigned>(result));
  }
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS && gripper_port_->active()) {
    gripper_port_->stop();
  }
  return map_object_operation_result(result);
}

uint8_t TaskExecutor::execute_gripper_open(std::uint64_t & baseline_sequence)
{
  if (!gripper_port_) {
    return MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
  }
  baseline_sequence = holding_sequence();
  const auto result = gripper_port_->open();
  if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    RCLCPP_INFO(
      rclcpp::get_logger("task_executor"),
      "gripper open command issued motion_execution_id=%s",
      motion_execution_id_.c_str());
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("task_executor"),
      "gripper open command failed to issue motion_execution_id=%s result_code=%u",
      motion_execution_id_.c_str(), static_cast<unsigned>(result));
  }
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS && gripper_port_->active()) {
    gripper_port_->stop();
  }
  return map_object_operation_result(result);
}

std::uint64_t TaskExecutor::holding_sequence() const
{
  return gripper_port_ ? gripper_port_->holding().sequence : 0;
}

HoldingWaitResult TaskExecutor::fresh_holding(
  HoldingState expected, std::uint64_t baseline_sequence) const
{
  if (!gripper_port_) {
    return HoldingWaitResult::BACKEND_UNAVAILABLE;
  }

  const auto interruption = [this]() {
      return interruption_checker_ ? interruption_checker_() :
             MotionTaskResultCode::TASK_RESULT_SUCCESS;
    };
  const auto interruption_result = [&interruption]() {
      const auto result = interruption();
      if (result == MotionTaskResultCode::TASK_RESULT_CANCELED) {
        return HoldingWaitResult::CANCELED;
      }
      if (result == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED) {
        return HoldingWaitResult::SAFETY_PREEMPTED;
      }
      if (result == MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE) {
        return HoldingWaitResult::BACKEND_UNAVAILABLE;
      }
      return HoldingWaitResult::CONFIRMED;
    };

  const auto start = HoldingObservation::Clock::now();
  const auto deadline = start + holding_confirmation_timeout_;
  HoldingObservation last_observation;
  bool have_last_observation = false;
  RCLCPP_INFO(
    rclcpp::get_logger("task_executor"),
    "holding confirmation wait started motion_execution_id=%s expected=%s",
    motion_execution_id_.c_str(), holding_state_name(expected));
  bool last_fresh = false;
  const auto finish = [&](HoldingWaitResult result) {
      const auto level_logger = rclcpp::get_logger("task_executor");
      if (result == HoldingWaitResult::CONFIRMED) {
        RCLCPP_INFO(
          level_logger,
          "holding confirmation ended motion_execution_id=%s result=CONFIRMED "
          "expected=%s baseline_seq=%llu last_seq=%llu state=%s fresh=%s age_ms=%lld",
          motion_execution_id_.c_str(), holding_state_name(expected),
          static_cast<unsigned long long>(baseline_sequence),
          static_cast<unsigned long long>(last_observation.sequence),
          holding_state_name(last_observation.state),
          last_observation.fresh() ? "true" : "false",
          observation_age_ms(last_observation));
      } else if (result == HoldingWaitResult::TIMEOUT ||
        result == HoldingWaitResult::UNKNOWN ||
        result == HoldingWaitResult::BACKEND_UNAVAILABLE)
      {
        RCLCPP_WARN(
          level_logger,
          "holding confirmation ended motion_execution_id=%s result=%s "
          "expected=%s baseline_seq=%llu last_seq=%llu state=%s fresh=%s age_ms=%lld",
          motion_execution_id_.c_str(), holding_wait_result_name(result),
          holding_state_name(expected),
          static_cast<unsigned long long>(baseline_sequence),
          static_cast<unsigned long long>(last_observation.sequence),
          holding_state_name(last_observation.state),
          last_observation.fresh() ? "true" : "false",
          observation_age_ms(last_observation));
      } else {
        RCLCPP_DEBUG(
          level_logger,
          "holding confirmation ended motion_execution_id=%s result=%s "
          "expected=%s baseline_seq=%llu last_seq=%llu state=%s fresh=%s age_ms=%lld",
          motion_execution_id_.c_str(), holding_wait_result_name(result),
          holding_state_name(expected),
          static_cast<unsigned long long>(baseline_sequence),
          static_cast<unsigned long long>(last_observation.sequence),
          holding_state_name(last_observation.state),
          last_observation.fresh() ? "true" : "false",
          observation_age_ms(last_observation));
      }
      return result;
    };
  while (HoldingObservation::Clock::now() < deadline) {
    const auto interrupted = interruption_result();
    if (interrupted != HoldingWaitResult::CONFIRMED) {
      return finish(interrupted);
    }

    const auto observation = gripper_port_->holding();
    const bool observation_fresh = observation.fresh();
    if (!have_last_observation || observation.state != last_observation.state ||
      observation_fresh != last_fresh)
    {
      if (observation_fresh && observation.state != HoldingState::UNKNOWN) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("task_executor"),
          "holding observation transition diagnostic motion_execution_id=%s state=%s",
          motion_execution_id_.c_str(), holding_state_name(observation.state));
      } else {
        RCLCPP_DEBUG(
          rclcpp::get_logger("task_executor"),
          "holding observation diagnostic state=%s fresh=false motion_execution_id=%s",
          holding_state_name(observation.state), motion_execution_id_.c_str());
      }
      RCLCPP_DEBUG(
        rclcpp::get_logger("task_executor"),
        "holding observation motion_execution_id=%s seq=%llu age_ms=%lld fresh=%s",
        motion_execution_id_.c_str(), static_cast<unsigned long long>(observation.sequence),
        observation_age_ms(observation), observation_fresh ? "true" : "false");
    }
    last_observation = observation;
    have_last_observation = true;
    last_fresh = observation_fresh;
    if (observation.sequence > baseline_sequence &&
      observation.state == expected && observation.fresh())
    {
      const auto final_interruption = interruption_result();
      return finish(
        final_interruption == HoldingWaitResult::CONFIRMED ?
        HoldingWaitResult::CONFIRMED : final_interruption);
    }
    if (observation.sequence > baseline_sequence &&
      observation.state == HoldingState::UNKNOWN)
    {
      return finish(HoldingWaitResult::UNKNOWN);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto interrupted = interruption_result();
  return finish(
    interrupted == HoldingWaitResult::CONFIRMED ?
    HoldingWaitResult::TIMEOUT : interrupted);
}

uint8_t TaskExecutor::map_object_operation_result(uint8_t result)
{
  if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS ||
    result == MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE ||
    result == MotionTaskResultCode::TASK_RESULT_CANCELED ||
    result == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED)
  {
    return result;
  }
  return MotionTaskResultCode::TASK_RESULT_GRASP_FAILURE;
}

NormalizedMotionRequest TaskExecutor::normalized_request(
  const ExecuteTask::Goal & goal, uint8_t phase) const
{
  NormalizedMotionRequest request;
  request.task_type = goal.task_type;
  request.phase = phase;
  const bool is_pick = goal.task_type.value == MotionTaskType::TASK_TYPE_PICK;
  const bool is_place = goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE;
  request.observation_reference = is_pick ? geometry_.observation_reference : "";
  request.grasp_width_mm = goal.grasp_width_mm;
  request.observation_pose = is_pick ? goal.target_pose : geometry_msgs::msg::PoseStamped{};
  const auto object_pose = is_pick ? resolve_object_pose(goal.target_pose, geometry_) : goal.target_pose;
  request.object_pose = object_pose;
  request.target_pose = compose_target_pose(object_pose, geometry_);
  request.object_to_grasp_tcp = geometry_.object_to_grasp_tcp;
  request.place_position_tolerance_m = is_place ? goal.place_position_tolerance_m : 0.0;
  request.place_orientation_tolerance_rad =
    is_place ? goal.place_orientation_tolerance_rad : 0.0;
  request.place_orientation_constraint = is_place ?
    place_orientation_constraint(goal.place_orientation_constraint) :
    PlaceOrientationConstraint::FIXED;
  request.has_place_tool_orientation_preference =
    is_place && goal.has_place_tool_orientation_preference;
  request.place_tool_orientation_preference = goal.place_tool_orientation_preference;
  request.place_approach_direction_object = is_place ?
    goal.place_approach_direction_object : geometry_msgs::msg::Vector3{};
  request.insertion_axis_tcp = geometry_.insertion_axis_tcp;
  const auto approach_distance = is_pick ? geometry_.pick_approach_distance_m :
    goal.place_approach_distance_m;
  request.approach_distance_m = approach_distance;
  request.retract_distance_m = is_place ? goal.place_retract_distance_m :
    geometry_.pick_retract_distance_m;
  request.target_roll_tolerance_rad = geometry_.target_roll_tolerance_rad;
  request.target_pitch_tolerance_rad = geometry_.target_pitch_tolerance_rad;
  request.target_roll_sample_step_rad = geometry_.target_roll_sample_step_rad;
  request.target_pitch_sample_step_rad = geometry_.target_pitch_sample_step_rad;
  request.approach_target = offset_along_insertion(
    request.target_pose, geometry_, approach_distance);
  request.retract_target = offset_along_insertion(
    request.target_pose, geometry_, request.retract_distance_m);
  if (is_place) {
    request.approach_target = compose_target_pose(
      offset_object_pose(
        request.object_pose, goal.place_approach_direction_object,
        goal.place_approach_distance_m), geometry_);
    request.retract_target = compose_target_pose(
      offset_object_pose(
        request.object_pose, goal.place_approach_direction_object,
        goal.place_retract_distance_m), geometry_);
  }
  if (phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING) {
    request.tcp_target = request.approach_target;
  } else if (phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING) {
    request.tcp_target = request.retract_target;
  } else {
    request.tcp_target = request.target_pose;
  }
  return request;
}

}  // namespace arm_cell_motion_moveit2
