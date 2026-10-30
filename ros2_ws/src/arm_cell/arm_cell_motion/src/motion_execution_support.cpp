#include "arm_cell_motion/motion_execution_support.hpp"

#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <arm_cell_interfaces/msg/motion_task_phase.hpp>

#include "arm_cell_motion/motion_result_code_name.hpp"
#include "arm_cell_motion/candidate_occupancy.hpp"
#include "arm_cell_motion/pick_approach_geometry.hpp"

namespace arm_cell_motion
{
namespace
{

bool finite(double value) {return std::isfinite(value);}

PlaceOrientationConstraint to_place_orientation_constraint(std::uint8_t value)
{
  switch (value) {
    case ExecuteTask::Goal::PLACE_ORIENTATION_BOUNDED:
      return PlaceOrientationConstraint::BOUNDED;
    case ExecuteTask::Goal::PLACE_ORIENTATION_FREE:
      return PlaceOrientationConstraint::FREE;
    default:
      return PlaceOrientationConstraint::FIXED;
  }
}

std::string execution_uuid_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : uuid.uuid) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
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
  const auto object_rotation = observation_rotation *
    to_tf2(geometry.observation_to_object.rotation);
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


}  // namespace

bool MotionGeometry::valid() const
{
  const auto & translation = object_to_grasp_tcp.translation;
  const auto & rotation = object_to_grasp_tcp.rotation;
  const auto & observation_translation = observation_to_object.translation;
  const auto & observation_rotation = observation_to_object.rotation;
  const auto observation_rotation_norm = observation_rotation.x * observation_rotation.x +
    observation_rotation.y * observation_rotation.y + observation_rotation.z *
    observation_rotation.z +
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
         finite(pick_retract_distance_m) && pick_retract_distance_m > 0.0;
}

MotionExecutionSupport::MotionExecutionSupport(
  std::shared_ptr<MotionBackend> backend, MotionGeometry geometry)
: backend_(std::move(backend)), geometry_(std::move(geometry))
{}

void MotionExecutionSupport::set_interruption_checker(InterruptionChecker checker)
{interruption_checker_ = std::move(checker);}

void MotionExecutionSupport::set_runtime_geometry(MotionGeometry geometry)
{geometry_ = std::move(geometry);}

uint8_t MotionExecutionSupport::execute_motion(const MotionPortRequest & request)
{
  const auto started = std::chrono::steady_clock::now();
  const auto parent_execution_id = execution_uuid_string(request.execution_id);
  auto normalized = normalized_request(request);
  normalized.interruption_result = interruption_checker_;
  RCLCPP_DEBUG(
    rclcpp::get_logger("motion_execution_support"),
    "Executing movement_id=%s under execution_id=%s",
    request.movement_id.c_str(), parent_execution_id.c_str());

  std::uint8_t result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  std::string failure_detail;
  try {
    result = interruption_checker_ ? interruption_checker_() :
      MotionTaskResultCode::TASK_RESULT_SUCCESS;
    if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      if (backend_->supports_split_planning()) {
        NormalizedTrajectoryPlan plan;
        result = backend_->plan_normalized(normalized, plan);
        if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
          result = interruption_checker_ ? interruption_checker_() :
            MotionTaskResultCode::TASK_RESULT_SUCCESS;
          if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
            result = execute_planned_motion(plan, false);
          }
        }
      } else {
        result = backend_->execute_normalized(normalized);
      }
      if (interruption_checker_) {
        const auto interrupted = interruption_checker_();
        if (interrupted != MotionTaskResultCode::TASK_RESULT_SUCCESS) {result = interrupted;}
      }
    }
  } catch (const std::exception & error) {
    failure_detail = error.what();
    result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  } catch (...) {
    failure_detail = "unknown exception";
    result = MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  std::string failure_stage;
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    failure_stage = result == MotionTaskResultCode::TASK_RESULT_IK_UNREACHABLE ||
      result == MotionTaskResultCode::TASK_RESULT_PATH_OBSTRUCTED ||
      result == MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED ?
      "motion_port:planning" : "motion_port:execution";
  }
  try {
    backend_->finalize_motion_planning_diagnostic(
      result,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
      failure_stage, failure_detail);
  } catch (...) {
    // Diagnostic capture is best-effort and cannot replace the movement result.
  }
  return result;
}

std::uint8_t MotionExecutionSupport::plan_motion(
  const MotionPortRequest & request, NormalizedTrajectoryPlan & plan)
{
  auto normalized = normalized_request(request);
  normalized.interruption_result = interruption_checker_;
  if (!backend_->supports_split_planning()) {
    return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  const auto interrupted = interruption_checker_ ? interruption_checker_() :
    MotionTaskResultCode::TASK_RESULT_SUCCESS;
  if (interrupted != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    const auto failure_stage = interrupted == MotionTaskResultCode::TASK_RESULT_CANCELED ?
      "motion_port:canceled" : "motion_port:safety_preempted";
    try {
      backend_->finalize_motion_planning_diagnostic(interrupted, 0.0, failure_stage, "");
    } catch (...) {
      // Diagnostic capture is best-effort and cannot replace the motion result.
    }
    return interrupted;
  }
  const auto started = std::chrono::steady_clock::now();
  const auto result = backend_->plan_normalized(normalized, plan);
  if (result != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    const auto failure_stage = result == MotionTaskResultCode::TASK_RESULT_CANCELED ?
      "motion_port:canceled" : result == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED ?
      "motion_port:safety_preempted" : "motion_port:planning";
    try {
      backend_->finalize_motion_planning_diagnostic(
        result,
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
        failure_stage, "");
    } catch (...) {
      // Diagnostic capture is best-effort and cannot replace the planning result.
    }
  }
  return result;
}

std::uint8_t MotionExecutionSupport::execute_planned_motion(
  const NormalizedTrajectoryPlan & plan, bool finalize_diagnostics)
{
  if (!backend_->supports_split_planning()) {
    return MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }
  const auto started = std::chrono::steady_clock::now();
  const auto interrupted = interruption_checker_ ? interruption_checker_() :
    MotionTaskResultCode::TASK_RESULT_SUCCESS;
  if (interrupted != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    if (finalize_diagnostics) {
      const auto failure_stage = interrupted == MotionTaskResultCode::TASK_RESULT_CANCELED ?
        "motion_port:canceled" : "motion_port:safety_preempted";
      backend_->finalize_motion_planning_diagnostic(interrupted, 0.0, failure_stage, "");
    }
    return interrupted;
  }
  auto executable_plan = plan;
  executable_plan.request.interruption_result = interruption_checker_;
  auto result = backend_->execute_planned_normalized(executable_plan);
  if (interruption_checker_) {
    const auto final_interruption = interruption_checker_();
    if (final_interruption != MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      result = final_interruption;
    }
  }
  const auto failure_stage = result == MotionTaskResultCode::TASK_RESULT_SUCCESS ? "" :
    result == MotionTaskResultCode::TASK_RESULT_CANCELED ? "motion_port:canceled" :
    result == MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED ?
    "motion_port:safety_preempted" : "motion_port:execution";
  if (finalize_diagnostics) {
    backend_->finalize_motion_planning_diagnostic(
      result, std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
      failure_stage, "");
  }
  return result;
}

std::uint8_t MotionExecutionSupport::execute_task(
  const ExecuteTask::Goal & goal, const std::string & execution_id)
{
  const auto started = std::chrono::steady_clock::now();
  auto result = interruption_checker_ ? interruption_checker_() :
    MotionTaskResultCode::TASK_RESULT_SUCCESS;
  if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    result = backend_->execute_task(goal, execution_id);
    if (interruption_checker_) {
      const auto interrupted = interruption_checker_();
      if (interrupted != MotionTaskResultCode::TASK_RESULT_SUCCESS) {result = interrupted;}
    }
  }
  backend_->finalize_motion_planning_diagnostic(
    result, std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
    result == MotionTaskResultCode::TASK_RESULT_SUCCESS ? "" : "motion_port:execution", "");
  return result;
}

std::uint8_t MotionExecutionSupport::execute_joint_target()
{
  const auto started = std::chrono::steady_clock::now();
  auto result = interruption_checker_ ? interruption_checker_() :
    MotionTaskResultCode::TASK_RESULT_SUCCESS;
  if (result == MotionTaskResultCode::TASK_RESULT_SUCCESS) {
    result = backend_->execute_joint_target();
    if (interruption_checker_) {
      const auto interrupted = interruption_checker_();
      if (interrupted != MotionTaskResultCode::TASK_RESULT_SUCCESS) {result = interrupted;}
    }
  }
  backend_->finalize_motion_planning_diagnostic(
    result, std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
    result == MotionTaskResultCode::TASK_RESULT_SUCCESS ? "" : "motion_port:execution", "");
  return result;
}

NormalizedMotionRequest MotionExecutionSupport::normalized_request(
  const ExecuteTask::Goal & goal, std::uint8_t phase, const std::string & execution_id) const
{
  NormalizedMotionRequest request;
  request.task_type = goal.task_type;
  request.phase = phase;
  const bool is_pick = goal.task_type.value == MotionTaskType::TASK_TYPE_PICK;
  const bool is_place = goal.task_type.value == MotionTaskType::TASK_TYPE_PLACE;
  request.observation_reference = is_pick ? geometry_.observation_reference : "";
  request.orientation_symmetry_order = is_pick ? goal.orientation_symmetry_order : 0;
  request.has_estimated_object_height = is_pick && goal.has_estimated_object_height;
  request.has_unobserved_yaw_fallback = is_pick && goal.has_unobserved_yaw_fallback;
  request.unobserved_yaw_fallback_rad = is_pick ? goal.unobserved_yaw_fallback_rad : 0.0;
  request.vision_acquisition_id = is_pick ? goal.vision_acquisition_id : "";
  request.vision_candidate_id = is_pick ? goal.vision_candidate_id : "";
  request.material_candidates = is_pick ? goal.material_candidates :
    std::vector<arm_cell_interfaces::msg::PickCandidate>{};
  const bool vision_derived_pick = is_pick && !request.material_candidates.empty() &&
    !request.vision_candidate_id.empty();
  request.has_non_target_occupancy = is_pick && (vision_derived_pick || goal.has_non_target_occupancy);
  if (vision_derived_pick) {
    request.non_target_occupancy = candidate_occupancy_from_candidates(
      request.material_candidates, request.vision_acquisition_id, request.vision_candidate_id);
  } else if (is_pick) {
    request.non_target_occupancy = goal.non_target_occupancy;
  }
  request.has_observed_rectangle = is_pick && goal.has_observed_rectangle;
  request.rectangle_long_side_m = is_pick ? goal.rectangle_long_side_m : 0.0F;
  request.rectangle_short_side_m = is_pick ? goal.rectangle_short_side_m : 0.0F;
  request.grasp_width_min_mm = is_pick ? goal.grasp_width_min_mm : 0.0F;
  request.grasp_width_max_mm = is_pick ? goal.grasp_width_max_mm : 0.0F;
  request.motion_goal_uuid = execution_id;
  request.observation_pose = geometry_msgs::msg::PoseStamped{};
  if (is_pick && goal.orientation_symmetry_order == ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE) {
    request.observation_pose.header = goal.target_pose.header;
    request.observation_pose.pose.position = goal.target_pose.pose.position;
    const tf2::Quaternion fallback_rotation(
      tf2::Vector3(0.0, 0.0, 1.0), goal.unobserved_yaw_fallback_rad);
    const auto fallback_orientation = from_tf2(fallback_rotation);
    request.observation_pose.pose.orientation = fallback_orientation;
  } else if (is_pick) {
    request.observation_pose = goal.target_pose;
  }
  auto object_pose =
    is_pick ? resolve_object_pose(request.observation_pose, geometry_) : goal.target_pose;
  if (is_pick && goal.has_estimated_object_height &&
    std::isfinite(goal.rectangle_long_side_m) && goal.rectangle_long_side_m > 0.0F &&
    std::isfinite(goal.rectangle_short_side_m) && goal.rectangle_short_side_m > 0.0F)
  {
    const auto rotation = to_tf2(request.observation_pose.pose.orientation);
    const auto inward = tf2::quatRotate(
      rotation, tf2::Vector3(0.0, 0.0, -static_cast<double>(goal.estimated_object_height_m) / 2.0));
    object_pose.pose.position.x = request.observation_pose.pose.position.x + inward.x();
    object_pose.pose.position.y = request.observation_pose.pose.position.y + inward.y();
    object_pose.pose.position.z = request.observation_pose.pose.position.z + inward.z();
  }
  request.object_pose = object_pose;
  request.target_pose = compose_target_pose(object_pose, geometry_);
  request.object_to_grasp_tcp = geometry_.object_to_grasp_tcp;
  request.place_position_tolerance_m = is_place ? goal.place_position_tolerance_m : 0.0;
  request.place_orientation_tolerance_rad =
    is_place ? goal.place_orientation_tolerance_rad : 0.0;
  request.place_orientation_constraint = is_place ?
    to_place_orientation_constraint(goal.place_orientation_constraint) :
    PlaceOrientationConstraint::FIXED;
  request.has_place_tool_orientation_preference =
    is_place && goal.has_place_tool_orientation_preference;
  request.place_tool_orientation_preference = goal.place_tool_orientation_preference;
  request.place_approach_direction_object = is_place ?
    goal.place_approach_direction_object : geometry_msgs::msg::Vector3{};
  request.insertion_axis_tcp = geometry_.insertion_axis_tcp;
  request.pick_approach_axis = goal.pick_approach_translation_direction ==
    ExecuteTask::Goal::PICK_APPROACH_PLANNING_FRAME_VERTICAL ?
    PickApproachAxis::PLANNING_FRAME_VERTICAL : PickApproachAxis::TCP_INSERTION_AXIS;
  request.pick_grasp_position_tolerance_m = is_pick ? goal.pick_grasp_position_tolerance_m : 0.0;
  request.pick_grasp_orientation_tolerance_rad =
    is_pick ? goal.pick_grasp_orientation_tolerance_rad : 0.0;
  const auto approach_distance = is_pick ? goal.pick_approach_distance_m :
    goal.place_approach_distance_m;
  request.approach_distance_m = approach_distance;
  request.retract_distance_m = is_place ? goal.place_retract_distance_m :
    geometry_.pick_retract_distance_m;
  request.estimated_object_height_m = is_pick ? goal.estimated_object_height_m : 0.0;
  request.approach_target = is_pick ? pick_approach_pose(
    request.target_pose, request.pick_approach_axis,
    geometry_.insertion_axis_tcp, approach_distance) : offset_along_insertion(
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

NormalizedMotionRequest MotionExecutionSupport::normalized_request(
  const MotionPortRequest & movement) const
{
  NormalizedMotionRequest request;
  const bool pick = movement.segment == MotionSegmentKind::PICK_APPROACH ||
    movement.segment == MotionSegmentKind::PICK_DESCENT ||
    movement.segment == MotionSegmentKind::PICK_RETRACT;
  const bool disturbance = movement.segment == MotionSegmentKind::DISTURB_APPROACH ||
    movement.segment == MotionSegmentKind::DISTURB_PUSH ||
    movement.segment == MotionSegmentKind::DISTURB_RETRACT;
  const bool place = !pick;
  request.task_type.value = pick ? MotionTaskType::TASK_TYPE_PICK :
    MotionTaskType::TASK_TYPE_PLACE;
  switch (movement.segment) {
    case MotionSegmentKind::PICK_APPROACH:
    case MotionSegmentKind::PLACE_APPROACH:
      request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
      break;
    case MotionSegmentKind::PICK_RETRACT:
    case MotionSegmentKind::PLACE_RETRACT:
      request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING;
      break;
    case MotionSegmentKind::PICK_DESCENT:
    case MotionSegmentKind::PLACE_DESCENT:
    case MotionSegmentKind::PLACE_CLEARANCE:
      request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING;
      break;
    case MotionSegmentKind::DISTURB_APPROACH:
      request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING;
      break;
    case MotionSegmentKind::DISTURB_PUSH:
      request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING;
      break;
    case MotionSegmentKind::DISTURB_RETRACT:
      request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING;
      break;
  }
  request.motion_goal_uuid = execution_uuid_string(movement.execution_id);
  request.movement_id = movement.movement_id;
  request.segment = movement.segment;
  request.has_start_joint_state = movement.has_start_joint_state;
  request.start_joint_state = movement.start_joint_state;
  request.target_is_tcp_pose = movement.target_is_tcp_pose;
  request.place_waypoint_approach = movement.place_waypoint_approach;
  request.place_waypoint_clearance_y_margin_m = movement.place_waypoint_clearance_y_margin_m;
  request.observation_reference = pick ? geometry_.observation_reference : "";
  request.orientation_symmetry_order = static_cast<std::uint8_t>(
    movement.constraints.orientation_symmetry_order);
  request.has_estimated_object_height = pick && movement.constraints.has_estimated_object_height;
  request.has_unobserved_yaw_fallback = pick && movement.has_unobserved_yaw_fallback;
  request.unobserved_yaw_fallback_rad = pick ? movement.unobserved_yaw_fallback_rad : 0.0;
  request.vision_acquisition_id = pick ? movement.vision_acquisition_id : "";
  request.vision_candidate_id = pick ? movement.vision_candidate_id : "";
  request.material_candidates = pick ? movement.material_candidates :
    std::vector<arm_cell_interfaces::msg::PickCandidate>{};
  const bool vision_derived_pick = pick && !request.material_candidates.empty() &&
    !request.vision_candidate_id.empty();
  request.has_non_target_occupancy = pick &&
    (vision_derived_pick || movement.has_non_target_occupancy);
  if (vision_derived_pick) {
    request.non_target_occupancy = candidate_occupancy_from_candidates(
      request.material_candidates, request.vision_acquisition_id, request.vision_candidate_id);
  } else if (pick) {
    request.non_target_occupancy = movement.non_target_occupancy;
  }
  request.has_observed_rectangle = pick && movement.constraints.has_observed_rectangle;
  request.rectangle_long_side_m = pick ? movement.constraints.rectangle_long_side_m : 0.0F;
  request.rectangle_short_side_m = pick ? movement.constraints.rectangle_short_side_m : 0.0F;
  request.grasp_width_min_mm = pick ? movement.constraints.grasp_width_min_mm : 0.0F;
  request.grasp_width_max_mm = pick ? movement.constraints.grasp_width_max_mm : 0.0F;
  request.observation_pose = geometry_msgs::msg::PoseStamped{};
  if (pick && movement.constraints.orientation_symmetry_order ==
    MotionOrientationSymmetry::UNOBSERVABLE)
  {
    request.observation_pose.header = movement.target_pose.header;
    request.observation_pose.pose.position = movement.target_pose.pose.position;
    const tf2::Quaternion fallback_rotation(
      tf2::Vector3(0.0, 0.0, 1.0), movement.unobserved_yaw_fallback_rad);
    request.observation_pose.pose.orientation = from_tf2(fallback_rotation);
  } else if (pick) {
    request.observation_pose = movement.target_pose;
  }
  auto object_pose = pick ? resolve_object_pose(request.observation_pose, geometry_) :
    movement.target_pose;
  if (pick && movement.constraints.has_estimated_object_height &&
    std::isfinite(movement.constraints.rectangle_long_side_m) &&
    movement.constraints.rectangle_long_side_m > 0.0F &&
    std::isfinite(movement.constraints.rectangle_short_side_m) &&
    movement.constraints.rectangle_short_side_m > 0.0F)
  {
    const auto rotation = to_tf2(request.observation_pose.pose.orientation);
    const auto inward = tf2::quatRotate(
      rotation,
      tf2::Vector3(
        0.0, 0.0,
      -static_cast<double>(movement.constraints.estimated_object_height_m) / 2.0));
    object_pose.pose.position.x = request.observation_pose.pose.position.x + inward.x();
    object_pose.pose.position.y = request.observation_pose.pose.position.y + inward.y();
    object_pose.pose.position.z = request.observation_pose.pose.position.z + inward.z();
  }
  request.object_pose = object_pose;
  request.target_pose = disturbance || movement.target_is_tcp_pose ? movement.target_pose :
    compose_target_pose(object_pose, geometry_);
  request.object_to_grasp_tcp = geometry_.object_to_grasp_tcp;
  request.pick_approach_axis = movement.constraints.pick_approach_axis;
  request.pick_grasp_position_tolerance_m = pick ?
    movement.constraints.position_tolerance_m : 0.0;
  request.pick_grasp_orientation_tolerance_rad = pick ?
    movement.constraints.orientation_tolerance_rad : 0.0;
  request.place_position_tolerance_m = place ? movement.constraints.position_tolerance_m : 0.0;
  request.place_orientation_tolerance_rad =
    place ? movement.constraints.orientation_tolerance_rad : 0.0;
  request.place_orientation_constraint = place ? movement.constraints.place_orientation_constraint :
    PlaceOrientationConstraint::FIXED;
  request.has_place_tool_orientation_preference =
    place && movement.has_place_tool_orientation_preference;
  request.place_tool_orientation_preference = movement.place_tool_orientation_preference;
  request.place_approach_direction_object = place ?
    movement.constraints.approach_direction_object : geometry_msgs::msg::Vector3{};
  request.insertion_axis_tcp = geometry_.insertion_axis_tcp;
  request.approach_distance_m = movement.constraints.approach_distance_m;
  request.retract_distance_m = movement.constraints.retract_distance_m;
  request.estimated_object_height_m = pick ? movement.constraints.estimated_object_height_m : 0.0;
  request.is_disturbance = disturbance;
  request.motion_speed_fraction = disturbance ? movement.constraints.motion_speed_fraction : 1.0;
  request.approach_target = pick ? pick_approach_pose(
    request.target_pose, request.pick_approach_axis,
    geometry_.insertion_axis_tcp, request.approach_distance_m) : offset_along_insertion(
    request.target_pose, geometry_, request.approach_distance_m);
  request.retract_target = offset_along_insertion(
    request.target_pose, geometry_, request.retract_distance_m);
  if (place && !disturbance && !movement.target_is_tcp_pose) {
    request.approach_target = compose_target_pose(
      offset_object_pose(
        request.object_pose, movement.constraints.approach_direction_object,
        movement.constraints.approach_distance_m), geometry_);
    request.retract_target = compose_target_pose(
      offset_object_pose(
        request.object_pose, movement.constraints.approach_direction_object,
        movement.constraints.retract_distance_m), geometry_);
  }
  if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_APPROACHING) {
    request.tcp_target = disturbance ? movement.target_pose : request.approach_target;
  } else if (request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_RETRACTING) {
    request.tcp_target = disturbance ? movement.target_pose : request.retract_target;
  } else {
    request.tcp_target = request.target_pose;
  }
  return request;
}

}  // namespace arm_cell_motion
