#include <arm_cell_interfaces/action/execute_cycle.hpp>
#include <arm_cell_interfaces/action/execute_disturb.hpp>
#include <arm_cell_interfaces/action/execute_pnp.hpp>
#include <arm_cell_interfaces/msg/material_handoff_state.hpp>
#include <arm_cell_interfaces/msg/handoff_trigger.hpp>
#include <arm_cell_interfaces/msg/material_readiness.hpp>
#include <arm_cell_interfaces/msg/transfer_completion.hpp>
#include <arm_cell_interfaces/msg/detect_target_result_code.hpp>
#include <arm_cell_interfaces/msg/disturbance_candidate.hpp>
#include <arm_cell_interfaces/msg/disturbance_failure_reason.hpp>
#include <arm_cell_interfaces/msg/disturbance_outcome.hpp>
#include <arm_cell_interfaces/msg/disturbance_phase.hpp>
#include <arm_cell_interfaces/msg/mission_exit_reason.hpp>
#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/safety_cause_set.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>
#include <arm_cell_interfaces/msg/scene_stability.hpp>
#include <arm_cell_interfaces/msg/stop_mode.hpp>
#include <arm_cell_interfaces/msg/vision_detector_snapshot.hpp>
#include <arm_cell_interfaces/srv/reset_safety.hpp>
#include <arm_cell_interfaces/srv/detect_target.hpp>

int main()
{
  using namespace arm_cell_interfaces;

  msg::MaterialHandoffState handoff;
  handoff.delivery_id.uuid[0] = 1;
  handoff.phase = msg::MaterialHandoffState::HANDOFF_UNLOADED;
  handoff.valid = true;
  msg::MaterialReadiness readiness;
  readiness.delivery_id = handoff.delivery_id;
  readiness.material_type = "RawPart";
  readiness.quantity = 3;
  readiness.material_ready = true;
  readiness.valid = true;
  const bool delivery_round_trip = readiness.delivery_id == handoff.delivery_id &&
    readiness.material_type == "RawPart" && readiness.quantity == 3;

  msg::HandoffTrigger trigger;
  trigger.header.stamp.sec = 1;
  trigger.delivery_id = readiness.delivery_id;
  trigger.material_type = readiness.material_type;
  trigger.quantity = readiness.quantity;
  trigger.has_random_seed = true;
  trigger.random_seed = 42;
  trigger.valid = true;

  msg::TransferCompletion completion;
  completion.header.stamp = trigger.header.stamp;
  completion.delivery_id = trigger.delivery_id;
  completion.material_type = trigger.material_type;
  completion.requested_quantity = trigger.quantity;
  completion.supplied_quantity = trigger.quantity;
  completion.completed = true;
  completion.valid = true;
  completion.failure_reason = msg::TransferCompletion::TRANSFER_FAILURE_NONE;
  const bool transfer_contract = completion.completed && completion.valid &&
    completion.delivery_id == trigger.delivery_id &&
    completion.material_type == trigger.material_type &&
    completion.requested_quantity == trigger.quantity &&
    completion.supplied_quantity == trigger.quantity &&
    completion.failure_reason == msg::TransferCompletion::TRANSFER_FAILURE_NONE;
  const bool transfer_failure_values =
    msg::TransferCompletion::TRANSFER_FAILURE_SUPPLY == 1 &&
    msg::TransferCompletion::TRANSFER_FAILURE_CANCELED == 4;

  msg::SceneStability scene_stability;
  scene_stability.target_id = "rawpart";
  scene_stability.scene_generation = 4;
  scene_stability.status = msg::SceneStability::SCENE_STABILITY_STABLE;

  srv::ResetSafety::Request reset_request;
  reset_request.request_id.uuid[0] = 2;
  reset_request.operator_acknowledged = true;
  srv::ResetSafety::Response reset_response;
  // `applied` is a Safety policy result, independent of request-field realization.
  (void)reset_response.applied;

  action::ExecuteCycle::Goal goal;
  goal.target_id = "rawpart";
  goal.delivery_id = readiness.delivery_id;
  goal.quantity = readiness.quantity;
  action::ExecuteCycle::Feedback feedback;
  feedback.delivery_id = goal.delivery_id;
  feedback.quantity = goal.quantity;
  feedback.batch_processed_count = 1;
  feedback.retry_count = 0;
  feedback.selected_target_pose.header.frame_id = "base_link";
  feedback.selected_target_pose.pose.position.x = 0.1;
  feedback.has_selected_target_pose = true;
  feedback.target_valid = true;
  feedback.target_observation_stamp.sec = 1;
  feedback.interruption_count = 1;
  feedback.first_interruption_reason.value = msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED;
  feedback.first_interruption_causes.value = msg::SafetyCauseSet::COMMUNICATION_LOSS;
  feedback.first_interruption_stop_mode.value = msg::StopMode::STOP_MODE_IMMEDIATE;
  action::ExecutePnp::Goal pnp_goal;
  pnp_goal.vision_acquisition_id = "acq-1";
  pnp_goal.material_candidates.resize(2);
  pnp_goal.material_candidates[0].candidate_id = "candidate-1";
  pnp_goal.material_candidates[1].candidate_id = "candidate-2";
  pnp_goal.place_target_pose.header.frame_id = "base_link";
  pnp_goal.has_place_target_pose = true;
  action::ExecutePnp::Result pnp_result;
  pnp_result.pnp_outcome = action::ExecutePnp::Result::PNP_OUTCOME_COMPLETED;
  pnp_result.material_placed = true;
  action::ExecutePnp::Feedback pnp_feedback;
  pnp_feedback.next_acquisition_hint =
    action::ExecutePnp::Feedback::NEXT_ACQUISITION_HINT_PREFETCH_NEXT;
  const bool pnp_contract = pnp_goal.material_candidates.size() == 2 &&
    pnp_goal.has_place_target_pose &&
    pnp_goal.material_candidates[0].candidate_id != "" &&
    pnp_result.pnp_outcome == action::ExecutePnp::Result::PNP_OUTCOME_COMPLETED &&
    pnp_result.material_placed &&
    pnp_feedback.next_acquisition_hint ==
    action::ExecutePnp::Feedback::NEXT_ACQUISITION_HINT_PREFETCH_NEXT;

  srv::DetectTarget::Response detect_response;
  detect_response.acquisition_id = "acq-disturb-1";
  detect_response.scene_generation = 19;
  detect_response.disturbance_candidates.resize(1);
  auto & disturbance_candidate = detect_response.disturbance_candidates.front();
  disturbance_candidate.candidate_id = "unresolved-1";
  disturbance_candidate.category = msg::DisturbanceCandidate::CATEGORY_INCLINED_SURFACE;
  disturbance_candidate.center.header.frame_id = "base_link";
  disturbance_candidate.center.header.stamp.sec = 42;
  disturbance_candidate.center.pose.position.x = 0.1;
  disturbance_candidate.center.pose.orientation.w = 1.0;
  disturbance_candidate.observed_height_m = 0.035;
  const bool detect_disturbance_contract =
    detect_response.acquisition_id == "acq-disturb-1" &&
    detect_response.scene_generation == 19 &&
    detect_response.disturbance_candidates.size() == 1 &&
    disturbance_candidate.candidate_id == "unresolved-1" &&
    disturbance_candidate.center.header.frame_id == "base_link" &&
    disturbance_candidate.center.header.stamp.sec == 42 &&
    disturbance_candidate.observed_height_m == 0.035;

  action::ExecuteDisturb::Goal disturb_goal;
  disturb_goal.acquisition_id = detect_response.acquisition_id;
  disturb_goal.scene_generation = detect_response.scene_generation;
  disturb_goal.disturbance_candidates = detect_response.disturbance_candidates;
  action::ExecuteDisturb::Result disturb_result;
  disturb_result.outcome.value = msg::DisturbanceOutcome::OUTCOME_FAILED;
  disturb_result.failure_reason.value =
    msg::DisturbanceFailureReason::FAILURE_REASON_PLAN_EXHAUSTED;
  disturb_result.safety_cause.value = msg::SafetyCauseSet::E_STOP;
  disturb_result.backend_inactivity_confirmed = true;
  action::ExecuteDisturb::Feedback disturb_feedback;
  disturb_feedback.phase.value = msg::DisturbancePhase::PHASE_PUSHING;
  const bool disturb_action_contract =
    disturb_goal.acquisition_id == detect_response.acquisition_id &&
    disturb_goal.scene_generation == detect_response.scene_generation &&
    disturb_goal.disturbance_candidates.size() == 1 &&
    disturb_result.outcome.value == msg::DisturbanceOutcome::OUTCOME_FAILED &&
    disturb_result.failure_reason.value ==
    msg::DisturbanceFailureReason::FAILURE_REASON_PLAN_EXHAUSTED &&
    disturb_result.safety_cause.value == msg::SafetyCauseSet::E_STOP &&
    disturb_result.backend_inactivity_confirmed &&
    disturb_feedback.phase.value == msg::DisturbancePhase::PHASE_PUSHING;
  const bool mission_exit_values =
    msg::MissionExitReason::MISSION_EXIT_DEPLETED == 1 &&
    msg::MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR == 6 &&
    msg::MissionExitReason::MISSION_EXIT_QUANTITY_REACHED == 7 &&
    msg::MissionExitReason::MISSION_EXIT_QUANTITY_SHORTFALL == 8;

  msg::SafetyState safety;
  safety.motion_envelope_valid = true;
  safety.max_velocity_scale = 0.5F;
  safety.max_acceleration_scale = 0.25F;
  msg::MotionStatus motion;
  motion.holding_state = msg::MotionStatus::HOLDING_HELD;
  motion.applied_velocity_scale = safety.max_velocity_scale;
  motion.applied_acceleration_scale = safety.max_acceleration_scale;
  motion.applied_envelope_valid = true;

  msg::DetectTargetResultCode result_code;
  result_code.value = msg::DetectTargetResultCode::DETECT_RESULT_BUSY;
  const bool detect_result_code_contract =
    msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET == 7 &&
    msg::DetectTargetResultCode::DETECT_RESULT_BUSY == 8 && result_code.value == 8;
  msg::VisionDetectorSnapshot detector_snapshot;
  detector_snapshot.snapshot_id = "run:1";
  detector_snapshot.revision = 1;
  detector_snapshot.event_kind = msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS;
  detector_snapshot.vision_run_id = "run";
  detector_snapshot.image_png.format = "png";
  detector_snapshot.metadata_json = "{}";
  detector_snapshot.diagnostic_record_json = "{}";
  const bool detector_snapshot_contract =
    msg::VisionDetectorSnapshot::CALIBRATION_COMPLETE == 0 &&
    msg::VisionDetectorSnapshot::DETECT_TARGET_PROGRESS == 1 &&
    msg::VisionDetectorSnapshot::DETECT_TARGET_SUCCESS == 2 &&
    msg::VisionDetectorSnapshot::DETECT_TARGET_FAILURE == 3 &&
    msg::VisionDetectorSnapshot::CALIBRATION_PROGRESS == 4 &&
    detector_snapshot.revision == 1 && detector_snapshot.image_png.format == "png" &&
    detector_snapshot.diagnostic_record_json == "{}";

  return delivery_round_trip && trigger.valid &&
         trigger.has_random_seed && trigger.random_seed == 42 && transfer_contract &&
         transfer_failure_values && mission_exit_values &&
         scene_stability.status == msg::SceneStability::SCENE_STABILITY_STABLE &&
         scene_stability.scene_generation == 4 &&
         reset_request.operator_acknowledged &&
         feedback.quantity == goal.quantity &&
         feedback.delivery_id == goal.delivery_id && safety.motion_envelope_valid &&
         motion.applied_envelope_valid && feedback.has_selected_target_pose &&
         feedback.first_interruption_causes.value == msg::SafetyCauseSet::COMMUNICATION_LOSS &&
         detect_result_code_contract && detector_snapshot_contract && pnp_contract &&
         detect_disturbance_contract && disturb_action_contract ? 0 : 1;
}
