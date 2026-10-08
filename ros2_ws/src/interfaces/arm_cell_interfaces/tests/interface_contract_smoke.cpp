#include <arm_cell_interfaces/action/execute_cycle.hpp>
#include <arm_cell_interfaces/msg/material_handoff_state.hpp>
#include <arm_cell_interfaces/msg/material_readiness.hpp>
#include <arm_cell_interfaces/msg/detect_target_result_code.hpp>
#include <arm_cell_interfaces/msg/mission_exit_reason.hpp>
#include <arm_cell_interfaces/msg/motion_status.hpp>
#include <arm_cell_interfaces/msg/safety_cause_set.hpp>
#include <arm_cell_interfaces/msg/safety_state.hpp>
#include <arm_cell_interfaces/msg/stop_mode.hpp>
#include <arm_cell_interfaces/srv/request_material.hpp>
#include <arm_cell_interfaces/srv/request_material_supply.hpp>
#include <arm_cell_interfaces/srv/reset_safety.hpp>

int main()
{
  using namespace arm_cell_interfaces;

  srv::RequestMaterial::Request request;
  request.delivery_id.uuid[0] = 1;
  srv::RequestMaterial::Response response;
  response.accepted = true;
  const bool request_round_trip = response.accepted && request.delivery_id.uuid[0] == 1;

  msg::MaterialHandoffState handoff;
  handoff.delivery_id = request.delivery_id;
  handoff.phase = msg::MaterialHandoffState::HANDOFF_UNLOADED;
  handoff.valid = true;
  msg::MaterialReadiness readiness;
  readiness.delivery_id = handoff.delivery_id;
  readiness.material_ready = true;
  readiness.valid = true;
  const bool delivery_round_trip = readiness.delivery_id == handoff.delivery_id;

  srv::ResetSafety::Request reset_request;
  reset_request.request_id.uuid[0] = 2;
  reset_request.operator_acknowledged = true;
  srv::ResetSafety::Response reset_response;
  // `applied` is a Safety policy result, independent of request-field realization.
  (void)reset_response.applied;

  srv::RequestMaterialSupply::Request material_supply_request;
  material_supply_request.request_id.uuid[0] = 3;
  srv::RequestMaterialSupply::Response material_supply_response;
  material_supply_response.accepted = true;
  material_supply_response.delivery_id.uuid[0] = 4;
  material_supply_response.diagnostic_detail = "accepted";
  const bool material_supply_contract =
    material_supply_request.request_id.uuid[0] == 3 &&
    material_supply_response.accepted &&
    material_supply_response.delivery_id.uuid[0] == 4 &&
    material_supply_response.diagnostic_detail == "accepted";

  action::ExecuteCycle::Goal goal;
  goal.target_id = "rawpart";
  goal.delivery_id = readiness.delivery_id;
  action::ExecuteCycle::Feedback feedback;
  feedback.delivery_id = goal.delivery_id;
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
  result_code.value = msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET;

  return request_round_trip && delivery_round_trip && reset_request.operator_acknowledged &&
         material_supply_contract &&
         feedback.delivery_id == goal.delivery_id && safety.motion_envelope_valid &&
         motion.applied_envelope_valid && feedback.has_selected_target_pose &&
         feedback.first_interruption_causes.value == msg::SafetyCauseSet::COMMUNICATION_LOSS &&
         result_code.value == 7 ? 0 : 1;
}
