#include "arm_cell_orchestration_bt/cycle_coordinator.hpp"
#include "arm_cell_orchestration_bt/vision_request_timeout.hpp"
#include "arm_cell_orchestration_bt/execute_cycle_terminal.hpp"
#include "arm_cell_orchestration_bt/failure_trace_record.hpp"
#include "arm_cell_orchestration_bt/material_admission_gate.hpp"
#include "arm_cell_orchestration_bt/recovery_entry_latch.hpp"
#include "arm_cell_orchestration_bt/recovery_wait_predicate.hpp"
#include "arm_cell_orchestration_bt/recipe_repository.hpp"

#include <chrono>
#include <cstdint>
#include <condition_variable>
#include <cmath>
#include <future>
#include <iomanip>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arm_cell_interfaces/action/execute_cycle.hpp>
#include <arm_cell_interfaces/msg/material_readiness.hpp>

namespace arm_cell_orchestration_bt
{

class OrchestrationNode : public rclcpp::Node
{
public:
  using ExecuteCycle = arm_cell_interfaces::action::ExecuteCycle;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteCycle>;
  using MotionAction = arm_cell_interfaces::action::ExecuteTask;
  using DetectTarget = arm_cell_interfaces::srv::DetectTarget;
  using SafetyState = arm_cell_interfaces::msg::SafetyState;
  using MotionStatus = arm_cell_interfaces::msg::MotionStatus;
  using MaterialReadiness = arm_cell_interfaces::msg::MaterialReadiness;

  OrchestrationNode()
  : Node("orchestration_node")
  {
    mission_target_id_ = declare_parameter<std::string>("mission_target_id", "");
    const auto recipe_directory = declare_parameter<std::string>("recipe_directory", "");
    if (mission_target_id_.empty() || recipe_directory.empty()) {
      throw std::invalid_argument("mission target_id and recipe_directory are required");
    }
    recipe_repository_ = std::make_unique<RecipeRepository>(recipe_directory);
    RCLCPP_INFO(get_logger(), "recipe registry ready source=%s", recipe_directory.c_str());
    RCLCPP_DEBUG(get_logger(), "recipe registry identity directory=%s", recipe_directory.c_str());
    vision_request_timeout_ms_ = declare_parameter<int64_t>(
      "vision_request_timeout_ms", 500);
    if (vision_request_timeout_ms_ <= 0) {
      throw std::invalid_argument("vision_request_timeout_ms must be positive");
    }
    material_readiness_freshness_timeout_ms_ = declare_parameter<int>(
      "material_readiness_freshness_timeout_ms", 500);
    detect_client_ = create_client<DetectTarget>("/vision/detect_target");
    motion_client_ = rclcpp_action::create_client<MotionAction>(
      this, "/motion/execute_task");
    material_cycle_client_ = rclcpp_action::create_client<ExecuteCycle>(
      this, "/orchestration/execute_cycle");
    safety_subscription_ = create_subscription<SafetyState>(
      "/safety/state", rclcpp::QoS(10),
      [this](const SafetyState::SharedPtr message) {
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          latest_safety_ = *message;
          latest_safety_receipt_ = MaterialAdmissionGate::Clock::now();
          for (const auto & entry : recovery_entries_) {
            observe_recovery_entry(entry.first, latest_safety_, recovery_entries_);
          }
          ++recovery_generation_;
          recovery_cv_.notify_all();
        }
        attempt_material_admission();
      });
    material_readiness_subscription_ = create_subscription<MaterialReadiness>(
      "/integration/material_readiness",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const MaterialReadiness::SharedPtr message) {
        const auto source_age = std::chrono::nanoseconds(
          (now() - rclcpp::Time(
            message->header.stamp, get_clock()->get_clock_type())).nanoseconds());
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          material_admission_gate_.observe(
            *message, MaterialAdmissionGate::Clock::now(), source_age);
        }
        attempt_material_admission();
      });
    motion_subscription_ = create_subscription<MotionStatus>(
      "/motion/state", rclcpp::QoS(10),
      [this](const MotionStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(recovery_mutex_);
        latest_motion_ = *message;
        ++recovery_generation_;
        recovery_cv_.notify_all();
      });
    action_server_ = rclcpp_action::create_server<ExecuteCycle>(
      this,
      "/orchestration/execute_cycle",
      [this](const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ExecuteCycle::Goal> goal) {
        std::lock_guard<std::mutex> lock(recovery_mutex_);
        const auto delivery_key = delivery_identity_key(goal->delivery_id);
        if (goal->target_id != mission_target_id_ || delivery_key.empty() ||
        authorized_material_deliveries_.erase(delivery_key) == 0U)
        {
          RCLCPP_WARN(
            get_logger(),
            "mission rejected execute_cycle_id=%s delivery_id=%s target_id=%s reason=material_not_authorized",
            uuid_string(uuid).c_str(), uuid_string(goal->delivery_id.uuid).c_str(),
            goal->target_id.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(
          get_logger(), "mission accepted execute_cycle_id=%s delivery_id=%s target_id=%s",
          uuid_string(uuid).c_str(), uuid_string(goal->delivery_id.uuid).c_str(),
          goal->target_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        const auto key = uuid_key(handle->get_goal_id());
        const auto & goal = *handle->get_goal();
        RCLCPP_INFO(
          get_logger(), "mission cancellation requested execute_cycle_id=%s delivery_id=%s target_id=%s",
          uuid_string(handle->get_goal_id()).c_str(), uuid_string(goal.delivery_id.uuid).c_str(),
          goal.target_id.c_str());
        cancellation_registry_.request_caller_cancel(key);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        const auto key = uuid_key(handle->get_goal_id());
        {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          begin_recovery_entry(key, recovery_entries_);
        }
        auto vision_result_code = std::make_shared<uint8_t>(
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_INVALID_RESULT);
        auto coordinator = std::make_shared<CycleCoordinator>(
          make_callbacks(handle, vision_result_code));
        cancellation_registry_.register_execution(
          key, [coordinator]() {coordinator->set_cancel_requested(true);});
        std::thread(
          [this, handle, coordinator, key, vision_result_code]() {
            const auto & goal = *handle->get_goal();
            RCLCPP_INFO(
              get_logger(), "mission started execute_cycle_id=%s delivery_id=%s target_id=%s",
              uuid_string(handle->get_goal_id()).c_str(), uuid_string(goal.delivery_id.uuid).c_str(),
              goal.target_id.c_str());
            execute_goal(handle, coordinator, key, vision_result_code);
          }).detach();
      });
  }

private:
  static std::string uuid_key(const rclcpp_action::GoalUUID & uuid)
  {
    return std::string(uuid.begin(), uuid.end());
  }

  template<typename UuidT>
  static std::string uuid_string(const UuidT & uuid)
  {
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (const auto byte : uuid) {
      stream << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return stream.str();
  }

  static std::string delivery_identity_key(
    const unique_identifier_msgs::msg::UUID & uuid)
  {
    bool all_zero = true;
    std::string key;
    key.reserve(uuid.uuid.size());
    for (const auto byte : uuid.uuid) {
      all_zero = all_zero && byte == 0;
      key.push_back(static_cast<char>(byte));
    }
    return all_zero ? std::string{} : key;
  }

  void attempt_material_admission()
  {
    std::optional<MaterialAdmissionGate::DeliveryId> delivery_id;
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      delivery_id = material_admission_gate_.try_admit(
        MaterialAdmissionGate::Clock::now(), latest_safety_, latest_safety_receipt_,
        std::chrono::milliseconds(material_readiness_freshness_timeout_ms_));
      if (!delivery_id) {
        return;
      }
      authorized_material_deliveries_.insert(delivery_identity_key(*delivery_id));
    }

    ExecuteCycle::Goal goal;
    goal.target_id = mission_target_id_;
    goal.delivery_id = *delivery_id;
    const auto delivery_key = delivery_identity_key(*delivery_id);
    const auto delivery_text = uuid_string(delivery_id->uuid);
    rclcpp_action::Client<ExecuteCycle>::SendGoalOptions options;
    options.goal_response_callback = [this, delivery_key, delivery_text](
      const rclcpp_action::ClientGoalHandle<ExecuteCycle>::SharedPtr & goal_handle) {
        if (!goal_handle) {
          std::lock_guard<std::mutex> lock(recovery_mutex_);
          authorized_material_deliveries_.erase(delivery_key);
          RCLCPP_ERROR(
            get_logger(),
            "material admission action rejected delivery_id=%s reason=execute_cycle_goal_rejected",
            delivery_text.c_str());
          return;
        }
        RCLCPP_INFO(
          get_logger(), "material admission action accepted execute_cycle_id=%s delivery_id=%s",
          uuid_string(goal_handle->get_goal_id()).c_str(), delivery_text.c_str());
      };
    RCLCPP_INFO(
      get_logger(), "material admission action dispatched delivery_id=%s target_id=%s",
      delivery_text.c_str(), goal.target_id.c_str());
    material_cycle_client_->async_send_goal(goal, options);
  }

  void execute_goal(
    const std::shared_ptr<GoalHandle> & handle,
    const std::shared_ptr<CycleCoordinator> & coordinator,
    const std::string & key,
    const std::shared_ptr<uint8_t> & vision_result_code)
  {
    const auto result = coordinator->execute(handle->get_goal()->target_id);
    if (result.exit_reason.value ==
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR)
    {
      const auto & goal = *handle->get_goal();
      RCLCPP_ERROR(
        get_logger(),
        "mission terminated execute_cycle_id=%s delivery_id=%s target_id=%s "
        "exit_reason=%u reason=recipe_configuration_terminal diagnostic=%s",
        uuid_string(handle->get_goal_id()).c_str(),
        uuid_string(goal.delivery_id.uuid).c_str(), goal.target_id.c_str(),
        result.exit_reason.value, result.diagnostic_detail.c_str());
    }
    if (result.motion_failure) {
      const auto & failure = *result.motion_failure;
      const auto & goal = *handle->get_goal();
      const auto record = TerminalFailureRecord{
        uuid_string(handle->get_goal_id()), uuid_string(goal.delivery_id.uuid), goal.target_id,
        failure.motion_execution_id, failure.task_type, failure.code.value,
        result.exit_reason.value, failure.diagnostic_detail};
      emit_terminal_failure_record(
        record, [this](const std::string & line) {
          RCLCPP_ERROR(get_logger(), "%s", line.c_str());
        });
    } else if (result.exit_reason.value ==
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_DEPLETED)
    {
      const auto & goal = *handle->get_goal();
      RCLCPP_INFO(
        get_logger(), "mission completed execute_cycle_id=%s delivery_id=%s target_id=%s exit_reason=%u",
        uuid_string(handle->get_goal_id()).c_str(), uuid_string(goal.delivery_id.uuid).c_str(),
        goal.target_id.c_str(), result.exit_reason.value);
    } else if (result.exit_reason.value ==
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_CANCELED)
    {
      const auto & goal = *handle->get_goal();
      RCLCPP_INFO(
        get_logger(), "mission canceled execute_cycle_id=%s delivery_id=%s target_id=%s exit_reason=%u",
        uuid_string(handle->get_goal_id()).c_str(), uuid_string(goal.delivery_id.uuid).c_str(),
        goal.target_id.c_str(), result.exit_reason.value);
    } else if (result.exit_reason.value ==
      arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_CONFIGURATION_ERROR)
    {
      // The configuration terminal above owns this mission failure summary.
    } else {
      const auto & goal = *handle->get_goal();
      const bool vision_failure = result.exit_reason.value ==
        arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_VISION_ERROR &&
        *vision_result_code !=
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
      const bool safety_failure = result.exit_reason.value ==
        arm_cell_interfaces::msg::MissionExitReason::MISSION_EXIT_SAFETY_PREEMPTED;
      const char * causal_owner = vision_failure ? "Vision" :
        safety_failure ? "Safety" : "Orchestration";
      if (vision_failure) {
        RCLCPP_ERROR(
          get_logger(),
          "mission operation failed execute_cycle_id=%s delivery_id=%s target_id=%s exit_reason=%u causal_owner=%s causal_result_code=%u diagnostic=%s",
          uuid_string(handle->get_goal_id()).c_str(), uuid_string(goal.delivery_id.uuid).c_str(),
          goal.target_id.c_str(), result.exit_reason.value, causal_owner,
          *vision_result_code, result.diagnostic_detail.c_str());
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "mission operation failed execute_cycle_id=%s delivery_id=%s target_id=%s exit_reason=%u causal_owner=%s causal_result=MissionExitReason:%u diagnostic=%s",
          uuid_string(handle->get_goal_id()).c_str(), uuid_string(goal.delivery_id.uuid).c_str(),
          goal.target_id.c_str(), result.exit_reason.value, causal_owner,
          result.exit_reason.value, result.diagnostic_detail.c_str());
      }
    }
    auto action_result = std::make_shared<ExecuteCycle::Result>();
    action_result->exit_reason = result.exit_reason;
    action_result->diagnostic_detail = result.diagnostic_detail;
    complete_execute_cycle_goal(handle, action_result);

    cancellation_registry_.erase_execution(key);
    {
      std::lock_guard<std::mutex> lock(recovery_mutex_);
      erase_recovery_entry(key, recovery_entries_);
    }
  }

  CycleCoordinator::Callbacks make_callbacks(
    const std::shared_ptr<GoalHandle> & handle,
    const std::shared_ptr<uint8_t> & vision_result_code)
  {
    const auto key = uuid_key(handle->get_goal_id());
    auto previous_phase = std::make_shared<uint8_t>(
      arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_UNKNOWN);
    CycleCoordinator::Callbacks callbacks;
    auto selected_recipe_id = std::make_shared<std::string>();
    callbacks.detect_target = [this, handle, vision_result_code](
      const DetectTarget::Request & request) {
        const auto detected = detect_target(handle, request);
        *vision_result_code = detected.result_code.value;
        if (detected.result_code.value ==
          arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS)
        {
          RCLCPP_INFO(
            get_logger(), "target selected execute_cycle_id=%s delivery_id=%s target_id=%s",
            uuid_string(handle->get_goal_id()).c_str(),
            uuid_string(handle->get_goal()->delivery_id.uuid).c_str(), request.target_id.c_str());
        }
        return detected;
      };
    callbacks.execute_task = [this, handle, key, selected_recipe_id](const MotionAction::Goal & goal) {
        return execute_task(handle, key, goal, *selected_recipe_id);
      };
    callbacks.read_safety = [this, key]() {return read_safety(key);};
    callbacks.wait_for_recovery = [this]() {return wait_for_recovery();};
    callbacks.read_recovery_entry = [this, key]() {
        std::lock_guard<std::mutex> lock(recovery_mutex_);
        return read_recovery_entry(key, recovery_entries_);
      };
    callbacks.record_recovery_result = [this](const CycleCoordinator::MotionResult & result) {
        RCLCPP_WARN(
          get_logger(), "Controlled RETRACT terminated with result code %u: %s",
          result.code.value, result.diagnostic_detail.c_str());
      };
    callbacks.load_recipe = [this, selected_recipe_id](const std::string & target_id) {
        const auto loaded = load_recipe(target_id);
        if (!loaded.recipe) {
          RCLCPP_WARN(
            get_logger(), "recipe rejected target_id=%s source=%s reason=%s",
            target_id.c_str(), loaded.source.c_str(), recipe_reason_token(loaded.reason));
          return std::optional<CycleCoordinator::MissionRecipe>{};
        }
        const auto & recipe = *loaded.recipe;
        *selected_recipe_id = recipe.recipe_id;
        const auto & position = recipe.place_pose.pose.position;
        const auto & orientation = recipe.place_pose.pose.orientation;
        const auto & tool_orientation = recipe.place_tool_orientation_preference;
        const auto & direction = recipe.place_approach_direction_object;
        RCLCPP_DEBUG(
          get_logger(),
          "recipe parsed target_id=%s recipe_id=%s source=%s schema_version=%u "
          "grasp_width_mm=%.3f grasp_yaw_rad=%.9f place_frame=%s place_position=(%.9f,%.9f,%.9f) "
          "place_orientation_xyzw=(%.9f,%.9f,%.9f,%.9f) position_tolerance_m=%.9f "
          "orientation_tolerance_rad=%.9f tool_orientation_preference_present=%s "
          "tool_orientation_preference_xyzw=(%.9f,%.9f,%.9f,%.9f) "
          "approach_direction_object=(%.9f,%.9f,%.9f) "
          "approach_distance_m=%.9f retract_distance_m=%.9f",
          recipe.target_id.c_str(), recipe.recipe_id.c_str(), loaded.source.c_str(),
          recipe.schema_version, recipe.grasp_width_mm, recipe.grasp_yaw_rad,
          recipe.place_pose.header.frame_id.c_str(), position.x, position.y, position.z,
          orientation.x, orientation.y, orientation.z, orientation.w,
          recipe.place_position_tolerance_m, recipe.place_orientation_tolerance_rad,
          recipe.has_place_tool_orientation_preference ? "true" : "false",
          tool_orientation.x, tool_orientation.y, tool_orientation.z, tool_orientation.w,
          direction.x, direction.y, direction.z, recipe.place_approach_distance_m,
          recipe.retract_distance_m);
        RCLCPP_INFO(
          get_logger(), "mission recipe selected target_id=%s recipe_id=%s",
          recipe.target_id.c_str(), recipe.recipe_id.c_str());
        return loaded.recipe;
      };
    callbacks.cancel_motion = [this, key]() {
        cancellation_registry_.request_secondary_motion_cancel(key);
      };
    callbacks.publish_phase = [this, handle, previous_phase](uint8_t phase) {
        auto feedback = std::make_shared<ExecuteCycle::Feedback>();
        feedback->phase.value = phase;
        handle->publish_feedback(feedback);
        if (*previous_phase != phase) {
          const auto & goal = *handle->get_goal();
          const char * name = phase == arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_ENSURING_HOME ? "GO_HOME" :
            phase == arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_DETECTING_TARGET ? "DETECT_TARGET" :
            phase == arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_PICKING ? "PICK" :
            phase == arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_PLACING ? "PLACE" :
            phase == arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_RETURNING_HOME ? "GO_HOME" :
            phase == arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_WAITING_RECOVERY ? "WAIT_RECOVERY" :
            phase == arm_cell_interfaces::msg::MissionPhase::MISSION_PHASE_RETRACTING ? "RETRACT" : "FINISHED";
          RCLCPP_INFO(
            get_logger(), "mission phase transition execute_cycle_id=%s delivery_id=%s target_id=%s phase=%s",
            uuid_string(handle->get_goal_id()).c_str(), uuid_string(goal.delivery_id.uuid).c_str(),
            goal.target_id.c_str(), name);
          *previous_phase = phase;
        }
      };
    return callbacks;
  }

  CycleCoordinator::RecoveryObservation wait_for_recovery()
  {
    std::unique_lock<std::mutex> lock(recovery_mutex_);
    const auto current_motion = [this]() {return latest_motion_;};
    const auto immediate_return_ready = [this, &current_motion]() {
        return recovery_ready_for_immediate_return(latest_safety_, current_motion()) ||
               recovery_severity_escalation_ready_for_immediate_return(latest_safety_);
      };
    if (recovery_generation_ != last_recovery_returned_generation_ && immediate_return_ready()) {
      last_recovery_returned_generation_ = recovery_generation_;
      return {latest_safety_, current_motion()};
    }

    const auto generation = recovery_generation_;
    recovery_cv_.wait(lock, [this, generation]() {return recovery_generation_ != generation;});
    last_recovery_returned_generation_ = recovery_generation_;
    return {latest_safety_, current_motion()};
  }

  SafetyState read_safety(const std::string & key)
  {
    std::lock_guard<std::mutex> lock(recovery_mutex_);
    observe_recovery_entry(key, latest_safety_, recovery_entries_);
    return latest_safety_;
  }

  DetectTarget::Response detect_target(
    const std::shared_ptr<GoalHandle> & handle, const DetectTarget::Request & request)
  {
    DetectTarget::Response response;
    const auto & goal = *handle->get_goal();
    const auto execute_cycle_id = uuid_string(handle->get_goal_id());
    const auto delivery_id = uuid_string(goal.delivery_id.uuid);
    if (!detect_client_->wait_for_service(std::chrono::seconds(1))) {
      response.result_code.value =
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SENSOR_ERROR;
      response.diagnostic_detail = "Vision service unavailable";
      RCLCPP_WARN(
        get_logger(),
        "vision_request_timing execute_cycle_id=%s delivery_id=%s target_id=%s "
        "response_received=false result_code=%u diagnostic=%s",
        execute_cycle_id.c_str(), delivery_id.c_str(), request.target_id.c_str(),
        response.result_code.value, response.diagnostic_detail.c_str());
      return response;
    }
    auto request_ptr = std::make_shared<DetectTarget::Request>(request);
    if (request_ptr->timeout.sec == 0 && request_ptr->timeout.nanosec == 0) {
      request_ptr->timeout.sec = static_cast<int32_t>(vision_request_timeout_ms_ / 1000);
      request_ptr->timeout.nanosec = static_cast<uint32_t>(
        (vision_request_timeout_ms_ % 1000) * 1'000'000);
    }
    const auto request_timeout = arm_cell_orchestration_bt::vision_request_timeout(
      request_ptr->timeout);
    if (!request_timeout) {
      response.result_code.value =
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_INVALID_RESULT;
      response.diagnostic_detail = "Vision timeout must be a non-negative duration";
      RCLCPP_ERROR(
        get_logger(),
        "vision_request_timing execute_cycle_id=%s delivery_id=%s target_id=%s "
        "response_received=false result_code=%u diagnostic=%s",
        execute_cycle_id.c_str(), delivery_id.c_str(), request.target_id.c_str(),
        response.result_code.value, response.diagnostic_detail.c_str());
      return response;
    }
    RCLCPP_INFO(
      get_logger(),
      "vision request dispatched execute_cycle_id=%s delivery_id=%s target_id=%s",
      execute_cycle_id.c_str(), delivery_id.c_str(), request.target_id.c_str());
    auto future = detect_client_->async_send_request(request_ptr);
    const auto response_margin =
      arm_cell_orchestration_bt::vision_response_wait_timeout(*request_timeout) - *request_timeout;
    const auto orchestration_wait_start = std::chrono::steady_clock::now();
    if (future.wait_for(*request_timeout + response_margin) !=
      std::future_status::ready)
    {
      const auto response_wait_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - orchestration_wait_start).count();
      response.result_code.value =
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT;
      response.diagnostic_detail = "Vision request timed out";
      RCLCPP_WARN(
        get_logger(),
        "vision_request_timing execute_cycle_id=%s delivery_id=%s target_id=%s "
        "request_timeout_ms=%.1f response_margin_ms=%.1f "
        "response_wait_ms=%.1f response_received=false result_code=%u",
        execute_cycle_id.c_str(), delivery_id.c_str(), request_ptr->target_id.c_str(),
        std::chrono::duration<double, std::milli>(*request_timeout).count(),
        std::chrono::duration<double, std::milli>(response_margin).count(), response_wait_ms,
        response.result_code.value);
      return response;
    }
    auto vision_response = *future.get();
    const auto response_wait_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - orchestration_wait_start).count();
    RCLCPP_INFO(
      get_logger(),
      "vision_request_timing execute_cycle_id=%s delivery_id=%s target_id=%s "
      "request_timeout_ms=%.1f response_margin_ms=%.1f "
      "response_wait_ms=%.1f response_received=true result_code=%u",
      execute_cycle_id.c_str(), delivery_id.c_str(), request_ptr->target_id.c_str(),
      std::chrono::duration<double, std::milli>(*request_timeout).count(),
      std::chrono::duration<double, std::milli>(response_margin).count(), response_wait_ms,
      vision_response.result_code.value);
    return vision_response;
  }

  CycleCoordinator::MotionResult execute_task(
    const std::shared_ptr<GoalHandle> & mission_handle,
    const std::string & key, const MotionAction::Goal & goal,
    const std::string & recipe_id)
  {
    CycleCoordinator::MotionResult result;
    const auto & mission_goal = *mission_handle->get_goal();
    if (goal.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK) {
      RCLCPP_DEBUG(
        get_logger(),
        "MissionRecipe projected to ExecuteTask target_id=%s recipe_id=%s task=PICK "
        "grasp_width_mm=%.3f pick_yaw_rad=%.9f",
        mission_goal.target_id.c_str(), recipe_id.c_str(), goal.grasp_width_mm,
        std::atan2(2.0 * (goal.target_pose.pose.orientation.w * goal.target_pose.pose.orientation.z +
        goal.target_pose.pose.orientation.x * goal.target_pose.pose.orientation.y),
        1.0 - 2.0 * (goal.target_pose.pose.orientation.y * goal.target_pose.pose.orientation.y +
        goal.target_pose.pose.orientation.z * goal.target_pose.pose.orientation.z)));
    } else if (goal.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE) {
      const auto & pose = goal.target_pose;
      RCLCPP_DEBUG(
        get_logger(),
        "MissionRecipe projected to ExecuteTask target_id=%s recipe_id=%s task=PLACE "
        "frame=%s object_position=(%.9f,%.9f,%.9f) object_orientation_xyzw=(%.9f,%.9f,%.9f,%.9f) "
        "position_tolerance_m=%.9f orientation_tolerance_rad=%.9f "
        "tool_orientation_preference_present=%s preference_xyzw=(%.9f,%.9f,%.9f,%.9f) "
        "approach_direction_object=(%.9f,%.9f,%.9f) approach_distance_m=%.9f retract_distance_m=%.9f",
        mission_goal.target_id.c_str(), recipe_id.c_str(), pose.header.frame_id.c_str(),
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
        pose.pose.orientation.w, goal.place_position_tolerance_m,
        goal.place_orientation_tolerance_rad,
        goal.has_place_tool_orientation_preference ? "true" : "false",
        goal.place_tool_orientation_preference.x, goal.place_tool_orientation_preference.y,
        goal.place_tool_orientation_preference.z, goal.place_tool_orientation_preference.w,
        goal.place_approach_direction_object.x,
        goal.place_approach_direction_object.y, goal.place_approach_direction_object.z,
        goal.place_approach_distance_m, goal.place_retract_distance_m);
    }
    RCLCPP_INFO(
      get_logger(),
      "mission motion dispatch execute_cycle_id=%s delivery_id=%s target_id=%s recipe_id=%s task_type=%u",
      uuid_string(mission_handle->get_goal_id()).c_str(),
      uuid_string(mission_goal.delivery_id.uuid).c_str(), mission_goal.target_id.c_str(),
      recipe_id.c_str(), goal.task_type.value);
    result.code.value =
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_UNAVAILABLE;
    if (!motion_client_->wait_for_action_server(std::chrono::seconds(1))) {
      result.diagnostic_detail = "Motion action unavailable";
      return result;
    }
    rclcpp_action::Client<MotionAction>::SendGoalOptions options;
    auto goal_future = motion_client_->async_send_goal(goal, options);
    if (goal_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      result.diagnostic_detail = "Motion goal submission timed out";
      return result;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      result.diagnostic_detail = "Motion goal rejected";
      return result;
    }
    result.motion_execution_id = uuid_string(goal_handle->get_goal_id());
    RCLCPP_INFO(
      get_logger(),
      "mission motion accepted execute_cycle_id=%s delivery_id=%s target_id=%s motion_execution_id=%s task_type=%u",
      uuid_string(mission_handle->get_goal_id()).c_str(),
      uuid_string(mission_goal.delivery_id.uuid).c_str(), mission_goal.target_id.c_str(),
      result.motion_execution_id.c_str(), goal.task_type.value);
    cancellation_registry_.register_active_motion(
      key, [this, goal_handle]() {motion_client_->async_cancel_goal(goal_handle);});
    auto result_future = motion_client_->async_get_result(goal_handle);
    while (result_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      if (!motion_client_->action_server_is_ready() &&
        !motion_client_->wait_for_action_server(std::chrono::seconds(1)))
      {
        cancellation_registry_.clear_active_motion(key);
        result.diagnostic_detail = "Motion action server lost before terminal result";
        return result;
      }
    }
    const auto motion_result = result_future.get();
    result.code = motion_result.result->result_code;
    result.diagnostic_detail = motion_result.result->diagnostic_detail;
    if (result.code.value == arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      const char * operation =
        goal.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK ? "PICK" :
        goal.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PLACE ? "PLACE" :
        goal.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME ? "GO_HOME" :
        goal.task_type.value == arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_RETRACT ? "RETRACT" :
        "UNKNOWN";
      RCLCPP_INFO(
        get_logger(),
        "mission operation completed execute_cycle_id=%s delivery_id=%s target_id=%s recipe_id=%s motion_execution_id=%s operation=%s task_type=%u result_code=%u",
        uuid_string(mission_handle->get_goal_id()).c_str(),
        uuid_string(mission_goal.delivery_id.uuid).c_str(), mission_goal.target_id.c_str(),
        recipe_id.c_str(), result.motion_execution_id.c_str(), operation,
        goal.task_type.value, result.code.value);
    }
    cancellation_registry_.clear_active_motion(key);
    return result;
  }

  RecipeLoadResult load_recipe(
    const std::string & target_id) const
  {
    if (target_id != mission_target_id_) {
      return RecipeLoadResult{
        std::nullopt, RecipeDiagnosticReason::TARGET_MISMATCH, std::filesystem::path{}};
    }
    return recipe_repository_->load(target_id);
  }

  rclcpp::Client<DetectTarget>::SharedPtr detect_client_;
  rclcpp_action::Client<MotionAction>::SharedPtr motion_client_;
  rclcpp_action::Client<ExecuteCycle>::SharedPtr material_cycle_client_;
  rclcpp_action::Server<ExecuteCycle>::SharedPtr action_server_;
  rclcpp::Subscription<SafetyState>::SharedPtr safety_subscription_;
  rclcpp::Subscription<MaterialReadiness>::SharedPtr material_readiness_subscription_;
  rclcpp::Subscription<MotionStatus>::SharedPtr motion_subscription_;
  SafetyState latest_safety_;
  MaterialAdmissionGate::Clock::time_point latest_safety_receipt_{};
  MotionStatus latest_motion_;
  std::mutex recovery_mutex_;
  std::condition_variable recovery_cv_;
  uint64_t recovery_generation_{0};
  uint64_t last_recovery_returned_generation_{0};
  RecoveryEntryMap recovery_entries_;
  MaterialAdmissionGate material_admission_gate_;
  std::unordered_set<std::string> authorized_material_deliveries_;
  CancellationRegistry cancellation_registry_;
  std::string mission_target_id_;
  std::unique_ptr<RecipeRepository> recipe_repository_;
  int64_t vision_request_timeout_ms_{500};
  int material_readiness_freshness_timeout_ms_{500};
};

}  // namespace arm_cell_orchestration_bt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<arm_cell_orchestration_bt::OrchestrationNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
