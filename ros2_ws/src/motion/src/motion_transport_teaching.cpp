#include "arm_cell_motion/isaac_ros_motion_transport.hpp"
#include "arm_cell_motion/trajectory_generation.hpp"
#include "arm_cell_motion/trajectory_motion_bt.hpp"
#include "arm_cell_motion/motion_result_code_name.hpp"
#include "arm_cell_motion/pick_retract_collision_policy.hpp"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <nlohmann/json.hpp>
#include <algorithm>

namespace arm_cell_motion
{
uint8_t IsaacRosMotionTransport::execute_teaching_motion(
  const TaughtMotion & taught, const std::string & motion_name, const std::string & execution_id,
  const std::function<uint8_t()> & interruption,
  const std::vector<PreparedTeachingEntry> * prepared)
{
  using Code = arm_cell_interfaces::msg::MotionTaskResultCode;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    normalized_failure_result_ = Code::TASK_RESULT_BACKEND_ERROR;
    last_executed_path_selection_ = {execution_id,
      arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING, "TAUGHT"};
  }
  const bool joint_target = taught.cartesian_waypoints.empty();
  if (!joint_target && !teaching_submission_check_) {return Code::TASK_RESULT_BACKEND_UNAVAILABLE;}
  auto waypoints = taught.cartesian_waypoints;
  if (joint_target) {
    CartesianTeachingWaypoint waypoint;
    waypoint.motion = taught.movement;
    waypoints.push_back(waypoint);
  }
  std::vector<trajectory_msgs::msg::JointTrajectory> blend_segments;
  sensor_msgs::msg::JointState blend_start;
  bool blend_check_requested = false;
  for (std::size_t index = 0; index < waypoints.size(); ++index) {
    const auto first_waypoint_number = index + 1 - blend_segments.size();
    const auto permission = [&]() -> uint8_t {
        if (interruption) {
          const auto cause = interruption();
          if (cause != Code::TASK_RESULT_SUCCESS) {return cause;}
        }
        return teaching_submission_check_ ? teaching_submission_check_() : Code::TASK_RESULT_SUCCESS;
      };
    if (const auto cause = permission(); cause != Code::TASK_RESULT_SUCCESS) {return cause;}
    sensor_msgs::msg::JointState measured;
    if (!measured_joint_state(measured) || !current_state_fresh()) {
      return Code::TASK_RESULT_BACKEND_UNAVAILABLE;
    }
    const auto envelope = applied_motion_envelope();
    if (!envelope.valid) {return Code::TASK_RESULT_PERMISSION_DENIED;}
    const auto scales = apply_motion_envelope_to_planning_scales(
      taught.velocity_scale, taught.acceleration_scale, envelope);
    std::vector<double> start_positions;
    if (!extract_arm_positions(measured, start_positions))
    {return Code::TASK_RESULT_BACKEND_UNAVAILABLE;}
    if (taught_joint_limits_.size() != arm_joint_names_.size())
    {return Code::TASK_RESULT_INVALID_GOAL;}
    auto limits = taught_joint_limits_;
    for (auto & limit : limits) {
      limit.max_velocity *= scales.max_velocity;
      limit.max_acceleration *= scales.max_acceleration;
    }
    const PreparedTeachingEntry * cached = prepared && blend_segments.empty() ?
      find_prepared_teaching(*prepared, index, start_positions, trajectory_goal_tolerance_rad_) :
      nullptr;
    trajectory_msgs::msg::JointTrajectory cached_trajectory;
    if (cached) {
      cached_trajectory = cached->trajectory;
      std::vector<double> velocities, accelerations;
      for (const auto & limit : limits) {
        velocities.push_back(limit.max_velocity);
        accelerations.push_back(limit.max_acceleration);
      }
      if (!stretch_taught_trajectory_to_limits(cached_trajectory, velocities, accelerations)) {
        cached = nullptr;
      }
    }
    if (cached) {index = cached->last_waypoint;}
    const auto & waypoint = waypoints[index];
    const auto waypoint_number = index + 1;
    const bool blended = !cached && (waypoint.blend || !blend_segments.empty());
    if (blended && blend_segments.empty()) {
      blend_start = measured;
      blend_check_requested = false;
    }
    if (!blend_segments.empty()) {
      start_positions = blend_segments.back().points.back().positions;
    }
    moveit::core::RobotModelConstPtr model;
    std::unique_ptr<moveit::core::RobotState> start;
    const auto & joint_goal = joint_target ? taught.joint_positions : waypoint.joint_positions;
    if (!cached && (blended || joint_goal.empty() || waypoint.motion != "MoveJ")) {
      std::lock_guard<std::mutex> lock(move_group_state_->mutex);
      model = teaching_model_locked();
      if (!model) {return Code::TASK_RESULT_BACKEND_UNAVAILABLE;}
      start = std::make_unique<moveit::core::RobotState>(model);
      start->setToDefaultValues();
      if (!moveit::core::jointStateToRobotState(measured, *start))
      {return Code::TASK_RESULT_BACKEND_UNAVAILABLE;}
      const auto * joints = model->getJointModelGroup(planning_group_);
      if (!joints || joints->getVariableNames() != arm_joint_names_) {
        return Code::TASK_RESULT_INVALID_GOAL;
      }
      if (blended) {start->setJointGroupPositions(joints, start_positions);}
      start->update();
      if (!start->satisfiesBounds(joints)) {return Code::TASK_RESULT_INVALID_GOAL;}
    }
    geometry_msgs::msg::Pose pose;
    pose.position.x = waypoint.pose[0]; pose.position.y = waypoint.pose[1];
    pose.position.z = waypoint.pose[2]; pose.orientation.x = waypoint.pose[3];
    pose.orientation.y = waypoint.pose[4]; pose.orientation.z = waypoint.pose[5];
    pose.orientation.w = waypoint.pose[6];
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
      pose.orientation.y, pose.orientation.z).toRotationMatrix();
    target.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    if (start) {target = start->getGlobalLinkTransform("base_link") * target;}
    NormalizedTrajectoryPlan plan;
    plan.planning_scales = scales;
    plan.request.task_type.value = arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_GO_HOME;
    plan.request.phase = arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING;
    plan.request.motion_goal_uuid = execution_id;
    plan.request.interruption_result = interruption;
    plan.start_state = blended ? blend_start : measured;
    std::vector<double> target_joints;
    std::string detail;
    bool scene_collision_checked = false;
    LinearJointChangeDiagnostics linear_joint_change;
    if (cached) {
      plan.trajectory = std::move(cached_trajectory);
      target_joints = cached->target_joints;
      linear_joint_change = cached->linear_joint_change;
    }
    const auto record = [&](const std::string & stage, uint8_t code, bool terminal = false) {
        if (!diagnostics_capture_enabled_) {return;}
        const auto duration = plan.trajectory.points.empty() ? 0.0 :
          plan.trajectory.points.back().time_from_start.sec +
          plan.trajectory.points.back().time_from_start.nanosec * 1e-9;
        auto evidence = nlohmann::json{{"motion", waypoint.motion}, {"motion_name", motion_name},
          {"planner", waypoint.motion == "MovePlanned" ? "moveit" : "not_used"},
          {"waypoint_number", waypoint_number}, {"start_joint_positions", start_positions},
          {"target_joint_positions", target_joints},
          {"velocity_scale", scales.max_velocity}, {"acceleration_scale", scales.max_acceleration},
          {"duration_s", duration},
          {"target_tcp_pose", joint_target ? nlohmann::json(nullptr) : nlohmann::json(
              waypoint.pose)},
          {"trajectory_point_count", plan.trajectory.points.size()},
          {"failure_stage", stage}, {"detail", detail}, {"result_code", code},
          {"scene_collision_checked", scene_collision_checked},
          {"scene_collision_check_requested", waypoint.motion == "MoveJChecked" ||
            waypoint.motion == "MoveLChecked" || waypoint.motion == "MovePlanned"}};
        evidence["movement_phase"] = terminal ? "waypoint_terminal" : "waypoint_prepared";
        evidence["trajectory_source"] = cached ? "prepared_memory" : "live_planning";
        if (cached && first_waypoint_number != waypoint_number) {
          evidence["blend_first_waypoint_number"] = first_waypoint_number;
        }
        if (blended) {
          evidence["blend_first_waypoint_number"] = first_waypoint_number;
          evidence["start_joint_positions"] = nlohmann::json::array();
          std::vector<double> group_start_positions;
          if (extract_arm_positions(blend_start, group_start_positions)) {
            evidence["start_joint_positions"] = group_start_positions;
          }
          evidence["scene_collision_check_requested"] = blend_check_requested;
        }
        if (waypoint.motion == "MoveL" || waypoint.motion == "MoveLChecked") {
          evidence["linear_joint_change"] = {
            {"available", linear_joint_change.available}, {"execution_gate", false},
            {"relative_threshold_factor", linear_joint_change.relative_threshold_factor},
            {"distance_metric", "moveit_joint_model_group_distance"},
            {"average_step_distance", linear_joint_change.average_step_distance},
            {"maximum_step_distance", linear_joint_change.maximum_step_distance},
            {"maximum_step_end_index", linear_joint_change.available ?
              nlohmann::json(linear_joint_change.maximum_step_end_index) : nlohmann::json(nullptr)},
            {"exceeding_step_count", linear_joint_change.exceeding_step_count},
            {"joint_names", linear_joint_change.joint_names},
            {"maximum_step_joint_deltas", linear_joint_change.maximum_step_joint_deltas}};
        }
        if (terminal) {
          sensor_msgs::msg::JointState feedback;
          if (measured_joint_state(feedback)) {
            evidence["measured_joint_feedback"] = {
              {"status", "observed"}, {"names", feedback.name},
              {"positions_rad", feedback.position}, {"velocities_rad_s", feedback.velocity},
              {"source_stamp_sec", feedback.header.stamp.sec},
              {"source_stamp_nanosec", feedback.header.stamp.nanosec}};
          } else {evidence["measured_joint_feedback"] = {{"status", "unavailable"}};}
          if (planning_diagnostic_capture_) {
            planning_diagnostic_capture_->enqueue(
              execution_id, "TAUGHT_WAYPOINT", code,
              evidence.dump());
          }
        }
        set_pending_motion_planning_evidence(
          execution_id, plan.request.task_type.value,
          evidence.dump());
      };
    TrajectoryMotionOperations operations;
    operations.interruption = permission;
    operations.resolve_joint_target = [&]() -> uint8_t {
        if (!joint_goal.empty()) {
          target_joints = joint_goal;
          return Code::TASK_RESULT_SUCCESS;
        }
        std::lock_guard<std::mutex> lock(move_group_state_->mutex);
        return resolve_pose_joint_target(*start, planning_group_, "sf_grasp_tcp", target, target_joints);
      };
    operations.generate_joint_trajectory = [&]() -> uint8_t {
        const auto generated = make_taught_joint_trajectory(arm_joint_names_, start_positions,
          target_joints, taught_joint_limits_, scales.max_velocity, scales.max_acceleration);
        if (!generated.valid) {detail = generated.reason; return Code::TASK_RESULT_INVALID_GOAL;}
        plan.trajectory = generated.trajectory;
        return Code::TASK_RESULT_SUCCESS;
      };
    const auto retime = [&](moveit_msgs::msg::RobotTrajectory & trajectory) -> uint8_t {
        if (trajectory.joint_trajectory.points.size() < 2U ||
          !retime_taught_entry(model, planning_group_, *start, trajectory, scales, &taught_joint_limits_))
        {detail = "trajectory_time_parameterization_failed"; return Code::TASK_RESULT_BACKEND_ERROR;}
        plan.trajectory = trajectory.joint_trajectory;
        return Code::TASK_RESULT_SUCCESS;
      };
    operations.generate_linear_trajectory = [&]() -> uint8_t {
        std::lock_guard<std::mutex> lock(move_group_state_->mutex);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const auto result = generate_linear_joint_trajectory(
          *start, planning_group_, "sf_grasp_tcp", target, trajectory, interruption,
          diagnostics_capture_enabled_ ? &linear_joint_change : nullptr);
        if (result != Code::TASK_RESULT_SUCCESS) {return result;}
        if (!blended) {return retime(trajectory);}
        plan.trajectory = std::move(trajectory.joint_trajectory);
        return Code::TASK_RESULT_SUCCESS;
      };
    const auto check_trajectory_collision = [&]() -> uint8_t {
        if (!planning_scene_client_ ||
          !planning_scene_client_->wait_for_service(std::chrono::milliseconds(500)))
        {return Code::TASK_RESULT_BACKEND_UNAVAILABLE;}
        auto query = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        query->components.components = pick_retract_planning_scene_components();
        auto future = planning_scene_client_->async_send_request(query);
        if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready)
        {return Code::TASK_RESULT_BACKEND_UNAVAILABLE;}
        const auto response = future.get();
        if (!response) {return Code::TASK_RESULT_BACKEND_ERROR;}
        planning_scene::PlanningScene scene(model);
        if (!scene.setPlanningSceneMsg(response->scene)) {return Code::TASK_RESULT_BACKEND_ERROR;}
        moveit_msgs::msg::RobotState start_message;
        moveit::core::robotStateToRobotStateMsg(*start, start_message, true);
        moveit_msgs::msg::RobotTrajectory trajectory;
        trajectory.joint_trajectory = plan.trajectory;
        const bool valid = scene.isPathValid(start_message, trajectory, planning_group_);
        scene_collision_checked = true;
        if (!valid) {
          detail = "trajectory_collision_or_invalid_state";
          return Code::TASK_RESULT_PATH_OBSTRUCTED;
        }
        return Code::TASK_RESULT_SUCCESS;
      };
    operations.check_trajectory_collision = [&]() -> uint8_t {
        if (!blended) {return check_trajectory_collision();}
        blend_check_requested = true;
        return Code::TASK_RESULT_SUCCESS;
      };
    operations.plan_collision_free_path = [&]() -> uint8_t {
        {
          std::lock_guard<std::mutex> lock(move_group_state_->mutex);
          auto & group = *move_group_state_->move_group;
          group.setStartState(*start);
          group.setPoseReferenceFrame("base_link");
          if (!group.setEndEffectorLink("sf_grasp_tcp")) {return Code::TASK_RESULT_BACKEND_ERROR;}
          group.clearPoseTargets(); group.clearPathConstraints();
          group.setMaxVelocityScalingFactor(scales.max_velocity);
          group.setMaxAccelerationScalingFactor(scales.max_acceleration);
          group.setPlanningTime(planning_time_s_);
          const bool target_set = joint_target ? group.setJointValueTarget(taught.joint_positions) :
            group.setPoseTarget(pose, "sf_grasp_tcp");
          if (!target_set) {return Code::TASK_RESULT_INVALID_GOAL;}
          moveit::planning_interface::MoveGroupInterface::Plan planned;
          const auto result = group.plan(planned);
          group.clearPoseTargets();
          if (result != moveit::core::MoveItErrorCode::SUCCESS) {
            detail = "moveit_error_code=" + std::to_string(result.val);
            return Code::TASK_RESULT_PATH_OBSTRUCTED;
          }
          const auto timed = retime(planned.trajectory_);
          if (timed != Code::TASK_RESULT_SUCCESS) {return timed;}
        }
        return operations.check_trajectory_collision();
      };
    operations.execute_trajectory = [&]() -> uint8_t {
        if (blended) {
          blend_segments.push_back(std::move(plan.trajectory));
          if (waypoint.blend) {return Code::TASK_RESULT_SUCCESS;}

          moveit_msgs::msg::RobotTrajectory trajectory;
          if (!blend_taught_joint_paths(blend_segments, trajectory)) {
            detail = "invalid_teaching_blend_junction";
            return Code::TASK_RESULT_INVALID_GOAL;
          }
          if (!moveit::core::jointStateToRobotState(blend_start, *start)) {
            return Code::TASK_RESULT_BACKEND_UNAVAILABLE;
          }
          start->update();
          if (const auto timed = retime(trajectory); timed != Code::TASK_RESULT_SUCCESS) {
            return timed;
          }
          if (blend_check_requested) {
            const auto checked = check_trajectory_collision();
            if (checked != Code::TASK_RESULT_SUCCESS) {return checked;}
          }
          sensor_msgs::msg::JointState latest;
          std::vector<double> actual;
          if (const auto cause = permission(); cause != Code::TASK_RESULT_SUCCESS) {return cause;}
          if (!measured_joint_state(latest) || !current_state_fresh() ||
            !extract_arm_positions(latest, actual))
          {return Code::TASK_RESULT_BACKEND_UNAVAILABLE;}
          const auto & initial = plan.trajectory.points.front().positions;
          if (!std::equal(initial.begin(), initial.end(), actual.begin(),
              [this](double a, double b) {return std::abs(a - b) <= trajectory_goal_tolerance_rad_;}))
          {
            detail = "teaching_blend_start_changed";
            return Code::TASK_RESULT_INVALID_GOAL;
          }
        }
        if (const auto cause = permission(); cause != Code::TASK_RESULT_SUCCESS) {return cause;}
        record("", Code::TASK_RESULT_SUCCESS);
        const auto discard = [&] {
            const auto cause = permission();
            if (cause == Code::TASK_RESULT_SUCCESS) {return false;}
            std::lock_guard<std::mutex> lock(mutex_);
            normalized_failure_result_ = cause;
            return true;
          };
        if (!execute_taught_entry(plan, discard, limits)) {
          const auto cause = permission();
          return cause == Code::TASK_RESULT_SUCCESS ? normalized_failure_result() : cause;
        }
        return wait_for_joint_target_result();
      };
    auto result = TrajectoryMotionResult{Code::TASK_RESULT_SUCCESS, {}};
    if (cached) {
      result.code = operations.execute_trajectory();
      if (result.code != Code::TASK_RESULT_SUCCESS) {result.failure_stage = "ExecuteTrajectory";}
    } else {
      result = run_trajectory_motion(waypoint.motion, operations);
    }
    if (!waypoint.blend || result.code != Code::TASK_RESULT_SUCCESS) {
      record(result.failure_stage, result.code, true);
    }
    if (result.code != Code::TASK_RESULT_SUCCESS) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Taught motion failed motion=%s waypoint=%zu stage=%s result=%s detail=%s",
        waypoint.motion.c_str(), waypoint_number, result.failure_stage.c_str(),
        motion_result_code_name(result.code), detail.c_str());
      return result.code;
    }
    if (!waypoint.blend) {blend_segments.clear();}
  }
  return blend_segments.empty() ? Code::TASK_RESULT_SUCCESS : Code::TASK_RESULT_INVALID_GOAL;
}
}  // namespace arm_cell_motion
