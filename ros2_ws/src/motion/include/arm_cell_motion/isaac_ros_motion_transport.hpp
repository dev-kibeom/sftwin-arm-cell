#pragma once

#include "arm_cell_motion/grasp_collision_validator.hpp"

#include <chrono>
#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <pluginlib/class_loader.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

#include "arm_cell_motion/isaac_motion_backend.hpp"
#include "arm_cell_motion/trajectory_execution_worker.hpp"
#include "arm_cell_motion/trajectory_watchdog.hpp"
#include "arm_cell_motion/clock_diagnostic_capture.hpp"
#include "arm_cell_motion/motion_progress_watchdog.hpp"
#include "arm_cell_motion/pick_collision_seed_policy.hpp"
#include "arm_cell_motion/motion_tuning.hpp"
#include "arm_cell_motion/motion_planning_diagnostic_capture.hpp"

namespace arm_cell_motion
{

struct MoveGroupInitializationState
{
  mutable std::mutex mutex;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> pick_ik_loader;
  kinematics::KinematicsBasePtr pick_ik_solver;
  rclcpp::Node::SharedPtr pick_ik_node;
  bool required{false};
  bool ready{false};
  bool in_progress{false};
  std::string failure_reason;
};

class IsaacRosMotionTransport final : public IsaacMotionTransport
{
public:
  IsaacRosMotionTransport(
    rclcpp::Node::SharedPtr node,
    std::vector<std::string> arm_joint_names,
    std::vector<double> home_joint_positions,
    std::vector<double> retract_joint_positions,
    bool initialize_move_group = true,
    bool validation_grasp_contact_policy_enabled = false,
    std::string validation_grasp_contact_object_id_prefix = "",
    std::vector<std::string> validation_grasp_contact_links = {},
    double planning_max_acceleration_rad_s2 = 0.0,
    std::string trajectory_sampler_name = "linear",
    double trajectory_clock_liveness_timeout_s = 2.0,
    double cartesian_tcp_orientation_tolerance_rad = 0.08726646259971647,
    double motion_progress_stall_timeout_s = 60.0,
    double trajectory_goal_tolerance_rad = 0.03,
    double planning_velocity_scaling_factor = 0.1,
    double planning_acceleration_scaling_factor = 0.1,
    double planning_time_s = 5.0,
    double place_candidate_planning_time_s = 1.0,
    double place_total_planning_time_s = 5.0,
    bool diagnostics_capture_enabled = false,
    std::string diagnostics_output_directory = ".local_artifacts/observability",
    std::string snapshot_run_directory = ".local_artifacts/snapshots",
    std::string planning_planner_id = "RRTConnect",
    std::string place_waypoint_trajectory_file = "");

  ~IsaacRosMotionTransport() override;

  void initialize_move_group();
  void configure_waypoint_mode(bool pick_waypoint, bool place_waypoint);

  bool available() const override;
  void begin_pick_search() override;
  void begin_pick_search(const std::string & execution_id) override;
  void begin_pick_candidate_search() override;
  void record_material_candidate_search(
    const std::vector<MaterialCandidateSearchAttempt> & attempts,
    const std::string & outcome) override;
  bool apply_runtime_tuning(const MotionTuning & tuning) override;
  bool set_motion_envelope(const MotionEnvelope & envelope) override;
  MotionEnvelope applied_motion_envelope() const override;
  bool measured_tcp_pose(geometry_msgs::msg::PoseStamped & pose) const override;
  uint8_t generate_place_waypoint_approach(
    const NormalizedMotionRequest & request, const std::string & path) override;
  std::string availability_diagnostic() const;
  bool submit(const arm_cell_interfaces::action::ExecuteTask::Goal & goal) override;
  bool submit(
    const arm_cell_interfaces::action::ExecuteTask::Goal & goal,
    const std::string & motion_goal_uuid) override;
  bool submit_normalized(const NormalizedMotionRequest & request) override;
  uint8_t plan_normalized(
    const NormalizedMotionRequest & request, NormalizedTrajectoryPlan & plan) override;
  bool execute_planned(const NormalizedTrajectoryPlan & plan) override;
  uint8_t execute_joint_target() override;
  MotionDisturbanceParameters disturbance_parameters() const override;
  void reset_pick_grasp_insertion_started() override;
  bool pick_grasp_insertion_started() const override;
  void record_task_accuracy(
    const NormalizedMotionRequest & request, uint8_t execution_result) override;
  std::string last_pick_candidate_evaluation() const override;
  std::string last_place_failure_stage() const override {return last_place_failure_stage_;}
  bool advance_to_next_planning_candidate() override;
  bool retry_place_retreat() override;
  void finalize_motion_planning_diagnostic(uint8_t result_code) override;
  void finalize_motion_planning_diagnostic(uint8_t result_code, double elapsed_s) override;
  void finalize_motion_planning_diagnostic(
    uint8_t result_code, double elapsed_s, const std::string & failure_stage,
    const std::string & failure_detail) override;
  uint8_t normalized_failure_result() const override;
  void begin_pick_grasp_relation() override;
  void stage_selected_pick_grasp_relation(const geometry_msgs::msg::Transform & relation) override;
  bool stage_pick_capture_reference(
    const geometry_msgs::msg::PoseStamped & pose, double width_mm, uint8_t symmetry_order,
    const std::string & execution_id) override;
  void discard_pending_grasp_relation() override;
  bool commit_pending_grasp_relation() override;
  void clear_grasp_relation_after_release() override;
  MotionCompletionOutcome wait_for_motion_completion() override;
  void cancel() override;
  void safety_preempt(uint8_t stop_mode) override;
  void hold() override;
  void stop() override;
  bool execution_active() const override;
  bool inactivity_confirmed() const override;
  std::string inactivity_diagnostic() const;
  bool measured_joint_state(sensor_msgs::msg::JointState & state) const override;
  bool command_normalized_gripper(double opening) override;
  bool command_gripper_close() override;
  HoldingObservation holding_observation() const override;
  bool gripper_active() const override;
  void stop_gripper() override;

private:
  static constexpr auto kStateFreshness = std::chrono::milliseconds(500);
  MotionCompletionOutcome wait_for_motion_completion_until(
    std::chrono::steady_clock::time_point deadline);
  // Serialized by the owning Action, never inferred from result-code strings.
  bool submit_normalized_once(
    const NormalizedMotionRequest & request,
    const std::vector<double> * waypoint = nullptr, double budget_s = 0.0,
    bool qualify_place_target = false,
    NormalizedTrajectoryPlan * plan_output = nullptr);
  bool execute_planned_trajectory(
    const NormalizedMotionRequest & request,
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    bool cartesian_primitive, bool place_recovery_waypoint,
    const MotionPlanningScales & planning_scales,
    const std::function<bool()> & discard_if_interrupted);
  bool place_recovery_state_valid(const NormalizedMotionRequest & request, std::string & reason);
  void record_place_recovery(
    const std::string & event, const std::string & reason,
    std::size_t candidate = 0U, bool include_planning = false,
    std::optional<uint8_t> execution_result = std::nullopt);
  std::vector<double> place_recovery_joint_candidates_;
  std::size_t place_recovery_max_moves_{1U};
  double place_recovery_budget_s_{30.0};
  double place_recovery_planning_time_s_{3.0};
  bool place_recovery_eligible_{false};
  bool place_recovery_candidate_fatal_{false};
  int place_recovery_last_plan_code_{0};
  std::string place_recovery_waypoint_reason_;
  moveit_msgs::msg::PlanningScene place_recovery_scene_;
  std::string place_recovery_goal_uuid_;
  std::size_t place_recovery_moves_{0U};
  std::size_t place_recovery_next_candidate_{0U};
  std::chrono::steady_clock::time_point place_recovery_started_{};
  std::vector<std::string> place_recovery_history_;
  std::string last_place_failure_stage_;
  std::string place_retreat_goal_uuid_;
  std::size_t place_retreat_planning_requests_{0U};

  struct ExecutedPathSelection
  {
    std::string motion_goal_uuid;
    uint8_t phase{arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_UNKNOWN};
    std::string strategy;
  };

  static std::string diagnostic_path_strategy(
    const ExecutedPathSelection & selection, const NormalizedMotionRequest & request);
  void log_task_accuracy_summary(
    const NormalizedMotionRequest & request, uint8_t execution_result,
    double position_error_mm, double orientation_error_rad) const;

  bool publish_joint_target(const std::vector<double> & positions);
  bool publish_timed_joint_target(const TimedJointSample & sample);
  bool publish_joint_target_locked(
    const std::vector<double> & positions,
    const std::vector<double> * velocities = nullptr,
    std::string * diagnostic_rejection_reason = nullptr);
  void republish_joint_target_locked();
  bool publish_current_hold_target(MotionCompletionOutcome interruption);
  void reassert_hold_after_raced_motion_command();
  bool extract_arm_positions(
    const sensor_msgs::msg::JointState & message,
    std::vector<double> & positions) const;
  std::vector<double> extract_arm_velocities(
    const sensor_msgs::msg::JointState & message) const;
  void begin_profile_trace(
    const std::string & kind, const std::string & config,
    const std::string & motion_goal_uuid = {}, std::uint64_t execution_id = 0);
  void finish_profile_trace(const std::string & outcome);
  TrajectorySampleResult start_global_profiled_trajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory, std::uint64_t execution_id,
    double start_simulation_time, const std::string & config,
    const std::string & motion_goal_uuid = {});
  void finish_global_profile_capture(const std::string & outcome, bool completed);
  void on_joint_state(const sensor_msgs::msg::JointState::ConstSharedPtr message);
  void on_simulation_clock(const rosgraph_msgs::msg::Clock::ConstSharedPtr message);
  void load_place_waypoint_trajectory();
  void on_gripper_status(const std_msgs::msg::String::ConstSharedPtr message);
  bool current_state_fresh() const;
  bool measured_target_reached_locked() const;
  void log_motion_state_locked(const std::string & event) const;
  bool apply_validation_grasp_contact_policy(
    moveit_msgs::msg::AllowedCollisionMatrix & original_acm);
  bool restore_planning_scene_acm(
    const moveit_msgs::msg::AllowedCollisionMatrix & original_acm);
  void set_pending_motion_planning_evidence(
    const std::string & goal_uuid, uint8_t task_type, std::string evidence);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MoveGroupInitializationState> move_group_state_{
    std::make_shared<MoveGroupInitializationState>()};
  std::thread move_group_worker_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_close_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr capture_reference_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_fixture_object_id_subscription_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr simulation_clock_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_status_subscription_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr planning_scene_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;
  // True only after a PICK trajectory has actually started execution during
  // the current Motion Action. Planning and trajectory generation do not set it.
  bool pick_action_trajectory_execution_started_{false};
  bool pick_grasp_insertion_started_{false};
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_client_;
  std::vector<std::string> arm_joint_names_;
  std::vector<double> home_joint_positions_;
  std::vector<double> retract_joint_positions_;
  std::vector<double> place_tray_return_joint_positions_;
  MotionProgressWatchdog motion_progress_watchdog_{60.0};
  bool motion_progress_watchdog_active_{false};
  mutable std::mutex mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::chrono::steady_clock::time_point latest_state_receipt_;
  std::vector<double> target_positions_;
  bool execution_active_{false};
  bool stop_requested_{false};
  MotionCompletionOutcome completion_outcome_{MotionCompletionOutcome::COMPLETED};
  bool inactivity_confirmed_{true};
  uint8_t active_motion_task_type_{
    arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_UNSPECIFIED};
  std::uint64_t joint_state_generation_{0};
  std::uint64_t motion_command_generation_{0};
  std::uint64_t stop_command_generation_{0};
  std::size_t settled_feedback_count_{0};
  std::string stop_observation_diagnostic_;
  bool stop_command_published_{false};
  bool gripper_active_{false};
  bool gripper_runtime_ready_{false};
  bool grasp_confirmed_{false};
  bool object_attached_{false};
  bool release_confirmed_{false};
  double gripper_width_mm_{0.0};
  std::uint64_t gripper_status_sequence_{0};
  std::string capture_reference_id_;
  std::string applied_capture_reference_id_;
  std::condition_variable gripper_status_condition_;
  std::string last_gripper_status_rejection_reason_;
  std::chrono::steady_clock::time_point latest_gripper_status_receipt_;
  std::condition_variable joint_state_condition_;
  bool validation_grasp_contact_policy_enabled_{false};
  std::string validation_grasp_contact_object_id_prefix_;
  mutable std::mutex current_fixture_object_id_mutex_;
  std::string current_fixture_object_id_;
  std::vector<std::string> validation_grasp_contact_links_;
  double planning_max_acceleration_rad_s2_{0.0};
  double planning_velocity_scaling_factor_{0.1};
  double planning_acceleration_scaling_factor_{0.1};
  double disturbance_push_distance_m_{0.02};
  double disturbance_approach_clearance_m_{0.01};
  double disturbance_speed_fraction_{0.2};
  double planning_time_s_{5.0};
  unsigned int planning_num_attempts_{3U};
  std::string planning_planner_id_{"RRTConnect"};
  std::string pick_ik_solver_name_;
  std::string pick_ik_solve_type_;
  double pick_ik_timeout_s_{1.0};
  GraspOccupancyMode grasp_occupancy_mode_{GraspOccupancyMode::FULL_COLLISION};
  FingerClearanceParameters finger_clearance_;
  double pick_grasp_ik_timeout_s_{1.0};
  double pick_approach_ik_timeout_s_{1.0};
  double pick_search_budget_s_{100.0};
  double pick_candidate_search_budget_s_{15.0};
  std::chrono::steady_clock::time_point pick_candidate_search_started_at_{};
  std::mutex pick_search_budget_mutex_;
  std::chrono::steady_clock::time_point pick_search_started_at_{};
  std::string pick_search_goal_uuid_;
  std::size_t pick_candidate_grasp_pose_candidates_remaining_{0U};
  trajectory_msgs::msg::JointTrajectory pending_pick_descent_trajectory_;
  trajectory_msgs::msg::JointTrajectory reusable_pick_descent_trajectory_;
  bool pending_pick_descent_for_current_execution_{false};
  double pick_max_withdrawal_tilt_deg_{90.0};
  std::string pick_grasp_pose_order_policy_{"distance_then_tilt"};
  double pick_position_range_half_length_fraction_{0.5};
  double pick_max_downward_position_offset_m_{0.005};
  std::size_t pick_position_offset_samples_per_tangent_axis_{3U};
  std::size_t pick_orientation_samples_per_closing_axis_{8U};
  std::size_t pick_grasp_pose_candidate_limit_{48U};
  std::size_t pick_ik_seed_attempts_per_grasp_pose_{2U};
  std::size_t place_pose_candidate_limit_{13U};
  std::size_t pick_approach_global_max_requests_{3U};
  std::size_t pick_final_cartesian_max_requests_{1U};
  std::size_t pick_final_global_fallback_max_requests_{3U};
  std::size_t place_approach_global_max_requests_{3U};
  std::size_t place_final_cartesian_max_requests_{1U};
  std::size_t place_final_global_fallback_max_requests_{3U};
  std::string planning_candidate_goal_uuid_;
  std::size_t pick_grasp_pose_candidate_floor_{0U};
  std::size_t place_candidate_pose_floor_{0U};
  std::size_t selected_planning_candidate_index_{0U};
  uint8_t selected_planning_candidate_task_{0U};
  bool next_candidate_available_{false};
  std::vector<PickCollisionContact> pick_collision_feedback_;
  std::string pick_collision_feedback_goal_uuid_;
  std::size_t pick_active_grasp_pose_candidate_count_{0U};
  std::size_t selected_pick_ik_seed_attempt_{0U};
  std::size_t place_pose_candidate_count_{0U};
  std::vector<std::string> planning_candidate_history_;
  double place_candidate_planning_time_s_{1.0};
  double place_total_planning_time_s_{5.0};
  mutable std::mutex motion_envelope_mutex_;
  MotionEnvelope safety_motion_envelope_{1.0F, 1.0F, true};
  std::string trajectory_sampler_name_;
  ExecutedPathSelection last_executed_path_selection_;
  double cartesian_tcp_orientation_tolerance_rad_{0.08726646259971647};
  double trajectory_goal_tolerance_rad_{0.03};
  geometry_msgs::msg::Quaternion selected_pick_target_orientation_{};
  bool selected_pick_target_orientation_valid_{false};
  geometry_msgs::msg::PoseStamped selected_place_object_pose_{};
  geometry_msgs::msg::PoseStamped selected_place_target_pose_{};
  geometry_msgs::msg::PoseStamped selected_place_approach_pose_{};
  geometry_msgs::msg::PoseStamped selected_place_retract_pose_{};
  bool selected_place_pose_valid_{false};
  geometry_msgs::msg::Transform accepted_object_to_tcp_{};
  bool accepted_object_to_tcp_valid_{false};
  geometry_msgs::msg::Transform pending_object_to_tcp_{};
  bool pending_object_to_tcp_valid_{false};
  std::shared_ptr<TrajectoryExecutionWorker> trajectory_worker_;
  std::shared_ptr<MotionProfileTrace> profile_trace_;
  bool diagnostics_capture_enabled_{false};
  std::string diagnostics_output_directory_;
  std::string snapshot_run_directory_;
  std::shared_ptr<MotionPlanningDiagnosticCapture> planning_diagnostic_capture_;
  std::string pending_motion_planning_evidence_;
  std::string pending_motion_planning_goal_uuid_;
  std::string pending_motion_planning_vision_acquisition_id_;
  uint8_t pending_motion_planning_task_type_{0};
  std::string task_accuracy_goal_uuid_;
  mutable std::mutex place_waypoint_cache_mutex_;
  std::optional<NormalizedTrajectoryPlan> cached_place_waypoint_approach_;
  std::string place_waypoint_trajectory_file_;
  bool place_waypoint_trajectory_load_attempted_{false};
  std::vector<double> place_waypoint_object_target_signature_;
  std::vector<double> place_waypoint_tcp_target_signature_;
  std::vector<std::string> task_accuracy_history_;
  uint8_t task_accuracy_task_type_{0};
  mutable std::mutex planning_diagnostic_mutex_;
  bool go_home_profile_trace_active_{false};
  mutable std::mutex profile_capture_mutex_;
  bool global_profile_capture_complete_{false};
  bool global_profile_trace_active_{false};
  std::uint64_t next_profile_trace_id_{0};
  std::string active_profile_configuration_;
  std::uint64_t next_trajectory_execution_id_{0};
  double latest_simulation_time_{0.0};
  bool has_simulation_time_{false};
  bool timed_trajectory_active_{false};
  std::uint64_t timed_clock_callback_count_{0};
  std::uint64_t timed_command_publish_attempt_count_{0};
  std::uint64_t timed_command_publish_success_count_{0};
  TrajectoryWatchdog trajectory_watchdog_;
  std::unique_ptr<ClockDiagnosticCapture> clock_diagnostic_capture_;
  std::chrono::steady_clock::time_point latest_simulation_clock_receipt_{};
  std::string trajectory_sampler_configuration_error_;
  std::string pick_candidate_evaluation_;
  geometry_msgs::msg::PoseStamped selected_pick_object_pose_;
  geometry_msgs::msg::PoseStamped selected_pick_target_pose_;
  geometry_msgs::msg::PoseStamped selected_pick_approach_pose_;
  geometry_msgs::msg::PoseStamped selected_pick_retract_pose_;
  bool selected_pick_pose_valid_{false};
  uint8_t normalized_failure_result_{
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR};
};

}  // namespace arm_cell_motion
