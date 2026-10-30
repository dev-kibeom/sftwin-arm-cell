#pragma once

#include <chrono>
#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include "arm_cell_motion_moveit2/isaac_motion_backend.hpp"
#include "arm_cell_motion_moveit2/approach_entry_acceptance.hpp"
#include "arm_cell_motion_moveit2/final_target_hold.hpp"
#include "arm_cell_motion_moveit2/trajectory_execution_worker.hpp"
#include "arm_cell_motion_moveit2/trajectory_watchdog.hpp"
#include "arm_cell_motion_moveit2/motion_progress_watchdog.hpp"
#include "arm_cell_motion_moveit2/motion_tuning.hpp"

namespace arm_cell_motion_moveit2
{

struct MoveGroupInitializationState
{
  mutable std::mutex mutex;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
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
    double approach_entry_position_tolerance_m = 0.005,
    double approach_entry_orientation_tolerance_rad = 0.08726646259971647,
    bool approach_entry_require_settled = true,
    double approach_entry_settled_joint_delta_rad = 0.001,
    std::size_t approach_entry_settled_samples = 3,
    double motion_progress_stall_timeout_s = 60.0,
    double place_tracking_tolerance_rad = 0.01,
    double place_approach_entry_position_tolerance_m = -1.0,
    double planning_velocity_scaling_factor = 0.1,
    double planning_acceleration_scaling_factor = 0.1,
    double planning_time_s = 5.0,
    double place_candidate_planning_time_s = 1.0,
    double place_total_planning_time_s = 5.0);

  ~IsaacRosMotionTransport() override;

  void initialize_move_group();

  bool available() const override;
  bool apply_runtime_tuning(const MotionTuning & tuning) override;
  bool set_motion_envelope(const MotionEnvelope & envelope) override;
  MotionEnvelope applied_motion_envelope() const override;
  std::string availability_diagnostic() const;
  bool submit(const arm_cell_interfaces::action::ExecuteTask::Goal & goal) override;
  bool submit_normalized(const NormalizedMotionRequest & request) override;
  void begin_pick_grasp_relation() override;
  void stage_selected_pick_grasp_relation(const geometry_msgs::msg::Transform & relation) override;
  void discard_pending_grasp_relation() override;
  bool confirm_fresh_held_grasp_relation() override;
  void confirm_fresh_released_grasp_relation() override;
  MotionCompletionOutcome wait_for_motion_completion() override;
  void cancel() override;
  void safety_preempt(uint8_t stop_mode) override;
  void hold() override;
  void stop() override;
  bool execution_active() const override;
  bool inactivity_confirmed() const override;
  bool command_normalized_gripper(double opening) override;
  HoldingObservation holding_observation() const override;
  bool gripper_active() const override;
  void stop_gripper() override;

private:
  bool publish_joint_target(const std::vector<double> & positions);
  bool publish_timed_joint_target(const TimedJointSample & sample);
  bool publish_final_target_hold_locked();
  bool publish_joint_target_locked(
    const std::vector<double> & positions,
    const std::vector<double> * velocities = nullptr);
  void republish_joint_target_locked();
  bool publish_current_hold_target(MotionCompletionOutcome interruption);
  bool extract_arm_positions(
    const sensor_msgs::msg::JointState & message,
    std::vector<double> & positions) const;
  std::vector<double> extract_arm_velocities(
    const sensor_msgs::msg::JointState & message) const;
  void begin_profile_trace(const std::string & kind, const std::string & config);
  void finish_profile_trace(const std::string & outcome);
  TrajectorySampleResult start_global_profiled_trajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory, std::uint64_t execution_id,
    double start_simulation_time, const std::string & config);
  void finish_global_profile_capture(const std::string & outcome, bool completed);
  void on_joint_state(const sensor_msgs::msg::JointState::ConstSharedPtr message);
  void on_simulation_clock(const rosgraph_msgs::msg::Clock::ConstSharedPtr message);
  void on_gripper_status(const std_msgs::msg::String::ConstSharedPtr message);
  bool current_state_fresh() const;
  bool actual_approach_settled_locked() const;
  bool validate_approach_entry();
  bool measured_target_reached_locked() const;
  void log_motion_state_locked(const std::string & event) const;
  bool apply_validation_grasp_contact_policy(
    moveit_msgs::msg::AllowedCollisionMatrix & original_acm);
  bool restore_planning_scene_acm(
    const moveit_msgs::msg::AllowedCollisionMatrix & original_acm);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MoveGroupInitializationState> move_group_state_{
    std::make_shared<MoveGroupInitializationState>()};
  std::thread move_group_worker_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_fixture_object_id_subscription_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr simulation_clock_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_status_subscription_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr planning_scene_client_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_client_;
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_client_;
  std::vector<std::string> arm_joint_names_;
  std::vector<double> home_joint_positions_;
  std::vector<double> retract_joint_positions_;
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
  std::uint64_t joint_state_generation_{0};
  std::uint64_t motion_command_generation_{0};
  std::uint64_t stop_command_generation_{0};
  std::size_t settled_feedback_count_{0};
  bool gripper_active_{false};
  bool gripper_runtime_ready_{false};
  bool grasp_confirmed_{false};
  bool object_attached_{false};
  bool release_confirmed_{false};
  double gripper_width_mm_{0.0};
  std::uint64_t gripper_status_sequence_{0};
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
  double planning_time_s_{5.0};
  double place_candidate_planning_time_s_{1.0};
  double place_total_planning_time_s_{5.0};
  mutable std::mutex motion_envelope_mutex_;
  MotionEnvelope safety_motion_envelope_{1.0F, 1.0F, true};
  std::string trajectory_sampler_name_;
  double place_tracking_tolerance_rad_{0.01};
  double place_approach_entry_position_tolerance_m_{0.005};
  ApproachEntryPolicy approach_entry_policy_{};
  ActualFeedbackStabilityTracker approach_settling_tracker_{0.001, 3};
  bool approach_reference_completion_observed_{false};
  bool approach_acceptance_active_{false};
  bool approach_acceptance_log_initialized_{false};
  bool last_approach_acceptance_accepted_{false};
  bool last_approach_acceptance_settled_{false};
  bool last_approach_acceptance_fresh_feedback_{false};
  bool last_approach_acceptance_joint_start_sync_{false};
  bool last_approach_acceptance_state_valid_{false};
  std::string last_approach_acceptance_reason_;
  std::string last_approach_rejection_reason_;
  double last_approach_acceptance_position_error_m_{0.0};
  double last_approach_acceptance_orientation_error_rad_{0.0};
  double last_approach_rejection_position_error_m_{0.0};
  double last_approach_rejection_orientation_error_rad_{0.0};
  std::vector<double> approach_expected_joint_positions_;
  std::vector<double> validated_approach_start_joint_positions_;
  bool approach_start_state_validated_{false};
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
  uint8_t active_phase_{0};
  bool active_trajectory_is_place_{false};
  geometry_msgs::msg::PoseStamped approach_entry_target_{};
  std::size_t approach_acceptance_candidate_index_{0};
  bool approach_acceptance_candidate_valid_{false};
  std::shared_ptr<TrajectoryExecutionWorker> trajectory_worker_;
  std::shared_ptr<MotionProfileTrace> profile_trace_;
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
  FinalTargetHold final_target_hold_;
  bool final_target_hold_diagnostic_started_{false};
  std::chrono::steady_clock::time_point final_target_hold_diagnostic_start_;
  std::uint64_t final_target_hold_publish_count_{0};
  bool settling_diagnostics_enabled_{false};
  bool settling_diagnostic_logged_{false};
  double last_settling_diagnostic_simulation_time_{0.0};
  std::uint64_t timed_clock_callback_count_{0};
  std::uint64_t timed_command_publish_attempt_count_{0};
  std::uint64_t timed_command_publish_success_count_{0};
  double last_timed_telemetry_simulation_time_{0.0};
  std::uint64_t final_lifecycle_logged_execution_id_{0};
  TrajectoryWatchdog trajectory_watchdog_;
  std::string trajectory_sampler_configuration_error_;
};

}  // namespace arm_cell_motion_moveit2
