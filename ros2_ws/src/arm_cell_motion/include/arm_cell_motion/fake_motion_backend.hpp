#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <thread>

#include "arm_cell_motion/motion_backend.hpp"

namespace arm_cell_motion
{

class FakeMotionBackendGripperPort final : public GripperPort
{
public:
  uint8_t close() override
  {
    observation_.state = HoldingState::HELD;
    observation_.observed_at = HoldingObservation::Clock::now();
    ++observation_.sequence;
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  uint8_t open() override
  {
    observation_.state = HoldingState::RELEASED;
    observation_.observed_at = HoldingObservation::Clock::now();
    ++observation_.sequence;
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  HoldingObservation holding() const override {return observation_;}
  bool active() const override {return active_;}
  void stop() override {active_ = false;}

private:
  HoldingObservation observation_;
  bool active_{false};
};

class FakeMotionBackend final : public MotionBackend
{
public:
  FakeMotionBackend()
  : gripper_port_(std::make_shared<FakeMotionBackendGripperPort>())
  {
  }

  bool available() const override {return available_;}

  bool apply_runtime_tuning(const MotionTuning & tuning) override
  {
    applied_tuning_ = tuning;
    return true;
  }

  bool set_motion_envelope(const MotionEnvelope & envelope) override
  {
    if (!envelope.valid || envelope.max_velocity_scale <= 0.0F ||
      envelope.max_velocity_scale > 1.0F || envelope.max_acceleration_scale <= 0.0F ||
      envelope.max_acceleration_scale > 1.0F)
    {
      return false;
    }
    applied_envelope_ = envelope;
    return true;
  }

  MotionEnvelope applied_motion_envelope() const override {return applied_envelope_;}

  bool submit(const arm_cell_interfaces::action::ExecuteTask::Goal &) override
  {
    ++submit_calls_;
    if (!available_) {
      return false;
    }
    execution_active_ = true;
    inactivity_confirmed_ = false;
    return true;
  }

  uint8_t execute_task(const arm_cell_interfaces::action::ExecuteTask::Goal & goal) override
  {
    ++execute_task_calls_;
    submitted_task_type_ = goal.task_type;
    if (throw_on_task_execution_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      throw std::runtime_error("fake backend task execution exception");
    }
    if (task_result_ != arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      return task_result_;
    }
    if (!submit(goal)) {
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    }
    if (block_task_execution_) {
      std::unique_lock<std::mutex> lock(execution_mutex_);
      execution_started_ = true;
      execution_condition_.notify_all();
      execution_condition_.wait(lock, [this]() {return release_execution_;});
      release_execution_ = false;
      if (stop_requested_) {
        const auto result = interruption_result_;
        if (defer_terminal_return_) {
          interruption_ready_ = true;
          interruption_condition_.notify_all();
          terminal_return_condition_.wait(lock, [this]() {return terminal_return_released_;});
          terminal_return_released_ = false;
        }
        return result;
      }
    }
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  uint8_t execute_normalized(const NormalizedMotionRequest & request) override
  {
    ++execute_task_calls_;
    submitted_task_type_ = request.task_type;
    if (task_result_ != arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      if (task_result_ ==
        arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_JOINT_PROGRESS_STALLED)
      {
        execution_active_ = false;
        inactivity_confirmed_ = false;
      }
      return task_result_;
    }
    if (request.task_type.value ==
      arm_cell_interfaces::msg::MotionTaskType::TASK_TYPE_PICK &&
      request.phase == arm_cell_interfaces::msg::MotionTaskPhase::TASK_PHASE_EXECUTING)
    {
      pick_grasp_insertion_started_ = true;
    }
    ++submit_calls_;
    if (!available_) {
      return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
    }
    execution_active_ = true;
    inactivity_confirmed_ = false;
    if (block_execution_) {
      std::unique_lock<std::mutex> lock(execution_mutex_);
      execution_started_ = true;
      execution_condition_.notify_all();
      execution_condition_.wait(lock, [this]() {return release_execution_;});
      release_execution_ = false;
      if (stop_requested_) {
        const auto result = interruption_result_;
        if (defer_terminal_return_) {
          interruption_ready_ = true;
          interruption_condition_.notify_all();
          terminal_return_condition_.wait(lock, [this]() {return terminal_return_released_;});
          terminal_return_released_ = false;
        }
        return result;
      }
    }
    if (complete_execution_on_return_) {
      execution_active_ = false;
      inactivity_confirmed_ = !leave_inactivity_unconfirmed_after_completion_;
    }
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  bool supports_split_planning() const override {return true;}


  void finalize_motion_planning_diagnostic(uint8_t) override
  {++diagnostic_finalize_calls_;}

  uint8_t generate_place_waypoint_approach(
    const NormalizedMotionRequest & request, const std::string & path) override
  {
    ++place_waypoint_generation_calls_;
    generated_request_ = request;
    generated_path_ = path;
    return place_waypoint_generation_result_;
  }

  uint8_t plan_normalized(
    const NormalizedMotionRequest & request, NormalizedTrajectoryPlan & plan) override
  {
    ++planning_calls_;
    submitted_task_type_ = request.task_type;
    if (block_planning_) {
      std::unique_lock<std::mutex> lock(execution_mutex_);
      planning_started_ = true;
      planning_condition_.notify_all();
      planning_condition_.wait(lock, [this]() {return release_planning_;});
      release_planning_ = false;
      if (stop_requested_) {return interruption_result_;}
    }
    if (task_result_ != arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      return task_result_;
    }
    plan.request = request;
    plan.trajectory.joint_names = {"joint_1"};
    if (request.has_start_joint_state) {
      plan.start_state = request.start_joint_state;
    } else {
      plan.start_state.name = {"joint_1"};
      plan.start_state.position = {0.0};
    }
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {plan.start_state.position.front()};
    point.time_from_start.sec = 1;
    plan.trajectory.points.push_back(point);
    plan.end_state = plan.start_state;
    plan.end_state.position.front() = point.positions.front();
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  uint8_t execute_planned_normalized(const NormalizedTrajectoryPlan & plan) override
  {
    ++planned_execution_calls_;
    return execute_normalized(plan.request);
  }

  uint8_t execute_joint_target() override
  {
    ++joint_target_calls_;
    return joint_target_result_;
  }

  uint8_t command_gripper(float) override
  {
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
  }

  void cancel() override
  {
    ++cancel_calls_;
    interruption_result_ = arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED;
    stop_requested_ = true;
    release_execution();
    release_planning();
  }
  void hold() override
  {
    ++hold_calls_;
    interruption_result_ =
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
    stop_requested_ = true;
    release_execution();
    release_planning();
  }
  void stop() override
  {
    ++stop_calls_;
    interruption_result_ =
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
    stop_requested_ = true;
    release_execution();
    release_planning();
  }

  bool execution_active() const override {return execution_active_;}
  bool backend_inactivity_confirmed() const override {return inactivity_confirmed_;}
  std::string inactivity_diagnostic() const override {return inactivity_diagnostic_;}
  bool measured_joint_state(sensor_msgs::msg::JointState & state) const override
  {
    if (!has_measured_joint_state_) {return false;}
    state = measured_joint_state_;
    return true;
  }

  void set_gripper_port(std::shared_ptr<GripperPort> port) {gripper_port_ = std::move(port);}
  std::shared_ptr<GripperPort> gripper_port() const override {return gripper_port_;}

  void set_available(bool value) {available_ = value;}
  void set_execution_active(bool value) {execution_active_ = value;}
  void set_inactivity_confirmed(bool value) {inactivity_confirmed_ = value;}
  void set_inactivity_diagnostic(std::string value)
  {inactivity_diagnostic_ = std::move(value);}
  void set_measured_joint_state(const sensor_msgs::msg::JointState & state)
  {measured_joint_state_ = state; has_measured_joint_state_ = true;}
  bool stop_requested() const {return stop_requested_;}
  int cancel_calls() const {return cancel_calls_;}
  int stop_calls() const {return stop_calls_;}
  int hold_calls() const {return hold_calls_;}
  const MotionTuning & applied_tuning() const {return applied_tuning_;}
  int submit_calls() const {return submit_calls_;}
  int execute_task_calls() const {return execute_task_calls_;}
  int planning_calls() const {return planning_calls_;}
  int place_waypoint_generation_calls() const {return place_waypoint_generation_calls_;}
  const NormalizedMotionRequest & generated_request() const {return generated_request_;}
  const std::string & generated_path() const {return generated_path_;}
  void set_place_waypoint_generation_result(uint8_t result)
  {place_waypoint_generation_result_ = result;}
  int planned_execution_calls() const {return planned_execution_calls_;}
  std::size_t joint_target_calls() const {return joint_target_calls_;}
  int diagnostic_finalize_calls() const {return diagnostic_finalize_calls_.load();}
  void set_task_result(uint8_t result) {task_result_ = result;}
  void reset_pick_grasp_insertion_started() override
  {pick_grasp_insertion_started_ = false;}
  bool pick_grasp_insertion_started() const override
  {return pick_grasp_insertion_started_;}
  void set_throw_on_task_execution(bool value) {throw_on_task_execution_ = value;}
  void set_block_execution(bool value) {block_execution_ = value;}
  void set_block_planning(bool value) {block_planning_ = value;}
  void set_complete_execution_on_return(bool value) {complete_execution_on_return_ = value;}
  void set_leave_inactivity_unconfirmed_after_completion(bool value)
  {leave_inactivity_unconfirmed_after_completion_ = value;}
  void set_block_task_execution(bool value) {block_task_execution_ = value;}
  void set_defer_terminal_return(bool value) {defer_terminal_return_ = value;}
  bool wait_until_execution_started()
  {
    std::unique_lock<std::mutex> lock(execution_mutex_);
    return execution_condition_.wait_for(
      lock, std::chrono::seconds(1), [this]() {return execution_started_;});
  }
  bool wait_until_planning_started()
  {
    std::unique_lock<std::mutex> lock(execution_mutex_);
    return planning_condition_.wait_for(
      lock, std::chrono::seconds(1), [this]() {return planning_started_;});
  }
  void release_planning()
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    release_planning_ = true;
    planning_condition_.notify_all();
  }
  bool wait_until_interruption_ready()
  {
    std::unique_lock<std::mutex> lock(execution_mutex_);
    return interruption_condition_.wait_for(
      lock, std::chrono::seconds(1), [this]() {return interruption_ready_;});
  }
  void release_terminal_return()
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    terminal_return_released_ = true;
    terminal_return_condition_.notify_all();
  }
  void release_execution()
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);
    release_execution_ = true;
    execution_condition_.notify_all();
  }
  auto submitted_task_type() const {return submitted_task_type_;}

private:
  std::shared_ptr<GripperPort> gripper_port_;
  int place_waypoint_generation_calls_{0};
  uint8_t place_waypoint_generation_result_{
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS};
  NormalizedMotionRequest generated_request_;
  std::string generated_path_;
  MotionEnvelope applied_envelope_{1.0F, 1.0F, true};
  bool available_{true};
  bool execution_active_{false};
  bool inactivity_confirmed_{true};
  bool has_measured_joint_state_{false};
  sensor_msgs::msg::JointState measured_joint_state_;
  std::string inactivity_diagnostic_;
  bool stop_requested_{false};
  uint8_t interruption_result_{
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED};
  int cancel_calls_{0};
  int stop_calls_{0};
  int hold_calls_{0};
  MotionTuning applied_tuning_{};
  int submit_calls_{0};
  int execute_task_calls_{0};
  int planning_calls_{0};
  int planned_execution_calls_{0};
  std::atomic<int> diagnostic_finalize_calls_{0};
  uint8_t task_result_{arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS};
  uint8_t joint_target_result_{arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS};
  std::size_t joint_target_calls_{0U};
  bool pick_grasp_insertion_started_{false};
  arm_cell_interfaces::msg::MotionTaskType submitted_task_type_{};
  bool block_execution_{false};
  bool block_planning_{false};
  bool complete_execution_on_return_{false};
  bool leave_inactivity_unconfirmed_after_completion_{false};
  bool block_task_execution_{false};
  bool throw_on_task_execution_{false};
  bool defer_terminal_return_{false};
  bool execution_started_{false};
  bool planning_started_{false};
  bool release_planning_{false};
  bool release_execution_{false};
  bool interruption_ready_{false};
  bool terminal_return_released_{false};
  mutable std::mutex execution_mutex_;
  std::condition_variable execution_condition_;
  std::condition_variable planning_condition_;
  std::condition_variable interruption_condition_;
  std::condition_variable terminal_return_condition_;
};

}  // namespace arm_cell_motion
