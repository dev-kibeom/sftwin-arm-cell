#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <memory>

#include "arm_cell_motion_moveit2/motion_backend.hpp"

namespace arm_cell_motion_moveit2
{

class FakeMotionBackendGripperPort final : public GripperPort
{
public:
  uint8_t close(float) override
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
    if (task_result_ != arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      return task_result_;
    }
    return submit(goal) ?
           arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS :
           arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_BACKEND_ERROR;
  }

  uint8_t execute_normalized(const NormalizedMotionRequest & request) override
  {
    ++execute_task_calls_;
    submitted_task_type_ = request.task_type;
    if (task_result_ != arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS) {
      return task_result_;
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
    return arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS;
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
  }
  void hold() override
  {
    ++hold_calls_;
    interruption_result_ =
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
    stop_requested_ = true;
    release_execution();
  }
  void stop() override
  {
    ++stop_calls_;
    interruption_result_ =
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SAFETY_PREEMPTED;
    stop_requested_ = true;
    release_execution();
  }

  bool execution_active() const override {return execution_active_;}
  bool backend_inactivity_confirmed() const override {return inactivity_confirmed_;}

  void set_gripper_port(std::shared_ptr<GripperPort> port) {gripper_port_ = std::move(port);}
  std::shared_ptr<GripperPort> gripper_port() const override {return gripper_port_;}

  void set_available(bool value) {available_ = value;}
  void set_execution_active(bool value) {execution_active_ = value;}
  void set_inactivity_confirmed(bool value) {inactivity_confirmed_ = value;}
  bool stop_requested() const {return stop_requested_;}
  int cancel_calls() const {return cancel_calls_;}
  int stop_calls() const {return stop_calls_;}
  int hold_calls() const {return hold_calls_;}
  const MotionTuning & applied_tuning() const {return applied_tuning_;}
  int submit_calls() const {return submit_calls_;}
  int execute_task_calls() const {return execute_task_calls_;}
  void set_task_result(uint8_t result) {task_result_ = result;}
  void set_block_execution(bool value) {block_execution_ = value;}
  void set_defer_terminal_return(bool value) {defer_terminal_return_ = value;}
  bool wait_until_execution_started()
  {
    std::unique_lock<std::mutex> lock(execution_mutex_);
    return execution_condition_.wait_for(
      lock, std::chrono::seconds(1), [this]() {return execution_started_;});
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
  MotionEnvelope applied_envelope_{1.0F, 1.0F, true};
  bool available_{true};
  bool execution_active_{false};
  bool inactivity_confirmed_{true};
  bool stop_requested_{false};
  uint8_t interruption_result_{
    arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_CANCELED};
  int cancel_calls_{0};
  int stop_calls_{0};
  int hold_calls_{0};
  MotionTuning applied_tuning_{};
  int submit_calls_{0};
  int execute_task_calls_{0};
  uint8_t task_result_{arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_SUCCESS};
  arm_cell_interfaces::msg::MotionTaskType submitted_task_type_{};
  bool block_execution_{false};
  bool defer_terminal_return_{false};
  bool execution_started_{false};
  bool release_execution_{false};
  bool interruption_ready_{false};
  bool terminal_return_released_{false};
  mutable std::mutex execution_mutex_;
  std::condition_variable execution_condition_;
  std::condition_variable interruption_condition_;
  std::condition_variable terminal_return_condition_;
};

}  // namespace arm_cell_motion_moveit2
