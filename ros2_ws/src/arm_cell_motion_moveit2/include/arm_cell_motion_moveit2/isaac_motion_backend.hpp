#pragma once

#include <cstdint>
#include <memory>

#include <arm_cell_interfaces/msg/stop_mode.hpp>

#include "arm_cell_motion_moveit2/motion_backend.hpp"

namespace arm_cell_motion_moveit2
{

enum class MotionCompletionOutcome : uint8_t
{
  COMPLETED,
  CANCELED,
  SAFETY_PREEMPTED,
  TIMEOUT,
  STALE_FEEDBACK,
  TRANSPORT_UNAVAILABLE
};

class IsaacMotionTransport
{
public:
  virtual ~IsaacMotionTransport() = default;

  virtual bool available() const = 0;
  virtual bool apply_runtime_tuning(const MotionTuning &) {return true;}
  virtual bool set_motion_envelope(const MotionEnvelope & envelope)
  {
    return envelope.valid && envelope.max_velocity_scale == 1.0F &&
           envelope.max_acceleration_scale == 1.0F;
  }
  virtual MotionEnvelope applied_motion_envelope() const
  {
    return MotionEnvelope{1.0F, 1.0F, true};
  }
  virtual std::string availability_diagnostic() const {return "transport_readiness=unknown";}
  virtual bool submit(
    const arm_cell_interfaces::action::ExecuteTask::Goal & goal) = 0;
  virtual bool submit_normalized(const NormalizedMotionRequest & request) = 0;
  virtual MotionCompletionOutcome wait_for_motion_completion() = 0;
  virtual void cancel() = 0;
  virtual void safety_preempt(uint8_t stop_mode) = 0;
  virtual void hold() = 0;
  virtual void stop() = 0;
  virtual bool execution_active() const = 0;
  virtual bool inactivity_confirmed() const = 0;
  virtual bool command_normalized_gripper(double opening) = 0;
  virtual void begin_pick_grasp_relation() {}
  virtual void stage_selected_pick_grasp_relation(const geometry_msgs::msg::Transform &) {}
  virtual void discard_pending_grasp_relation() {}
  virtual bool confirm_fresh_held_grasp_relation() {return true;}
  virtual void confirm_fresh_released_grasp_relation() {}
  virtual bool gripper_active() const = 0;
  virtual HoldingObservation holding_observation() const {return {};}
  virtual void stop_gripper() = 0;
};

class IsaacMotionBackend final : public MotionBackend
{
public:
  explicit IsaacMotionBackend(std::shared_ptr<IsaacMotionTransport> transport);

  bool available() const override;
  bool apply_runtime_tuning(const MotionTuning & tuning) override;
  bool set_motion_envelope(const MotionEnvelope & envelope) override;
  MotionEnvelope applied_motion_envelope() const override;
  std::shared_ptr<GripperPort> gripper_port() const override;
  std::string availability_diagnostic() const override;
  bool submit(const arm_cell_interfaces::action::ExecuteTask::Goal & goal) override;
  uint8_t execute_task(const arm_cell_interfaces::action::ExecuteTask::Goal & goal) override;
  uint8_t execute_normalized(const NormalizedMotionRequest & request) override;
  void cancel() override;
  void safety_preempt(uint8_t stop_mode) override;
  void hold() override;
  void stop() override;
  bool execution_active() const override;
  bool backend_inactivity_confirmed() const override;
  uint8_t command_gripper(float width_mm) override;
  void begin_pick_grasp_relation() override;
  void stage_selected_pick_grasp_relation(const geometry_msgs::msg::Transform & relation) override;
  void discard_pending_grasp_relation() override;
  bool confirm_fresh_held_grasp_relation() override;
  void confirm_fresh_released_grasp_relation() override;
  bool gripper_active() const override;
  void stop_gripper() override;

private:
  std::shared_ptr<IsaacMotionTransport> transport_;
  std::shared_ptr<GripperPort> gripper_port_;
};

}  // namespace arm_cell_motion_moveit2
