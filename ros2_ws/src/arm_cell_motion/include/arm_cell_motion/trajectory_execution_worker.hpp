#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arm_cell_motion/trajectory_sampler.hpp"
#include "arm_cell_motion/motion_profile_trace.hpp"

namespace arm_cell_motion
{

struct TrajectoryExecutionDiagnostics
{
  std::uint64_t execution_id{0};
  std::size_t point_count{0};
  double start_simulation_time{0.0};
  std::size_t current_segment{0};
  double trajectory_time{0.0};
  std::uint64_t simulation_callback_count{0};
  std::uint64_t sample_count{0};
  std::uint64_t command_publish_attempt_count{0};
  std::uint64_t command_publish_success_count{0};
  std::uint64_t final_command_publish_success_count{0};
  double command_simulation_time{0.0};
  double command_simulation_dt{0.0};
  std::vector<double> commanded_positions;
  std::vector<double> commanded_delta;
  std::vector<double> commanded_velocity;
  double actual_simulation_time{0.0};
  double actual_simulation_dt{0.0};
  std::vector<double> actual_positions;
  std::vector<double> actual_delta;
  std::vector<double> actual_velocity;
  std::vector<double> tracking_error;
  std::optional<double> max_tracking_error;
  std::optional<double> final_joint_target_error_max_rad;
  std::vector<double> final_target;
  bool final_command_accepted{false};
  std::optional<double> final_command_attempt_simulation_time;
  std::optional<double> final_command_attempt_steady_time_s;
  std::optional<double> final_command_accepted_steady_time_s;
  double planned_duration_s{0.0};
  bool final_joint_target_within_tolerance{false};
  std::string terminal_reason;
  bool trajectory_execution_complete{false};
};

class TrajectoryExecutionWorker final
{
public:
  using TimedCommandSink = std::function<bool (const TimedJointSample &)>;
  using PositionCommandSink = std::function<bool (const std::vector<double> &)>;

  TrajectoryExecutionWorker(
    std::shared_ptr<TrajectorySampler> sampler,
    TimedCommandSink command_sink,
    double joint_target_accuracy_tolerance_rad = 0.01);

  TrajectoryExecutionWorker(
    std::shared_ptr<TrajectorySampler> sampler,
    PositionCommandSink command_sink,
    double joint_target_accuracy_tolerance_rad = 0.01);

  void set_velocity_limits(const std::vector<double> & velocity_limits);
  void set_joint_target_accuracy_tolerance(double tolerance);
  void set_profile_trace(std::shared_ptr<MotionProfileTrace> trace);

  TrajectorySampleResult start(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    std::uint64_t execution_id,
    double start_simulation_time);
  TrajectorySampleResult on_simulation_time(double simulation_time);
  void update_actual_positions(
    const std::vector<double> & positions, double simulation_time = -1.0);
  void cancel(const std::string & reason = "cancelled");

  bool active() const;
  bool failed() const;
  bool finished() const;
  bool completion_eligible() const;
  std::string failure_reason() const;
  TrajectoryExecutionDiagnostics diagnostics() const;

private:
  TrajectorySampleResult validate_sample_velocity(const TimedJointSample & sample) const;

  std::shared_ptr<TrajectorySampler> sampler_;
  TimedCommandSink command_sink_;
  std::vector<double> velocity_limits_;
  double joint_target_accuracy_tolerance_rad_{0.01};
  trajectory_msgs::msg::JointTrajectory trajectory_;
  std::shared_ptr<MotionProfileTrace> profile_trace_;
  TrajectoryExecutionDiagnostics diagnostics_;
  double last_simulation_time_{0.0};
  bool active_{false};
  bool failed_{false};
  bool finished_{false};
  bool have_previous_command_{false};
  bool have_previous_actual_{false};
  double previous_command_simulation_time_{0.0};
  double previous_actual_simulation_time_{0.0};
  std::string failure_reason_;
  std::uint64_t execution_generation_{0};
  // Serialize command delivery without holding the state lock across the sink.
  std::mutex command_mutex_;
  mutable std::mutex mutex_;
};

}  // namespace arm_cell_motion
