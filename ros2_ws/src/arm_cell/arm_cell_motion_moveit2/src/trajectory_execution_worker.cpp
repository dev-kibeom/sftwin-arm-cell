#include "arm_cell_motion_moveit2/trajectory_execution_worker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arm_cell_motion_moveit2
{
namespace
{
double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) / 1e9;
}
}

TrajectoryExecutionWorker::TrajectoryExecutionWorker(
  std::shared_ptr<TrajectorySampler> sampler,
  TimedCommandSink command_sink,
  double convergence_tolerance)
: sampler_(std::move(sampler)),
  command_sink_(std::move(command_sink)),
  convergence_tolerance_(convergence_tolerance)
{
}

TrajectoryExecutionWorker::TrajectoryExecutionWorker(
  std::shared_ptr<TrajectorySampler> sampler,
  PositionCommandSink command_sink,
  double convergence_tolerance)
: TrajectoryExecutionWorker(
    std::move(sampler),
    [command_sink = std::move(command_sink)](const TimedJointSample & sample) {
      return command_sink(sample.positions);
    },
    convergence_tolerance)
{
}

void TrajectoryExecutionWorker::set_velocity_limits(
  const std::vector<double> & velocity_limits)
{
  std::lock_guard<std::mutex> lock(mutex_);
  velocity_limits_ = velocity_limits;
}

void TrajectoryExecutionWorker::set_convergence_tolerance(double tolerance)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (std::isfinite(tolerance) && tolerance > 0.0) {
    convergence_tolerance_ = tolerance;
  }
}

void TrajectoryExecutionWorker::set_profile_trace(std::shared_ptr<MotionProfileTrace> trace)
{
  std::lock_guard<std::mutex> lock(mutex_);
  profile_trace_ = std::move(trace);
}

TrajectorySampleResult TrajectoryExecutionWorker::validate_sample_velocity(
  const TimedJointSample & sample) const
{
  if (sample.velocities.size() != sample.positions.size() ||
    (!velocity_limits_.empty() && sample.velocities.size() != velocity_limits_.size()))
  {
    return {false, "derived trajectory velocity count is invalid"};
  }
  for (std::size_t index = 0; index < sample.velocities.size(); ++index) {
    if (!std::isfinite(sample.velocities[index]) ||
      (!velocity_limits_.empty() &&
      std::abs(sample.velocities[index]) > velocity_limits_[index] + 1e-9))
    {
      return {false, "derived trajectory velocity exceeds velocity limit"};
    }
  }
  return {true, ""};
}

TrajectorySampleResult TrajectoryExecutionWorker::start(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  std::uint64_t execution_id,
  double start_simulation_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!sampler_ || !command_sink_) {
    return {false, "trajectory worker is not configured"};
  }
  if (!std::isfinite(start_simulation_time)) {
    return {false, "trajectory start simulation time must be finite"};
  }
  TimedJointSample initial_sample;
  const auto validation = sampler_->sample_timed(trajectory, 0.0, initial_sample);
  if (!validation.valid) {
    return validation;
  }
  auto sample_velocity_validation = validate_sample_velocity(initial_sample);
  if (!sample_velocity_validation.valid) {
    return sample_velocity_validation;
  }
  for (std::size_t index = 0; index + 1 < trajectory.points.size(); ++index) {
    const auto start_time = duration_seconds(trajectory.points[index].time_from_start);
    const auto end_time = duration_seconds(trajectory.points[index + 1].time_from_start);
    TimedJointSample segment_sample;
    const auto segment_result = sampler_->sample_timed(
      trajectory, start_time + (end_time - start_time) * 0.5, segment_sample);
    if (!segment_result.valid) {
      return segment_result;
    }
    sample_velocity_validation = validate_sample_velocity(segment_sample);
    if (!sample_velocity_validation.valid) {
      return sample_velocity_validation;
    }
  }
  trajectory_ = trajectory;
  if (profile_trace_) {
    for (const auto & point : trajectory_.points) {
      const auto waypoint_time = start_simulation_time + duration_seconds(point.time_from_start);
      profile_trace_->add_row(
        {
          waypoint_time, "MOVEIT_WAYPOINT", point.positions, point.velocities,
          point.accelerations, {}, {}});
    }
  }
  diagnostics_ = {};
  diagnostics_.execution_id = execution_id;
  diagnostics_.point_count = trajectory.points.size();
  diagnostics_.start_simulation_time = start_simulation_time;
  diagnostics_.current_segment = initial_sample.segment_index;
  diagnostics_.commanded_positions = initial_sample.positions;
  diagnostics_.final_target = trajectory.points.back().positions;
  last_simulation_time_ = start_simulation_time;
  have_previous_command_ = false;
  have_previous_actual_ = false;
  final_sample_observed_ = false;
  active_ = true;
  failed_ = false;
  finished_ = false;
  diagnostics_.execution_complete = false;
  failure_reason_.clear();
  return {true, ""};
}

TrajectorySampleResult TrajectoryExecutionWorker::on_simulation_time(double simulation_time)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (failed_) {
    return {false, failure_reason_};
  }
  if (!active_) {
    return {true, ""};
  }
  if (!std::isfinite(simulation_time)) {
    failed_ = true;
    active_ = false;
    failure_reason_ = "simulation clock is non-finite";
    return {false, failure_reason_};
  }
  if (simulation_time < last_simulation_time_) {
    failed_ = true;
    active_ = false;
    failure_reason_ = "simulation clock moved backward";
    return {false, failure_reason_};
  }
  last_simulation_time_ = simulation_time;
  ++diagnostics_.simulation_callback_count;
  diagnostics_.trajectory_time =
    std::max(0.0, simulation_time - diagnostics_.start_simulation_time);

  TimedJointSample sample;
  const auto result = sampler_->sample_timed(
    trajectory_, diagnostics_.trajectory_time, sample);
  if (!result.valid) {
    failed_ = true;
    active_ = false;
    failure_reason_ = result.reason;
    return result;
  }
  const auto sample_velocity_result = validate_sample_velocity(sample);
  if (!sample_velocity_result.valid) {
    failed_ = true;
    active_ = false;
    failure_reason_ = sample_velocity_result.reason;
    return sample_velocity_result;
  }
  diagnostics_.current_segment = sample.segment_index;
  ++diagnostics_.sample_count;
  diagnostics_.command_simulation_time = simulation_time;
  diagnostics_.command_simulation_dt = have_previous_command_ ?
    simulation_time - previous_command_simulation_time_ : 0.0;
  diagnostics_.commanded_delta.assign(sample.positions.size(), 0.0);
  diagnostics_.commanded_velocity = sample.velocities;
  if (have_previous_command_ && diagnostics_.command_simulation_dt > 0.0) {
    for (std::size_t index = 0; index < sample.positions.size(); ++index) {
      diagnostics_.commanded_delta[index] =
        sample.positions[index] - diagnostics_.commanded_positions[index];
    }
  }
  diagnostics_.commanded_positions = sample.positions;
  if (profile_trace_) {
    profile_trace_->add_row(
      {
        simulation_time, "SAMPLER_COMMAND", sample.positions, sample.velocities, {},
        diagnostics_.actual_positions, diagnostics_.actual_velocity});
  }
  previous_command_simulation_time_ = simulation_time;
  have_previous_command_ = true;
  const auto final_time = duration_seconds(trajectory_.points.back().time_from_start);
  const bool final_sample = diagnostics_.trajectory_time >= final_time;
  if (final_sample && !final_sample_observed_) {
    diagnostics_.final_sample_first_published_simulation_time = simulation_time;
    final_sample_observed_ = true;
  }
  if (diagnostics_.final_target_active && sample.positions != diagnostics_.final_target) {
    diagnostics_.final_target_changed_after_publish = true;
  }
  ++diagnostics_.command_publish_attempt_count;
  lock.unlock();
  const bool published = command_sink_(sample);
  if (!published) {
    lock.lock();
    failed_ = true;
    active_ = false;
    finished_ = false;
    diagnostics_.final_converged = false;
    failure_reason_ = "trajectory command sink rejected position target";
    return {false, failure_reason_};
  }
  lock.lock();
  ++diagnostics_.command_publish_success_count;
  if (final_sample) {
    ++diagnostics_.final_sample_publish_count;
    diagnostics_.final_target_active = true;
  }
  if (!active_ || failed_) {
    return failed_ ? TrajectorySampleResult{false, failure_reason_} :
           TrajectorySampleResult{true, ""};
  }
  return {true, ""};
}

void TrajectoryExecutionWorker::update_actual_positions(
  const std::vector<double> & positions, double simulation_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (std::isfinite(simulation_time)) {
    diagnostics_.actual_simulation_time = simulation_time;
    diagnostics_.actual_simulation_dt = have_previous_actual_ ?
      simulation_time - previous_actual_simulation_time_ : 0.0;
    diagnostics_.actual_delta.assign(positions.size(), 0.0);
    diagnostics_.actual_velocity.assign(positions.size(), 0.0);
    if (have_previous_actual_ && diagnostics_.actual_simulation_dt > 0.0 &&
      diagnostics_.actual_positions.size() == positions.size())
    {
      for (std::size_t index = 0; index < positions.size(); ++index) {
        diagnostics_.actual_delta[index] = positions[index] - diagnostics_.actual_positions[index];
        diagnostics_.actual_velocity[index] =
          diagnostics_.actual_delta[index] / diagnostics_.actual_simulation_dt;
      }
    }
    previous_actual_simulation_time_ = simulation_time;
    have_previous_actual_ = true;
  }
  diagnostics_.actual_positions = positions;
  if (profile_trace_) {
    profile_trace_->add_row(
      {
        simulation_time, "ARTICULATION_FEEDBACK", {}, {}, {}, positions,
        diagnostics_.actual_velocity});
  }
  diagnostics_.tracking_error.clear();
  diagnostics_.max_tracking_error = 0.0;
  if (positions.size() != diagnostics_.commanded_positions.size()) {
    return;
  }
  diagnostics_.tracking_error.resize(positions.size());
  for (std::size_t index = 0; index < positions.size(); ++index) {
    diagnostics_.tracking_error[index] =
      std::abs(diagnostics_.commanded_positions[index] - positions[index]);
    diagnostics_.max_tracking_error = std::max(
      diagnostics_.max_tracking_error, diagnostics_.tracking_error[index]);
  }
  const auto final_time = duration_seconds(trajectory_.points.back().time_from_start);
  diagnostics_.final_converged = diagnostics_.trajectory_time >= final_time &&
    diagnostics_.max_tracking_error <= convergence_tolerance_;
  if (diagnostics_.trajectory_time >= final_time) {
    if (diagnostics_.final_target_active && std::isfinite(simulation_time)) {
      diagnostics_.final_target_active_duration =
        std::max(0.0, simulation_time - diagnostics_.final_sample_first_published_simulation_time);
    }
    diagnostics_.final_target_active = false;
    active_ = false;
    finished_ = true;
    diagnostics_.execution_complete = true;
  }
}

void TrajectoryExecutionWorker::cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
}

bool TrajectoryExecutionWorker::active() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

bool TrajectoryExecutionWorker::failed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return failed_;
}

bool TrajectoryExecutionWorker::finished() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return finished_;
}

bool TrajectoryExecutionWorker::completion_eligible() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return finished_ && diagnostics_.execution_complete;
}

std::string TrajectoryExecutionWorker::failure_reason() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return failure_reason_;
}

TrajectoryExecutionDiagnostics TrajectoryExecutionWorker::diagnostics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return diagnostics_;
}
}  // namespace arm_cell_motion_moveit2
