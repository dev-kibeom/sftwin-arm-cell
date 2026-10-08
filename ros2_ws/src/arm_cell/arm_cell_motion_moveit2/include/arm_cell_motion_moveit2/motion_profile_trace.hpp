#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace arm_cell_motion_moveit2
{

struct MotionProfileTraceRow
{
  double simulation_time{0.0};
  std::string kind;
  std::vector<double> commanded_positions;
  std::vector<double> commanded_velocities;
  std::vector<double> commanded_accelerations;
  std::vector<double> actual_positions;
  std::vector<double> actual_velocities;
};

struct MotionProfileTraceResult
{
  bool success{false};
  std::string error;
  std::string samples_path;
  std::string manifest_path;
  std::size_t sample_count{0};
  std::size_t dropped_sample_count{0};
  double simulation_start{0.0};
  double simulation_end{0.0};
};

class MotionProfileTrace final
{
public:
  explicit MotionProfileTrace(std::string output_directory, std::size_t max_samples = 100000);

  void begin(
    std::string run_id, std::string runtime_identity, std::string profile_identity,
    std::string time_basis, std::string commit_sha);
  bool add_row(MotionProfileTraceRow row);
  MotionProfileTraceResult finish(
    std::string outcome, const std::vector<std::string> & joint_names,
    std::string active_configuration);
  std::shared_ptr<MotionProfileTrace> take_snapshot();
  std::size_t sample_count() const;

private:
  std::string output_directory_;
  std::size_t max_samples_;
  mutable std::mutex mutex_;
  std::vector<MotionProfileTraceRow> rows_;
  std::size_t dropped_sample_count_{0};
  std::string run_id_;
  std::string runtime_identity_;
  std::string profile_identity_;
  std::string time_basis_;
  std::string commit_sha_;
};

}  // namespace arm_cell_motion_moveit2
