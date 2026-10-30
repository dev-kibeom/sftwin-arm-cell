#include "arm_cell_motion_moveit2/motion_profile_trace.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace arm_cell_motion_moveit2
{
namespace
{
std::string safe_filename(std::string value)
{
  std::replace_if(
    value.begin(), value.end(), [](unsigned char c) {
      return !std::isalnum(c) && c != '-' && c != '_';
    }, '_');
  return value.empty() ? "run" : value;
}

std::string json_string(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  for (const auto c : value) {
    if (c == '"' || c == '\\') {
      out << '\\' << c;
    } else if (c == '\n') {
      out << "\\n";
    } else if (c == '\r') {
      out << "\\r";
    } else if (c == '\t') {
      out << "\\t";
    } else if (static_cast<unsigned char>(c) >= 0x20) {
      out << c;
    }
  }
  out << '"';
  return out.str();
}

void csv_values(std::ostream & out, const std::vector<double> & values, std::size_t columns)
{
  for (std::size_t i = 0; i < columns; ++i) {
    out << ',';
    if (i < values.size() && std::isfinite(values[i])) {
      out << values[i];
    }
  }
}
}  // namespace

MotionProfileTrace::MotionProfileTrace(std::string output_directory, std::size_t max_samples)
: output_directory_(std::move(output_directory)), max_samples_(max_samples)
{
  rows_.reserve(std::min(max_samples_, static_cast<std::size_t>(100000)));
}

void MotionProfileTrace::begin(
  std::string run_id, std::string runtime_identity, std::string profile_identity,
  std::string time_basis, std::string commit_sha)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rows_.clear();
  dropped_sample_count_ = 0;
  run_id_ = std::move(run_id);
  runtime_identity_ = std::move(runtime_identity);
  profile_identity_ = std::move(profile_identity);
  time_basis_ = std::move(time_basis);
  commit_sha_ = std::move(commit_sha);
}

bool MotionProfileTrace::add_row(MotionProfileTraceRow row)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(row.simulation_time)) {
    return false;
  }
  if (rows_.size() >= max_samples_) {
    ++dropped_sample_count_;
    return false;
  }
  rows_.push_back(std::move(row));
  return true;
}

MotionProfileTraceResult MotionProfileTrace::finish(
  std::string outcome, const std::vector<std::string> & joint_names,
  std::string active_configuration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  MotionProfileTraceResult result;
  result.sample_count = rows_.size();
  result.dropped_sample_count = dropped_sample_count_;
  if (!rows_.empty()) {
    result.simulation_start = rows_.front().simulation_time;
    result.simulation_end = rows_.front().simulation_time;
    for (const auto & row : rows_) {
      result.simulation_start = std::min(result.simulation_start, row.simulation_time);
      result.simulation_end = std::max(result.simulation_end, row.simulation_time);
    }
  }
  const auto basename = safe_filename(run_id_);
  const auto directory = std::filesystem::path(output_directory_);
  result.samples_path = (directory / (basename + ".csv")).string();
  result.manifest_path = (directory / (basename + ".manifest.json")).string();
  try {
    std::filesystem::create_directories(directory);
    std::ofstream csv(result.samples_path, std::ios::trunc);
    if (!csv) {
      result.error = "unable to open profile samples artifact";
      return result;
    }
    csv << std::setprecision(17) << "simulation_time,kind";
    for (const auto & name : joint_names) {
      csv << ",commanded_position[" << name << ']';
    }
    for (const auto & name : joint_names) {
      csv << ",commanded_velocity[" << name << ']';
    }
    for (const auto & name : joint_names) {
      csv << ",commanded_acceleration[" << name << ']';
    }
    for (const auto & name : joint_names) {
      csv << ",actual_position[" << name << ']';
    }
    for (const auto & name : joint_names) {
      csv << ",actual_velocity[" << name << ']';
    }
    csv << '\n';
    for (const auto & row : rows_) {
      csv << row.simulation_time << ',' << row.kind;
      csv_values(csv, row.commanded_positions, joint_names.size());
      csv_values(csv, row.commanded_velocities, joint_names.size());
      csv_values(csv, row.commanded_accelerations, joint_names.size());
      csv_values(csv, row.actual_positions, joint_names.size());
      csv_values(csv, row.actual_velocities, joint_names.size());
      csv << '\n';
    }
    csv.close();
    if (!csv) {
      result.error = "failed while writing profile samples artifact";
      return result;
    }
    std::ofstream manifest(result.manifest_path, std::ios::trunc);
    if (!manifest) {
      result.error = "unable to open profile evidence manifest";
      return result;
    }
    manifest << "{\n"
             << "  \"evidence_id\": " << json_string("motion-profile-" + run_id_) << ",\n"
             << "  \"run_id\": " << json_string(run_id_) << ",\n"
             << "  \"runtime_identity\": " << json_string(runtime_identity_) << ",\n"
             << "  \"profile_identity\": " << json_string(profile_identity_) << ",\n"
             << "  \"commit_sha\": " << json_string(commit_sha_) << ",\n"
             << "  \"time_basis\": " << json_string(time_basis_) << ",\n"
             << "  \"outcome\": " << json_string(outcome) << ",\n"
             << "  \"active_configuration\": " << json_string(active_configuration) << ",\n"
             << "  \"joint_names\": [";
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
      if (i != 0) {manifest << ", ";}
      manifest << json_string(joint_names[i]);
    }
    manifest << "],\n"
             << "  \"artifact\": " << json_string(result.samples_path) << ",\n"
             << "  \"sample_count\": " << rows_.size() << ",\n"
             << "  \"dropped_sample_count\": " << dropped_sample_count_ << ",\n"
             << "  \"simulation_start\": " << result.simulation_start << ",\n"
             << "  \"simulation_end\": " << result.simulation_end << ",\n"
             << "  \"truncated\": " << (dropped_sample_count_ != 0 ? "true" : "false") << "\n"
             << "}\n";
    if (!manifest) {
      result.error = "failed while writing profile evidence manifest";
      return result;
    }
    result.success = true;
  } catch (const std::exception & error) {
    result.error = error.what();
  }
  return result;
}

std::size_t MotionProfileTrace::sample_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rows_.size();
}

std::shared_ptr<MotionProfileTrace> MotionProfileTrace::take_snapshot()
{
  auto snapshot = std::make_shared<MotionProfileTrace>(output_directory_, max_samples_);
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot->rows_ = std::move(rows_);
  snapshot->dropped_sample_count_ = dropped_sample_count_;
  snapshot->run_id_ = std::move(run_id_);
  snapshot->runtime_identity_ = std::move(runtime_identity_);
  snapshot->profile_identity_ = std::move(profile_identity_);
  snapshot->time_basis_ = std::move(time_basis_);
  snapshot->commit_sha_ = std::move(commit_sha_);
  dropped_sample_count_ = 0;
  return snapshot;
}

}  // namespace arm_cell_motion_moveit2
