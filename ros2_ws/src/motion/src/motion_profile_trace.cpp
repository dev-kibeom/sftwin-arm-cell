#include "arm_cell_motion/motion_profile_trace.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace arm_cell_motion
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

MotionProfileTrace::MotionProfileTrace(
  std::string output_directory, std::size_t max_samples,
  std::uintmax_t max_directory_bytes)
: storage_(std::make_shared<StorageBudget>(max_directory_bytes)),
  output_directory_(std::move(output_directory)), max_samples_(max_samples)
{
  rows_.reserve(std::min(max_samples_, static_cast<std::size_t>(100000)));
}

void MotionProfileTrace::begin(
  std::string run_id, std::string runtime_identity, std::string profile_identity,
  std::string time_basis, std::string commit_sha,
  std::string motion_goal_uuid, std::uint64_t execution_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rows_.clear();
  dropped_sample_count_ = 0;
  run_id_ = std::move(run_id);
  runtime_identity_ = std::move(runtime_identity);
  profile_identity_ = std::move(profile_identity);
  time_basis_ = std::move(time_basis);
  commit_sha_ = std::move(commit_sha);
  motion_goal_uuid_ = std::move(motion_goal_uuid);
  execution_id_ = execution_id;
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
  const auto temporary_csv = result.samples_path + ".tmp";
  const auto temporary_manifest = result.manifest_path + ".tmp";
  try {
    std::lock_guard<std::mutex> storage_lock(storage_->mutex);
    std::filesystem::create_directories(directory);
    if (!storage_->initialized) {
      for (const auto & entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file()) {storage_->retained_bytes += entry.file_size();}
      }
      storage_->initialized = true;
    }
    if (std::filesystem::exists(result.samples_path) || std::filesystem::exists(
        result.manifest_path))
    {
      throw std::runtime_error("profile destination already exists");
    }
    std::ofstream csv(temporary_csv, std::ios::trunc);
    if (!csv) {
      throw std::runtime_error("unable to open profile samples artifact");
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
    std::size_t row_number = 0;
    for (const auto & row : rows_) {
      csv << row.simulation_time << ',' << row.kind;
      csv_values(csv, row.commanded_positions, joint_names.size());
      csv_values(csv, row.commanded_velocities, joint_names.size());
      csv_values(csv, row.commanded_accelerations, joint_names.size());
      csv_values(csv, row.actual_positions, joint_names.size());
      csv_values(csv, row.actual_velocities, joint_names.size());
      csv << '\n';
      if (++row_number % 1024U == 0U &&
        storage_->retained_bytes + static_cast<std::uintmax_t>(csv.tellp()) > storage_->limit)
      {throw std::runtime_error("profile directory byte limit reached");}
    }
    csv.close();
    if (!csv) {
      throw std::runtime_error("failed while writing profile samples artifact");
    }
    std::ofstream manifest(temporary_manifest, std::ios::trunc);
    if (!manifest) {
      throw std::runtime_error("unable to open profile evidence manifest");
    }
    manifest << "{\n"
             << "  \"evidence_id\": " << json_string("motion-profile-" + run_id_) << ",\n"
             << "  \"run_id\": " << json_string(run_id_) << ",\n"
             << "  \"motion_goal_uuid\": " <<
    (motion_goal_uuid_.empty() ? "null" : json_string(motion_goal_uuid_)) << ",\n"
             << "  \"execution_id\": " <<
    (execution_id_ == 0 ? "null" : std::to_string(execution_id_)) << ",\n"
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
      throw std::runtime_error("failed while writing profile evidence manifest");
    }
    manifest.close();
    if (!manifest) {throw std::runtime_error("profile manifest close failed");}
    const auto csv_bytes = std::filesystem::file_size(temporary_csv);
    const auto manifest_bytes = std::filesystem::file_size(temporary_manifest);
    if (storage_->retained_bytes + csv_bytes + manifest_bytes > storage_->limit) {
      throw std::runtime_error("profile directory byte limit reached");
    }
    std::filesystem::rename(temporary_csv, result.samples_path);
    storage_->retained_bytes += csv_bytes;
    std::filesystem::rename(temporary_manifest, result.manifest_path);
    storage_->retained_bytes += manifest_bytes;
    result.success = true;
  } catch (const std::exception & error) {
    result.error = error.what();
  }
  std::error_code cleanup_error;
  std::filesystem::remove(temporary_csv, cleanup_error);
  std::filesystem::remove(temporary_manifest, cleanup_error);
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
  snapshot->storage_ = storage_;
  snapshot->dropped_sample_count_ = dropped_sample_count_;
  snapshot->run_id_ = std::move(run_id_);
  snapshot->runtime_identity_ = std::move(runtime_identity_);
  snapshot->profile_identity_ = std::move(profile_identity_);
  snapshot->time_basis_ = std::move(time_basis_);
  snapshot->commit_sha_ = std::move(commit_sha_);
  snapshot->motion_goal_uuid_ = std::move(motion_goal_uuid_);
  snapshot->execution_id_ = execution_id_;
  dropped_sample_count_ = 0;
  return snapshot;
}

}  // namespace arm_cell_motion
