#include "arm_cell_task_execution/pick_execution_record.hpp"

#include <array>
#include <cctype>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "arm_cell_motion/motion_result_code_name.hpp"

namespace arm_cell_task_execution
{
namespace
{
std::string json_escape(const std::string & value)
{
  std::ostringstream out;
  for (const auto ch : value) {
    if (ch == '"' || ch == '\\') {out << '\\' << ch;} else if (ch == '\n') {
      out << "\\n";
    } else if (ch == '\r') {out << "\\r";} else if (static_cast<unsigned char>(ch) >= 0x20) {
      out << ch;
    }
  }
  return out.str();
}

void write_pick_execution_record(
  const ExecuteTask::Goal & goal, const std::string & motion_goal_uuid,
  const std::string & stage, uint8_t result_code, bool held, bool retract_succeeded,
  const std::string & candidate_evaluation, const std::filesystem::path & directory)
{
  std::string file_id = motion_goal_uuid.empty() ? "unknown-goal" : motion_goal_uuid;
  for (auto & ch : file_id) {
    if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '-' && ch != '_') {ch = '_';}
  }
  auto candidate_summary = nlohmann::json::object();
  const auto candidate_evidence = nlohmann::json::parse(candidate_evaluation, nullptr, false);
  if (candidate_evidence.is_object()) {
    static const std::array<const char *, 13> kSummaryFields = {
      "generated_candidate_count", "evaluated_candidate_count", "candidate_count",
      "candidate_result", "selected_candidate", "selection_path",
      "selected_candidate_group", "selected_candidate_origin",
      "selected_object_relative_offset_xyz", "selected_tcp_orientation_xyzw",
      "selected_grasp_pose", "selected_approach_pose", "expected_contact_width_mm"};
    for (const auto * field : kSummaryFields) {
      if (candidate_evidence.contains(field)) {
        candidate_summary[field] = candidate_evidence[field];
      }
    }
    if (candidate_evidence.contains("final_failure_stage")) {
      candidate_summary["final_failure_stage"] = candidate_evidence["final_failure_stage"];
      candidate_summary["final_failure_reason"] = candidate_evidence.value(
        "final_failure_reason", "unavailable");
    }
    if (candidate_evidence.contains("rejections")) {
      candidate_summary["rejections"] = candidate_evidence["rejections"];
    }
    if (candidate_evidence.contains("global_planning")) {
      candidate_summary["global_planning"] = candidate_evidence["global_planning"];
    }
    candidate_summary["motion_planning_summary_reference"] = {
      {"path", "motion_planning_summaries/" + file_id + ".json"},
      {"base", "motion_planning_records"}};
  }
  try {
    std::filesystem::create_directories(directory);
    std::ofstream output(directory / (file_id + ".json"), std::ios::trunc);
    if (!output) {throw std::runtime_error("unable to open Pick Execution Record");}
    const auto & q = goal.target_pose.pose.orientation;
    output << std::setprecision(17)
           << "{\n  \"schema_version\": 1,\n"
           << "  \"motion_goal_uuid\": \"" << json_escape(motion_goal_uuid) << "\",\n"
           << "  \"capture_diagnostic_record_relative_path\": ";
    if (motion_goal_uuid.empty()) {
      output << "null,\n";
    } else {
      output << "\"capture_diagnostic_records/" << file_id << ".json\",\n";
    }
    output
      << "  \"vision_acquisition_id\": \"" << json_escape(goal.vision_acquisition_id) << "\",\n"
      << "  \"target_frame_id\": \"" << json_escape(goal.target_pose.header.frame_id) << "\",\n"
      << "  \"requested_grasp_width_min_mm\": " << goal.grasp_width_min_mm << ",\n"
      << "  \"requested_grasp_width_max_mm\": " << goal.grasp_width_max_mm << ",\n"
      << "  \"orientation_symmetry_order\": " <<
      static_cast<unsigned>(goal.orientation_symmetry_order) << ",\n"
      << "  \"orientation_source\": \"" <<
      (goal.orientation_symmetry_order == ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE ?
    "explicit_fallback" : "observed") << "\",\n"
      << "  \"has_unobserved_yaw_fallback\": " <<
      (goal.has_unobserved_yaw_fallback ? "true" : "false") << ",\n"
      << "  \"unobserved_yaw_fallback_rad\": " << goal.unobserved_yaw_fallback_rad << ",\n"
      << "  \"has_observed_rectangle\": " <<
      (goal.has_observed_rectangle ? "true" : "false") << ",\n"
      << "  \"rectangle_long_side_m\": " << goal.rectangle_long_side_m << ",\n"
      << "  \"rectangle_short_side_m\": " << goal.rectangle_short_side_m << ",\n"
      << "  \"estimated_object_height_m\": " << goal.estimated_object_height_m << ",\n"
      << "  \"observation_position_xyz\": [" << goal.target_pose.pose.position.x << ','
      << goal.target_pose.pose.position.y << ',' << goal.target_pose.pose.position.z << "],\n"
      << "  \"observed_orientation_xyzw\": ";
    if (goal.orientation_symmetry_order == ExecuteTask::Goal::ORIENTATION_UNOBSERVABLE) {
      output << "null,\n";
    } else {
      output << '[' << q.x << ',' << q.y << ',' << q.z << ',' << q.w << "],\n";
    }
    output << "  \"candidate_evaluation\": " <<
      (candidate_summary.empty() ? "null" : candidate_summary.dump()) << ",\n"
           << "  \"candidate_evaluation_status\": \"" <<
      (candidate_summary.empty() ? "unavailable" : "available") << "\",\n"
           << "  \"candidate_evaluation_unavailable_reason\": " <<
      (candidate_summary.empty() ?
    "\"candidate evaluation was not produced before terminal result\"" : "null") << ",\n"
           << "  \"selected_tcp_orientation_xyzw\": ";
    if (candidate_summary.empty() || !candidate_summary.contains("selected_tcp_orientation_xyzw")) {
      output << "null,\n";
    } else {
      output << candidate_summary["selected_tcp_orientation_xyzw"].dump() << ",\n";
    }
    output << "  \"execution_stage\": \"" << json_escape(stage) << "\",\n"
           << "  \"result_code\": " << static_cast<unsigned>(result_code) << ",\n"
           << "  \"gripper_held_confirmed\": " << (held ? "true" : "false") << ",\n"
           << "  \"retract_succeeded\": " << (retract_succeeded ? "true" : "false") << "\n}\n";
    if (!output) {throw std::runtime_error("failed while writing Pick Execution Record");}
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      rclcpp::get_logger("task_executor"),
      "Pick Execution Record save failed cause=%s", error.what());
  }
}

class PickExecutionRecordWriter final
{
public:
  PickExecutionRecordWriter()
  : worker_([this]() {run();}) {}

  ~PickExecutionRecordWriter()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    if (worker_.joinable()) {worker_.join();}
  }

  bool submit(std::function<void()> write)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_ || queue_.size() >= kQueueCapacity) {return false;}
      queue_.push_back(std::move(write));
    }
    condition_.notify_one();
    return true;
  }

private:
  static constexpr std::size_t kQueueCapacity = 32;

  void run()
  {
    for (;; ) {
      std::function<void()> write;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() {return stopping_ || !queue_.empty();});
        if (queue_.empty() && stopping_) {return;}
        write = std::move(queue_.front());
        queue_.pop_front();
      }
      try {
        write();
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          rclcpp::get_logger("task_executor"),
          "Pick Execution Record writer failed: %s", error.what());
      } catch (...) {
        RCLCPP_WARN(
          rclcpp::get_logger("task_executor"), "Pick Execution Record writer failed");
      }
    }
  }

  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<std::function<void()>> queue_;
  bool stopping_{false};
  std::thread worker_;
};

PickExecutionRecordWriter & pick_execution_record_writer()
{
  static PickExecutionRecordWriter writer;
  return writer;
}

void queue_pick_execution_record_impl(
  const ExecuteTask::Goal & goal, const std::string & motion_goal_uuid,
  const std::string & stage, uint8_t result_code, bool held, bool retract_succeeded,
  const std::string & candidate_evaluation)
{
  // Copy the bounded terminal snapshot here; formatting and filesystem I/O run
  // on the single writer thread. Queue saturation drops evidence only.
  const char * configured = std::getenv("ARM_CELL_PICK_EXECUTION_RECORD_DIR");
  const char * project_root = std::getenv("SFTWIN_PROJECT_ROOT");
  const auto directory = configured && *configured ? std::filesystem::path(configured) :
    (project_root && *project_root ? std::filesystem::path(project_root) :
    std::filesystem::path(".")) / ".local_artifacts/pick_execution_records";
  const auto accepted = pick_execution_record_writer().submit(
    [goal, motion_goal_uuid, stage, result_code, held, retract_succeeded,
    candidate_evaluation, directory]() {
      write_pick_execution_record(
        goal, motion_goal_uuid, stage, result_code, held, retract_succeeded,
        candidate_evaluation, directory);
    });
  if (!accepted) {
    RCLCPP_WARN(
      rclcpp::get_logger("task_executor"),
      "Pick Execution Record unavailable reason=writer_queue_full");
  }
}

}  // namespace

void queue_pick_execution_record(
  const ExecuteTask::Goal & goal, const std::string & motion_goal_uuid,
  const std::string & stage, std::uint8_t result_code, bool held, bool retract_succeeded,
  const std::string & candidate_evaluation)
{
  queue_pick_execution_record_impl(
    goal, motion_goal_uuid, stage, result_code, held, retract_succeeded, candidate_evaluation);
}

}  // namespace arm_cell_task_execution
