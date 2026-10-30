#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <arm_cell_interfaces/msg/collision_primitive.hpp>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arm_cell_motion
{

class MotionPlanningDiagnosticCapture final
{
public:
  MotionPlanningDiagnosticCapture(
    std::string directory, rclcpp::Logger logger,
    std::string snapshot_directory = ".local_artifacts/snapshots/motion");
  ~MotionPlanningDiagnosticCapture();

  bool enqueue(
    const std::string & goal_uuid, const char * task_type, uint8_t result_code,
    const std::string & evidence, const std::string & vision_acquisition_id = {});
  bool enqueue_progress(
    const std::string & goal_uuid, const char * task_type, const std::string & evidence,
    const std::string & vision_acquisition_id = {});
  bool enqueue_occupancy_snapshot(
    const std::string & goal_uuid, const std::string & vision_acquisition_id,
    const std::string & candidate_id,
    const std::vector<arm_cell_interfaces::msg::CollisionPrimitive> & primitives,
    std::string & relative_path);
  std::size_t enqueue_pick_details(
    const std::string & goal_uuid, const std::string & vision_acquisition_id,
    const std::string & evidence);
  bool enqueue_pick_summary(
    const std::string & goal_uuid, const std::string & vision_acquisition_id,
    uint8_t result_code, bool insertion_started, const std::string & evidence,
    std::size_t accepted_detail_count);

private:
  struct PendingRecord
  {
    std::string goal_uuid;
    std::string file_name;
    std::string serialized;
    bool finalize_coverage{false};
    uint64_t detail_generation{0U};
  };

  struct DetailSnapshot
  {
    nlohmann::json value;
    uint64_t generation{0U};
    bool write_pending{false};
    bool persisted{false};
  };

  struct PendingOccupancySnapshot
  {
    std::string file_name;
    std::string acquisition_id;
    std::string candidate_id;
    std::vector<arm_cell_interfaces::msg::CollisionPrimitive> primitives;
  };

  void write_loop() noexcept;
  void queue_ready_summary_finalizations();
  void clear_detail_snapshots_for_goal(const std::string & goal_uuid);
  static std::string safe_goal_id(std::string value);
  bool enqueue_serialized(const std::string & goal_uuid, std::string serialized);

  static constexpr std::size_t kQueueCapacity = 16;
  static constexpr std::size_t kDetailQueueCapacity = 128;
  static constexpr std::size_t kSummaryQueueCapacity = 64;
  static constexpr std::size_t kOccupancySnapshotQueueCapacity = 128;
  static constexpr std::size_t kDetailSnapshotCacheCapacity = 256;
  static constexpr std::size_t kMaxRecordBytes = 512U * 1024U;
  static constexpr std::size_t kMaxDetailBytes = 64U * 1024U;
  static constexpr std::size_t kMaxSummaryBytes = 64U * 1024U;
  static constexpr std::uintmax_t kMaxDirectoryBytes = 16U * 1024U * 1024U;

  std::string directory_;
  std::string snapshot_directory_;
  rclcpp::Logger logger_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<PendingRecord> queue_;
  std::deque<PendingRecord> detail_queue_;
  std::deque<PendingRecord> summary_queue_;
  std::deque<PendingOccupancySnapshot> occupancy_snapshot_queue_;
  std::unordered_map<std::string, PendingRecord> pending_summary_coverage_;
  std::unordered_map<std::string, DetailSnapshot> detail_snapshots_;
  std::unordered_map<std::string, std::string> occupancy_snapshot_paths_;
  uint64_t occupancy_snapshot_sequence_{0U};
  bool stopping_{false};
  std::thread worker_;
};

}  // namespace arm_cell_motion
