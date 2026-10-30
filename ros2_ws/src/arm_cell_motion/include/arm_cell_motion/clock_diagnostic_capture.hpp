#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace arm_cell_motion {

// Component-local, best-effort clock evidence. observe() only copies bounded
// typed values into memory; all filesystem work belongs to the writer thread.
class ClockDiagnosticCapture final {
public:
  struct Sample {
    int64_t clock_time_ns{0};
    int64_t node_ros_time_ns{0};
    bool node_ros_time_available{false};
    bool node_ros_time_sim_active{false};
    int64_t receipt_steady_ns{0};
    int64_t source_age_ns{0};
    bool source_age_available{false};
    int64_t decision_steady_ns{0};
    double receipt_age_s{0.0};
    bool receipt_age_available{false};
    double watchdog_threshold_s{0.0};
    std::string watchdog_result{"unavailable"};
    std::string observer;
    std::string existing_execution_id;
  };

  ClockDiagnosticCapture(std::string artifact_root,
                         std::string process_instance,
                         std::chrono::steady_clock::duration
                             receipt_gap_threshold = std::chrono::seconds(2),
                         std::string capture_id = {});
  ~ClockDiagnosticCapture();
  ClockDiagnosticCapture(const ClockDiagnosticCapture &) = delete;
  ClockDiagnosticCapture &operator=(const ClockDiagnosticCapture &) = delete;

  // Returns the local classification for this received /clock sample.
  std::string observe(const Sample &sample);
  void record_decision(const Sample &sample);
  uint64_t epoch() const;

private:
  struct Event {
    Sample sample;
    std::string name;
    uint64_t sequence{0};
    uint64_t epoch{0};
    int64_t rtf_start_clock_ns{0};
    int64_t rtf_start_steady_ns{0};
    int64_t forward_jump_threshold_ns{0};
    double rtf{0.0};
    bool rtf_available{false};
  };

  void enqueue_locked(const Event &event);
  void start_capture_locked();
  void writer_loop();
  void write_manifest(bool final);

  std::string artifact_root_;
  std::string process_instance_;
  std::string capture_id_;
  std::chrono::steady_clock::duration receipt_gap_threshold_;
  int64_t receipt_gap_threshold_ns_{0};
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<Event> ring_;
  std::deque<Event> queue_;
  Event previous_;
  bool has_previous_{false};
  bool stopping_{false};
  bool triggered_{false};
  bool capture_active_{false};
  bool watchdog_stall_latched_{false};
  bool writer_failed_{false};
  bool truncated_{false};
  uint64_t epoch_{0};
  uint64_t sequence_{0};
  uint64_t last_enqueued_sequence_{0};
  uint64_t observed_count_{0};
  uint64_t retained_count_{0};
  uint64_t dropped_count_{0};
  uint64_t written_count_{0};
  uint64_t written_bytes_{0};
  int64_t coverage_start_steady_ns_{0};
  int64_t coverage_end_steady_ns_{0};
  std::size_t post_event_remaining_{0};
  std::size_t post_event_extension_remaining_{0};
  std::string last_classification_;
  std::thread writer_;
};

} // namespace arm_cell_motion
