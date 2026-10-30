#include "arm_cell_motion/clock_diagnostic_capture.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace arm_cell_motion {
namespace {
constexpr std::size_t kRingCapacity = 64;
constexpr std::size_t kQueueCapacity = 256;
constexpr std::size_t kPostEventSamples = 32;
constexpr std::size_t kMaximumPostEventSamples = 64;
constexpr std::size_t kAnomalyExtensionSamples = 8;
constexpr uint64_t kRecordLimit = 512;
constexpr uint64_t kByteLimit = 512 * 1024;

std::string escape_json(const std::string &value) {
  std::ostringstream escaped;
  for (const unsigned char ch : value) {
    switch (ch) {
    case '"':
      escaped << "\\\"";
      break;
    case '\\':
      escaped << "\\\\";
      break;
    case '\n':
      escaped << "\\n";
      break;
    case '\r':
      escaped << "\\r";
      break;
    case '\t':
      escaped << "\\t";
      break;
    default:
      if (ch >= 0x20) {
        escaped << ch;
      }
    }
  }
  return escaped.str();
}

std::string make_capture_id() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return "clock-" +
         std::to_string(
             std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}
} // namespace

ClockDiagnosticCapture::ClockDiagnosticCapture(
    std::string artifact_root, std::string process_instance,
    std::chrono::steady_clock::duration receipt_gap_threshold,
    std::string capture_id)
    : artifact_root_(std::move(artifact_root)),
      process_instance_(std::move(process_instance)),
      capture_id_(capture_id.empty() ? make_capture_id()
                                     : std::move(capture_id)),
      receipt_gap_threshold_(receipt_gap_threshold),
      receipt_gap_threshold_ns_(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              receipt_gap_threshold)
              .count()),
      writer_(&ClockDiagnosticCapture::writer_loop, this) {}

ClockDiagnosticCapture::~ClockDiagnosticCapture() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  condition_.notify_one();
  if (writer_.joinable()) {
    writer_.join();
  }
  write_manifest(true);
}

std::string ClockDiagnosticCapture::observe(const Sample &sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++observed_count_;
  if (coverage_start_steady_ns_ == 0) {
    coverage_start_steady_ns_ = sample.receipt_steady_ns;
  }
  coverage_end_steady_ns_ = sample.receipt_steady_ns;
  Event event;
  event.sample = sample;
  event.sequence = ++sequence_;
  event.epoch = epoch_;
  event.name = "clock_progress";

  const auto steady = std::chrono::steady_clock::time_point(
      std::chrono::nanoseconds(sample.receipt_steady_ns));
  std::string classification = "clock_progress";
  bool anomaly = false;
  if (!has_previous_) {
    classification = "initialized";
  } else {
    const auto prior_steady = std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(previous_.sample.receipt_steady_ns));
    const auto steady_delta = steady - prior_steady;
    const auto clock_delta =
        sample.clock_time_ns - previous_.sample.clock_time_ns;
    if (clock_delta < 0) {
      ++epoch_;
      classification = "backward_epoch";
      anomaly = true;
    } else if (steady_delta > receipt_gap_threshold_) {
      classification =
          clock_delta > 0 ? "receipt_gap_and_resume" : "receipt_gap";
      anomaly = true;
    } else if (clock_delta == 0) {
      classification = "repeated_receipt";
      anomaly = last_classification_ != classification;
    } else if (last_classification_ == "repeated_receipt" ||
               last_classification_ == "receipt_gap") {
      classification = "clock_resumed";
      anomaly = true;
    } else {
      const auto steady_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(steady_delta)
              .count();
      const auto forward_jump_threshold_ns =
          std::max<int64_t>(0, steady_ns) * 2 + 1000000000LL;
      event.forward_jump_threshold_ns = forward_jump_threshold_ns;
      if (clock_delta > forward_jump_threshold_ns) {
        classification = "suspected_forward_jump";
        anomaly = true;
      }
      if (steady_ns > 0) {
        event.rtf_start_clock_ns = previous_.sample.clock_time_ns;
        event.rtf_start_steady_ns = previous_.sample.receipt_steady_ns;
        event.rtf =
            static_cast<double>(clock_delta) / static_cast<double>(steady_ns);
        event.rtf_available = true;
      }
    }
  }
  event.name = classification;
  event.epoch = epoch_;
  if (classification == "clock_progress" || classification == "clock_resumed") {
    watchdog_stall_latched_ = false;
  }
  ring_.push_back(event);
  if (ring_.size() > kRingCapacity) {
    ring_.pop_front();
  }

  if (anomaly && !capture_active_) {
    start_capture_locked();
  } else if (capture_active_ && post_event_remaining_ > 0) {
    enqueue_locked(event);
    --post_event_remaining_;
    if (anomaly) {
      const auto extension =
          std::min(kAnomalyExtensionSamples, post_event_extension_remaining_);
      post_event_remaining_ += extension;
      post_event_extension_remaining_ -= extension;
    }
    if (post_event_remaining_ == 0) {
      capture_active_ = false;
    }
  }
  previous_ = event;
  has_previous_ = true;
  last_classification_ = classification;
  return classification;
}

void ClockDiagnosticCapture::record_decision(const Sample &sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  Event event;
  event.sample = sample;
  event.name = "watchdog_decision";
  event.sequence = ++sequence_;
  event.epoch = epoch_;
  ring_.push_back(event);
  if (ring_.size() > kRingCapacity) {
    ring_.pop_front();
  }
  const bool stall = sample.watchdog_result == "SIMULATION_CLOCK_STALL";
  const bool start_stall_capture =
      stall && !watchdog_stall_latched_ && !capture_active_;
  if (start_stall_capture) {
    start_capture_locked();
  }
  if (stall) {
    watchdog_stall_latched_ = true;
  } else {
    watchdog_stall_latched_ = false;
  }
  if (!start_stall_capture && capture_active_ && post_event_remaining_ > 0) {
    enqueue_locked(event);
    --post_event_remaining_;
    if (post_event_remaining_ == 0) {
      capture_active_ = false;
    }
  }
}

uint64_t ClockDiagnosticCapture::epoch() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return epoch_;
}

void ClockDiagnosticCapture::enqueue_locked(const Event &event) {
  if (event.sequence <= last_enqueued_sequence_) {
    return;
  }
  last_enqueued_sequence_ = event.sequence;
  if (retained_count_ >= kRecordLimit) {
    truncated_ = true;
    return;
  }
  if (queue_.size() >= kQueueCapacity) {
    ++dropped_count_;
    return;
  }
  queue_.push_back(event);
  ++retained_count_;
  condition_.notify_one();
}

void ClockDiagnosticCapture::start_capture_locked() {
  triggered_ = true;
  capture_active_ = true;
  post_event_remaining_ = kPostEventSamples;
  post_event_extension_remaining_ =
      kMaximumPostEventSamples - kPostEventSamples;
  for (const auto &retained : ring_) {
    enqueue_locked(retained);
  }
}

void ClockDiagnosticCapture::writer_loop() {
  const auto directory =
      std::filesystem::path(artifact_root_) / capture_id_ / process_instance_;
  std::ofstream output;
  uint64_t written_bytes = 0;
  while (true) {
    Event event;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]() { return stopping_ || !queue_.empty(); });
      if (queue_.empty() && stopping_) {
        break;
      }
      event = std::move(queue_.front());
      queue_.pop_front();
    }
    if (!output.is_open()) {
      std::error_code error;
      std::filesystem::create_directories(directory, error);
      if (error) {
        std::lock_guard<std::mutex> lock(mutex_);
        writer_failed_ = true;
        continue;
      }
      output.open(directory / "events.jsonl", std::ios::out | std::ios::app);
      std::error_code size_error;
      const auto existing_bytes =
          std::filesystem::file_size(directory / "events.jsonl", size_error);
      if (!size_error) {
        written_bytes = existing_bytes;
        std::lock_guard<std::mutex> lock(mutex_);
        written_bytes_ = written_bytes;
        if (written_bytes >= kByteLimit) {
          truncated_ = true;
        }
      }
    }
    if (!output) {
      std::lock_guard<std::mutex> lock(mutex_);
      writer_failed_ = true;
      continue;
    }
    const auto &s = event.sample;
    std::ostringstream record;
    record << "{\"schema_version\":1,\"capture_id\":\""
           << escape_json(capture_id_) << "\",\"process_instance\":\""
           << escape_json(process_instance_) << "\",\"observer\":\""
           << escape_json(s.observer) << "\",\"phase\":\""
           << (event.name == "watchdog_decision" ? "decision" : "observation")
           << "\",\"event\":\"" << escape_json(event.name)
           << "\",\"sequence\":" << event.sequence
           << ",\"clock_epoch\":" << event.epoch
           << ",\"clock_domain\":\"ros_sim_time\",\"clock_time_ns\":"
           << s.clock_time_ns << ",\"forward_jump_threshold_ns\":";
    if (event.forward_jump_threshold_ns > 0) {
      record << event.forward_jump_threshold_ns;
    } else {
      record << "null";
    }
    record << ",\"rtf_interval\":{\"start_clock_ns\":";
    if (event.rtf_available) {
      record << event.rtf_start_clock_ns;
    } else {
      record << "null";
    }
    record << ",\"start_receipt_steady_ns\":";
    if (event.rtf_available) {
      record << event.rtf_start_steady_ns;
    } else {
      record << "null";
    }
    record << ",\"end_clock_ns\":";
    if (event.rtf_available) {
      record << s.clock_time_ns;
    } else {
      record << "null";
    }
    record << ",\"end_receipt_steady_ns\":";
    if (event.rtf_available) {
      record << s.receipt_steady_ns;
    } else {
      record << "null";
    }
    record << ",\"sample_count\":" << (event.rtf_available ? 2 : 0) << "}"
           << ",\"node_ros_time_ns\":";
    if (s.node_ros_time_available) {
      record << s.node_ros_time_ns;
    } else {
      record << "null";
    }
    record << ",\"node_ros_time_status\":\""
           << (s.node_ros_time_available ? "observed" : "unavailable") << "\""
           << ",\"node_ros_time_sim_active\":"
           << (s.node_ros_time_sim_active ? "true" : "false")
           << ",\"source_age_ns\":";
    if (s.source_age_available) {
      record << s.source_age_ns;
    } else {
      record << "null";
    }
    record << ",\"source_age_status\":\""
           << (s.source_age_available ? "observed_same_ros_clock_domain"
                                      : "unavailable")
           << "\""
           << ",\"receipt_steady_ns\":" << s.receipt_steady_ns
           << ",\"decision_steady_ns\":";
    if (s.decision_steady_ns != 0) {
      record << s.decision_steady_ns;
    } else {
      record << "null";
    }
    record << ",\"receipt_age_s\":";
    if (s.receipt_age_available) {
      record << std::setprecision(9) << s.receipt_age_s;
    } else {
      record << "null";
    }
    record << ",\"rtf\":";
    if (event.rtf_available) {
      record << std::setprecision(9) << event.rtf;
    } else {
      record << "null";
    }
    record << ",\"rtf_status\":\""
           << (event.rtf_available ? "observed_interval" : "unavailable")
           << "\""
           << ",\"watchdog_threshold_s\":";
    if (s.watchdog_threshold_s > 0.0) {
      record << s.watchdog_threshold_s;
    } else {
      record << "null";
    }
    record << ",\"watchdog_result\":\"" << escape_json(s.watchdog_result)
           << "\",\"execution_id\":";
    if (s.existing_execution_id.empty()) {
      record << "null";
    } else {
      record << '"' << escape_json(s.existing_execution_id) << '"';
    }
    record << ",\"isaac_sim_time\":{\"status\":\"unavailable\",\"reason\":"
              "\"publish_graph_has_no_non_intrusive_observation_callback\"}"
           << "}\n";
    const auto line = record.str();
    if (written_bytes + line.size() > kByteLimit) {
      std::lock_guard<std::mutex> lock(mutex_);
      truncated_ = true;
      continue;
    }
    output << line;
    if (!output) {
      std::lock_guard<std::mutex> lock(mutex_);
      writer_failed_ = true;
    } else {
      written_bytes += line.size();
      std::lock_guard<std::mutex> lock(mutex_);
      written_bytes_ = written_bytes;
      ++written_count_;
    }
  }
  if (output.is_open()) {
    output.flush();
    if (!output) {
      std::lock_guard<std::mutex> lock(mutex_);
      writer_failed_ = true;
    }
    output.close();
    if (output.fail()) {
      std::lock_guard<std::mutex> lock(mutex_);
      writer_failed_ = true;
    }
  }
}

void ClockDiagnosticCapture::write_manifest(bool final) {
  const auto directory =
      std::filesystem::path(artifact_root_) / capture_id_ / process_instance_;
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  std::ofstream output(directory / "manifest.json",
                       std::ios::out | std::ios::trunc);
  if (!output) {
    writer_failed_ = true;
    return;
  }
  output << "{\"schema_version\":1,\"capture_id\":\""
         << escape_json(capture_id_) << "\",\"process_instance\":\""
         << escape_json(process_instance_)
         << "\",\"observer\":\"motion_node\",\"artifact\":\"events.jsonl\""
         << ",\"clock_domains\":[\"ros_sim_time\",\"steady_clock_local\"]"
         << ",\"receipt_gap_threshold_ns\":" << receipt_gap_threshold_ns_
         << ",\"isaac_sim_time_status\":\"unavailable\",\"observed_count\":"
         << observed_count_ << ",\"retained_count\":" << retained_count_
         << ",\"dropped_count\":" << dropped_count_
         << ",\"queue_capacity\":" << kQueueCapacity
         << ",\"ring_capacity\":" << kRingCapacity
         << ",\"record_limit\":" << kRecordLimit
         << ",\"byte_limit\":" << kByteLimit
         << ",\"written_count\":" << written_count_
         << ",\"written_bytes\":" << written_bytes_
         << ",\"coverage_start_steady_ns\":";
  if (coverage_start_steady_ns_ != 0) {
    output << coverage_start_steady_ns_;
  } else {
    output << "null";
  }
  output << ",\"coverage_end_steady_ns\":";
  if (coverage_end_steady_ns_ != 0) {
    output << coverage_end_steady_ns_;
  } else {
    output << "null";
  }
  output
      << ",\"coverage_mode\":\"anomaly_pre_event_ring_plus_post_event_samples\""
      << ",\"writer_failed\":" << (writer_failed_ ? "true" : "false")
      << ",\"truncated\":" << (truncated_ ? "true" : "false")
      << ",\"coverage_start\":\""
      << (observed_count_ > 0 ? "first_clock_received" : "clock_not_observed")
      << "\""
      << ",\"window_complete\":"
      << (triggered_ && post_event_remaining_ == 0 ? "true" : "false")
      << ",\"shutdown_status\":\""
      << (final && !writer_failed_ ? "clean" : "incomplete") << "\""
      << ",\"complete\":"
      << (final && observed_count_ > 0 && triggered_ &&
                  post_event_remaining_ == 0 && !writer_failed_ &&
                  !truncated_ && dropped_count_ == 0
              ? "true"
              : "false")
      << ",\"finalized\":" << (final ? "true" : "false") << "}\n";
}

} // namespace arm_cell_motion
