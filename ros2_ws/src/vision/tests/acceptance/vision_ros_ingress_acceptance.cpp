#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "arm_cell_vision/vision_ros_ingress.hpp"
#include "vision_ros_ingress_acceptance_logic.hpp"

namespace
{

using namespace std::chrono_literals;

const char * status_name(arm_cell_vision::IngressStatus status)
{
  using arm_cell_vision::IngressStatus;
  switch (status) {
    case IngressStatus::kWaiting: return "waiting";
    case IngressStatus::kNoActiveAcquisition: return "no_active_acquisition";
    case IngressStatus::kObservationReady: return "observation_ready";
    case IngressStatus::kDuplicateObservation: return "duplicate_observation";
    case IngressStatus::kSynchronizationSlopExceeded: return "sync_slop_exceeded";
    case IngressStatus::kStaleObservation: return "stale_observation";
    case IngressStatus::kReceiptStaleObservation: return "receipt_stale_observation";
    case IngressStatus::kCalibrationUnavailable: return "calibration_unavailable";
    case IngressStatus::kIncompatibleMetadata: return "incompatible_metadata";
    case IngressStatus::kMalformedObservation: return "malformed_observation";
    case IngressStatus::kRequestTimeout: return "request_timeout";
    case IngressStatus::kTimeRollback: return "time_rollback";
    case IngressStatus::kEpochInvalidated: return "epoch_invalidated";
  }
  return "unknown";
}

template<typename T>
void print_optional(const char * key, const std::optional<T> & value, bool & first)
{
  if (!first) {std::cout << ",";}
  first = false;
  std::cout << "\"" << key << "\":";
  if (value) {std::cout << *value;} else {std::cout << "null";}
}

void print_snapshot(const arm_cell_vision::VisionIngressAcceptanceSnapshot & snapshot)
{
  bool first = true;
  std::cout << "{";
  std::cout << "\"synchronized_callback_count\":" << snapshot.synchronized_callback_count;
  first = false;
  std::cout << ",\"policy_delivery_count\":" << snapshot.policy_delivery_count;
  std::cout << ",\"observation_ready_count\":" << snapshot.observation_ready_count;
  print_optional("latest_rgb_stamp_ns", snapshot.latest_rgb_stamp_ns, first);
  print_optional("latest_depth_stamp_ns", snapshot.latest_depth_stamp_ns, first);
  print_optional("latest_camera_info_stamp_ns", snapshot.latest_camera_info_stamp_ns, first);
  print_optional("latest_canonical_stamp_ns", snapshot.latest_canonical_stamp_ns, first);
  print_optional(
    "latest_rgb_receipt_monotonic_ns", snapshot.latest_rgb_receipt_monotonic_ns,
    first);
  print_optional(
    "latest_depth_receipt_monotonic_ns", snapshot.latest_depth_receipt_monotonic_ns,
    first);
  print_optional(
    "latest_camera_info_receipt_monotonic_ns",
    snapshot.latest_camera_info_receipt_monotonic_ns, first);
  print_optional("latest_oldest_receipt_age_ns", snapshot.latest_oldest_receipt_age_ns, first);
  print_optional("latest_receipt_spread_ns", snapshot.latest_receipt_spread_ns, first);
  print_optional("latest_clock_stamp_ns", snapshot.latest_clock_stamp_ns, first);
  if (!first) {std::cout << ",";}
  std::cout << "\"clock_sample_count\":" << snapshot.clock_sample_count;
  std::cout << ",\"clock_rollback_observed\":"
            << (snapshot.clock_rollback_observed ? "true" : "false");
  std::cout << ",\"observation_rollback_observed\":"
            << (snapshot.observation_rollback_observed ? "true" : "false");
  std::cout << ",\"epoch_invalidated\":"
            << (snapshot.epoch_invalidated ? "true" : "false");
  std::cout << ",\"last_status\":";
  if (snapshot.last_status) {
    std::cout << "\"" << status_name(*snapshot.last_status) << "\"";
  } else {
    std::cout << "null";
  }
  std::cout << "}\n";
}

int parse_duration(int argc, char ** argv)
{
  for (int index = 1; index + 1 < argc; ++index) {
    if (std::string(argv[index]) == "--duration-sec") {
      const auto duration = std::stoi(argv[index + 1]);
      if (duration <= 0 || duration > 3600) {
        throw std::invalid_argument("--duration-sec must be in [1, 3600]");
      }
      return duration;
    }
  }
  return 30;
}

bool parse_expect_timeout(int argc, char ** argv)
{
  for (int index = 1; index < argc; ++index) {
    if (std::string(argv[index]) == "--expect-timeout") {
      return true;
    }
  }
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto duration_sec = parse_duration(argc, argv);
    const auto expect_timeout = parse_expect_timeout(argc, argv);
    rclcpp::init(argc, argv);
    auto ingress = std::make_shared<arm_cell_vision::VisionRosIngress>();
    const auto begin = ingress->begin_acquisition(0);
    std::cout << "begin_acquisition_status=\"" << status_name(begin.status) << "\"\n";
    arm_cell_vision::acceptance::DeadlineStatusLatch deadline_latch;

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(ingress);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(duration_sec);
    auto next_report = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(100ms);
      if (!deadline_latch.latched()) {
        deadline_latch.observe(ingress->check_deadline().status);
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_report) {
        print_snapshot(ingress->acceptance_snapshot());
        next_report = now + 1s;
      }
    }
    const auto snapshot = ingress->acceptance_snapshot();
    print_snapshot(snapshot);
    const auto deadline_status = deadline_latch.status();
    std::cout << "deadline_status=\"" <<
      (deadline_status ? status_name(*deadline_status) : "not_reached") << "\"\n";
    const bool pass = expect_timeout ?
      arm_cell_vision::acceptance::expect_timeout_mode_pass(
      begin.status, deadline_status, snapshot.observation_ready_count,
      snapshot.epoch_invalidated, snapshot.clock_rollback_observed,
      snapshot.observation_rollback_observed) :
      arm_cell_vision::acceptance::normal_mode_pass(
      begin.status, deadline_status, snapshot.observation_ready_count,
      snapshot.epoch_invalidated);
    std::cout << "acceptance_result=\"" << (pass ? "PASS" : "FAIL") << "\"\n";
    rclcpp::shutdown();
    return pass ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "acceptance_result=\"ERROR\" message=\"" << error.what() << "\"\n";
    if (rclcpp::ok()) {rclcpp::shutdown();}
    return 2;
  }
}
