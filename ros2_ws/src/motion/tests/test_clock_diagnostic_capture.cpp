#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "arm_cell_motion/clock_diagnostic_capture.hpp"

namespace arm_cell_motion {
namespace {
ClockDiagnosticCapture::Sample sample(double clock_s, double steady_s) {
  ClockDiagnosticCapture::Sample value;
  value.clock_time_ns = static_cast<int64_t>(clock_s * 1e9);
  value.node_ros_time_ns = value.clock_time_ns;
  value.node_ros_time_available = true;
  value.receipt_steady_ns = static_cast<int64_t>(steady_s * 1e9);
  value.observer = "motion_node";
  return value;
}

TEST(ClockDiagnosticCapture,
     DistinguishesInitializationRepeatedGapRollbackAndResume) {
  const auto root =
      std::filesystem::temp_directory_path() / "clock-capture-state-test";
  std::filesystem::remove_all(root);
  {
    ClockDiagnosticCapture capture(root, "test-process",
                                   std::chrono::seconds(2), "test-capture");
    EXPECT_EQ(capture.observe(sample(0.0, 0.0)), "initialized");
    EXPECT_EQ(capture.observe(sample(0.0, 0.1)), "repeated_receipt");
    EXPECT_EQ(capture.observe(sample(0.1, 0.2)), "clock_resumed");
    EXPECT_EQ(capture.observe(sample(0.1, 2.3)), "receipt_gap");
    EXPECT_EQ(capture.observe(sample(0.2, 2.4)), "clock_resumed");
    EXPECT_EQ(capture.observe(sample(0.15, 2.5)), "backward_epoch");
    EXPECT_EQ(capture.epoch(), 1u);
  }
  const auto capture_dir = root / "test-capture" / "test-process";
  const auto jsonl = capture_dir / "events.jsonl";
  ASSERT_TRUE(std::filesystem::exists(jsonl));
  std::ifstream input(jsonl);
  const std::string contents((std::istreambuf_iterator<char>(input)), {});
  EXPECT_NE(contents.find("\"event\":\"backward_epoch\""), std::string::npos);
  EXPECT_NE(contents.find("\"clock_epoch\":1"), std::string::npos);
  EXPECT_NE(contents.find("\"capture_id\":\"test-capture\""),
            std::string::npos);
  EXPECT_NE(contents.find("\"receipt_steady_ns\":"), std::string::npos);
  EXPECT_NE(contents.find("\"node_ros_time_ns\":"), std::string::npos);
  EXPECT_NE(contents.find("\"rtf_interval\":"), std::string::npos);
  std::ifstream manifest(capture_dir / "manifest.json");
  ASSERT_TRUE(manifest.good());
  const std::string manifest_contents(
      (std::istreambuf_iterator<char>(manifest)), {});
  EXPECT_NE(manifest_contents.find("\"receipt_gap_threshold_ns\":2000000000"),
            std::string::npos);
  std::filesystem::remove_all(root);
}

TEST(ClockDiagnosticCapture, SourceStampAgeDoesNotMasqueradeAsReceiptDelay) {
  const auto root =
      std::filesystem::temp_directory_path() / "clock-capture-source-age-test";
  std::filesystem::remove_all(root);
  {
    ClockDiagnosticCapture capture(root, "test-process",
                                   std::chrono::seconds(2), "age-capture");
    auto first = sample(1.0, 1.0);
    first.node_ros_time_ns = 10'000'000'000LL;
    first.node_ros_time_sim_active = true;
    first.source_age_ns = 9'000'000'000LL;
    first.source_age_available = true;
    EXPECT_EQ(capture.observe(first), "initialized");
    EXPECT_EQ(capture.observe(sample(1.1, 1.1)), "clock_progress");
    EXPECT_EQ(capture.observe(sample(0.9, 1.2)), "backward_epoch");
  }
  std::ifstream input(root / "age-capture" / "test-process" / "events.jsonl");
  ASSERT_TRUE(input.good());
  const std::string contents((std::istreambuf_iterator<char>(input)), {});
  EXPECT_NE(contents.find("\"source_age_ns\":9000000000"), std::string::npos);
  EXPECT_NE(contents.find("\"event\":\"initialized\""), std::string::npos);
  std::filesystem::remove_all(root);
}

TEST(ClockDiagnosticCapture, ForwardJumpIsReportedWithObservedThreshold) {
  const auto root =
      std::filesystem::temp_directory_path() / "clock-capture-jump-test";
  std::filesystem::remove_all(root);
  {
    ClockDiagnosticCapture capture(root, "test-process",
                                   std::chrono::seconds(2), "jump-capture");
    EXPECT_EQ(capture.observe(sample(1.0, 1.0)), "initialized");
    EXPECT_EQ(capture.observe(sample(100.0, 1.1)), "suspected_forward_jump");
    EXPECT_EQ(capture.epoch(), 0u);
  }
  std::ifstream input(root / "jump-capture" / "test-process" / "events.jsonl");
  ASSERT_TRUE(input.good());
  const std::string contents((std::istreambuf_iterator<char>(input)), {});
  EXPECT_NE(contents.find("\"forward_jump_threshold_ns\":1200000000"),
            std::string::npos);
  EXPECT_NE(contents.find("\"event\":\"suspected_forward_jump\""),
            std::string::npos);
  std::filesystem::remove_all(root);
}

TEST(ClockDiagnosticCapture, WriterFailureDoesNotThrow) {
  ClockDiagnosticCapture capture("/proc/clock-capture-is-not-writable",
                                 "test-process");
  EXPECT_EQ(capture.observe(sample(0.0, 0.0)), "initialized");
  EXPECT_NO_THROW({
    for (int i = 0; i < 10000; ++i) {
      capture.observe(
          sample(static_cast<double>(i) * 0.01, static_cast<double>(i) * 0.01));
    }
  });
}

TEST(ClockDiagnosticCapture, FlushFailureMarksCaptureIncomplete) {
  if (!std::filesystem::exists("/dev/full")) {
    GTEST_SKIP() << "/dev/full is unavailable on this platform";
  }
  const auto root = std::filesystem::temp_directory_path() /
                    "clock-capture-flush-failure-test";
  const auto directory = root / "failed-capture" / "test-process";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(directory);
  std::filesystem::create_symlink("/dev/full", directory / "events.jsonl");
  {
    ClockDiagnosticCapture capture(root, "test-process",
                                   std::chrono::seconds(2), "failed-capture");
    EXPECT_NO_THROW(capture.observe(sample(1.0, 1.0)));
    EXPECT_NO_THROW(capture.observe(sample(0.9, 1.1)));
  }
  std::ifstream manifest(directory / "manifest.json");
  ASSERT_TRUE(manifest.good());
  const std::string contents((std::istreambuf_iterator<char>(manifest)), {});
  EXPECT_NE(contents.find("\"writer_failed\":true"), std::string::npos);
  EXPECT_NE(contents.find("\"complete\":false"), std::string::npos);
  EXPECT_NE(contents.find("\"shutdown_status\":\"incomplete\""),
            std::string::npos);
  std::filesystem::remove_all(root);
}

TEST(ClockDiagnosticCapture,
     HighRateAnomaliesStayBoundedWithoutBlockingProducer) {
  const auto root =
      std::filesystem::temp_directory_path() / "clock-capture-overflow-test";
  std::filesystem::remove_all(root);
  {
    ClockDiagnosticCapture capture(root, "test-process",
                                   std::chrono::seconds(2), "overflow-capture");
    for (int i = 0; i < 4096; ++i) {
      capture.observe(sample(1000.0 - i, static_cast<double>(i) * 0.001));
    }
  }
  std::ifstream manifest(root / "overflow-capture" / "test-process" /
                         "manifest.json");
  ASSERT_TRUE(manifest.good());
  const std::string contents((std::istreambuf_iterator<char>(manifest)), {});
  const auto count_pos = contents.find("\"dropped_count\":");
  ASSERT_NE(count_pos, std::string::npos);
  const auto dropped_value = contents.find(':', count_pos) + 1;
  EXPECT_GT(std::stoull(contents.substr(dropped_value)), 0u);
  const auto retained_pos = contents.find("\"retained_count\":");
  ASSERT_NE(retained_pos, std::string::npos);
  const auto retained_value = contents.find(':', retained_pos) + 1;
  EXPECT_LE(std::stoull(contents.substr(retained_value)), 512u);
  EXPECT_NE(contents.find("\"complete\":false"), std::string::npos);
  EXPECT_NE(contents.find("\"shutdown_status\":\"clean\""), std::string::npos);
  std::filesystem::remove_all(root);
}

TEST(ClockDiagnosticCapture, RepeatedRollbackDoesNotDuplicateSequenceRecords) {
  const auto root = std::filesystem::temp_directory_path() /
                    "clock-capture-rollback-duplicate-test";
  std::filesystem::remove_all(root);
  {
    ClockDiagnosticCapture capture(root, "test-process",
                                   std::chrono::seconds(2), "rollback-capture");
    EXPECT_EQ(capture.observe(sample(10.0, 1.0)), "initialized");
    for (int i = 0; i < 12; ++i) {
      EXPECT_EQ(capture.observe(sample(9.0 - i, 1.1 + i * 0.1)),
                "backward_epoch");
    }
  }
  std::ifstream input(root / "rollback-capture" / "test-process" /
                      "events.jsonl");
  ASSERT_TRUE(input.good());
  std::string line;
  std::vector<uint64_t> sequences;
  while (std::getline(input, line)) {
    const auto pos = line.find("\"sequence\":");
    ASSERT_NE(pos, std::string::npos);
    sequences.push_back(std::stoull(line.substr(pos + 11)));
  }
  EXPECT_EQ(sequences.size(), 13u);
  EXPECT_TRUE(std::is_sorted(sequences.begin(), sequences.end()));
  EXPECT_EQ(std::adjacent_find(sequences.begin(), sequences.end()),
            sequences.end());
  std::filesystem::remove_all(root);
}

TEST(ClockDiagnosticCapture, RepeatedStallDecisionsConsumeFinitePostWindow) {
  const auto root = std::filesystem::temp_directory_path() /
                    "clock-capture-stall-window-test";
  std::filesystem::remove_all(root);
  {
    ClockDiagnosticCapture capture(root, "test-process",
                                   std::chrono::seconds(2), "stall-capture");
    EXPECT_EQ(capture.observe(sample(1.0, 1.0)), "initialized");
    auto decision = sample(1.0, 1.1);
    decision.watchdog_result = "SIMULATION_CLOCK_STALL";
    for (int i = 0; i < 100; ++i) {
      decision.decision_steady_ns = 1'100'000'000LL + i;
      capture.record_decision(decision);
    }
  }
  std::ifstream input(root / "stall-capture" / "test-process" / "events.jsonl");
  ASSERT_TRUE(input.good());
  std::size_t records = 0;
  std::string line;
  while (std::getline(input, line))
    ++records;
  EXPECT_LE(records, 34u);
  std::ifstream manifest(root / "stall-capture" / "test-process" /
                         "manifest.json");
  ASSERT_TRUE(manifest.good());
  const std::string contents((std::istreambuf_iterator<char>(manifest)), {});
  EXPECT_NE(contents.find("\"window_complete\":true"), std::string::npos);
}

TEST(ClockDiagnosticCapture,
     NormalClockInitializationKeepsRingUntilLaterWatchdogStall) {
  const auto root =
      std::filesystem::temp_directory_path() / "clock-capture-late-stall-test";
  std::filesystem::remove_all(root);
  {
    ClockDiagnosticCapture capture(root, "test-process",
                                   std::chrono::seconds(2), "late-stall");
    for (int i = 0; i < 80; ++i) {
      EXPECT_EQ(capture.observe(sample(1.0 + i * 0.1, 1.0 + i * 0.1)),
                i == 0 ? "initialized" : "clock_progress");
    }
    auto stall = sample(8.9, 9.1);
    stall.watchdog_result = "SIMULATION_CLOCK_STALL";
    capture.record_decision(stall);
    for (int i = 0; i < 32; ++i) {
      capture.observe(sample(9.0 + i * 0.1, 9.2 + i * 0.1));
    }
  }
  std::ifstream input(root / "late-stall" / "test-process" / "events.jsonl");
  ASSERT_TRUE(input.good());
  std::string first_record;
  ASSERT_TRUE(static_cast<bool>(std::getline(input, first_record)));
  EXPECT_NE(first_record.find("\"event\":\"clock_progress\""),
            std::string::npos);
  const auto clock_time_pos = first_record.find("\"clock_time_ns\":");
  ASSERT_NE(clock_time_pos, std::string::npos);
  const auto clock_time_value = first_record.find(':', clock_time_pos) + 1;
  const auto first_clock_time =
      std::stoll(first_record.substr(clock_time_value));
  EXPECT_GT(first_clock_time, 2'000'000'000LL);
  EXPECT_LT(first_clock_time, 3'000'000'000LL);
  std::string contents(first_record);
  contents.append(std::istreambuf_iterator<char>(input), {});
  EXPECT_NE(contents.find("\"event\":\"watchdog_decision\""),
            std::string::npos);
  EXPECT_EQ(contents.find("\"event\":\"initialized\""), std::string::npos);
  std::ifstream manifest(root / "late-stall" / "test-process" /
                         "manifest.json");
  ASSERT_TRUE(manifest.good());
  const std::string manifest_contents(
      (std::istreambuf_iterator<char>(manifest)), {});
  EXPECT_NE(manifest_contents.find("\"complete\":true"), std::string::npos);
  std::filesystem::remove_all(root);
}

TEST(ClockDiagnosticCapture, RearmsForSecondAnomalyAfterFirstWindowCompletes) {
  const auto root =
      std::filesystem::temp_directory_path() / "clock-capture-rearm-test";
  std::filesystem::remove_all(root);
  {
    ClockDiagnosticCapture capture(root, "test-process",
                                   std::chrono::seconds(2), "rearm-capture");
    for (int i = 0; i < 10; ++i) {
      capture.observe(sample(10.0 + i * 0.1, 1.0 + i * 0.1));
    }
    EXPECT_EQ(capture.observe(sample(9.0, 2.1)), "backward_epoch");
    for (int i = 0; i < 32; ++i) {
      capture.observe(sample(9.1 + i * 0.1, 2.2 + i * 0.1));
    }
    EXPECT_EQ(capture.observe(sample(5.0, 5.5)), "backward_epoch");
    for (int i = 0; i < 32; ++i) {
      capture.observe(sample(5.1 + i * 0.1, 5.6 + i * 0.1));
    }
  }
  std::ifstream input(root / "rearm-capture" / "test-process" / "events.jsonl");
  ASSERT_TRUE(input.good());
  const std::string contents((std::istreambuf_iterator<char>(input)), {});
  const auto first = contents.find("\"event\":\"backward_epoch\"");
  ASSERT_NE(first, std::string::npos);
  EXPECT_NE(contents.find("\"event\":\"backward_epoch\"", first + 1),
            std::string::npos);
  std::ifstream manifest(root / "rearm-capture" / "test-process" /
                         "manifest.json");
  ASSERT_TRUE(manifest.good());
  const std::string manifest_contents(
      (std::istreambuf_iterator<char>(manifest)), {});
  EXPECT_NE(manifest_contents.find("\"complete\":true"), std::string::npos);
  std::filesystem::remove_all(root);
}

} // namespace
} // namespace arm_cell_motion
