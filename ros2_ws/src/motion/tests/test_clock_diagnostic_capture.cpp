#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "arm_cell_motion/clock_diagnostic_capture.hpp"

namespace arm_cell_motion {
namespace {
std::vector<nlohmann::json> read_events(std::istream & input) {
  std::vector<nlohmann::json> records;
  std::string line;
  while (std::getline(input, line)) {
    records.push_back(nlohmann::json::parse(line));
  }
  return records;
}

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
  const auto contents = read_events(input);
  EXPECT_TRUE(std::any_of(contents.begin(), contents.end(),
      [](const auto & record) {return record.at("event") == "backward_epoch";}));
  EXPECT_TRUE(std::any_of(contents.begin(), contents.end(),
      [](const auto & record) {return record.at("clock_epoch") == 1;}));
  EXPECT_TRUE(std::any_of(contents.begin(), contents.end(),
      [](const auto & record) {return record.at("capture_id") == "test-capture";}));
  EXPECT_TRUE(std::any_of(contents.begin(), contents.end(),
      [](const auto & record) {return record.contains("receipt_steady_ns");}));
  EXPECT_TRUE(std::any_of(contents.begin(), contents.end(),
      [](const auto & record) {return record.contains("node_ros_time_ns");}));
  EXPECT_TRUE(std::any_of(contents.begin(), contents.end(),
      [](const auto & record) {return record.contains("rtf_interval");}));
  std::ifstream manifest(capture_dir / "manifest.json");
  ASSERT_TRUE(manifest.good());
  const auto manifest_contents = nlohmann::json::parse(manifest);
  EXPECT_EQ(manifest_contents.at("receipt_gap_threshold_ns"), 2000000000);
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
  const auto contents = read_events(input);
  EXPECT_TRUE(std::any_of(contents.begin(), contents.end(),
      [](const auto & record) {return record.at("source_age_ns") == 9000000000;}));
  EXPECT_TRUE(std::any_of(contents.begin(), contents.end(),
      [](const auto & record) {return record.at("event") == "initialized";}));
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
  const auto contents = read_events(input);
  EXPECT_TRUE(std::any_of(contents.begin(), contents.end(),
      [](const auto & record) {return record.at("forward_jump_threshold_ns") == 1200000000;}));
  EXPECT_TRUE(std::any_of(contents.begin(), contents.end(),
      [](const auto & record) {return record.at("event") == "suspected_forward_jump";}));
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
  const auto contents = nlohmann::json::parse(manifest);
  EXPECT_EQ(contents.at("writer_failed"), true);
  EXPECT_EQ(contents.at("complete"), false);
  EXPECT_EQ(contents.at("shutdown_status"), "incomplete");
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
  const auto contents = nlohmann::json::parse(manifest);
  EXPECT_GT(contents.at("dropped_count").get<std::uint64_t>(), 0u);
  EXPECT_LE(contents.at("retained_count").get<std::uint64_t>(), 512u);
  EXPECT_EQ(contents.at("complete"), false);
  EXPECT_EQ(contents.at("shutdown_status"), "clean");
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
    sequences.push_back(nlohmann::json::parse(line).at("sequence").get<uint64_t>());
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
  const auto contents = nlohmann::json::parse(manifest);
  EXPECT_EQ(contents.at("window_complete"), true);
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
  const auto records = read_events(input);
  ASSERT_FALSE(records.empty());
  EXPECT_EQ(records.front().at("event"), "clock_progress");
  const auto first_clock_time = records.front().at("clock_time_ns").get<int64_t>();
  EXPECT_GT(first_clock_time, 2'000'000'000LL);
  EXPECT_LT(first_clock_time, 3'000'000'000LL);
  EXPECT_TRUE(std::any_of(records.begin(), records.end(), [](const auto & record) {
      return record.at("event") == "watchdog_decision";
    }));
  EXPECT_FALSE(std::any_of(records.begin(), records.end(), [](const auto & record) {
      return record.at("event") == "initialized";
    }));
  std::ifstream manifest(root / "late-stall" / "test-process" /
                         "manifest.json");
  ASSERT_TRUE(manifest.good());
  const auto manifest_contents = nlohmann::json::parse(manifest);
  EXPECT_EQ(manifest_contents.at("complete"), true);
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
  const auto contents = read_events(input);
  EXPECT_GE(std::count_if(contents.begin(), contents.end(), [](const auto & record) {
      return record.at("event") == "backward_epoch";
    }), 2);
  std::ifstream manifest(root / "rearm-capture" / "test-process" /
                         "manifest.json");
  ASSERT_TRUE(manifest.good());
  const auto manifest_contents = nlohmann::json::parse(manifest);
  EXPECT_EQ(manifest_contents.at("complete"), true);
  std::filesystem::remove_all(root);
}

} // namespace
} // namespace arm_cell_motion

TEST(ClockDiagnosticCapture, IdleCaptureDoesNotCreateArtifacts) {
  const auto root = std::filesystem::temp_directory_path() /
    ("arm-cell-idle-clock-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  {
    arm_cell_motion::ClockDiagnosticCapture capture(root.string(), "idle-process");
  }
  EXPECT_FALSE(std::filesystem::exists(root));
}
