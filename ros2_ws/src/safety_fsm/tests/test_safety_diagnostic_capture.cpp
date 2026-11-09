#include <gtest/gtest.h>

#define main arm_cell_safety_node_main
#include "../src/safety_node.cpp"
#undef main

TEST(SafetyDiagnosticCaptureStorage, IdleWriterDoesNotCreateDirectory)
{
  const auto root = std::filesystem::temp_directory_path() /
    ("arm-cell-idle-safety-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  {
    arm_cell_safety_fsm::SafetyDiagnosticCapture capture(
      root.string(), rclcpp::get_logger("safety_capture_test"));
  }
  EXPECT_FALSE(std::filesystem::exists(root));
}

TEST(SafetyDiagnosticCaptureStorage, FirstEventIsWrittenOnShutdown)
{
  const auto root = std::filesystem::temp_directory_path() /
    ("arm-cell-event-safety-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  {
    arm_cell_safety_fsm::SafetyDiagnosticCapture capture(
      root.string(), rclcpp::get_logger("safety_capture_test"));
    EXPECT_TRUE(capture.enqueue(arm_cell_safety_fsm::SafetyDiagnosticRecord{}));
  }
  ASSERT_TRUE(std::filesystem::is_directory(root));
  std::size_t count = 0;
  for (const auto & entry : std::filesystem::directory_iterator(root)) {
    std::ifstream input(entry.path());
    std::string line;
    ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
    EXPECT_NE(line.find("schema_version"), std::string::npos);
    ASSERT_TRUE(static_cast<bool>(std::getline(input, line)));
    EXPECT_NE(line.find("capture_summary"), std::string::npos);
    ++count;
  }
  EXPECT_EQ(count, 1U);
  std::filesystem::remove_all(root);
}
