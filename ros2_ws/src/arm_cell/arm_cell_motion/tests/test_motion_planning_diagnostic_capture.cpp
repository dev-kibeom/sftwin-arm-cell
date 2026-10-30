#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <arm_cell_interfaces/msg/motion_task_result_code.hpp>

#include <opencv2/imgcodecs.hpp>

#include "arm_cell_motion/motion_planning_diagnostic_capture.hpp"
#include "arm_cell_motion/occupancy_snapshot_renderer.hpp"

namespace arm_cell_motion
{

TEST(MotionPlanningDiagnosticCaptureTest, SnapshotCanBeSavedWithoutDetailedDiagnosticRecords)
{
  const auto root = std::filesystem::temp_directory_path() /
    ("motion_snapshot_without_diagnostics_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto snapshots = root / "snapshots";
  std::filesystem::remove_all(root);
  const auto logger = rclcpp::get_logger("motion_snapshot_without_diagnostics_test");
  std::string first_path;
  {
    MotionPlanningDiagnosticCapture capture((root / "diagnostic_records").string(), logger,
      snapshots.string());
    EXPECT_TRUE(capture.enqueue_occupancy_snapshot(
      "goal", "acq-1", "candidate-a", {}, first_path));
    std::string duplicate_path;
    EXPECT_TRUE(capture.enqueue_occupancy_snapshot(
      "goal", "acq-1", "candidate-a", {}, duplicate_path));
    EXPECT_EQ(first_path, duplicate_path);
    std::string candidate_path;
    EXPECT_TRUE(capture.enqueue_occupancy_snapshot(
      "goal", "acq-1", "candidate-b", {}, candidate_path));
    std::string other_acquisition_path;
    EXPECT_TRUE(capture.enqueue_occupancy_snapshot(
      "goal", "acq-2", "candidate-a", {}, other_acquisition_path));
    EXPECT_NE(first_path, candidate_path);
    EXPECT_NE(first_path, other_acquisition_path);
    EXPECT_NE(first_path.find(
        "snapshots/acquisition/acq-1/occupancy_goal_candidate_candidate-a_attempt_1.png"),
      std::string::npos);
  }
  ASSERT_TRUE(std::filesystem::exists(snapshots));
  std::size_t png_count = 0U;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(snapshots)) {
    if (entry.path().extension() == ".png") {
      ++png_count;
      EXPECT_FALSE(cv::imread(entry.path().string(), cv::IMREAD_COLOR).empty());
    }
  }
  EXPECT_EQ(png_count, 3U);
  EXPECT_TRUE(std::filesystem::exists(
      snapshots / "acquisition" / "acq-1" / "occupancy_goal_candidate_candidate-a_attempt_1.png"));
  EXPECT_TRUE(std::filesystem::exists(
      snapshots / "acquisition" / "acq-1" / "occupancy_goal_candidate_candidate-b_attempt_2.png"));
  EXPECT_TRUE(std::filesystem::exists(
      snapshots / "acquisition" / "acq-2" / "occupancy_goal_candidate_candidate-a_attempt_3.png"));
  EXPECT_FALSE(std::filesystem::exists(root / "diagnostic_records"));
  std::filesystem::remove_all(root);
}

TEST(MotionPlanningDiagnosticCaptureTest, OccupancySnapshotReferenceResolvesInDiagnosticRecord)
{
  const auto root = std::filesystem::temp_directory_path() /
    ("motion_snapshot_reference_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::remove_all(root);
  std::string image_reference;
  {
    MotionPlanningDiagnosticCapture capture((root / "diagnostic_records").string(),
      rclcpp::get_logger("motion_snapshot_reference_test"), (root / "snapshots").string());
    ASSERT_TRUE(capture.enqueue_occupancy_snapshot(
      "goal", "acq-1", "candidate-a", {}, image_reference));
    ASSERT_TRUE(capture.enqueue("goal", "TASK_TYPE_PICK", 0U,
      nlohmann::json{{"occupancy_snapshot", {{"path", image_reference}}}}.dump(), "acq-1"));
  }
  std::ifstream record(root / "diagnostic_records" / "goal.json");
  ASSERT_TRUE(record.good());
  const auto saved_record = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(record)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(saved_record.at("planning_evidence").at("occupancy_snapshot").at("path"),
    image_reference);
  EXPECT_TRUE(std::filesystem::is_regular_file(root / image_reference));
  std::filesystem::remove_all(root);
}

TEST(MotionPlanningDiagnosticCaptureTest, TerminalSummaryPreservesActualFailure)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_terminal_summary_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  using Result = arm_cell_interfaces::msg::MotionTaskResultCode;
  const nlohmann::json evidence = {
    {"final_failure_stage", "GLOBAL_PLANNING"},
    {"final_failure_reason", "APPROACH_ROUTE_NOT_FOUND"},
    {"vision_acquisition_id", "acquisition-terminal"},
    {"material_candidate_attempts", nlohmann::json::array({
      {{"attempt_order", 1}, {"candidate_id", "candidate-a"},
        {"result", "TASK_RESULT_PATH_OBSTRUCTED"}, {"failure_stage", "approach"}},
      {{"attempt_order", 2}, {"candidate_id", "candidate-b"},
        {"result", "TASK_RESULT_PATH_OBSTRUCTED"}, {"failure_stage", "grasp_motion"}},
      {{"attempt_order", 3}, {"candidate_id", "candidate-c"},
        {"result", "TASK_RESULT_PATH_OBSTRUCTED"}, {"failure_stage", "approach"}}})},
    {"outcome_material_candidate_id", "candidate-c"}};
  const auto logger = rclcpp::get_logger("motion_pick_terminal_summary_test");
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    ASSERT_TRUE(capture.enqueue_pick_summary(
      "terminal-pick", "acquisition-terminal", Result::TASK_RESULT_PATH_OBSTRUCTED,
      false, evidence.dump(), 0U));
  }
  std::ifstream input(directory / "motion_planning_summaries" / "terminal-pick.json");
  ASSERT_TRUE(input.good());
  const auto summary = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(summary.at("final_failure_reason"), evidence.at("final_failure_reason"));
  EXPECT_EQ(summary.at("task_executor_result_code_value"), Result::TASK_RESULT_PATH_OBSTRUCTED);
  EXPECT_FALSE(summary.at("pick_grasp_insertion_started").get<bool>());
  const auto & material = summary.at("material_candidate_search");
  ASSERT_EQ(material.at("attempts").size(), 3U);
  EXPECT_EQ(material.at("attempts").at(0).at("candidate_id"), "candidate-a");
  EXPECT_EQ(material.at("attempts").at(1).at("failure_stage"), "grasp_motion");
  EXPECT_EQ(material.at("attempts").at(2).at("result"), "TASK_RESULT_PATH_OBSTRUCTED");
  EXPECT_TRUE(material.at("candidate_switch_occurred").get<bool>());
  EXPECT_EQ(material.at("final_target_candidate_id"), "candidate-c");
  EXPECT_EQ(material.at("acquisition_id"), "acquisition-terminal");
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, SuccessfulCandidateMatchesFinalActionResult)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_terminal_success_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  using Result = arm_cell_interfaces::msg::MotionTaskResultCode;
  const nlohmann::json evidence = {
    {"vision_acquisition_id", "acquisition-success"},
    {"material_candidate_attempts", nlohmann::json::array({
      {{"attempt_order", 1}, {"candidate_id", "candidate-a"},
        {"result", "TASK_RESULT_PATH_OBSTRUCTED"}, {"failure_stage", "approach"}},
      {{"attempt_order", 2}, {"candidate_id", "candidate-b"},
        {"result", "TASK_RESULT_PATH_OBSTRUCTED"}, {"failure_stage", "grasp_motion"}},
      {{"attempt_order", 3}, {"candidate_id", "candidate-c"},
        {"result", "TASK_RESULT_SUCCESS"}, {"failure_stage", ""}}})},
    {"outcome_material_candidate_id", "candidate-c"}};
  const auto logger = rclcpp::get_logger("motion_pick_terminal_success_test");
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    ASSERT_TRUE(capture.enqueue_pick_summary(
      "successful-pick", "acquisition-success", Result::TASK_RESULT_SUCCESS,
      true, evidence.dump(), 0U));
  }
  std::ifstream input(
    directory / "motion_planning_summaries" / "successful-pick.json", std::ios::binary);
  ASSERT_TRUE(input.good());
  const auto summary = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(summary.at("task_executor_result_code"), "TASK_RESULT_SUCCESS");
  EXPECT_EQ(summary.at("task_executor_result_code_value"), Result::TASK_RESULT_SUCCESS);
  const auto & material = summary.at("material_candidate_search");
  ASSERT_EQ(material.at("attempts").size(), 3U);
  EXPECT_EQ(material.at("attempts").at(0).at("result"), "TASK_RESULT_PATH_OBSTRUCTED");
  EXPECT_EQ(material.at("attempts").at(1).at("result"), "TASK_RESULT_PATH_OBSTRUCTED");
  EXPECT_EQ(material.at("attempts").at(2).at("result"), "TASK_RESULT_SUCCESS");
  EXPECT_EQ(material.at("final_target_candidate_id"), "candidate-c");
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, OversizedMaterialCandidateHistoryStaysWithinSummaryLimit)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_material_summary_limit_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  using Result = arm_cell_interfaces::msg::MotionTaskResultCode;
  nlohmann::json attempts = nlohmann::json::array();
  for (std::size_t index = 0; index < 700U; ++index) {
    attempts.push_back({
      {"attempt_order", index + 1U},
      {"candidate_id", "material-candidate-" + std::to_string(index) + std::string(96U, 'x')},
      {"result", "TASK_RESULT_PATH_OBSTRUCTED"},
      {"failure_stage", "approach"},
      {"pick_grasp_insertion_started", false},
      {"redundant_detail", std::string(256U, 'd')}});
  }
  const nlohmann::json evidence = {
    {"vision_acquisition_id", "acquisition-oversized"},
    {"material_candidate_attempts", attempts},
    {"outcome_material_candidate_id", attempts.back().at("candidate_id")},
    {"final_failure_stage", "GLOBAL_PLANNING"},
    {"final_failure_reason", "APPROACH_ROUTE_NOT_FOUND"}};
  const auto logger = rclcpp::get_logger("motion_pick_material_summary_limit_test");
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    ASSERT_TRUE(capture.enqueue_pick_summary(
      "oversized-material-pick", "acquisition-oversized",
      Result::TASK_RESULT_PATH_OBSTRUCTED, false, evidence.dump(), 0U));
  }
  const auto path = directory / "motion_planning_summaries" / "oversized-material-pick.json";
  ASSERT_TRUE(std::filesystem::is_regular_file(path));
  EXPECT_LE(std::filesystem::file_size(path), 64U * 1024U);
  std::ifstream input(path, std::ios::binary);
  ASSERT_TRUE(input.good());
  const auto summary = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(summary.at("task_executor_result_code"), "TASK_RESULT_PATH_OBSTRUCTED");
  EXPECT_EQ(summary.at("final_failure_reason"), "APPROACH_ROUTE_NOT_FOUND");
  const auto & material = summary.at("material_candidate_search");
  EXPECT_EQ(material.at("final_target_candidate_id"), attempts.back().at("candidate_id"));
  ASSERT_TRUE(material.contains("omitted_attempts"));
  EXPECT_GT(material.at("omitted_attempts").at("count").get<std::size_t>(), 0U);
  EXPECT_EQ(material.at("omitted_attempts").at("reason"), "summary_byte_limit");
  EXPECT_EQ(material.at("omitted_attempts").at("failure_counts_by_result_and_stage")
    .at("TASK_RESULT_PATH_OBSTRUCTED:approach"),
    material.at("omitted_attempts").at("count"));
  ASSERT_FALSE(material.at("attempts").empty());
  EXPECT_EQ(material.at("attempts").front().at("result"), "TASK_RESULT_PATH_OBSTRUCTED");
  EXPECT_EQ(material.at("attempts").front().at("failure_stage"), "approach");
  EXPECT_EQ(material.at("attempts").back().at("candidate_id"), attempts.back().at("candidate_id"));
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, PersistsBoundedEvidenceAndTerminalResult)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_planning_diagnostic_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  const std::string uuid = "test-pick-goal";
  const std::string evidence =
    "{\"stage\":\"planning\",\"motion_goal_uuid\":\"test-pick-goal\","
    "\"best_candidate_so_far\":null,"
    "\"moveit_num_planning_attempts\":3,\"ik_solve_type\":\"Distance\","
    "\"candidates\":[{\"candidate_index\":2,\"nominal_pose_deviation\":0.15,"
    "\"tool_orientation_preference_error_rad\":0.05,\"raw_delta\":[1.2],"
    "\"wrap_aware_delta\":[-0.3],\"state_valid\":false,"
    "\"contact_bodies\":[],\"rejection_reason\":\"state_validity_failed\"}],"
    "\"attempts\":[{\"candidate_index\":1,\"planner_result_code\":-1,"
    "\"target_setter_attempted\":true,\"target_setter_accepted\":false,"
    "\"planner_invoked\":false,\"target_group_validation\":{"
    "\"group_name\":\"arm\",\"group_variable_names\":[\"joint_3\"],"
    "\"group_contract_matches\":true,\"goal_size_matches_group\":true,"
    "\"group_bounds_satisfied\":false,"
    "\"variables\":[{\"name\":\"joint_3\",\"value\":2.7,"
    "\"lower_bound\":-2.618,\"upper_bound\":2.618,\"within_bounds\":false}]},"
    "\"accepted\":false}],\"cartesian_descent\":{\"motion_goal_uuid\":"
    "\"test-pick-goal\",\"stage\":\"cartesian_descent\","
    "\"path_fraction\":0.5,\"sample_count\":11,\"validated_sample_count\":6,"
    "\"failure_reason\":\"collision/contact\",\"failure_sample\":{"
    "\"available\":true,\"index\":5,\"validation_reason\":\"collision/contact\","
    "\"contact_bodies\":{\"available\":true,\"pairs\":["
    "[\"arm_link_4\",\"rawpart_body\"]]}}}}";
  const auto logger = rclcpp::get_logger("motion_planning_diagnostic_test");
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    ASSERT_TRUE(
      capture.enqueue_progress(
      uuid, "PICK",
        "{\"active_candidate\":2,\"best_candidate_so_far\":1,"
        "\"candidate_stage\":\"before_plan\"}", "vision-acq-pick-1"));
    ASSERT_TRUE(capture.enqueue(uuid, "PICK", 9, evidence, "vision-acq-pick-1"));
    ASSERT_TRUE(capture.enqueue("test-next-goal", "PLACE", 0, "{}"));
  }

  const auto path = directory / (uuid + ".json");
  ASSERT_TRUE(std::filesystem::exists(path));
  std::ifstream input(path, std::ios::binary);
  const std::string stored(
    (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  const auto parsed = nlohmann::json::parse(stored);
  ASSERT_TRUE(parsed.is_object());
  ASSERT_TRUE(parsed.at("planning_evidence").is_object());
  EXPECT_EQ(parsed.at("planning_evidence").at("moveit_num_planning_attempts"), 3);
  EXPECT_EQ(parsed.at("planning_evidence").at("ik_solve_type"), "Distance");
  EXPECT_EQ(parsed.at("planning_evidence").at("motion_goal_uuid"), "test-pick-goal");
  EXPECT_EQ(parsed.at("vision_acquisition_id"), "vision-acq-pick-1");
  EXPECT_EQ(parsed.at("planning_evidence").at("cartesian_descent").at("stage"),
    "cartesian_descent");
  EXPECT_EQ(parsed.at("planning_evidence").at("cartesian_descent").at("path_fraction"), 0.5);
  EXPECT_EQ(
    parsed.at("planning_evidence").at("cartesian_descent").at("validated_sample_count"), 6);
  EXPECT_EQ(
    parsed.at("planning_evidence").at("cartesian_descent").at("failure_sample").at("index"), 5);
  EXPECT_EQ(
    parsed.at("planning_evidence").at("cartesian_descent").at("failure_sample").at(
      "contact_bodies").at("pairs").at(0).at(0), "arm_link_4");
  EXPECT_NE(stored.find("\"motion_goal_uuid\":\"test-pick-goal\""), std::string::npos);
  EXPECT_NE(stored.find("\"task_type\":\"PICK\""), std::string::npos);
  EXPECT_NE(stored.find("\"record_status\":\"terminal\""), std::string::npos);
  EXPECT_NE(
    stored.find("\"task_executor_result_code\":\"TASK_RESULT_BACKEND_ERROR\""),
    std::string::npos);
  EXPECT_NE(stored.find("\"contact_bodies\":[]"), std::string::npos);
  EXPECT_NE(stored.find("\"rejection_reason\":\"state_validity_failed\""), std::string::npos);
  EXPECT_NE(stored.find("\"tool_orientation_preference_error_rad\":0.05"), std::string::npos);
  EXPECT_NE(stored.find("\"wrap_aware_delta\":[-0.3]"), std::string::npos);
  EXPECT_NE(stored.find("\"best_candidate_so_far\":null"), std::string::npos);
  EXPECT_NE(stored.find("\"planner_result_code\":-1"), std::string::npos);
  EXPECT_NE(stored.find("\"target_setter_attempted\":true"), std::string::npos);
  EXPECT_NE(stored.find("\"target_setter_accepted\":false"), std::string::npos);
  EXPECT_NE(stored.find("\"planner_invoked\":false"), std::string::npos);
  EXPECT_NE(stored.find("\"group_variable_names\":[\"joint_3\"]"), std::string::npos);
  EXPECT_NE(stored.find("\"group_contract_matches\":true"), std::string::npos);
  EXPECT_NE(stored.find("\"group_bounds_satisfied\":false"), std::string::npos);
  EXPECT_NE(stored.find("\"name\":\"joint_3\",\"value\":2.7"), std::string::npos);
  EXPECT_EQ(stored.find("\"collision\":true"), std::string::npos);
  const auto next_path = directory / "test-next-goal.json";
  ASSERT_TRUE(std::filesystem::exists(next_path));
  std::ifstream next_input(next_path, std::ios::binary);
  const auto next = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(next_input)), std::istreambuf_iterator<char>()));
  EXPECT_TRUE(next.at("vision_acquisition_id").is_null());
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, TerminalSnapshotKeepsDeferredRevisitHistory)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("pick_revisit_history_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  nlohmann::json evidence = {
    {"material_candidate_id", "material-a"},
    {"candidate_search_termination_reason", "SEARCHING"},
    {"pick_attempts", nlohmann::json::array({
      {{"attempt", 1}, {"grasp_pose_candidate", 1}, {"ik_seed_attempt", 1}},
      {{"attempt", 2}, {"grasp_pose_candidate", 2}, {"ik_seed_attempt", 1}}})}};
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), rclcpp::get_logger("pick_history"));
    ASSERT_TRUE(capture.enqueue_progress("pick-history", "TASK_TYPE_PICK", evidence.dump(), "acq"));
    evidence["pick_attempts"].push_back(
      {{"attempt", 3}, {"grasp_pose_candidate", 1}, {"ik_seed_attempt", 2}});
    evidence["candidate_search_termination_reason"] = "SEARCH_BUDGET_EXHAUSTED";
    ASSERT_TRUE(capture.enqueue_progress("pick-history", "TASK_TYPE_PICK", evidence.dump(), "acq"));
    ASSERT_TRUE(capture.enqueue("pick-history", "TASK_TYPE_PICK",
      arm_cell_interfaces::msg::MotionTaskResultCode::TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED,
      evidence.dump(), "acq"));
  }
  std::ifstream input(directory / "pick-history.json");
  ASSERT_TRUE(input.good());
  const auto record = nlohmann::json::parse(input);
  EXPECT_EQ(record.at("record_status"), "terminal");
  EXPECT_EQ(record.at("motion_goal_uuid"), "pick-history");
  EXPECT_EQ(record.at("vision_acquisition_id"), "acq");
  EXPECT_EQ(record.at("planning_evidence"), evidence);
  EXPECT_EQ(record.at("task_executor_result_code"), "TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED");
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, KeepsMaterialCandidatesSeparateFromGraspPoseCandidates)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_material_candidate_details_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  const nlohmann::json evidence = {
    {"candidate_attempt_history", nlohmann::json::array({
      {{"material_candidate_id", "material-a"}, {"grasp_pose_candidates", nlohmann::json::array({
        {{"candidate", 1}, {"result", "IK_FAILED"}}})},
        {"pick_attempts", nlohmann::json::array({
          {{"grasp_pose_candidate", 1}, {"attempt", 1}, {"reason", "IK_FAILED"}}})}},
      {{"material_candidate_id", "material-b"}, {"grasp_pose_candidates", nlohmann::json::array({
        {{"candidate", 1}, {"result", "SUCCESS"}}})},
        {"pick_attempts", nlohmann::json::array({
          {{"grasp_pose_candidate", 1}, {"attempt", 1}, {"reason", "SUCCESS"}}})}}})}};
  const auto logger = rclcpp::get_logger("motion_pick_material_candidate_details_test");
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    EXPECT_EQ(capture.enqueue_pick_details("material-pick", "acquisition-material",
      evidence.dump()), 2U);
  }
  const auto first_path = directory / "motion_planning_details" / "material-pick_candidate_001.json";
  const auto second_path = directory / "motion_planning_details" / "material-pick_candidate_002.json";
  ASSERT_TRUE(std::filesystem::exists(first_path));
  ASSERT_TRUE(std::filesystem::exists(second_path));
  std::ifstream first_input(first_path);
  std::ifstream second_input(second_path);
  const auto first = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(first_input)), std::istreambuf_iterator<char>()));
  const auto second = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(second_input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(first.at("material_candidate_id"), "material-a");
  EXPECT_EQ(first.at("grasp_pose_candidate_number"), 1U);
  EXPECT_EQ(first.at("attempts").at(0).at("material_candidate_id"), "material-a");
  EXPECT_EQ(second.at("material_candidate_id"), "material-b");
  EXPECT_EQ(second.at("grasp_pose_candidate_number"), 1U);
  EXPECT_EQ(second.at("attempts").at(0).at("material_candidate_id"), "material-b");
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, SplitsAllPickCandidatesAndKeepsSummaryIndependent)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_details_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  auto evidence = nlohmann::json::object();
  evidence["grasp_pose_candidates"] = nlohmann::json::array();
  evidence["pick_attempts"] = nlohmann::json::array();
  evidence["approach_global_planning_call_count"] = 18;
  evidence["selected_candidate"] = -1;
  evidence["active_candidate"] = 6;
  evidence["final_failure_stage"] = "CARTESIAN_GRASP";
  evidence["final_failure_reason"] = "NO_VALID_CARTESIAN_PATH";
  evidence["task_accuracy"] = nlohmann::json::array({
    {{"segment", "APPROACH"}, {"execution_result", "TASK_RESULT_SUCCESS"},
      {"measurement_status", "AVAILABLE"}, {"path_strategy", "GLOBAL"}},
    {{"segment", "APPROACH"}, {"execution_result", "TASK_RESULT_BACKEND_ERROR"},
      {"measurement_status", "UNAVAILABLE"}, {"path_strategy", "GLOBAL"}}});
  evidence["final_segment_planning"] = {
    {"cartesian_result", "FAILED"},
    {"cartesian_attempts", {{{"result", "FAILED"}, {"failure_reason", "collision"}}}},
    {"global_fallback_attempts", {
      {{"result", "PLANNING_FAILED"}, {"moveit_error_code", -1}},
      {{"result", "TRAJECTORY_INVALID"}, {"finalization_reason", "invalid_timing"}}}},
    {"final_planning_result", "FAILED"}, {"cartesian_failure_reason", "collision"}};
  for (std::size_t index = 0; index < 48; ++index) {
    evidence["grasp_pose_candidates"].push_back({
      {"candidate", index + 1}, {"tcp_pose", {{"position_xyz", {0, 0, 0.1}}}},
      {"search_group", index == 5U ? "VERTICAL_DESCENT" : "CENTER"},
      {"result", index < 6 ? "IK_SEED_ATTEMPTS_EXHAUSTED" : "NOT_EVALUATED"},
      {"planning_status", "NOT_ATTEMPTED"},
      {"transition_reason", index < 6 ? "TRAC_IK_NO_SOLUTION" : "SEARCH_STOPPED"}});
  }
  // New records carry pose metrics; keep candidate 6 above as a historical
  // group-bearing fixture to check both payload generations in the same writer.
  evidence["grasp_pose_candidates"][6].erase("search_group");
  evidence["grasp_pose_candidates"][6]["reference_grasp_tcp_distance_m"] = 0.012;
  evidence["grasp_pose_candidates"][6]["withdrawal_tilt_rad"] = 0.7;
  evidence["pick_attempts"].push_back({
    {"grasp_pose_candidate", 6}, {"attempt", 1}, {"stage", "IK"},
    {"reason", "TRAC_IK_NO_SOLUTION"}, {"grasp_ik", {{"reason", "TRAC_IK_NO_SOLUTION"}}}});
  evidence["pick_attempts"].push_back({
    {"grasp_pose_candidate", 7}, {"attempt", 1}, {"stage", "GRASP_VALIDATION"},
    {"reason", "final_grasp_state_collision"}, {"collision", true}});
  const auto logger = rclcpp::get_logger("motion_pick_detail_split_test");
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    const auto accepted = capture.enqueue_pick_details(
      "pick-48", "acquisition-1", evidence.dump());
    ASSERT_EQ(accepted, 48U);
    ASSERT_TRUE(capture.enqueue_pick_summary(
      "pick-48", "acquisition-1", 7, false, evidence.dump(), accepted));
  }
  const auto details = directory / "motion_planning_details";
  ASSERT_TRUE(std::filesystem::exists(details / "pick-48_candidate_048.json"));
  std::size_t detail_count = 0;
  for (const auto & entry : std::filesystem::directory_iterator(details)) {
    if (entry.path().extension() == ".json") {++detail_count;}
  }
  EXPECT_EQ(detail_count, 48U);
  std::ifstream summary_input(
    directory / "motion_planning_summaries" / "pick-48.json", std::ios::binary);
  ASSERT_TRUE(summary_input.good());
  const auto summary = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(summary_input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(summary.at("motion_goal_uuid"), "pick-48");
  EXPECT_EQ(summary.at("candidate_counts").at("generated"), 48);
  EXPECT_EQ(summary.at("candidate_counts").at("evaluated"), 6);
  EXPECT_EQ(summary.at("candidate_counts").at("unevaluated"), 42);
  EXPECT_EQ(summary.at("approach_attempt_count"), 18);
  EXPECT_EQ(summary.at("approach_execution_count"), 2);
  ASSERT_EQ(summary.at("approach_execution_results").size(), 2U);
  EXPECT_EQ(summary.at("approach_execution_results").at(1).at("execution_result"),
    "TASK_RESULT_BACKEND_ERROR");
  EXPECT_EQ(summary.at("final_failure_stage"), "GLOBAL_FALLBACK");
  EXPECT_EQ(summary.at("final_segment_planning").at("cartesian_attempts").size(), 1U);
  EXPECT_EQ(summary.at("final_segment_planning").at("global_fallback_attempts").size(), 2U);
  EXPECT_EQ(summary.at("stage_failure_totals").at("cartesian"), 1);
  EXPECT_EQ(summary.at("stage_failure_totals").at("fallback"), 2);
  EXPECT_EQ(summary.at("detail_coverage").at("status"), "persisted");
  EXPECT_EQ(summary.at("detail_coverage").at("persisted_count"), 48U);
  EXPECT_EQ(summary.at("detail_coverage").at("missing_count"), 0U);
  std::ifstream vertical_input(details / "pick-48_candidate_006.json", std::ios::binary);
  ASSERT_TRUE(vertical_input.good());
  const auto vertical = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(vertical_input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(summary.at("detail_coverage").at("expected_detail_revisions")
    .at("motion_planning_details/pick-48_candidate_006.json"),
    vertical.at("detail_revision"));
  EXPECT_EQ(vertical.at("candidate").at("search_group"), "VERTICAL_DESCENT");
  EXPECT_EQ(vertical.at("attempts").at(0).at("grasp_ik").at("reason"), "TRAC_IK_NO_SOLUTION");
  std::ifstream collision_input(details / "pick-48_candidate_007.json", std::ios::binary);
  ASSERT_TRUE(collision_input.good());
  const auto collision = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(collision_input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(collision.at("attempts").at(0).at("reason"), "final_grasp_state_collision");
  EXPECT_FALSE(collision.at("candidate").contains("search_group"));
  EXPECT_DOUBLE_EQ(collision.at("candidate").at("reference_grasp_tcp_distance_m"), 0.012);
  EXPECT_DOUBLE_EQ(collision.at("candidate").at("withdrawal_tilt_rad"), 0.7);
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, ProgressBacklogDoesNotDelayOrSuppressTerminalSummary)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_progress_backlog_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto logger = rclcpp::get_logger("motion_pick_progress_backlog_test");
  const auto progress = nlohmann::json{
    {"planning_evidence", std::string(48U * 1024U, 'p')}}.dump();
  const auto evidence = nlohmann::json{
    {"grasp_pose_candidates", nlohmann::json::array({
      {{"candidate", 1}, {"result", "IK_SEED_ATTEMPTS_EXHAUSTED"}}})},
    {"pick_attempts", nlohmann::json::array({{
      {"grasp_pose_candidate", 1}, {"attempt", 1}, {"stage", "IK"},
      {"reason", "NO_IK_SOLUTION"}}})}};
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    for (std::size_t index = 0; index < 16U; ++index) {
      ASSERT_TRUE(capture.enqueue_progress(
        "progress-backlog-" + std::to_string(index), "TASK_TYPE_PLACE", progress));
    }
    const auto accepted = capture.enqueue_pick_details(
      "pick-after-backlog", "acquisition-backlog", evidence.dump());
    ASSERT_EQ(accepted, 1U);
    ASSERT_TRUE(capture.enqueue_pick_summary(
      "pick-after-backlog", "acquisition-backlog", 3, false, evidence.dump(), accepted));
  }
  std::ifstream summary_input(
    directory / "motion_planning_summaries" / "pick-after-backlog.json");
  ASSERT_TRUE(summary_input.good());
  const auto summary = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(summary_input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(summary.at("task_executor_result_code"), "TASK_RESULT_PATH_OBSTRUCTED");
  EXPECT_EQ(summary.at("detail_coverage").at("status"), "persisted");
  EXPECT_EQ(summary.at("detail_coverage").at("persisted_count"), 1U);
  EXPECT_TRUE(std::filesystem::is_regular_file(
    directory / "motion_planning_details" / "pick-after-backlog_candidate_001.json"));
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, IdenticalProgressDoesNotRewritePersistedDetails)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_detail_dedup_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  auto evidence = nlohmann::json::object();
  evidence["grasp_pose_candidates"] = nlohmann::json::array();
  for (std::size_t index = 0; index < 48U; ++index) {
    evidence["grasp_pose_candidates"].push_back({
      {"candidate", index + 1U}, {"result", "NOT_EVALUATED"},
      {"transition_reason", "SEARCH_BUDGET_EXHAUSTED"}});
  }
  const auto logger = rclcpp::get_logger("motion_pick_detail_dedup_test");
  const auto detail_path = directory / "motion_planning_details" /
    "pick-dedup_candidate_048.json";
  std::filesystem::file_time_type original_write_time;
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    ASSERT_TRUE(capture.enqueue_progress("pick-dedup", "TASK_TYPE_PICK", evidence.dump()));
    bool detail_ready = false;
    for (int attempt = 0; attempt < 200 && !detail_ready; ++attempt) {
      detail_ready = std::filesystem::is_regular_file(detail_path);
      if (!detail_ready) {std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    }
    ASSERT_TRUE(detail_ready);
    original_write_time = std::filesystem::last_write_time(detail_path);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    ASSERT_TRUE(capture.enqueue_progress("pick-dedup", "TASK_TYPE_PICK", evidence.dump()));
  }
  EXPECT_EQ(std::filesystem::last_write_time(detail_path), original_write_time);
  std::size_t detail_count = 0U;
  for (const auto & entry : std::filesystem::directory_iterator(detail_path.parent_path())) {
    if (entry.path().extension() == ".json") {++detail_count;}
  }
  EXPECT_EQ(detail_count, 48U);
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, OversizedDetailDoesNotPreventSummary)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_detail_limit_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  auto evidence = nlohmann::json::object();
  evidence["grasp_pose_candidates"] = nlohmann::json::array({
    {{"candidate", 1}, {"result", "NOT_EVALUATED"}},
    {{"candidate", 2}, {"result", "NOT_EVALUATED"},
      {"diagnostic_padding", std::string(70U * 1024U, 'x')}}});
  const auto logger = rclcpp::get_logger("motion_pick_detail_limit_test");
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    const auto accepted = capture.enqueue_pick_details(
      "pick-limit", "acquisition-2", evidence.dump());
    EXPECT_EQ(accepted, 1U);
    ASSERT_TRUE(capture.enqueue_pick_summary(
      "pick-limit", "acquisition-2", 8, false, evidence.dump(), accepted));
  }
  EXPECT_TRUE(std::filesystem::exists(
    directory / "motion_planning_details" / "pick-limit_candidate_001.json"));
  EXPECT_FALSE(std::filesystem::exists(
    directory / "motion_planning_details" / "pick-limit_candidate_002.json"));
  std::ifstream summary_input(
    directory / "motion_planning_summaries" / "pick-limit.json", std::ios::binary);
  ASSERT_TRUE(summary_input.good());
  const auto summary = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(summary_input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(summary.at("detail_coverage").at("missing_count"), 1);
  EXPECT_EQ(summary.at("detail_coverage").at("status"), "missing");
  EXPECT_EQ(summary.at("detail_coverage").at("persisted_count"), 1U);
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, CandidateTransitionMergesPriorFailureEvidence)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_history_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto evidence = nlohmann::json{
    {"candidate_attempt_history", nlohmann::json::array({{
      {"generated_candidate_count", 2}, {"evaluated_candidate_count", 1},
      {"approach_global_planning_call_count", 8},
      {"rejections", {{"fk", 2}, {"collision_or_validity", 3}}},
      {"grasp_pose_candidates", nlohmann::json::array({
        {{"candidate", 1}, {"result", "IK_SEED_ATTEMPTS_EXHAUSTED"},
          {"transition_reason", "TRAC_IK_NO_SOLUTION"}},
        {{"candidate", 2}, {"result", "NOT_EVALUATED"}}})},
      {"pick_attempts", nlohmann::json::array({{
        {"grasp_pose_candidate", 1}, {"attempt", 1}, {"stage", "IK"},
        {"reason", "TRAC_IK_NO_SOLUTION"}}})}}})},
    {"approach_global_planning_call_count", 10},
    {"rejections", {{"fk", 1}, {"collision_or_validity", 4}}},
    {"grasp_pose_candidates", nlohmann::json::array({
      {{"candidate", 1}, {"result", "NOT_EVALUATED"}},
      {{"candidate", 2}, {"result", "APPROACH_PLANNING_FAILED"},
        {"transition_reason", "APPROACH_NO_PATH"}}})},
    {"pick_attempts", nlohmann::json::array({{
      {"grasp_pose_candidate", 2}, {"attempt", 1}, {"stage", "APPROACH"},
      {"reason", "APPROACH_NO_PATH"}}})}};
  const auto logger = rclcpp::get_logger("motion_pick_history_test");
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    const auto accepted = capture.enqueue_pick_details(
      "pick-history", "acquisition-history", evidence.dump());
    ASSERT_EQ(accepted, 2U);
    ASSERT_TRUE(capture.enqueue_pick_summary(
      "pick-history", "acquisition-history", 3, false, evidence.dump(), accepted));
  }
  std::ifstream prior_detail_input(
    directory / "motion_planning_details" / "pick-history_candidate_001.json");
  ASSERT_TRUE(prior_detail_input.good());
  const auto prior_detail = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(prior_detail_input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(prior_detail.at("candidate").at("result"), "IK_SEED_ATTEMPTS_EXHAUSTED");
  ASSERT_EQ(prior_detail.at("attempts").size(), 1U);
  EXPECT_EQ(prior_detail.at("attempts").at(0).at("reason"), "TRAC_IK_NO_SOLUTION");
  std::ifstream summary_input(
    directory / "motion_planning_summaries" / "pick-history.json");
  ASSERT_TRUE(summary_input.good());
  const auto summary = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(summary_input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(summary.at("candidate_counts").at("evaluated"), 2U);
  EXPECT_EQ(summary.at("approach_attempt_count"), 18U);
  EXPECT_EQ(summary.at("stage_failure_totals").at("fk"), 3U);
  EXPECT_EQ(summary.at("stage_failure_totals").at("collision"), 7U);
  EXPECT_EQ(summary.at("failure_stage_counts").at("IK").at("TRAC_IK_NO_SOLUTION"), 1U);
  EXPECT_EQ(summary.at("failure_stage_counts").at("APPROACH").at("APPROACH_NO_PATH"), 1U);
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, CandidateReevaluationKeepsNewAttemptAndAvoidsSnapshotDoubleCount)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_reevaluation_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto first_evaluation = nlohmann::json{
    {"evaluation_sequence", 1}, {"generated_candidate_count", 1},
    {"evaluated_candidate_count", 1}, {"approach_global_planning_call_count", 18},
    {"grasp_pose_candidates", nlohmann::json::array({
      {{"candidate", 1}, {"result", "IK_SEED_ATTEMPTS_EXHAUSTED"}}})},
    {"pick_attempts", nlohmann::json::array({{
      {"grasp_pose_candidate", 1}, {"attempt", 1}, {"stage", "IK"},
      {"reason", "TRAC_IK_NO_SOLUTION"}}})}};
  const auto second_evaluation = nlohmann::json{
    {"evaluation_sequence", 2}, {"generated_candidate_count", 1},
    {"evaluated_candidate_count", 1}, {"approach_global_planning_call_count", 18},
    {"grasp_pose_candidates", nlohmann::json::array({
      {{"candidate", 1}, {"result", "IK_SEED_ATTEMPTS_EXHAUSTED"}}})},
    {"pick_attempts", nlohmann::json::array({{
      {"grasp_pose_candidate", 1}, {"attempt", 1}, {"stage", "IK"},
      {"reason", "TRAC_IK_NO_SOLUTION"}}})}};
  auto evidence = second_evaluation;
  evidence["candidate_attempt_history"] = nlohmann::json::array({
    first_evaluation, first_evaluation});
  const auto logger = rclcpp::get_logger("motion_pick_reevaluation_test");
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    const auto accepted = capture.enqueue_pick_details(
      "pick-reevaluated", "acquisition-reevaluation", evidence.dump());
    ASSERT_EQ(accepted, 1U);
    ASSERT_TRUE(capture.enqueue_pick_summary(
      "pick-reevaluated", "acquisition-reevaluation", 3, false, evidence.dump(), accepted));
  }
  std::ifstream detail_input(
    directory / "motion_planning_details" / "pick-reevaluated_candidate_001.json");
  ASSERT_TRUE(detail_input.good());
  const auto detail = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(detail_input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(detail.at("detail_revision"), 1U);
  ASSERT_EQ(detail.at("attempts").size(), 2U);
  EXPECT_EQ(detail.at("attempts").at(0).at("attempt"), 1U);
  EXPECT_EQ(detail.at("attempts").at(1).at("attempt"), 1U);
  std::ifstream summary_input(
    directory / "motion_planning_summaries" / "pick-reevaluated.json");
  ASSERT_TRUE(summary_input.good());
  const auto summary = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(summary_input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(summary.at("candidate_counts").at("generated"), 1U);
  EXPECT_EQ(summary.at("candidate_counts").at("evaluated"), 1U);
  EXPECT_EQ(summary.at("approach_attempt_count"), 36U);
  EXPECT_EQ(summary.at("failure_stage_counts").at("IK").at("TRAC_IK_NO_SOLUTION"), 2U);
  EXPECT_EQ(summary.at("detail_coverage").at("persisted_count"), 1U);
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, FullDetailQueueLeavesTerminalSummaryAvailable)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_queue_full_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  auto evidence = nlohmann::json::object();
  evidence["grasp_pose_candidates"] = nlohmann::json::array();
  for (std::size_t index = 0; index < 130U; ++index) {
    evidence["grasp_pose_candidates"].push_back({
      {"candidate", index + 1U}, {"result", "NOT_EVALUATED"},
      {"transition_reason", "SEARCH_BUDGET_EXHAUSTED"}});
  }
  const auto logger = rclcpp::get_logger("motion_pick_queue_full_test");
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    const auto accepted = capture.enqueue_pick_details(
      "pick-queue", "acquisition-queue", evidence.dump());
    EXPECT_LT(accepted, 130U);
    ASSERT_TRUE(capture.enqueue_pick_summary(
      "pick-queue", "acquisition-queue", 5, false, evidence.dump(), accepted));
  }
  std::ifstream summary_input(
    directory / "motion_planning_summaries" / "pick-queue.json", std::ios::binary);
  ASSERT_TRUE(summary_input.good());
  const auto summary = nlohmann::json::parse(std::string(
      (std::istreambuf_iterator<char>(summary_input)), std::istreambuf_iterator<char>()));
  EXPECT_EQ(summary.at("task_executor_result_code"), "TASK_RESULT_CANCELED");
  EXPECT_EQ(summary.at("candidate_counts").at("generated"), 130);
  EXPECT_GT(summary.at("detail_coverage").at("missing_count").get<std::size_t>(), 0U);
  EXPECT_EQ(summary.at("detail_coverage").at("status"), "missing");
  EXPECT_EQ(summary.at("detail_coverage").at("persisted_count"), 128U);
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, TerminalOutcomesRemainDistinctAndCandidateAttemptsAreScoped)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_pick_terminals_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto logger = rclcpp::get_logger("motion_pick_terminal_outcomes_test");
  const auto empty = nlohmann::json::object().dump();
  const auto results = std::vector<std::pair<std::string, uint8_t>>{
    {"success", 0U}, {"canceled", 5U}, {"safety", 6U}, {"exception", 9U},
    {"failure", 3U}, {"exhausted", 11U}};
  {
    MotionPlanningDiagnosticCapture capture(directory.string(), logger);
    for (const auto & result : results) {
      ASSERT_TRUE(capture.enqueue_pick_summary(
        result.first, "shared-acquisition", result.second, result.first == "safety",
        empty, 0U));
    }
  }
  for (const auto & result : results) {
    std::ifstream input(
      directory / "motion_planning_summaries" / (result.first + ".json"), std::ios::binary);
    ASSERT_TRUE(input.good());
    const auto summary = nlohmann::json::parse(std::string(
        (std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>()));
    EXPECT_EQ(summary.at("motion_goal_uuid"), result.first);
    EXPECT_EQ(summary.at("task_executor_result_code_value"), result.second);
    if (result.first == "exhausted") {
      EXPECT_EQ(
        summary.at("task_executor_result_code"), "TASK_RESULT_GRASP_CANDIDATE_EXHAUSTED");
    }
    EXPECT_EQ(summary.at("pick_grasp_insertion_started"), result.first == "safety");
  }
  std::filesystem::remove_all(directory);
}

TEST(MotionPlanningDiagnosticCaptureTest, StorageFailureIsContainedByDiagnosticWriter)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("motion_diagnostic_storage_failure_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(directory.parent_path());
  {
    std::ofstream blocker(directory, std::ios::binary);
    ASSERT_TRUE(blocker.good());
    blocker << "file blocks directory creation";
  }
  {
    MotionPlanningDiagnosticCapture capture(directory.string(),
      rclcpp::get_logger("motion_diagnostic_storage_failure_test"));
    EXPECT_TRUE(capture.enqueue("blocked-goal", "TASK_TYPE_PLACE", 3U, "{}"));
  }
  EXPECT_TRUE(std::filesystem::is_regular_file(directory));
  std::filesystem::remove(directory);
}

}  // namespace arm_cell_motion
