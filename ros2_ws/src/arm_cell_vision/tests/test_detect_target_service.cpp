#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include "arm_cell_vision/detect_target_service.hpp"

namespace
{

class DetectTargetServiceTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}
};

std::shared_ptr<const arm_cell_vision::AcquisitionResult> make_acquisition_result(
  arm_cell_vision::DetectionResult detection, uint64_t generation)
{
  auto result = std::make_shared<arm_cell_vision::AcquisitionResult>();
  result->generation = generation;
  result->target_id = "RawPart";
  detection.scene_generation = generation;
  if (detection.algorithm.empty() && detection.has_target_pose && detection.ordered_candidates.empty()) {
    auto candidate = detection;
    candidate.confidence_grade = "HIGH";
    detection.ordered_candidates = {std::move(candidate)};
  }
  result->detection = std::move(detection);
  return result;
}

std::string captured_failure_log;

void capture_failure_log(
  const rcutils_log_location_t *, int severity, const char *, rcutils_time_point_value_t,
  const char * format, va_list * arguments)
{
  if (severity < RCUTILS_LOG_SEVERITY_WARN) {return;}
  char buffer[4096];
  va_list copy;
  va_copy(copy, *arguments);
  std::vsnprintf(buffer, sizeof(buffer), format, copy);
  va_end(copy);
  captured_failure_log += buffer;
}

class FailureLogCapture
{
public:
  FailureLogCapture()
  : previous_(rcutils_logging_get_output_handler())
  {
    captured_failure_log.clear();
    rcutils_logging_set_output_handler(capture_failure_log);
  }
  ~FailureLogCapture() {rcutils_logging_set_output_handler(previous_);}
private:
  rcutils_logging_output_handler_t previous_;
};

TEST_F(DetectTargetServiceTest, FailureLogsKeepCauseWhileResponseKeepsDetailedEvidence)
{
  using Code = arm_cell_interfaces::msg::DetectTargetResultCode;
  arm_cell_vision::VisionRosIngress ingress;
  arm_cell_vision::DetectionResult retained;
  retained.result_code = Code::DETECT_RESULT_GEOMETRY_ERROR;
  retained.failure_cause = "cuboid top geometry unresolved";
  retained.diagnostic_detail =
    "source_stamp_ns=123456789; acquisition_id=private-id; cuboid top geometry unresolved";
  arm_cell_interfaces::msg::DisturbanceCandidate unresolved;
  unresolved.category =
    arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_OTHER_UNRESOLVED;
  retained.disturbance_candidates.push_back(unresolved);
  arm_cell_interfaces::srv::DetectTarget::Response terminal_response;
  arm_cell_vision::DetectTargetService service(
    ingress, []() {return int64_t{100};}, {}, {},
    [&retained](const std::string &, uint64_t generation) {
      return make_acquisition_result(retained, generation);
    }, {},
    [&terminal_response](
      const std::string &, uint64_t,
      const arm_cell_interfaces::srv::DetectTarget::Response & response,
      const arm_cell_vision::SharedAcquisitionResult &,
      const arm_cell_vision::VisionRequestDiagnosticRecord &) {
      terminal_response = response;
    });
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.target_id = "RawPart";
  request.timeout.sec = 1;
  arm_cell_interfaces::srv::DetectTarget::Response response;
  FailureLogCapture capture;
  service.handle(request, response);
  EXPECT_EQ(response.result_code.value, Code::DETECT_RESULT_GEOMETRY_ERROR);
  ASSERT_EQ(response.disturbance_candidates.size(), 1U);
  EXPECT_EQ(response.disturbance_candidates.front().category,
    arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_OTHER_UNRESOLVED);
  EXPECT_NE(captured_failure_log.find("DETECT_RESULT_GEOMETRY_ERROR"), std::string::npos);
  EXPECT_NE(captured_failure_log.find("cause=cuboid top geometry unresolved"), std::string::npos);
  EXPECT_EQ(captured_failure_log.find("123456789"), std::string::npos);
  EXPECT_EQ(captured_failure_log.find("private-id"), std::string::npos);
  EXPECT_NE(response.diagnostic_detail.find("123456789"), std::string::npos);
  EXPECT_NE(response.diagnostic_detail.find("private-id"), std::string::npos);
  EXPECT_NE(terminal_response.diagnostic_detail.find(retained.diagnostic_detail), std::string::npos);
}

TEST_F(DetectTargetServiceTest, ReturnsUnifiedCandidatesWithoutRepresentativeProjection)
{
  using Code = arm_cell_interfaces::msg::DetectTargetResultCode;
  arm_cell_vision::VisionRosIngress ingress;
  arm_cell_vision::DetectionResult retained;
  retained.algorithm = "cuboid_top_face";
  retained.result_code = Code::DETECT_RESULT_SUCCESS;
  retained.target_pose.pose.orientation.w = 1.0;
  auto make_candidate = [](const std::string & grade, double x) {
      arm_cell_vision::DetectionResult candidate;
      candidate.algorithm = "cuboid_top_face";
      candidate.result_code = Code::DETECT_RESULT_SUCCESS;
      candidate.has_target_pose = true;
      candidate.confidence_grade = grade;
      candidate.target_pose.pose.position.x = x;
      candidate.target_pose.pose.orientation.w = 1.0;
      candidate.has_estimated_height = true;
      candidate.estimated_height_m = 0.03F;
      return candidate;
    };
  retained.ordered_candidates = {
    make_candidate("MEDIUM", 0.2), make_candidate("HIGH", 0.1),
    make_candidate("HIGH", 0.0)};
  arm_cell_vision::DetectTargetService service(
    ingress, []() {return int64_t{100};}, {}, {},
    [&retained](const std::string &, uint64_t generation) {
      return make_acquisition_result(retained, generation);
    });
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.target_id = "RawPart";
  request.timeout.sec = 1;
  arm_cell_interfaces::srv::DetectTarget::Response response;
  service.handle(request, response);

  ASSERT_EQ(response.result_code.value, Code::DETECT_RESULT_SUCCESS);
  ASSERT_EQ(response.material_candidates.size(), 3U);
  for (size_t i = 0; i < response.material_candidates.size(); ++i) {
    EXPECT_EQ(response.material_candidates[i].candidate_id, "candidate-" + std::to_string(i + 1));
    EXPECT_EQ(response.material_candidates[i].acquisition_id, response.acquisition_id);
  }
  EXPECT_DOUBLE_EQ(response.material_candidates[0].target_pose.pose.position.x, 0.2);
  EXPECT_DOUBLE_EQ(response.material_candidates[1].target_pose.pose.position.x, 0.1);

  retained.ordered_candidates = {make_candidate("MEDIUM", 0.25)};
  arm_cell_interfaces::srv::DetectTarget::Response medium_only;
  service.handle(request, medium_only);
  ASSERT_EQ(medium_only.result_code.value, Code::DETECT_RESULT_SUCCESS);
  ASSERT_EQ(medium_only.material_candidates.size(), 1U);
  EXPECT_DOUBLE_EQ(
    medium_only.material_candidates.at(0).target_pose.pose.position.x,
    medium_only.material_candidates.front().target_pose.pose.position.x);
  retained.ordered_candidates = {make_candidate("HIGH", 0.1)};
  arm_cell_interfaces::srv::DetectTarget::Response high_only;
  service.handle(request, high_only);
  EXPECT_EQ(high_only.result_code.value, Code::DETECT_RESULT_SUCCESS);
  ASSERT_EQ(high_only.material_candidates.size(), 1U);

  retained.ordered_candidates = {make_candidate("LOW", 0.2), make_candidate("", 0.3)};
  arm_cell_interfaces::srv::DetectTarget::Response ineligible;
  service.handle(request, ineligible);
  EXPECT_EQ(ineligible.result_code.value, Code::DETECT_RESULT_GEOMETRY_ERROR);
  EXPECT_TRUE(ineligible.material_candidates.empty());
}

TEST_F(DetectTargetServiceTest, TimeoutLogsPreserveTheObservedPreflightCause)
{
  arm_cell_vision::VisionRosIngress ingress;
  arm_cell_vision::DetectTargetService service(
    ingress, []() {return int64_t{100};}, {}, {}, {},
    []() {
      return arm_cell_vision::AcquisitionDiagnosticSummary{
        "Depth Scene Stability STABLE; stable_depth_stamp_ns=123456789",
        "tf_unavailable"};
    });
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.timeout.nanosec = 0;
  arm_cell_interfaces::srv::DetectTarget::Response response;
  FailureLogCapture capture;
  service.handle(request, response);
  EXPECT_EQ(
    response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
  EXPECT_NE(captured_failure_log.find("DETECT_RESULT_TIMEOUT"), std::string::npos);
  EXPECT_NE(captured_failure_log.find("tf_unavailable"), std::string::npos);
  EXPECT_EQ(captured_failure_log.find("123456789"), std::string::npos);
  EXPECT_NE(response.diagnostic_detail.find("123456789"), std::string::npos);
  EXPECT_EQ(
    response.diagnostic_detail.find("processing_elapsed_ms=not_started"),
    std::string::npos);
}

TEST_F(DetectTargetServiceTest, AcquisitionIdentityChangesWithVisionRun)
{
  constexpr uint64_t generation = 1;
  const auto previous_run = arm_cell_vision::make_acquisition_id("vision-run-old", generation);
  const auto current_run = arm_cell_vision::make_acquisition_id("vision-run-new", generation);

  EXPECT_FALSE(previous_run.empty());
  EXPECT_NE(previous_run, current_run);
}

TEST_F(DetectTargetServiceTest, RejectsNegativeRelativeTimeout)
{
  arm_cell_vision::VisionRosIngress ingress;
  arm_cell_vision::DetectTargetService service(
    ingress, []() {return int64_t{100};});
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.timeout.sec = -1;
  arm_cell_interfaces::srv::DetectTarget::Response response;

  service.handle(request, response);

  EXPECT_EQ(
    response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_INVALID_RESULT);
  EXPECT_TRUE(response.acquisition_id.empty());
}

TEST_F(DetectTargetServiceTest, ValidatesAndPassesEachRequestedAlgorithmToAcquisition)
{
  arm_cell_vision::VisionRosIngress ingress;
  std::vector<std::string> selected;
  arm_cell_vision::DetectTargetService service(
    ingress, []() {return int64_t{100};},
    [&](const std::string &, uint64_t, int64_t, int64_t, const std::string & algorithm) {
      selected.push_back(algorithm);
      return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET;
    });
  arm_cell_interfaces::srv::DetectTarget::Request request;
  EXPECT_EQ(request.algorithm, "cuboid_top_face");
  request.target_id = "RawPart";
  request.timeout.sec = 30;
  for (const auto & algorithm : {"cuboid_top_face", "cuboid_top_face"}) {
    request.algorithm = algorithm;
    arm_cell_interfaces::srv::DetectTarget::Response response;
    service.handle(request, response);
    ASSERT_FALSE(selected.empty());
    EXPECT_EQ(selected.back(), algorithm);
  }
  EXPECT_EQ(selected.size(), 2U);
  for (const auto & algorithm : {"", "unsupported", "CUBOID_TOP_FACE"}) {
    request.algorithm = algorithm;
    arm_cell_interfaces::srv::DetectTarget::Response response;
    service.handle(request, response);
    EXPECT_EQ(response.result_code.value,
      arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_INVALID_RESULT);
    EXPECT_NE(response.diagnostic_detail.find("algorithm"), std::string::npos);
    EXPECT_TRUE(response.acquisition_id.empty());
  }
  EXPECT_EQ(selected.size(), 2U);
}

TEST_F(DetectTargetServiceTest, ReturnsCalibrationNotReadyWithoutStartingAcquisitionDeadline)
{
  arm_cell_vision::VisionRosIngress ingress;
  int begin_count = 0;
  arm_cell_vision::DetectTargetService service(
    ingress, []() {return int64_t{100};},
    [&begin_count](const std::string &, uint64_t, int64_t, int64_t, const std::string &) {
      ++begin_count;
      return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET;
    });
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.target_id = "RawPart";
  request.timeout.sec = 30;
  arm_cell_interfaces::srv::DetectTarget::Response response;

  service.handle(request, response);

  EXPECT_EQ(response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
  EXPECT_TRUE(response.acquisition_id.empty());
  EXPECT_NE(response.diagnostic_detail.find("calibration is not READY"), std::string::npos);
  EXPECT_EQ(begin_count, 1);
}

TEST_F(DetectTargetServiceTest, UsesOneDeadlineForTheActiveAcquisition)
{
  rcutils_logging_set_logger_level("vision_ros_ingress", RCUTILS_LOG_SEVERITY_DEBUG);
  arm_cell_vision::VisionRosIngress ingress(rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("diagnostics.capture_enabled", true)}));
  int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  arm_cell_interfaces::srv::DetectTarget::Response terminal_response;
  uint64_t terminal_generation = 0;
  arm_cell_vision::VisionRequestDiagnosticRecord terminal_diagnostics;
  arm_cell_vision::DetectTargetService service(
    ingress, [&now]() {now += 100'000; return now;}, {}, {}, {}, {},
    [&terminal_response, &terminal_generation, &terminal_diagnostics](const std::string &, uint64_t generation,
      const arm_cell_interfaces::srv::DetectTarget::Response & terminal,
      const arm_cell_vision::SharedAcquisitionResult &,
      const arm_cell_vision::VisionRequestDiagnosticRecord & diagnostics) {
      terminal_response = terminal;
      terminal_generation = generation;
      terminal_diagnostics = diagnostics;
    }, "vision-run-timeout");
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.timeout.nanosec = 2'000'000U;
  arm_cell_interfaces::srv::DetectTarget::Response response;

  service.handle(request, response);

  EXPECT_EQ(
    response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
  EXPECT_EQ(response.acquisition_id, terminal_response.acquisition_id);
  EXPECT_EQ(response.acquisition_id,
    arm_cell_vision::make_acquisition_id("vision-run-timeout", terminal_generation));
  EXPECT_FALSE(response.acquisition_id.empty());
  EXPECT_NE(response.diagnostic_detail.find("no depth observed"), std::string::npos);
  EXPECT_NE(response.diagnostic_detail.find("latest_age_ms"), std::string::npos);
  EXPECT_NE(response.diagnostic_detail.find("last_ingress_status"), std::string::npos);
  EXPECT_GT(terminal_diagnostics.deadline_monotonic_ns, 0);
  EXPECT_EQ(terminal_diagnostics.timeout_ns, 2'000'000);
  EXPECT_EQ(terminal_diagnostics.result_code, response.result_code.value);
  EXPECT_EQ(terminal_diagnostics.ingress.clock_epoch_start, 0U);
  rcutils_logging_set_logger_level("vision_ros_ingress", RCUTILS_LOG_SEVERITY_INFO);
}

TEST_F(DetectTargetServiceTest, HandsOffOnlyTheMatchingAcquisitionResultOnce)
{
  arm_cell_vision::VisionRosIngress ingress;
  int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  uint64_t acquisition_generation = 0;
  int begin_count = 0;
  int end_count = 0;
  int lookup_count = 0;
  arm_cell_vision::DetectionResult retained;
  retained.result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  retained.has_target_pose = true;
  retained.target_pose.header.frame_id = "base_link";
  retained.target_pose.pose.orientation.w = 1.0;
  retained.has_observed_rectangle = true;
  retained.rectangle_long_side_m = 0.08F;
  retained.rectangle_short_side_m = 0.07F;
  retained.estimated_height_m = 0.075F;
  retained.has_estimated_height = true;
  retained.orientation_symmetry_order = 2;
  retained.source_rgb_header.frame_id = "camera_color_optical_frame";
  arm_cell_vision::DetectTargetService service(
    ingress, [&now]() {return now++;},
    [&acquisition_generation, &begin_count](const std::string & target_id,
      uint64_t generation, int64_t, int64_t, const std::string &) {
      EXPECT_EQ(target_id, "RawPart");
      ++begin_count;
      acquisition_generation = generation;
      return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
    },
    [&end_count]() {++end_count;},
    [&retained, &lookup_count](const std::string & target_id, uint64_t generation) {
      ++lookup_count;
      if (target_id != "RawPart" || generation == 0U) {
        return arm_cell_vision::SharedAcquisitionResult{};
      }
      // A completed callback from the prior request must not satisfy this one.
      return make_acquisition_result(retained, generation + (lookup_count == 1 ? 1U : 0U));
    }, {}, {}, "vision-run-success");
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.target_id = "RawPart";
  request.timeout.sec = 1;
  arm_cell_interfaces::srv::DetectTarget::Response response;

  service.handle(request, response);

  EXPECT_EQ(response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS);
  EXPECT_EQ(response.scene_generation, acquisition_generation);
  EXPECT_EQ(response.acquisition_id,
    arm_cell_vision::make_acquisition_id("vision-run-success", acquisition_generation));
  EXPECT_EQ(response.material_candidates.at(0).target_pose.header.frame_id, "base_link");
  EXPECT_TRUE(response.material_candidates.at(0).has_observed_rectangle);
  EXPECT_FLOAT_EQ(response.material_candidates.at(0).rectangle_long_side_m, 0.08F);
  EXPECT_FLOAT_EQ(response.material_candidates.at(0).rectangle_short_side_m, 0.07F);
  EXPECT_FLOAT_EQ(response.material_candidates.at(0).estimated_height_m, 0.075F);
  EXPECT_EQ(response.material_candidates.at(0).orientation_symmetry_order, 2U);
  EXPECT_EQ(begin_count, 1);
  EXPECT_EQ(end_count, 1);
  EXPECT_EQ(lookup_count, 2);
}

TEST_F(DetectTargetServiceTest, MislocalizationIsRetainedAcrossFailureAndConsumedOnceOnSuccess)
{
  arm_cell_vision::VisionRosIngress ingress;
  int64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  arm_cell_vision::DetectionResult retained;
  retained.result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  retained.has_target_pose = true;
  retained.target_pose.header.frame_id = "base_link";
  retained.target_pose.pose.position.z = 0.25;
  retained.target_pose.pose.orientation.w = 1.0;
  retained.source_rgb_header.frame_id = "camera_color_optical_frame";
  retained.ordered_candidates = {retained, retained};
  retained.ordered_candidates[0].confidence_grade = "MEDIUM";
  retained.ordered_candidates[1].confidence_grade = "HIGH";
  retained.ordered_candidates[1].target_pose.pose.position.z = 0.45;
  bool detection_success = false;
  bool pose_transform_available = true;
  arm_cell_vision::DetectTargetService service(
    ingress, [&now]() {return now++;},
    [](const std::string &, uint64_t, int64_t, int64_t, const std::string &) {
      return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
    }, []() {},
    [&retained, &detection_success](const std::string &, uint64_t generation) {
      retained.scene_generation = generation;
      retained.result_code = detection_success ?
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS :
        arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND;
      return make_acquisition_result(retained, generation);
    }, {}, {}, "vision-run-mislocalize",
    [&pose_transform_available](const std::string &) ->
      std::optional<geometry_msgs::msg::Vector3> {
      if (!pose_transform_available) {return std::nullopt;}
      geometry_msgs::msg::Vector3 normal;
      normal.z = 1.0;
      return normal;
    });
  service.set_mislocalize_next(true);

  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.target_id = "RawPart";
  request.timeout.sec = 1;
  arm_cell_interfaces::srv::DetectTarget::Response failed_response;
  service.handle(request, failed_response);
  EXPECT_EQ(failed_response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_OBJECT_NOT_FOUND);
  EXPECT_TRUE(failed_response.disturbance_candidates.empty());
  EXPECT_TRUE(service.mislocalize_pending());

  detection_success = true;
  arm_cell_interfaces::srv::DetectTarget::Response injected_response;
  service.handle(request, injected_response);
  EXPECT_EQ(injected_response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS);
  ASSERT_EQ(injected_response.material_candidates.size(), 2U);
  EXPECT_DOUBLE_EQ(injected_response.material_candidates[0].target_pose.pose.position.z, 0.35);
  EXPECT_DOUBLE_EQ(injected_response.material_candidates[1].target_pose.pose.position.z, 0.55);
  EXPECT_NE(injected_response.diagnostic_detail.find("mislocalization_applied=true"),
    std::string::npos);
  EXPECT_NE(injected_response.diagnostic_detail.find(injected_response.acquisition_id),
    std::string::npos);
  EXPECT_FALSE(service.mislocalize_pending());

  arm_cell_interfaces::srv::DetectTarget::Response normal_response;
  service.handle(request, normal_response);
  EXPECT_DOUBLE_EQ(normal_response.material_candidates.at(0).target_pose.pose.position.z, 0.25);
  EXPECT_DOUBLE_EQ(normal_response.material_candidates.at(1).target_pose.pose.position.z, 0.45);
}

TEST_F(DetectTargetServiceTest, RejectsOverlappingRequestAsBusyWithoutStartingAcquisition)
{
  arm_cell_vision::VisionRosIngress ingress;
  std::mutex gate_mutex;
  std::condition_variable gate_condition;
  bool acquisition_begin_entered = false;
  bool release_acquisition_begin = false;
  std::atomic<int> acquisition_begin_count{0};
  std::atomic<uint64_t> first_acquisition_generation{0};
  std::atomic<uint64_t> next_acquisition_generation{0};
  std::atomic<int> result_lookup_count{0};
  std::atomic<int> terminal_snapshot_count{0};
  arm_cell_vision::DetectTargetService service(
    ingress,
    []() {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    },
    [&](const std::string &, uint64_t generation, int64_t, int64_t, const std::string &) {
      const auto begin_count = ++acquisition_begin_count;
      if (begin_count == 1) {
        first_acquisition_generation = generation;
      } else {
        next_acquisition_generation = generation;
      }
      std::unique_lock<std::mutex> lock(gate_mutex);
      acquisition_begin_entered = true;
      gate_condition.notify_one();
      gate_condition.wait(lock, [&]() {return release_acquisition_begin;});
      return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
    },
    []() {},
    [&](const std::string &, uint64_t) {
      ++result_lookup_count;
      return arm_cell_vision::SharedAcquisitionResult{};
    },
    {},
    [&](const std::string &, uint64_t,
      const arm_cell_interfaces::srv::DetectTarget::Response &,
      const arm_cell_vision::SharedAcquisitionResult &,
      const arm_cell_vision::VisionRequestDiagnosticRecord &) {
      ++terminal_snapshot_count;
    });

  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.target_id = "RawPart";
  request.timeout.sec = 1;
  arm_cell_interfaces::srv::DetectTarget::Response active_response;
  std::thread active_request([&]() {service.handle(request, active_response);});
  bool acquisition_started = false;
  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    acquisition_started = gate_condition.wait_for(
      lock, std::chrono::seconds(1), [&]() {return acquisition_begin_entered;});
  }
  if (!acquisition_started) {
    {
      std::lock_guard<std::mutex> lock(gate_mutex);
      release_acquisition_begin = true;
    }
    gate_condition.notify_one();
    active_request.join();
    FAIL() << "active DetectTarget request did not claim the acquisition slot";
    return;
  }

  arm_cell_interfaces::srv::DetectTarget::Response competing_response;
  service.handle(request, competing_response);

  EXPECT_EQ(
    competing_response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_BUSY);
  EXPECT_TRUE(competing_response.acquisition_id.empty());
  EXPECT_NE(competing_response.diagnostic_detail.find("contention"), std::string::npos);
  EXPECT_EQ(acquisition_begin_count.load(), 1);
  EXPECT_EQ(result_lookup_count.load(), 0);
  EXPECT_EQ(terminal_snapshot_count.load(), 0);

  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    release_acquisition_begin = true;
  }
  gate_condition.notify_one();
  active_request.join();

  EXPECT_EQ(terminal_snapshot_count.load(), 1);

  request.timeout.sec = 0;
  request.timeout.nanosec = 1;
  arm_cell_interfaces::srv::DetectTarget::Response next_response;
  service.handle(request, next_response);
  EXPECT_EQ(next_acquisition_generation.load(), first_acquisition_generation.load() + 1);
}

TEST_F(DetectTargetServiceTest, DoesNotAcceptAResultAfterItsDeadline)
{
  arm_cell_vision::VisionRosIngress ingress;
  int64_t now = 100;
  int end_count = 0;
  int terminal_count = 0;
  uint8_t terminal_result_code = 0;
  arm_cell_interfaces::srv::DetectTarget::Response terminal_response;
  arm_cell_vision::DetectionResult retained;
  retained.result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
  retained.has_target_pose = true;
  retained.scene_generation = 1;
  arm_cell_vision::DetectTargetService service(
    ingress, [&now]() {return now;},
    [](const std::string &, uint64_t, int64_t, int64_t, const std::string &) {
      return arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS;
    },
    [&end_count]() {++end_count;},
    [&now, &retained](const std::string &, uint64_t generation) {
      now = 1'000'000'101;
      return make_acquisition_result(retained, generation);
    }, {}, [&terminal_count, &terminal_result_code, &terminal_response](const std::string & target_id,
      uint64_t generation, const arm_cell_interfaces::srv::DetectTarget::Response & result,
      const arm_cell_vision::SharedAcquisitionResult &,
      const arm_cell_vision::VisionRequestDiagnosticRecord & diagnostic_record) {
      EXPECT_EQ(target_id, "");
      EXPECT_NE(generation, 0U);
      ++terminal_count;
      terminal_result_code = result.result_code.value;
      terminal_response = result;
      EXPECT_FALSE(diagnostic_record.ingress.detailed_capture_enabled);
    });
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.timeout.nanosec = 1U;
  arm_cell_interfaces::srv::DetectTarget::Response response;

  service.handle(request, response);

  EXPECT_EQ(response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
  EXPECT_EQ(end_count, 1);
  EXPECT_EQ(terminal_count, 1);
  EXPECT_EQ(terminal_result_code,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
  EXPECT_FALSE(response.acquisition_id.empty());
  EXPECT_EQ(response.acquisition_id, terminal_response.acquisition_id);
}

TEST_F(DetectTargetServiceTest, ReturnsOrderedCandidatesFromOneQualifiedAcquisition)
{
  using Code = arm_cell_interfaces::msg::DetectTargetResultCode;
  arm_cell_vision::VisionRosIngress ingress;
  int64_t now = 100;
  int observations = 0;
  arm_cell_vision::DetectionResult retained;
  retained.result_code = Code::DETECT_RESULT_SUCCESS;
  retained.has_target_pose = true;
  retained.algorithm = "cuboid_top_face";
  retained.applied_profile = std::make_shared<arm_cell_vision::DetectorProfile>();
  retained.target_pose.header.frame_id = "base_link";
  retained.target_pose.header.stamp.sec = 42;
  retained.target_pose.pose.orientation.w = 1;
  retained.estimated_height_m = 0.03F;
  retained.has_estimated_height = true;
  arm_cell_interfaces::msg::DisturbanceCandidate disturbance;
  disturbance.candidate_id = "disturbance-1";
  disturbance.category =
    arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_OTHER_UNRESOLVED;
  disturbance.center.header.frame_id = "base_link";
  disturbance.center.header.stamp.sec = 42;
  disturbance.center.pose.position.x = 0.4;
  disturbance.observed_height_m = 0.07;
  retained.disturbance_candidates = {disturbance};
  retained.ordered_candidates = {retained, retained, retained};
  retained.ordered_candidates[0].confidence_grade = "HIGH";
  retained.ordered_candidates[1].confidence_grade = "MEDIUM";
  retained.ordered_candidates[2].confidence_grade = "HIGH";
  retained.ordered_candidates[1].target_pose.pose.position.x = 0.1;
  retained.ordered_candidates[2].target_pose.pose.position.x = 0.2;
  for (size_t i = 0; i < retained.ordered_candidates.size(); ++i) {
    auto & source = retained.ordered_candidates[i];
    source.orientation_symmetry_order = 2;
    source.has_observed_rectangle = i != 1;
    source.rectangle_long_side_m = 0.08F;
    source.rectangle_short_side_m = 0.04F;
    arm_cell_interfaces::msg::CollisionPrimitive occupied;
    occupied.shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    occupied.pose.orientation.w = 1.0;
    occupied.pose.position.x = 1.0 + i;
    occupied.shape.dimensions = {0.01, 0.02, 0.03};
  }
  arm_cell_vision::DetectTargetService service(ingress, [&]() {return now++;},
    {}, {}, [&](const std::string &, uint64_t generation) {
      ++observations; retained.scene_generation = generation;
      return make_acquisition_result(retained, generation);
    });
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.algorithm = "cuboid_top_face";
  request.target_id = "RawPart"; request.timeout.sec = 1;
  arm_cell_interfaces::srv::DetectTarget::Response first;
  service.handle(request, first);
  ASSERT_EQ(first.result_code.value, Code::DETECT_RESULT_SUCCESS);
  ASSERT_EQ(first.material_candidates.size(), 3U);
  ASSERT_EQ(first.disturbance_candidates.size(), 1U);
  EXPECT_EQ(first.disturbance_candidates.front().candidate_id, "disturbance-1");
  EXPECT_EQ(
    first.disturbance_candidates.front().category,
    arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_OTHER_UNRESOLVED);
  EXPECT_EQ(first.disturbance_candidates.front().center.header.frame_id, "base_link");
  EXPECT_EQ(first.disturbance_candidates.front().center.header.stamp.sec, 42);
  EXPECT_DOUBLE_EQ(first.disturbance_candidates.front().center.pose.position.x, 0.4);
  EXPECT_DOUBLE_EQ(first.disturbance_candidates.front().observed_height_m, 0.07);
  EXPECT_NE(
    first.diagnostic_detail.find(
      "disturbance_candidates=1{inclined_surface=0,elevated_unresolved=0,other_unresolved=1}"),
    std::string::npos);
  for (const auto & candidate : first.material_candidates) {
    EXPECT_DOUBLE_EQ(candidate.top_height_tolerance_m, 0.003);
  }
  EXPECT_DOUBLE_EQ(first.material_candidates[1].top_height_tolerance_m, 0.003);
  EXPECT_DOUBLE_EQ(first.material_candidates[1].target_pose.pose.position.x, 0.1);
  EXPECT_EQ(first.material_candidates[1].target_pose.header.stamp.sec, 42);
  EXPECT_DOUBLE_EQ(first.material_candidates[2].target_pose.pose.position.x, 0.2);
  EXPECT_NE(
    first.material_candidates[0].candidate_id,
    first.material_candidates[1].candidate_id);
  EXPECT_NE(
    first.material_candidates[2].candidate_id,
    first.material_candidates[1].candidate_id);
  for (size_t i = 0; i < first.material_candidates.size(); ++i) {
    const auto & actual = first.material_candidates[i];
    const auto & source = retained.ordered_candidates[i];
    EXPECT_EQ(actual.acquisition_id, first.acquisition_id);
    EXPECT_EQ(actual.target_pose, source.target_pose);
    EXPECT_EQ(actual.orientation_symmetry_order, source.orientation_symmetry_order);
    EXPECT_EQ(actual.has_estimated_height, source.has_estimated_height);
    EXPECT_EQ(actual.estimated_height_m, source.estimated_height_m);
    EXPECT_EQ(actual.has_observed_rectangle, source.has_observed_rectangle);
    EXPECT_EQ(actual.rectangle_long_side_m, source.rectangle_long_side_m);
    EXPECT_EQ(actual.rectangle_short_side_m, source.rectangle_short_side_m);
  }
  EXPECT_EQ(observations, 1);
  arm_cell_interfaces::srv::DetectTarget::Response fresh;
  service.handle(request, fresh);
  EXPECT_NE(fresh.acquisition_id, first.acquisition_id);
}

TEST_F(DetectTargetServiceTest, TemporaryInvalidTargetReturnsSameAcquisitionDisturbanceEvidence)
{
  using Code = arm_cell_interfaces::msg::DetectTargetResultCode;
  arm_cell_vision::VisionRosIngress ingress;
  arm_cell_vision::DetectionResult retained;
  retained.algorithm = "cuboid_top_face";
  retained.result_code = Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET;
  arm_cell_interfaces::msg::DisturbanceCandidate disturbance;
  disturbance.category =
    arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_ELEVATED_UNRESOLVED;
  disturbance.center.header.frame_id = "base_link";
  disturbance.center.header.stamp.sec = 71;
  disturbance.observed_height_m = 0.04;
  retained.disturbance_candidates = {disturbance};
  arm_cell_vision::DetectTargetService service(
    ingress, []() {return int64_t{100};}, {}, {},
    [&retained](const std::string &, uint64_t generation) {
      return make_acquisition_result(retained, generation);
    });

  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.target_id = "RawPart";
  request.timeout.sec = 1;
  arm_cell_interfaces::srv::DetectTarget::Response response;
  service.handle(request, response);

  EXPECT_EQ(
    response.result_code.value, Code::DETECT_RESULT_TEMPORARY_INVALID_TARGET);
  EXPECT_TRUE(response.material_candidates.empty());
  ASSERT_EQ(response.disturbance_candidates.size(), 1U);
  EXPECT_EQ(response.disturbance_candidates.front().candidate_id, "disturbance-1");
  EXPECT_EQ(response.disturbance_candidates.front().center.header.frame_id, "base_link");
  EXPECT_EQ(response.disturbance_candidates.front().center.header.stamp.sec, 71);
  EXPECT_DOUBLE_EQ(response.disturbance_candidates.front().observed_height_m, 0.04);
  EXPECT_FALSE(response.acquisition_id.empty());
}

TEST_F(DetectTargetServiceTest, RejectsLegacyRectangleAlgorithm)
{
  arm_cell_vision::VisionRosIngress ingress;
  arm_cell_vision::DetectTargetService service(ingress, []() {return int64_t{100};});
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.target_id = "RawPart";
  request.algorithm = "legacy_rectangle";
  arm_cell_interfaces::srv::DetectTarget::Response response;

  service.handle(request, response);

  EXPECT_EQ(response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_INVALID_RESULT);
  EXPECT_TRUE(response.acquisition_id.empty());
  EXPECT_NE(
    response.diagnostic_detail.find("unsupported DetectTarget.algorithm"), std::string::npos);
}

}  // namespace
