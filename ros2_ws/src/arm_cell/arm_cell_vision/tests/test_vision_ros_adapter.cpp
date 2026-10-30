#include <chrono>
#include <condition_variable>
#include <fstream>
#include <future>
#include <memory>
#include <limits>
#include <sstream>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <rosgraph_msgs/msg/clock.hpp>

#include "arm_cell_vision/clock_epoch_observer.hpp"
#include "arm_cell_vision/vision_ros_adapter.hpp"
#include "arm_cell_vision/vision_ros_ingress.hpp"

namespace arm_cell_vision
{
namespace
{

int64_t ns(int32_t sec, uint32_t nanosec = 0)
{
  return static_cast<int64_t>(sec) * 1'000'000'000LL + nanosec;
}

int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

sensor_msgs::msg::Image image(int32_t sec, uint32_t nanosec = 0)
{
  sensor_msgs::msg::Image message;
  message.header.stamp.sec = sec;
  message.header.stamp.nanosec = nanosec;
  message.header.frame_id = "camera_color_optical_frame";
  message.width = 1280;
  message.height = 720;
  return message;
}

sensor_msgs::msg::CameraInfo camera_info(int32_t sec, uint32_t nanosec = 0)
{
  sensor_msgs::msg::CameraInfo message;
  message.header.stamp.sec = sec;
  message.header.stamp.nanosec = nanosec;
  message.header.frame_id = "camera_color_optical_frame";
  message.width = 1280;
  message.height = 720;
  message.distortion_model = "plumb_bob";
  message.k = {600.0, 0.0, 640.0, 0.0, 600.0, 360.0, 0.0, 0.0, 1.0};
  message.p = {600.0, 0.0, 640.0, 0.0, 0.0, 600.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  message.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  return message;
}

class VisionRosIngressTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<VisionRosIngress> make_ingress(
    double slop_ms, double reusable_pair_max_age_ms = 200.0, bool capture_details = false)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("sync_slop_ms", slop_ms),
      rclcpp::Parameter("reusable_pair_max_age_ms", reusable_pair_max_age_ms),
      rclcpp::Parameter("diagnostics.capture_enabled", capture_details)});
    return std::make_shared<VisionRosIngress>(options);
  }

  void spin_until(
    rclcpp::executors::SingleThreadedExecutor & executor,
    const std::function<bool()> & predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(std::chrono::milliseconds(5));
    }
    ASSERT_TRUE(predicate());
  }

  void spin_for(
    rclcpp::executors::SingleThreadedExecutor & executor,
    std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_once(std::chrono::milliseconds(5));
    }
  }
};

TEST(VisionRosAdapter, PreservesIndividualStampsAndUsesRgbAsCanonicalStamp)
{
  const auto rgb = image(10, 1);
  const auto depth = image(10, 2);
  const auto info = camera_info(10, 3);
  const auto candidate = VisionRosAdapter::to_candidate(rgb, depth, info, {11, 12, 13});

  EXPECT_EQ(candidate.common_stamp_ns, ns(10, 1));
  EXPECT_EQ(candidate.rgb.stamp_ns, ns(10, 1));
  EXPECT_EQ(candidate.depth.stamp_ns, ns(10, 2));
  EXPECT_EQ(candidate.camera_info.stamp_ns, ns(10, 3));
  EXPECT_EQ(candidate.rgb.receipt_monotonic_ns, 11);
  EXPECT_EQ(candidate.depth.receipt_monotonic_ns, 12);
  EXPECT_EQ(candidate.camera_info.receipt_monotonic_ns, 13);
}

TEST_F(VisionRosIngressTest, RequestCaptureDoesNotRequireDebugLogging)
{
  ASSERT_EQ(
    rcutils_logging_set_logger_level("vision_ros_ingress", RCUTILS_LOG_SEVERITY_INFO),
    RCUTILS_RET_OK);
  auto ingress = make_ingress(10.0, 200.0, true);
  ingress->begin_acquisition(0);

  const auto diagnostics = ingress->finish_request_diagnostics();

  EXPECT_TRUE(diagnostics.detailed_capture_enabled);
  EXPECT_EQ(
    rcutils_logging_set_logger_level("vision_ros_ingress", RCUTILS_LOG_SEVERITY_UNSET),
    RCUTILS_RET_OK);
}

TEST(VisionRosAdapter, M1cFixtureNominalFieldsReachPolicyWithoutRewriting)
{
  std::ifstream manifest_stream(M1C_FIXTURE_MANIFEST);
  ASSERT_TRUE(manifest_stream.good());
  std::stringstream manifest_content;
  manifest_content << manifest_stream.rdbuf();
  const auto manifest = manifest_content.str();
  ASSERT_NE(manifest.find("\"status\": \"PARTIALLY VERIFIED\""), std::string::npos);
  ASSERT_NE(manifest.find("\"frame_id\": \"camera_color_optical_frame\""), std::string::npos);
  ASSERT_NE(manifest.find("\"width\": 1280"), std::string::npos);
  ASSERT_NE(manifest.find("\"height\": 720"), std::string::npos);
  ASSERT_NE(
    manifest.find("\"selected_image_stamp_range_ns\": [\n    36933335259"),
    std::string::npos);
  ASSERT_NE(manifest.find("\"depth_unit\": \"NOT VERIFIED\""), std::string::npos);
  ASSERT_NE(manifest.find("\"depth_semantics\": \"NOT VERIFIED\""), std::string::npos);
  ASSERT_NE(manifest.find("\"pixel_correspondence\": \"NOT VERIFIED\""), std::string::npos);

  sensor_msgs::msg::Image rgb = image(36, 933335259);
  sensor_msgs::msg::Image depth = image(36, 933335259);
  sensor_msgs::msg::CameraInfo info = camera_info(36, 933335259);
  rgb.encoding = "rgb8";
  depth.encoding = "32FC1";
  info.distortion_model = "plumb_bob";
  info.k = {1465.9985736982494, 0.0, 640.0, 0.0, 1465.9985736982494, 360.0, 0.0, 0.0, 1.0};
  info.p = {1465.9985736982494, 0.0, 640.0, 0.0, 0.0, 1465.9985736982494, 360.0,
    0.0, 0.0, 0.0, 1.0, 0.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

  const auto candidate = VisionRosAdapter::to_candidate(rgb, depth, info, {100, 101, 102});
  EXPECT_TRUE(VisionRosAdapter::within_sync_slop(candidate, 0));
  EXPECT_EQ(candidate.common_stamp_ns, ns(36, 933335259));
  EXPECT_EQ(candidate.rgb.stamp_ns, candidate.depth.stamp_ns);
  EXPECT_EQ(candidate.depth.stamp_ns, candidate.camera_info.stamp_ns);
  EXPECT_EQ(candidate.rgb.frame_id, "camera_color_optical_frame");
  EXPECT_EQ(candidate.rgb.width, 1280);
  EXPECT_EQ(candidate.rgb.height, 720);
  EXPECT_EQ(candidate.rgb.receipt_monotonic_ns, 100);
  EXPECT_EQ(candidate.depth.receipt_monotonic_ns, 101);
  EXPECT_EQ(candidate.camera_info.receipt_monotonic_ns, 102);

  SensorIngressPolicy policy({1'000'000'000, std::nullopt});
  ASSERT_EQ(policy.begin_acquisition(0, 0).status, IngressStatus::kWaiting);
  const auto result = policy.evaluate(candidate, 200);
  EXPECT_EQ(result.status, IngressStatus::kObservationReady);
  ASSERT_TRUE(result.observation.has_value());
  EXPECT_EQ(result.observation->stamp_ns(), ns(36, 933335259));
  EXPECT_FALSE(policy.epoch_invalidated());
}

TEST(VisionRosAdapter, AppliesSlopOnlyToRgbAndDepthNotCachedCameraInfoTimestamp)
{
  auto at_boundary = VisionRosAdapter::to_candidate(
    image(10), image(10, 1'000'000), camera_info(99), {11, 12, 13});
  EXPECT_TRUE(VisionRosAdapter::within_sync_slop(at_boundary, 1'000'000));
  EXPECT_FALSE(VisionRosAdapter::within_sync_slop(at_boundary, 999'999));
}

TEST(VisionRosAdapter, ClockObserverLatchesAfterBackwardClockWithoutObservations)
{
  SensorIngressPolicy ingress({500'000'000, std::nullopt});
  ClockEpochObserver observer(ingress);
  EXPECT_EQ(observer.observe(100).status, IngressStatus::kWaiting);
  EXPECT_EQ(observer.observe(101).status, IngressStatus::kWaiting);
  EXPECT_EQ(observer.observe(50).status, IngressStatus::kTimeRollback);
  EXPECT_TRUE(ingress.epoch_invalidated());
  EXPECT_EQ(observer.observe(103).status, IngressStatus::kWaiting);
  EXPECT_TRUE(ingress.epoch_invalidated());
}

TEST_F(VisionRosIngressTest, SlopParameterAndSubscriberQosAreExplicit)
{
  {
    auto default_ingress = std::make_shared<VisionRosIngress>();
    EXPECT_EQ(default_ingress->sync_slop_ns(), 10'000'000);
  }
  auto ingress = make_ingress(1.0);
  EXPECT_EQ(ingress->sync_slop_ns(), 1'000'000);
  EXPECT_EQ(
    ingress->subscription_qos().get_rmw_qos_profile().reliability,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  EXPECT_EQ(
    ingress->subscription_qos().get_rmw_qos_profile().durability,
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

TEST_F(VisionRosIngressTest, NegativeSlopIsRejected)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("sync_slop_ms", -0.1)});
  EXPECT_THROW(std::make_shared<VisionRosIngress>(options), std::invalid_argument);
}

TEST_F(VisionRosIngressTest, ReusablePairFreshnessWindowIsCapped)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("reusable_pair_max_age_ms", 200.1)});
  EXPECT_THROW(std::make_shared<VisionRosIngress>(options), std::invalid_argument);
}

TEST_F(VisionRosIngressTest, CameraInfoIsCachedIndependentlyOfRgbDepthTimestamp)
{
  auto ingress = make_ingress(1.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_cached_info_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return info->get_subscription_count() >= 1;});
  info->publish(camera_info(1));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);
  rgb->publish(image(10));
  depth->publish(image(10, 500'000));
  rgb->publish(image(11));
  spin_until(executor, [&] {return ingress->last_result().has_value();});

  ASSERT_EQ(ingress->last_result()->status, IngressStatus::kObservationReady);
  EXPECT_EQ(ingress->last_result()->observation->stamp_ns(), ns(10, 500'000));
  EXPECT_EQ(ingress->calibration_cache_generation(), 1U);

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, FreshDepthWithoutRgbIsAnObservation)
{
  auto ingress = make_ingress(1.0, 200.0, true);
  auto publisher = std::make_shared<rclcpp::Node>("vision_depth_only_publisher");
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return info->get_subscription_count() >= 1;});
  ingress->set_observation_observer([&](const VisionObservation & observation) {
      (void)ingress->evaluate_depth_source_stamp_freshness(observation, ns(1));
    });

  info->publish(camera_info(1));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  ASSERT_EQ(ingress->begin_acquisition(0, steady_now_ns()).status, IngressStatus::kWaiting);
  depth->publish(image(10, 500'000));
  spin_until(executor, [&] {return ingress->last_result().has_value();});

  ASSERT_EQ(ingress->last_result()->status, IngressStatus::kObservationReady);
  ASSERT_TRUE(ingress->latest_observation());
  EXPECT_EQ(ingress->latest_observation()->depth.header.stamp.nanosec, 500'000U);
  EXPECT_EQ(ingress->latest_observation()->rgb.width, 0U);
  EXPECT_EQ(ingress->acceptance_snapshot().synchronized_callback_count, 0U);
  const auto diagnostics = ingress->finish_request_diagnostics();
  EXPECT_EQ(diagnostics.depth_source_stamp_freshness_evaluated, 1U);
  ASSERT_EQ(diagnostics.depth_source_freshness_details.size(), 1U);
  EXPECT_EQ(diagnostics.depth_source_freshness_details.front().depth_source_stamp_ns,
    ns(10, 500'000));

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, SnapshotUsesLatestSpatiallyCompatibleRgbWithoutTimestampGate)
{
  auto ingress = make_ingress(10.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_snapshot_latest_rgb_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});

  auto camera = camera_info(1);
  camera.width = 2;
  camera.height = 2;
  camera.header.frame_id = "camera_color_optical_frame";
  auto depth = image(10);
  depth.width = 2;
  depth.height = 2;
  depth.header.frame_id = "camera_color_optical_frame";
  depth.encoding = "32FC1";
  depth.step = 8;
  depth.data.resize(16);

  const auto candidate = [](int32_t sec, uint32_t nanosec, uint8_t pixel) {
      auto value = image(sec, nanosec);
      value.width = 2;
      value.height = 2;
      value.header.frame_id = "camera_color_optical_frame";
      value.encoding = "rgb8";
      value.step = 6;
      value.data.assign(12, pixel);
      return value;
    };
  EXPECT_EQ(ingress->latest_snapshot_rgb(depth, camera).status, "not_received");

  const auto publish_rgb = [&](const sensor_msgs::msg::Image & message) {
      const auto count = ingress->acceptance_snapshot().rgb_message_count;
      rgb->publish(message);
      spin_until(executor, [&] {
          return ingress->acceptance_snapshot().rgb_message_count > count;
        });
    };
  publish_rgb(candidate(30, 0, 7));
  publish_rgb(candidate(40, 0, 8));
  auto selected = ingress->latest_snapshot_rgb(depth, camera);
  ASSERT_TRUE(selected.image);
  EXPECT_EQ(selected.status, "available");
  EXPECT_EQ(selected.image->header.stamp.sec, 40);
  EXPECT_EQ(selected.image->data.front(), 8);
  EXPECT_GT(ns(selected.image->header.stamp.sec, selected.image->header.stamp.nanosec) -
    ns(depth.header.stamp.sec, depth.header.stamp.nanosec), 100'000'000LL);

  auto wrong_camera = candidate(41, 0, 9);
  wrong_camera.header.frame_id = "another_camera";
  publish_rgb(wrong_camera);
  selected = ingress->latest_snapshot_rgb(depth, camera);
  EXPECT_FALSE(selected.image);
  EXPECT_EQ(selected.status, "frame_mismatch");

  auto bad_encoding = candidate(42, 0, 10);
  bad_encoding.encoding = "bgr8";
  publish_rgb(bad_encoding);
  selected = ingress->latest_snapshot_rgb(depth, camera);
  EXPECT_FALSE(selected.image);
  EXPECT_EQ(selected.status, "encoding_unsupported");

  auto short_buffer = candidate(43, 0, 11);
  short_buffer.data.resize(1);
  publish_rgb(short_buffer);
  selected = ingress->latest_snapshot_rgb(depth, camera);
  EXPECT_FALSE(selected.image);
  EXPECT_EQ(selected.status, "buffer_too_short");

  depth.width = 3;
  selected = ingress->latest_snapshot_rgb(depth, camera);
  EXPECT_FALSE(selected.image);
  EXPECT_EQ(selected.status, "depth_camera_info_mismatch");

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, PairWithoutValidCachedCameraInfoIsNotPromoted)
{
  auto ingress = make_ingress(1.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_no_info_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});
  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);
  rgb->publish(image(1));
  depth->publish(image(1));
  rgb->publish(image(2));
  spin_until(executor, [&] {return ingress->last_result().has_value();});

  EXPECT_FALSE(ingress->last_result()->observation.has_value());
  EXPECT_FALSE(ingress->has_cached_calibration());

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST(VisionRosAdapter, CameraCalibrationValidationCoversRequiredFields)
{
  const auto valid = camera_info(1);
  EXPECT_TRUE(VisionRosAdapter::has_valid_calibration(valid));

  auto bad_frame = valid;
  bad_frame.header.frame_id.clear();
  EXPECT_FALSE(VisionRosAdapter::has_valid_calibration(bad_frame));
  auto bad_resolution = valid;
  bad_resolution.width = 0;
  EXPECT_FALSE(VisionRosAdapter::has_valid_calibration(bad_resolution));
  auto bad_k = valid;
  bad_k.k[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(VisionRosAdapter::has_valid_calibration(bad_k));
  auto bad_distortion = valid;
  bad_distortion.d[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(VisionRosAdapter::has_valid_calibration(bad_distortion));
  auto bad_projection = valid;
  bad_projection.p[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(VisionRosAdapter::has_valid_calibration(bad_projection));
}

TEST_F(VisionRosIngressTest, CalibrationChangesAreRevalidatedAndRefreshTheCache)
{
  auto ingress = make_ingress(1.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_calibration_refresh_publisher");
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return info->get_subscription_count() >= 1;});

  auto calibration = camera_info(1);
  info->publish(calibration);
  spin_until(executor, [&] {return ingress->calibration_cache_generation() == 1U;});
  calibration.k[0] += 1.0;
  info->publish(calibration);
  spin_until(executor, [&] {return ingress->calibration_cache_generation() == 2U;});
  calibration.distortion_model = "rational_polynomial";
  info->publish(calibration);
  spin_until(executor, [&] {return ingress->calibration_cache_generation() == 3U;});
  calibration.d[0] += 0.01;
  info->publish(calibration);
  spin_until(executor, [&] {return ingress->calibration_cache_generation() == 4U;});
  calibration.p[0] += 1.0;
  info->publish(calibration);
  spin_until(executor, [&] {return ingress->calibration_cache_generation() == 5U;});
  EXPECT_TRUE(ingress->has_cached_calibration());

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, IncompatibleCachedCalibrationIsRejectedAndInvalidated)
{
  auto ingress = make_ingress(1.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_incompatible_cache_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return info->get_subscription_count() >= 1;});

  auto mismatched = camera_info(1);
  mismatched.header.frame_id = "other_camera_optical_frame";
  info->publish(mismatched);
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);
  rgb->publish(image(2));
  depth->publish(image(2));
  rgb->publish(image(3));
  spin_until(executor, [&] {return ingress->last_result().has_value();});
  EXPECT_EQ(ingress->last_result()->status, IngressStatus::kIncompatibleMetadata);
  EXPECT_FALSE(ingress->has_cached_calibration());

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, ResolutionMismatchedCachedCalibrationIsRejectedAndInvalidated)
{
  auto ingress = make_ingress(1.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_resolution_cache_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return info->get_subscription_count() >= 1;});

  auto mismatched = camera_info(1);
  mismatched.width = 640;
  info->publish(mismatched);
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);
  rgb->publish(image(2));
  depth->publish(image(2));
  rgb->publish(image(3));
  spin_until(executor, [&] {return ingress->last_result().has_value();});
  EXPECT_EQ(ingress->last_result()->status, IngressStatus::kIncompatibleMetadata);
  EXPECT_FALSE(ingress->has_cached_calibration());

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, BestEffortPublisherIsCompatibleWithVisionConsumerPolicy)
{
  auto ingress = make_ingress(1.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_best_effort_publisher");
  const auto qos = rclcpp::SensorDataQoS().keep_last(10).durability_volatile();
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", qos);
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", qos);
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", qos);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});
  info->publish(camera_info(1));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);
  rgb->publish(image(2));
  depth->publish(image(2));
  rgb->publish(image(3));
  spin_until(executor, [&] {return ingress->last_result().has_value();});
  EXPECT_EQ(ingress->last_result()->status, IngressStatus::kObservationReady);

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, ReceiptMetadataIsBoundedAndPruningCannotCreateFalseMatch)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("sync_slop_ms", 0.0),
        rclcpp::Parameter("sync_queue_size", 2),
        rclcpp::Parameter("request_timeout_ms", 5000.0)});
  auto ingress = std::make_shared<VisionRosIngress>(options);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_receipt_bound_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});
  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);

  for (int32_t stamp = 1; stamp <= 10; ++stamp) {
    rgb->publish(image(stamp));
  }
  spin_for(executor, std::chrono::milliseconds(100));
  const auto sizes = ingress->receipt_metadata_sizes();
  EXPECT_LE(sizes[0], 2U);
  EXPECT_LE(sizes[1], 2U);

  depth->publish(image(1));
  info->publish(camera_info(1));
  spin_for(executor, std::chrono::milliseconds(50));
  EXPECT_TRUE(ingress->synchronized_callback_rgb_stamps().empty());
  EXPECT_FALSE(ingress->acceptance_snapshot().epoch_invalidated);

  depth->publish(image(10));
  info->publish(camera_info(10));
  spin_until(executor, [&] {return !ingress->synchronized_callback_rgb_stamps().empty();});
  ASSERT_TRUE(ingress->last_result().has_value());
  EXPECT_EQ(ingress->last_result()->status, IngressStatus::kObservationReady);
  EXPECT_EQ(ingress->last_result()->observation->stamp_ns(), ns(10));
  EXPECT_FALSE(ingress->acceptance_snapshot().epoch_invalidated);

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, CallbackHistoryRemainsBoundedDuringLongSynchronizedOperation)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        rclcpp::Parameter("sync_slop_ms", 0.0),
        rclcpp::Parameter("sync_queue_size", 2)});
  auto ingress = std::make_shared<VisionRosIngress>(options);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_history_bound_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});
  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);

  for (int32_t stamp = 1; stamp <= 6; ++stamp) {
    rgb->publish(image(stamp));
    depth->publish(image(stamp));
    info->publish(camera_info(stamp));
    spin_for(executor, std::chrono::milliseconds(20));
  }
  spin_until(
    executor,
    [&] {return ingress->acceptance_snapshot().synchronized_callback_count >= 6;});
  EXPECT_EQ(ingress->synchronized_callback_rgb_stamps().size(), 2U);
  EXPECT_FALSE(ingress->acceptance_snapshot().epoch_invalidated);

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, ClockRollbackInvalidatesWithoutAnySynchronizedObservation)
{
  auto ingress = make_ingress(0.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_clock_publisher");
  auto clock = publisher->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_for(executor, std::chrono::milliseconds(100));

  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);
  for (const auto stamp : {100, 101, 50}) {
    rosgraph_msgs::msg::Clock message;
    message.clock.sec = stamp;
    clock->publish(message);
    spin_for(executor, std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(ingress->last_result().has_value());
  EXPECT_EQ(ingress->last_result()->status, IngressStatus::kTimeRollback);
  EXPECT_EQ(ingress->acceptance_snapshot().clock_epoch, 1U);

  rosgraph_msgs::msg::Clock later;
  later.clock.sec = 103;
  clock->publish(later);
  spin_for(executor, std::chrono::milliseconds(10));
  EXPECT_EQ(ingress->check_deadline().status, IngressStatus::kEpochInvalidated);

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, ExactAndStrictlyInsideSlopSynchronize)
{
  auto ingress = make_ingress(1.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_test_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_for(executor, std::chrono::milliseconds(100));

  info->publish(camera_info(1, 500'000));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);
  rgb->publish(image(1));
  depth->publish(image(1, 500'000));
  rgb->publish(image(2));
  spin_until(executor, [&] {return !ingress->synchronized_callback_rgb_stamps().empty();});
  ASSERT_EQ(ingress->synchronized_callback_rgb_stamps().front(), ns(1));
  ASSERT_TRUE(ingress->last_result().has_value());
  ASSERT_EQ(ingress->last_result()->status, IngressStatus::kObservationReady);
  ASSERT_TRUE(ingress->last_result()->observation);
  EXPECT_EQ(ingress->last_result()->observation->stamp_ns(), ns(1, 500'000));

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, EqualityAtSlopIsCharacterizedAfterLaterProgression)
{
  auto ingress = make_ingress(1.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_boundary_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});

  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);
  rgb->publish(image(1));
  depth->publish(image(1, 1'000'000));
  info->publish(camera_info(1, 1'000'000));
  spin_for(executor, std::chrono::milliseconds(100));
  EXPECT_TRUE(ingress->synchronized_callback_rgb_stamps().empty());

  rgb->publish(image(2));
  depth->publish(image(2));
  info->publish(camera_info(2));
  spin_until(executor, [&] {return !ingress->synchronized_callback_rgb_stamps().empty();});
  EXPECT_EQ(ingress->synchronized_callback_rgb_stamps().front(), ns(1));

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, OutsideSlopDoesNotReachThePolicy)
{
  auto ingress = make_ingress(1.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_outside_slop_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_for(executor, std::chrono::milliseconds(100));

  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);
  rgb->publish(image(1));
  depth->publish(image(1, 1'000'001));
  info->publish(camera_info(1, 1'000'001));
  rgb->publish(image(2));
  depth->publish(image(2));
  info->publish(camera_info(2));
  spin_until(executor, [&] {return !ingress->synchronized_callback_rgb_stamps().empty();});
  EXPECT_EQ(ingress->synchronized_callback_rgb_stamps().front(), ns(2));

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, CrossStreamInterleavingProducesT1WithoutRollback)
{
  auto ingress = make_ingress(0.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_reorder_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_for(executor, std::chrono::milliseconds(100));

  ASSERT_EQ(ingress->begin_acquisition(0).status, IngressStatus::kWaiting);
  info->publish(camera_info(1));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  rgb->publish(image(1));
  rgb->publish(image(2));
  depth->publish(image(1));
  spin_until(executor, [&] {return ingress->last_result().has_value();});
  ASSERT_EQ(ingress->last_result()->status, IngressStatus::kObservationReady);
  EXPECT_EQ(ingress->last_result()->observation->stamp_ns(), ns(1));

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, RequestDiagnosticsSeparateLatestSamplesFromActualPairsAndReset)
{
  rcutils_logging_set_logger_level("vision_ros_ingress", RCUTILS_LOG_SEVERITY_DEBUG);
  auto ingress = make_ingress(0.0, 200.0, true);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_request_diagnostics_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});

  info->publish(camera_info(100));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  rgb->publish(image(100));
  spin_until(executor, [&] {return ingress->acceptance_snapshot().rgb_message_count == 1;});
  const auto request_start = steady_now_ns();
  ASSERT_EQ(ingress->begin_acquisition(0, request_start).status, IngressStatus::kWaiting);

  depth->publish(image(100));
  spin_until(
    executor,
    [&] {return ingress->acceptance_snapshot().synchronized_callback_count == 1;});
  rgb->publish(image(101));
  depth->publish(image(101));
  spin_until(
    executor,
    [&] {return ingress->acceptance_snapshot().synchronized_callback_count == 2;});

  auto diagnostics = ingress->finish_request_diagnostics();
  EXPECT_FALSE(diagnostics.active);
  EXPECT_EQ(diagnostics.request_start_monotonic_ns, request_start);
  EXPECT_EQ(diagnostics.request_watermark_stamp_ns, 0);
  EXPECT_EQ(diagnostics.rgb_received, 1U);
  EXPECT_EQ(diagnostics.depth_received, 2U);
  EXPECT_EQ(diagnostics.sync_callbacks, 2U);
  EXPECT_EQ(diagnostics.policy_evaluations, 2U);
  EXPECT_EQ(diagnostics.observation_ready, 1U);
  EXPECT_EQ(diagnostics.watermark_rejected_pairs, 0U);
  EXPECT_EQ(diagnostics.latest_stamp_delta_ns, 0);
  EXPECT_EQ(diagnostics.minimum_cross_stream_delta_ns, 0);
  EXPECT_EQ(diagnostics.latest_pair_rgb_stamp_ns, ns(101));
  EXPECT_EQ(diagnostics.latest_pair_depth_stamp_ns, ns(101));
  EXPECT_EQ(diagnostics.latest_pair_delta_ns, 0);
  rgb->publish(image(102));
  spin_until(executor, [&] {return ingress->acceptance_snapshot().rgb_message_count == 3;});
  const auto ingress_snapshot = ingress->acceptance_snapshot();
  EXPECT_EQ(ingress_snapshot.rgb_message_count, 3U);
  EXPECT_EQ(ingress_snapshot.depth_message_count, 2U);
  EXPECT_EQ(ingress_snapshot.synchronized_callback_count, 2U);
  // The pair stamps remain bound to the last synchronized callback even when
  // the latest independently received RGB sample has moved ahead.
  EXPECT_EQ(ingress_snapshot.latest_rgb_stamp_ns, ns(101));
  EXPECT_EQ(ingress_snapshot.latest_depth_stamp_ns, ns(101));
  EXPECT_EQ(ingress_snapshot.latest_rgb_received_stamp_ns, ns(102));
  ASSERT_TRUE(ingress_snapshot.last_synchronized_receipt_monotonic_ns);
  EXPECT_GE(steady_now_ns(), *ingress_snapshot.last_synchronized_receipt_monotonic_ns);
  ASSERT_TRUE(ingress_snapshot.latest_oldest_receipt_age_ns);
  EXPECT_GE(*ingress_snapshot.latest_oldest_receipt_age_ns, 0);
  ASSERT_EQ(diagnostics.pair_details.size(), 2U);
  EXPECT_TRUE(diagnostics.pair_details[0].pre_request_sample);
  EXPECT_EQ(diagnostics.pair_details[0].status, IngressStatus::kWaiting);
  EXPECT_EQ(diagnostics.pair_details[0].rgb_frame_id, "camera_color_optical_frame");
  EXPECT_EQ(diagnostics.pair_details[0].depth_frame_id, "camera_color_optical_frame");
  EXPECT_FALSE(diagnostics.pair_details[0].policy_evaluated.value());
  EXPECT_FALSE(diagnostics.pair_details[0].observation_ready.value());
  EXPECT_FALSE(diagnostics.pair_details[1].pre_request_sample);
  EXPECT_TRUE(diagnostics.pair_details[1].watermark_passed);
  EXPECT_EQ(diagnostics.pair_details[1].status, IngressStatus::kWaiting);
  EXPECT_FALSE(diagnostics.pair_details[1].policy_evaluated.value());
  EXPECT_FALSE(diagnostics.pair_details[1].observation_ready.value());
  ASSERT_EQ(diagnostics.sample_details.size(), 3U);
  EXPECT_TRUE(diagnostics.sample_details[0].receipt_monotonic_ns > request_start);
  EXPECT_EQ(diagnostics.sample_details[0].frame_id, "camera_color_optical_frame");
  EXPECT_TRUE(diagnostics.sample_details[1].receipt_monotonic_ns > request_start);
  EXPECT_TRUE(diagnostics.depth_stamp_period_mean_ns.has_value());

  const auto next_start = steady_now_ns();
  ASSERT_EQ(ingress->begin_acquisition(ns(101), next_start).status, IngressStatus::kWaiting);
  const auto reset = ingress->finish_request_diagnostics();
  EXPECT_GT(reset.request_id, diagnostics.request_id);
  EXPECT_EQ(reset.rgb_received, 0U);
  EXPECT_EQ(reset.depth_received, 0U);
  EXPECT_EQ(reset.sync_callbacks, 0U);
  EXPECT_EQ(reset.policy_evaluations, 0U);
  EXPECT_EQ(reset.observation_ready, 0U);
  EXPECT_TRUE(reset.sample_details.empty());
  EXPECT_TRUE(reset.pair_details.empty());

  executor.remove_node(publisher);
  executor.remove_node(ingress);
  rcutils_logging_set_logger_level("vision_ros_ingress", RCUTILS_LOG_SEVERITY_INFO);
}

TEST_F(VisionRosIngressTest, DetailedCapturePreservesSignedPairFreshnessEvidence)
{
  auto ingress = make_ingress(10.0, 200.0, true);
  auto publisher = std::make_shared<rclcpp::Node>("vision_pair_freshness_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  auto clock = publisher->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});

  std::vector<VisionIngressPairFreshnessDiagnostic> evaluations;
  ingress->set_synchronized_observer([&](const VisionObservation & observation) {
      evaluations.push_back(ingress->evaluate_pair_source_stamp_freshness(
          observation, ns(1)));
    });
  rosgraph_msgs::msg::Clock clock_message;
  clock_message.clock.sec = 100;
  clock->publish(clock_message);
  spin_until(executor, [&] {return ingress->acceptance_snapshot().clock_sample_count == 1;});
  info->publish(camera_info(100));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  ASSERT_EQ(ingress->begin_acquisition(0, steady_now_ns()).status, IngressStatus::kWaiting);

  const auto publish_pair = [&](int32_t sec, uint32_t nanosec, uint64_t expected_count) {
      rgb->publish(image(sec, nanosec));
      depth->publish(image(sec, nanosec));
      spin_until(executor, [&] {
        return ingress->acceptance_snapshot().synchronized_callback_count >= expected_count;
      });
    };
  publish_pair(98, 900'000'000U, 1U);
  publish_pair(100, 500'000'000U, 2U);
  publish_pair(101, 100'000'000U, 3U);

  ASSERT_EQ(evaluations.size(), 3U);
  EXPECT_FALSE(evaluations[0].accepted);
  EXPECT_EQ(evaluations[0].rgb_source_minus_clock_ns, -1'100'000'000LL);
  EXPECT_EQ(evaluations[0].depth_source_minus_clock_ns, -1'100'000'000LL);
  EXPECT_EQ(evaluations[0].reason, "rgb_source_clock_delta_exceeded");
  ASSERT_TRUE(evaluations[0].clock_receipt_age_ns.has_value());
  EXPECT_EQ(
    *evaluations[0].clock_receipt_age_ns,
    evaluations[0].evaluation_steady_ns - *evaluations[0].clock_received_steady_ns);
  EXPECT_TRUE(evaluations[1].accepted);
  EXPECT_EQ(evaluations[1].rgb_source_minus_clock_ns, 500'000'000LL);
  EXPECT_FALSE(evaluations[2].accepted);
  EXPECT_EQ(evaluations[2].rgb_source_minus_clock_ns, 1'100'000'000LL);
  EXPECT_EQ(evaluations[2].depth_source_minus_clock_ns, 1'100'000'000LL);
  EXPECT_EQ(evaluations[2].reason, "rgb_source_clock_delta_exceeded");

  const auto diagnostics = ingress->finish_request_diagnostics();
  ASSERT_EQ(diagnostics.pair_details.size(), 3U);
  ASSERT_TRUE(diagnostics.pair_details[0].freshness_evaluation.has_value());
  EXPECT_EQ(
    diagnostics.pair_details[0].freshness_evaluation->rgb_source_minus_clock_ns,
    -1'100'000'000LL);
  EXPECT_EQ(
    diagnostics.pair_details[0].freshness_evaluation->maximum_absolute_delta_ns,
    1'000'000'000LL);
  EXPECT_FALSE(diagnostics.pair_details[0].freshness_evaluation->accepted);
  EXPECT_EQ(diagnostics.source_stamp_freshness_evaluated, 3U);
  EXPECT_EQ(diagnostics.source_stamp_freshness_accepted, 1U);
  EXPECT_EQ(diagnostics.source_stamp_freshness_rejected, 2U);
  EXPECT_EQ(diagnostics.source_stamp_freshness_clock_delta_rejected, 2U);

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, MissingClockIsReportedWithoutInventingTimeValues)
{
  auto ingress = make_ingress(10.0, 200.0, true);
  auto publisher = std::make_shared<rclcpp::Node>("vision_clock_relation_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});
  std::optional<VisionIngressPairFreshnessDiagnostic> no_clock;
  ingress->set_synchronized_observer([&](const VisionObservation & observation) {
      no_clock = ingress->evaluate_pair_source_stamp_freshness(observation, ns(1));
    });
  info->publish(camera_info(5));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  ASSERT_EQ(ingress->begin_acquisition(0, steady_now_ns()).status, IngressStatus::kWaiting);
  rgb->publish(image(5));
  depth->publish(image(5));
  spin_until(executor, [&] {return no_clock.has_value();});
  ASSERT_TRUE(no_clock->accepted);
  EXPECT_FALSE(no_clock->evaluation_clock_ns.has_value());
  EXPECT_FALSE(no_clock->rgb_source_minus_clock_ns.has_value());
  EXPECT_FALSE(no_clock->clock_receipt_age_ns.has_value());
  EXPECT_EQ(no_clock->clock_sample_relation, VisionClockSampleRelation::kNotReceived);
  EXPECT_EQ(no_clock->reason, "clock_not_received_gate_not_applied");
  const auto no_clock_diagnostics = ingress->finish_request_diagnostics();
  ASSERT_EQ(no_clock_diagnostics.pair_details.size(), 1U);
  ASSERT_TRUE(no_clock_diagnostics.pair_details.front().freshness_evaluation.has_value());
  EXPECT_EQ(no_clock_diagnostics.source_stamp_freshness_clock_unavailable, 1U);

  ingress->begin_acquisition(0, steady_now_ns());
  auto clock = publisher->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS());
  spin_until(executor, [&] {return clock->get_subscription_count() >= 1;});
  const auto publish_clock = [&](int32_t sec, uint64_t count) {
      rosgraph_msgs::msg::Clock message;
      message.clock.sec = sec;
      clock->publish(message);
      spin_until(executor, [&] {return ingress->acceptance_snapshot().clock_sample_count >= count;});
    };
  publish_clock(0, 1U);
  auto first_sample = ingress->evaluate_pair_source_stamp_freshness(
    VisionObservation{image(0), image(0), camera_info(0), 1, 2}, ns(1));
  EXPECT_EQ(first_sample.clock_sample_relation, VisionClockSampleRelation::kFirstSample);
  ASSERT_TRUE(first_sample.evaluation_clock_ns.has_value());
  EXPECT_EQ(*first_sample.evaluation_clock_ns, 0);
  publish_clock(10, 2U);
  const auto advanced_sample = ingress->evaluate_pair_source_stamp_freshness(
    VisionObservation{image(10), image(10), camera_info(10), 1, 2}, ns(1));
  EXPECT_EQ(advanced_sample.clock_sample_relation, VisionClockSampleRelation::kAdvanced);
  publish_clock(10, 3U);
  auto unchanged_sample = ingress->evaluate_pair_source_stamp_freshness(
    VisionObservation{image(10), image(10), camera_info(10), 1, 2}, ns(1));
  EXPECT_EQ(unchanged_sample.clock_sample_relation, VisionClockSampleRelation::kUnchanged);
  publish_clock(9, 4U);
  const auto rollback_sample = ingress->evaluate_pair_source_stamp_freshness(
    VisionObservation{image(9), image(9), camera_info(9), 1, 2}, ns(1));
  EXPECT_EQ(rollback_sample.clock_sample_relation, VisionClockSampleRelation::kBackward);
  EXPECT_TRUE(rollback_sample.clock_rollback_observed);
  EXPECT_EQ(rollback_sample.clock_epoch, 1U);

  const auto diagnostics = ingress->finish_request_diagnostics();
  EXPECT_EQ(diagnostics.pair_details.size(), 0U);
  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, DiagnosticsOffDoesNotRetainFreshnessEvidence)
{
  auto ingress = make_ingress(10.0, 200.0, false);
  auto publisher = std::make_shared<rclcpp::Node>("vision_freshness_capture_off_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});
  ingress->set_observation_observer([&](const VisionObservation & observation) {
      ingress->evaluate_pair_source_stamp_freshness(observation, ns(1));
    });
  info->publish(camera_info(5));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  ASSERT_EQ(ingress->begin_acquisition(0, steady_now_ns()).status, IngressStatus::kWaiting);
  rgb->publish(image(5));
  depth->publish(image(5));
  spin_until(executor, [&] {
    return ingress->acceptance_snapshot().policy_delivery_count == 1U;
  });
  const auto diagnostics = ingress->finish_request_diagnostics();
  EXPECT_FALSE(diagnostics.detailed_capture_enabled);
  EXPECT_TRUE(diagnostics.pair_details.empty());
  EXPECT_EQ(diagnostics.source_stamp_freshness_evaluated, 0U);
  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, DetailedFreshnessExamplesStayBoundedAndAggregateRejects)
{
  auto ingress = make_ingress(0.0, 200.0, true);
  auto publisher = std::make_shared<rclcpp::Node>("vision_freshness_bound_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  auto clock = publisher->create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::ClockQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});
  ingress->set_observation_observer([&](const VisionObservation & observation) {
      ingress->evaluate_pair_source_stamp_freshness(observation, ns(1));
    });
  rosgraph_msgs::msg::Clock clock_message;
  clock_message.clock.sec = 100;
  clock->publish(clock_message);
  spin_until(executor, [&] {return ingress->acceptance_snapshot().clock_sample_count == 1;});
  info->publish(camera_info(100));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  ASSERT_EQ(ingress->begin_acquisition(0, steady_now_ns()).status, IngressStatus::kWaiting);

  for (int32_t index = 1; index <= 40; ++index) {
    rgb->publish(image(98, static_cast<uint32_t>(index)));
    depth->publish(image(98, static_cast<uint32_t>(index)));
    spin_until(executor, [&] {
      return ingress->acceptance_snapshot().policy_delivery_count >=
        static_cast<uint64_t>(index);
    });
  }

  const auto diagnostics = ingress->finish_request_diagnostics();
  EXPECT_EQ(diagnostics.source_stamp_freshness_rejected, 40U);
  EXPECT_EQ(diagnostics.source_stamp_freshness_clock_delta_rejected, 40U);
  EXPECT_EQ(diagnostics.pair_details.size(), 32U);
  EXPECT_EQ(diagnostics.pair_detail_dropped, 8U);
  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, DetailedCaptureDropsOversizedFrameIdentityWithinTheBound)
{
  rcutils_logging_set_logger_level("vision_ros_ingress", RCUTILS_LOG_SEVERITY_DEBUG);
  auto ingress = make_ingress(1.0, 200.0, true);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_long_frame_id_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});
  ASSERT_EQ(ingress->begin_acquisition(0, steady_now_ns()).status, IngressStatus::kWaiting);
  auto message = image(1);
  message.header.frame_id = std::string(129, 'f');
  rgb->publish(message);
  spin_until(executor, [&] {return ingress->acceptance_snapshot().rgb_message_count == 1;});
  const auto details = ingress->finish_request_diagnostics();
  EXPECT_TRUE(details.sample_details.empty());
  EXPECT_EQ(details.sample_detail_dropped, 1U);
  EXPECT_EQ(details.rgb_received, 1U);
  executor.remove_node(publisher);
  executor.remove_node(ingress);
  rcutils_logging_set_logger_level("vision_ros_ingress", RCUTILS_LOG_SEVERITY_INFO);
}

TEST_F(VisionRosIngressTest, FreshPreRequestPairIsReusedAsNewDetectionInput)
{
  auto ingress = make_ingress(10.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_cached_pair_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});

  info->publish(camera_info(10));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  auto color = image(10);
  color.encoding = "rgb8";
  color.step = color.width * 3;
  color.data.assign(static_cast<size_t>(color.step) * color.height, 20);
  rgb->publish(color);
  depth->publish(image(10));
  spin_until(executor, [&] {
    return ingress->acceptance_snapshot().synchronized_callback_count == 1;
  });

  ASSERT_EQ(ingress->begin_acquisition(ns(10), steady_now_ns()).status, IngressStatus::kWaiting);
  const auto observation = ingress->latest_observation();
  ASSERT_TRUE(observation);
  EXPECT_EQ(observation->rgb_presentation_status, "deferred_to_snapshot");
  EXPECT_EQ(observation->rgb.width, 0U);
  EXPECT_EQ(observation->depth.header.stamp.sec, 10);
  EXPECT_TRUE(ingress->acceptance_snapshot().cached_pair_reused);
  const auto diagnostics = ingress->finish_request_diagnostics();
  EXPECT_TRUE(diagnostics.cached_pair_available);
  EXPECT_TRUE(diagnostics.cached_pair_reused);
  EXPECT_EQ(diagnostics.cached_pair_status, IngressStatus::kObservationReady);

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, CachedPairOlderThanConfiguredFreshnessWindowIsNotReused)
{
  auto ingress = make_ingress(10.0, 5.0);
  auto publisher = std::make_shared<rclcpp::Node>("vision_ingress_expired_pair_publisher");
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::QoS(10).reliable().durability_volatile());
  auto info = publisher->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::QoS(10).reliable().durability_volatile());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ingress);
  executor.add_node(publisher);
  spin_until(executor, [&] {return rgb->get_subscription_count() >= 1;});

  info->publish(camera_info(20));
  spin_until(executor, [&] {return ingress->has_cached_calibration();});
  rgb->publish(image(20));
  depth->publish(image(20));
  spin_until(executor, [&] {
    return ingress->acceptance_snapshot().synchronized_callback_count == 1;
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  ASSERT_EQ(ingress->begin_acquisition(ns(20), steady_now_ns()).status, IngressStatus::kWaiting);
  EXPECT_FALSE(ingress->latest_observation());
  EXPECT_FALSE(ingress->acceptance_snapshot().cached_pair_reused);
  const auto diagnostics = ingress->finish_request_diagnostics();
  EXPECT_TRUE(diagnostics.cached_pair_available);
  EXPECT_EQ(diagnostics.cached_pair_status, IngressStatus::kReceiptStaleObservation);

  executor.remove_node(publisher);
  executor.remove_node(ingress);
}

TEST_F(VisionRosIngressTest, DepthFreshnessCauseRemainsAvailableWithoutDetailedCapture)
{
  auto ingress = make_ingress(10.0, 200.0, false);
  VisionObservation observation;
  observation.depth = image(5);
  const auto evaluation = ingress->evaluate_depth_source_stamp_freshness(observation, ns(1));
  EXPECT_TRUE(evaluation.accepted);
  EXPECT_EQ(evaluation.reason, "clock_not_received_gate_not_applied");
  const auto diagnostics = ingress->finish_request_diagnostics();
  EXPECT_TRUE(diagnostics.depth_source_freshness_details.empty());
  EXPECT_EQ(diagnostics.depth_source_stamp_freshness_evaluated, 0U);
}

}  // namespace
}  // namespace arm_cell_vision
