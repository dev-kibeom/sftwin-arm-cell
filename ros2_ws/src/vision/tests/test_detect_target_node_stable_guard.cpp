#include <chrono>
#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <arm_cell_interfaces/msg/vision_calibration.hpp>
#include <arm_cell_interfaces/msg/scene_stability.hpp>
#include <arm_cell_interfaces/msg/vision_diagnostic.hpp>
#include <arm_cell_interfaces/srv/detect_target.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "arm_cell_vision/calibration_file.hpp"
#include "detect_target_node_factory.hpp"

namespace
{
using namespace std::chrono_literals;
using CalibrationState = arm_cell_interfaces::msg::VisionCalibration;

sensor_msgs::msg::CameraInfo camera_info()
{
  sensor_msgs::msg::CameraInfo message;
  message.header.frame_id = "camera_color_optical_frame";
  message.width = 320;
  message.height = 300;
  message.distortion_model = "plumb_bob";
  message.k = {1000.0, 0.0, 160.0, 0.0, 1000.0, 150.0, 0.0, 0.0, 1.0};
  message.d.assign(5, 0.0);
  message.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  message.p = {1000.0, 0.0, 160.0, 0.0, 1000.0, 150.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  return message;
}

sensor_msgs::msg::Image image(
  uint32_t sec, bool depth, bool with_cuboid = false, bool millimeter_depth = false,
  const std::string & frame_id = "camera_color_optical_frame")
{
  sensor_msgs::msg::Image message;
  message.header.stamp.sec = static_cast<int32_t>(sec);
  message.header.frame_id = frame_id;
  message.width = 320;
  message.height = 300;
  message.encoding = depth ? (millimeter_depth ? "16UC1" : "32FC1") : "rgb8";
  const size_t depth_bytes = millimeter_depth ? sizeof(uint16_t) : sizeof(float);
  message.step = static_cast<uint32_t>(message.width * (depth ? depth_bytes : 3U));
  message.data.resize(static_cast<size_t>(message.step) * message.height);
  if (depth && with_cuboid) {
    const float top_depth_m = 0.92F;
    const uint16_t top_depth_mm = 920;
    for (uint32_t row = 112; row < 188; ++row) {
      for (uint32_t column = 122; column < 198; ++column) {
        const auto offset = static_cast<size_t>(row) * message.step +
          static_cast<size_t>(column) * depth_bytes;
        const void * pixel = millimeter_depth ?
          static_cast<const void *>(&top_depth_mm) : static_cast<const void *>(&top_depth_m);
        std::memcpy(message.data.data() + offset, pixel, depth_bytes);
      }
    }
  }
  return message;
}

class DetectTargetNodeRosBoundaryTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    start_node("camera_color_optical_frame", true);
  }

  void start_node(const std::string & camera_frame, bool publish_camera_transform)
  {
    camera_frame_ = camera_frame;
    root_ = std::filesystem::temp_directory_path() /
      ("vision-node-calibration-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root_);
    selected_calibration_ = root_ / "selected.yaml";

    arm_cell_vision::CalibrationFileData calibration;
    calibration.tray_from_camera.header.frame_id = "Tray";
    calibration.tray_from_camera.child_frame_id = camera_frame_;
    calibration.tray_from_camera.transform.rotation.x = 1.0;
    calibration.tray_from_camera.transform.rotation.w = 0.0;
    calibration.tray_from_camera.transform.translation.z = 1.0;
    calibration.camera_info = camera_info();
    calibration.camera_info.header.frame_id = camera_frame_;
    calibration.camera_info_topic = "/camera/color/camera_info";
    std::string created;
    std::string diagnostic;
    ASSERT_TRUE(arm_cell_vision::CalibrationFile::save_new_sibling(
        selected_calibration_.string(), calibration, created, diagnostic)) << diagnostic;
    std::filesystem::rename(created, selected_calibration_);

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("calibration_file", selected_calibration_.string()),
      rclcpp::Parameter("camera_info_topic", "/camera/color/camera_info"),
      rclcpp::Parameter("sync_slop_ms", 10.0),
      rclcpp::Parameter("detector_profiles.RawPart.target_id", std::string("RawPart")),
      rclcpp::Parameter("detector_profiles.RawPart.material_id", std::string("RawPart")),
      rclcpp::Parameter("material_definitions.RawPart.shape", std::string("box")),
      rclcpp::Parameter("material_definitions.RawPart.nominal_dimensions_m",
        std::vector<double>{0.07, 0.07, 0.08}),
      rclcpp::Parameter("detector_profiles.RawPart.dimension_tolerance_m",
        std::vector<double>{0.01, 0.01, 0.01}),
      rclcpp::Parameter("detector_profiles.RawPart.workspace_roi_m.x",
        std::vector<double>{-0.3, 0.3}),
      rclcpp::Parameter("detector_profiles.RawPart.workspace_roi_m.y",
        std::vector<double>{-0.2, 0.2}),
      rclcpp::Parameter("detector_profiles.RawPart.workspace_roi_m.z",
        std::vector<double>{0.01, 0.12}),
      rclcpp::Parameter("detector_profiles.RawPart.tray_roi_m.x",
        std::vector<double>{-0.3, 0.3}),
      rclcpp::Parameter("detector_profiles.RawPart.tray_roi_m.y",
        std::vector<double>{-0.2, 0.2}),
      rclcpp::Parameter("detector_profiles.RawPart.support_contact_tolerance_m", 0.01),
      rclcpp::Parameter("detector_profiles.RawPart.top_surface_geometry_tolerance_m", 0.01),
      rclcpp::Parameter("detector_profiles.RawPart.fiducial.dictionary", std::string("DICT_4X4_50")),
      rclcpp::Parameter("detector_profiles.RawPart.fiducial.marker_id", int64_t{0}),
      rclcpp::Parameter("detector_profiles.RawPart.fiducial.size_m", 0.04),
      rclcpp::Parameter("detector_profiles.RawPart.fiducial.pose_translation_scale", 1.0011),
      rclcpp::Parameter("detector_profiles.RawPart.fiducial.marker_in_support.position_m",
        std::vector<double>{0.0, -0.2525, 0.1101}),
      rclcpp::Parameter("detector_profiles.RawPart.fiducial.marker_in_support.rpy_rad",
        std::vector<double>{0.0, 0.0, 0.0})});

    vision_ = arm_cell_vision::make_detect_target_node(options);
    driver_ = std::make_shared<rclcpp::Node>("vision_node_ros_boundary_test_driver");
    auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    calibration_subscription_ = driver_->create_subscription<CalibrationState>(
      "/vision/calibration_status", transient_qos,
      [this](CalibrationState::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        states_.push_back(message->state);
        condition_.notify_all();
      });
    scene_subscription_ = driver_->create_subscription<arm_cell_interfaces::msg::SceneStability>(
      "/vision/scene_stability", rclcpp::QoS(10).reliable(),
      [this](arm_cell_interfaces::msg::SceneStability::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        processed_scenes_.emplace_back(message->header.stamp.sec, message->status);
        condition_.notify_all();
      });
    diagnostic_subscription_ = driver_->create_subscription<arm_cell_interfaces::msg::VisionDiagnostic>(
      "/vision/diagnostics", rclcpp::QoS(1).reliable().transient_local(),
      [this](arm_cell_interfaces::msg::VisionDiagnostic::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        detecting_seen_ = detecting_seen_ ||
          message->state == arm_cell_interfaces::msg::VisionDiagnostic::DETECTING;
        condition_.notify_all();
      });
    rgb_publisher_ = driver_->create_publisher<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", rclcpp::SensorDataQoS());
    depth_publisher_ = driver_->create_publisher<sensor_msgs::msg::Image>(
      "/camera/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS());
    info_publisher_ = driver_->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", rclcpp::SensorDataQoS());
    detect_client_ = driver_->create_client<arm_cell_interfaces::srv::DetectTarget>(
      "/vision/detect_target");
    calibration_client_ = driver_->create_client<std_srvs::srv::Trigger>(
      "/vision/calibrate");
    if (publish_camera_transform) {
      transform_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(driver_);
      geometry_msgs::msg::TransformStamped camera_transform;
      camera_transform.header.frame_id = "base_link";
      camera_transform.child_frame_id = camera_frame_;
      camera_transform.transform.rotation.w = 1.0;
      transform_broadcaster_->sendTransform(camera_transform);
    }

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), 2);
    executor_->add_node(vision_);
    executor_->add_node(driver_);
    spin_thread_ = std::thread([this]() {executor_->spin();});
    ASSERT_TRUE(wait_for_camera_subscriptions());
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
      if (spin_thread_.joinable()) {spin_thread_.join();}
      if (driver_) {executor_->remove_node(driver_);}
      if (vision_) {executor_->remove_node(vision_);}
    }
    calibration_subscription_.reset();
    scene_subscription_.reset();
    diagnostic_subscription_.reset();
    vision_.reset();
    driver_.reset();
    executor_.reset();
    std::filesystem::remove_all(root_);
  }

  bool wait_for_state(uint8_t state, size_t required_occurrences = 1)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [&] {
      return static_cast<size_t>(std::count(states_.begin(), states_.end(), state)) >=
             required_occurrences;
    });
  }

  bool publish_until_calibration_state(
    uint8_t expected, uint32_t first_stamp, const sensor_msgs::msg::CameraInfo & info,
    size_t required_occurrences = 1)
  {
    for (uint32_t attempt = 0; attempt < 10; ++attempt) {
      publish_observation_pair(first_stamp + attempt, info);
      std::unique_lock<std::mutex> lock(mutex_);
      if (condition_.wait_for(lock, 100ms, [&] {
          return static_cast<size_t>(std::count(states_.begin(), states_.end(), expected)) >=
                 required_occurrences;
        }))
      {
        return true;
      }
    }
    return false;
  }

  bool wait_for_camera_subscriptions()
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    std::unique_lock<std::mutex> lock(mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      if (rgb_publisher_->get_subscription_count() > 0 &&
        depth_publisher_->get_subscription_count() > 0 &&
        info_publisher_->get_subscription_count() > 0)
      {
        return true;
      }
      condition_.wait_for(lock, 10ms);
    }
    return false;
  }

  bool wait_for_scene_stamp(uint32_t stamp_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [&] {
      return std::any_of(processed_scenes_.begin(), processed_scenes_.end(),
        [stamp_sec](const auto & item) {return item.first == stamp_sec;});
    });
  }

  bool wait_for_detecting()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this]() {return detecting_seen_;});
  }

  void publish_observation_pair(
    uint32_t stamp_sec, sensor_msgs::msg::CameraInfo info, bool with_cuboid = false,
    bool millimeter_depth = false)
  {
    info.header.stamp.sec = static_cast<int32_t>(stamp_sec);
    info.header.frame_id = camera_frame_;
    info_publisher_->publish(info);
    rgb_publisher_->publish(image(stamp_sec, false, false, false, camera_frame_));
    depth_publisher_->publish(
      image(stamp_sec, true, with_cuboid, millimeter_depth, camera_frame_));
  }

  bool wait_for_tray_depth()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this]() {return !tray_depth_messages_.empty();});
  }

  std::filesystem::path root_;
  std::filesystem::path selected_calibration_;
  std::string camera_frame_;
  rclcpp::Node::SharedPtr vision_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::Subscription<CalibrationState>::SharedPtr calibration_subscription_;
  rclcpp::Subscription<arm_cell_interfaces::msg::SceneStability>::SharedPtr scene_subscription_;
  rclcpp::Subscription<arm_cell_interfaces::msg::VisionDiagnostic>::SharedPtr diagnostic_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
  rclcpp::Client<arm_cell_interfaces::srv::DetectTarget>::SharedPtr detect_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibration_client_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> transform_broadcaster_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<uint8_t> states_;
  std::vector<std::pair<uint32_t, uint8_t>> processed_scenes_;
  std::vector<sensor_msgs::msg::Image> tray_depth_messages_;
  bool detecting_seen_{false};
};

class DetectTargetNodeWithoutTransformTest : public DetectTargetNodeRosBoundaryTest
{
protected:
  void SetUp() override {start_node("camera_without_tf", false);}
};

TEST_F(DetectTargetNodeRosBoundaryTest, CameraInfoValidationPublishesCalibrationStateChanges)
{
  ASSERT_TRUE(publish_until_calibration_state(CalibrationState::STATE_READY, 42, camera_info()));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ASSERT_FALSE(states_.empty());
    EXPECT_EQ(states_.front(), CalibrationState::STATE_READY);
  }

  auto incompatible = camera_info();
  incompatible.k[0] += 1.0;
  ASSERT_TRUE(publish_until_calibration_state(
      CalibrationState::STATE_NEEDS_CALIBRATION, 100, incompatible));

  ASSERT_TRUE(publish_until_calibration_state(
      CalibrationState::STATE_READY, 200, camera_info(), 2));
  EXPECT_TRUE(wait_for_state(CalibrationState::STATE_READY, 2));
}

TEST_F(DetectTargetNodeRosBoundaryTest, OverlappingRequestsReturnBusyWithoutReplacingTheActiveRequest)
{
  ASSERT_TRUE(publish_until_calibration_state(CalibrationState::STATE_READY, 42, camera_info()));
  ASSERT_TRUE(detect_client_->wait_for_service(2s));

  auto active_request = std::make_shared<arm_cell_interfaces::srv::DetectTarget::Request>();
  active_request->target_id = "RawPart";
  active_request->algorithm = "cuboid_top_face";
  active_request->timeout.sec = 1;
  auto active_response = detect_client_->async_send_request(active_request);
  ASSERT_TRUE(wait_for_detecting());

  ASSERT_TRUE(calibration_client_->wait_for_service(2s));
  auto calibration_response = calibration_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(calibration_response.wait_for(2s), std::future_status::ready);
  const auto calibration_result = calibration_response.get();
  EXPECT_FALSE(calibration_result->success);
  EXPECT_NE(calibration_result->message.find("Acquisition is active"), std::string::npos);

  auto overlapping_request =
    std::make_shared<arm_cell_interfaces::srv::DetectTarget::Request>(*active_request);
  auto overlapping_response = detect_client_->async_send_request(overlapping_request);
  ASSERT_EQ(overlapping_response.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(overlapping_response.get()->result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_BUSY);

  ASSERT_EQ(active_response.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(active_response.get()->result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
}

TEST_F(DetectTargetNodeRosBoundaryTest, DetectionUsesTheObservationThatCompletesSceneStability)
{
  ASSERT_TRUE(publish_until_calibration_state(CalibrationState::STATE_READY, 42, camera_info()));

  ASSERT_TRUE(detect_client_->wait_for_service(2s));
  auto request = std::make_shared<arm_cell_interfaces::srv::DetectTarget::Request>();
  request->target_id = "RawPart";
  request->algorithm = "cuboid_top_face";
  request->timeout.sec = 3;
  auto response_future = detect_client_->async_send_request(request);
  ASSERT_TRUE(wait_for_detecting());

  for (uint32_t stamp = 100; stamp < 104; ++stamp) {
    publish_observation_pair(stamp, camera_info(), true);
    ASSERT_TRUE(wait_for_scene_stamp(stamp)) << "Depth observation was not processed";
  }
  {
    std::lock_guard<std::mutex> state_lock(mutex_);
    EXPECT_FALSE(std::any_of(processed_scenes_.begin(), processed_scenes_.end(),
      [](const auto & item) {
        return item.second == arm_cell_interfaces::msg::SceneStability::SCENE_STABILITY_STABLE;
      }));
  }
  EXPECT_EQ(response_future.wait_for(0ms), std::future_status::timeout)
    << "the request must not complete from a cached pre-request observation";

  publish_observation_pair(104, camera_info(), true, true);
  ASSERT_TRUE(wait_for_scene_stamp(104));
  ASSERT_EQ(response_future.wait_for(2s), std::future_status::ready);
  const auto response = response_future.get();
  ASSERT_EQ(response->result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_SUCCESS)
    << response->diagnostic_detail;
  ASSERT_FALSE(response->material_candidates.empty());
  ASSERT_TRUE(response->material_candidates.at(0).has_estimated_height);
  EXPECT_NEAR(response->material_candidates.at(0).estimated_height_m, 0.08, 0.01);
  EXPECT_EQ(response->material_candidates.at(0).target_pose.header.stamp.sec, 104);
  ASSERT_FALSE(response->material_candidates.empty());
  EXPECT_EQ(response->material_candidates.front().target_pose.header.stamp.sec, 104);
}

TEST_F(DetectTargetNodeRosBoundaryTest, TrayDepthPublishesWithoutAnActiveDetectionRequest)
{
  auto subscription = driver_->create_subscription<sensor_msgs::msg::Image>(
    "/vision/tray_aligned_depth/image_raw", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      tray_depth_messages_.push_back(*message);
      condition_.notify_all();
    });
  ASSERT_TRUE(publish_until_calibration_state(CalibrationState::STATE_READY, 42, camera_info()));

  for (uint32_t attempt = 0; attempt < 10; ++attempt) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!tray_depth_messages_.empty()) {break;}
    }
    publish_observation_pair(100 + attempt, camera_info(), true);
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait_for(lock, 100ms, [this]() {return !tray_depth_messages_.empty();});
  }
  ASSERT_TRUE(wait_for_tray_depth());

  sensor_msgs::msg::Image output;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    output = tray_depth_messages_.front();
  }
  EXPECT_EQ(output.header.frame_id, "Tray");
  EXPECT_EQ(output.encoding, "32FC1");
  EXPECT_GT(output.width, 0U);
  EXPECT_GT(output.height, 0U);
  ASSERT_EQ(output.data.size(), static_cast<size_t>(output.step) * output.height);
  bool observed_height = false;
  for (size_t offset = 0; offset + sizeof(float) <= output.data.size(); offset += sizeof(float)) {
    float height = 0.0F;
    std::memcpy(&height, output.data.data() + offset, sizeof(height));
    if (std::isfinite(height) && height > 0.0F) {
      observed_height = true;
      break;
    }
  }
  EXPECT_TRUE(observed_height);
}

TEST_F(DetectTargetNodeWithoutTransformTest, MissingTransformCannotProduceDetectionSuccess)
{
  ASSERT_TRUE(publish_until_calibration_state(CalibrationState::STATE_READY, 42, camera_info()));
  ASSERT_TRUE(detect_client_->wait_for_service(2s));

  auto request = std::make_shared<arm_cell_interfaces::srv::DetectTarget::Request>();
  request->target_id = "RawPart";
  request->algorithm = "cuboid_top_face";
  request->timeout.sec = 1;
  auto response_future = detect_client_->async_send_request(request);
  ASSERT_TRUE(wait_for_detecting());

  for (uint32_t stamp = 100; stamp < 105; ++stamp) {
    publish_observation_pair(stamp, camera_info(), true);
    ASSERT_TRUE(wait_for_scene_stamp(stamp)) << "depth observation was not processed";
  }

  ASSERT_EQ(response_future.wait_for(2s), std::future_status::ready);
  const auto response = response_future.get();
  EXPECT_EQ(response->result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
  EXPECT_TRUE(response->material_candidates.empty());
  EXPECT_NE(response->diagnostic_detail.find("tf_unavailable"), std::string::npos)
    << response->diagnostic_detail;
}

}  // namespace
