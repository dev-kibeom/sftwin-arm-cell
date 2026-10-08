#include <cstdint>
#include <chrono>
#include <atomic>
#include <cstring>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

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

TEST_F(DetectTargetServiceTest, RejectsNegativeRelativeTimeout)
{
  arm_cell_vision::VisionRosIngress ingress;
  arm_cell_vision::DetectTargetService service(
    ingress, arm_cell_vision::DetectTargetProcessor(), []() {return int64_t{100};},
    [](const std::string &, int64_t) {
      return std::optional<geometry_msgs::msg::TransformStamped>();
    });
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.timeout.sec = -1;
  arm_cell_interfaces::srv::DetectTarget::Response response;

  service.handle(request, response);

  EXPECT_EQ(
    response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_INVALID_RESULT);
}

TEST_F(DetectTargetServiceTest, UsesLocalSteadyDeadlineForEmptyIngress)
{
  arm_cell_vision::VisionRosIngress ingress;
  int64_t now = 100;
  arm_cell_vision::DetectTargetService service(
    ingress, arm_cell_vision::DetectTargetProcessor(), [&now]() {return now += 1;},
    [](const std::string &, int64_t) {
      return std::optional<geometry_msgs::msg::TransformStamped>();
    });
  arm_cell_interfaces::srv::DetectTarget::Request request;
  request.timeout.nanosec = 2;
  arm_cell_interfaces::srv::DetectTarget::Response response;

  service.handle(request, response);

  EXPECT_EQ(
    response.result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
  EXPECT_NE(response.diagnostic_detail.find("no RGB observed"), std::string::npos);
  EXPECT_NE(response.diagnostic_detail.find("latest_age_ms"), std::string::npos);
  EXPECT_NE(response.diagnostic_detail.find("last_ingress_status"), std::string::npos);
}

TEST_F(DetectTargetServiceTest, ServiceRequestProgressesWithIngressCallbacks)
{
  std::atomic<int> deadline_phase{0};
  auto ingress = std::make_shared<arm_cell_vision::VisionRosIngress>(
    rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter("request_timeout_ms", 1000.0)}));
  arm_cell_vision::DetectTargetService service(
    *ingress, arm_cell_vision::DetectTargetProcessor(),
    [&deadline_phase]() -> int64_t {
      const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      const auto phase = deadline_phase.load();
      if (phase == 1) {
        deadline_phase.store(2);
        return now;
      }
      if (phase == 2) {
        deadline_phase.store(3);
        return now + 2'000'000'000LL;
      }
      if (phase >= 3) {
        deadline_phase.store(4);
        return now + 2'100'000'000LL;
      }
      return now;
    },
    [&deadline_phase](const std::string & source_frame, int64_t) {
      deadline_phase.store(1);
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "base_link";
      transform.child_frame_id = source_frame;
      transform.transform.rotation.w = 1.0;
      return std::optional<geometry_msgs::msg::TransformStamped>(transform);
    });
  auto service_server = service.advertise(*ingress);
  auto client_node = std::make_shared<rclcpp::Node>("detect_target_service_test_client");
  auto client = client_node->create_client<arm_cell_interfaces::srv::DetectTarget>(
    "/vision/detect_target");
  auto info_publisher = client_node->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::SensorDataQoS());
  auto rgb_publisher = client_node->create_publisher<sensor_msgs::msg::Image>(
    "/camera/color/image_raw", rclcpp::SensorDataQoS());
  auto depth_publisher = client_node->create_publisher<sensor_msgs::msg::Image>(
    "/camera/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(ingress);
  executor.add_node(client_node);
  std::thread spin_thread([&executor]() {executor.spin();});
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  const auto wait_for_subscription = [](const auto & publisher) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      while (publisher->get_subscription_count() == 0 &&
        std::chrono::steady_clock::now() < deadline)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      return publisher->get_subscription_count() > 0;
    };

  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = "camera_color_optical_frame";
  info.width = 2;
  info.height = 2;
  info.distortion_model = "plumb_bob";
  info.k[0] = 100.0;
  info.k[2] = 0.5;
  info.k[4] = 100.0;
  info.k[5] = 0.5;
  info.p[0] = 100.0;
  info.p[2] = 0.5;
  info.p[5] = 100.0;
  info.p[6] = 0.5;
  ASSERT_TRUE(wait_for_subscription(info_publisher));
  for (size_t attempt = 0; attempt < 20 && !ingress->has_cached_calibration(); ++attempt) {
    info_publisher->publish(info);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(ingress->has_cached_calibration());

  auto request = std::make_shared<arm_cell_interfaces::srv::DetectTarget::Request>();
  request->timeout.sec = 1;
  auto future = client->async_send_request(request);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  sensor_msgs::msg::Image rgb;
  rgb.header.frame_id = info.header.frame_id;
  rgb.header.stamp.sec = 1;
  rgb.width = 2;
  rgb.height = 2;
  rgb.encoding = "rgb8";
  rgb.step = 6;
  rgb.data.resize(12, 100);
  sensor_msgs::msg::Image depth;
  depth.header = rgb.header;
  depth.width = 2;
  depth.height = 2;
  depth.encoding = "32FC1";
  depth.step = 8;
  depth.data.resize(4 * sizeof(float));
  const float background = 1.0F;
  for (size_t index = 0; index < 4; ++index) {
    std::memcpy(depth.data.data() + index * sizeof(float), &background, sizeof(float));
  }
  ASSERT_TRUE(wait_for_subscription(rgb_publisher));
  ASSERT_TRUE(wait_for_subscription(depth_publisher));
  for (uint32_t attempt = 0; attempt < 20 && future.wait_for(std::chrono::milliseconds(0)) !=
    std::future_status::ready; ++attempt)
  {
    rgb.header.stamp.nanosec = attempt * 10'000'000U;
    depth.header.stamp = rgb.header.stamp;
    rgb_publisher->publish(rgb);
    depth_publisher->publish(depth);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_GT(ingress->acceptance_snapshot().synchronized_callback_count, 0U);

  ASSERT_EQ(
    future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto response = future.get();
  EXPECT_EQ(
    response->result_code.value,
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_TIMEOUT);
  EXPECT_NE(
    response->diagnostic_detail.find("processing started but request deadline expired"),
    std::string::npos);
  EXPECT_NE(
    response->diagnostic_detail.find("processing_elapsed_ms="), std::string::npos);
  EXPECT_NE(
    response->diagnostic_detail.find("latest_age_ms{rgb="), std::string::npos);
  EXPECT_NE(
    response->diagnostic_detail.find("latest_received_rgb_depth_delta_ms="), std::string::npos);
  EXPECT_NE(
    response->diagnostic_detail.find("last_ingress_status="), std::string::npos);
  executor.cancel();
  spin_thread.join();
  (void)service_server;
}

}  // namespace
