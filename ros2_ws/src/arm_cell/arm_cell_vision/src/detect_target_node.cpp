#include <chrono>
#include <array>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "arm_cell_vision/detect_target_service.hpp"

namespace arm_cell_vision
{

class DetectTargetNode : public VisionRosIngress
{
public:
  explicit DetectTargetNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : VisionRosIngress(options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    detector_profiles_(load_detector_profiles()),
    service_(
      *this, DetectTargetProcessor(),
      []() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      },
      [this](const std::string & source_frame, int64_t) {
        try {
          return std::optional<geometry_msgs::msg::TransformStamped>(
            tf_buffer_.lookupTransform("base_link", source_frame, tf2::TimePointZero));
        } catch (const tf2::TransformException &) {
          return std::optional<geometry_msgs::msg::TransformStamped>();
        }
      }, detector_profiles_)
  {
    service_.advertise(*this);
  }

private:
  std::vector<DetectorProfile> load_detector_profiles()
  {
    const auto prefix = std::string("detector_profiles.RawPart.");
    DetectorProfile profile;
    profile.target_id = declare_parameter<std::string>(prefix + "target_id", "");
    profile.shape = declare_parameter<std::string>(prefix + "shape", "");
    const auto dimensions = declare_parameter<std::vector<double>>(
      prefix + "dimensions_m", {});
    const auto dimension_tolerance = declare_parameter<std::vector<double>>(
      prefix + "dimension_tolerance_m", {});
    const auto roi_x = declare_parameter<std::vector<double>>(
      prefix + "workspace_roi_m.x", {});
    const auto roi_y = declare_parameter<std::vector<double>>(
      prefix + "workspace_roi_m.y", {});
    const auto roi_z = declare_parameter<std::vector<double>>(
      prefix + "workspace_roi_m.z", {});
    const auto marker_position = declare_parameter<std::vector<double>>(
      prefix + "fiducial.marker_in_support.position_m", {});
    const auto marker_rpy = declare_parameter<std::vector<double>>(
      prefix + "fiducial.marker_in_support.rpy_rad", {});
    const auto copy_three = [](const std::vector<double> & source,
        std::array<double, 3> & target) {
        if (source.size() == target.size()) {
          std::copy(source.begin(), source.end(), target.begin());
        }
      };
    const auto copy_roi = [](const std::vector<double> & source,
        size_t axis, std::array<double, 3> & minimum, std::array<double, 3> & maximum) {
        if (source.size() == 2) {
          minimum[axis] = source[0];
          maximum[axis] = source[1];
        }
      };
    copy_three(dimensions, profile.dimensions_m);
    copy_three(dimension_tolerance, profile.dimension_tolerance_m);
    copy_three(marker_position, profile.marker_in_support_m);
    copy_three(marker_rpy, profile.marker_rpy_in_support_rad);
    copy_roi(roi_x, 0, profile.workspace_roi_min_m, profile.workspace_roi_max_m);
    copy_roi(roi_y, 1, profile.workspace_roi_min_m, profile.workspace_roi_max_m);
    copy_roi(roi_z, 2, profile.workspace_roi_min_m, profile.workspace_roi_max_m);
    profile.support_contact_tolerance_m = declare_parameter<double>(
      prefix + "support_contact_tolerance_m", 0.0);
    profile.top_surface_geometry_tolerance_m = declare_parameter<double>(
      prefix + "top_surface_geometry_tolerance_m", 0.0);
    profile.fiducial_dictionary = declare_parameter<std::string>(
      prefix + "fiducial.dictionary", "");
    profile.fiducial_marker_id = static_cast<int>(declare_parameter<int64_t>(
        prefix + "fiducial.marker_id", -1));
    profile.fiducial_size_m = declare_parameter<double>(
      prefix + "fiducial.size_m", 0.0);
    profile.fiducial_pose_translation_scale = declare_parameter<double>(
      prefix + "fiducial.pose_translation_scale", 1.0);
    return {profile};
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<DetectorProfile> detector_profiles_;
  DetectTargetService service_;
};

}  // namespace arm_cell_vision

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  auto node = std::make_shared<arm_cell_vision::DetectTargetNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
