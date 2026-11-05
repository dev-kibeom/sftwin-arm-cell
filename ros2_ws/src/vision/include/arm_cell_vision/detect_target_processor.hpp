#ifndef ARM_CELL_VISION__DETECT_TARGET_PROCESSOR_HPP_
#define ARM_CELL_VISION__DETECT_TARGET_PROCESSOR_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <arm_cell_interfaces/msg/detect_target_result_code.hpp>
#include <arm_cell_interfaces/msg/collision_primitive.hpp>
#include <arm_cell_interfaces/msg/disturbance_candidate.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <tf2/LinearMath/Transform.h>

namespace arm_cell_vision
{

struct DetectionInput
{
  // Empty when no compatible presentation RGB frame was available. Geometric
  // strategies consume depth; explicit Calibration still requires RGB.
  sensor_msgs::msg::Image rgb;
  sensor_msgs::msg::Image depth;
  sensor_msgs::msg::CameraInfo camera_info;
  geometry_msgs::msg::TransformStamped optical_to_base;
  bool detailed_diagnostics_enabled = false;
  std::string rgb_presentation_status = "not_received";
  int64_t depth_receipt_monotonic_ns = -1;
};

struct RawPartCandidateDiagnostic
{
  uint32_t candidate_id = 0;
  double position_x_m = 0.0;
  double position_y_m = 0.0;
  double position_z_m = 0.0;
  double measured_width_m = 0.0;
  double measured_height_m = 0.0;
  double expected_width_m = 0.0;
  double expected_height_m = 0.0;
  double width_tolerance_m = 0.0;
  double height_tolerance_m = 0.0;
  double measured_vertical_height_m = 0.0;
  double expected_vertical_height_m = 0.0;
  double vertical_height_tolerance_m = 0.0;
  uint8_t orientation_symmetry_order = 0;
  bool observed_yaw_valid = false;
  double observed_yaw_rad = 0.0;
  double inlier_fraction = 0.0;
  double coverage = 0.0;
  double edge_support = 0.0;
  double max_normalized_residual = 0.0;
  double selection_residual = 0.0;
  bool profile_match = false;
  std::string rejection_reason;
  uint32_t greedy_rank = 0;
  bool selected_for_target = false;
  std::vector<geometry_msgs::msg::Point32> fitted_outline_support;
  std::vector<geometry_msgs::msg::Point32> fitted_outline_pixels;
  std::vector<geometry_msgs::msg::Point32> orientation_axis_pixels;
  std::string confidence_grade;
  std::string confidence_reason;
};

struct CuboidStageDiagnostic
{
  std::string stage;
  std::string status = "computed";
  std::string reason;
  std::string count_unit;
  std::optional<uint64_t> input_count;
  std::optional<uint64_t> output_count;
  std::optional<uint64_t> rejected_count;
  uint64_t candidate_evaluation_attempt_count = 0;
  uint64_t rejected_candidate_evaluation_count = 0;
  uint64_t fitted_edge_count = 0;
  uint64_t pre_object_rejected_count = 0;
  uint64_t deduplicated_count = 0;
  uint64_t generated_count = 0;
  std::map<std::string, uint64_t> deduplicated_by_reason;
  std::map<std::string, uint64_t> rejected_by_reason;
  std::map<std::string, uint64_t> outcome_by_status;
  std::map<std::string, uint64_t> auxiliary_counts;
  std::map<std::string, uint64_t> edge_fit_rejected_by_reason;
};

struct CuboidReplayEvidence
{
  uint32_t rows = 0;
  uint32_t columns = 0;
  double resolution_m = 0.0;
  double origin_x_m = 0.0;
  double origin_y_m = 0.0;
  std::string frame_id;
  // Shallow cv::Mat headers share the immutable buffers produced by this run.
  // Capturing evidence does not copy the grid or mask payload. NaN means invalid.
  cv::Mat height_grid_m;
  // The edge of the source RGB-D image is excluded from some hypotheses.
  cv::Mat image_boundary_mask;
};

struct CuboidHeightPatchDiagnostic
{
  uint32_t source_boundary_id = 0;
  std::array<double, 3> plane_normal{0, 0, 1};
  std::array<double, 3> plane_center_height_m{};
  std::array<double, 2> observed_height_p05_p95_m{};
  std::vector<double> fitted_edge_residual_px;
  std::vector<std::vector<uint32_t>> fitted_edge_support_cell_ids;
  double plane_residual_m = 0;
  double plane_height_span_m = 0;
  bool plane_fitted = false;
  std::string plane_status;
  // source Heightmap cell IDs, adjacency IDs and signed observed height steps.
  std::vector<uint32_t> observed_cell_ids;
  std::vector<std::array<double, 3>> adjacent_planes;
  std::vector<std::array<double, 5>> rectangle_regions;
  std::vector<std::array<double, 4>> inferred_partition_edges;
  std::vector<uint64_t> fitted_edge_source_ids;
  std::vector<std::array<double, 3>> corners_xy_turn;
  std::vector<std::array<uint32_t, 2>> rectangle_alternatives;
  double height_m = 0.0;
  uint64_t cell_count = 0;
  uint64_t contour_count = 0;
  uint64_t contours_with_fitted_edges_count = 0;
  uint64_t fitted_edge_count = 0;
  uint64_t edge_fit_attempt_count = 0;
  std::map<std::string, uint64_t> contour_rejected_by_reason;
  std::map<std::string, uint64_t> edge_fit_rejected_by_reason;
  uint64_t candidate_evaluation_attempt_count = 0;
  uint64_t pre_object_rejected_count = 0;
  uint64_t deduplicated_count = 0;
  std::map<std::string, uint64_t> deduplicated_by_reason;
  uint64_t generated_hypothesis_count = 0;
  uint64_t geometry_valid_before_deduplication_count = 0;
  uint64_t partitioned_candidate_count = 0;
  std::string partition_status;
  std::string partition_reason;
  std::map<std::string, uint64_t> hypothesis_rejected_by_reason;
  std::map<std::string, uint64_t> geometry_rejected_by_reason;
  // Exact contour vertices and accepted fitted edge segments from the detector.
  // Populated only when detailed replay evidence is requested.
  std::vector<std::vector<geometry_msgs::msg::Point32>> observed_contours_xy;
  std::vector<std::array<double, 5>> fitted_edges_xy_support;
  std::vector<geometry_msgs::msg::Point32> observed_cells_xy;
  std::string confidence_grade;
  std::string confidence_reason;
};

// Internal observation references and diagnostic projection of the exact Fits computed by
// cuboid_top_face. IDs are local to this one DetectionResult/observation.
struct CuboidHypothesisVisualization
{
  uint32_t candidate_id = 0;
  uint32_t patch_index = 0;
  double height_m = 0.0;
  double center_x_m = 0.0;
  double center_y_m = 0.0;
  double yaw_rad = 0.0;
  double length_m = 0.0;
  double width_m = 0.0;
  double thickness_m = 0.0;
  double observed_area_ratio = 0.0;
  std::optional<double> boundary_support;
  std::string disposition;
  std::string reason;
  uint32_t deduplicated_to_candidate_id = 0;
  std::string confidence_grade;
  std::string confidence_reason;
  double anchor_x_m = 0.0;
  double anchor_y_m = 0.0;
  uint32_t observed_boundary_sides = 0;
  std::string tray_support_evidence;
  std::vector<double> parallel_edge_distances_m;
  std::array<double, 2> observed_anchor_edge_lengths_m{};
  std::string selection_state;
  std::string selection_reason;
  std::vector<uint32_t> competing_candidate_ids;
  uint32_t source_boundary_id = 0;
  uint32_t outline_id = 0;
  uint64_t observed_support_cells = 0;
  std::map<uint64_t, double> observed_line_support_m;
  double height_residual = 0;
  double dimension_residual = 0;
  uint32_t dimension_observation_count = 0;
  std::optional<double> complete_iou;
  uint32_t rectangle_id = 0;
};

// Internal provenance, aligned by candidate_id with the existing wire candidates.
struct DisturbanceRegionDiagnostic
{
  std::string candidate_id;
  uint32_t source_boundary_id = 0;
  uint32_t outline_id = 0;
  uint32_t patch_index = 0;
  uint32_t rectangle_id = 0;
  std::vector<uint32_t> hypothesis_ids;
  std::string representative_method;
  std::string reason;
};

struct DetectorProfile;

struct DetectionResult
{
  std::vector<DisturbanceRegionDiagnostic> disturbance_regions;
  // Internal Vision classification. Never projected onto PickCandidate.
  std::string confidence_grade;
  std::string confidence_reason;
  std::string algorithm;
  std::shared_ptr<const DetectorProfile> applied_profile;
  std::string thickness_evidence;
  // Strategy order, from this exact observation. Entries never contain another sequence.
  std::vector<DetectionResult> ordered_candidates;
  uint64_t unresolved_pixels = 0;
  uint8_t result_code =
    arm_cell_interfaces::msg::DetectTargetResultCode::DETECT_RESULT_INVALID_RESULT;
  // Short semantic failure cause for operational logs; diagnostic_detail keeps full evidence.
  std::string failure_cause;
  // Candidate-local geometry, also used by independent calibration/profile results.
  // Cuboid Acquisition stores all geometry in ordered_candidates; no representative.
  geometry_msgs::msg::PoseStamped target_pose;
  bool has_target_pose = false;
  std::vector<arm_cell_interfaces::msg::DisturbanceCandidate> disturbance_candidates;
  uint8_t orientation_symmetry_order = 0;
  float estimated_height_m = 0.0F;
  bool has_estimated_height = false;
  bool has_observed_rectangle = false;
  float rectangle_long_side_m = 0.0F;
  float rectangle_short_side_m = 0.0F;
  uint64_t scene_generation = 0;
  bool profile_candidate_valid = false;
  std::array<double, 6> profile_candidate_features{};
  double observed_yaw_rad = 0.0;
  std::string diagnostic_detail;
  std::string processing_stage;
  std_msgs::msg::Header source_rgb_header;
  bool centroid_pixel_valid = false;
  float centroid_pixel_x = 0.0F;
  float centroid_pixel_y = 0.0F;
  bool object_region_valid = false;
  uint32_t object_region_x = 0;
  uint32_t object_region_y = 0;
  uint32_t object_region_width = 0;
  uint32_t object_region_height = 0;
  bool support_region_valid = false;
  std::vector<geometry_msgs::msg::Point32> support_region;
  bool has_rawpart_diagnostics = false;
  uint64_t rawpart_workspace_xyz_points = 0;
  uint64_t rawpart_candidate_count = 0;
  uint64_t rawpart_profile_match_count = 0;
  std::string rawpart_candidate_generation;
  std::map<std::string, uint64_t> rawpart_rejected_by_reason;
  bool rawpart_absence_confirmation_available = false;
  bool rawpart_absence_confirmed = false;
  std::string rawpart_selected_candidate;
  std::vector<RawPartCandidateDiagnostic> rawpart_candidate_diagnostics;
  std::vector<CuboidStageDiagnostic> cuboid_stages;
  // Legacy collection name retained for Snapshot compatibility; entries are planes.
  std::vector<CuboidHeightPatchDiagnostic> cuboid_height_patches;
  std::vector<CuboidHeightPatchDiagnostic> cuboid_boundaries;
  std::vector<CuboidHypothesisVisualization> cuboid_hypotheses;
  std::shared_ptr<CuboidReplayEvidence> cuboid_replay_evidence;
};

struct CuboidTopFaceConfig
{
  double grid_resolution_m = 0.0025;
  double height_tolerance_m = 0.003;
  double line_residual_limit_px = 1.25;
  double minimum_line_support = 0.8;
};

struct DetectorProfile
{
  std::string algorithm = "cuboid_top_face";
  CuboidTopFaceConfig cuboid;
  std::string target_id;
  std::string material_id;
  std::string shape;
  std::array<double, 3> dimensions_m{};
  std::array<double, 3> dimension_tolerance_m{};
  std::array<double, 3> workspace_roi_min_m{};
  std::array<double, 3> workspace_roi_max_m{};
  std::array<double, 2> tray_roi_min_m{};
  std::array<double, 2> tray_roi_max_m{};
  double support_contact_tolerance_m = 0.0;
  double top_surface_geometry_tolerance_m = 0.0;
  double minimum_observation_coverage = 0.55;
  std::string fiducial_dictionary;
  int fiducial_marker_id = -1;
  double fiducial_size_m = 0.0;
  double fiducial_pose_translation_scale = 1.0;
  std::array<double, 3> marker_in_support_m{};
  std::array<double, 3> marker_rpy_in_support_rad{};
};

struct TrayDepthCoverage
{
  double valid_depth_fraction = 0.0;
  bool fully_visible = false;
};

std::optional<TrayDepthCoverage> rawpart_tray_depth_coverage(
  const sensor_msgs::msg::Image & depth, const sensor_msgs::msg::CameraInfo & camera_info,
  const DetectorProfile & profile, const tf2::Transform & support_T_camera);

struct SupportCalibrationSnapshot
{
  bool ready = false;
  uint32_t accepted_frames = 0;
  uint32_t required_frames = 1;
  double median_depth_residual_m = 0.0;
  struct DepthValidation
  {
    uint32_t valid_sample_count = 0;
    bool residual_available = false;
    double median_signed_residual_m = 0.0;
    double minimum_signed_residual_m = 0.0;
    double maximum_signed_residual_m = 0.0;
    double support_contact_tolerance_m = 0.0;
    double configured_marker_height_m = 0.0;
    bool accepted = false;
    std::string reason;
  };
  std::vector<DepthValidation> recent_depth_validations;
  geometry_msgs::msg::TransformStamped tray_from_camera;
  std::vector<geometry_msgs::msg::Point32> support_region;
};

class DetectTargetProcessor
{
public:
  DetectionResult process(const DetectionInput & input) const;
  DetectionResult process(
    const DetectionInput & input, const std::string & target_id,
    const std::vector<DetectorProfile> & profiles,
    bool calibration_only = false) const;
  void set_loaded_support_calibration(const SupportCalibrationSnapshot & calibration);
  void clear_loaded_support_calibration();
  SupportCalibrationSnapshot support_calibration_snapshot() const;

private:
  DetectionResult process_profile(
    const DetectionInput & input, const std::string & target_id,
    const std::vector<DetectorProfile> & profiles, bool calibration_only) const;
  struct SelectionState
  {
    std::mutex mutex;
    uint32_t explicit_calibration_observations = 0;
    std::vector<SupportCalibrationSnapshot::DepthValidation> recent_depth_validations;
    std::optional<geometry_msgs::msg::TransformStamped> cached_tray_from_camera;
    std::vector<geometry_msgs::msg::Point32> cached_support_region;
  };

  std::shared_ptr<SelectionState> selection_state_ = std::make_shared<SelectionState>();
};

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__DETECT_TARGET_PROCESSOR_HPP_
