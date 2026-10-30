#ifndef ARM_CELL_VISION__CUBOID_TOP_FACE_GEOMETRY_HPP_
#define ARM_CELL_VISION__CUBOID_TOP_FACE_GEOMETRY_HPP_

#include <optional>
#include <limits>

#include "arm_cell_vision/cuboid_top_face.hpp"

namespace arm_cell_vision
{
namespace cuboid_geometry
{
struct Face
{
  double length;
  double width;
  double thickness;
  double length_tolerance;
  double width_tolerance;
};
struct Segment
{
  cv::Point2d first;
  cv::Point2d last;
  double support;
  double residual_px;
  bool observed = true;
  uint64_t source_line_id = 0;
};
struct ContourEvidence
{
  uint32_t outline_id = 0;
  std::vector<cv::Point> points;
  std::vector<cv::Point> vertices;
  std::vector<Segment> lines;
  std::shared_ptr<const std::vector<cv::Point>> observed_boundary;
};
struct Fit
{
  uint32_t candidate_id = 0;
  uint32_t patch_index = 0;
  uint32_t source_boundary_id = 0;
  uint32_t outline_id = 0;
  cv::Point2d center;
  Face face;
  double yaw;
  double top;
  double area;
  std::optional<double> boundary;
  uint32_t observed_boundary_sides = 0;
  cv::Mat mask;
  size_t mask_cell_count = 0;
  cv::Mat tolerant;
  double measured_length = 0.0;
  double measured_width = 0.0;
  cv::Point2d observed_center;
  double observed_top = 0.0;
  bool is_partial_reconstruction = false;
  bool rectangle_measured = false;
  bool covered_by_upper_surface = false;
  cv::Point2d anchor;
  std::string tray_support_evidence;
  std::vector<double> parallel_edge_distances;
  std::vector<uint32_t> observed_cell_ids;
  std::map<uint64_t, double> observed_line_support_m;
  std::optional<double> support_height_m;
  double height_residual = 0;
  double dimension_residual = 0;
  uint32_t dimension_observation_count = 0;
  std::optional<double> complete_iou;
  uint32_t rectangle_id = 0;
};


struct PlaneRegion
{
  std::shared_ptr<const std::vector<ContourEvidence>> source_contours;
  uint32_t source_boundary_id = 0;
  cv::Vec3d normal{0, 0, 1};
  std::array<double, 3> center_height{};
  std::array<double, 2> observed_height_p05_p95{};
  double residual_m = 0;
  double height_span_m = 0;
  bool plane_fitted = false;
  std::vector<std::array<double, 3>> adjacency;
  double height = 0;
  cv::Mat cells;
  std::vector<cv::Point> pixels;
  std::string status;
};
// Identity and scalar observation statistics of an existing outline/partition.
// No disturbance-specific masks or cell collections are retained.
struct ObservationRegion
{
  uint32_t source_boundary_id = 0;
  uint32_t outline_id = 0;
  uint32_t patch_index = 0;
  uint32_t rectangle_id = 0;
  cv::Point2d observed_centroid{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::optional<double> mean_observed_height;
  // Existing observed contour in grid coordinates and optional partition in Tray.
  std::shared_ptr<const std::vector<cv::Point>> observed_boundary;
  std::vector<cv::Point2f> partition_boundary;
  uint8_t category = arm_cell_interfaces::msg::DisturbanceCandidate::CATEGORY_OTHER_UNRESOLVED;
  bool independent = true;
  bool unresolved = false;
};
struct PlaneFits
{
  std::vector<ContourEvidence> occupancy_contours;
  std::vector<ObservationRegion> partitions;
  std::vector<uint32_t> unresolved_outline_ids;
  std::vector<Fit> fits;
  std::vector<CuboidHypothesisVisualization> hypotheses;
  CuboidHeightPatchDiagnostic diagnostic;
  std::map<std::string, uint64_t> counts;
};
struct Selection
{
  std::string state = "SELECTED";
  std::string reason = "independently_supported_compatible_object";
  std::vector<uint32_t> competitors;
};

std::map<uint32_t, Selection> select_hypotheses(
  const std::vector<Fit> & candidates, const CuboidHeightObservation & obs,
  const DetectorProfile & profile, size_t & operations);
void append_disturbance_regions(
  DetectionResult & result, const std::vector<ObservationRegion> & outlines,
  const std::vector<ObservationRegion> & partitions,
  const CuboidHeightObservation & observation, const DetectorProfile & profile);
cv::Point2d axis(double yaw);
cv::Point2d normal(double yaw);
std::vector<cv::Point2f> corners(const Fit & fit);
cv::Mat footprint(const Fit & fit, const CuboidHeightObservation & obs);
bool within_height_tolerance(double a, double b, double tolerance);
std::vector<PlaneRegion> extract_plane_regions(
  const CuboidHeightObservation & obs, const DetectorProfile & profile,
  const cv::Mat * allowed_cells = nullptr);
struct PlaneMap
{
  std::vector<ObservationRegion> outlines;
  std::vector<PlaneRegion> regions;
  std::vector<std::vector<ContourEvidence>> region_contours;
  std::vector<PlaneRegion> recovered_inclined_regions;
  std::vector<std::vector<ContourEvidence>> recovered_inclined_contours;
  std::vector<CuboidHeightPatchDiagnostic> boundaries;
};
PlaneMap build_plane_map(const CuboidHeightObservation & obs, const DetectorProfile & profile,
  size_t & operations, bool retain_evidence);
bool has_positive_inclined_surface_evidence(
  const PlaneRegion & patch, const CuboidHeightObservation & obs, double tolerance);
PlaneFits reconstruct_plane(
  const PlaneRegion & patch, uint32_t patch_index, uint32_t & next_id, size_t & search_operations,
  const CuboidHeightObservation & obs, const DetectorProfile & profile,
  bool retain_visualization_evidence = false);
}  // namespace cuboid_geometry
}  // namespace arm_cell_vision
#endif
