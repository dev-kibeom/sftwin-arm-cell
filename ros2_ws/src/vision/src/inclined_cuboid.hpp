#pragma once

#include "cuboid_top_face_geometry.hpp"

namespace arm_cell_vision::cuboid_geometry
{
void normalize_cuboid_hypothesis_aliases(std::vector<CuboidHypothesisVisualization> & hypotheses);
void finalize_cuboid_diagnostics(DetectionResult & result);

void reconcile_cuboid_volumes(DetectionResult & result, double tolerance);

// Uses retained Plane Map contours and fitted lines, never resamples Depth.
void append_inclined_cuboids(
  DetectionResult & result, const PlaneMap & map, const CuboidHeightObservation & observation,
  const DetectorProfile & profile, uint32_t & next_id, size_t & operations);
}
