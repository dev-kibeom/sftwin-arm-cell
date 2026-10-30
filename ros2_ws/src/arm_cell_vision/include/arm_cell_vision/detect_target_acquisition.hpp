#ifndef ARM_CELL_VISION__DETECT_TARGET_ACQUISITION_HPP_
#define ARM_CELL_VISION__DETECT_TARGET_ACQUISITION_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "arm_cell_vision/detect_target_processor.hpp"

namespace arm_cell_vision
{

// One completed observation/result pair shared read-only by service and Snapshot.
struct AcquisitionResult
{
  uint64_t generation = 0;
  std::string acquisition_id;
  std::string target_id;
  std::shared_ptr<const DetectionInput> input;
  DetectionResult detection;
  SupportCalibrationSnapshot calibration;
};

using SharedAcquisitionResult = std::shared_ptr<const AcquisitionResult>;

}  // namespace arm_cell_vision

#endif  // ARM_CELL_VISION__DETECT_TARGET_ACQUISITION_HPP_
