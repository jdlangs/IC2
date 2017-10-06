#ifndef CAMERA_ON_WRIST_EXTRINSIC_H
#define CAMERA_ON_WRIST_EXTRINSIC_H

#include <industrial_calibration_libs/calibration.h>

namespace industrial_calibration_libs
{
struct CameraOnWristExtrinsicParams 
{
  // Known Values
  IntrinsicsPartial intrinsics;
  std::vector<Pose6D> base_to_tool; // For every observation.

  // Unknown Values
  Extrinsics tool_to_camera;
  Extrinsics target_to_base;
};

class CameraOnWristExtrinsic : public CalibrationJob
{
public:
  struct Result
  {
    double extrinsics[6];
    double target_to_base[6];
  };

  CameraOnWristExtrinsic(const ObservationData &observation_data,
    const Target &target, const CameraOnWristExtrinsicParams &params);

  ~CameraOnWristExtrinsic(void) { }

  bool runCalibration(void);

  void displayCovariance(void);

  Result getResults(void) {return result_;}

private:
  double intrinsics_[4];
  Result result_;
};
} // namespace industrial_calibration_libs
#endif // CAMERA_ON_WRIST_EXTRINSIC_H