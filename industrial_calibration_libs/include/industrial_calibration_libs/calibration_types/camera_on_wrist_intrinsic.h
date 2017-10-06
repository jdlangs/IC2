#ifndef CAMERA_ON_WRIST_INTRINSIC_H
#define CAMERA_ON_WRIST_INTRINSIC_H

#include <industrial_calibration_libs/calibration.h>

namespace industrial_calibration_libs
{
struct CameraOnWristIntrinsicParams
{
  // Known Values
  std::vector<Pose6D> base_to_tool; // For every observation.

  // Uknown Values
  IntrinsicsFull intrinsics;
  Extrinsics tool_to_camera;
  Extrinsics target_to_base;
};

class CameraOnWristIntrinsic : public CalibrationJob
{
public:
  struct Result
  {
    double extrinsics[6];
    double target_to_base[6];
    double intrinsics[9];
  };

  CameraOnWristIntrinsic(const ObservationData &observation_data,
    const Target &target, const CameraOnWristIntrinsicParams &params);

  ~CameraOnWristIntrinsic(void) { }

  bool runCalibration(void);

  void displayCovariance(void);

  Result getResults(void) {return result_;}

private:
  Result result_;
};
} // namespace industrial_calibration_libs
#endif // CAMERA_ON_WRIST_INTRINSIC_H