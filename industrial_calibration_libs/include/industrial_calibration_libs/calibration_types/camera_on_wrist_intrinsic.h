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
  Extrinsics target_to_camera;
};

class CameraOnWristIntrinsic : public CalibrationJob
{
public:
  struct Result
  {
    double target_to_camera[6];
    double intrinsics[9];
  };

  CameraOnWristIntrinsic(const ObservationData &observation_data,
    const Target &target, const CameraOnWristIntrinsicParams &params);

  ~CameraOnWristIntrinsic(void) { }

  bool runCalibration(void);

  void displayCovariance(void);

  Result getResults(void) {return result_;}

private:
  bool runCalculateFirstPose(void);

  bool findDistortedTarget(const ObservationPoints &observation_points,
    Pose6D &position);

  Result result_;
};
} // namespace industrial_calibration_libs
#endif // CAMERA_ON_WRIST_INTRINSIC_H