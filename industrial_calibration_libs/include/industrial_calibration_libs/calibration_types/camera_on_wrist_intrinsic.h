#ifndef CAMERA_ON_WRIST_INTRINSIC_H
#define CAMERA_ON_WRIST_INTRINSIC_H

#include <industrial_calibration_libs/calibration.h>

namespace industrial_calibration_libs
{
struct IntrinsicsVerification
{
  double target_diff_x; // m
  double tool_diff_x; // m
  double absolute_error_x; // m

  double target_diff_y; // m
  double tool_diff_y; // m
  double absolute_error_y; // m

  double target_diff_z; // m
  double tool_diff_z; // m
  double absolute_error_z; // m
};

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

  IntrinsicsVerification verifyIntrinsics(const ObservationPoints &observation_1,
    const Pose6D &pose_1, const ObservationPoints &observation_2, const Pose6D &pose_2,
    double intrinsics[9], double target_guess[6]);

private:
  bool runCalculateFirstPose(void);

  bool findDistortedTarget(const ObservationPoints &observation_points,
    Pose6D &position, double intrinsics[9], double guess_pose[6]);

  Result result_;
};
} // namespace industrial_calibration_libs
#endif // CAMERA_ON_WRIST_INTRINSIC_H