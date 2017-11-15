#ifndef RESEARCH_INTRINSIC_H
#define RESEARCH_INTRINSIC_H

#include <industrial_calibration_libs/calibration.h>
namespace industrial_calibration_libs
{
struct ResearchIntrinsicParams
{
  // Unknown Values
  IntrinsicsFull intrinsics;
  std::vector<Extrinsics> target_to_camera_seed;
};

class ResearchIntrinsic : public CalibrationJob
{
public:
  struct Result
  {
    double intrinsics[9];
    std::vector<Extrinsics> target_to_camera_poses;
  };

  ResearchIntrinsic(const ObservationData &observation_data,
    const Target &target, const ResearchIntrinsicParams &params);

  ~ResearchIntrinsic(void) { }

  bool runCalibration(void);

  bool displayCovariance(void);

  Result getResults(void) {return result_;}

private:
  bool findDistortedTarget(const ObservationPoints &observation_points,
    Pose6D &position, double intrinsics[9], double guess_pose[6]);

  Result result_;
};
} // namespace industrial_calibration_libs
#endif // RESEARCH_INTRINSIC_H
