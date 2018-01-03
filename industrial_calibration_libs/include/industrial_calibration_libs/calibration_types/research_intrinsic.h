#ifndef RESEARCH_INTRINSIC_H
#define RESEARCH_INTRINSIC_H

#include <industrial_calibration_libs/calibration.h>
namespace industrial_calibration_libs
{
struct IntrinsicsVerificationResearch
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

  ~ResearchIntrinsic(void) 
  { 
    delete[] extrinsics_;
  }

  bool runCalibration(void);

  IntrinsicsVerificationResearch verifyIntrinsics(const ObservationPoints &observation_1,
    const Pose6D &pose_1, const ObservationPoints &observation_2, const Pose6D &pose_2,
    double intrinsics[9], double target_guess[6]);

  void displayCovariance(void);

  Result getResults(void) {return result_;}

private:
  bool findDistortedTarget(const ObservationPoints &observation_points,
    Pose6D &position, double intrinsics[9], double guess_pose[6]);

  Result result_;
  double* extrinsics_;
};

class ResearchIntrinsicTheory : public CalibrationJob
{
public:
  struct Result
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"  
    union
    {
      double intrinsics[9];
      struct
      {
        double camera_matrix[4];
        double distortion_k[3];
        double distortion_p[2];     
      };
    };
#pragma GCC diagnostic pop
    std::vector<Extrinsics> target_to_camera_poses;
  };

  ResearchIntrinsicTheory(const ObservationData &observation_data,
    const Target &target, const ResearchIntrinsicParams &params);

  ~ResearchIntrinsicTheory(void) { }

  bool runCalibration(void);

  void displayCovariance(void) { }

  void displayCovarianceA(void);

  void displayCovarianceB(void);

  Result getResults(void) {return result_;}

private:
  bool findDistortedTarget(const ObservationPoints &observation_points,
    Pose6D &position, double intrinsics[9], double guess_pose[6]);

  Result result_;
  double* extrinsics_;
};

} // namespace industrial_calibration_libs
#endif // RESEARCH_INTRINSIC_H
