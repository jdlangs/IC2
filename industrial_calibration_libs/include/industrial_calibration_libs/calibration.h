#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <industrial_calibration_libs/cost_functions.h>
#include <industrial_calibration_libs/observations.h>
#include <industrial_calibration_libs/targets.h>

// Note(gChiou): Name for other classes
// CameraToBaseExtrinsic
// CameraToBaseIntrinsic

namespace industrial_calibration_libs
{
enum CovarianceRequestType
{
  IntrinsicParams,
  ExtrinsicParams,
  TargetPoseParams,
};

struct CovarianceRequest
{
  CovarianceRequestType request_type;
  std::string object_name;
};

struct CameraOnWristExtrinsicParams 
{
  // Known Values
  IntrinsicsPartial intrinsics;
  std::vector<Pose6D> base_to_tool; // For every observation.

  // Unknown Values
  Extrinsics tool_to_camera;
  Extrinsics target_to_base;
};

struct CameraOnWristIntrinsicParams
{
  // Known Values
  std::vector<Pose6D> base_to_tool; // For every observation.

  // Uknown Values
  IntrinsicsFull intrinsics;
  Extrinsics tool_to_camera;
  Extrinsics target_to_base;
};

class CalibrationJob
{
public:
  CalibrationJob(const ObservationData &observation_data, const Target &target);

  ~CalibrationJob(void) { }

  double getInitialCost(void) {return initial_cost_;}
  
  double getFinalCost(void) {return final_cost_;}

protected:
  bool checkObservations(void);

  bool computeCovariance(const std::vector<CovarianceRequest> &requests,
    double* extrinsics_result = 0, double* target_to_base_result = 0, 
    double* intrinsics_result = 0);

  std::size_t num_images_;
  std::size_t observations_per_image_;
  std::size_t total_observations_;

  ceres::Problem problem_;
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;

  std::vector<Pose6D> link_poses_;
  ObservationData observation_data_;
  Target target_;

  double initial_cost_;
  double final_cost_;
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
    const Target &target const MCOWSTE_Params &params);

  ~CameraOnWristExtrinsic(void) { }

  bool runCalibration(void);

  void displayCovariance(void);

  Result getResults(void) {return result_;}

private:
  double intrinsics_[4];
  Result result_;
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
    const Target &target, const MCOWSTI_Params &params);

  ~CameraOnWristIntrinsic(void) { }

  bool runCalibration(void);

  void displayCovariance(void);

  Result getResults(void) {return result_;}

private:
  Result result_;
};
} // namespace industrial_calibration_libs
#endif