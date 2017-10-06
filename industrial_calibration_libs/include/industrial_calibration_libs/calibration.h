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

class CalibrationJob
{
public:
  CalibrationJob(const ObservationData &observation_data, const Target &target);

  ~CalibrationJob(void) { }

  double getInitialCost(void) {return initial_cost_;}
  
  double getFinalCost(void) {return final_cost_;}

  void setOutput(bool output) {output_results_ = output;}

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

  bool output_results_;
};
} // namespace industrial_calibration_libs
#endif // CALIBRATION_H