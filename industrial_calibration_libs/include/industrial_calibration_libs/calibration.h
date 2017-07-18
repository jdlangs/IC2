#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <industrial_calibration_libs/cost_functions.h>
#include <industrial_calibration_libs/observations.h>
#include <industrial_calibration_libs/targets.h>

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

  ~CalibrationJob(void);

  virtual void initKnownValues(void) = 0;

  virtual void initSeedValues(void) = 0;

  virtual bool runCalibration(void) = 0;

  virtual void displayCovariance(void) = 0;

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

class MovingCameraOnWristStaticTargetExtrinsic : public CalibrationJob
{
public:
  struct Result
  {
    double extrinsics[6];
    double target_to_base[6];
  };

  using CalibrationJob::initKnownValues;
  using CalibrationJob::initSeedValues;

  MovingCameraOnWristStaticTargetExtrinsic(const ObservationData &observation_data,
    const Target &target);

  ~MovingCameraOnWristStaticTargetExtrinsic() { }

  virtual void initKnownValues(const std::vector<Pose6D> &link_poses, 
    const double intrinsics[4]);

  virtual void initSeedValues(const double extrinsics[6], 
    const double target_to_base[6]);

  virtual bool runCalibration(void);

  virtual void displayCovariance(void);

  Result getResult(void) {return result_;}

private:
  double intrinsics_[4];
  Result result_;
};

class MovingCameraOnWristStaticTargetIntrinsic : public CalibrationJob
{
public:
  struct Result
  {
    double extrinsics[6];
    double target_to_base[6];
    double intrinsics[9];
  };

  using CalibrationJob::initKnownValues;
  using CalibrationJob::initSeedValues;

  MovingCameraOnWristStaticTargetIntrinsic(const ObservationData &observation_data,
    const Target &target);

  ~MovingCameraOnWristStaticTargetIntrinsic() { }

  virtual void initKnownValues(const std::vector<Pose6D> &link_poses);

  virtual void initSeedValues(const double extrinsics[6], 
    const double target_to_base[6], const double intrinsics[9]);

  virtual bool runCalibration(void);

  virtual void displayCovariance(void);

  Result getResult(void) {return result_;}

private:
  Result result_;
};
} // namespace industrial_calibration_libs

#endif