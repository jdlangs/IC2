#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <industrial_calibration_libs/cost_functions.h>
#include <industrial_calibration_libs/observations.h>
#include <industrial_calibration_libs/targets.h>

namespace industrial_calibration_libs
{
class CalibrationJob
{
public:
  virtual bool initKnownValues(void);

  virtual bool initSeedValues(void);

  virtual bool calibrate(void);

  bool computeCovariance(void);

protected:
  ceres::Problem problem_;
  std::vector<Pose6D> link_poses_;
  ObservationData observation_data_;
  Target target_;

  double initial_cost_;
  double final_cost_;
};

class MovingCameraOnWristStaticTargetExtrinsic : public CalibrationJob
{
public:
  struct Results
  {
    double extrinsics[6];
    double target_to_base[6];
  };

  using CalibrationJob::initKnownValues;
  using CalibrationJob::initSeedValues;

  MovingCameraOnWristStaticTargetExtrinsic(const ObservationData &observation_data,
    const Target &target);

  ~MovingCameraOnWristStaticTargetExtrinsic() { }

  void initKnownValues(const std::vector<Pose6D> &link_poses, 
    const double intrinsics[4]);

  void initSeedValues(const double extrinsics[6], const double target_to_base[6]);

  bool calibrate(void);

  Results getResults(void) {return results_;}

private:
  double intrinsics_[4];
  double extrinsics_seed_[6];
  double target_to_base_seed_[6];
  Results results_;
};

class MovingCameraOnWristStaticTargetIntrinsic : public CalibrationJob
{
public:
  struct Results
  {
    double extrinsics[6];
    double intrinsics[9];
    double target_to_base[6];
  };

  using CalibrationJob::initKnownValues;
  using CalibrationJob::initSeedValues;

  MovingCameraOnWristStaticTargetIntrinsic(const ObservationData &observation_data,
    const Target &target);

  ~MovingCameraOnWristStaticTargetIntrinsic() { }

  void initKnownValues(const std::vector<Pose6D> &link_poses);

  void initSeedValues(const double extrinsics[6], const double target_to_base[6],
    const double intrinsics[9]);

  bool calibrate(void);

  Results getResults(void) {return results_;}

private:
  double extrinsics_seed_[6];
  double target_to_base_seed_[6];
  double intrinsics_seed_[9];
  Results results_;
};

#if 0
// Note(gChiou): This is temporary
struct ExtrinsicResults
{
  double extrinsics[6];
  double target_to_world[6];
};

class ExtrinsicCalibration
{
public:
  ExtrinsicCalibration(const ObservationData &observation_data, const Target &target,
    const std::vector<Pose6D> link_poses, const double intrinsics[4]);

  ~ExtrinsicCalibration(void) { }

  void setSeedValues(const double extrinsics[6], const double target_to_world[6]);

  bool calibrate(void);

  ExtrinsicResults getResults(void);

private:
  void setIntrinsics(const double intrinsics[4]);

  void setResults(const double extrinsics[6], const double target_to_world[6]);

  ExtrinsicResults results_;
  ObservationData observation_data_;
  Target target_;
  std::vector<Pose6D> link_poses_;
  double intrinsics_[4];
  double extrinsics_seed_[6];
  double target_to_world_seed_[6];
  double initial_cost_;
  double final_cost_;
};
#endif

#if 0
struct IntrinsicResults
{
  double extrinsics[6];
  double intrinsics[9];
  double target_to_world[6];
};

class IntrinsicCalibration
{
public:
  IntrinsicCalibration(const ObservationData &observation_data, const Target &target,
    const std::vector<Pose6D> link_poses);

  ~IntrinsicCalibration(void) { }

  void setSeedValues(const double extrinsics[6], const double target_to_world[6],
    const double intrinsics[9]);

  bool calibrate(void);

  IntrinsicResults getResults(void);

private:
  void setResults(const double extrinsics[6], const double intrinsics[9],
    const double target_to_world[6]);

  IntrinsicResults results_;
  ObservationData observation_data_;
  Target target_;
  std::vector<Pose6D> link_poses_;
  double extrinsics_seed_[6];
  double target_to_world_seed_[6];
  double intrinsics_seed_[9];
  double initial_cost_;
  double final_cost_;
};
#endif

} // namespace industrial_calibration_libs

#endif