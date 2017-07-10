#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <industrial_calibration_libs/cost_functions.h>
#include <industrial_calibration_libs/observations.h>
#include <industrial_calibration_libs/targets.h>

namespace industrial_calibration_libs
{
// Note(gChiou): Going to leave this out for now and focus on getting intrinsic
// calibration to work.

// class CalibrationJob
// {
// public:
//   CalibrationJob(const ObservationData &observation_data, const Target &target);

//   ~CalibrationJob();

//   virtual bool calibrate();

// protected:
//   ObservationData observation_data_;
//   Target target_;
// };

// class CalibrateCameraOnWristStaticTargetExtrinsic : public CalibrationJob
// {
// public:
// private:
//   struct Results
//   {

//   };
// };

// class CalibrateCameraOnWristStaticTargetIntrinsic : public CalibrationJob
// {
// public:
// private:
//   struct Results
//   {

//   };
// };

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
  double initial_cost_;
  double final_cost_;
};

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

  bool calibrate(void);

  IntrinsicResults getResults(void);

private:
  void setResults(const double extrinsics[6], const double intrinsics[9],
    const double target_to_world[6]);

  IntrinsicResults results_;
  ObservationData observation_data_;
  Target target_;
  std::vector<Pose6D> link_poses_;
  double initial_cost_;
  double final_cost_;
};

} // namespace industrial_calibration_libs

#endif