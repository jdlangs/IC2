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
class IntrinsicCalibration
{
public:
  IntrinsicCalibration(const ObservationData &observation_data, const Target &target,
    const std::vector<Pose6D> link_poses);

  ~IntrinsicCalibration(void);

  bool Calibrate(void);

private:
  ObservationData observation_data_;
  Target target_;
  std::vector<Pose6D> link_poses_;
  double initial_cost_;
  double final_cost_;
};

} // namespace industrial_calibration_libs

#endif