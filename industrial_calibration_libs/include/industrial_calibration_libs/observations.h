#ifndef OBSERVATIONS_H
#define OBSERVATIONS_H

#include <industrial_calibration_libs/targets.h>

namespace industrial_calibration_libs
{
struct Observation
{
  std::shared_ptr<Target> target;
  int point_id;
  double point_x;
  double point_y;
  // Note(gChiou): May need cost function type associated with observation and
  // an 'intermediate_frame'. Leaving it out for now.
  // see: https://github.com/ros-industrial/industrial_calibration/blob/kinetic-devel/industrial_extrinsic_cal/include/industrial_extrinsic_cal/camera_observer.hpp
};

typedef std::vector<Observation> CameraObservations;

// Note(gChiou): I think the above typedef is for a single image.
// The input of the ObservationExtractor class should be a vector of 
// all images containing a target and the output should be a vector of:
// typedef std::vector<CameraObservations> ObservationData 
// (or something like that) of the same size as input vector of images.
// Something to test for.
typedef std::vector<CameraObservations> ObservationData;

class ObservationExtractor
{
public:
  ObservationExtractor(void);

  ~ObservationExtractor(void);

  bool getObservations(CameraObservations &observations);

private:
};

} // namespace industrial_calibration_libs
#endif // OBSERVATIONS_H