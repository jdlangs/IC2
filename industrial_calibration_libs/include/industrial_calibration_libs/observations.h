#ifndef OBSERVATIONS_H
#define OBSERVATIONS_H

#include <industrial_calibration_libs/targets.h>
#include <industrial_calibration_libs/types.h>
#include <industrial_calibration_libs/circle_detector.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv_modules.hpp>

namespace industrial_calibration_libs
{
typedef std::vector<cv::Point2d> ObservationPoints;

typedef std::vector<ObservationPoints> ObservationData;

class ObservationExtractor
{
public:
  ObservationExtractor(const Target &target, bool custom_circle_detector = true);

  ~ObservationExtractor(void) { }

  ObservationData getObservationData(void) {return observation_data_;}

  bool extractObservation(const cv::Mat &image, cv::Mat &output_image);

private:
  template<typename PARAMS, typename DETECTOR_PTR, typename DETECTOR>
  bool extractModifiedCircleGrid(const cv::Mat &image, 
    ObservationPoints &observation_points, cv::Mat &output_image);

  template<typename DETECTOR_PTR>
  bool extractKeyPoints(const ObservationPoints &centers,
    ObservationPoints &observation_points, cv::Ptr<DETECTOR_PTR> &detector_ptr, 
    std::size_t rows, std::size_t cols, bool flipped,
    const cv::Mat &image);

  // Data Members
  Target target_;

  std::vector<cv::Mat> images_;
  std::vector<cv::Mat> grid_images_;

  ObservationData observation_data_;
  bool custom_circle_detector_;

  std::size_t target_cols_;
  std::size_t target_rows_;
};

} // namespace industrial_calibration_libs
#endif // OBSERVATIONS_H

