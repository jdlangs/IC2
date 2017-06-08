#include <industrial_calibration_libs/observations.h>

namespace industrial_calibration_libs
{
ObservationExtractor::ObservationExtractor(const std::vector<cv::Mat> &images,
  const Target &target) : images_(images), target_(target), custom_circle_detector_(true) { }

bool ObservationExtractor::extractObservations(void)
{
  if (!checkData()) {return false;}

  switch (target_.getData()->target_type)
  {
    case Chessboard:
      if (extractChessboard()) {return true;}
      break;

    case CircleGrid:
      if (target_.getData()->asymmetric_grid)
      {
        if (extractCircleGridAsymmetric()) {return true;}
      }
      else
      {
        if (extractCircleGridSymmetric()) {return true;}
      }

    case ModifiedCircleGrid:
      if (extractModifiedCircleGrid()) {return true;};
      break;

    default:
      break;
  }
  return false;
}

bool ObservationExtractor::checkData(void) const
{
  // Check if image vector is empty
  if (images_.size() == 0) {return false;}

  // Checks if any of the images have no data
  for (std::size_t i = 0; i < images_.size(); i++)
  {
    if (images_[i].empty()) {return false;}
  }

  // TODO(gChiou): Add more checks
  return true;
}

bool ObservationExtractor::extractChessboard(void)
{
  std::size_t cols = target_.getData()->target_cols;
  std::size_t rows = target_.getData()->target_rows;

  observation_data_.clear();
  observation_data_.resize(images_.size());

  cv::Size pattern_size(cols, rows); // CV uses (cols, rows)

  #pragma omp parallel for
  for (std::size_t i = 0; i < images_.size(); i++)
  {
    ObservationPoints observation_points;
    if (cv::findChessboardCorners(images_[i], pattern_size, observation_points,
      cv::CALIB_CB_ADAPTIVE_THRESH))
    {
      observation_data_[i] = observation_points;
    }
  }

  // Note(gChiou): Checks if any of the images failed to return observations.
  for (std::size_t i = 0; i < observation_data_.size(); i++)
  {
    if (observation_data_[i].size() == 0) {return false;}
  }
  return true;
}

bool ObservationExtractor::extractCircleGridSymmetric(void)
{
  cv::Ptr<cv::CircleDetector> circle_detector_ptr = cv::CircleDetector::create();

  std::size_t cols = target_.getData()->target_cols;
  std::size_t rows = target_.getData()->target_rows;

  observation_data_.clear();
  observation_data_.resize(images_.size());

  cv::Size pattern_size(cols, rows); // CV uses (cols, rows)
  cv::Size pattern_size_flipped(rows, cols);

  // TODO(gChiou): Rethink this... may have to fix other ones as well.
  // Should it be flipping the pattern half way?
  if (custom_circle_detector_)
  {
    #pragma omp parallel for
    for (std::size_t i = 0; i < images_.size(); i++)
    {
      ObservationPoints observation_points;
      // Try regular pattern_size
      if (cv::findCirclesGrid(images_[i], pattern_size, observation_points, cv::CALIB_CB_SYMMETRIC_GRID, circle_detector_ptr))
      {
        observation_data_[i] = observation_points;
      }
      else // Try flipped_pattern_size
      {
        if (cv::findCirclesGrid(images_[i], pattern_size_flipped, observation_points, cv::CALIB_CB_SYMMETRIC_GRID, circle_detector_ptr))
        {
          observation_data_[i] = observation_points;
        }
      }
    }

    // Note(gChiou): Check if any images failed to return observations
    for (std::size_t i = 0; i < observation_data_.size(); i++)
    {
      if (observation_data_[i].size() == 0) {return false;}
    }
    return true;
  }

  else
  {
    #pragma omp parallel for
    for (std::size_t i = 0; i < images_.size(); i++)
    {
      ObservationPoints observation_points;
      // Try regular pattern_size
      if (cv::findCirclesGrid(images_[i], pattern_size, observation_points, cv::CALIB_CB_SYMMETRIC_GRID))
      {
        observation_data_[i] = observation_points;
      }
      else // Try flipped_pattern_size
      {
        if (cv::findCirclesGrid(images_[i], pattern_size_flipped, observation_points, cv::CALIB_CB_SYMMETRIC_GRID))
        {
          observation_data_[i] = observation_points;
        }
      }
    }

    // Note(gChiou): Check if any images failed to return observations
    for (std::size_t i = 0; i < observation_data_.size(); i++)
    {
      if (observation_data_[i].size() == 0) {return false;}
    }
    return true;    
  }
}

bool ObservationExtractor::extractCircleGridAsymmetric(void)
{
  cv::Ptr<cv::CircleDetector> circle_detector_ptr = cv::CircleDetector::create();

  std::size_t cols = target_.getData()->target_cols;
  std::size_t rows = target_.getData()->target_rows;

  observation_data_.clear();
  observation_data_.resize(images_.size());

  cv::Size pattern_size(cols, rows); // CV uses (cols, rows)
  cv::Size pattern_size_flipped(rows, cols);
  
  if (custom_circle_detector_)
  {
    #pragma omp parallel for
    for (std::size_t i = 0; i < images_.size(); i++)
    {
      ObservationPoints observation_points;
      // Try regular pattern_size
      if (cv::findCirclesGrid(images_[i], pattern_size, observation_points, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, circle_detector_ptr))
      {
        observation_data_[i] = observation_points;
      }
      else // Try flipped_pattern_size
      {
        if (cv::findCirclesGrid(images_[i], pattern_size_flipped, observation_points, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, circle_detector_ptr))
        {
          observation_data_[i] = observation_points;
        }
      }
    }

    // Note(gChiou): Check if any images failed to return observations
    for (std::size_t i = 0; i < observation_data_.size(); i++)
    {
      if (observation_data_[i].size() == 0) {return false;}
    }
    return true;
  }

  else
  {
    #pragma omp parallel for
    for (std::size_t i = 0; i < images_.size(); i++)
    {
      ObservationPoints observation_points;
      // Try regular pattern size
      if (cv::findCirclesGrid(images_[i], pattern_size, observation_points, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING))
      {
        observation_data_[i] = observation_points;
      }
      else // Try flipped pattern size
      {
        if (cv::findCirclesGrid(images_[i], pattern_size_flipped, observation_points, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING))
        {
          observation_data_[i] = observation_points;
        }
      }
    }

    // Note(gChiou): Check if any images failed to return observations
    for (std::size_t i = 0; i < observation_data_.size(); i++)
    {
      if (observation_data_[i].size() == 0) {return false;}
    }
    return true;
  }
}

bool ObservationExtractor::extractModifiedCircleGrid(void)
{

  // TODO(gChiou): Temporary (return false)
  return false;
}

} // namespace industrial_calibration_libs