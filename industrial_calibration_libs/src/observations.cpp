#include <industrial_calibration_libs/observations.h>

namespace industrial_calibration_libs
{

ObservationExtractor::ObservationExtractor(const std::vector<cv::Mat> &images,
  const Target &target) : images_(images), target_(target) { }

bool ObservationExtractor::extractObservations(void)
{
  if (!checkData()) {return false;}

  switch (target_.getData()->target_type)
  {
    case Chessboard:
      if (extractChessboard()) {return true;}
      break;

    case CircleGrid:
      if (extractCircleGrid()) {return true;}
      break;

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

bool ObservationExtractor::extractCircleGrid(void)
{
  // Note(gChiou): I may not keept his in...
  if (target_.getData()->asymmetric_grid)
  {
    
  }

  else
  {

  }
  // TODO(gChiou): Temporary (return false)
  return false;
}

bool ObservationExtractor::extractModifiedCircleGrid(void)
{

  // TODO(gChiou): Temporary (return false)
  return false;
}

} // namespace industrial_calibration_libs