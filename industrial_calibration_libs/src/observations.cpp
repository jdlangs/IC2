#include <industrial_calibration_libs/observations.h>

namespace industrial_calibration_libs
{
ObservationExtractor::ObservationExtractor(const std::vector<cv::Mat> &images,
  const Target &target) : images_(images), target_(target), custom_circle_detector_(true) 
{
  // Threshold images
  for (std::size_t i = 0; i < images_.size(); i++)
  {
    images_[i] = images_[i] > 128;
  }
}

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
      break;

    case ModifiedCircleGrid:
      if (extractModifiedCircleGrid()) {return true;}

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

  // pragma omp parallel for
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
    // pragma omp parallel for
    for (std::size_t i = 0; i < images_.size(); i++)
    {
      ObservationPoints observation_points;
      // Try regular pattern_size
      if (cv::findCirclesGrid(images_[i], pattern_size, observation_points, cv::CALIB_CB_SYMMETRIC_GRID, circle_detector_ptr))
      {
        observation_data_[i] = observation_points;
      }
      else // Try flipped pattern size
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
    // pragma omp parallel for
    for (std::size_t i = 0; i < images_.size(); i++)
    {
      ObservationPoints observation_points;
      // Try regular pattern_size
      if (cv::findCirclesGrid(images_[i], pattern_size, observation_points, cv::CALIB_CB_SYMMETRIC_GRID))
      {
        observation_data_[i] = observation_points;
      }
      else // Try flipped pattern size
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
    // pragma omp parallel for
    for (std::size_t i = 0; i < images_.size(); i++)
    {
      ObservationPoints observation_points;
      // Try regular pattern_size
      if (cv::findCirclesGrid(images_[i], pattern_size, observation_points, cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, circle_detector_ptr))
      {
        observation_data_[i] = observation_points;
      }
      else // Try flipped pattern size
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
    // pragma omp parallel for
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
  cv::Ptr<cv::CircleDetector> circle_detector_ptr = cv::CircleDetector::create();
  cv::SimpleBlobDetector::Params simple_blob_params;
  cv::Ptr<cv::FeatureDetector> blob_detector_ptr = cv::SimpleBlobDetector::create(simple_blob_params);

  // Note(gChiou): Keep track of which images are flipped
  // Start by setting all to false.
  std::vector<bool> flipped;
  if (images_.size() > 0) 
  {
    flipped.resize(images_.size());
    for (std::size_t i = 0; i < images_.size(); i++)
    {
      flipped[i] = false;
    }
  }

  std::size_t cols = target_.getData()->target_cols;
  std::size_t rows = target_.getData()->target_rows;

  observation_data_.clear();
  observation_data_.resize(images_.size());

  cv::Size pattern_size(cols, rows);
  cv::Size pattern_size_flipped(rows, cols);

  ObservationData center_data;
  center_data.resize(images_.size());

  if (custom_circle_detector_)
  {
    // pragma omp parallel for
    for (std::size_t i = 0; i < images_.size(); i++)
    {
      ObservationPoints centers;
      // Try regular pattern size
      if (cv::findCirclesGrid(images_[i], pattern_size, centers, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, circle_detector_ptr))
      {
        center_data[i] = centers;
      }
      else // Try flipped pattern size
      {
        if (cv::findCirclesGrid(images_[i], pattern_size_flipped, centers, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, circle_detector_ptr))
        {
          center_data[i] = centers;
          flipped[i] = true;
        }
      }
    }
    // Note(gChiou): Check if any images failed to return observations
    for (std::size_t i = 0; i < center_data.size(); i++)
    {
      if (center_data[i].size() == 0) {return false;}
    }
  }

  else
  {
    // pragma omp parallel for
    for (std::size_t i = 0; i < images_.size(); i++)
    {
      ObservationPoints centers;
      // Try regular pattern size
      if (cv::findCirclesGrid(images_[i], pattern_size, centers, cv::CALIB_CB_SYMMETRIC_GRID))
      {
        center_data[i] = centers;
      }
      else // Try pattern_size_flipped
      {
        if (cv::findCirclesGrid(images_[i], pattern_size_flipped, centers, cv::CALIB_CB_SYMMETRIC_GRID))
        {
          center_data[i] = centers;
          flipped[i] = true;
        }
      }
    }

    // Note(gChiou): Check if any images failed to return observations
    for (std::size_t i = 0; i < center_data.size(); i++)
    {
      if (center_data[i].size() == 0) {return false;}
    }
  }

  // Extract KeyPoints
  // Note(cLewis): This is the same method called in the beginning of findCirclesGrid,
  // unfortunately, they don't return their keypoints. If OpenCV changes, the keypoint
  // locations may not match, which has the risk of failing with updates to OpenCV.
  std::vector<cv::KeyPoint> keypoints; // May not need this
  std::vector<std::vector<cv::KeyPoint>> keypoint_vector;
  keypoint_vector.resize(images_.size());
  std::vector<cv::Point> large_point;
  large_point.resize(images_.size());

  std::size_t start_first_row = 0;
  std::size_t end_first_row = cols-1;
  std::size_t start_last_row = rows*cols - cols;
  std::size_t end_last_row = rows*cols - 1;

  for (std::size_t i = 0; i < images_.size(); i++)
  {
    if (custom_circle_detector_)
    {
      circle_detector_ptr->detect(images_[i], keypoint_vector[i]);
    }
    else
    {
      blob_detector_ptr->detect(images_[i], keypoint_vector[i]);
    }

    // If a flipped pattern is found, flip the rows/columns
    std::size_t temp_rows = flipped[i] ? cols : rows;
    std::size_t temp_cols = flipped[i] ? rows : cols;

    // Determine which circle is the largest
    // TODO(gChiou): WHY ARE THESE DOUBLES???
    double start_first_row_size = -1.0;
    double start_last_row_size = -1.0;
    double end_first_row_size = -1.0;
    double end_last_row_size = -1.0;

    for (std::size_t j = 0; j < keypoint_vector[i].size(); j++)
    {
      double x = keypoint_vector[i][j].pt.x;
      double y = keypoint_vector[i][j].pt.y;
      double ksize = keypoint_vector[i][j].size;

      if (x == center_data[i][start_last_row].x && y == center_data[i][start_last_row].y) 
      {
        start_last_row_size = ksize;
      }
      if (x == center_data[i][end_last_row].x && y == center_data[i][end_last_row].y)
      {
        end_last_row_size = ksize;
      }
      if (x == center_data[i][start_first_row].x && y == center_data[i][start_first_row].y)
      {
        start_first_row_size = ksize;
      }
      if (x == center_data[i][end_first_row].x && y == center_data[i][end_first_row].y)
      {
        end_first_row_size = ksize;
      }
    }

    // No keypoint match for one or more corners
    if (start_last_row_size < 0.0 || start_first_row_size < 0.0 ||
    end_last_row_size < 0.0 || end_first_row_size < 0.0)
    {
      return false;
    }

    // Note(cLewis): Determine if ordering is usual by computing cross product of two vectors
    // normal ordering has z axis positive in cross
    // The most common ordering is with points going from left to right then to to bottom
    bool usual_ordering = true;
    double v1x, v1y, v2x, v2y;

    v1x = center_data[i][end_last_row].x - center_data[i][start_last_row].x;
    v1y = -center_data[i][end_last_row].y + center_data[i][start_last_row].y;
    v2x = center_data[i][end_first_row].x - center_data[i][end_last_row].x;
    v2y = -center_data[i][end_first_row].y + center_data[i][end_last_row].y;

    double cross = v1x*v2y - v1y*v2x;
    if (cross < 0.0)
    {
      usual_ordering = false;
    }

    ObservationPoints observation_points;

    // Note(cLewis): Largest circle at start of last row
    // ......
    // ......
    // o.....
    // This is a simple picture of the grid with the largest circle indicated
    // by the letter o.
    if (start_last_row_size > start_first_row_size && 
      start_last_row_size > end_first_row_size && 
      start_last_row_size > end_last_row_size)
    {
      large_point[i].x = center_data[i][start_last_row].x;
      large_point[i].y = center_data[i][start_last_row].y;
      if (usual_ordering)
      {
        for (std::size_t j = 0; j < center_data[i].size(); j++)
        {
          observation_points.push_back(center_data[i][j]);
        }
      }
      else // unusual-ordering
      {
        for (int j = temp_cols - 1; j >= 0; j--)
        {
          for (int k = temp_rows - 1; k >= 0; k--)
          {
            // Note(gChiou): I have warnings turned on >:[
            observation_points.push_back(center_data[i][static_cast<std::size_t>(k)*temp_cols + static_cast<std::size_t>(j)]);
          }
        }
      }
    }

    // Largest circle at end of first row
    // .....o
    // ......
    // ......
    else if (end_first_row_size > end_last_row_size &&
      end_first_row_size > start_last_row_size &&
      end_first_row_size > start_first_row_size)
    {
      large_point[i].x = center_data[i][end_first_row].x;
      large_point[i].y = center_data[i][end_first_row].y;
      if (usual_ordering)
      {
        for (std::size_t j = 0; j < center_data[i].size(); j++)
        {
          observation_points.push_back(center_data[i][j]);
        }
      }
      else // Unusual Ordering
      {
        for (std::size_t j = 0; j < temp_cols; j++)
        {
          for (std::size_t k = 0; i < temp_rows; k++)
          {
            observation_points.push_back(center_data[i][k*temp_cols + j]);
          }
        }
      }
    }

    // Largest circle at end of last row
    // ......
    // ......
    // .....o
    else if (end_last_row_size > start_last_row_size &&
      end_last_row_size > end_first_row_size &&
      end_last_row_size > start_first_row_size)
    {
      large_point[i].x = center_data[i][end_last_row].x;
      large_point[i].y = center_data[i][end_last_row].y;
      if (usual_ordering)
      {
        for (std::size_t j = 0; j < temp_cols; j++)
        {
          for (int k = temp_rows - 1; k >= 0; k--)
          {
            observation_points.push_back(center_data[i][static_cast<std::size_t>(k)*temp_cols + j]);
          }
        }
      }
      else // Unusual Ordering
      {
        for (std::size_t j = 0; j < temp_cols; j++)
        {
          for (std::size_t k = 0; k < temp_rows; k++)
          {
            observation_points.push_back(center_data[i][k*temp_cols + j]);
          }
        }
      }
    }

    // Largest circle at start of first row
    // o.....
    // ......
    // ......
    else if (start_first_row_size > end_last_row_size &&
      start_first_row_size > end_first_row_size &&
      start_first_row_size > start_last_row_size)
    {
      large_point[i].x = center_data[i][start_first_row].x;
      large_point[i].y = center_data[i][start_first_row].y;
      if (usual_ordering)
      {
        for (int j = temp_cols - 1; j >= 0; j--)
        {
          for (std::size_t k = 0; k < temp_rows; k++)
          {
            observation_points.push_back(center_data[i][k*temp_cols + static_cast<std::size_t>(j)]);
          }
        }
      }
      else // Unusual Ordering
      {
        for (int j = temp_cols - 1; j >= 0; j--)
        {
          for (int k = temp_rows - 1; k >= 0; k--)
          {
            observation_points.push_back(center_data[i][static_cast<std::size_t>(k)*temp_cols + static_cast<std::size_t>(j)]);
          }
        }
      }
    }

    else
    {
      return false;
    }
    observation_data_[i] = observation_points;
  }

  return true;
}

} // namespace industrial_calibration_libs