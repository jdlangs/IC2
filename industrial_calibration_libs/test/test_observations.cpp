#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <gtest/gtest.h>
#include "test_utils.h"

TEST(Observations, load_observations)
{
  // Load in calibration images
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(5);
  std::string cal_image_path = "cal_images/mcircles_7x5/";

  for (std::size_t i = 0; i < 5; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i+1) + ".jpg";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    ASSERT_TRUE(!image.empty());
    ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    calibration_images.push_back(image);
  }


  // Load in target
  industrial_calibration_libs::Target target;
  target.loadTargetFromYAML("cal_targets/mcircles_7x5.yaml");

  // Create Observation Extractor Object
  industrial_calibration_libs::ObservationExtractor observation_extractor(calibration_images, target);
  observation_extractor.extractObservations();
}
