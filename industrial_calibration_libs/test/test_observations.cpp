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

    // TODO(gChiou): Do this inside of observations???
    image = image > 128;

    calibration_images.push_back(image);
  }


  // Load in target
  industrial_calibration_libs::Target target;
  target.loadTargetFromYAML("cal_targets/mcircles_7x5.yaml");

  // Create Observation Extractor Object
  industrial_calibration_libs::ObservationExtractor observation_extractor(calibration_images, target);
  observation_extractor.extractObservations();

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData();

  CONSOLE_OUTPUT("Total Observations: " << observation_data.size());
  for (std::size_t i = 0; i < observation_data.size(); i++)
  {
    CONSOLE_OUTPUT("Observation #" << i << " Size: " << observation_data[i].size());
    CONSOLE_OUTPUT("Observation #" << i << " Points:");
    for (std::size_t j = 0; j < observation_data[i].size(); j++)
    {
      CONSOLE_OUTPUT(observation_data[i][j]);
    }
  }
}
