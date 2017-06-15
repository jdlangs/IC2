#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <gtest/gtest.h>
#include "test_utils.h"

#include <string>
#include <limits.h>
#include <unistd.h>

TEST(Observations, load_observations)
{
  // Load in calibration images
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(5);
  std::string cal_image_path = "../../src/IC2/industrial_calibration_libs/test/res/cal_images/mcircles_7x5/";

  // CONSOLE_OUTPUT("Observation Path: " << getExecutionPath());

  for (std::size_t i = 0; i < 5; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i+1) + ".jpg";
    CONSOLE_OUTPUT(image_path);
    cv::Mat image = cv::imread(cal_image_path, CV_LOAD_IMAGE_COLOR);

    // ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    calibration_images.push_back(image);
  }

  cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    cv::imshow("Display window", calibration_images[i]);
    cv::waitKey(0);    
  }

  EXPECT_TRUE(true);
}
