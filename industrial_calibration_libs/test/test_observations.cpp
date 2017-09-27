#include <test_utils.h>

TEST(Observations, load_observation_set_1)
{
  const std::size_t num_images = 15;

  // Load in calibration images
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = "mcircles_7x5/dataset_1/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i+1) + ".jpg";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    ASSERT_TRUE(!image.empty());
    ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    calibration_images.push_back(image);
  }

  // Load in target
  industrial_calibration_libs::Target target;
  target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5.yaml");

  // Create Observation Extractor Object
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    cv::Mat output_image;
    ASSERT_TRUE(observation_extractor.extractObservation(calibration_images[i], output_image));
#if 0
    cv::namedWindow("Image " + std::to_string(i+1), cv::WINDOW_NORMAL);
    cv::imshow("Image " + std::to_string(i+1), output_image);
    cv::waitKey(0);    
#endif
  }

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData();

  EXPECT_EQ(observation_data.size(), num_images);

  for (std::size_t i = 0; i < num_images; i++)
  {
    EXPECT_EQ(observation_data[i].size(), target.getDefinition().target_points);
  }

  // TODO(gChiou): Find a way to verify observation data...

#if 0
  CONSOLE_OUTPUT("Total Observations: " << observation_data.size());
  for (std::size_t i = 0; i < observation_data.size(); i++)
  {
    CONSOLE_OUTPUT("Observations for Image " << i+1 << " Size: " << observation_data[i].size());
    CONSOLE_OUTPUT("Observations for Image " << i+1 << " Points:");
    for (std::size_t j = 0; j < observation_data[i].size(); j++)
    {
      CONSOLE_OUTPUT(observation_data[i][j]);
    }
  }
#endif
}

TEST(Observations, load_observation_set_2)
{
  const std::size_t num_images = 24;

  // Load in calibration images
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = "mcircles_7x5/dataset_2/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i+1) + ".jpg";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    ASSERT_TRUE(!image.empty());
    ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    calibration_images.push_back(image);
  }

  // Load in target
  industrial_calibration_libs::Target target;
  target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5.yaml");

  // Create Observation Extractor Object
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    cv::Mat output_image;
    ASSERT_TRUE(observation_extractor.extractObservation(calibration_images[i], output_image));
#if 0 
    cv::namedWindow("Image " + std::to_string(i+1), cv::WINDOW_NORMAL);
    cv::imshow("Image " + std::to_string(i+1), output_image);
    cv::waitKey(0);    
#endif
  }

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData();

  EXPECT_EQ(observation_data.size(), num_images);

  for (std::size_t i = 0; i < num_images; i++)
  {
    EXPECT_EQ(observation_data[i].size(), target.getDefinition().target_points);
  }

  // TODO(gChiou): Find a way to verify observation data...

#if 0
  CONSOLE_OUTPUT("Total Observations: " << observation_data.size());
  for (std::size_t i = 0; i < observation_data.size(); i++)
  {
    CONSOLE_OUTPUT("Observations for Image " << i+1 << " Size: " << observation_data[i].size());
    CONSOLE_OUTPUT("Observations for Image " << i+1 << " Points:");
    for (std::size_t j = 0; j < observation_data[i].size(); j++)
    {
      CONSOLE_OUTPUT(observation_data[i][j]);
    }
  }
#endif
}
