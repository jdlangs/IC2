#include <test_utils.h>

// Note(gChiou): These tests only check that the correct number of 
//               "observations" were extracted from an image.

TEST(Observations, observation_from_single_image)
{
  std::size_t num_observations = 15;

  // Load in calibration images
  std::vector<cv::Mat> calibration_images;
  std::string image_path = "mcircles_10x10/extrinsic/images/";
  loadImagesFromPath(image_path, num_observations, calibration_images);

  #if 0
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    showImage(calibration_images[i], "Original Image: " + std::to_string(i));
  }
  #endif

  // Load in target
  industrial_calibration_libs::Target target("mcircles_10x10/mcircles_10x10.yaml");

  // Create observation extractor and load one at a time
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);

  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    ASSERT_TRUE(observation_extractor.extractObservation(calibration_images[i]));
  }

  // Get the observation data
  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData();

  EXPECT_EQ(observation_data.size(), num_observations);

  std::size_t target_rows = target.getDefinition().target_rows;
  std::size_t target_cols = target.getDefinition().target_cols;
  std::size_t total_points = target_rows * target_cols;

  for (std::size_t i = 0; i < num_observations; i++)
  {
    EXPECT_EQ(observation_data[i].size(), total_points);
  }

  #if 0
  printObservationData(observation_data);
  #endif
}

TEST(Observations, observation_from_single_image_output)
{
  std::size_t num_observations = 15;

  // Load in calibration images
  std::vector<cv::Mat> calibration_images;
  std::string image_path = "mcircles_10x10/extrinsic/images/";
  loadImagesFromPath(image_path, num_observations, calibration_images);

  #if 0
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    showImage(calibration_images[i], "Original Image: " + std::to_string(i));
  }
  #endif

  // Load in target
  industrial_calibration_libs::Target target("mcircles_10x10/mcircles_10x10.yaml");

  // Create observation extractor and load one at a time
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);

  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    cv::Mat output_image;
    ASSERT_TRUE(observation_extractor.extractObservation(calibration_images[i],
      output_image));
    #if 0
    showImage(output_image, "Observation Image: " + std::to_string(i));
    #endif
  }

  // Get the observation data
  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData();

  EXPECT_EQ(observation_data.size(), num_observations);

  std::size_t target_rows = target.getDefinition().target_rows;
  std::size_t target_cols = target.getDefinition().target_cols;
  std::size_t total_points = target_rows * target_cols;

  for (std::size_t i = 0; i < num_observations; i++)
  {
    EXPECT_EQ(observation_data[i].size(), total_points);
  }

  #if 0
  printObservationData(observation_data);
  #endif
}

TEST(Observations, observations_from_vector_images)
{
  std::size_t num_observations = 15;

  // Load in calibration images
  std::vector<cv::Mat> calibration_images;
  std::string image_path = "mcircles_10x10/extrinsic/images/";
  loadImagesFromPath(image_path, num_observations, calibration_images);

  #if 0
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    showImage(calibration_images[i], "Original Image: " + std::to_string(i));
  }
  #endif

  // Load in target
  industrial_calibration_libs::Target target("mcircles_10x10/mcircles_10x10.yaml");

  // Create observation extractor and load one at a time
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);

  std::vector<bool> success;
  int num_success = observation_extractor.extractObservations(calibration_images,
    success);

  int check_success = 0;
  for (std::size_t i = 0; i < success.size(); i++)
  {
    if (success[i]) {check_success++;}
  }

  EXPECT_EQ(num_success, check_success);
  EXPECT_EQ(static_cast<std::size_t>(num_success), num_observations);

  // Get the observation data
  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData();

  EXPECT_EQ(observation_data.size(), num_observations);

  std::size_t target_rows = target.getDefinition().target_rows;
  std::size_t target_cols = target.getDefinition().target_cols;
  std::size_t total_points = target_rows * target_cols;

  for (std::size_t i = 0; i < num_observations; i++)
  {
    EXPECT_EQ(observation_data[i].size(), total_points);
  }

  #if 0
  printObservationData(observation_data);
  #endif
}


