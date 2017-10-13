#include <test_utils.h>

TEST(Calibration, CameraOnWristExtrinsic)
{
  // Load target data
  industrial_calibration_libs::Target target("mcircles_10x10/mcircles_10x10.yaml");

  // Load calibration images
  std::size_t num_images = 15;
  std::vector<cv::Mat> calibration_images;
  loadImagesFromPath("mcircles_10x10/extrinsic/images/", num_images,
    calibration_images);

  // Extract observations and get data
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  int success = observation_extractor.extractObservations(calibration_images);
  EXPECT_EQ(success, num_images);

  industrial_calibration_libs::ObservationData observation_data;
  observation_data = observation_extractor.getObservationData();

  // Load link data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp;
    loadLinkData("mcircles_10x10/extrinsic/tf/" + std::to_string(i) + ".yaml", 
      &temp, "base_link_to_tool0");
    link_data.push_back(temp);
  }

  // Set camera extrinsics seed
  industrial_calibration_libs::Pose6D link_6_to_camera_;
  link_6_to_camera_.setOrigin(0.0197, 0.0908, 0.112141);
  link_6_to_camera_.setAngleAxis(0.0, 0.0, -3.14/2.0);

  // Set target_to_base seed
  industrial_calibration_libs::Pose6D target_pose;
  target_pose.setOrigin(0.0, 0.0, 0.0);
  target_pose.setEulerZYX(0.0, 0.0, 0.0);

  // Set calibration params
  industrial_calibration_libs::CameraOnWristExtrinsicParams params;
  params.intrinsics = industrial_calibration_libs::IntrinsicsPartial(509.5179, 
    511.6581, 320.2695, 208.9545);
  params.tool_to_camera = industrial_calibration_libs::Extrinsics(link_6_to_camera_.getInverse());
  params.target_to_base = industrial_calibration_libs::Extrinsics(target_pose);
  convertToPose6D(link_data, &params.base_to_tool);  

  // Create calibration object and run
  industrial_calibration_libs::CameraOnWristExtrinsic calibration(observation_data, 
    target, params);
  calibration.runCalibration();
  calibration.displayCovariance();

  // Get results
  industrial_calibration_libs::CameraOnWristExtrinsic::Result results 
    = calibration.getResults();    
}

TEST(Calibration, CameraOnWristIntrinsic_ABB)
{
  // Load target data
  industrial_calibration_libs::Target target("mcircles_9x12/mcircles_9x12.yaml");

  // Load calibration images
  std::size_t num_images = 7;
  std::vector<cv::Mat> calibration_images;
  loadImagesFromPath("mcircles_9x12/intrinsic_abb/images/", num_images,
    calibration_images);

  // Extract observations and get data
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  int success = observation_extractor.extractObservations(calibration_images);
  EXPECT_EQ(success, num_images);

  industrial_calibration_libs::ObservationData observation_data;
  observation_data = observation_extractor.getObservationData();

  // Load link data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp;
    loadLinkData("mcircles_9x12/intrinsic_abb/tf/" + std::to_string(i) + ".yaml", 
      &temp, "base_to_tool0");
    link_data.push_back(temp);
  }

  // Set initial target pose seed
  double distance_to_target = 0.1778; // meters
  double qx, qy, qz, qw;
  getQuaternion(distance_to_target, qx, qy, qz, qw);

  industrial_calibration_libs::Pose6D target_pose;
  target_pose.setQuaternion(qx, qy, qz, qw);
  target_pose.setOrigin(0.0089, 0.127, 0.1778); // meters

  // Load camera info
  double camera_info[9];
  ASSERT_TRUE(loadCameraInfo("mcircles_9x12/intrinsic_abb/camera_info.yaml", camera_info));

  // Set calibration params
  industrial_calibration_libs::CameraOnWristIntrinsicParams params;
  params.intrinsics = industrial_calibration_libs::IntrinsicsFull(camera_info);
  params.target_to_camera = industrial_calibration_libs::Extrinsics(target_pose);
  convertToPose6D(link_data, &params.base_to_tool);  

  // Create calibration object and run
  industrial_calibration_libs::CameraOnWristIntrinsic calibration(
    observation_data, target, params);
  calibration.setOutput(false); // Disable output to console.
  calibration.runCalibration();
  calibration.displayCovariance();  

  // Get results
  industrial_calibration_libs::CameraOnWristIntrinsic::Result results 
    = calibration.getResults();

  // Verify Results
  // Load verification images
  const std::size_t num_verification_images = 2;
  std::vector<cv::Mat> verification_images;
  verification_images.reserve(num_verification_images);

  for (std::size_t i = 0; i < num_verification_images; i++)
  {
    std::string image_path = "mcircles_9x12/intrinsic_abb/images/v" + std::to_string(i) + ".png";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
    verification_images.push_back(image);
  }

  // Extract observations from verification images
  industrial_calibration_libs::ObservationExtractor vobservation_extractor(target);
  int vnum_success = vobservation_extractor.extractObservations(verification_images);

  EXPECT_EQ(vnum_success, num_verification_images);
  industrial_calibration_libs::ObservationData vobservation_data =
    vobservation_extractor.getObservationData();

  // Load verification link_data
  std::vector<LinkData> vlink_data;
  vlink_data.reserve(num_verification_images);
  for (std::size_t i = 0; i < num_verification_images; i++)
  {
    LinkData temp_link_data;
    loadLinkData("mcircles_9x12/intrinsic_abb/tf/v" + std::to_string(i) + ".yaml",
      &temp_link_data, "base_to_tool0"); 
    vlink_data.push_back(temp_link_data);
  }    

  industrial_calibration_libs::Pose6D pose_0, pose_1;
  convertToPose6D(vlink_data[0], pose_0);
  convertToPose6D(vlink_data[num_verification_images-1], pose_1);

  industrial_calibration_libs::IntrinsicsVerification verification 
    = calibration.verifyIntrinsics(vobservation_data[0], pose_0, 
      vobservation_data[num_verification_images-1], pose_1,
      results.intrinsics, params.target_to_camera.data);

  EXPECT_NEAR(verification.target_diff_x, verification.tool_diff_x, 0.001);
  EXPECT_NEAR(verification.target_diff_y, verification.tool_diff_y, 0.001);
  EXPECT_NEAR(verification.target_diff_z, verification.tool_diff_z, 0.001);
}

TEST(Calibration, CameraOnWristIntrinsic_MH3)
{
  // Load target data
  industrial_calibration_libs::Target target("mcircles_9x12/mcircles_9x12.yaml");

  // Load calibration images
  std::size_t num_images = 7;
  std::vector<cv::Mat> calibration_images;
  loadImagesFromPath("mcircles_9x12/intrinsic_mh3/images/", num_images,
    calibration_images);

  // Extract observations and get data
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  int success = observation_extractor.extractObservations(calibration_images);
  EXPECT_EQ(success, num_images);

  industrial_calibration_libs::ObservationData observation_data;
  observation_data = observation_extractor.getObservationData();

  // Load link data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp;
    loadLinkData("mcircles_9x12/intrinsic_mh3/tf/" + std::to_string(i) + ".yaml", 
      &temp, "base_to_tool0");
    link_data.push_back(temp);
  }

  // Set initial target pose seed
  double distance_to_target = 0.1778; // meters
  double qx, qy, qz, qw;
  getQuaternion(distance_to_target, qx, qy, qz, qw);

  industrial_calibration_libs::Pose6D target_pose;
  target_pose.setQuaternion(qx, qy, qz, qw);
  target_pose.setOrigin(0.0089, 0.127, 0.1778); // meters

  // Load camera info
  double camera_info[9];
  ASSERT_TRUE(loadCameraInfo("mcircles_9x12/intrinsic_mh3/camera_info.yaml", camera_info));

  // Set calibration params
  industrial_calibration_libs::CameraOnWristIntrinsicParams params;
  params.intrinsics = industrial_calibration_libs::IntrinsicsFull(camera_info);
  params.target_to_camera = industrial_calibration_libs::Extrinsics(target_pose);
  convertToPose6D(link_data, &params.base_to_tool);  

  // Create calibration object and run
  industrial_calibration_libs::CameraOnWristIntrinsic calibration(
    observation_data, target, params);
  calibration.setOutput(false); // Disable output to console.
  calibration.runCalibration();
  calibration.displayCovariance();  

  // Get results
  industrial_calibration_libs::CameraOnWristIntrinsic::Result results 
    = calibration.getResults();

  // Verify Results (using same data)
  industrial_calibration_libs::IntrinsicsVerification verification 
    = calibration.verifyIntrinsics(observation_data[0], params.base_to_tool[0], 
      observation_data[num_images-1], params.base_to_tool[num_images-1],
      results.intrinsics, params.target_to_camera.data);

  EXPECT_NEAR(verification.target_diff_x, verification.tool_diff_x, 0.001);
  EXPECT_NEAR(verification.target_diff_y, verification.tool_diff_y, 0.001);
  EXPECT_NEAR(verification.target_diff_z, verification.tool_diff_z, 0.001);
}
