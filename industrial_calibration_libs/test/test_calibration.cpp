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
    loadLinkData(i, "mcircles_10x10/extrinsic/tf/", &temp);
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


  // I don't know what else to test...
}
