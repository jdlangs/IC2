#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <industrial_calibration/helper_functions.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intrinsic example");
  ros::NodeHandle pnh("~");

  std::string data_path;
  pnh.getParam("data_path", data_path);

  // Load Target Data
  industrial_calibration_libs::Target target(data_path +
    "mcircles_9x12/mcircles_9x12.yaml");

  // Load Calibration Images
  const std::size_t num_images = 7;
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = data_path + "mcircles_9x12/intrinsic_abb/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i) + ".png";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
    calibration_images.push_back(image);
  }

  // Extract Observations and get Observation Data
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  int num_success = observation_extractor.extractObservations(calibration_images);

  if (num_success != num_images)
  {
    ROS_ERROR_STREAM("Unable to extract all observations");
    return 0;
  }

  industrial_calibration_libs::ObservationData observation_data =
    observation_extractor.getObservationData();

  // Load link data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp_link_data;
    loadLinkData(data_path + "mcircles_9x12/intrinsic_abb/tf/" + std::to_string(i) + ".yaml",
      &temp_link_data, "base_to_tool0");    
    link_data.push_back(temp_link_data);
  }

  // Set initial target pose seed
  double distance_to_target = 0.1778; // meters
  double qx, qy, qz, qw;
  getQuaternion(distance_to_target, qx, qy, qz, qw);

  industrial_calibration_libs::Pose6D target_pose;
  target_pose.setQuaternion(qx, qy, qz, qw);
  target_pose.setOrigin(0.0089, 0.127, 0.1778); // meters

  // Load camera_info from YAML.
  std::string camera_info_path = data_path + "mcircles_9x12/intrinsic_abb/camera_info.yaml";
  double camera_info[9];
  loadCameraInfo(camera_info_path, camera_info);

  // Set your calibration parameters to be passed to the calibration object.
  industrial_calibration_libs::CameraOnWristIntrinsicParams params;

  // Seed intrinsic parameters
  params.intrinsics = industrial_calibration_libs::IntrinsicsFull(camera_info);

  // Seed target pose
  params.target_to_camera = industrial_calibration_libs::Extrinsics(target_pose);

  // Robot tool positions for every calibration image.
  convertToPose6D(link_data, &params.base_to_tool);

  // Create calibration object and run
  industrial_calibration_libs::CameraOnWristIntrinsic calibration(
    observation_data, target, params);
  calibration.setOutput(true); // Enable output to console.
  calibration.runCalibration();
  calibration.displayCovariance();

  // Print out results.
  industrial_calibration_libs::CameraOnWristIntrinsic::Result results = calibration.getResults();
  ROS_INFO_STREAM("Intrinsic Parameters");
  ROS_INFO_STREAM("----------------------------------------");
  ROS_INFO_STREAM("Focal Length x: " << results.intrinsics[0]);
  ROS_INFO_STREAM("Focal Length y: " << results.intrinsics[1]);
  ROS_INFO_STREAM("Optical Center x: " << results.intrinsics[2]);
  ROS_INFO_STREAM("Optical Center y: " << results.intrinsics[3]);
  ROS_INFO_STREAM("Distortion k1: " << results.intrinsics[4]);
  ROS_INFO_STREAM("Distortion k2: " << results.intrinsics[5]);
  ROS_INFO_STREAM("Distortion k3: " << results.intrinsics[6]);
  ROS_INFO_STREAM("Distortion p1: " << results.intrinsics[7]);
  ROS_INFO_STREAM("Distortion p2: " << results.intrinsics[8]);
  ROS_INFO_STREAM("Target Pose");
  ROS_INFO_STREAM("----------------------------------------");
  ROS_INFO_STREAM("Translation x: " << results.target_to_camera[3]);
  ROS_INFO_STREAM("Translation y: " << results.target_to_camera[4]);
  ROS_INFO_STREAM("Translation z: " << results.target_to_camera[5]);
  ROS_INFO_STREAM("Angle Axis x: " << results.target_to_camera[0]);
  ROS_INFO_STREAM("Angle Axis y: " << results.target_to_camera[1]);
  ROS_INFO_STREAM("Angle Axis z: " << results.target_to_camera[2]);
  ROS_INFO_STREAM("----------------------------------------");
  ROS_INFO_STREAM("Initial Cost: " << calibration.getInitialCost());
  ROS_INFO_STREAM("Final Cost: " << calibration.getFinalCost());

  // Use these results to verify a calibration. This is done by taking
  // two images that are furthest away from each other and calculating the
  // target pose. If the distance moved for the target matches the distance
  // moved for the robot tool, your calibration should be accurate.

  // This is passing in:
  // 1] First observation image data
  // 2] Pose at first observation image
  // 3] Second observation image data
  // 4] Pose at second observation
  // 5] Results of intrinsic calibration
  // 6] Initial guess for target position

  // Grab verfication observations
  const std::size_t num_verification_images = 2;
  std::vector<cv::Mat> verification_images;
  verification_images.reserve(num_verification_images);

  for (std::size_t i = 0; i < num_verification_images; i++)
  {
    std::string image_path = cal_image_path + "v" + std::to_string(i) + ".png";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
    verification_images.push_back(image);
  }

  // Extract observations from verification images
  industrial_calibration_libs::ObservationExtractor vobservation_extractor(target);
  int vnum_success = vobservation_extractor.extractObservations(verification_images);

  if (vnum_success != num_verification_images)
  {
    ROS_ERROR_STREAM("Unable to extract observations from verification images");
    return 0;
  }

  industrial_calibration_libs::ObservationData vobservation_data =
    vobservation_extractor.getObservationData();

  // Load verification link_data
  std::vector<LinkData> vlink_data;
  vlink_data.reserve(num_verification_images);
  for (std::size_t i = 0; i < num_verification_images; i++)
  {
    LinkData temp_link_data;
    loadLinkData(data_path + "mcircles_9x12/intrinsic_abb/tf/v" + std::to_string(i) + ".yaml",
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

  ROS_INFO_STREAM("x Direction:");
  ROS_INFO_STREAM("Target (Pose_1 - Pose_2) x: " << verification.target_diff_x << " m");
  ROS_INFO_STREAM("Tool Diff (Pose_1 - Pose_2) x: " << verification.tool_diff_x << " m");
  ROS_INFO_STREAM("Absolute Error (Tool - Target) x: " << verification.absolute_error_x << " m");

  ROS_INFO_STREAM("y Direction:");
  ROS_INFO_STREAM("Target Diff (Pose_1 - Pose_2) y: " << verification.target_diff_y << " m");
  ROS_INFO_STREAM("Tool Diff (Pose_1 - Pose_2) y: " << verification.tool_diff_y << " m");
  ROS_INFO_STREAM("Absolute Error (Tool - Target) y: " << verification.absolute_error_y << " m");

  ROS_INFO_STREAM("z Direction:");
  ROS_INFO_STREAM("Target Diff (Pose_1 - Pose_2) z: " << verification.target_diff_z << " m");
  ROS_INFO_STREAM("Tool Diff (Pose_1 - Pose_2) z: " << verification.tool_diff_z << " m");
  ROS_INFO_STREAM("Absolute Error (Tool - Target) z: " << verification.absolute_error_z << " m");

  return 0;    
}

