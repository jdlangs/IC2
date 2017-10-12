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
  std::string cal_image_path = data_path + "mcircles_9x12/intrinsic_mh3/images/";

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
    loadLinkData2(i, data_path + "mcircles_9x12/intrinsic_mh3/tf/", &temp_link_data);
    link_data.push_back(temp_link_data);
  }

  // Set initial target pose seed
  double distance_to_target = 0.1778; // meters
  double qx, qy, qz, qw;
  getQuaternion(distance_to_target, qx, qy, qz, qw);

  industrial_calibration_libs::Pose6D target_pose;
  target_pose.setQuaternion(qx, qy, qz, qw);
  target_pose.setOrigin(0.0089, 0.127, 0.1778); // meters

  // Set Camera Info (manually extracted from camera_info.txt).
  // Focal length and optical center are from K matrix.
  // Distortion is from P matrix.
  // This is un-calibrated data and is used as an initial guess.
  double focal_length_x = 570.3422;
  double focal_length_y = 570.3422;
  double optical_center_x = 319.5;
  double optical_center_y = 239.5;
  double distortion_k1 = 0.0;
  double distortion_k2 = 0.0;
  double distortion_k3 = 0.0;
  double distortion_p1 = 0.0;
  double distortion_p2 = 0.0;

  // Set your calibration parameters to be passed to the calibration object.
  industrial_calibration_libs::CameraOnWristIntrinsicParams params;

  // Seed intrinsic parameters
  params.intrinsics = industrial_calibration_libs::IntrinsicsFull(focal_length_x,
    focal_length_y, optical_center_x, optical_center_y, distortion_k1, 
    distortion_k2, distortion_k3, distortion_p1, distortion_p2);

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

  // In this case we will (temporarily) use the same data that was used to solve 
  // for the intrinsics. I will take new images in the future to verify. (gChiou).

  // This is passing in:
  // 1] First observation image data
  // 2] Pose at first observation image
  // 3] Second observation image data
  // 4] Pose at second observation
  // 5] Results of intrinsic calibration
  // 6] Initial guess for target position
  industrial_calibration_libs::IntrinsicsVerification verification 
    = calibration.verifyIntrinsics(observation_data[0], params.base_to_tool[0], 
      observation_data[num_images-1], params.base_to_tool[num_images-1],
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

