#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <industrial_calibration/helper_functions.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "extrinsic_example");
  ros::NodeHandle pnh("~");

  std::string data_path;
  pnh.getParam("data_path", data_path);

  // Load Target Data
  industrial_calibration_libs::Target target(data_path + 
    "mcircles_10x10/mcircles_10x10.yaml");

  // Load Calibration Images
  const std::size_t num_images = 5;
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = data_path + "mcircles_10x10/intrinsic/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i) + ".png";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
    calibration_images.push_back(image);    
  }

  // Extract Observations
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    cv::Mat output_image;
    observation_extractor.extractObservation(calibration_images[i],
      output_image);
  }

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData(); 

  // Load Link Data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp_link_data;
    loadLinkData(i, data_path + "mcircles_10x10/extrinsic/tf/", &temp_link_data);
    link_data.push_back(temp_link_data);
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
  calibration.setOutput(true);
  calibration.runCalibration();
  calibration.displayCovariance();

  // Print out results
  industrial_calibration_libs::CameraOnWristExtrinsic::Result results = calibration.getResults();
  ROS_INFO_STREAM("Extrinsic Parameters");
  ROS_INFO_STREAM("Translation x: " << results.extrinsics[3]);
  ROS_INFO_STREAM("Translation y: " << results.extrinsics[4]);
  ROS_INFO_STREAM("Translation z: " << results.extrinsics[5]);
  ROS_INFO_STREAM("Rotation x: " << results.extrinsics[0]);
  ROS_INFO_STREAM("Rotation y: " << results.extrinsics[1]);
  ROS_INFO_STREAM("Rotation z: " << results.extrinsics[2]);

  ROS_INFO_STREAM("Target to Base");
  ROS_INFO_STREAM("Translation x: " << results.target_to_base[3]);
  ROS_INFO_STREAM("Translation y: " << results.target_to_base[4]);
  ROS_INFO_STREAM("Translation z: " << results.target_to_base[5]);
  ROS_INFO_STREAM("Rotation x: " << results.target_to_base[0]);
  ROS_INFO_STREAM("Rotation y: " << results.target_to_base[1]);
  ROS_INFO_STREAM("Rotation z: " << results.target_to_base[2]);

  ROS_INFO_STREAM("Initial Cost: " << calibration.getInitialCost());
  ROS_INFO_STREAM("Final Cost: " << calibration.getFinalCost());
}
