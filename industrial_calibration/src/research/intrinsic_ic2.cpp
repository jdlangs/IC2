/*
  This file will run through the data set using industrial_calibration's 
  circle grid finder and calibration solver.
*/

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <industrial_calibration/helper_functions.h>

#include <opencv2/core/core.hpp>

#define ICL industrial_calibration_libs

// Function Declarations
void calibrateDataSet(const std::string &data_dir, const std::string &data_set);

// Function Implementatins
void calibrateDataSet(const std::string &data_dir, const std::string &data_set)
{
  std::string data_path = data_dir + data_set + "/";

  // Load Target Data
  ICL::Target target(data_path + "mcircles_9x12.yaml");

  // Load Calibration Images
  CalibrationImages cal_images;
  ROS_INFO_STREAM("Loading Calibration Images for Data Set: " << data_set);
  getCalibrationImages(data_path, cal_images);  

  // Extract Observations
  ROS_INFO_STREAM("Extracting Observations from Data Set: " << data_set);
  ICL::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < cal_images.size(); i++)
  {
    cv::Mat grid_image;
    observation_extractor.extractObservation(cal_images[i], grid_image);
  }

  // Get observations from extractor
  ICL::ObservationData observation_data = observation_extractor.getObservationData();

  // Seed parameters
  double camera_info[9] = {0.0};
  // camera_info[0] = 570.342224;
  // camera_info[1] = 570.342224;
  // camera_info[2] = 319.5;
  // camera_info[3] = 239.5;
  // camera_info[4] = 0.0;
  // camera_info[5] = 0.0;
  // camera_info[6] = 0.0;
  // camera_info[7] = 0.0;
  // camera_info[8] = 0.0;

  ICL::ResearchIntrinsicParams params;
  params.intrinsics = ICL::IntrinsicsFull(camera_info);
  ICL::ResearchIntrinsic calibration(observation_data, target, params);
  
  calibration.setOutput(true); // Enable output to console.
  calibration.runCalibration();

  // Print out results.
  ICL::ResearchIntrinsic::Result results = calibration.getResults();
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
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intrinsic_opencv");
  ros::NodeHandle pnh("~");
  
  std::string data_dir;
  pnh.getParam("data_dir", data_dir);
  data_dir = addSlashToEnd(data_dir);

  // std::vector<std::string> data_sets = { "01", "02", "03", "04", "05", 
  //   "06", "07", "08", "09", "10", "11", "12", "13", "14", "15" };

  std::vector<std::string> data_sets = { "01" };

  for (std::size_t i = 0; i < data_sets.size(); i++)
  {
    calibrateDataSet(data_dir, data_sets[i]);
  }

  return 0;
}