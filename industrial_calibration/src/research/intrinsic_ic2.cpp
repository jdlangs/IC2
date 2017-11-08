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
  double camera_info[9];
  camera_info[0] = 570.342224;
  camera_info[1] = 570.342224;
  camera_info[2] = 319.5;
  camera_info[3] = 239.5;
  camera_info[4] = 0.0;
  camera_info[5] = 0.0;
  camera_info[6] = 0.0;
  camera_info[7] = 0.0;
  camera_info[8] = 0.0;

  // camera_info[0] = 537.7096612596631;
  // camera_info[1] = 536.4955104533378;
  // camera_info[2] = 328.9036170597059;
  // camera_info[3] = 230.3843195009159;
  // camera_info[4] = 0.0;
  // camera_info[5] = 0.0;
  // camera_info[6] = 0.0;
  // camera_info[7] = 0.0;
  // camera_info[8] = 0.0;

  // camera_info[0] = 1.0;
  // camera_info[1] = 1.0;
  // camera_info[2] = 1.0;
  // camera_info[3] = 1.0;
  // camera_info[4] = 0.0;
  // camera_info[5] = 0.0;
  // camera_info[6] = 0.0;
  // camera_info[7] = 0.0;
  // camera_info[8] = 0.0;

/*
[ INFO] [1510132661.136932042]: Focal Length x: 570.604
[ INFO] [1510132661.136958861]: Focal Length y: 571.167
[ INFO] [1510132661.136972760]: Optical Center x: 319.575
[ INFO] [1510132661.136991198]: Optical Center y: 239.445

New Method (seeded)
[ INFO] [1510134128.680457658]: Focal Length x: 578.649
[ INFO] [1510134128.680477353]: Focal Length y: 267.815
[ INFO] [1510134128.680498305]: Optical Center x: 320.637
[ INFO] [1510134128.680519398]: Optical Center y: 200.849

New Method (seeded with opencv values)
[ INFO] [1510134256.678787543]: Focal Length x: 542.053
[ INFO] [1510134256.678803047]: Focal Length y: 260.413
[ INFO] [1510134256.678820298]: Optical Center x: 331.072
[ INFO] [1510134256.678840552]: Optical Center y: 195.553

*/

  ICL::ResearchIntrinsicParams params;
  params.intrinsics = ICL::IntrinsicsFull(camera_info);
  ICL::ResearchIntrinsic calibration(observation_data, target, params);
  
  calibration.setOutput(true); // Enable output to console.
  calibration.runCalibration();

  // Print out results.
  ICL::ResearchIntrinsic::Result results = calibration.getResults();
  ROS_INFO_STREAM("Initial Cost: " << calibration.getInitialCost());
  ROS_INFO_STREAM("Final Cost: " << calibration.getFinalCost());
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