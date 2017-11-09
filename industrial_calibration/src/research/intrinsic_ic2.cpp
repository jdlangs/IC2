/*
  This file will run through the data set using industrial_calibration's 
  circle grid finder and calibration solver.
*/

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <industrial_calibration/helper_functions.h>

#include <opencv2/core/core.hpp>

#include <fstream>

#define ICL industrial_calibration_libs

// Function Declarations
void calibrateDataSet(const std::string &data_dir, const std::string &data_set);

bool saveResultdata(const std::string &result_path, double final_cost, 
    double intrinsics[9]);

// Function Implementatins
bool saveResultdata(const std::string &result_path, double final_cost, 
    double intrinsics[9])
{
  std::vector<double> camera_matrix = { intrinsics[0], 0, intrinsics[2],
    0, intrinsics[1], intrinsics[3], 0, 0, 1 };
  // Mine are: k1, k2, k3, p1, p2
  // Output as: k1, k2, p1, p2, k3
  std::vector<double> dist_coeffs = { intrinsics[4], intrinsics[5],
    intrinsics[7], intrinsics[8], intrinsics[6] };

  YAML::Emitter out;

  out << YAML::BeginMap;

  out << YAML::Key << "camera_matrix";
    out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "rows";
        out << YAML::Value << 3;
      out << YAML::Key << "cols";
        out << YAML::Value << 3;
      out << YAML::Key << "data";
        out << YAML::Value << YAML::Flow << camera_matrix;
    out << YAML::EndMap;

  out << YAML::Key << "distortion_coefficients";
    out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "rows";
        out << YAML::Value << 1;
      out << YAML::Key << "cols";
        out << YAML::Value << 9;
      out << YAML::Key << "data";
        out << YAML::Value << YAML::Flow << dist_coeffs;
    out << YAML::EndMap;

  out << YAML::Key << "reprojection_error";
    out << YAML::Value << final_cost;

  out << YAML::EndMap;

  // Write to yaml file
  if (out.good())
  {
    std::ofstream yaml_file(result_path);
    if (yaml_file)
    {
      yaml_file << out.c_str();
      yaml_file.close();
      return true;
    }
    if (yaml_file.bad())
    {
      return false;
    }
  }

  return false;
}

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

  // Seed parameters (Average of intrinsic values)
  double camera_info[9];
  camera_info[0] = 537.1;
  camera_info[1] = 536.1;
  camera_info[2] = 325.5;
  camera_info[3] = 231.9;
  camera_info[4] = 0.0;
  camera_info[5] = 0.0;
  camera_info[6] = 0.0;
  camera_info[7] = 0.0;
  camera_info[8] = 0.0;

  ROS_INFO_STREAM("Running Calibration for Data Set: " << data_set);
  ICL::ResearchIntrinsicParams params;
  params.intrinsics = ICL::IntrinsicsFull(camera_info);
  ICL::ResearchIntrinsic calibration(observation_data, target, params);
  
  // calibration.setOutput(true); // Enable output to console.
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

  std::string result_path = data_dir + "results/ic2_" + data_set + ".yaml";  
  if (saveResultdata(result_path, calibration.getFinalCost(), 
    results.intrinsics))
  {
    ROS_INFO_STREAM("Data Set: " << data_set << " Results Saved To: " <<
      result_path);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to Save Results for Data Set: " << data_set);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intrinsic_opencv");
  ros::NodeHandle pnh("~");
  
  std::string data_dir;
  pnh.getParam("data_dir", data_dir);
  data_dir = addSlashToEnd(data_dir);

  std::vector<std::string> data_sets = { "01", "02", "03", "04", "05", 
    "06", "07", "08", "09", "10", "11", "12", "13", "14", "15" };

  // std::vector<std::string> data_sets = { "01" };

  for (std::size_t i = 0; i < data_sets.size(); i++)
  {
    calibrateDataSet(data_dir, data_sets[i]);
  }

  return 0;
}