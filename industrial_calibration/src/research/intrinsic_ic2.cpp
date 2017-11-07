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

#include <boost/filesystem.hpp>

#define ICL industrial_calibration_libs

typedef std::vector<cv::Mat> CalibrationImages;

// Function Declarations
void calibrateDataSet(const std::string &data_dir, const std::string &data_set);

// Function Declarations

void calibrateDataSet(const std::string &data_dir, const std::string &data_set)
{
  std::cout << data_dir << std::endl;
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