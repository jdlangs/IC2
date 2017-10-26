/*
  This file will run through the data set using industrial_calibration's 
  circle grid finder but run the data through OpenCV's solver.
*/

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <industrial_calibration/helper_functions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

#define ICL industrial_calibration_libs

typedef std::vector<cv::Mat> CalibrationImages;

void getCalibrationImages(const std::string &path, CalibrationImages &images);

bool isPNG(const std::string &file_name);

bool isPNG(const std::string &file_name)
{
  std::size_t dot_location = file_name.find('.');
  std::string extension = file_name.substr(dot_location + 1);
  if (extension.compare("png") == 0) {return true;}
  return false;
}

void getCalibrationImages(const std::string &path, CalibrationImages &images)
{
  boost::filesystem::path image_dir(path);
  boost::filesystem::directory_iterator end_iter;

  if (boost::filesystem::exists(image_dir) &&
    boost::filesystem::is_directory(image_dir))
  {
    for (boost::filesystem::directory_iterator dir_iter(image_dir);
      dir_iter != end_iter; ++dir_iter)
    {
      if (boost::filesystem::is_regular_file(dir_iter->status()) &&
        isPNG(dir_iter->path().filename().string()))
      {
        std::string image_path = path + dir_iter->path().filename().string();
        cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
        images.push_back(image);
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "intrinsic_opencv");
  ros::NodeHandle pnh("~");

  std::string data_path = "/home/gchiou/Desktop/01/";

  // Load Target Data
  ICL::Target target(data_path + "mcircles_9x12.yaml");

  // Load Calibration Images
  CalibrationImages cal_images;
  getCalibrationImages(data_path, cal_images);

  // Extract Observations
  ICL::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < cal_images.size(); i++)
  {
    cv::Mat grid_image;
    observation_extractor.extractObservation(cal_images[i], grid_image);

    std::cout << "Image: " << i << '\n'; 

    // Visualize
    cv::imshow("grid", grid_image);
    cv::waitKey(0);
  }

  return 0;
}