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

#include <fstream>

#define ICL industrial_calibration_libs
#define SAVE_ESTIMATED_EXTRINSICS

// Function Declarations
cv::Point2f point2dToPoint2f(const cv::Point2d &point2d);

cv::Point3f point3dToPoint3f(const ICL::Point3D &point3d);

// Copied from OpenCV tutorial:
// https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>> &object_points,
  const std::vector<std::vector<cv::Point2f>> &image_points,
  const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
  const cv::Mat &camera_matrix, const cv::Mat &dist_coefficients,
  std::vector<float> &per_view_errors);

std::vector<double> matToVec(const cv::Mat &mat);

bool saveResultData(const std::string &result_path, double rms, double total_average_error,
  const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs);

bool saveEstimatedExtrinsics(const std::string &estimated_poses_path, 
  const std::string &data_set, const std::vector<cv::Mat> &rvecs, 
  const std::vector<cv::Mat> &tvecs);

void calibrateDataSet(const std::string &data_dir, const std::string &data_set);


// Function Implementations
cv::Point2f point2dToPoint2f(const cv::Point2d &point2d)
{
  cv::Point2f point2f;
  point2f.x = static_cast<float>(point2d.x);
  point2f.y = static_cast<float>(point2d.y);
  return point2f;
}

cv::Point3f point3dToPoint3f(const ICL::Point3D &point3d)
{
  cv::Point3f point3f;
  point3f.x = static_cast<float>(point3d.x);
  point3f.y = static_cast<float>(point3d.y);
  point3f.z = static_cast<float>(point3d.z);
  return point3f;
}

// Copied from OpenCV tutorial:
// https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>> &object_points,
  const std::vector<std::vector<cv::Point2f>> &image_points,
  const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
  const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
  std::vector<float> &per_view_errors)
{
  std::vector<cv::Point2f> image_points_s2;
  int total_points = 0;
  double total_error = 0;
  double error;

  per_view_errors.resize(object_points.size());

  for (std::size_t i = 0; i < object_points.size(); ++i)
  {
    cv::projectPoints(cv::Mat(object_points[i]), rvecs[i], tvecs[i], camera_matrix,
    dist_coeffs, image_points_s2);
    error = cv::norm(cv::Mat(image_points[i]), cv::Mat(image_points_s2), CV_L2);

    int n = static_cast<int>(object_points[i].size());

    per_view_errors[i] = static_cast<float>(std::sqrt(error*error/n));
    total_error += error*error;
    total_points += n;
  }
  return std::sqrt(total_error / total_points);
}

std::vector<double> matToVec(const cv::Mat &mat)
{
  std::vector<double> vec(mat.begin<double>(), mat.end<double>());
  return vec;
}

bool saveResultData(const std::string &result_path, double rms, double total_average_error,
  const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs)
{
  YAML::Emitter out;

  out << YAML::BeginMap;

  out << YAML::Key << "camera_matrix";
    out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "rows";
        out << YAML::Value << camera_matrix.rows;
      out << YAML::Key << "cols";
        out << YAML::Value << camera_matrix.cols;
      out << YAML::Key << "data";
        out << YAML::Value << YAML::Flow << matToVec(camera_matrix);
    out << YAML::EndMap;

  out << YAML::Key << "distortion_coefficients";
    out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "rows";
        out << YAML::Value << dist_coeffs.rows;
      out << YAML::Key << "cols";
        out << YAML::Value << dist_coeffs.cols;
      out << YAML::Key << "data";
        out << YAML::Value << YAML::Flow << matToVec(dist_coeffs);
    out << YAML::EndMap;

  out << YAML::Key << "reprojection_error";
    out << YAML::Value << rms;

  out << YAML::Key << "total_average_error";
    out << YAML::Value << total_average_error;

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

bool saveEstimatedExtrinsics(const std::string &estimated_poses_path, 
  const std::string &data_set, const std::vector<cv::Mat> &rvecs, 
  const std::vector<cv::Mat> &tvecs)
{
  YAML::Emitter out;

  out << YAML::BeginMap;

  if (rvecs.size() == tvecs.size())
  {
    for (std::size_t i = 0; i < rvecs.size(); i++)
    {
      out << YAML::Key << "rvec_" + std::to_string(i);
        out << YAML::Value << YAML::Flow << matToVec(rvecs[i].t());
      out << YAML::Key << "tvec_" + std::to_string(i);
        out << YAML::Value << YAML::Flow << matToVec(tvecs[i].t());
    }
  }
  else {return false;}

  out << YAML::EndMap;

  // Write to yaml file
  if (out.good())
  {
    std::ofstream yaml_file(estimated_poses_path);
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

  // Convert target points to std::vector<std::vector<cv::Point2f>>
  // ICL::TargetDefinition::points = std::vector<ICL::Point3D>
  std::vector<std::vector<cv::Point3f>> object_points;
  object_points.reserve(observation_data.size());

  ICL::TargetDefinition target_definition = target.getDefinition();

  std::vector<cv::Point3f> object_points_temp;
  object_points_temp.reserve(target_definition.points.size());
  for (std::size_t i = 0; i < target_definition.points.size(); i++)
  {
    object_points_temp.push_back(point3dToPoint3f(target_definition.points[i]));
  }
  for (std::size_t i = 0; i < observation_data.size(); i++)
  {
    object_points.push_back(object_points_temp);
  }

  // Convert observation data to std::vector<std::vector<cv::Point2f>>
  // ICL::ObservationData = std::vector<std::vector<cv::Point2d>>
  std::vector<std::vector<cv::Point2f>> image_points;
  image_points.reserve(observation_data.size());

  for (std::size_t i = 0; i < observation_data.size(); i++)
  {
    std::vector<cv::Point2f> observation_temp;
    observation_temp.reserve(observation_data[0].size());
    for (std::size_t j = 0; j < observation_data[i].size(); j++)
    {
      observation_temp.push_back(point2dToPoint2f(observation_data[i][j]));
    }
    image_points.push_back(observation_temp);
  }

  cv::Size image_size = cal_images[0].size();
  
  // Set seed parameters
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  camera_matrix.at<double>(0, 0) = 570.342224; // Fx
  camera_matrix.at<double>(1, 1) = 570.342224; // Fy
  camera_matrix.at<double>(0, 2) = 319.5; // Cx
  camera_matrix.at<double>(1, 2) = 239.5; // Cy

  // std::cout << camera_matrix << std::endl;

  cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
  
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  int flag = 0;

  // flag |= CV_CALIB_FIX_ASPECT_RATIO;  
  // flag |= CV_CALIB_ZERO_TANGENT_DIST; // Hell no.
  // flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
  flag |= CV_CALIB_USE_INTRINSIC_GUESS;
  flag |= CV_CALIB_FIX_K4;
  flag |= CV_CALIB_FIX_K5;
  flag |= CV_CALIB_FIX_K6;

  ROS_INFO_STREAM("Running Calibration on Data Set: " << data_set);
  double rms = cv::calibrateCamera(object_points, image_points, image_size,
    camera_matrix, dist_coeffs, rvecs, tvecs, flag);

  bool ok = cv::checkRange(camera_matrix) && cv::checkRange(dist_coeffs);

  if (ok) 
    ROS_INFO_STREAM("cv::checkRange() says OK");
  else 
    ROS_INFO_STREAM("cv::checkRange() does not say OK");

  ROS_INFO_STREAM("Calculating Reprojection Errors for Data Set: " << data_set);
  
  std::vector<float> reprojection_errors;
  double total_average_error = computeReprojectionErrors(object_points,
    image_points, rvecs, tvecs, camera_matrix, dist_coeffs, reprojection_errors);

  bool DISPLAY_RESULTS = true;
  if (DISPLAY_RESULTS)
  {
    ROS_INFO_STREAM("Data Set: " << data_set << " results:");
    ROS_INFO_STREAM("Re-projection error reported by cv::calibrateCamera: " << rms);
    ROS_INFO_STREAM("Total Average Error: " << total_average_error);
    ROS_INFO_STREAM("Camera Matrix = \n" << camera_matrix);
    ROS_INFO_STREAM("Distortion Coefficients = " << dist_coeffs);
  }

  std::string result_path = data_dir + "results/opencv_" + data_set + ".yaml";
  ROS_INFO_STREAM("Saving Results of Data Set: " << data_set << " to "
    << result_path);

  if (saveResultData(result_path, rms, total_average_error, camera_matrix, dist_coeffs))
  {
    ROS_INFO_STREAM("Data Set: " << data_set << " Results Saved To: " << result_path);
    ROS_INFO_STREAM("Data Successfully Saved!");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to Save Results for Data Set: " << data_set);
  }

#ifdef SAVE_ESTIMATED_EXTRINSICS
  std::string estimated_poses_path = data_dir + data_set + "/estimated_poses.yaml";
  if (!saveEstimatedExtrinsics(estimated_poses_path, data_set, rvecs, tvecs))
  {
    ROS_ERROR_STREAM("Failed to Save Estimated Poses for Data Set: " << data_set);
  }
#else
#endif
}

int main(int argc, char **argv)
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