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
#define DISPLAY_RESULTS
// #define DEBUG_OUTPUTS

struct CalibrationResult
{
  std::size_t observation_set;
  double rms;
  double total_average_error;
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
};

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

bool saveResultData(const std::string &result_path,
  const std::vector<CalibrationResult> &results);

bool saveEstimatedExtrinsics(const std::string &estimated_poses_path, 
  const std::string &data_set, const std::vector<cv::Mat> &rvecs, 
  const std::vector<cv::Mat> &tvecs);

void calibrateObservationSet(const std::string &data_dir,
  const ICL::ObservationData &observation_data,
  const cv::Size &image_size,
  const ICL::TargetDefinition &target_definition,
  const std::size_t &observation_set_index,
  std::vector<CalibrationResult> &results);


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

bool saveResultData(const std::string &result_path, 
  const std::vector<CalibrationResult> &results)
{
  std::ofstream result_file;
  result_file.open(result_path);

  result_file << "Set,Fx,Fy,Cx,Cy,k1,k2,p1,p2,k3,Reprj Error" << '\n';
  for (std::size_t i = 0; i < results.size(); i++)
  {
    result_file << results[i].observation_set << ','
      << results[i].camera_matrix.at<double>(0,0) << ','
      << results[i].camera_matrix.at<double>(1,1) << ','
      << results[i].camera_matrix.at<double>(0,2) << ','
      << results[i].camera_matrix.at<double>(1,2) << ','
      << results[i].dist_coeffs.at<double>(0,0) << ','
      << results[i].dist_coeffs.at<double>(1,0) << ','
      << results[i].dist_coeffs.at<double>(2,0) << ','
      << results[i].dist_coeffs.at<double>(3,0) << ','
      << results[i].dist_coeffs.at<double>(4,0) << ','
      << results[i].rms << '\n';
  }
  result_file.close();

  return true;
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


void calibrateObservationSet(const std::string &data_dir, 
  const ICL::ObservationData &observation_data,
  const cv::Size &image_size,
  const ICL::TargetDefinition &target_definition,
  const std::size_t &observation_set_index,
  std::vector<CalibrationResult> &results)
{
  // Convert target points to std::vector<std::vector<cv::Point2f>>
  // ICL::TargetDefinition::points = std::vector<ICL::Point3D>
  std::vector<std::vector<cv::Point3f>> object_points;
  object_points.reserve(observation_data.size());

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

  // Set seed parameters
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
  
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  int flag = 0;

  // flag |= CV_CALIB_FIX_ASPECT_RATIO;  
  // flag |= CV_CALIB_ZERO_TANGENT_DIST; // Hell no.
  // flag |= CV_CALIB_FIX_PRINCIPAL_POINT;
  // flag |= CV_CALIB_USE_INTRINSIC_GUESS;
  flag |= CV_CALIB_FIX_K4;
  flag |= CV_CALIB_FIX_K5;
  flag |= CV_CALIB_FIX_K6;

  ROS_INFO_STREAM("Running Calibration on Observation Set: " << observation_set_index);
  double rms = cv::calibrateCamera(object_points, image_points, image_size,
    camera_matrix, dist_coeffs, rvecs, tvecs, flag);

  bool ok = cv::checkRange(camera_matrix) && cv::checkRange(dist_coeffs);

  if (ok) 
    ROS_INFO_STREAM("cv::checkRange() says OK");
  else 
    ROS_INFO_STREAM("cv::checkRange() does not say OK");

  ROS_INFO_STREAM("Calculating Reprojection Errors for Observation Set: " 
    << observation_set_index);
  
  std::vector<float> reprojection_errors;
  double total_average_error = computeReprojectionErrors(object_points,
    image_points, rvecs, tvecs, camera_matrix, dist_coeffs, reprojection_errors);

#ifdef DISPLAY_RESULTS
  ROS_INFO_STREAM("Observation Set: " << observation_set_index << " results:");
  ROS_INFO_STREAM("Re-projection error reported by cv::calibrateCamera: " << rms);
  ROS_INFO_STREAM("Total Average Error: " << total_average_error);
  ROS_INFO_STREAM("Camera Matrix = \n" << camera_matrix);
  ROS_INFO_STREAM("Distortion Coefficients = \n" << dist_coeffs);
#endif

  // Set result
  CalibrationResult result;
  result.observation_set = observation_set_index;
  result.rms = rms;
  result.total_average_error = total_average_error;
  result.camera_matrix = camera_matrix;
  result.dist_coeffs = dist_coeffs;
  results[observation_set_index] = result;

#ifdef SAVE_ESTIMATED_EXTRINSICS
  std::string estimated_poses_path = data_dir + "results/" 
    + std::to_string(observation_set_index) + "_estimated_poses.yaml";
  if (!saveEstimatedExtrinsics(estimated_poses_path, 
    std::to_string(observation_set_index), rvecs, tvecs))
  {
    ROS_ERROR_STREAM("Failed to Save Estimated Poses for Observation Set: " << observation_set_index);
  }
#else
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "intrinsic_opencv_single");
  ros::NodeHandle pnh("~");

  std::string data_dir;
  pnh.getParam("data_dir", data_dir);
  data_dir = addSlashToEnd(data_dir);

  // This calibration will run on a single data set but leave out 
  // a single image every calibration.

  // Load target data
  ICL::Target target(data_dir + "mcircles_11x15.yaml");

  // Load Calibration Images
  CalibrationImages cal_images;
  ROS_INFO_STREAM("Loading calibration images from: " << data_dir);
  getCalibrationImages(data_dir, cal_images);

  // Extract Observations
  ROS_INFO_STREAM("Extracting observations from data");
  ICL::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < cal_images.size(); i++)
  {
    cv::Mat grid_image;
    observation_extractor.extractObservation(cal_images[i], grid_image);
  }

  // Get Observations from extractor
  ICL::ObservationData observation_data = observation_extractor.getObservationData();

  // Split observations into sets
  std::vector<ICL::ObservationData> observation_sets;
  observation_sets.reserve(observation_data.size());
  for (std::size_t i = 0; i < observation_data.size(); i++)
  {
    ICL::ObservationData temp_observation_data;
    temp_observation_data.reserve(observation_data.size()-1);

    for (std::size_t j = 0; j < observation_data.size(); j++)
    {
      if (i != j)
      {
        temp_observation_data.push_back(observation_data[j]);
      }
    }
    observation_sets.push_back(temp_observation_data);
  }

#ifdef DEBUG_OUTPUTS
  ROS_INFO_STREAM("Total Observation Sets: " << observation_sets.size());
  for (std::size_t i = 0; i  < observation_sets.size(); i++)
  {
    ROS_INFO_STREAM("Observation Set: " << i << " Total Observations: " << observation_sets[i].size());
  }
#endif

  // Results
  std::vector<CalibrationResult> results;
  results.resize(observation_sets.size());

  // Run the calibration on each set
  #pragma omp parallel for
  for (std::size_t i = 0; i < observation_sets.size(); i++)
  {
    calibrateObservationSet(data_dir, observation_sets[i], 
      cal_images[0].size(), target.getDefinition(), i,
      results);
  }

  std::string result_path = data_dir + "results/opencv_results.csv";
  ROS_INFO_STREAM("Saving Results to: " << result_path);
  saveResultData(result_path, results);

  return 0;
}