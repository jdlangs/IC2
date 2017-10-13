#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#define TEST_CONSOLE_OUTPUT
#ifdef TEST_CONSOLE_OUTPUT
#define CONSOLE_OUTPUT(str) do { std::cerr << "[>>>>>>>>>>] " << str << '\n'; } while  (false)
#else
#define CONSOLE_OUTPUT(str) do { } while (false)
#endif

#include <string>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <industrial_calibration_libs/industrial_calibration_libs.h>

static void
printPoints(const std::vector<industrial_calibration_libs::Point3D> &points);

static void
showImage(const cv::Mat &image, const std::string &window_name);

static void
printObservationData(const industrial_calibration_libs::ObservationData &data);

static void
loadImagesFromPath(const std::string &path, const std::size_t &num_images,
  std::vector<cv::Mat> &images);

static void
printPoint3DVector(const std::vector<industrial_calibration_libs::Point3D> &points)
{
  for (std::size_t i = 0; i < points.size(); i++)
  {
    industrial_calibration_libs::Point3D point(points[i]);
    CONSOLE_OUTPUT(std::setprecision(4) << std::fixed << "Point: " 
      << i+1 << " x: " << point.x << " y: " << point.y 
      << " z:" << point.z);    
  }
}

static void
showImage(const cv::Mat &image, const std::string &window_name)
{
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::imshow(window_name, image);
  cv::waitKey(0);
  cv::destroyWindow(window_name);
}

static void
printObservationData(const industrial_calibration_libs::ObservationData &data)
{
  for (std::size_t i = 0; i < data.size(); i++)
  {
    CONSOLE_OUTPUT("Observations for Image: " << i << " has Size: " 
      << data[i].size());
    CONSOLE_OUTPUT("Observations for Image: " << i << " has Points: ");
    for (std::size_t j = 0; j < data[i].size(); j++)
    {
      CONSOLE_OUTPUT(data[i][j]);
    }
  }
}

static void
loadImagesFromPath(const std::string &path, const std::size_t &num_images,
  std::vector<cv::Mat> &images)
{
  images.clear();
  images.reserve(num_images);

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = path + std::to_string(i) + ".png";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    ASSERT_TRUE(!image.empty());
    ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    images.push_back(image);    
  }
}

// These are used to test a calibration.
// Taken from industrial_calibration/helper_functions.h
typedef std::vector<double> Translation;
typedef std::vector<double> Quaternion;

struct LinkData
{
  Translation translation;
  Quaternion rotation_quat;
};

static bool 
parseYAML(const YAML::Node &node, const std::string &var_name, 
  std::vector<double> &var_value);

static bool 
loadLinkData(const std::string &file_path, LinkData *link_data, 
  const std::string &node);

static void 
convertToPose6D(const LinkData &link_data, 
  industrial_calibration_libs::Pose6D &link_pose);

static bool 
convertToPose6D(const std::vector<LinkData> &link_data, 
  std::vector<industrial_calibration_libs::Pose6D> *link_poses);

static void 
getQuaternion(double distance, double &qx, double &qy, double &qz,
  double &qw);

static bool 
loadCameraInfo(const std::string &path, double *camera_info);

static bool 
parseYAML(const YAML::Node &node, const std::string &var_name, 
  std::vector<double> &var_value)
{

  var_value.clear();
  if (node[var_name])
  {
    const YAML::Node n = node[var_name];
    var_value.reserve(n.size());
    for (std::size_t i = 0; i < n.size(); i++)
    {
      double value = n[i].as<double>();
      var_value.push_back(value);
    }
    if (var_value.size() == n.size()) {return true;}
  }
  return false;
}

static bool 
loadLinkData(const std::string &file_path, LinkData *link_data,
  const std::string &node)
{
  bool success = true;

  YAML::Node data_yaml;
  try
  {
    data_yaml = YAML::LoadFile(file_path);
    if (!data_yaml[node]) {return false;}
  }
  catch (YAML::BadFile &bf) {return false;}

  success &= parseYAML(data_yaml[node], "Translation", link_data->translation);
  success &= parseYAML(data_yaml[node], "Quaternion", link_data->rotation_quat);
  return success;
}

static void 
convertToPose6D(const LinkData &link_data, 
  industrial_calibration_libs::Pose6D &link_pose)
{
  double tx = link_data.translation[0];
  double ty = link_data.translation[1];
  double tz = link_data.translation[2];
  double qx = link_data.rotation_quat[0];
  double qy = link_data.rotation_quat[1];
  double qz = link_data.rotation_quat[2];
  double qw = link_data.rotation_quat[3];

  link_pose.setOrigin(tx, ty, tz);
  link_pose.setQuaternion(qx, qy, qz, qw);  
}

static bool 
convertToPose6D(const std::vector<LinkData> &link_data, 
  std::vector<industrial_calibration_libs::Pose6D> *link_poses)
{
  link_poses->reserve(link_data.size());

  for (std::size_t i = 0; i < link_data.size(); i++)
  {
    industrial_calibration_libs::Pose6D link_pose;

    convertToPose6D(link_data[i], link_pose);

    link_poses->push_back(link_pose);
  }

  if (link_poses->size() == link_data.size()) {return true;}
  else {return false;}
}

static void 
getQuaternion(double distance, double &qx, double &qy, double &qz,
  double &qw)
{
  Eigen::Matrix3d m;

  m(0,0) =  1; m(0,1) =   0; m(0,2) =  0;
  m(1,0) =  0; m(1,1) =  -1; m(1,2) =  0;
  m(2,0) =  0; m(2,1) =   0; m(2,2) = -1;

  industrial_calibration_libs::Pose6D temp_pose;
  temp_pose.setBasis(m);
  temp_pose.setOrigin(-0.1, -0.1, distance);
  temp_pose.getQuaternion(qx, qy, qz, qw);   
}

static bool 
loadCameraInfo(const std::string &path, double *camera_info)
{
  bool success = true;

  YAML::Node camera_info_yaml;
  try
  {
    camera_info_yaml = YAML::LoadFile(path);
    if (!camera_info_yaml["camera_matrix"]) {return false;}
    if (!camera_info_yaml["distortion_coefficients"]) {return false;}
  }
  catch (YAML::BadFile &bf) {return false;}

  std::vector<double> camera_matrix;
  std::vector<double> distortion_coefficients;

  success &= parseYAML(camera_info_yaml["camera_matrix"], "data", camera_matrix);
  success &= parseYAML(camera_info_yaml["distortion_coefficients"], 
    "data", distortion_coefficients);

  // Assuming it is formatted correctly
  camera_info[0] = camera_matrix[0]; // focal length x
  camera_info[1] = camera_matrix[4]; // focal_length y
  camera_info[2] = camera_matrix[2]; // optical center x
  camera_info[3] = camera_matrix[5]; // optical center y
  camera_info[4] = distortion_coefficients[0]; // distortion k1
  camera_info[5] = distortion_coefficients[1]; // distortion k2
  camera_info[6] = distortion_coefficients[2]; // distortion k3
  camera_info[7] = distortion_coefficients[3]; // distortion p1
  camera_info[8] = distortion_coefficients[4]; // distortion p2

  return success;
}
#endif // TEST_UTILS_H
