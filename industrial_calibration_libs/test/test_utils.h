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
loadLinkData(const std::size_t &index, const std::string &path,
  LinkData *link_data);

static bool 
convertToPose6D(const std::vector<LinkData> &link_data, 
  std::vector<industrial_calibration_libs::Pose6D> *link_poses);

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
loadLinkData(const std::size_t &index, const std::string &path,
  LinkData *link_data)
{
  bool success = true;
  std::string file_path = path + std::to_string(index) + ".yaml";

  YAML::Node data_yaml;
  try
  {
    data_yaml = YAML::LoadFile(file_path);
    if (!data_yaml["base_link_to_tool0"]) {return false;}
  }
  catch (YAML::BadFile &bf) {return false;}

  success &= parseYAML(data_yaml["base_link_to_tool0"], "Translation", link_data->translation);
  success &= parseYAML(data_yaml["base_link_to_tool0"], "Quaternion", link_data->rotation_quat);
  return success;
}

static bool 
convertToPose6D(const std::vector<LinkData> &link_data, 
  std::vector<industrial_calibration_libs::Pose6D> *link_poses)
{
  link_poses->reserve(link_data.size());

  for (std::size_t i = 0; i < link_data.size(); i++)
  {
    industrial_calibration_libs::Pose6D link_pose;
    double tx = link_data[i].translation[0];
    double ty = link_data[i].translation[1];
    double tz = link_data[i].translation[2];
    double qx = link_data[i].rotation_quat[0];
    double qy = link_data[i].rotation_quat[1];
    double qz = link_data[i].rotation_quat[2];
    double qw = link_data[i].rotation_quat[3];
    link_pose.setOrigin(tx, ty, tz);
    link_pose.setQuaternion(qx, qy, qz, qw);

    link_poses->push_back(link_pose);
  }

  if (link_poses->size() == link_data.size()) {return true;}
  else {return false;}
}



#endif // TEST_UTILS_H
