/*
  NOTE: These helper functions are just my lazy way of getting
  the "data" into the calibration library. This is not the best
  way to do it.
*/

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <industrial_calibration_libs/industrial_calibration_libs.h>

typedef std::vector<double> JointStates;
typedef std::vector<double> Translation;
typedef std::vector<double> Quaternion;
typedef std::vector<double> RotationRad;
typedef std::vector<double> RotationDeg;

struct LinkData
{
  Translation translation;
  Quaternion rotation_quat;
  JointStates joint_states;
  RotationRad rotation_rad;
  RotationDeg rotation_deg;
};

bool parseYAML(const YAML::Node &node, const std::string &var_name, 
  std::vector<double> &var_value);

bool loadLinkData(const std::size_t &index, const std::string &path,
  LinkData *link_data);

bool loadLinkData2(const std::size_t &index, const std::string &path,
  LinkData *link_data, bool verification = false);

void printVector(const std::vector<double> &vec);

void convertToPose6D(const LinkData &link_data, 
  industrial_calibration_libs::Pose6D &link_pose);

bool convertToPose6D(const std::vector<LinkData> &link_data, 
  std::vector<industrial_calibration_libs::Pose6D> *link_poses);

bool loadLinkData(const std::size_t &index, const std::string &path,
  industrial_calibration_libs::Pose6D &pose);

void drawResultPoints(const cv::Mat &input_image, cv::Mat &output_image,
  const industrial_calibration_libs::ObservationPoints &observation_points,
  std::size_t rows, std::size_t cols);

void getQuaternion(double distance, double &qx, double &qy, double &qz,
  double &qw);

bool loadCameraInfo(const std::string &path, double *camera_info);

bool parseYAML(const YAML::Node &node, const std::string &var_name, 
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

bool loadLinkData(const std::size_t &index, const std::string &path,
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

bool loadLinkData2(const std::size_t &index, const std::string &path,
  LinkData *link_data, bool verification)
{
  bool success = true;

  std::string file_path;
  if (verification)
  {
    file_path = path + "v" + std::to_string(index) + ".yaml";
  }
  else
  {
    file_path = path + std::to_string(index) + ".yaml";
  }

  YAML::Node data_yaml;
  try
  {
    data_yaml = YAML::LoadFile(file_path);
    if (!data_yaml["base_to_tool0"]) {return false;}
  }
  catch (YAML::BadFile &bf) {return false;}

  success &= parseYAML(data_yaml["base_to_tool0"], "Translation", link_data->translation);
  success &= parseYAML(data_yaml["base_to_tool0"], "Quaternion", link_data->rotation_quat);
  return success;
}

void printVector(const std::vector<double> &vec)
{
  for (std::size_t i = 0; i < vec.size(); i++)
  {
    ROS_INFO_STREAM(vec[i]);
  }
}

bool loadLinkData(const std::size_t &index, const std::string &path,
  industrial_calibration_libs::Pose6D &pose)
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

  LinkData link_data;
  success &= parseYAML(data_yaml["base_link_to_tool0"], "Translation", link_data.translation);
  success &= parseYAML(data_yaml["base_link_to_tool0"], "Quaternion", link_data.rotation_quat);

  double tx, ty, tz, qx, qy, qz, qw;
  tx = link_data.translation[0];
  ty = link_data.translation[1];
  tz = link_data.translation[2];
  qx = link_data.rotation_quat[0];
  qy = link_data.rotation_quat[1];
  qz = link_data.rotation_quat[2];
  qw = link_data.rotation_quat[3];

  pose.setOrigin(tx, ty, tz);
  pose.setQuaternion(qx, qy, qz, qw);

  return success;
}

void convertToPose6D(const LinkData &link_data, 
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

bool convertToPose6D(const std::vector<LinkData> &link_data, 
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

void drawResultPoints(const cv::Mat &input_image, cv::Mat &output_image,
  const industrial_calibration_libs::ObservationPoints &observation_points,
  std::size_t rows, std::size_t cols)
{
  const int RADIUS = 5;
  input_image.copyTo(output_image);

  for (std::size_t i = 0; i < observation_points.size(); i++)
  {
    if (i == (rows*cols) - cols)
      cv::circle(output_image, observation_points[i], 2*RADIUS, cv::Scalar(0, 0, 255), -1);
    else if (i == (rows*cols -1))
      cv::circle(output_image, observation_points[i], RADIUS, cv::Scalar(255, 0, 0), -1);
    else
      cv::circle(output_image, observation_points[i], RADIUS, cv::Scalar(0, 255, 0), -1);
  }
}

void getQuaternion(double distance, double &qx, double &qy, double &qz,
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

bool loadCameraInfo(const std::string &path, double *camera_info)
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
