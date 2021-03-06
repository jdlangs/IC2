/*
  NOTE: These helper functions are just my lazy way of getting
  the "data" into the calibration library. This is not the best
  way to do it.
*/

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <industrial_calibration_libs/industrial_calibration_libs.h>

#include <boost/filesystem.hpp>

typedef std::vector<double> JointStates;
typedef std::vector<double> Translation;
typedef std::vector<double> Quaternion;
typedef std::vector<double> RotationRad;
typedef std::vector<double> RotationDeg;

typedef std::vector<cv::Mat> CalibrationImages;

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

bool loadLinkData(const std::string &file_path,
  LinkData *link_data, const std::string &node);

bool loadLinkData(const std::string &file_path,
  industrial_calibration_libs::Pose6D &pose, const std::string &node);

void printVector(const std::vector<double> &vec);

void convertToPose6D(const LinkData &link_data, 
  industrial_calibration_libs::Pose6D &link_pose);

bool convertToPose6D(const std::vector<LinkData> &link_data, 
  std::vector<industrial_calibration_libs::Pose6D> *link_poses);

void drawResultPoints(const cv::Mat &input_image, cv::Mat &output_image,
  const industrial_calibration_libs::ObservationPoints &observation_points,
  std::size_t rows, std::size_t cols);

void getQuaternion(double distance, double &qx, double &qy, double &qz,
  double &qw);

bool loadCameraInfo(const std::string &path, double *camera_info);

std::string addSlashToEnd(const std::string &directory);

bool isPNG(const std::string &file_name);

void getCalibrationImages(const std::string &path, CalibrationImages &images);

// https://stackoverflow.com/questions/33665257/
// how-to-overload-ostream-for-vector-to-print-all-collection-from-vector
template<class T>
std::ostream& operator<<(std::ostream& stream, const std::vector<T>& values)
{
    stream << "[ ";
    std::copy( std::begin(values), std::end(values), std::ostream_iterator<T>(stream, " ") );
    stream << ']';
    return stream;
}

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

bool loadLinkData(const std::string &file_path, LinkData *link_data, 
  const std::string &node)
{
  bool success = true;

  YAML::Node data_yaml;
  try
  {
    data_yaml = YAML::LoadFile(file_path);
    // if (!data_yaml["base_link_to_tool0"]) {return false;}
    if (!data_yaml[node]) {return false;}
  }
  catch (YAML::BadFile &bf) {return false;}

  success &= parseYAML(data_yaml[node], "Translation", link_data->translation);
  success &= parseYAML(data_yaml[node], "Quaternion", link_data->rotation_quat);
  return success;
}

bool loadLinkData(const std::string &file_path, 
  industrial_calibration_libs::Pose6D &pose, const std::string &node)
{
  LinkData link_data;
  bool success = loadLinkData(file_path, &link_data, node);

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

void printVector(const std::vector<double> &vec)
{
  for (std::size_t i = 0; i < vec.size(); i++)
  {
    ROS_INFO_STREAM(vec[i]);
  }
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

std::string addSlashToEnd(const std::string &directory)
{
  if (directory.back() != '/')
  {
    return directory + '/';
  }
  return directory;
}

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