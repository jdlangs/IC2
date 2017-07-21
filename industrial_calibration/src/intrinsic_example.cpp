#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_calibration_libs/industrial_calibration_libs.h>

typedef std::vector<double> JointStates;
typedef std::vector<double> Translation;
typedef std::vector<double> Quaternion;
typedef std::vector<double> RotationRad;
typedef std::vector<double> RotationDeg;

struct LinkData
{
  JointStates joint_states;
  Translation translation;
  Quaternion rotation_quat;
  RotationRad rotation_rad;
  RotationDeg rotation_deg;
};

bool parseYAML(const YAML::Node &node, const std::string &var_name, 
  std::vector<double> &var_value);

bool loadLinkData(const std::size_t &index, const std::string &path,
  LinkData *link_data);

void printVector(const std::vector<double> &vec);

bool convertToPose6D(const std::vector<LinkData> &link_data, 
  std::vector<industrial_calibration_libs::Pose6D> *link_poses);

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
  std::string file_path = path + std::to_string(index+1) + ".yaml";

  YAML::Node data_yaml;
  try
  {
    data_yaml = YAML::LoadFile(file_path);
    if (!data_yaml["joints"]) {return false;}
  }
  catch (YAML::BadFile &bf) {return false;}

  success &= parseYAML(data_yaml, "joints", link_data->joint_states);
  success &= parseYAML(data_yaml, "translation", link_data->translation);
  success &= parseYAML(data_yaml, "rotation_quat", link_data->rotation_quat);
  success &= parseYAML(data_yaml, "rotation_rad", link_data->rotation_rad);
  success &= parseYAML(data_yaml, "rotation_deg", link_data->rotation_deg);

  return success;
}

void printVector(const std::vector<double> &vec)
{
  for (std::size_t i = 0; i < vec.size(); i++)
  {
    ROS_INFO_STREAM(vec[i]);
  }
}

bool convertToPose6D(const std::vector<LinkData> &link_data, 
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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intrinsic_example");
  ros::NodeHandle pnh("~");

  std::string data_path;
  pnh.getParam("data_path", data_path);

  // Load Target Data
  industrial_calibration_libs::Target target;
  target.loadTargetFromYAML(data_path + "mcircles_7x5/mcircles_7x5.yaml");

  // Load Calibration Images
  const std::size_t num_images = 24;
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = data_path + "mcircles_7x5/dataset_2/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i+1) + ".jpg";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
    calibration_images.push_back(image);    
  }

  // Extract Observations
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    cv::Mat output_image;
    observation_extractor.extractObservation(calibration_images[i],
      output_image);
  }

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData(); 

  // Load Link Data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp_link_data;
    loadLinkData(i, data_path + "mcircles_7x5/dataset_2/tf/", &temp_link_data);
    link_data.push_back(temp_link_data);
  }

  // Convert Link Data to a vector of Pose6D poses
  double intrinsics[9] = {0};
  intrinsics[0] = 944.72;
  intrinsics[1] = 940.92;
  intrinsics[2] = 925.56;
  intrinsics[3] = 518.89;

  // Set camera extrinsics seed
  industrial_calibration_libs::Pose6D link_6_to_camera;
  link_6_to_camera.setOrigin(0.125, 0.000, 0.091);
  link_6_to_camera.setQuaternion(0.500, 0.500, 0.500, 0.500);

  double extrinsics[6];
  extrinsics[0] = link_6_to_camera.ax;
  extrinsics[1] = link_6_to_camera.ay;
  extrinsics[2] = link_6_to_camera.az;
  extrinsics[3] = link_6_to_camera.x;
  extrinsics[4] = link_6_to_camera.y;
  extrinsics[5] = link_6_to_camera.z;

  double target_to_base[6] = {0};

  target_to_base[3] = 0.75;
  target_to_base[4] = 0.1;
  target_to_base[5] = 0.1;

  std::vector<industrial_calibration_libs::Pose6D> link_poses;
  convertToPose6D(link_data, &link_poses);

  industrial_calibration_libs::MovingCameraOnWristStaticTargetIntrinsic calibration(observation_data, target);

  calibration.initKnownValues(link_poses);
  calibration.initSeedValues(extrinsics, target_to_base, intrinsics);

  calibration.runCalibration();

  calibration.displayCovariance();

  industrial_calibration_libs::MovingCameraOnWristStaticTargetIntrinsic::Result results = calibration.getResults();

  // Note(gChiou): Prints out calibration results
  ROS_INFO_STREAM("Extrinsic Parameters");
  ROS_INFO_STREAM("Translation x: " << results.extrinsics[3]);
  ROS_INFO_STREAM("Translation y: " << results.extrinsics[4]);
  ROS_INFO_STREAM("Translation z: " << results.extrinsics[5]);
  ROS_INFO_STREAM("Rotation x: " << results.extrinsics[0]);
  ROS_INFO_STREAM("Rotation y: " << results.extrinsics[1]);
  ROS_INFO_STREAM("Rotation z: " << results.extrinsics[2]);

  ROS_INFO_STREAM("Target to Base");
  ROS_INFO_STREAM("Translation x: " << results.target_to_base[3]);
  ROS_INFO_STREAM("Translation y: " << results.target_to_base[4]);
  ROS_INFO_STREAM("Translation z: " << results.target_to_base[5]);
  ROS_INFO_STREAM("Rotation x: " << results.target_to_base[0]);
  ROS_INFO_STREAM("Rotation y: " << results.target_to_base[1]);
  ROS_INFO_STREAM("Rotation z: " << results.target_to_base[2]);

  ROS_INFO_STREAM("Intrinsic Parameters");
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