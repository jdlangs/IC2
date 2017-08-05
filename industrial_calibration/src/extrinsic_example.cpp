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

void drawResultPoints(const cv::Mat &input_image, cv::Mat &output_image,
  const industrial_calibration_libs::ObservationPoints &observation_points,
  std::size_t rows, std::size_t cols);

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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "extrinsic_example");
  ros::NodeHandle pnh("~");

  std::string data_path;
  pnh.getParam("data_path", data_path);

  // Load Target Data
  industrial_calibration_libs::Target target;
  target.loadTargetFromYAML(data_path + "mcircles_10x10/mcircles_10x10.yaml");

  // Load Calibration Images
  const std::size_t num_images = 15;
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = data_path + "mcircles_10x10/extrinsic/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i) + ".png";
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

    // Visualize the "corners"
    // cv::imshow("grid", output_image);
    // cv::waitKey(0);
  }

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData(); 

  // Load Link Data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp_link_data;
    loadLinkData(i, data_path + "mcircles_10x10/extrinsic/tf/", &temp_link_data);
    link_data.push_back(temp_link_data);
  }

#if 0
  for (std::size_t i = 0; i < link_data.size(); i++)
  {
    ROS_INFO_STREAM("Translation");
    for (std::size_t j = 0; j < link_data[i].translation.size(); j++)
    {
      ROS_INFO_STREAM(link_data[i].translation[j]);
    }
    ROS_INFO_STREAM("Quaternion");
    for (std::size_t j = 0; j < link_data[i].rotation_quat.size(); j++)
    {
      ROS_INFO_STREAM(link_data[i].rotation_quat[j]);
    }
  }

  for (std::size_t i = 0; i < target.getData().points.size(); i++)
  {
    industrial_calibration_libs::Point3D point(target.getData().points[i]);
    ROS_INFO_STREAM(std::setprecision(4) << std::fixed << "Point: " << i+1 << " x: " << point.x << " y: " << point.y << " z:" << point.z);
  }

  ROS_INFO_STREAM("Total Observations: " << observation_data.size());
  for (std::size_t i = 0; i < observation_data.size(); i++)
  {
    ROS_INFO_STREAM("Observations for Image " << i << " Size: " << observation_data[i].size());
    ROS_INFO_STREAM("Observations for Image " << i << " Points:");
    for (std::size_t j = 0; j < observation_data[i].size(); j++)
    {
      ROS_INFO_STREAM(observation_data[i][j]);
    }
  }
#endif 

  // Setting initial conditions:
  double intrinsics[4];
  intrinsics[0] = 509.5179;
  intrinsics[1] = 511.6581;
  intrinsics[2] = 320.2695;
  intrinsics[3] = 208.9545;

  // Set camera extrinsics seed
  industrial_calibration_libs::Pose6D link_6_to_camera_;

  link_6_to_camera_.setOrigin(0.0197, 0.0908, 0.112141);
  link_6_to_camera_.setAngleAxis(0.0, 0.0, -3.14/2.0);

  industrial_calibration_libs::Pose6D link_6_to_camera = link_6_to_camera_.getInverse();

  double extrinsics[6];
  extrinsics[0] = link_6_to_camera.ax;
  extrinsics[1] = link_6_to_camera.ay;
  extrinsics[2] = link_6_to_camera.az;
  extrinsics[3] = link_6_to_camera.x;
  extrinsics[4] = link_6_to_camera.y;
  extrinsics[5] = link_6_to_camera.z;

  industrial_calibration_libs::Pose6D target_pose;

  // target_pose.setOrigin(0.62, 0.15, -0.15);
  // target_pose.setEulerZYX(-3.14/2.0, 0.0, 0.0);

  target_pose.setOrigin(0.0, 0.0, 0.0);
  target_pose.setEulerZYX(0.0, 0.0, 0.0);

  double target_to_base[6];

  target_to_base[0] = target_pose.ax;
  target_to_base[1] = target_pose.ay;
  target_to_base[2] = target_pose.az;
  target_to_base[3] = target_pose.x;
  target_to_base[4] = target_pose.y;
  target_to_base[5] = target_pose.z;

  std::vector<industrial_calibration_libs::Pose6D> link_poses;
  convertToPose6D(link_data, &link_poses);

// This draws the position of the dots based off of the uncalibrated position.
#if 0
{
  const double* camera_angle_axis(&extrinsics[0]);
  const double* camera_position(&extrinsics[3]);
  const double* target_angle_axis(&target_to_base[0]);
  const double* target_position(&target_to_base[3]);

  double fx = intrinsics[0];
  double fy = intrinsics[1];
  double cx = intrinsics[2];
  double cy = intrinsics[3];

  for (std::size_t i = 0; i < link_poses.size(); i++)
  {
    // if (i == 0 || i == 1 || i == 2 || i == 10 || i == 11 || i == 12) {continue;}
    industrial_calibration_libs::ObservationPoints observation_points;
    industrial_calibration_libs::Pose6D link_pose_inverse = link_poses[i].getInverse();

    for (std::size_t j = 0; j < target.getData().points.size(); j++)
    {
      industrial_calibration_libs::Point3D target_point(target.getData().points[j]);

      double world_point[3];
      double link_point[3];
      double camera_point[3];

      industrial_calibration_libs::transformPoint3D(target_angle_axis, target_position,
        target_point.asVector(), world_point);
      industrial_calibration_libs::poseTransformPoint(link_pose_inverse, world_point,
        link_point);
      industrial_calibration_libs::transformPoint(camera_angle_axis, camera_position,
        link_point, camera_point);

      double xp1 = camera_point[0];
      double yp1 = camera_point[1];
      double zp1 = camera_point[2];

      double xp, yp;
      if (zp1 == 0.0)
      {
        xp = xp1;
        yp = yp1;
      }
      else
      {
        xp = xp1 / zp1;
        yp = yp1 / zp1;
      }

      double point_x = fx * xp + cx;
      double point_y = fy * yp + cy;

      cv::Point2d cv_point(point_x, point_y);
      observation_points.push_back(cv_point);
    }

    cv::Mat result_image;
    drawResultPoints(calibration_images[i], result_image, observation_points,
      target.getData().target_rows, target.getData().target_cols);
    cv::imshow("Result Image", result_image);
    cv::waitKey(0);
  }
}
#endif

#if 1
  industrial_calibration_libs::MovingCameraOnWristStaticTargetExtrinsic calibration(observation_data, target);

  calibration.initKnownValues(link_poses, intrinsics);
  calibration.initSeedValues(extrinsics, target_to_base);

  calibration.runCalibration();

  calibration.displayCovariance();

  industrial_calibration_libs::MovingCameraOnWristStaticTargetExtrinsic::Result results = calibration.getResults();

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

  ROS_INFO_STREAM("Initial Cost: " << calibration.getInitialCost());
  ROS_INFO_STREAM("Final Cost: " << calibration.getFinalCost());

#if 0
  industrial_calibration_libs::Pose6D result_pose_(results.extrinsics[3],
    results.extrinsics[4], results.extrinsics[5], results.extrinsics[0], 
    results.extrinsics[1], results.extrinsics[2]);

  industrial_calibration_libs::Pose6D result_pose = result_pose_.getInverse();

  double ez, ey, ex;
  result_pose.getEulerZYX(ez, ey, ex);

  ROS_INFO_STREAM("Extrinsic Parameters");
  ROS_INFO_STREAM("Translation x: " << result_pose.x);
  ROS_INFO_STREAM("Translation y: " << result_pose.y);
  ROS_INFO_STREAM("Translation z: " << result_pose.z);
  ROS_INFO_STREAM("Rotation x: " << ex);
  ROS_INFO_STREAM("Rotation y: " << ey);
  ROS_INFO_STREAM("Rotation z: " << ez);  
#endif
  
  // Draw the results back onto the image
  const double* camera_angle_axis(&results.extrinsics[0]);
  const double* camera_position(&results.extrinsics[3]);
  const double* target_angle_axis(&results.target_to_base[0]);
  const double* target_position(&results.target_to_base[3]);

  double fx = intrinsics[0];
  double fy = intrinsics[1];
  double cx = intrinsics[2];
  double cy = intrinsics[3];

  for (std::size_t i = 0; i < link_poses.size(); i++)
  {
    industrial_calibration_libs::ObservationPoints observation_points;
    industrial_calibration_libs::Pose6D link_pose_inverse = link_poses[i].getInverse();

    for (std::size_t j = 0; j < target.getData().points.size(); j++)
    {
      industrial_calibration_libs::Point3D target_point(target.getData().points[j]);

      double world_point[3];
      double link_point[3];
      double camera_point[3];

      industrial_calibration_libs::transformPoint3D(target_angle_axis, target_position,
        target_point.asVector(), world_point);
      industrial_calibration_libs::poseTransformPoint(link_pose_inverse, world_point,
        link_point);
      industrial_calibration_libs::transformPoint(camera_angle_axis, camera_position,
        link_point, camera_point);

      double xp1 = camera_point[0];
      double yp1 = camera_point[1];
      double zp1 = camera_point[2];

      double xp, yp;
      if (zp1 == 0.0)
      {
        xp = xp1;
        yp = yp1;
      }
      else
      {
        xp = xp1 / zp1;
        yp = yp1 / zp1;
      }

      double point_x = fx * xp + cx;
      double point_y = fy * yp + cy;

      cv::Point2d cv_point(point_x, point_y);
      observation_points.push_back(cv_point);
    }

    cv::Mat result_image;
    drawResultPoints(calibration_images[i], result_image, observation_points,
      target.getData().target_rows, target.getData().target_cols);
    cv::imshow("Result Image", result_image);
    cv::waitKey(0);
  }
#endif
}
