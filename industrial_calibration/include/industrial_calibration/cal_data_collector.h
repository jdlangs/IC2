#ifndef CAL_DATA_COLLECTOR_H
#define CAL_DATA_COLLECTOR_H

#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
  sensor_msgs::JointState> SyncPolicy;

class CalDataCollector
{
public:
  CalDataCollector(ros::NodeHandle nh, ros::NodeHandle pnh);

  ~CalDataCollector(void) { }

  void collectData(void);

// Private Methods
private:
  void synchronizedMessageCallback(const sensor_msgs::ImageConstPtr &image_msg,
    const sensor_msgs::JointStateConstPtr &joint_state_msg);

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

  inline bool drawGrid(cv::Mat &image);

  inline void printTransform(const tf::StampedTransform &transform);

  inline void writeTransformToYAML(YAML::Emitter &out, const std::string &from_link,
    const std::string &to_link, const tf::StampedTransform &transform);

  inline void writeJointStateToYAML(YAML::Emitter &out, 
    const std::vector<std::string> &joint_names, const std::vector<float> &joint_state);

  inline void writeIntrinsicMatrixToYAML(YAML::Emitter &out,
    const std::vector<double> &intrinsic_matrix);

  inline void saveCalibrationData(const cv::Mat &image, 
    const std::vector<std::string> &joint_names, const std::vector<float> &joint_state);

  void initDisplayWindow(const std::string &window_name);

  bool checkSettings(void);

// Private Variables
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf::TransformListener tf_;
  std::string cv_window_name_;
  std::string save_path_;
  std::string from_link_;
  std::string to_link_;
  int pattern_cols_;
  int pattern_rows_;
  std::size_t i_;
  ros::Subscriber camera_info_sub_;
  std::vector<double> intrinsic_matrix_;
};
#endif // CAL_DATA_COLLECTOR_H