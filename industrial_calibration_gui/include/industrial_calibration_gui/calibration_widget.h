#ifndef CALIBRATION_WIDGET_H
#define CALIBRATION_WIDGET_H

#include <QWidget>
#include <QFileDialog>

// Industrial Calibration
#include <ui_calibration_widget.h>
#include <industrial_calibration_libs/industrial_calibration_libs.h>

// Standard Library
#include <fstream>
#include <thread>
#include <mutex>

// Boost
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// ROS
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/master.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>

// YAML
#include <yaml-cpp/yaml.h>

namespace Ui
{
class CalibrationWidget;
}

namespace industrial_calibration_gui
{
enum CalibrationType
{
  welcome_screen,
  camera_on_wrist_extrinsic,
  camera_on_wrist_intrinsic,
  camera_in_world_extrinsic,
  camera_in_world_intrinsic
};

class CalibrationWidget : public QWidget
{
  Q_OBJECT
public:
  CalibrationWidget(QWidget* parent = 0);

  virtual ~CalibrationWidget();

protected Q_SLOTS:
  // Start page
  void instructionsCheckbox(void);
  void startDataCollectionButton(void);
  void selectCalibrationTypeComboBox(void);
  void updateCalibrationTypeText(int current_index);  
  
  // Manual data collection page
  #define CONSOLE_LOG_INFO(args) \
  { \
    ROS_INFO_STREAM(args); \
    std::stringstream ss; \
    ss << args; \
    consoleLogInfo(ss.str()); \
  }

  #define CONSOLE_LOG_ERROR(args) \
  { \
    ROS_ERROR_STREAM(args); \
    std::stringstream ss; \
    ss << args; \
    consoleLogError(ss.str()); \
  }

  void consoleLogInfo(const std::string &message);
  void consoleLogError(const std::string &message);
  void outputLocationButton(void);
  void outputLocationLine(void);
  void loadTargetLine(void);
  void loadTargetButton(void);
  void setTargetLines(const industrial_calibration_libs::Target &target);
  void updateTopicList(void);
  void updateFrameList(void);
  void setInputsButton(void);
  bool checkEmptyLines(void);
  bool checkTarget(const industrial_calibration_libs::TargetDefinition &target_definition);
  void collectData(const std::string &base_link, const std::string &tip_link,
    const std::string &camera_frame, const std::string &image_topic, 
    const std::string &camera_info_topic);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  bool drawGrid(cv::Mat &image);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
  void saveImageButton(void);
  void startCalibrationButton(void);
  void runCameraOnWristExtrinsic(void);
  void runCameraOnWristIntrinsic(void);
  void runCameraInWorldExtrinsic(void);
  void runCameraInWorldIntrinsic(void);
  industrial_calibration_libs::Pose6D tfToPose6D(const tf::StampedTransform &t);  
  industrial_calibration_libs::ObservationData cvMatToObservations(const std::vector<cv::Mat> &images);
  void saveData(const std::string &directory);
  std::string getDateTimeString(void);
  void tfToYAML(const std::string &filename, const tf::StampedTransform &transform,
    const std::string &from_link, const std::string &to_link);
  void refreshComboBoxes(void);

protected:
  Ui::CalibrationWidget* ui_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

private:
  // GUI variables
  CalibrationType calibration_type_;
  bool instructions_checkbox_state_;
  QString save_data_directory_;
  bool target_set_from_file_;
  bool collecting_data_;
  std::mutex topic_list_mutex_;
  std::mutex frame_list_mutex_;
  std::vector<std::string> camera_info_topic_list_;
  std::vector<std::string> image_topic_list_;
  std::vector<std::string> frame_list_;

  // Data collection variables
  tf::TransformListener tf_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber camera_image_subscriber_;
  image_transport::Publisher grid_image_publisher_;
  ros::Subscriber camera_info_subscriber_;
  sensor_msgs::CameraInfo camera_info_;
  cv::Mat camera_image_;
  std::mutex camera_image_mutex_;

  // Calibration variables
  industrial_calibration_libs::Target target_;
  std::string base_link_;
  std::string tip_link_;
  std::string camera_frame_;
  std::vector<cv::Mat> observation_images_;
  std::vector<tf::StampedTransform> base_to_tool_transforms_;
  std::vector<tf::StampedTransform> tool_to_camera_transforms_;
  bool tool_to_camera_transforms_match_;
};

bool operator!=(const tf::StampedTransform &t1, const tf::StampedTransform &t2);

} // namespace industrial_calibration_gui

#endif //CALIBRATION_WIDGET_H
