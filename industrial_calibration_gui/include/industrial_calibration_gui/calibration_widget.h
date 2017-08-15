#ifndef CALIBRATION_WIDGET_H
#define CALIBRATION_WIDGET_H

#include <QWidget>
#include <QFileDialog>

#include <ros/ros.h>
#include <ros/console.h>
#include <ui_calibration_widget.h>
#include <industrial_calibration_libs/industrial_calibration_libs.h>

// CUT THESE DOWN
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
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
#include <thread>
#include <tf/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace Ui
{
  class CalibrationWidget;
}

namespace industrial_calibration_gui
{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
  sensor_msgs::JointState> SyncPolicy;

class CalibrationWidget : public QWidget
{
  Q_OBJECT
public:
  CalibrationWidget(QWidget* parent = 0);

  virtual ~CalibrationWidget();

protected Q_SLOTS:
  // Start page
  void instructionsCheckbox(void);
  void startCalibrationButton(void);
  void selectCalibrationTypeComboBox(void);
  void updateCalibrationTypeText(int current_index);  
  
  // Data collection page
  void outputLocationButton(void);
  void outputLocationLine(void);
  void loadTargetLine(void);
  void loadTargetButton(void);
  void setTargetLines(const industrial_calibration_libs::Target &target);
  void setInputsButton(void);
  bool checkEmptyLines(void);
  bool checkTarget(const industrial_calibration_libs::TargetDefinition &target_definition);
  void collectData(const std::string &base_link, const std::string &tip_link,
    const std::string &camera_frame, const std::string &image_topic, 
    const std::string &camera_info_topic);
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

protected:
  Ui::CalibrationWidget* ui_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

private:
  // GUI variables
  bool instructions_checkbox_state_;
  QString save_data_directory_;
  bool target_set_from_file_;
  bool collecting_data_;

  // Data collection variables
  ros::Subscriber camera_info_sub_;
  tf::TransformListener tf_;

  // Calibration variables
  industrial_calibration_libs::Target target_;
};
} // namespace industrial_calibration_gui

#endif //CALIBRATION_WIDGET_H
