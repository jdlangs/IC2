#include <ros/console.h>
#include <QVBoxLayout>
#include <industrial_calibration_gui/calibration_panel.h>
#include <industrial_calibration_gui/calibration_widget.h>

namespace industrial_calibration_gui
{
CalibrationPanel::CalibrationPanel(QWidget* parent) : rviz::Panel(parent)
{
  ROS_INFO("Loaded simple calibration panel");

  QVBoxLayout* layout = new QVBoxLayout(this);
  widget_ = new CalibrationWidget();
  layout->addWidget(widget_);
  setLayout(layout);
}

CalibrationPanel::~CalibrationPanel() {}

void CalibrationPanel::onInitialize()
{
  ROS_INFO("Initializng calibration panel");
}
} // namespace industrial_calibration_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(industrial_calibration_gui::CalibrationPanel, rviz::Panel)
