#ifndef CALIBRATION_WIDGET_H
#define CALIBRATION_WIDGET_H

#include <QWidget>
#include <QFileDialog>

#include <ros/ros.h>
#include <ros/console.h>
#include <ui_calibration_widget.h>
#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <industrial_calibration_gui/calibration_data_collector.h>

namespace Ui
{
  class CalibrationWidget;
}

namespace industrial_calibration_gui
{
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

  // Calibration variables
  industrial_calibration_libs::Target target_;
};
} // namespace industrial_calibration_gui

#endif //CALIBRATION_WIDGET_H
