#include <ros/console.h>
#include <industrial_calibration_gui/calibration_widget.h>
#include <ui_calibration_widget.h>

namespace industrial_calibration_gui
{
CalibrationWidget::CalibrationWidget(QWidget* parent) : QWidget(parent)
{
  // UI setup
  ui_ = new Ui::CalibrationWidget;
  ui_->setupUi(this);

  ui_->tabWidget->setTabEnabled(1, false);
  ui_->tabWidget->setTabEnabled(2, false);

  connect(ui_->pushButton, SIGNAL(clicked()), this, SLOT(startCalibrationButton()));
}

void CalibrationWidget::startCalibrationButton(void)
{
  ROS_INFO_STREAM("BUTTON PUSHED");
  ui_->tabWidget->setTabEnabled(1, true);
  ui_->tabWidget->setCurrentIndex(1);
  ui_->tabWidget->setTabEnabled(0, false);
}

CalibrationWidget::~CalibrationWidget() { }
} // namespace industrial_calibration_gui