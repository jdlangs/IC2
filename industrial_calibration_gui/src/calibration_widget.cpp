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
}

CalibrationWidget::~CalibrationWidget() { }
} // namespace industrial_calibration_gui