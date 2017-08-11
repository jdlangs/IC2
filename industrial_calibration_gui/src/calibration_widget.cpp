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

  // ui_->tabWidget->setTabEnabled(1, false);
  // ui_->tabWidget->setTabEnabled(2, false);

  connect(ui_->pushButton, SIGNAL(clicked()), this, SLOT(startCalibrationButton()));
  connect(ui_->comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxDisplayText()));

  this->updateInstructionText(ui_->comboBox->currentIndex());
}

CalibrationWidget::~CalibrationWidget() { }

void CalibrationWidget::startCalibrationButton(void)
{
  ROS_INFO_STREAM("BUTTON PUSHED");
  ui_->stackedWidget->setCurrentIndex(1);
  // ui_->tabWidget->setTabEnabled(1, true);
  // ui_->tabWidget->setTabEnabled(0, false);
}

void CalibrationWidget::comboBoxDisplayText(void)
{
  int current_index = ui_->comboBox->currentIndex();
  ROS_INFO_STREAM("Combo Box State Changed " << current_index);
  this->updateInstructionText(current_index);
}

void CalibrationWidget::updateInstructionText(int current_index)
{
  switch (current_index)
  {
    case 0:
      ui_->textBrowser->setText("Welcome to the industrial_calibration_gui.");
      break;

    case 1:
      ui_->textBrowser->setText("Static Target Moving Camera on Wrist (Extrinsic)");
      break;

    case 2:
      ui_->textBrowser->setText("Static Target Moving Camera on Wrist (Extrinsic + Intrinsic)");
      break;

    default:
      ui_->textBrowser->setText("Welcome to the industrial_calibration_gui.");
      break;
  }
}

} // namespace industrial_calibration_gui