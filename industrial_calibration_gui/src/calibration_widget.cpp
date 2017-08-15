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

  this->instructions_checkbox_state_ = false;
  this->updateCalibrationTypeText(ui_->calibration_type_combo_box->currentIndex());

  // Start page
  connect(ui_->instructions_checkbox, SIGNAL(stateChanged(int)),
    this, SLOT(instructionsCheckbox()));
  connect(ui_->start_calibration_button, SIGNAL(clicked()), 
    this, SLOT(startCalibrationButton()));
  connect(ui_->calibration_type_combo_box, SIGNAL(currentIndexChanged(int)), 
    this, SLOT(selectCalibrationTypeComboBox()));

}

CalibrationWidget::~CalibrationWidget() { }


void CalibrationWidget::instructionsCheckbox(void)
{
  if (ui_->instructions_checkbox->isChecked())
  {
    this->instructions_checkbox_state_ = true;
  }
  else
  {
    this->instructions_checkbox_state_ = false;
  }
}

void CalibrationWidget::startCalibrationButton(void)
{
  if (this->instructions_checkbox_state_)
  {
    ui_->stackedWidget->setCurrentIndex(1);
  }
}

void CalibrationWidget::selectCalibrationTypeComboBox(void)
{
  int current_index = ui_->calibration_type_combo_box->currentIndex();
  ROS_INFO_STREAM("Combo Box State Changed " << current_index);
  this->updateCalibrationTypeText(current_index);
}

void CalibrationWidget::updateCalibrationTypeText(int current_index)
{
  switch (current_index)
  {
    case 0:
      ui_->calibration_type_text_browser->setText("Welcome to the industrial_calibration_gui.");
      break;

    case 1:
      ui_->calibration_type_text_browser->setText("Static Target Moving Camera on Wrist (Extrinsic)");
      break;

    case 2:
      ui_->calibration_type_text_browser->setText("Static Target Moving Camera on Wrist (Extrinsic + Intrinsic) [EXPERIMENTAL]");
      break;

    case 3:
      ui_->calibration_type_text_browser->setText("Static Camera Moving Target on Wrist (Extrinsic)");

    case 4:
      ui_->calibration_type_text_browser->setText("Static Camera Moving Target on Wrist (Extrinsic + Intrinsic) [EXPERIMENTAL]");

    default:
      ui_->calibration_type_text_browser->setText("Welcome to the industrial_calibration_gui.");
      break;
  }
}

} // namespace industrial_calibration_gui