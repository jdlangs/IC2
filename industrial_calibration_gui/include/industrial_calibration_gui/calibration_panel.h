#ifndef CALIBRATION_PANEL_H
#define CALIBRATION_PANEL_H

#include <rviz/panel.h>

namespace industrial_calibration_gui
{
// Forward declare calibration widget
class CalibrationWidget;

class CalibrationPanel : public rviz::Panel
{
  Q_OBJECT
public:
  CalibrationPanel(QWidget* parent = 0);

  virtual ~CalibrationPanel();

  virtual void onInitialize();

protected:
  CalibrationWidget* widget_;
};
} // namespace industrial_calibration_gui
#endif // CALIBRATION_PANEL_H
