#include <industrial_calibration_libs/targets.h>

namespace industrial_calibration_libs
{

Target::Target(std::string target_name, std::string target_frame, std::string transform_interface, 
  double angle_axis_ax = 0.0, double angle_axis_ay = 0.0, double angle_axis_az = 0.0,
  double position_x = 0.0, double position_y = 0.0, double position_z = 0.0, 
  std::size_t num_points) : target_name_(target_name), target_frame_(target_frame), 
  transform_interface_(transform_interface), angle_axis_ax_(angle_axis_ax), 
  angle_axis_ay_(angle_axis_ay), angle_axis_az_(angle_axis_az), position_x_(position_x),
  position_y_(position_y), position_z_(position_z)
{

}

CheckerBoardTarget::CheckerBoardTarget(std::string target_name, std::string target_frame, 
  std::string transform_interface, double angle_axis_ax = 0.0, double angle_axis_ay = 0.0, 
  double angle_axis_az = 0.0, double position_x = 0.0, double position_y = 0.0, 
  double position_z = 0.0, std::size_t num_rows, std::size_t num_cols) : 
  Target(target_name, target_frame, transform_interface, angle_axis_ax, angle_axis_ay,
  angle_axis_az, position_x, position_y, position_z), num_rows_(num_rows), num_cols_(num_cols)
{

}                                                                 

CircleGridTarget::CircleGridTarget(std::string target_name, std::string target_frame, 
  std::string transform_interface, double angle_axis_ax = 0.0, double angle_axis_ay = 0.0, 
  double angle_axis_az = 0.0, double position_x = 0.0, double position_y = 0.0, 
  double position_z = 0.0, std::size_t num_rows, std::size_t num_cols, bool is_symmetric, 
  double circle_diameter) : Target(target_name, target_frame, transform_interface, 
  angle_axis_ax, angle_axis_ay, angle_axis_az, position_x, position_y, position_z), 
  num_rows_(num_rows), num_cols_(num_cols), is_symmetric_(is_symmetric), 
  circle_diameter_(circle_diameter)
{

}                                                    

} // namespace industrial_calibration_libs