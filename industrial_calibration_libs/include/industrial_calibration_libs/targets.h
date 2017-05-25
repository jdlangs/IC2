#ifndef TARGETS_H
#define TARGETS_H

#include <vector>

namespace industrial_calibration_libs
{

struct Point3D
{
  double x;
  double y;
  double z;
};

enum target_types
{
  ChessBoard = 0;
  CircleGrid = 1;
  ModifiedCircleGrid = 2;
};

class Target
{
public:
  Target(std::string target_name, std::string target_frame, std::string transform_interface,
             double angle_axis_ax = 0.0, double angle_axis_ay = 0.0, double angle_axis_az = 0.0,
             double position_x = 0.0, double position_y = 0.0, double position_z = 0.0);

protected:
  std::string target_name_;
  std::string target_frame_;
  std::string transform_interface_; 
  double angle_axis_ax_;
  double angle_axis_ay_;
  double angle_axis_az_;
  double position_x_;
  double position_y_;
  double position_z_;
  std::size_t num_points;

  std::vector<Point3D> points_;
};

class CheckerBoardTarget : public Target
{
public:
  CheckerBoardTarget(std::string target_name, std::string target_frame, 
                                  std::string transform_interface, double angle_axis_ax = 0.0, 
                                  double angle_axis_ay = 0.0, double angle_axis_az = 0.0,
                                  double position_x = 0.0, double position_y = 0.0, 
                                  double position_z = 0.0, std::size_t num_rows, std::size_t num_cols);

protected:
  std::size_t num_rows_;
  std::size_t num_cols_;
};

class CircleGridTarget : public Target
{
public:
  CircleGridTarget(std::string target_name, std::string target_frame, 
                            std::string transform_interface, double angle_axis_ax = 0.0, 
                            double angle_axis_ay = 0.0, double angle_axis_az = 0.0,
                            double position_x = 0.0, double position_y = 0.0, 
                            double position_z = 0.0, std::size_t num_rows, std::size_t num_cols,
                            bool is_symmetric, double circle_diameter);
protected:
  std::size_t num_rows_;
  std::size_t num_cols_;
  bool is_symmetric_;
  double circle_diameter_;
};
  
} // namespace industrial_calibration_libs

#endif // TARGETS_H

