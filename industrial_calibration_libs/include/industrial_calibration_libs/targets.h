#ifndef TARGETS_H
#define TARGETS_H

#include <vector>
#include <string>

namespace industrial_calibration_libs
{

// enum target_types
// {
//   ChessBoard = 0;
//   CircleGrid = 1;
//   ModifiedCircleGrid = 2;
// };

int number = 5;

struct Point3D
{
  double x;
  double y;
  double z;
};

struct TargetDefinition
{
  std::string target_name;
  std::string target_frame;
  std::string transform_interface;
  
  double angle_axis_ax;
  double angle_axis_ay;
  double angle_axis_az;
  
  double position_x;
  double position_y;
  double position_z;

  std::size_t traget_type;

  std::size_t num_points;
  std::size_t num_rows;
  std::size_t num_cols;

  std::vector<Point3D> points;
  double circle_diameter;
};

class Target
{
public:

protected:
  TargetDefinition target_definition_;
  std::vector<Point3D> points_;
};

class CheckerBoardTarget : public Target
{
public:

protected:
};

class CircleGridTarget : public Target
{
public:
  CircleGridTarget(const std::string yaml_path);


protected:
  bool is_symmetric_;
};

} // namespace industrial_calibration_libs

#endif // TARGETS_H

