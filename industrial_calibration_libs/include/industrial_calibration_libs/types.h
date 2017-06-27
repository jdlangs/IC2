#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <cassert>

namespace industrial_calibration_libs
{
struct Point3D
{
  Point3D(void);

  Point3D(double x_in, double y_in, double z_in);
  
  Point3D(const std::vector<double> &points);

  void setPoints(double x_in, double y_in, double z_in);

  bool setPoints(const std::vector<double> &points);

  std::vector<double> asVector(void);

  bool operator==(const Point3D &p2)
  {
    if (this->x == p2.x && this->y == p2.y && this->z == p2.z) {return true;}
    else {return false;}
  }

  double x;
  double y;
  double z;
};

} // namespace industrial_calibration_libs
#endif