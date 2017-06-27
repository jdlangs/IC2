#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include <cassert>
#include <limits>

#include <ceres/rotation.h>
#include <Eigen/Dense>

namespace industrial_calibration_libs
{
struct Point3D
{
  Point3D(void);

  Point3D(double x_in, double y_in, double z_in);
  
  Point3D(const std::vector<double> &points);

  void setPoints(double x_in, double y_in, double z_in);

  bool setPoints(const std::vector<double> &points);

  std::vector<double> asVector(void) const;

  bool operator==(const Point3D &p2)
  {
    if (this->x == p2.x && this->y == p2.y && this->z == p2.z) {return true;}
    else {return false;}
  }

  double x;
  double y;
  double z;
};

// TODO(gChiou): Refactor this and remove un-needed code.
class Pose6D
{
public:
  Pose6D(double tx, double ty, double tz, double aax, double aay, double aaz);

  Pose6D();

  // void setBasis( tf::Matrix3x3 & m);
  void setBasis(Eigen::Matrix3d &m);

  // void setOrigin(tf::Vector3 & v);
  void setOrigin(Eigen::Vector3d &v);

  void setOrigin(double tx, double ty, double tz);

  void setEulerZYX(double ez, double ey, double ex);

  void setQuaternion(double qx, double qy, double qz, double qw);

  void setAngleAxis(double aax, double aay, double aaz);

  // tf::Matrix3x3 getBasis() const;
  Eigen::Matrix3d getBasis(void) const;

  void getEulerZYX(double &ez, double &ey, double &ex) const;

  // tf::Vector3 getOrigin() const;
  Eigen::Vector3d getOrigin(void) const;

  void getQuaternion(double &qx, double &qy, double &qz, double &qw) const;

  Pose6D getInverse(void) const;

  // void show(std::string message);

  Pose6D operator *(Pose6D pose2) const;

  // union
  // {
  //   struct
  //   {
  //     double ax; 
  //     double ay; 
  //     double az; 
  //     double x;  
  //     double y;  
  //     double z;  
  //   };

  //   struct
  //   {
  //     double pb_aa[3]; 
  //     double pb_loc[3];
  //   };

  //   struct
  //   {
  //     double pb_pose[6]; 
  //   };
  // }; 

  double ax; 
  double ay; 
  double az; 
  double x;  
  double y;  
  double z;   
};

} // namespace industrial_calibration_libs
#endif