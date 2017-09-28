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

/*
  fx = Focal length x
  fy = Focal length y
  cx = Optical center x
  cy = Optical center y
*/  
struct IntrinsicsPartial
{
  IntrinsicsPartial(double fx, double fy, double cx, double cy) :
    data{fx, fy, cx, cy} { }

  const double &fx(void) const {return data[0];}
  const double &fy(void) const {return data[1];}
  const double &cx(void) const {return data[2];}
  const double &cy(void) const {return data[3];}

  double data[4];
};

/*
  fx = Focal length x
  fy = Focal length y
  cx = Optical center x
  cy = Optical center y
  k1 = Distortion k1 (radial)
  k2 = Distortion k2 (radial)
  k3 = Distortion k3 (radial)
  p1 = Distortion p1 (tangential)
  p2 = Distortion p2 (tangential)
*/
struct IntrinsicsFull
{
  IntrinsicsFull(double fx, double fy, double cx, double cy,
    double k1, double k2, double k3, double p1, double p2) :
  data{fx, fy, cx, cy, k1, k2, k3, p1, p2} { }

  const double &fx(void) const {return data[0];}
  const double &fy(void) const {return data[1];}
  const double &cx(void) const {return data[2];}
  const double &cy(void) const {return data[3];}
  const double &k1(void) const {return data[4];}
  const double &k2(void) const {return data[5];}
  const double &k3(void) const {return data[6];}
  const double &p1(void) const {return data[7];}
  const double &p2(void) const {return data[8];}

  double data[9];
};

/*
  ax = Angle Axis x
  ay = Angle Axis y
  az = Angle Axis z
  y = Translation y
  x = Translation x
  z = Translation z
*/
struct Extrinsics
{
  Extrinsics(double ax, double ay, double az, double x,
    double y, double z) : data{ax, ay, az, x, y, z} { }

  Extrinsics(Pose6D pose) : data{pose.ax, pose.ay, pose.az, 
    pose.x, pose.y, pose.z} { }

  Pose6D asPose6D(void) {return Pose6D(data[3], data[4], data[5], 
    data[0], data[1], data[2]);}

  const double &ax(void) const {return data[0];}
  const double &ay(void) const {return data[1];}
  const double &az(void) const {return data[2];}
  const double &x(void) const {return data[3];}
  const double &y(void) const {return data[4];}
  const double &z(void) const {return data[5];}

  double data[6];
};

} // namespace industrial_calibration_libs
#endif