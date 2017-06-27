#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H

#include <industrial_calibration_libs/types.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/types.h>

namespace industrial_calibration_libs
{
template<typename T> inline void transformPoint3D(const T angle_axis[3], 
  const T tx[3], const std::vector<double> &point, T t_point[3])
{
  T point_[3];

  point_[0] = T(point[0]);
  point_[1] = T(point[1]);
  point_[2] = T(point[2]);

  ceres::AngleAxisRotatePoint(angle_axis, point_, t_point);

  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}

template<typename T> inline void poseTransformPoint(const Pose6D &pose, 
  const T point[3], T t_point[3])
{
  T angle_axis[3];

  angle_axis[0]  = T(pose.ax);
  angle_axis[1]  = T(pose.ay);
  angle_axis[2]  = T(pose.az);

  ceres::AngleAxisRotatePoint(angle_axis, point, t_point);

  t_point[0] = t_point[0] + T(pose.x);
  t_point[1] = t_point[1] + T(pose.y);
  t_point[2] = t_point[2] + T(pose.z);
}

template<typename T> inline void transformPoint(const T angle_axis[3], 
  const T tx[3], const T point[3], T t_point[3])
{
  ceres::AngleAxisRotatePoint(angle_axis, point, t_point);

  t_point[0] = t_point[0] + tx[0];
  t_point[1] = t_point[1] + tx[1];
  t_point[2] = t_point[2] + tx[2];
}

template<typename T> inline void cameraPointResidual(T point[3], T &fx, T &fy, 
  T &cx, T &cy, T &ox, T &oy, T residual[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  // Scale into the image plane by distance away from camera
  T xp, yp;

  if (zp1 == T(0)) // Avoid divide by zero
  { 
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }
  
  // Perform projection using focal length and camera optical center into image plane
  residual[0] = fx * xp + cx - ox;
  residual[1] = fy * yp + cy - oy;
}

struct CameraOnWristStaticTargetExtrinsic
{
  CameraOnWristStaticTargetExtrinsic(const double observed_x, 
    const double observed_y, const double focal_length_x, const double focal_length_y, const double optical_center_x, const double optical_center_y, Pose6D link_pose, Point3D point) : observed_x_(observed_x), observed_y_(observed_y), focal_length_x_(focal_length_x), focal_length_y_(focal_length_y), optical_center_x_(optical_center_x), optical_center_y_(optical_center_y), link_pose_(link_pose), point_(point)
  {
    link_pose_i_ = link_pose_.getInverse();
  }

  template<typename T> bool operator() (const T* const extrinsic_parameters, const T* target_to_world, T* residual) const
  {
    const T* camera_angle_axis(&extrinsic_parameters[0]);
    const T* camera_position(&extrinsic_parameters[3]);

    const T* target_angle_axis(&target_to_world[0]);
    const T* target_position(&target_to_world[3]);

    T world_point[3]; // Point in world coordinates
    T link_point[3]; // Point in link coordinates
    T camera_point[3]; // Point in camera coordinates

    // Transform points into camera coordinates
    transformPoint3D(target_angle_axis, target_position, point_.asVector(), world_point);
    poseTransformPoint(link_pose_i_, world_point, link_point);
    transformPoint(camera_angle_axis, camera_position, link_point, camera_point);

    // Compute projected point into image plane and compute residual.
    T focal_length_x = T(focal_length_x_);
    T focal_length_y = T(focal_length_y_);
    T optical_center_x = T(optical_center_x_);
    T optical_center_y = T(optical_center_y_);
    T observed_x = T(observed_x_);
    T observed_y = T(observed_y_);

    cameraPointResidual(camera_point, focal_length_x, focal_length_y, optical_center_x, optical_center_y, observed_x, observed_y, residual);

    return true;
  }

  // Factory to hide the construction of the Cost Function object from
  // client code.
  static ceres::CostFunction *Create(const double observed_x, const double observed_y, const double focal_length_x, const double focal_length_y, const double optical_center_x, const double optical_center_y, Pose6D pose, Point3D point)
  {
    return (new ceres::AutoDiffCostFunction<CameraOnWristStaticTargetExtrinsic, 2, 6, 6>(new CameraOnWristStaticTargetExtrinsic(observed_x, observed_y, focal_length_x, focal_length_y, optical_center_x, optical_center_y, 
      pose, point)));
  }

  double observed_x_; // Observed x location of object in the image
  double observed_y_; // Observed y location of object in the image
  double focal_length_x_; // Known focal length of the camera in x
  double focal_length_y_; // Known focal length of the camera in y
  double optical_center_x_; // Known optical center of camera in x
  double optical_center_y_; // Known optical center of camera in y
  Pose6D link_pose_; 
  Pose6D link_pose_i_;
  Point3D point_;
};

} // namespace industrial_calibration_libs

#endif


