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

template<typename T> inline void extractCameraIntrinsics(const T intrinsics[9], 
  T &fx, T &fy, T &cx, T &cy, T &k1, T &k2, T &k3, T &p1, T &p2)
{
  fx  = intrinsics[0]; /** focal length x */
  fy  = intrinsics[1]; /** focal length y */
  cx  = intrinsics[2]; /** central point x */
  cy  = intrinsics[3]; /** central point y */
  k1  = intrinsics[4]; /** distortion k1  */
  k2  = intrinsics[5]; /** distortion k2  */
  k3  = intrinsics[6]; /** distortion k3  */
  p1  = intrinsics[7]; /** distortion p1  */
  p2  = intrinsics[8]; /** distortion p2  */
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

template<typename T> inline void cameraPointResidualWithDistortion(T point[3], 
  T &k1, T &k2, T &k3, T &p1, T &p2, T &fx, T &fy, T &cx, T &cy, T &ox, T &oy, 
  T residual[2])
{
  T xp1 = point[0];
  T yp1 = point[1];
  T zp1 = point[2];

  // Scale into the image plane by distance away from camera
  T xp;
  T yp;
  if (zp1 == T(0)) // Avoid dividing by zero.
  {
    xp = xp1;
    yp = yp1;
  }
  else
  {
    xp = xp1 / zp1;
    yp = yp1 / zp1;
  }

  // Temporary variables for distortion model.
  T xp2 = xp * xp;    // x^2
  T yp2 = yp * yp;    // y^2 
  T r2  = xp2 + yp2;  // r^2 radius squared 
  T r4  = r2 * r2;    // r^4 
  T r6  = r2 * r4;    // r^6 
  
  // Apply the distortion coefficients to refine pixel location
  T xpp = xp 
    + k1 * r2 * xp    // 2nd order term
    + k2 * r4 * xp    // 4th order term
    + k3 * r6 * xp    // 6th order term
    + p2 * (r2 + T(2.0) * xp2) // tangential
    + p1 * xp * yp * T(2.0); // other tangential term

  T ypp = yp 
    + k1 * r2 * yp    // 2nd order term
    + k2 * r4 * yp    // 4th order term
    + k3 * r6 * yp    // 6th order term
    + p1 * (r2 + T(2.0) * yp2) // tangential term
    + p2 * xp * yp * T(2.0); // other tangential term
  
  // Perform projection using focal length and camera center into image plane
  residual[0] = fx * xpp + cx - ox;
  residual[1] = fy * ypp + cy - oy;
}

struct CameraOnWristExtrinsicCF
{
    CameraOnWristExtrinsicCF(const double observed_x, 
    const double observed_y, const double focal_length_x, 
    const double focal_length_y, const double optical_center_x, 
    const double optical_center_y, Pose6D link_pose, 
    Point3D point) : observed_x_(observed_x), observed_y_(observed_y), 
    focal_length_x_(focal_length_x), focal_length_y_(focal_length_y), 
    optical_center_x_(optical_center_x), optical_center_y_(optical_center_y), 
    link_pose_(link_pose), point_(point)
  {
    link_pose_i_ = link_pose_.getInverse();
  }

  template<typename T> bool operator() (const T* const extrinsic_parameters, 
  const T* target_to_world, T* residual) const
  {
    // Extract camera angle axis and position
    const T* camera_angle_axis(&extrinsic_parameters[0]);
    const T* camera_position(&extrinsic_parameters[3]);

    // Extract target angle axis and position
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

    cameraPointResidual(camera_point, focal_length_x, focal_length_y, optical_center_x, 
      optical_center_y, observed_x, observed_y, residual);

    return true;
  }

  // Factory to hide the construction of the Cost Function object from
  // client code.
  static ceres::CostFunction *Create(const double observed_x, const double observed_y, 
    const double focal_length_x, const double focal_length_y, const double optical_center_x, 
    const double optical_center_y, Pose6D pose, Point3D point)
  {
    return (new ceres::AutoDiffCostFunction<CameraOnWristExtrinsicCF, 2, 6, 6>(new CameraOnWristExtrinsicCF(observed_x, observed_y, focal_length_x, focal_length_y, 
      optical_center_x, optical_center_y, pose, point)));
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

struct CameraOnWristIntrinsicCF
{
  CameraOnWristIntrinsicCF(const double observed_x, const double observed_y,
    Point3D tool_position, Point3D point) : observed_x_(observed_x), 
    observed_y_(observed_y), tool_position_(tool_position), point_(point) { }

  template<typename T> bool operator() (const T* const intrinsic_parameters,
  const T* const target_pose, T* residual) const
  {
    T focal_length_x;
    T focal_length_y;
    T optical_center_x;
    T optical_center_y;
    T distortion_k1;
    T distortion_k2;
    T distortion_k3;
    T distortion_p1;
    T distortion_p2;

    // Extract intrinsics
    extractCameraIntrinsics(intrinsic_parameters, focal_length_x, focal_length_y, 
      optical_center_x, optical_center_y, distortion_k1, distortion_k2, 
      distortion_k3, distortion_p1, distortion_p2);

    // Extract target angle axis and position
    const T* target_angle_axis(&target_pose[0]);
    const T* target_position(&target_pose[3]);

    // Transform point into camera frame
    T camera_point[3];
    transformPoint3D(target_angle_axis, target_position, point_.asVector(), 
      camera_point);

    // Transform to camera location 
    camera_point[0] = camera_point[0] + T(tool_position_.x);
    camera_point[1] = camera_point[1] + T(tool_position_.y);
    camera_point[2] = camera_point[2] + T(tool_position_.z);

    // Compute projected point into image plane and residual
    T observed_x = T(observed_x_);
    T observed_y = T(observed_y_);

    cameraPointResidualWithDistortion(camera_point, distortion_k1, distortion_k2, 
      distortion_k3, distortion_p1, distortion_p2, focal_length_x, focal_length_y,
      optical_center_x, optical_center_y, observed_x, observed_y, residual);

    return true;
  }

  // Factory to hide the construction of the Cost Function object from
  // client code.
  static ceres::CostFunction *Create(const double observed_x, const double observed_y,
    Point3D tool_position, Point3D point)
  {
    return (new ceres::AutoDiffCostFunction<CameraOnWristIntrinsicCF, 2, 9, 6>(new 
      CameraOnWristIntrinsicCF(observed_x, observed_y, tool_position, point)));
  }

  double observed_x_; // Observed x location of object in the image
  double observed_y_; // Observed y location of object in the image
  Point3D tool_position_;
  Point3D point_;
};

struct ResearchIntrinsicCF
{
  ResearchIntrinsicCF(const double observed_x, const double observed_y,
    Pose6D target_pose, Point3D point) : observed_x_(observed_x), 
    observed_y_(observed_y), target_pose_(target_pose), point_(point) { }

  template<typename T> bool operator() (const T* const intrinsic_parameters,
  T* residual) const
  {
    T focal_length_x;
    T focal_length_y;
    T optical_center_x;
    T optical_center_y;
    T distortion_k1;
    T distortion_k2;
    T distortion_k3;
    T distortion_p1;
    T distortion_p2;

    // Extract intrinsics
    extractCameraIntrinsics(intrinsic_parameters, focal_length_x, focal_length_y, 
      optical_center_x, optical_center_y, distortion_k1, distortion_k2, 
      distortion_k3, distortion_p1, distortion_p2);

    // Extract target angle axis and position
    T target_pose[6];
    target_pose[0] = static_cast<T>(target_pose_.ax);
    target_pose[1] = static_cast<T>(target_pose_.ay);
    target_pose[2] = static_cast<T>(target_pose_.az);
    target_pose[3] = static_cast<T>(target_pose_.x);
    target_pose[4] = static_cast<T>(target_pose_.y);
    target_pose[5] = static_cast<T>(target_pose_.z);

    const T* target_angle_axis(&target_pose[0]);
    const T* target_position(&target_pose[3]);

    // Transform point into camera frame
    T camera_point[3];
    transformPoint3D(target_angle_axis, target_position, point_.asVector(), 
      camera_point);

    // Compute projected point into image plane and residual
    T observed_x = T(observed_x_);
    T observed_y = T(observed_y_);

    cameraPointResidualWithDistortion(camera_point, distortion_k1, distortion_k2, 
      distortion_k3, distortion_p1, distortion_p2, focal_length_x, focal_length_y,
      optical_center_x, optical_center_y, observed_x, observed_y, residual);

    return true;
  }

  // Factory to hide the construction of the Cost Function object from
  // client code.
  static ceres::CostFunction *Create(const double observed_x, const double observed_y,
    Pose6D target_pose, Point3D point)
  {
    return (new ceres::AutoDiffCostFunction<ResearchIntrinsicCF, 2, 9>(new 
      ResearchIntrinsicCF(observed_x, observed_y, target_pose, point)));
  }

  double observed_x_; // Observed x location of object in the image
  double observed_y_; // Observed y location of object in the image
  Pose6D target_pose_;
  Point3D point_;
};

#if 0
struct ResearchIntrinsicCF2
{
  ResearchIntrinsicCF2(const double observed_x, const double observed_y,
    Point3D point) : observed_x_(observed_x), 
    observed_y_(observed_y), point_(point) { }

  template<typename T> bool operator() (const T* const intrinsic_parameters,
  const T* const target_pose, T* residual) const
  {
    T focal_length_x;
    T focal_length_y;
    T optical_center_x;
    T optical_center_y;
    T distortion_k1;
    T distortion_k2;
    T distortion_k3;
    T distortion_p1;
    T distortion_p2;

    // Extract intrinsics
    extractCameraIntrinsics(intrinsic_parameters, focal_length_x, focal_length_y, 
      optical_center_x, optical_center_y, distortion_k1, distortion_k2, 
      distortion_k3, distortion_p1, distortion_p2);

    const T* target_angle_axis(&target_pose[0]);
    const T* target_position(&target_pose[3]);

    // Transform point into camera frame
    T camera_point[3];
    transformPoint3D(target_angle_axis, target_position, point_.asVector(), 
      camera_point);

    // Compute projected point into image plane and residual
    T observed_x = T(observed_x_);
    T observed_y = T(observed_y_);

    cameraPointResidualWithDistortion(camera_point, distortion_k1, distortion_k2, 
      distortion_k3, distortion_p1, distortion_p2, focal_length_x, focal_length_y,
      optical_center_x, optical_center_y, observed_x, observed_y, residual);

    return true;
  }

  // Factory to hide the construction of the Cost Function object from
  // client code.
  static ceres::CostFunction *Create(const double observed_x, const double observed_y,
    Point3D point)
  {
    return (new ceres::AutoDiffCostFunction<ResearchIntrinsicCF2, 2, 9, 6>(new 
      ResearchIntrinsicCF2(observed_x, observed_y, point)));
  }

  double observed_x_; // Observed x location of object in the image
  double observed_y_; // Observed y location of object in the image
  Point3D point_;
};
#endif

// DELETE THIS LATER
#if 1
struct CameraOnWristExtrinsicIntrinsicCF
{
  CameraOnWristExtrinsicIntrinsicCF(const double observed_x, 
    const double observed_y, Pose6D link_pose, Point3D point) : observed_x_(observed_x), 
    observed_y_(observed_y), link_pose_(link_pose), point_(point) 
  { 
    link_pose_i_ = link_pose_.getInverse();
  }

  template<typename T> bool operator() (const T* const extrinsic_parameters, 
    const T* const intrinsic_parameters, const T* const target_to_world, T* residual) const
  {
    T focal_length_x;
    T focal_length_y;
    T optical_center_x;
    T optical_center_y;
    T distortion_k1;
    T distortion_k2;
    T distortion_k3;
    T distortion_p1;
    T distortion_p2;

    // Extract intrinsics
    extractCameraIntrinsics(intrinsic_parameters, focal_length_x, focal_length_y, 
      optical_center_x, optical_center_y, distortion_k1, distortion_k2, 
      distortion_k3, distortion_p1, distortion_p2);

    // Extract camera angle axis and position
    const T* camera_angle_axis(&extrinsic_parameters[0]);
    const T* camera_position(&extrinsic_parameters[3]);

    // Extract target angle axis and position
    const T* target_angle_axis(&target_to_world[0]); 
    const T* target_position(&target_to_world[3]); 

    T world_point[3]; // Point in world coordinates
    T link_point[3]; // Point in link coordinates
    T camera_point[3]; // Point in camera coordinates
    
    // Transform point into camera coordinates
    transformPoint3D(target_angle_axis, target_position, point_.asVector(), 
      world_point);
    poseTransformPoint(link_pose_i_, world_point, link_point);
    transformPoint(camera_angle_axis, camera_position, link_point, camera_point);

    // Compute projected point into image plane and compute residual.
    T observed_x = T(observed_x_);
    T observed_y = T(observed_y_);

    cameraPointResidualWithDistortion(camera_point, distortion_k1, distortion_k2, 
      distortion_k3, distortion_p1, distortion_p2, focal_length_x, focal_length_y,
      optical_center_x, optical_center_y, observed_x, observed_y, residual);

    return true;
  }

  // Factory to hide the construction of the Cost Function object from
  // client code.
  static ceres::CostFunction *Create(const double observed_x, 
    const double observed_y, Pose6D link_pose, Point3D point)
  {
    return (new ceres::AutoDiffCostFunction<CameraOnWristExtrinsicIntrinsicCF, 2, 6, 9, 6>(new CameraOnWristExtrinsicIntrinsicCF(observed_x, observed_y, link_pose, point)));
  }  

  double observed_x_;
  double observed_y_;
  Pose6D link_pose_;
  Pose6D link_pose_i_;
  Point3D point_;
};
#endif

struct DistortedTargetFinder
{
  DistortedTargetFinder(const double observed_x, const double observed_y,
    const double focal_length_x, const double focal_length_y,
    const double optical_center_x, const double optical_center_y,
    const double distortion_k1, const double distortion_k2,
    const double distortion_k3, const double distortion_p1,
    const double distortion_p2, Point3D point) : observed_x_(observed_x), 
    observed_y_(observed_y), focal_length_x_(focal_length_x),
    focal_length_y_(focal_length_y), optical_center_x_(optical_center_x),
    optical_center_y_(optical_center_y), distortion_k1_(distortion_k1),
    distortion_k2_(distortion_k2), distortion_k3_(distortion_k3),
    distortion_p1_(distortion_p1), distortion_p2_(distortion_p2),
    point_(point) { }

  template<typename T> bool operator() (const T* const extrinsic_parameters,
    T* residual) const
  {
    // Extract camera angle axis and position
    const T* camera_angle_axis(&extrinsic_parameters[0]);
    const T* camera_position(&extrinsic_parameters[3]);

    T camera_point[3]; // Point in camera coordinates

    // Transform point into camera coordinates
    transformPoint3D(camera_angle_axis, camera_position, point_.asVector(), camera_point);

    // Project the point into image plane and compute the residual
    T observed_x = T(observed_x_);
    T observed_y = T(observed_y_);
    T focal_length_x = T(focal_length_x_);
    T focal_length_y = T(focal_length_y_);
    T optical_center_x = T(optical_center_x_);
    T optical_center_y = T(optical_center_y_);
    T distortion_k1 = T(distortion_k1_);
    T distortion_k2 = T(distortion_k2_);
    T distortion_k3 = T(distortion_k3_);
    T distortion_p1 = T(distortion_p1_);
    T distortion_p2 = T(distortion_p2_);

    cameraPointResidualWithDistortion(camera_point, distortion_k1, distortion_k2, 
      distortion_k3, distortion_p1, distortion_p2, focal_length_x, focal_length_y,
      optical_center_x, optical_center_y, observed_x, observed_y, residual);

    return true;
  }

  // Factory to hide the construction of the Cost Function object from
  // client code.
  static ceres::CostFunction *Create(const double observed_x, const double observed_y,
    const double focal_length_x, const double focal_length_y, const double optical_center_x,
    const double optical_center_y, const double distortion_k1, const double distortion_k2,
    const double distortion_k3, const double distortion_p1, const double distortion_p2,
    Point3D point)
  {
    return (new ceres::AutoDiffCostFunction<DistortedTargetFinder, 2, 6>(new 
      DistortedTargetFinder(observed_x, observed_y, focal_length_x, focal_length_y,
        optical_center_x, optical_center_y, distortion_k1, distortion_k2, distortion_k3,
        distortion_p1, distortion_p2, point)));
  }

  double observed_x_;
  double observed_y_;
  double focal_length_x_;
  double focal_length_y_;
  double optical_center_x_;
  double optical_center_y_;
  double distortion_k1_;
  double distortion_k2_;
  double distortion_k3_;
  double distortion_p1_;
  double distortion_p2_;
  Point3D point_;
};
} // namespace industrial_calibration_libs
#endif