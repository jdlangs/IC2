#include <industrial_calibration_libs/calibration_types/camera_on_wrist_intrinsic.h>

namespace industrial_calibration_libs
{
CameraOnWristIntrinsic::CameraOnWristIntrinsic(const ObservationData &observation_data, 
  const Target &target, const CameraOnWristIntrinsicParams &params) : 
  CalibrationJob(observation_data, target)
{
  link_poses_ = params.base_to_tool;
  std::memcpy(result_.intrinsics, params.intrinsics.data, 
    sizeof(result_.intrinsics));
  std::memcpy(result_.target_to_camera, params.target_to_camera.data,
    sizeof(result_.target_to_camera));
}

bool CameraOnWristIntrinsic::runCalibration(void)
{
  if (link_poses_.size() == 0)
  {
    std::cerr << "base to tool Poses are Empty!" << '\n';
    return false;
  }
  
  if (!checkObservations()) {return false;}

  return this->runCalculateFirstPose();
}

bool CameraOnWristIntrinsic::runCalculateFirstPose(void)
{
  Pose6D target_to_position_1; // Position of first calibration image
  Pose6D position_1_to_target; // Inverse of target_to_position_1

  // Calculate target_to_position_1 and get inverse
  this->findDistortedTarget(observation_data_[0], target_to_position_1,
    result_.intrinsics, result_.target_to_camera);
  position_1_to_target = target_to_position_1.getInverse();

  // Use position_1_to_target as seed
  double target_seed[6];
  target_seed[0] = target_to_position_1.ax;
  target_seed[1] = target_to_position_1.ay;
  target_seed[2] = target_to_position_1.az;
  target_seed[3] = target_to_position_1.x;
  target_seed[4] = target_to_position_1.y;
  target_seed[5] = target_to_position_1.z;

  std::memcpy(result_.target_to_camera, target_seed, 
    sizeof(result_.target_to_camera));

  // Iterate through every observation image
  for (std::size_t i = 0; i < num_images_; i++)
  {
    // Iterate through every observation in each observation image
    for (std::size_t j = 0; j < observations_per_image_; j++)
    {
      double x_position = link_poses_[i].x - link_poses_[0].x;
      double y_position = link_poses_[i].y - link_poses_[0].y;
      double z_position = link_poses_[i].z - link_poses_[0].z;
      Point3D position(x_position, y_position, z_position);

      Point3D point = target_.getDefinition().points[j];

      double observed_x = observation_data_[i][j].x;
      double observed_y = observation_data_[i][j].y;

      ceres::CostFunction *cost_function =
        CameraOnWristIntrinsicCF::Create(observed_x, observed_y,
          position, point);

      problem_.AddResidualBlock(cost_function, NULL, result_.intrinsics,
        result_.target_to_camera);
    }
  }

  // Solve
  options_.linear_solver_type = ceres::DENSE_SCHUR;
  options_.minimizer_progress_to_stdout = output_results_; 
  options_.max_num_iterations = 9001;

  ceres::Solve(options_, &problem_, &summary_);

  if (output_results_)
  {
    std::cout << summary_.FullReport() << '\n';  
  }

  if (summary_.termination_type != ceres::NO_CONVERGENCE)
  {
    initial_cost_ = summary_.initial_cost / total_observations_;
    final_cost_ = summary_.final_cost / total_observations_;
    return true;    
  }

  return false;
}

bool CameraOnWristIntrinsic::findDistortedTarget(const ObservationPoints &observation_points,
  Pose6D &result, double intrinsics[9], double guess_pose[6])
{ 
  double focal_x = intrinsics[0];
  double focal_y = intrinsics[1];
  double optical_center_x = intrinsics[2];
  double optical_center_y = intrinsics[3];
  double distortion_k1 = intrinsics[4];
  double distortion_k2 = intrinsics[5];
  double distortion_k3 = intrinsics[6];
  double distortion_p1 = intrinsics[7];
  double distortion_p2 = intrinsics[8];

  // Set initial conditions
  double result_pose[6];

  // Copy the initial guess into solver input.
  std::memcpy(result_pose, guess_pose, sizeof(result_pose));

  ceres::Problem problem;
  for (std::size_t i = 0; i < observations_per_image_; i++)
  {
    Point3D point = target_.getDefinition().points[i];

    double observed_x = observation_points[i].x;
    double observed_y = observation_points[i].y;

    ceres::CostFunction *cost_function = 
      DistortedTargetFinder::Create(observed_x, observed_y, focal_x, focal_y,
        optical_center_x, optical_center_y, distortion_k1, distortion_k2,
        distortion_k3, distortion_p1, distortion_p2, point);

    problem.AddResidualBlock(cost_function, NULL, result_pose);      
  }

  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.max_num_iterations = 9001;  

  if (output_results_)
  {
    options.minimizer_progress_to_stdout = true;
  }

  ceres::Solve(options, &problem, &summary);

  if (output_results_)
  {
    // Leaving this commented out for now, don't want to spam the terminal.
    std::cout << summary.FullReport() << '\n';
  }

  if (summary.termination_type != ceres::NO_CONVERGENCE)
  {
    if (output_results_)
    {
      double initial_cost = summary.initial_cost / observations_per_image_;
      double final_cost = summary.final_cost / observations_per_image_;

      std::cout << "Initial Cost: " << initial_cost << '\n';
      std::cout << "Final Cost: " << final_cost << '\n';
    }

    result = Pose6D(result_pose[3], result_pose[4], result_pose[5],
      result_pose[0], result_pose[1], result_pose[2]);
    return true;
  }
  else
  {
    std::cerr << "Failed to find a pose from the target, did not converge" << '\n';
    return false;
  }
  return false;
}

void CameraOnWristIntrinsic::displayCovariance(void)
{
  CovarianceRequest intrinsic_params_request;
  intrinsic_params_request.request_type = CovarianceRequestType::IntrinsicParams;
  intrinsic_params_request.object_name = "Intrinsics";

  CovarianceRequest target_pose_params_request;
  target_pose_params_request.request_type = CovarianceRequestType::TargetPoseParams;
  target_pose_params_request.object_name = "Target Pose";

  std::vector<CovarianceRequest> covariance_request;
  covariance_request.push_back(intrinsic_params_request);
  covariance_request.push_back(target_pose_params_request);

  this->computeCovariance(covariance_request, 0, 
    result_.target_to_camera, result_.intrinsics);   
}

IntrinsicsVerification 
CameraOnWristIntrinsic::verifyIntrinsics(const ObservationPoints &observation_1,
  const Pose6D &pose_1, const ObservationPoints &observation_2, const Pose6D &pose_2,
  double intrinsics[9], double target_guess[6])
{
  Pose6D target_to_pose_1;
  Pose6D target_to_pose_2;

  // Find the poses
  this->findDistortedTarget(observation_1, target_to_pose_1, intrinsics, target_guess);
  this->findDistortedTarget(observation_2, target_to_pose_2, intrinsics, target_guess);

  // Compare these poses to input poses
  // Absolute Error = Actual Value - Measured Value
  IntrinsicsVerification result;
  result.target_diff_x = target_to_pose_1.x - target_to_pose_2.x; // m
  result.tool_diff_x = pose_1.x - pose_2.x; // m
  result.absolute_error_x = result.tool_diff_x - result.target_diff_x; // m
  
  result.target_diff_y = target_to_pose_1.y - target_to_pose_2.y; // m
  result.tool_diff_y = pose_1.y - pose_2.y; // m
  result.absolute_error_y = result.tool_diff_y - result.target_diff_y; // m
  
  result.target_diff_z = target_to_pose_1.z - target_to_pose_2.z; // m
  result.tool_diff_z = pose_1.z - pose_2.z; // m
  result.absolute_error_z = result.tool_diff_z - result.target_diff_z; // m

  return result;
}

} // namespace industrial_calibration_libs


