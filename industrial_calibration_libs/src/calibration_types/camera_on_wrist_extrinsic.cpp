#include <industrial_calibration_libs/calibration_types/camera_on_wrist_extrinsic.h>

namespace industrial_calibration_libs
{
CameraOnWristExtrinsic::CameraOnWristExtrinsic(const ObservationData &observation_data, const Target &target, const CameraOnWristExtrinsicParams &params) 
  : CalibrationJob(observation_data, target) 
{
  link_poses_ = params.base_to_tool;
  std::memcpy(intrinsics_, params.intrinsics.data, 
    sizeof(intrinsics_));
  std::memcpy(result_.extrinsics, params.tool_to_camera.data,
    sizeof(result_.extrinsics));
  std::memcpy(result_.target_to_base, params.target_to_base.data,
    sizeof(result_.target_to_base));
}

bool CameraOnWristExtrinsic::runCalibration(void)
{
  if (link_poses_.size() == 0)
  {
    std::cerr << "base to tool Poses are Empty!" << '\n';
    return false;
  }

  if (!checkObservations()) {return false;}

  // Iterate through every observation image
  for (std::size_t i = 0; i < num_images_; i++)
  {
    // Iterate through every observation in each observation image
    for (std::size_t j = 0; j < observations_per_image_; j++)
    {
      Pose6D link_pose = link_poses_[i];
      Point3D point = target_.getDefinition().points[j];
      
      double observed_x = observation_data_[i][j].x;
      double observed_y = observation_data_[i][j].y;
      double focal_length_x = intrinsics_[0];
      double focal_length_y = intrinsics_[1];
      double optical_center_x = intrinsics_[2];
      double optical_center_y = intrinsics_[3];

      ceres::CostFunction *cost_function =
        CameraOnWristExtrinsicCF::Create(observed_x,
        observed_y, focal_length_x, focal_length_y, optical_center_x, 
        optical_center_y, link_pose, point);

      problem_.AddResidualBlock(cost_function, NULL, result_.extrinsics,
        result_.target_to_base);
    }
  }

  // Solve
  options_.linear_solver_type = ceres::DENSE_SCHUR;
  options_.minimizer_progress_to_stdout = output_results_;
  options_.max_num_iterations = 9001;

  if (output_results_)
  {
    options_.minimizer_progress_to_stdout = true;
  }

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

void CameraOnWristExtrinsic::displayCovariance(void)
{
  CovarianceRequest extrinsic_params_request;
  extrinsic_params_request.request_type = CovarianceRequestType::ExtrinsicParams;
  extrinsic_params_request.object_name = "Extrinsics";

  CovarianceRequest target_pose_params_request;
  target_pose_params_request.request_type = CovarianceRequestType::TargetPoseParams;
  target_pose_params_request.object_name = "Target Pose";

  std::vector<CovarianceRequest> covariance_request;
  covariance_request.push_back(extrinsic_params_request);
  covariance_request.push_back(target_pose_params_request);

  this->computeCovariance(covariance_request, result_.extrinsics, 
    result_.target_to_base); 
}
} // namespace industrial_calibration_libs
