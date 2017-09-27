#include <industrial_calibration_libs/cost_functions.h>
#include <industrial_calibration_libs/calibration.h>
#include <iomanip>

namespace industrial_calibration_libs
{
CalibrationJob::CalibrationJob(const ObservationData &observation_data,
  const Target &target) : observation_data_(observation_data), target_(target) { }

bool CalibrationJob::checkObservations(void)
{
  // Checks if number of observations per image is consistant by setting
  // observations_per_image_ to the number of observations in the first index
  // of observation_data_.
  num_images_ = observation_data_.size();
  for (std::size_t i = 0; i < num_images_; i++)
  {
    if (i == 0)
    {
      observations_per_image_ = observation_data_[i].size();
    }
    else
    {
      if (observations_per_image_ != observation_data_[i].size()) {return false;}
    }
  }
  // Set total observations
  total_observations_ = num_images_ * observations_per_image_; 
  return true; 
}

bool CalibrationJob::computeCovariance(const std::vector<CovarianceRequest> &requests, double* extrinsics_result, double* target_pose_result,
  double* intrinsics_result)
{
  ceres::Covariance::Options covariance_options;
  covariance_options.algorithm_type = ceres::DENSE_SVD;
  ceres::Covariance covariance(covariance_options);

  std::vector<const double*> covariance_blocks;
  std::vector<int> block_sizes;
  std::vector<std::string> block_names;
  std::vector<std::pair<const double*, const double*>> covariance_pairs;

  for (auto &request : requests)
  {
    double* intrinsics;
    double* extrinsics;
    double* target_pose;

    switch (request.request_type)
    {
      case IntrinsicParams:
        intrinsics = intrinsics_result;
        covariance_blocks.push_back(intrinsics);
        block_sizes.push_back(9);
        block_names.push_back(request.object_name);
        break;

      case ExtrinsicParams:
        extrinsics = extrinsics_result;
        covariance_blocks.push_back(extrinsics);
        block_sizes.push_back(6);
        block_names.push_back(request.object_name);
        break;

      case TargetPoseParams:
        target_pose = target_pose_result;
        covariance_blocks.push_back(target_pose);
        block_sizes.push_back(6);
        block_names.push_back(request.object_name);
        break;

      default:
        return false;
        break;
    }
  }

  // Create pairs from every combination of blocks in request
  for (std::size_t i = 0; i < covariance_blocks.size(); i++)
  {
    for (std::size_t j = 0; j < covariance_blocks.size(); j++)
    {
      covariance_pairs.push_back(std::make_pair(covariance_blocks[i],
        covariance_blocks[j]));
    }
  }

  covariance.Compute(covariance_pairs, &problem_);

  // TEMPORARY (OUTPUT RESULTS TO SCREEN)
  std::cout << "Covariance Blocks: " << '\n';
  for (std::size_t i = 0; i < covariance_blocks.size(); i++)
  {
    for (std::size_t j = 0; j < covariance_blocks.size(); j++)
    {
      std::cout << "Covariance [" << block_names[i] << ", " 
        << block_names[j] << "]" << '\n';

      int N = block_sizes[i];
      int M = block_sizes[j];
      double* ij_cov_block = new double[N*M];

      covariance.GetCovarianceBlock(covariance_blocks[i], covariance_blocks[j],
        ij_cov_block);

      for (int q = 0; q < N; q++)
      {
        std::cout << "[";
        for (int k = 0; k < M; k++)
        {
          double sigma_i = sqrt(ij_cov_block[q*N+q]);
          double sigma_j = sqrt(ij_cov_block[k*N+k]);
          if (q == k)
          {
            if (sigma_i > 1.0 || sigma_i < -1.0)
            {
              std::cout << " " << std::right << std::setw(9) << std::scientific 
                << std::setprecision(1) << sigma_i;              
            }
            else
            {
              std::cout << " " << std::right << std::setw(9) << std::fixed
                << std::setprecision(5) << sigma_i;
            }
          }
          else
          {
            if (ij_cov_block[q*N + k]/(sigma_i * sigma_j) > 1.0 ||
              ij_cov_block[q*N + k]/(sigma_i * sigma_j) < -1.0)
            {
              std::cout << " " << std::right << std::setw(9) << std::scientific
                << std::setprecision(1) << ij_cov_block[q*N + k]/(sigma_i * sigma_j);
            }
            else
            {
              std::cout << " " << std::right << std::setw(9) << std::fixed 
                << std::setprecision(5) << ij_cov_block[q*N + k]/(sigma_i * sigma_j);
            }
          }
        }
        std::cout << "]" << '\n';
      }
      delete [] ij_cov_block;
    }
  }
  return true;
}

MovingCameraOnWristStaticTargetExtrinsic::MovingCameraOnWristStaticTargetExtrinsic(const ObservationData &observation_data, const Target &target) : CalibrationJob(observation_data, target) { }

void MovingCameraOnWristStaticTargetExtrinsic::initKnownValues(const std::vector<Pose6D> &link_poses, const double intrinsics[4])
{
  link_poses_ = link_poses;
  std::memcpy(intrinsics_, intrinsics, sizeof(intrinsics_));
}

void MovingCameraOnWristStaticTargetExtrinsic::initSeedValues(const double extrinsics[6], const double target_to_base[6])
{
  std::memcpy(result_.extrinsics, extrinsics, sizeof(result_.extrinsics));
  std::memcpy(result_.target_to_base, target_to_base, 
    sizeof(result_.target_to_base));
}

bool MovingCameraOnWristStaticTargetExtrinsic::runCalibration(void)
{
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
        MovingCameraOnWristStaticTargetExtrinsicCF::Create(observed_x,
        observed_y, focal_length_x, focal_length_y, optical_center_x, 
        optical_center_y, link_pose, point);

      problem_.AddResidualBlock(cost_function, NULL, result_.extrinsics,
        result_.target_to_base);
    }
  }

  // Solve
  options_.linear_solver_type = ceres::DENSE_SCHUR;
  options_.minimizer_progress_to_stdout = true; // REMOVE THIS LATER ???
  options_.max_num_iterations = 9001;

  ceres::Solve(options_, &problem_, &summary_);

  // REMOVE THIS
  std::cout << summary_.FullReport() << '\n';

  if (summary_.termination_type != ceres::NO_CONVERGENCE)
  {
    initial_cost_ = summary_.initial_cost / total_observations_;
    final_cost_ = summary_.final_cost / total_observations_;
    return true;    
  }

  return false;
}

void MovingCameraOnWristStaticTargetExtrinsic::displayCovariance(void)
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

MovingCameraOnWristStaticTargetIntrinsic::MovingCameraOnWristStaticTargetIntrinsic(const ObservationData &observation_data, const Target &target) : CalibrationJob(observation_data, target) { }

void MovingCameraOnWristStaticTargetIntrinsic::initKnownValues(const std::vector<Pose6D> &link_poses)
{
  link_poses_ = link_poses;
}

void MovingCameraOnWristStaticTargetIntrinsic::initSeedValues(const double extrinsics[6], const double target_to_base[6], const double intrinsics[9])
{
  std::memcpy(result_.extrinsics, extrinsics, sizeof(result_.extrinsics));
  std::memcpy(result_.target_to_base, target_to_base, 
    sizeof(result_.target_to_base));
  std::memcpy(result_.intrinsics, intrinsics, sizeof(result_.intrinsics));
}

bool MovingCameraOnWristStaticTargetIntrinsic::runCalibration(void)
{
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

      ceres::CostFunction *cost_function =
        MovingCameraOnWristStaticTargetIntrinsicCF::Create(observed_x,
        observed_y, link_pose, point);

      problem_.AddResidualBlock(cost_function, NULL, result_.extrinsics,
        result_.intrinsics, result_.target_to_base);
    }
  }

  // Solve
  options_.linear_solver_type = ceres::DENSE_SCHUR;
  options_.minimizer_progress_to_stdout = true; // REMOVE THIS LATER ???
  options_.max_num_iterations = 9001;

  ceres::Solve(options_, &problem_, &summary_);

  // REMOVE THIS ONE DAY
  std::cout << summary_.FullReport() << '\n';  

  if (summary_.termination_type != ceres::NO_CONVERGENCE)
  {
    initial_cost_ = summary_.initial_cost / total_observations_;
    final_cost_ = summary_.final_cost / total_observations_;
    return true;    
  }

  return false;  
}

void MovingCameraOnWristStaticTargetIntrinsic::displayCovariance(void)
{
  CovarianceRequest extrinsic_params_request;
  extrinsic_params_request.request_type = CovarianceRequestType::ExtrinsicParams;
  extrinsic_params_request.object_name = "Extrinsics";

  CovarianceRequest target_pose_params_request;
  target_pose_params_request.request_type = CovarianceRequestType::TargetPoseParams;
  target_pose_params_request.object_name = "Target Pose";

  CovarianceRequest intrinsic_params_request;
  intrinsic_params_request.request_type = CovarianceRequestType::IntrinsicParams;
  intrinsic_params_request.object_name = "Intrinsics";

  std::vector<CovarianceRequest> covariance_request;
  covariance_request.push_back(extrinsic_params_request);
  covariance_request.push_back(target_pose_params_request);
  covariance_request.push_back(intrinsic_params_request);

  this->computeCovariance(covariance_request, result_.extrinsics, 
    result_.target_to_base, result_.intrinsics);   
}
} // namespace industrial_calibration_libs