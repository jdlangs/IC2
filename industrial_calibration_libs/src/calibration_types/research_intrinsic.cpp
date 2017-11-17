#include <industrial_calibration_libs/calibration_types/research_intrinsic.h>
#include <iomanip>

namespace industrial_calibration_libs
{
// ResearchIntrinsic
ResearchIntrinsic::ResearchIntrinsic(const ObservationData &observation_data,
  const Target &target, const ResearchIntrinsicParams &params) :
  CalibrationJob(observation_data, target)
{
  std::memcpy(result_.intrinsics, params.intrinsics.data, 
    sizeof(result_.intrinsics));
  result_.target_to_camera_poses = params.target_to_camera_seed;
}

bool ResearchIntrinsic::runCalibration(void)
{
  if (!checkObservations()) {return false;}
  
  // double camera_intrinsics_seed[9];
  // std::memcpy(camera_intrinsics_seed, result_.intrinsics, sizeof(camera_intrinsics_seed));

  extrinsics_ = new double[num_images_*6];
  // double target_to_camera_pose_guess[6] = {0.0, 0.0, 0.0, 0.15, 0.15, 0.25};

  for (std::size_t i = 0; i < result_.target_to_camera_poses.size(); i++)
  {
#if 0              
    Pose6D target_to_camera_pose;
    this->findDistortedTarget(observation_data_[i], target_to_camera_pose,
      camera_intrinsics_seed, target_to_camera_pose_guess);

    extrinsics[6*i + 0] = target_to_camera_pose.ax;
    extrinsics[6*i + 1] = target_to_camera_pose.ay;
    extrinsics[6*i + 2] = target_to_camera_pose.az;
    extrinsics[6*i + 3] = target_to_camera_pose.x;
    extrinsics[6*i + 4] = target_to_camera_pose.y;
    extrinsics[6*i + 5] = target_to_camera_pose.z;
#endif

    // extrinsics[6*i + 0] = target_to_camera_pose_guess[0];
    // extrinsics[6*i + 1] = target_to_camera_pose_guess[1];
    // extrinsics[6*i + 2] = target_to_camera_pose_guess[2];
    // extrinsics[6*i + 3] = target_to_camera_pose_guess[3];
    // extrinsics[6*i + 4] = target_to_camera_pose_guess[4];
    // extrinsics[6*i + 5] = target_to_camera_pose_guess[5];   

    extrinsics_[6*i + 0] = result_.target_to_camera_poses[i].data[0];
    extrinsics_[6*i + 1] = result_.target_to_camera_poses[i].data[1];
    extrinsics_[6*i + 2] = result_.target_to_camera_poses[i].data[2];
    extrinsics_[6*i + 3] = result_.target_to_camera_poses[i].data[3];
    extrinsics_[6*i + 4] = result_.target_to_camera_poses[i].data[4];
    extrinsics_[6*i + 5] = result_.target_to_camera_poses[i].data[5];       
  }

  for (std::size_t i = 0; i < num_images_; i++)
  {
    for (std::size_t j = 0; j < observations_per_image_; j++)
    {
      Point3D point = target_.getDefinition().points[j];
      double observed_x = observation_data_[i][j].x;    
      double observed_y = observation_data_[i][j].y;

      ceres::CostFunction *cost_function =
        ResearchIntrinsicCF::Create(observed_x, observed_y, point);
      problem_.AddResidualBlock(cost_function, NULL, result_.intrinsics,
        &extrinsics_[6*i]);               
    }
  }

  // Solve
  options_.linear_solver_type = ceres::DENSE_SCHUR;
  options_.minimizer_progress_to_stdout = output_results_; 
  options_.max_num_iterations = 9001;

  ceres::Solve(options_, &problem_, &summary_);
  
  for (std::size_t i = 0; i < num_images_; i++)
  {
    Pose6D target_to_camera_pose(extrinsics_[6*i+3], extrinsics_[6*i+4], extrinsics_[6*i+5],
      extrinsics_[6*i+0], extrinsics_[6*i+1], extrinsics_[6*i+2]);
    result_.target_to_camera_poses.push_back(Extrinsics(target_to_camera_pose));
  }

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

// Copied from camera_on_wrist_intrinsic.cpp, should probably break out
// into separate "calibration type".
bool ResearchIntrinsic::findDistortedTarget(const ObservationPoints &observation_points,
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
    // if (output_results_)
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

void ResearchIntrinsic::displayCovariance(void) 
{
  // Not calling computeCovariance() since this is weird...
  ceres::Covariance::Options covariance_options;
  covariance_options.algorithm_type = ceres::DENSE_SVD;
  ceres::Covariance covariance(covariance_options);  

  std::vector<const double*> covariance_blocks;
  std::vector<int> block_sizes;
  std::vector<std::string> block_names;
  std::vector<std::pair<const double*, const double*>> covariance_pairs;

  // Intrinsics
  covariance_blocks.push_back(result_.intrinsics);
  block_sizes.push_back(9);
  block_names.push_back("Intrinsics1");

  covariance_blocks.push_back(result_.intrinsics);
  block_sizes.push_back(9);
  block_names.push_back("Intrinsics2");

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

  if (output_results_)
  {
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
  }
}

// --------------------------------------------------------------------------------

// ResearchIntrinsicTheory
ResearchIntrinsicTheory::ResearchIntrinsicTheory(const ObservationData &observation_data,
  const Target &target, const ResearchIntrinsicParams &params) :
  CalibrationJob(observation_data, target)
{
  double camera_matrix[4];
  double distortion_k[3];
  double distortion_p[2];

  camera_matrix[0] = params.intrinsics.data[0];
  camera_matrix[1] = params.intrinsics.data[1];
  camera_matrix[2] = params.intrinsics.data[2];
  camera_matrix[3] = params.intrinsics.data[3];

  distortion_k[0] = params.intrinsics.data[4];
  distortion_k[1] = params.intrinsics.data[5];
  distortion_k[2] = params.intrinsics.data[6];

  distortion_p[0] = params.intrinsics.data[7];
  distortion_p[1] = params.intrinsics.data[8];

  std::memcpy(result_.camera_matrix, camera_matrix, sizeof(result_.camera_matrix));
  std::memcpy(result_.distortion_k, distortion_k, sizeof(result_.distortion_k));
  std::memcpy(result_.distortion_p, distortion_p, sizeof(result_.distortion_p));

  // std::memcpy(result_.camera_matrix, &params.intrinsics.data[0], sizeof(result_.camera_matrix));
  // std::memcpy(result_.distortion_k, &params.intrinsics.data[4], sizeof(result_.distortion_k));
  // std::memcpy(result_.distortion_p, &params.intrinsics.data[7], sizeof(result_.distortion_p));

  result_.target_to_camera_poses = params.target_to_camera_seed;
}

bool ResearchIntrinsicTheory::runCalibration(void)
{
  if (!checkObservations()) {return false;}

  extrinsics_ = new double[num_images_*6];

  for (std::size_t i = 0; i < result_.target_to_camera_poses.size(); i++)
  {
    extrinsics_[6*i + 0] = result_.target_to_camera_poses[i].data[0];
    extrinsics_[6*i + 1] = result_.target_to_camera_poses[i].data[1];
    extrinsics_[6*i + 2] = result_.target_to_camera_poses[i].data[2];
    extrinsics_[6*i + 3] = result_.target_to_camera_poses[i].data[3];
    extrinsics_[6*i + 4] = result_.target_to_camera_poses[i].data[4];
    extrinsics_[6*i + 5] = result_.target_to_camera_poses[i].data[5];       
  }

  for (std::size_t i = 0; i < num_images_; i++)
  {
    for (std::size_t j = 0; j < observations_per_image_; j++)
    {
      Point3D point = target_.getDefinition().points[j];
      double observed_x = observation_data_[i][j].x;    
      double observed_y = observation_data_[i][j].y;

      ceres::CostFunction *cost_function =
        ResearchIntrinsicTheoryCF::Create(observed_x, observed_y, point);
      problem_.AddResidualBlock(cost_function, NULL, result_.camera_matrix,
        result_.distortion_k, result_.distortion_p, &extrinsics_[6*i]);               
    }
  }

  // Solve
  options_.linear_solver_type = ceres::DENSE_SCHUR;
  options_.minimizer_progress_to_stdout = output_results_; 
  options_.max_num_iterations = 9001;

  // Set distortion p's to constant and solve for rest
  problem_.SetParameterBlockConstant(result_.distortion_p);
  ceres::Solve(options_, &problem_, &summary_);

  // Set camera_matrix, distortion_k, and extrinsics constant and
  // solve for distortion_p
  problem_.SetParameterBlockConstant(result_.camera_matrix);
  problem_.SetParameterBlockConstant(result_.distortion_k);

  for (std::size_t i = 0; i < num_images_; i++)
  {
    problem_.SetParameterBlockConstant(&extrinsics_[6*i]);
  }

  problem_.SetParameterBlockVariable(result_.distortion_p);
  ceres::Solve(options_, &problem_, &summary_);
  
  problem_.SetParameterBlockVariable(result_.camera_matrix);
  problem_.SetParameterBlockVariable(result_.distortion_k);

  // Push data to result
  for (std::size_t i = 0; i < num_images_; i++)
  {
    Pose6D target_to_camera_pose(extrinsics_[6*i+3], extrinsics_[6*i+4], extrinsics_[6*i+5],
      extrinsics_[6*i+0], extrinsics_[6*i+1], extrinsics_[6*i+2]);
    result_.target_to_camera_poses.push_back(Extrinsics(target_to_camera_pose));
  }

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

void ResearchIntrinsicTheory::displayCovariance(void) 
{
  // Not calling computeCovariance() since this is weird...
  ceres::Covariance::Options covariance_options;
  covariance_options.algorithm_type = ceres::DENSE_SVD;
  ceres::Covariance covariance(covariance_options);  

  std::vector<const double*> covariance_blocks;
  std::vector<int> block_sizes;
  std::vector<std::string> block_names;
  std::vector<std::pair<const double*, const double*>> covariance_pairs;

  // Intrinsic Matrix
  covariance_blocks.push_back(result_.intrinsics);
  block_sizes.push_back(9);
  block_names.push_back("Intrinsics");

#if 0
  // Camera Matrix
  covariance_blocks.push_back(result_.camera_matrix);
  block_sizes.push_back(4);
  block_names.push_back("Camera Matrix [fx, fy, cx, cy]");
  
  // Distortion K
  covariance_blocks.push_back(result_.distortion_k);
  block_sizes.push_back(3);
  block_names.push_back("Distortion (k1, k2, k3)");

  // Distortion P
  covariance_blocks.push_back(result_.distortion_p);
  block_sizes.push_back(2);
  block_names.push_back("Distortion (p1, p2)");
#endif

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

  if (output_results_)
  {
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
  }
}
} // namespace industrial_calibration_libs