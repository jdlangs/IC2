#include <industrial_calibration_libs/calibration_types/research_intrinsic.h>

namespace industrial_calibration_libs
{
ResearchIntrinsic::ResearchIntrinsic(const ObservationData &observation_data,
  const Target &target, const ResearchIntrinsicParams &params) :
  CalibrationJob(observation_data, target)
{
  std::memcpy(result_.intrinsics, params.intrinsics.data, 
    sizeof(result_.intrinsics)); 
}

bool ResearchIntrinsic::runCalibration(void)
{
  if (!checkObservations()) {return false;}
  
  // For each observation, find the pose between camera and target.
  double seed_intrinsics[9];
  std::memcpy(seed_intrinsics, result_.intrinsics, sizeof(seed_intrinsics));

  for (std::size_t i = 0; i < num_images_; i++)
  {
    // double target_to_camera_pose_guess[6] = {0.0, 0.0, 0.0, 0.15, 0.15, 0.25};
    
    // Pose6D target_to_camera_pose;
    // this->findDistortedTarget(observation_data_[i], target_to_camera_pose,
    //   seed_intrinsics, target_to_camera_pose_guess);

    // double target_to_camera[6];
    // target_to_camera[0] = target_to_camera_pose.ax;
    // target_to_camera[1] = target_to_camera_pose.ay;
    // target_to_camera[2] = target_to_camera_pose.az;
    // target_to_camera[3] = target_to_camera_pose.x;
    // target_to_camera[4] = target_to_camera_pose.y;
    // target_to_camera[5] = target_to_camera_pose.z;

    // Guess
    double target_to_camera[6] = {0.0, 0.0, 0.0, 0.15, 0.15, 0.25};

    // First get pose between target and camera while calibrating intrinsics
    // This is bad... fix it
    ceres::Problem problem;
    for (std::size_t j = 0; j < observations_per_image_; j++)
    {
      Point3D point = target_.getDefinition().points[j];
      double observed_x = observation_data_[i][j].x;    
      double observed_y = observation_data_[i][j].y;

      ceres::CostFunction *cost_function =
      ResearchIntrinsicCF2::Create(observed_x, observed_y, point);      
      problem.AddResidualBlock(cost_function, NULL, result_.intrinsics,
      target_to_camera);      
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
    }
    // ^^^^^^^^^^^^^^ MOVE THIS OUT

    Pose6D target_to_camera_pose;
    target_to_camera_pose.ax = target_to_camera[0];
    target_to_camera_pose.ax = target_to_camera[1];
    target_to_camera_pose.ax = target_to_camera[2];
    target_to_camera_pose.x =  target_to_camera[3];
    target_to_camera_pose.y =  target_to_camera[4];
    target_to_camera_pose.z =  target_to_camera[5];

    // std::cerr << target_to_camera_pose.ax
    //   << " " << target_to_camera_pose.ay
    //   << " " << target_to_camera_pose.az
    //   << " " << target_to_camera_pose.x
    //   << " " << target_to_camera_pose.y
    //   << " " << target_to_camera_pose.z << '\n';

    // Use that pose to calibrate intrinsics even more...
    for (std::size_t j = 0; j < observations_per_image_; j++)
    {
      Point3D point = target_.getDefinition().points[j];
      double observed_x = observation_data_[i][j].x;    
      double observed_y = observation_data_[i][j].y;    

      ceres::CostFunction *cost_function =
        ResearchIntrinsicCF::Create(observed_x, observed_y, 
          target_to_camera_pose, point);
        // ResearchIntrinsicCF2::Create(observed_x, observed_y, 
        //   point);      
      problem_.AddResidualBlock(cost_function, NULL, result_.intrinsics);        
      // problem_.AddResidualBlock(cost_function, NULL, result_.intrinsics,
      //   target_to_camera);
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
#if 0
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
#endif
} // namespace industrial_calibration_libs