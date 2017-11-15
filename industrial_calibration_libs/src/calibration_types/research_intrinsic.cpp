#include <industrial_calibration_libs/calibration_types/research_intrinsic.h>

namespace industrial_calibration_libs
{
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

  double* extrinsics = new double[num_images_*6];
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

    extrinsics[6*i + 0] = result_.target_to_camera_poses[i].data[0];
    extrinsics[6*i + 1] = result_.target_to_camera_poses[i].data[1];
    extrinsics[6*i + 2] = result_.target_to_camera_poses[i].data[2];
    extrinsics[6*i + 3] = result_.target_to_camera_poses[i].data[3];
    extrinsics[6*i + 4] = result_.target_to_camera_poses[i].data[4];
    extrinsics[6*i + 5] = result_.target_to_camera_poses[i].data[5];       
  }

  for (std::size_t i = 0; i < num_images_; i++)
  {
    for (std::size_t j = 0; j < observations_per_image_; j++)
    {
      Point3D point = target_.getDefinition().points[j];
      double observed_x = observation_data_[i][j].x;    
      double observed_y = observation_data_[i][j].y;

      ceres::CostFunction *cost_function =
        ResearchIntrinsicCF2::Create(observed_x, observed_y, point);
      problem_.AddResidualBlock(cost_function, NULL, result_.intrinsics,
        &extrinsics[6*i]);               
    }
  }

  // Solve
  options_.linear_solver_type = ceres::DENSE_SCHUR;
  options_.minimizer_progress_to_stdout = output_results_; 
  options_.max_num_iterations = 9001;

  ceres::Solve(options_, &problem_, &summary_);
  
  for (std::size_t i = 0; i < num_images_; i++)
  {
    Pose6D target_to_camera_pose(extrinsics[6*i+3], extrinsics[6*i+4], extrinsics[6*i+5],
      extrinsics[6*i+0], extrinsics[6*i+1], extrinsics[6*i+2]);
    result_.target_to_camera_poses.push_back(Extrinsics(target_to_camera_pose));
  }

  delete[] extrinsics;

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

#if 0
  // Calculate the guess pose for every image based off of guess intrinsics
  double target_to_camera_pose_guess[6] = {0.0, 0.0, 0.0, 0.15, 0.15, 0.25};
  for (std::size_t i = 0; i < num_images_; i++)
  {
    Pose6D target_to_camera_pose;
    this->findDistortedTarget(observation_data_[i], target_to_camera_pose,
      camera_intrinsics_seed, target_to_camera_pose_guess);
      // result_.intrinsics, target_to_camera_pose_guess);

    result_.target_to_camera_poses.push_back(Extrinsics(target_to_camera_pose));

    for (std::size_t j = 0; j < observations_per_image_; j++)
    {
      Point3D point = target_.getDefinition().points[j];
      double observed_x = observation_data_[i][j].x;    
      double observed_y = observation_data_[i][j].y;    

      ceres::CostFunction *cost_function =
        ResearchIntrinsicCF::Create(observed_x, observed_y, 
          result_.target_to_camera_poses[i].asPose6D(), point);
      problem_.AddResidualBlock(cost_function, NULL, result_.intrinsics);        
    }
  }
#endif
#if 0
  // Create vector if target_to_camera poses
  double target_to_camera_pose_guess[6] = {0.0, 0.0, 0.0, 0.15, 0.15, 0.25};
  double *target_to_camera = new double[num_images_*6];
  for (std::size_t i = 0; i < num_images_; i++)
  {
    Pose6D target_to_camera_pose;
    this->findDistortedTarget(observation_data_[i], target_to_camera_pose,
      result_.intrinsics, target_to_camera_pose_guess);
    target_to_camera[(6*i)+0] = target_to_camera_pose.ax;
    target_to_camera[(6*i)+1] = target_to_camera_pose.ay;
    target_to_camera[(6*i)+2] = target_to_camera_pose.az;
    target_to_camera[(6*i)+3] = target_to_camera_pose.x;
    target_to_camera[(6*i)+4] = target_to_camera_pose.y;
    target_to_camera[(6*i)+5] = target_to_camera_pose.z;
  }

  for (std::size_t i = 0; i < num_images_; i++)
  {
    for (std::size_t j = 0; j < observations_per_image_; j++)
    {
      Point3D point = target_.getDefinition().points[j];
      double observed_x = observation_data_[i][j].x;    
      double observed_y = observation_data_[i][j].y;    

      ceres::CostFunction *cost_function =
        ResearchIntrinsicCF3::Create(observed_x, observed_y, 
          point, num_images_, i);      
      problem_.AddResidualBlock(cost_function, NULL, result_.intrinsics,
        target_to_camera);      
    }
  }
#endif

#if 0
  for (std::size_t i = 0; i < num_images_; i++)
  {
    double target_to_camera_pose_guess[6] = {0.0, 0.0, 0.0, 0.15, 0.15, 0.25};

    Pose6D target_to_camera_pose;
    this->findDistortedTarget(observation_data_[i], target_to_camera_pose,
      result_.intrinsics, target_to_camera_pose_guess);

#if 0
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

    // Pose6D target_to_camera_pose;
    // target_to_camera_pose.ax = target_to_camera[0];
    // target_to_camera_pose.ax = target_to_camera[1];
    // target_to_camera_pose.ax = target_to_camera[2];
    // target_to_camera_pose.x =  target_to_camera[3];
    // target_to_camera_pose.y =  target_to_camera[4];
    // target_to_camera_pose.z =  target_to_camera[5];

    // std::cerr << target_to_camera_pose.ax
    //   << " " << target_to_camera_pose.ay
    //   << " " << target_to_camera_pose.az
    //   << " " << target_to_camera_pose.x
    //   << " " << target_to_camera_pose.y
    //   << " " << target_to_camera_pose.z << '\n';
#endif

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
#endif
#if 0
  // Solve
  options_.linear_solver_type = ceres::DENSE_SCHUR;
  options_.minimizer_progress_to_stdout = output_results_; 
  options_.max_num_iterations = 9001;

  ceres::Solve(options_, &problem_, &summary_);
  
  // delete[] target_to_camera;

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
#endif
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
} // namespace industrial_calibration_libs