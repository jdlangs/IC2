#include <industrial_calibration_libs/cost_functions.h>
#include <industrial_calibration_libs/calibration.h>

namespace industrial_calibration_libs
{
ExtrinsicCalibration::ExtrinsicCalibration(const ObservationData &observation_data,
  const Target &target, const std::vector<Pose6D> link_poses, 
  const double intrinsics[4]) : observation_data_(observation_data), 
  target_(target), link_poses_(link_poses) 
{
  setIntrinsics(intrinsics); 
}

void ExtrinsicCalibration::setSeedValues(const double extrinsics[6], 
  const double target_to_world[6])
{
  for (std::size_t i = 0; i < 6; i++)
  {
    extrinsics_seed_[i] = extrinsics[i];
    target_to_world_seed_[i] = target_to_world[i];
  }
}

bool ExtrinsicCalibration::calibrate(void)
{
  ceres::Problem problem;

  std::size_t num_images = observation_data_.size();

  // Check if number of observations per image is consistent.
  std::size_t observations_per_image;
  for (std::size_t i = 0; i < num_images; i++)
  {
    if (i == 0) {observations_per_image = observation_data_[i].size();}
    else
    {
      if (observations_per_image != observation_data_[i].size()) {return false;}
    }
  }

  // Calculates the total number of observations across all images
  std::size_t total_observations = num_images * observations_per_image;

  // Allocate outputs
  double extrinsics[6];
  double target_to_world[6];

  // TODO(gChiou): This is temporary, refactor this.
  for (std::size_t i = 0; i < 6; i++)
  {
    extrinsics[i] = extrinsics_seed_[i];
    target_to_world[i] = target_to_world_seed_[i];
  }

  // Iterate through every observation image
  for (std::size_t i = 0; i < num_images; i++)
  {
    // Itegrate through every observation in the observation image.
    for (std::size_t j = 0; j < observations_per_image; j++)
    {
      double observed_x = observation_data_[i][j].x;
      double observed_y = observation_data_[i][j].y;
      Pose6D link_pose = link_poses_[i];
      Point3D point = target_.getData().points[0];
      double focal_length_x = intrinsics_[0];
      double focal_length_y = intrinsics_[1];
      double optical_center_x = intrinsics_[2];
      double optical_center_y = intrinsics_[3];

      ceres::CostFunction *cost_function =
        CameraOnWristStaticTargetExtrinsic::Create(observed_x, observed_y,
          focal_length_x, focal_length_y, optical_center_x, optical_center_y,
          link_pose, point);

      problem.AddResidualBlock(cost_function, NULL, extrinsics, target_to_world);
    }
  }

  // Solve
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 2000;
  ceres::Solve(options, &problem, &summary);

  std::cerr << summary.FullReport() << '\n';
  
  if (summary.termination_type != ceres::NO_CONVERGENCE)
  {
    initial_cost_ = summary.initial_cost / total_observations;
    final_cost_ = summary.final_cost / total_observations;
    setResults(extrinsics, target_to_world);
    return true;
  }

  return false;
}

void ExtrinsicCalibration::setIntrinsics(const double intrinsics[4])
{
  for (std::size_t i = 0; i < 4; i++)
  {
    intrinsics_[i] = intrinsics[i];
  }
}

ExtrinsicResults ExtrinsicCalibration::getResults(void)
{
  return results_;
}

void ExtrinsicCalibration::setResults(const double extrinsics[6],
  const double target_to_world[6])
{
  for (std::size_t i = 0; i < 6; i++)
  {
    results_.extrinsics[i] = extrinsics[i];
    results_.target_to_world[i] = target_to_world[i];
  }
}

IntrinsicCalibration::IntrinsicCalibration(const ObservationData &observation_data, const Target &target, const std::vector<Pose6D> link_poses) : 
  observation_data_(observation_data), target_(target), link_poses_(link_poses) { }

void IntrinsicCalibration::setSeedValues(const double extrinsics[6],
  const double target_to_world[6], const double intrinsics[6])
{
  for (std::size_t i = 0; i < 9; i++)
  {
    if (i < 6)
    {
      extrinsics_seed_[i] = extrinsics[i];
      target_to_world_seed_[i] = target_to_world[i];
    }
    intrinsics_seed_[i] = intrinsics[i];
  }
}

bool IntrinsicCalibration::calibrate(void)
{
  ceres::Problem problem;

  std::size_t num_images = observation_data_.size();

  // Check if number of observations per image is consistent.
  std::size_t observations_per_image;
  for (std::size_t i = 0; i < num_images; i++)
  {
    if (i == 0) {observations_per_image = observation_data_[i].size();}
    else
    {
      if (observations_per_image != observation_data_[i].size()) {return false;}
    }
  }

  // Calculates the total number of observations across all images
  std::size_t total_observations = num_images * observations_per_image;

  // Allocate outputs
  double extrinsics[6];
  double intrinsics[9];
  double target_to_world[6];

  for (std::size_t i = 0; i < 9; i++)
  {
    if (i < 6)
    {
      extrinsics[i] = extrinsics_seed_[i];
      target_to_world[i] = target_to_world_seed_[i];
    }
    intrinsics[i] = intrinsics_seed_[i];
  }  

  // Iterate through every observation image.
  for (std::size_t i = 0; i < num_images; i++)
  {
    // Iterate through every observation in the observation image.
    for (std::size_t j = 0; j < observations_per_image; j++)
    {
      double observed_x = observation_data_[i][j].x;
      double observed_y = observation_data_[i][j].y;
      Pose6D link_pose = link_poses_[i];
      Point3D point = target_.getData().points[0];

      ceres::CostFunction *cost_function = 
        CameraOnWristStaticTargetIntrinsic::Create(observed_x, observed_y, 
          link_pose, point);

      problem.AddResidualBlock(cost_function, NULL, extrinsics, 
        intrinsics, target_to_world);
    }
  }

  // Solve
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 2000;
  ceres::Solve(options, &problem, &summary);
  
  std::cerr << summary.FullReport() << '\n';

  if (summary.termination_type != ceres::NO_CONVERGENCE)
  {
    initial_cost_ = summary.initial_cost / total_observations;
    final_cost_ = summary.final_cost / total_observations;
    setResults(extrinsics, intrinsics, target_to_world);
    return true;
  }

  return false;
}

IntrinsicResults IntrinsicCalibration::getResults(void)
{
  return results_;
}

void IntrinsicCalibration::setResults(const double extrinsics[6], 
  const double intrinsics[9], const double target_to_world[6])
{
  // Note(gChiou): This is messy...
  for (std::size_t i = 0; i < 9; i++)
  {
    if (i < 6)
    {
      results_.extrinsics[i] = extrinsics[i];
      results_.target_to_world[i] = target_to_world[i];
    }
    results_.intrinsics[i] = intrinsics[i];
  }
}

} // namespace industrial_calibration_libs