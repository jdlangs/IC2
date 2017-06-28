#include <industrial_calibration_libs/cost_functions.h>
#include <industrial_calibration_libs/calibration.h>

namespace industrial_calibration_libs
{
IntrinsicCalibration::IntrinsicCalibration(const ObservationData &observation_data, const Target &target, const std::vector<Pose6D> link_poses) : 
  observation_data_(observation_data), target_(target), link_poses_(link_poses) { }

bool IntrinsicCalibration::Calibrate(void)
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

  // Iterate through every observation image.
  double extrinsics[6];
  double intrinsics[9];
  double target_to_world[6];  
  for (std::size_t i = 0; i < num_images; i++)
  {
    // Iterate through every observation in the observation image.
    for (std::size_t j = 0; j < observations_per_image; j++)
    {
      double observed_x = observation_data_[i][j].x;
      double observed_y = observation_data_[i][j].y;
      Pose6D link_pose = link_poses_[i];
      Point3D point = target_.getData()->points[0];

      ceres::CostFunction* cost_function = 
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

  if (summary.termination_type != ceres::NO_CONVERGENCE)
  {
    initial_cost_ = summary.initial_cost / total_observations;
    final_cost_ = summary.final_cost / 
  }

  return false; // REPLACE THIS
}

} // namespace industrial_calibration_libs