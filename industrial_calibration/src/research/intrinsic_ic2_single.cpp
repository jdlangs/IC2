/*
  This file will run through the data set using industrial_calibration's 
  circle grid finder and calibration solver.
*/
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <industrial_calibration/helper_functions.h>

#include <opencv2/core/core.hpp>

#include <fstream>

#define ICL industrial_calibration_libs

#define RUN_THEORY true
#define OUTPUT_EXTRINSICS false
#define VISUALIZE_RESULTS false
#define SAVE_DATA false

struct CalibrationResult
{
  std::size_t observation_set;
  double rms;
  double total_average_error;
  double intrinsics[9];
};

// Function Declarations
void calibrateObservationSet(const std::string &data_dir,
  const ICL::ObservationData &observation_data,
  const ICL::Target &target,
  const std::size_t &observation_set_index,
  std::vector<CalibrationResult> &out_results);

bool loadSeedExtrinsics(const std::string &data_path, const std::size_t &index,
  std::size_t num_images, std::vector<ICL::Extrinsics> &extrinsics_seed);

bool saveResultData(const std::string &result_path,
  const std::vector<CalibrationResult> &results);

void visualizeResults(const ICL::ResearchIntrinsic::Result &results,
  const ICL::Target &target, const CalibrationImages cal_images);

// Function Implementatins
bool saveResultData(const std::string &result_path, 
  const std::vector<CalibrationResult> &results)
{
  std::ofstream result_file;
  result_file.open(result_path);

  result_file << "Set,Fx,Fy,Cx,Cy,k1,k2,p1,p2,k3,Reprj Error" << '\n';
  for (std::size_t i = 0; i < results.size(); i++)
  {
    result_file << results[i].observation_set << ','
      << results[i].intrinsics[0] << ','
      << results[i].intrinsics[1] << ','
      << results[i].intrinsics[2] << ','
      << results[i].intrinsics[3] << ','
      << results[i].intrinsics[4] << ','
      << results[i].intrinsics[5] << ','
      << results[i].intrinsics[7] << ','
      << results[i].intrinsics[8] << ','
      << results[i].intrinsics[6] << ','
      << results[i].rms << '\n';
  }
  result_file.close();

  return true;
}

bool loadSeedExtrinsics(const std::string &data_path, const std::size_t &index,
  std::size_t num_images, std::vector<ICL::Extrinsics> &extrinsics_seed)
{
  extrinsics_seed.reserve(num_images);

  YAML::Node extrinsics_seed_yaml;
  std::string path = data_path + "results/" + std::to_string(index) 
    + "_estimated_poses.yaml";

  try
  {
    extrinsics_seed_yaml = YAML::LoadFile(path);
    if (!extrinsics_seed_yaml["rvec_0"]) {return false;}
  }
  catch (YAML::BadFile &bf) {return false;}

  for (std::size_t i = 0; i < num_images; i++)
  {
    ICL::Extrinsics extrinsics_temp;
    std::vector<double> rvecs; rvecs.reserve(3);
    std::vector<double> tvecs; tvecs.reserve(3);

    if (extrinsics_seed_yaml["rvec_" + std::to_string(i)] 
      && extrinsics_seed_yaml["tvec_" + std::to_string(i)])
    {
      for (std::size_t j = 0; 
        j < extrinsics_seed_yaml["rvec_" + std::to_string(i)].size();
        j++)
      {
        double value = extrinsics_seed_yaml["rvec_" + std::to_string(i)][j].as<double>();
        rvecs.push_back(value);
      }
      for (std::size_t j = 0; 
        j < extrinsics_seed_yaml["tvec_" + std::to_string(i)].size();
        j++)
      {
        double value = extrinsics_seed_yaml["tvec_" + std::to_string(i)][j].as<double>();
        tvecs.push_back(value);        
      }      
    }
    // Check
    if (rvecs.size() == tvecs.size())
    {
      extrinsics_temp = ICL::Extrinsics(rvecs[0], rvecs[1], rvecs[2],
        tvecs[0], tvecs[1], tvecs[2]);
      extrinsics_seed.push_back(extrinsics_temp);
    }
    else {return false;}
  }
  return true;
}

void visualizeResults(const ICL::ResearchIntrinsic::Result &results,
  const ICL::Target &target, const CalibrationImages cal_images)
{
  for (std::size_t i = 0; i < cal_images.size(); i++)
  {
    ICL::Pose6D target_to_camera_p = results.target_to_camera_poses[i].asPose6D();
    double t_to_c[6];

    t_to_c[0] = target_to_camera_p.ax;
    t_to_c[1] = target_to_camera_p.ay;
    t_to_c[2] = target_to_camera_p.az;
    t_to_c[3] = target_to_camera_p.x;
    t_to_c[4] = target_to_camera_p.y;
    t_to_c[5] = target_to_camera_p.z;

    const double* target_angle_axis(&t_to_c[0]);
    const double* target_position(&t_to_c[3]);

    ICL::ObservationPoints observation_points;

    for (std::size_t j = 0; j < target.getDefinition().points.size(); j++)
    {
      double camera_point[3];
      ICL::Point3D target_point(target.getDefinition().points[j]);

      ICL::transformPoint3D(target_angle_axis, target_position, 
        target_point.asVector(), camera_point);

      double fx, fy, cx, cy, k1, k2, k3, p1, p2;
      fx  = results.intrinsics[0]; /** focal length x */
      fy  = results.intrinsics[1]; /** focal length y */
      cx  = results.intrinsics[2]; /** central point x */
      cy  = results.intrinsics[3]; /** central point y */
      k1  = results.intrinsics[4]; /** distortion k1  */
      k2  = results.intrinsics[5]; /** distortion k2  */
      k3  = results.intrinsics[6]; /** distortion k3  */
      p1  = results.intrinsics[7]; /** distortion p1  */
      p2  = results.intrinsics[8]; /** distortion p2  */      

      double xp1 = camera_point[0];   
      double yp1 = camera_point[1];
      double zp1 = camera_point[2];

      double xp, yp;
      if (std::roundf(zp1*10000)/10000 == 0.0)
      {
        xp = xp1;
        yp = yp1;
      }
      else
      {
        xp = xp1 / zp1;
        yp = yp1 / zp1;
      }

      double xp2 = xp * xp;
      double yp2 = yp * yp;
      double r2 = xp2 + yp2;
      double r4 = r2 * r2;
      double r6 = r2 * r4;

      double xpp = xp
        + k1 * r2 * xp    // 2nd order term
        + k2 * r4 * xp    // 4th order term
        + k3 * r6 * xp    // 6th order term  
        + p2 * (r2 + 2.0 * xp2) // tangential
        + p1 * xp * yp * 2.0; // other tangential term

      double ypp = yp
        + k1 * r2 * yp    // 2nd order term
        + k2 * r4 * yp    // 4th order term
        + k3 * r6 * yp    // 6th order term
        + p1 * (r2 + 2.0 * yp2) // tangential term
        + p2 * xp * yp * 2.0; // other tangential term      

      double point_x = fx * xpp + cx;
      double point_y = fy * ypp + cy;

      cv::Point2d cv_point(point_x, point_y);
      observation_points.push_back(cv_point);
    }

    cv::Mat result_image;
    drawResultPoints(cal_images[i], result_image, observation_points,
      target.getDefinition().target_rows, target.getDefinition().target_cols);
    cv::imshow("Result Image", result_image);
    cv::waitKey(0);
  }  
}

void calibrateObservationSet(const std::string &data_dir,
  const ICL::ObservationData &observation_data,
  const ICL::Target &target,
  const std::size_t &observation_set_index,
  std::vector<CalibrationResult> &out_results)
{
  // Seed parameters (Average of intrinsic values from OpenCV)
  double camera_info[9]; 
  camera_info[0] = 533.2;
  camera_info[1] = 534.3;
  camera_info[2] = 316.4;
  camera_info[3] = 233.2;
  camera_info[4] = 0.0;
  camera_info[5] = 0.0;
  camera_info[6] = 0.0;
  camera_info[7] = 0.0;
  camera_info[8] = 0.0;

  ROS_INFO_STREAM("Running Calibration for Observation Set: " << observation_set_index);
  ICL::ResearchIntrinsicParams params;
  params.intrinsics = ICL::IntrinsicsFull(camera_info);

  // Get seed extrinsics
  params.target_to_camera_seed;
  if (!loadSeedExtrinsics(data_dir, observation_set_index, 
    observation_data.size(), params.target_to_camera_seed))
  {
    ROS_ERROR_STREAM("Failed to load seed extrinsics");
  }

#if RUN_THEORY
  ICL::ResearchIntrinsicTheory calibration(observation_data, target, params);
#else
  ICL::ResearchIntrinsic calibration(observation_data, target, params);
#endif
  
  calibration.setOutput(true); // Enable output to console.
  calibration.runCalibration();
  // calibration.displayCovariance();

  // Print out results.
#if RUN_THEORY
  ICL::ResearchIntrinsicTheory::Result results = calibration.getResults();
#else
  ICL::ResearchIntrinsic::Result results = calibration.getResults();
#endif

  ROS_INFO_STREAM("Initial Cost: " << calibration.getInitialCost());
  ROS_INFO_STREAM("Final Cost: " << calibration.getFinalCost());
  ROS_INFO_STREAM("Intrinsic Parameters");
  ROS_INFO_STREAM("----------------------------------------");
  ROS_INFO_STREAM("Focal Length x: " << results.intrinsics[0]);
  ROS_INFO_STREAM("Focal Length y: " << results.intrinsics[1]);
  ROS_INFO_STREAM("Optical Center x: " << results.intrinsics[2]);
  ROS_INFO_STREAM("Optical Center y: " << results.intrinsics[3]);
  ROS_INFO_STREAM("Distortion k1: " << results.intrinsics[4]);
  ROS_INFO_STREAM("Distortion k2: " << results.intrinsics[5]);
  ROS_INFO_STREAM("Distortion k3: " << results.intrinsics[6]);
  ROS_INFO_STREAM("Distortion p1: " << results.intrinsics[7]);
  ROS_INFO_STREAM("Distortion p2: " << results.intrinsics[8]);

  // Set result
  CalibrationResult result;
  result.observation_set = observation_set_index;
  result.rms = calibration.getFinalCost();
  result.total_average_error = calibration.getFinalCost();
  std::memcpy(result.intrinsics, results.intrinsics, sizeof(result.intrinsics));
  out_results[observation_set_index] = result;

#if VISUALIZE_RESULTS
  visualizeResults(results, target, cal_images);
#else
#endif

#if OUTPUT_EXTRINSICS
  for (std::size_t i = 0; i < cal_images.size(); i++)
  {
    std::vector<double> rvecs = {results.target_to_camera_poses[i].data[0], 
      results.target_to_camera_poses[i].data[1], 
      results.target_to_camera_poses[i].data[2]};
    std::vector<double> tvecs = {results.target_to_camera_poses[i].data[3],
      results.target_to_camera_poses[i].data[4],
      results.target_to_camera_poses[i].data[5]};
    ROS_INFO_STREAM("rvecs: " << rvecs);
    ROS_INFO_STREAM("tvecs: " << tvecs);
  }
#else
#endif
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intrinsic_ic2_single");
  ros::NodeHandle pnh("~");
  
  std::string data_dir;
  pnh.getParam("data_dir", data_dir);
  data_dir = addSlashToEnd(data_dir);

  // This calibration will run on a single data set but leave out 
  // a single image every calibration.

  // Load target data
  ICL::Target target(data_dir + "mcircles_11x15.yaml");

  // Load Calibration Images
  CalibrationImages cal_images;
  ROS_INFO_STREAM("Loading calibration images from: " << data_dir);
  getCalibrationImages(data_dir, cal_images);

  // Extract Observations
  ROS_INFO_STREAM("Extracting observations from data");
  ICL::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < cal_images.size(); i++)
  {
    cv::Mat grid_image;
    observation_extractor.extractObservation(cal_images[i], grid_image);
  }

  // Get Observations from extractor
  ICL::ObservationData observation_data = observation_extractor.getObservationData();

  // Split observations into sets
  std::vector<ICL::ObservationData> observation_sets;
  observation_sets.reserve(observation_data.size());
  for (std::size_t i = 0; i < observation_data.size(); i++)
  {
    ICL::ObservationData temp_observation_data;
    temp_observation_data.reserve(observation_data.size()-1);

    for (std::size_t j = 0; j < observation_data.size(); j++)
    {
      if (i != j)
      {
        temp_observation_data.push_back(observation_data[j]);
      }
    }
    observation_sets.push_back(temp_observation_data);
  }

  // Results
  std::vector<CalibrationResult> results;
  results.resize(observation_sets.size());

  // Run the calibration on each set
  #pragma omp parallel for
  for (std::size_t i = 0; i < observation_sets.size(); i++)
  {
    calibrateObservationSet(data_dir, observation_sets[i], 
      target, i, results);
  }

#ifdef RUN_THEORY
  std::string result_path = data_dir + "results/ic2_theory_results.csv";
#else
  std::string result_path = data_dir + "results/ic2_results.csv";
#endif
  ROS_INFO_STREAM("Saving Results to: " << result_path);
  saveResultData(result_path, results);

  return 0;
}