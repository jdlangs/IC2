/*
  This file will simulate data using the  
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
#define RUN_ALL true
#define OUTPUT_EXTRINSICS false
#define SAVE_DATA true

// Function Declarations
bool saveResultdata(const std::string &result_path, double final_cost, 
    double intrinsics[9]);

void calibrateSimDataSet(const std::string &data_dir, const std::string &data_set);

bool loadSeedExtrinsics(const std::string &data_path, std::size_t num_images,
  std::vector<ICL::Extrinsics> &extrinsics_seed);

// Function Implementatins
bool saveResultdata(const std::string &result_path, double final_cost, 
    double intrinsics[9])
{
  std::vector<double> camera_matrix = { intrinsics[0], 0, intrinsics[2],
    0, intrinsics[1], intrinsics[3], 0, 0, 1 };
  // Mine are: k1, k2, k3, p1, p2
  // Output as: k1, k2, p1, p2, k3
  std::vector<double> dist_coeffs = { intrinsics[4], intrinsics[5],
    intrinsics[7], intrinsics[8], intrinsics[6] };

  YAML::Emitter out;

  out << YAML::BeginMap;

  out << YAML::Key << "camera_matrix";
    out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "rows";
        out << YAML::Value << 3;
      out << YAML::Key << "cols";
        out << YAML::Value << 3;
      out << YAML::Key << "data";
        out << YAML::Value << YAML::Flow << camera_matrix;
    out << YAML::EndMap;

  out << YAML::Key << "distortion_coefficients";
    out << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "rows";
        out << YAML::Value << 1;
      out << YAML::Key << "cols";
        out << YAML::Value << 5;
      out << YAML::Key << "data";
        out << YAML::Value << YAML::Flow << dist_coeffs;
    out << YAML::EndMap;

  out << YAML::Key << "reprojection_error";
    out << YAML::Value << final_cost;

  out << YAML::EndMap;

  // Write to yaml file
  if (out.good())
  {
    std::ofstream yaml_file(result_path);
    if (yaml_file)
    {
      yaml_file << out.c_str();
      yaml_file.close();
      return true;
    }
    if (yaml_file.bad())
    {
      return false;
    }
  }

  return false;
}

bool loadSeedExtrinsics(const std::string &data_path, std::size_t num_images,
  std::vector<ICL::Extrinsics> &extrinsics_seed)
{
  extrinsics_seed.reserve(num_images);

  YAML::Node extrinsics_seed_yaml;
  std::string path = data_path + "estimated_poses.yaml";

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

void calibrateSimDataSet(const std::string &data_dir, const std::string &data_set)
{
  std::string data_path = data_dir + data_set + "/";

  // We are going to use the original target for now.
  // TODO(gChiou): Make a fake target that can fill the FoV
  ICL::Target target(data_path + "mcircles_9x12.yaml");

  // I'm lazy, so we are going to load all the images just so I know how
  // many poses there are...
  CalibrationImages cal_images;
  ROS_INFO_STREAM("Loading Calibration Images for Data Set: " << data_set);
  getCalibrationImages(data_path, cal_images);    

  // Load Poses
  std::vector<ICL::Extrinsics> poses;
  if (!loadSeedExtrinsics(data_path, cal_images.size(), poses))
    ROS_ERROR_STREAM("Failed to load poses");

  // My pretend intrinsics, this should probably be loaded in somewhere else
  double fake_intrinsics[9];
  fake_intrinsics[0] = 570.542;  /** focal length x  */
  fake_intrinsics[1] = 570.531;  /** focal length y  */
  fake_intrinsics[2] = 325.451;  /** central point x */
  fake_intrinsics[3] = 352.217;  /** central point y */
  fake_intrinsics[4] = 1.5241;   /** distortion k1   */
  fake_intrinsics[5] = -0.2341;  /** distortion k2   */
  fake_intrinsics[6] = 0.3123;    /** distortion k3   */
  fake_intrinsics[7] = -.3584;   /** distortion p1   */
  fake_intrinsics[8] = 0.2511;    /** distortion p2   */   

  // Create "fake" observations from poses
  ICL::ObservationData observation_data;
  observation_data.resize(poses.size());

  // For every image
  for (std::size_t i = 0; i < poses.size(); i++)
  {
    // Get the current pose
    ICL::Pose6D target_to_camera_pose6D = poses[i].asPose6D();
    double target_to_camera[6];

    target_to_camera[0] = target_to_camera_pose6D.ax;
    target_to_camera[1] = target_to_camera_pose6D.ay;
    target_to_camera[2] = target_to_camera_pose6D.az;
    target_to_camera[3] = target_to_camera_pose6D.x;
    target_to_camera[4] = target_to_camera_pose6D.y;
    target_to_camera[5] = target_to_camera_pose6D.z;

    const double* target_angle_axis(&target_to_camera[0]);
    const double* target_position(&target_to_camera[3]);

    // For every point on the target
    for (std::size_t j = 0; j < target.getDefinition().points.size(); j++)
    {
      // Project the point into the image plane using the fake intrinsics
      double camera_point[3];
      ICL::Point3D target_point(target.getDefinition().points[j]);

      ICL::transformPoint3D(target_angle_axis, target_position,
        target_point.asVector(), camera_point);

      // Apply the intrinsics
      double fx, fy, cx, cy, k1, k2, k3, p1, p2;
      fx = fake_intrinsics[0]; /** focal length x  */ 
      fy = fake_intrinsics[1]; /** focal length y  */
      cx = fake_intrinsics[2]; /** central point x */
      cy = fake_intrinsics[3]; /** central point y */
      k1 = fake_intrinsics[4]; /** distortion k1   */
      k2 = fake_intrinsics[5]; /** distortion k2   */
      k3 = fake_intrinsics[6]; /** distortion k3   */
      p1 = fake_intrinsics[7]; /** distortion p1   */
      p2 = fake_intrinsics[8]; /** distortion p2   */

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

      cv::Point2d observation_point(point_x, point_y);
      observation_data[i].push_back(observation_point);
    }
  }

#if 01
  // Seed parameters (Average of intrinsic values)
  double camera_info[9];
  camera_info[0] = 0.0;
  camera_info[1] = 0.0;
  camera_info[2] = 0.0;
  camera_info[3] = 0.0;
  camera_info[4] = 0.0;
  camera_info[5] = 0.0;
  camera_info[6] = 0.0;
  camera_info[7] = 0.0;
  camera_info[8] = 0.0;

  ROS_INFO_STREAM("Running Calibration for Data Set: " << data_set);
  ICL::ResearchIntrinsicParams params;
  params.intrinsics = ICL::IntrinsicsFull(camera_info);

  // Get seed extrinsics
  params.target_to_camera_seed;
  if (!loadSeedExtrinsics(data_path, cal_images.size(), params.target_to_camera_seed))
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

#if SAVE_DATA
  std::string result_path = data_dir + "results/ic2_" + data_set + ".yaml";  
  if (saveResultdata(result_path, calibration.getFinalCost(), 
    results.intrinsics))
  {
    ROS_INFO_STREAM("Data Set: " << data_set << " Results Saved To: " <<
      result_path);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to Save Results for Data Set: " << data_set);
  }
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
#endif
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intrinsic_ic2_sim");
  ros::NodeHandle pnh("~");
  
  std::string data_dir;
  pnh.getParam("data_dir", data_dir);
  data_dir = addSlashToEnd(data_dir);

#if RUN_ALL
  std::vector<std::string> data_sets = { "01", "02", "03", "04", "05", 
    "06", "07", "08", "09", "10", "11", "12", "13", "14", "15" };
#else
  std::vector<std::string> data_sets = { "01" };
#endif

  for (std::size_t i = 0; i < data_sets.size(); i++)
  {
    calibrateSimDataSet(data_dir, data_sets[i]);
  }

  return 0;
}
