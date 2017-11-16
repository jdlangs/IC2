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

// #define OUTPUT_EXTRINSICS
// #define VISUALIZE_RESULTS
#define SAVE_DATA

// Function Declarations
void calibrateDataSet(const std::string &data_dir, const std::string &data_set);

bool loadSeedExtrinsics(const std::string &data_path, std::size_t num_images,
  std::vector<ICL::Extrinsics> &extrinsics_seed);

bool saveResultdata(const std::string &result_path, double final_cost, 
    double intrinsics[9]);

void visualizeResults(const ICL::ResearchIntrinsic::Result &results,
  const ICL::Target &target, const CalibrationImages cal_images);

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
        out << YAML::Value << 9;
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

      // fx  = 537.1;
      // fy  = 536.1;
      // cx  = 325.5;
      // cy  = 231.9;
      // k1  = 0.0;
      // k2  = 0.0;
      // k3  = 0.0;
      // p1  = 0.0;
      // p2  = 0.0; 

      // fx  = 537.816;
      // fy  = 537.058;
      // cx  = 325.5556;
      // cy  = 231.885;
      // k1  = 0.0;
      // k2  = 0.0;
      // k3  = 0.0;
      // p1  = 0.0;
      // p2  = 0.0; 

#if 1
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
#endif
#if 0   
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

      double point_x = fx * xp + cx;
      double point_y = fy * yp + cy;

      cv::Point2d cv_point(point_x, point_y);
      observation_points.push_back(cv_point);
#endif       
    }

    cv::Mat result_image;
    drawResultPoints(cal_images[i], result_image, observation_points,
      target.getDefinition().target_rows, target.getDefinition().target_cols);
    cv::imshow("Result Image", result_image);
    cv::waitKey(0);
  }  
}

void calibrateDataSet(const std::string &data_dir, const std::string &data_set)
{
  std::string data_path = data_dir + data_set + "/";

  // Load Target Data
  ICL::Target target(data_path + "mcircles_9x12.yaml");

  // Load Calibration Images
  CalibrationImages cal_images;
  ROS_INFO_STREAM("Loading Calibration Images for Data Set: " << data_set);
  getCalibrationImages(data_path, cal_images);  

  // Extract Observations
  ROS_INFO_STREAM("Extracting Observations from Data Set: " << data_set);
  ICL::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < cal_images.size(); i++)
  {
    cv::Mat grid_image;
    observation_extractor.extractObservation(cal_images[i], grid_image);
  }

  // Get observations from extractor
  ICL::ObservationData observation_data = observation_extractor.getObservationData();

  // Seed parameters (Average of intrinsic values)
  double camera_info[9];
  // camera_info[0] = 570.34;
  // camera_info[1] = 570.34;
  // camera_info[2] = 319.5;
  // camera_info[3] = 239.5;  
  camera_info[0] = 537.1;
  camera_info[1] = 536.1;
  camera_info[2] = 325.5;
  camera_info[3] = 231.9;
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

  ICL::ResearchIntrinsic calibration(observation_data, target, params);
  
  calibration.setOutput(true); // Enable output to console.
  calibration.runCalibration();

  // Print out results.
  ICL::ResearchIntrinsic::Result results = calibration.getResults();

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

#ifdef VISUALIZE_RESULTS
  visualizeResults(results, target, cal_images);
#else
#endif

#ifdef SAVE_DATA
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

#ifdef OUTPUT_EXTRINSICS
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
  ros::init(argc, argv, "intrinsic_opencv");
  ros::NodeHandle pnh("~");
  
  std::string data_dir;
  pnh.getParam("data_dir", data_dir);
  data_dir = addSlashToEnd(data_dir);

  std::vector<std::string> data_sets = { "01", "02", "03", "04", "05", 
    "06", "07", "08", "09", "10", "11", "12", "13", "14", "15" };

  // std::vector<std::string> data_sets = { "01" };

  for (std::size_t i = 0; i < data_sets.size(); i++)
  {
    calibrateDataSet(data_dir, data_sets[i]);
  }

  return 0;
}