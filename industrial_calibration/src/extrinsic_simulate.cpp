/*
  The goal of this program is to simulate a "calibration" by generating 
  the observations with known transform between tool and camera_frame. 
  The solver will be seeded with a slightly incorrect solution, 
  which should theoretically converge to the correct solution.

  The observations will be generated using existing transforms between 
  the base_to_tool, target_to_base, and a known transform between the 
  tool and camera_frame.
*/

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <industrial_calibration/helper_functions.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "extrinsic_simulate");
  ros::NodeHandle pnh("~");
  std::string data_path;
  pnh.getParam("data_path", data_path);
  std::size_t num_images = 11;

  // Load target data
  industrial_calibration_libs::Target target;
  target.loadTargetFromYAML(data_path + "mcircles_10x10/mcircles_10x10.yaml");

  // Load link data
  std::vector<industrial_calibration_libs::Pose6D> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    industrial_calibration_libs::Pose6D pose;
    loadLinkData(i, data_path + "mcircles_10x10/extrinsic/tf/", pose);
    link_data.push_back(pose);
  }

  for (std::size_t i = 0; i < link_data.size(); i++)
  {
    ROS_INFO_STREAM("Translation");
    ROS_INFO_STREAM(link_data[i].x);
    ROS_INFO_STREAM(link_data[i].y);
    ROS_INFO_STREAM(link_data[i].z);
    ROS_INFO_STREAM("Rotation (Angle Axis");
    ROS_INFO_STREAM(link_data[i].ax);
    ROS_INFO_STREAM(link_data[i].ay);
    ROS_INFO_STREAM(link_data[i].az);
  }

  /*
    Define some constants
  */
  // Intrinsic parameters of the camera
  double fx = 570.0;
  double fy = 570.0;
  double cx = 320.0;
  double cy = 240.0;

  // Extrinsic transform between tool0 and camera_optical_frame
  industrial_calibration_libs::Pose6D tool_to_camera;
  tool_to_camera.setOrigin(0.02, 0.10, 0.15);
  tool_to_camera.setAngleAxis(0.0, 0.0, 0.0);  

  double extrinsics[6];
  extrinsics[0] = tool_to_camera.ax;
  extrinsics[1] = tool_to_camera.ay;
  extrinsics[2] = tool_to_camera.az;
  extrinsics[3] = tool_to_camera.x;
  extrinsics[4] = tool_to_camera.y;
  extrinsics[5] = tool_to_camera.z;

  // Extrinsic transform between target to base of robot
  double target_to_base[6] = {0};
  target_to_base[3] = 0.75;

  const double* camera_angle_axis(&extrinsics[0]);
  const double* camera_position(&extrinsics[3]);
  const double* target_angle_axis(&target_to_base[0]);
  const double* target_position(&target_to_base[3]);

  // Generate "fake" observation data
  industrial_calibration_libs::ObservationData observation_data;
  for (std::size_t i = 0; i < link_data.size(); i++)
  {
    industrial_calibration_libs::ObservationPoints observation_points;
    industrial_calibration_libs::Pose6D link_pose_inverse = link_data[i].getInverse();

    for (std::size_t j = 0; j < target.getDefinition().points.size(); j++)
    {
      industrial_calibration_libs::Point3D target_point(target.getDefinition().points[j]);
 
      double world_point[3];
      double link_point[3];
      double camera_point[3];

      industrial_calibration_libs::transformPoint3D(target_angle_axis, target_position,
        target_point.asVector(), world_point);
      industrial_calibration_libs::poseTransformPoint(link_pose_inverse, world_point, 
        link_point);
      industrial_calibration_libs::transformPoint(camera_angle_axis, camera_position, 
        link_point, camera_point);

      double xp1 = camera_point[0];
      double yp1 = camera_point[1];
      double zp1 = camera_point[2];

      double xp, yp;
      if (zp1 == 0.0)
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
    }
    observation_data.push_back(observation_points);
  }

  ROS_INFO_STREAM("Total Observations: " << observation_data.size());
  for (std::size_t i = 0; i < observation_data.size(); i++)
  {
    ROS_INFO_STREAM("Observations for Image " << i << " Size: " << observation_data[i].size());
    ROS_INFO_STREAM("Observations for Image " << i << " Points:");
    for (std::size_t j = 0; j < observation_data[i].size(); j++)
    {
      ROS_INFO_STREAM(observation_data[i][j]);
    }
  }

  // Generating fake seed and add to params
  industrial_calibration_libs::CameraOnWristExtrinsicParams params;
  params.intrinsics = industrial_calibration_libs::IntrinsicsPartial(570.0,
    570.0, 320.0, 240.0);
  
  // Fake camera extrinsics seed data
  industrial_calibration_libs::Pose6D fake_tool_to_camera;
  fake_tool_to_camera.setOrigin(0.097, 0.1508, 0.112141);
  fake_tool_to_camera.setAngleAxis(0.0, 0.0, 0.0);  
  params.tool_to_camera = industrial_calibration_libs::Extrinsics(fake_tool_to_camera);
  params.target_to_base = industrial_calibration_libs::Extrinsics(0,0,0,0,0,0);
  params.base_to_tool = link_data;

  // Run a calibration
  industrial_calibration_libs::CameraOnWristExtrinsic calibration(observation_data,
    target, params);
  calibration.setOutput(true);
  calibration.runCalibration();
  calibration.displayCovariance();

  // Get results and print
  industrial_calibration_libs::CameraOnWristExtrinsic::Result results = calibration.getResults();
  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("Tool0 to camera_optical_frame");
  ROS_INFO_STREAM("Translation x: " << results.extrinsics[3]);
  ROS_INFO_STREAM("Translation y: " << results.extrinsics[4]);
  ROS_INFO_STREAM("Translation z: " << results.extrinsics[5]);
  ROS_INFO_STREAM("Rotation x: " << results.extrinsics[0]);
  ROS_INFO_STREAM("Rotation y: " << results.extrinsics[1]);
  ROS_INFO_STREAM("Rotation z: " << results.extrinsics[2]);

  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("Target to Base");
  ROS_INFO_STREAM("Translation x: " << results.target_to_base[3]);
  ROS_INFO_STREAM("Translation y: " << results.target_to_base[4]);
  ROS_INFO_STREAM("Translation z: " << results.target_to_base[5]);
  ROS_INFO_STREAM("Rotation x: " << results.target_to_base[0]);
  ROS_INFO_STREAM("Rotation y: " << results.target_to_base[1]);
  ROS_INFO_STREAM("Rotation z: " << results.target_to_base[2]);

  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("Initial Cost: " << calibration.getInitialCost());
  ROS_INFO_STREAM("Final Cost: " << calibration.getFinalCost());

  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("Result Comparison:");
  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("Tool0 to camera_optical_frame");
  ROS_INFO_STREAM("Expected Tx: " << 0.02 << " | Result Tx: " << results.extrinsics[3]);
  ROS_INFO_STREAM("Expected Ty: " << 0.10 << " | Result Ty: " << results.extrinsics[4]);
  ROS_INFO_STREAM("Expected Tz: " << 0.15 << " | Result Tz: " << results.extrinsics[5]);
  ROS_INFO_STREAM("Expected Rx: " << 0.00 << " | Result Rx: " << results.extrinsics[0]);
  ROS_INFO_STREAM("Expected Ry: " << 0.00 << " | Result Ry: " << results.extrinsics[1]);
  ROS_INFO_STREAM("Expected Rz: " << 0.00 << " | Result Rz: " << results.extrinsics[2]);

  ROS_INFO_STREAM("");
  ROS_INFO_STREAM("Target to Base");
  ROS_INFO_STREAM("Expected Tx: " << 0.75 << " | Result Tx: " << results.target_to_base[3]);
  ROS_INFO_STREAM("Expected Ty: " << 0.00 << " | Result Ty: " << results.target_to_base[4]);
  ROS_INFO_STREAM("Expected Tz: " << 0.00 << " | Result Tz: " << results.target_to_base[5]);
  ROS_INFO_STREAM("Expected Rx: " << 0.00 << " | Result Rx: " << results.target_to_base[0]);
  ROS_INFO_STREAM("Expected Ry: " << 0.00 << " | Result Ry: " << results.target_to_base[1]);
  ROS_INFO_STREAM("Expected Rz: " << 0.00 << " | Result Rz: " << results.target_to_base[2]);
}
