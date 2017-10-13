#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui/highgui.hpp>
#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <industrial_calibration/helper_functions.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "extrinsic_example");
  ros::NodeHandle pnh("~");

  std::string data_path;
  pnh.getParam("data_path", data_path);

  // Load Target Data
  industrial_calibration_libs::Target target;
  target.loadTargetFromYAML(data_path + "mcircles_10x10/mcircles_10x10.yaml");

  // Load Calibration Images
  const std::size_t num_images = 15;
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = data_path + "mcircles_10x10/extrinsic/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i) + ".png";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
    calibration_images.push_back(image);    
  }

  // Extract Observations
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    cv::Mat output_image;
    observation_extractor.extractObservation(calibration_images[i],
      output_image);

    // Visualize the "corners"
    cv::imshow("grid", output_image);
    cv::waitKey(0);
  }

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData(); 

  // Load Link Data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp_link_data;
    loadLinkData(data_path + "mcircles_10x10/extrinsic/tf/" + std::to_string(i) + ".yaml",
      &temp_link_data, "base_link_to_tool0");
    link_data.push_back(temp_link_data);
  }

  for (std::size_t i = 0; i < link_data.size(); i++)
  {
    ROS_INFO_STREAM("Translation");
    for (std::size_t j = 0; j < link_data[i].translation.size(); j++)
    {
      ROS_INFO_STREAM(link_data[i].translation[j]);
    }
    ROS_INFO_STREAM("Quaternion");
    for (std::size_t j = 0; j < link_data[i].rotation_quat.size(); j++)
    {
      ROS_INFO_STREAM(link_data[i].rotation_quat[j]);
    }
  }

  for (std::size_t i = 0; i < target.getDefinition().points.size(); i++)
  {
    industrial_calibration_libs::Point3D point(target.getDefinition().points[i]);
    ROS_INFO_STREAM(std::setprecision(4) << std::fixed << "Point: " << i+1 << " x: " << point.x << " y: " << point.y << " z:" << point.z);
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

  // Set camera extrinsics seed
  industrial_calibration_libs::Pose6D link_6_to_camera_;
  link_6_to_camera_.setOrigin(0.0197, 0.0908, 0.112141);
  link_6_to_camera_.setAngleAxis(0.0, 0.0, -3.14/2.0);

  // Set target_to_base seed
  industrial_calibration_libs::Pose6D target_pose;
  target_pose.setOrigin(0.0, 0.0, 0.0);
  target_pose.setEulerZYX(0.0, 0.0, 0.0);

  // Initialize calibration params
  industrial_calibration_libs::CameraOnWristExtrinsicParams params;
  params.intrinsics = industrial_calibration_libs::IntrinsicsPartial(509.5179, 
    511.6581, 320.2695, 208.9545);
  params.tool_to_camera = industrial_calibration_libs::Extrinsics(link_6_to_camera_.getInverse());
  params.target_to_base = industrial_calibration_libs::Extrinsics(target_pose);
  convertToPose6D(link_data, &params.base_to_tool);

  // This draws the position of the dots based off of the uncalibrated position.
  {
    // const double* camera_angle_axis(&extrinsics[0]);
    // const double* camera_position(&extrinsics[3]);
    // const double* target_angle_axis(&target_to_base[0]);
    // const double* target_position(&target_to_base[3]);

    const double* camera_angle_axis(&params.tool_to_camera.data[0]);
    const double* camera_position(&params.tool_to_camera.data[3]);
    const double* target_angle_axis(&params.target_to_base.data[0]);
    const double* target_position(&params.target_to_base.data[3]);
 

    double fx = params.intrinsics.data[0];
    double fy = params.intrinsics.data[1];
    double cx = params.intrinsics.data[2];
    double cy = params.intrinsics.data[3];

    for (std::size_t i = 0; i < params.base_to_tool.size(); i++)
    {
      // if (i == 0 || i == 1 || i == 2 || i == 10 || i == 11 || i == 12) {continue;}
      industrial_calibration_libs::ObservationPoints observation_points;
      industrial_calibration_libs::Pose6D link_pose_inverse = params.base_to_tool[i].getInverse();

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

      cv::Mat result_image;
      drawResultPoints(calibration_images[i], result_image, observation_points,
        target.getDefinition().target_rows, target.getDefinition().target_cols);
      cv::imshow("Result Image", result_image);
      cv::waitKey(0);
    }
  }

  // Create calibration object and run calibration
  industrial_calibration_libs::CameraOnWristExtrinsic calibration(observation_data, target, params);
  calibration.setOutput(true);
  calibration.runCalibration();
  calibration.displayCovariance();

  // Print out the results.
  industrial_calibration_libs::CameraOnWristExtrinsic::Result results = calibration.getResults();
  ROS_INFO_STREAM("Extrinsic Parameters");
  ROS_INFO_STREAM("Translation x: " << results.extrinsics[3]);
  ROS_INFO_STREAM("Translation y: " << results.extrinsics[4]);
  ROS_INFO_STREAM("Translation z: " << results.extrinsics[5]);
  ROS_INFO_STREAM("Rotation x: " << results.extrinsics[0]);
  ROS_INFO_STREAM("Rotation y: " << results.extrinsics[1]);
  ROS_INFO_STREAM("Rotation z: " << results.extrinsics[2]);

  ROS_INFO_STREAM("Target to Base");
  ROS_INFO_STREAM("Translation x: " << results.target_to_base[3]);
  ROS_INFO_STREAM("Translation y: " << results.target_to_base[4]);
  ROS_INFO_STREAM("Translation z: " << results.target_to_base[5]);
  ROS_INFO_STREAM("Rotation x: " << results.target_to_base[0]);
  ROS_INFO_STREAM("Rotation y: " << results.target_to_base[1]);
  ROS_INFO_STREAM("Rotation z: " << results.target_to_base[2]);

  ROS_INFO_STREAM("Initial Cost: " << calibration.getInitialCost());
  ROS_INFO_STREAM("Final Cost: " << calibration.getFinalCost());

  // Draw the results back onto the image
  const double* camera_angle_axis(&results.extrinsics[0]);
  const double* camera_position(&results.extrinsics[3]);
  const double* target_angle_axis(&results.target_to_base[0]);
  const double* target_position(&results.target_to_base[3]);

  double fx = params.intrinsics.data[0];
  double fy = params.intrinsics.data[1];
  double cx = params.intrinsics.data[2];
  double cy = params.intrinsics.data[3];

  for (std::size_t i = 0; i < params.base_to_tool.size(); i++)
  {
    industrial_calibration_libs::ObservationPoints observation_points;
    industrial_calibration_libs::Pose6D link_pose_inverse = params.base_to_tool[i].getInverse();

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

    cv::Mat result_image;
    drawResultPoints(calibration_images[i], result_image, observation_points,
      target.getDefinition().target_rows, target.getDefinition().target_cols);
    cv::imshow("Result Image", result_image);
    cv::waitKey(0);
  }
}
