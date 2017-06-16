#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <gtest/gtest.h>
#include "test_utils.h"

TEST(Targets, load_yaml)
{
  // Note(gChiou): The test runs this from the build/industrial_calibration_libs/ 
  // folder. Assuming this is being built from a catkin_ws, this relative path 
  // should take it directly to the targets directory.
  industrial_calibration_libs::Target my_target;
  EXPECT_TRUE(my_target.loadTargetFromYAML("cal_targets/mcircles_7x5.yaml"));

  // Note(gChiou): Checking fields for data that matches the input target.
  EXPECT_TRUE(my_target.getData()->target_name == "mcircles_7x5");
  EXPECT_TRUE(my_target.getData()->target_type == 2);
  EXPECT_TRUE(my_target.getData()->target_rows == 7);
  EXPECT_TRUE(my_target.getData()->target_cols == 5);
  EXPECT_TRUE(my_target.getData()->target_points == 35);
  EXPECT_TRUE(my_target.getData()->circle_diameter == 0.015);
  EXPECT_TRUE(my_target.getData()->spacing == 0.03);

  // Note(gChiou): Checking that the first and last point match.
  industrial_calibration_libs::Point3D first_point_actual(0.0000, 0.1800, 0.0000);
  industrial_calibration_libs::Point3D first_point_yaml(my_target.getData()->points[0]);

  industrial_calibration_libs::Point3D second_point_actual(0.0300, 0.1800, 0.0000);
  industrial_calibration_libs::Point3D second_point_yaml(my_target.getData()->points[1]);

  industrial_calibration_libs::Point3D second_to_last_point_actual(0.0900, 0.0000, 0.0000);
  industrial_calibration_libs::Point3D second_to_last_point_yaml(my_target.getData()->points[my_target.getData()->target_points-2]);

  industrial_calibration_libs::Point3D last_point_actual(0.120, 0.0000, 0.0000);
  industrial_calibration_libs::Point3D last_point_yaml(my_target.getData()->points[my_target.getData()->target_points-1]);

  // Note(gChiou): Prints out all points, leaving commented for debugging purposes
  // for (std::size_t i = 0; i < my_target.getData()->points.size(); i++)
  // {
  //   industrial_calibration_libs::Point3D point(my_target.getData()->points[i]);
  //   CONSOLE_OUTPUT(std::setprecision(4) << std::fixed << "Point: " << i+1 << " x: " << point.x << " y: " << point.y << " z:" << point.z);
  // }

  EXPECT_TRUE(first_point_actual == first_point_yaml);
  EXPECT_TRUE(second_point_actual == second_point_yaml);
  EXPECT_TRUE(second_to_last_point_actual == second_to_last_point_yaml);
  EXPECT_TRUE(last_point_actual == last_point_yaml);
}

