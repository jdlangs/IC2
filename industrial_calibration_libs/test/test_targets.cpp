#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <gtest/gtest.h>
#include "test_utils.h"

TEST(Targets, load_yaml)
{
  // Note(gChiou): The test runs this from the build/industrial_calibration_libs/ 
  // folder. Assuming this is being built from a catkin_ws, this relative path 
  // should take it directly to the targets directory.
  industrial_calibration_libs::Target my_target;
  EXPECT_TRUE(my_target.loadTargetFromYAML("../../src/IC2/Calibration Targets/mcircle_5x5.yaml"));

  // Note(gChiou): Checking fields for data that matches the input target.
  EXPECT_TRUE(my_target.getData()->target_name == "modified_circle_5x5");
  EXPECT_TRUE(my_target.getData()->target_type == 2);
  EXPECT_TRUE(my_target.getData()->target_rows == 5);
  EXPECT_TRUE(my_target.getData()->target_cols == 5);
  EXPECT_TRUE(my_target.getData()->target_points == 25);
  EXPECT_TRUE(my_target.getData()->circle_diameter == 0.0250);
  EXPECT_TRUE(my_target.getData()->spacing == 0.0350);

  // Note(gChiou): Checking that the first and last point match.
  industrial_calibration_libs::Point3D first_point_actual(0.0000, 0.1400, 0.0000);
  industrial_calibration_libs::Point3D first_point_yaml(my_target.getData()->points[0]);
  industrial_calibration_libs::Point3D last_point_actual(0.1750, 0.0000, 0.0000);
  industrial_calibration_libs::Point3D last_point_yaml(my_target.getData()->points[my_target.getData()->target_points-1]);
  for (std::size_t i = 0; i < first_point_actual.asVector().size(); i++)
  {
    CONSOLE_OUTPUT("I: " << i << " " << "Actual:" << first_point_actual.asVector()[i] << " | " << "YAML: " << first_point_yaml.asVector()[i]);
    // CONSOLE_OUTPUT("I: " << i << " " << "Actual:" << last_point_actual.asVector()[i] << " | " << "YAML: " << last_point_yaml.asVector()[i]);
    // EXPECT_TRUE(first_point_actual.asVector()[i] == first_point_yaml.asVector()[i]);
    // EXPECT_TRUE(last_point_actual.asVector()[i] == last_point_yaml.asVector()[i]);
  }
  for (std::size_t i = 0; i < first_point_actual.asVector().size(); i++)
  {
    // CONSOLE_OUTPUT("I: " << i << " " << "Actual:" << first_point_actual.asVector()[i] << " | " << "YAML: " << first_point_yaml.asVector()[i]);
    CONSOLE_OUTPUT("I: " << i << " " << "Actual:" << last_point_actual.asVector()[i] << " | " << "YAML: " << last_point_yaml.asVector()[i]);
    // EXPECT_TRUE(first_point_actual.asVector()[i] == first_point_yaml.asVector()[i]);
    // EXPECT_TRUE(last_point_actual.asVector()[i] == last_point_yaml.asVector()[i]);
  }  
  // BUG HERE, FIX THIS!!!
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}