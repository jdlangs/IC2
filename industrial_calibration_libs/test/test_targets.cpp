#include <test_utils.h>

TEST(Targets, load_target_yaml_with_points)
{
  industrial_calibration_libs::Target my_target;
  EXPECT_TRUE(my_target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5.yaml"));

  // Note(gChiou): Checking fields for data that matches the input target.
  EXPECT_EQ(my_target.getData().target_name, "mcircles_7x5");
  EXPECT_EQ(my_target.getData().target_type, 2);
  EXPECT_EQ(my_target.getData().target_rows, 7);
  EXPECT_EQ(my_target.getData().target_cols, 5);
  EXPECT_EQ(my_target.getData().target_points, 35);
  EXPECT_EQ(my_target.getData().circle_diameter, 0.015);
  EXPECT_EQ(my_target.getData().spacing, 0.03);
  EXPECT_EQ(my_target.getData().points.size(), 
    my_target.getData().target_rows*my_target.getData().target_cols);

  // Note(gChiou): Checking that the first and last point match.
  industrial_calibration_libs::Point3D first_point_actual(0.0000, 0.1800, 0.0000);
  industrial_calibration_libs::Point3D first_point_yaml(my_target.getData().points[0]);

  industrial_calibration_libs::Point3D second_point_actual(0.0300, 0.1800, 0.0000);
  industrial_calibration_libs::Point3D second_point_yaml(my_target.getData().points[1]);

  industrial_calibration_libs::Point3D second_to_last_point_actual(0.0900, 0.0000, 0.0000);
  industrial_calibration_libs::Point3D second_to_last_point_yaml(my_target.getData().points[my_target.getData().target_points-2]);

  industrial_calibration_libs::Point3D last_point_actual(0.120, 0.0000, 0.0000);
  industrial_calibration_libs::Point3D last_point_yaml(my_target.getData().points[my_target.getData().target_points-1]);

  // Note(gChiou): Prints out all points, leaving commented for debugging purposes
  #if 0
  for (std::size_t i = 0; i < my_target.getData().points.size(); i++)
  {
    industrial_calibration_libs::Point3D point(my_target.getData().points[i]);
    CONSOLE_OUTPUT(std::setprecision(4) << std::fixed << "Point: " << i+1 << " x: " << point.x << " y: " << point.y << " z:" << point.z);
  }
  #endif

  EXPECT_TRUE(first_point_actual == first_point_yaml);
  EXPECT_TRUE(second_point_actual == second_point_yaml);
  EXPECT_TRUE(second_to_last_point_actual == second_to_last_point_yaml);
  EXPECT_TRUE(last_point_actual == last_point_yaml);
}

TEST(Targets, load_target_yaml_without_points)
{
  industrial_calibration_libs::Target my_target;
  EXPECT_TRUE(my_target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5_np.yaml"));

  // Note(gChiou): Checking fields for data that matches the input target.
  EXPECT_EQ(my_target.getData().target_name, "mcircles_7x5");
  EXPECT_EQ(my_target.getData().target_type, 2);
  EXPECT_EQ(my_target.getData().target_rows, 7);
  EXPECT_EQ(my_target.getData().target_cols, 5);
  EXPECT_EQ(my_target.getData().target_points, 35);
  EXPECT_EQ(my_target.getData().circle_diameter, 0.015);
  EXPECT_EQ(my_target.getData().spacing, 0.03);
  EXPECT_EQ(my_target.getData().points.size(), 
    my_target.getData().target_rows*my_target.getData().target_cols);

  // Note(gChiou): Checking that the first and last point match.
  industrial_calibration_libs::Point3D first_point_actual(0.0000, 0.1800, 0.0000);
  industrial_calibration_libs::Point3D first_point_yaml(my_target.getData().points[0]);

  industrial_calibration_libs::Point3D second_point_actual(0.0300, 0.1800, 0.0000);
  industrial_calibration_libs::Point3D second_point_yaml(my_target.getData().points[1]);

  industrial_calibration_libs::Point3D second_to_last_point_actual(0.0900, 0.0000, 0.0000);
  industrial_calibration_libs::Point3D second_to_last_point_yaml(my_target.getData().points[my_target.getData().target_points-2]);

  industrial_calibration_libs::Point3D last_point_actual(0.120, 0.0000, 0.0000);
  industrial_calibration_libs::Point3D last_point_yaml(my_target.getData().points[my_target.getData().target_points-1]);  

  // Note(gChiou): Prints out all points, leaving commented for debugging purposes
  #if 0
  for (std::size_t i = 0; i < my_target.getData().points.size(); i++)
  {
    industrial_calibration_libs::Point3D point(my_target.getData().points[i]);
    CONSOLE_OUTPUT(std::setprecision(4) << std::fixed << "Point: " << i+1 << " x: " << point.x << " y: " << point.y << " z:" << point.z);
  }
  #endif

  EXPECT_TRUE(first_point_actual == first_point_yaml);
  EXPECT_TRUE(second_point_actual == second_point_yaml);
  EXPECT_TRUE(second_to_last_point_actual == second_to_last_point_yaml);
  EXPECT_TRUE(last_point_actual == last_point_yaml);  
}

TEST(Targets, load_target_yaml_compare)
{
  industrial_calibration_libs::Target full_target;
  EXPECT_TRUE(full_target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5.yaml"));

  industrial_calibration_libs::Target gen_target;
  EXPECT_TRUE(gen_target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5_np.yaml"));

  ASSERT_TRUE(full_target.getData().points.size() == gen_target.getData().points.size());

  for (std::size_t i = 0; i < full_target.getData().points.size(); i++)
  {
    EXPECT_EQ(full_target.getData().points[i].x, gen_target.getData().points[i].x);
    EXPECT_EQ(full_target.getData().points[i].y, gen_target.getData().points[i].y);
    EXPECT_EQ(full_target.getData().points[i].z, gen_target.getData().points[i].z);
  }  
}
