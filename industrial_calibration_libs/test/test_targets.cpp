#include <test_utils.h>

/*
  This test loads a target from a target yaml file.
*/

TEST(Targets, load_target_from_yaml)
{
  // Create a target object.
  industrial_calibration_libs::Target target("mcircles_10x10/mcircles_10x10.yaml");;

  // Note(gChiou): Checking fields for data that matches the input target.
  EXPECT_EQ(target.getDefinition().target_name, "mcircles_10x10");
  EXPECT_EQ(target.getDefinition().target_type, 2);
  EXPECT_EQ(target.getDefinition().target_rows, 10);
  EXPECT_EQ(target.getDefinition().target_cols, 10);
  EXPECT_EQ(target.getDefinition().circle_diameter, 0.01);
  EXPECT_EQ(target.getDefinition().spacing, 0.0248);

  #if 0
    // Note(gChiou): Prints out all points, leaving commented for debugging purposes
    printPoint3DVector(my_target.getDefinition().points);
  #endif
}

/*
  This test loads a target from a user defined "definition", and compares
  it to a test loaded from a yaml file.
*/

TEST(Targets, load_target_from_definition)
{
  // Create a target definition object
  industrial_calibration_libs::TargetDefinition target_definition;

  target_definition.target_name = "mcircles_10x10";
  target_definition.target_type = industrial_calibration_libs::ModifiedCircleGrid;
  target_definition.target_rows = 10;
  target_definition.target_cols = 10;
  target_definition.circle_diameter = 0.01;
  target_definition.spacing = 0.0248;

  // Create a target object from definition.
  industrial_calibration_libs::Target definition_target(target_definition);

  // Create a target object file.
  industrial_calibration_libs::Target yaml_target("mcircles_10x10/mcircles_10x10.yaml");;

  // Compare...
  EXPECT_EQ(definition_target.getDefinition().target_name, 
    yaml_target.getDefinition().target_name);
  EXPECT_EQ(definition_target.getDefinition().target_type, 
    yaml_target.getDefinition().target_type);
  EXPECT_EQ(definition_target.getDefinition().target_rows, 
    yaml_target.getDefinition().target_rows);
  EXPECT_EQ(definition_target.getDefinition().target_cols, 
    yaml_target.getDefinition().target_cols);
  EXPECT_EQ(definition_target.getDefinition().circle_diameter, 
    yaml_target.getDefinition().circle_diameter);
  EXPECT_EQ(definition_target.getDefinition().spacing, 
    yaml_target.getDefinition().spacing);
}
