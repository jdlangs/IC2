#ifndef TARGETS_H
#define TARGETS_H

#include <industrial_calibration_libs/types.h>

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace industrial_calibration_libs
{

enum target_types
{
  Chessboard = 0,
  CircleGrid = 1,
  ModifiedCircleGrid = 2
};

struct TargetDefinition
{
  std::string target_name;
  std::size_t target_type;
  std::size_t target_rows;
  std::size_t target_cols;
  std::size_t target_points;

  double circle_diameter; // Meters
  double row_spacing; // Meters
  double col_spacing; // Meters
  double spacing; // Meters

  bool asymmetric_grid;
  std::vector<Point3D> points;
};

class Target
{
public:
  Target(void);

  bool loadTargetFromYAML(const std::string &yaml_file_path);

  bool loadTargetFromDefinition(const TargetDefinition &target_definition);

  TargetDefinition getData(void) const;

private:
  bool parseYAML(const YAML::Node &node, const std::string &var_name, 
    std::string &var_value);

  bool parseYAML(const YAML::Node &node, const std::string &var_name,
    std::size_t &var_value);

  bool parseYAML(const YAML::Node &node, const std::string &var_name,
    double &var_value);

  bool parseYAML(const YAML::Node &node, const std::string &var_name,
    bool &var_value);

  bool parseYAML(const YAML::Node &node, const std::string &var_name,
    Point3D &var_value);

  bool parseYAML(const YAML::Node &node, const std::string &var_name,
    std::vector<Point3D> &var_value);

  bool checkForValidTarget(void);

  bool populatePoints(std::size_t rows, std::size_t cols, double spacing, 
    std::vector<Point3D> &points);

  // Data Members
  TargetDefinition target_params_;
};


} // namespace industrial_calibration_libs
#endif // TARGETS_H

