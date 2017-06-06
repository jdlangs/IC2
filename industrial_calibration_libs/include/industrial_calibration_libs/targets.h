#ifndef TARGETS_H
#define TARGETS_H

#include <yaml-cpp/yaml.h>
#include <cassert>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace industrial_calibration_libs
{

enum target_types
{
  ChessBoard = 0,
  CircleGrid = 1,
  ModifiedCircleGrid = 2
};

struct Point3D
{
  Point3D(void);

  Point3D(double x_in, double y_in, double z_in);
  
  Point3D(const std::vector<double> &points);

  void setPoints(double x_in, double y_in, double z_in);

  bool setPoints(const std::vector<double> &points);

  std::vector<double> asVector(void);

  double x;
  double y;
  double z;
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

  std::shared_ptr<TargetDefinition> getData(void);

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

  bool populatePoints(void);

  // Variables
  std::shared_ptr<TargetDefinition> target_params_;
};


} // namespace industrial_calibration_libs
#endif // TARGETS_H

