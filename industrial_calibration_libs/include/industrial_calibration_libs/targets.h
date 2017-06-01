#ifndef TARGETS_H
#define TARGETS_H

#include <yaml-cpp/yaml.h>
#include <cassert>
#include <vector>
#include <string>

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

  std::vector<double> asVector(void)
  {
    std::vector<double> points;
    points.resize(3);
    points[0] = x; points[1] = y; points[2] = z;
    return points;
  }

  double x;
  double y;
  double z;
};

struct TargetDefinition
{
  std::string target_name;
  std::size_t target_type;
  std::size_t num_rows;
  std::size_t num_cols;
  std::size_t num_points;

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

  bool checkForValidTarget(void);

  TargetDefinition target_params_;
};


} // namespace industrial_calibration_libs
#endif // TARGETS_H

