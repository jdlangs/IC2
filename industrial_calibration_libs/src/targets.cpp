#include <industrial_calibration_libs/targets.h>

namespace industrial_calibration_libs
{

Point3D::Point3D(void) { }

Point3D::Point3D(double x_in, double y_in, double z_in) : 
  x(x_in), y(y_in), z(z_in) { }

Point3D::Point3D(const std::vector<double> &points)
{
  assert(points.size() == 3);
  x = points[0];
  y = points[1];
  z = points[2];  
}

void Point3D::setPoints(double x_in, double y_in, double z_in)
{
  x = x_in;
  y = y_in;
  z = z_in;
}

bool Point3D::setPoints(const std::vector<double> &points)
{
  if (points.size() != 3) {return false;}
  else
  {
    x = points[0];
    y = points[1];
    z = points[2];
    return true;
  }
}

Target::Target(void) { }

bool Target::loadTargetFromYAML(const std::string &yaml_file_path)
{
  try
  {
    YAML::Node target_yaml = YAML::LoadFile(yaml_file_path);
  }
  catch (YAML::BadFile &bf) {return false;}

  bool success = true;

  success &= parseYAML(target_yaml, "target_name", target_params_.target_name);
  success &= parseYAML(target_yaml, "target_type", target_params_.target_type);
  success &= parseYAML(target_yaml, "target_rows", target_params_.target_rows);
  success &= parseYAML(target_yaml, "target_cols", target_params_.target_cols);
  success &= parseYAML(target_yaml, "target_points", target_params_.target_points);

  switch (target_params_.target_type)
  {
    case ChessBoard:
      success &= parseYAML(target_yaml, "row_spacing", target_params_.row_spacing);
      success &= parseYAML(target_yaml, "col_spacing", target_params_.col_spacing);
      // Should we assume chessboard targets always have even spacing???
      break;
    case CircleGrid:
      success &= parseYAML(target_yaml, "circle_diameter", target_params_.circle_diameter);
      success &= parseYAML(target_yaml, "spacing", target_params_.spacing);
      success &= parseYAML(target_yaml, "asymmetric_grid", target_params_.asymmetric_grid);
      break;
    case ModifiedCircleGrid:
      success &= parseYAML(target_yaml, "circle_diameter", target_params_.circle_diameter);
      success &= parseYAML(target_yaml, "spacing", target_params_.spacing);
      break;
    default:
      success = false;
      break;
  }

  success &= checkForValidTarget(void);

  return success;
}

bool Target::loadTargetFromDefinition(const TargetDefinition &target_definition)
{
  // TODO: Geoffrey
}

bool Target::parseYAML(const YAML::Node &node, const std::string &var_name,
  std::string &var_value)
{
  if (node[var_name])
  {
    var_value = node[var_name].as<std::string>();
    return true;
  }
  else {return false;}
}

bool Target::parseYAML(const YAML::Node &node, const std::string &var_name,
  std::size_t &var_value)
{
  if (node[var_name])
  {
    var_value = node[var_name].as<std::size_t>();
    return true;
  }
  else {return false;}
}

bool Target::parseYAML(const YAML::Node &node, const std::string &var_name,
  double &var_value)
{
  if (node[var_name])
  {
    var_value = node[var_name].as<double>();
    return true;
  }
  else {return false;}
}

bool Target::parseYAML(const YAML::Node &node, const std::string &var_name,
  bool &var_value)
{
  if (node[var_name])
  {
    var_value = node[var_name].as<bool>();
    return true;
  }
  else {return false;}
}

bool Target::parseYAML(const YAML::Node &node, const std::string &var_name,
  Point3D &var_value)
{
  std::vector<double> points;
  if (node[var_name])
  {
    const YAML::Node n = node[var_name];
    for (std::size_t i = 0; i < n.size(); i++)
    {
      double value = n[i].as<double>();
      points.push_back(value);
    }
    if (var_value.setPoints(points)) {return true;}
    else {return false;}
  }
  else {return false;}
}

bool Target::checkForValidTarget(void)
{
  if (target_params_.target_type == CircleGrid)
  {
    // Check if 
    if (target_params_.asymmetric_grid)
    {
      if (target_params_.num_points != (target_params_.num_rows*target_params_.num_cols) / 2)
      {
        return false;
      }
    }
    else
    {
      if (target_params_.num_points != (target_params_.num_rows*target_params_.num_cols))
      {
        return false;
      }
    }
  }  
}

} // namespace industrial_calibration_libs