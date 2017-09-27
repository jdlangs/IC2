#include <industrial_calibration_libs/targets.h>

namespace industrial_calibration_libs
{
Target::Target(const std::string &yaml_file_path) 
{
  if (!this->loadTargetFromYAML(yaml_file_path))
  {
    throw std::runtime_error("Failed to load target from file: " + yaml_file_path);
  }
}

Target::Target(const TargetDefinition &target_definition)
{
  if (!this->loadTargetFromDefinition(target_definition))
  {
    throw std::runtime_error("Failed to load target from definition");
  }
}

bool Target::loadTargetFromYAML(const std::string &yaml_file_path)
{
  YAML::Node target_yaml;
  try
  {
    target_yaml = YAML::LoadFile(yaml_file_path);
    if (!target_yaml["target_name"]) {return false;}
  }
  catch (YAML::BadFile &bf) {return false;}

  bool success = true;

  success &= parseYAML(target_yaml, "target_name", target_definition_.target_name);
  success &= parseYAML(target_yaml, "target_type", target_definition_.target_type);
  success &= parseYAML(target_yaml, "target_rows", target_definition_.target_rows);
  success &= parseYAML(target_yaml, "target_cols", target_definition_.target_cols);
  success &= parseYAML(target_yaml, "target_points", 
    target_definition_.target_points);

  switch (target_definition_.target_type)
  {
    case Chessboard:
      // success &= parseYAML(target_yaml, "row_spacing", target_definition_.row_spacing);
      // success &= parseYAML(target_yaml, "col_spacing", target_definition_.col_spacing);
      std::cerr << "Chessboard targets are not currently supported" << std::endl;
      return false;
      break;
      
    case CircleGrid:
      // success &= parseYAML(target_yaml, "circle_diameter", target_definition_.circle_diameter);
      // success &= parseYAML(target_yaml, "spacing", target_definition_.spacing);
      // TODO(gChiou): Set this to false by default, check if it even exists.
      // success &= parseYAML(target_yaml, "asymmetric_grid", target_definition_.asymmetric_grid);
      std::cerr << "Circlegrid targets are not currently supported" << std::endl;
      return false;
      break;

    case ModifiedCircleGrid:
      success &= parseYAML(target_yaml, "circle_diameter", target_definition_.circle_diameter);
      success &= parseYAML(target_yaml, "spacing", target_definition_.spacing);
      break;

    default:
      success = false;
      break;
  }

  if (!parseYAML(target_yaml, "points", target_definition_.points))
  {
    this->populatePoints(target_definition_.target_rows, 
      target_definition_.target_cols, target_definition_.spacing, 
      target_definition_.points);
  }

  success &= checkForValidTarget();
  return success;
}

bool Target::loadTargetFromDefinition(const TargetDefinition &target_definition)
{
  target_definition_ = target_definition;
  this->populatePoints(target_definition_.target_rows, 
    target_definition_.target_cols, target_definition_.spacing, 
    target_definition_.points);

  return checkForValidTarget();
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

// Note(gChiou): This is unused for now, remove later.
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

bool Target::parseYAML(const YAML::Node &node, const std::string &var_name,
  std::vector<Point3D> &var_value)
{
  var_value.clear();
  if (node[var_name])
  {
    const YAML::Node n = node[var_name];
    for (std::size_t i = 0; i < n.size(); i++)
    {
      Point3D point;
      std::vector<double> temp_point;
      for (std::size_t j = 0; j < n[i].size(); j++)
      {
        double value = n[i][j].as<double>();
        temp_point.push_back(value);
      }
      if (!point.setPoints(temp_point)) {return false;}
      var_value.push_back(point);
    }
    return true;
  }
  else {return false;}
}

bool Target::checkForValidTarget(void)
{
  // Don't feel like typing out target_definition_
  TargetDefinition target = this->target_definition_;

  if (target.target_type == ModifiedCircleGrid)
  {
    if (target.target_rows < 2 || target.target_cols < 2)
    {
      return false;
    }
    if ((target.target_rows * target.target_cols) != target.target_points)
    {
      return false;
    }
    if ((target.target_rows * target.target_cols) != target.points.size())
    {
      return false;
    }
    if (target.spacing <= 0.0)
    {
      return false;
    }
    // These values shouldn't be used for now...
    if (target.row_spacing > 0.0 || target.col_spacing > 0.0)
    {
      return false;
    }
  }
  else {return false;} // If target type is not modified circle grid.
  return true; // As long as it doesn't fail, it should return true. 
}

bool Target::populatePoints(std::size_t rows, std::size_t cols, double spacing, 
    std::vector<Point3D> &points)
{
  points.reserve(rows*cols);

  for (std::size_t i = 1; i < (rows+1); i++)
  {
    double y = (rows-i)*spacing;
    for (std::size_t j = 0; j < cols; j++)
    {
      double x = j*spacing;
      Point3D point(x, y, 0.000);
      points.push_back(point);
    }
  }

  // TODO(gChiou): May need a better check...
  if (points.size() == (rows*cols)) {return true;}
  else {return false;}
}

TargetDefinition Target::getDefinition(void) const
{
  return target_definition_;
}

} // namespace industrial_calibration_libs