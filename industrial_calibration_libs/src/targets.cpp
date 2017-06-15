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

std::vector<double> Point3D::asVector(void)
{
  std::vector<double> points;
  points.resize(3);
  points[0] = x; points[1] = y; points[2] = z;
  return points;  
}

Target::Target(void) : target_params_(new TargetDefinition) { }

bool Target::loadTargetFromYAML(const std::string &yaml_file_path)
{
  YAML::Node target_yaml;
  try
  {
    target_yaml = YAML::LoadFile(yaml_file_path);
    if (!target_yaml["target_name"]) {return false;}
    // Note(gChiou): Each target yaml description should only contain
    // the definition for a single target. The files were structured like
    // the way they are now when I got them (meant to hold an array of targets.
    // I will probably redefine the structure later. Keeping the 0 in for now.
  }
  catch (YAML::BadFile &bf) {return false;}

  bool success = true;

  success &= parseYAML(target_yaml, "target_name", target_params_->target_name);
  success &= parseYAML(target_yaml, "target_type", target_params_->target_type);
  success &= parseYAML(target_yaml, "target_rows", target_params_->target_rows);
  success &= parseYAML(target_yaml, "target_cols", target_params_->target_cols);
  success &= parseYAML(target_yaml, "target_points", target_params_->target_points);

  switch (target_params_->target_type)
  {
    case Chessboard:
      success &= parseYAML(target_yaml, "row_spacing", target_params_->row_spacing);
      success &= parseYAML(target_yaml, "col_spacing", target_params_->col_spacing);
      // TODO(gChiou): Should we assume chessboard targets always have even spacing???
      break;
      
    case CircleGrid:
      success &= parseYAML(target_yaml, "circle_diameter", target_params_->circle_diameter);
      success &= parseYAML(target_yaml, "spacing", target_params_->spacing);
      // TODO(gChiou): Set this to false by default, check if it even exists.
      success &= parseYAML(target_yaml, "asymmetric_grid", target_params_->asymmetric_grid);
      break;

    case ModifiedCircleGrid:
      success &= parseYAML(target_yaml, "circle_diameter", target_params_->circle_diameter);
      success &= parseYAML(target_yaml, "spacing", target_params_->spacing);
      break;

    default:
      success = false;
      break;
  }

  if (!parseYAML(target_yaml, "points", target_params_->points))
  {
    // TODO(gChiou): Populate points from rows, cols, and spacing.
  }


  // TODO(gChiou): Better implementation of this
  // success &= checkForValidTarget();
  return success;
}

bool Target::loadTargetFromDefinition(const TargetDefinition &target_definition)
{
  // TODO(gChiou): ...
  return false;
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

// TODO(gChiou): Refactor this...
bool Target::checkForValidTarget(void)
{
  if (target_params_->target_type == CircleGrid)
  {
    // Note(gChiou): Check if total number of points is half of number of rows times number of columns for an asymmetric circle grid.
    if (target_params_->asymmetric_grid)
    {
      if (target_params_->target_points != (target_params_->target_rows*target_params_->target_cols) / 2)
      {
        return false;
      }
    }
    else
    {
      if (target_params_->target_points != (target_params_->target_rows*target_params_->target_cols))
      {
        return false;
      }
    }
  }
  return false; //TODO(gChiou): Remove this...
  // TODO(gChiou): Write these same checks for modified circle grid and chessboard
}

// TODO(gChiou): Implement this...
bool Target::populatePoints(void)
{
  return false;
}

std::shared_ptr<TargetDefinition> Target::getData(void)
{
  return target_params_;
}

} // namespace industrial_calibration_libs