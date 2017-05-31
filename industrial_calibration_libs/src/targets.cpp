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
  YAML::Node target_yaml = YAML::LoadFile(yaml_file_path);
}

bool Target::loadTargetFromDefinition(const TargetDefinition &target_definition)
{

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

} // namespace industrial_calibration_libs