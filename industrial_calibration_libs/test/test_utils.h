#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#define TEST_CONSOLE_OUTPUT
#ifdef TEST_CONSOLE_OUTPUT
#define CONSOLE_OUTPUT(str) do { std::cerr << "[>>>>>>>>>>] " << str << '\n'; } while  (false)
#else
#define CONSOLE_OUTPUT(str) do { } while (false)
#endif

#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <string>
#include <gtest/gtest.h>

static void
printPoints(const std::vector<industrial_calibration_libs::Point3D> &points);

static void
printPoint3DVector(const std::vector<industrial_calibration_libs::Point3D> &points)
{
  for (std::size_t i = 0; i < points.size(); i++)
  {
    industrial_calibration_libs::Point3D point(points[i]);
    CONSOLE_OUTPUT(std::setprecision(4) << std::fixed << "Point: " 
      << i+1 << " x: " << point.x << " y: " << point.y 
      << " z:" << point.z);    
  }
}

#endif // TEST_UTILS_H
