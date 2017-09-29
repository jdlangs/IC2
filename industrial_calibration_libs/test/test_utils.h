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
showImage(const cv::Mat &image, const std::string &window_name);

static void
printObservationData(const industrial_calibration_libs::ObservationData &data);

static void
loadImagesFromPath(const std::string &path, const std::size_t &num_images,
  std::vector<cv::Mat> &images);

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

static void
showImage(const cv::Mat &image, const std::string &window_name)
{
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::imshow(window_name, image);
  cv::waitKey(0);
  cv::destroyWindow(window_name);
}

static void
printObservationData(const industrial_calibration_libs::ObservationData &data)
{
  for (std::size_t i = 0; i < data.size(); i++)
  {
    CONSOLE_OUTPUT("Observations for Image: " << i << " has Size: " 
      << data[i].size());
    CONSOLE_OUTPUT("Observations for Image: " << i << " has Points: ");
    for (std::size_t j = 0; j < data[i].size(); j++)
    {
      CONSOLE_OUTPUT(data[i][j]);
    }
  }
}

static void
loadImagesFromPath(const std::string &path, const std::size_t &num_images,
  std::vector<cv::Mat> &images)
{
  images.clear();
  images.reserve(num_images);

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = path + std::to_string(i) + ".png";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    ASSERT_TRUE(!image.empty());
    ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    images.push_back(image);    
  }
}
#endif // TEST_UTILS_H
