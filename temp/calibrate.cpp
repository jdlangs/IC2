// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// Standard Library
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <cstdarg>

int main(void)
{
  // cv::Mat image_gray = cv::imread("4.jpg", CV_LOAD_IMAGE_COLOR);
  // cv::Mat image_gray = cv::imread("4.jpg", CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat image_gray = cv::imread("BLAH.jpg", CV_LOAD_IMAGE_COLOR);
  // cv::Mat image_gray = cv::imread("3.png", CV_LOAD_IMAGE_COLOR);
  // cv::Mat image_gray; 
  // cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  // image_gray = image_gray > 128;


  // // Canny
  // cv::Mat canny_output;
  // cv::Canny(image_gray, canny_output, 1, 25, 3);

  // // Contours
  // std::vector<std::vector<cv::Point>> contours;
  // std::vector<cv::Vec4i> hierarchy;

  // cv::findContours(image_gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

  // std::vector<std::vector<cv::Point>> hull(contours.size());
  // {
  //   // if (cv::contourArea(contours[i]) > 5000)
  //   {
  //     cv::convexHull(cv::Mat(contours[i]), hull[i]);
  //     cv::approxPolyDP(cv::Mat(hull[i]), hull[i], 0.1*cv::arcLength(hull[i], true), true);
  //     if (hull.size() == 4)
  //     {
  //       cv::drawContours(image_color, hull[i], 0, cv::Scalar(0,255,0), 2);
  //     }
  //   }
  // }

  // cv::Size dsize = cv::Size(image_gray.cols / 4, image_gray.rows / 4);
  // cv::resize(image_gray, image_gray, dsize);

  cv::namedWindow("window", cv::WINDOW_NORMAL);

#if 1
  // cv::Size pattern_size(5, 7);
  cv::Size pattern_size(7, 5);
  // cv::Size pattern_size(3,4);

  // cv::Size pattern_size(5,10);
  // cv::Size pattern_size(10,5);

  std::size_t max_area = image_gray.cols*image_gray.rows;
  std::cout << "AREA: " << image_gray.cols*image_gray.rows << std::endl;

  cv::SimpleBlobDetector::Params parameters;
  parameters.maxArea = max_area;
  const cv::Ptr<cv::FeatureDetector> &blob_detector = new cv::SimpleBlobDetector(parameters);

  std::vector<cv::Point2f> centers;
  bool patternfound = cv::findCirclesGrid(image_gray, pattern_size, centers, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blob_detector);

  std::cout << "Circles Found: " << patternfound << std::endl;
  std::cout << centers.size() << std::endl;
  for (std::size_t i = 0; i < centers.size(); i++)
  {
    std::cout << centers[i] << std::endl;
  }

  cv::drawChessboardCorners(image_gray, pattern_size, cv::Mat(centers), patternfound);
#endif

  cv::imshow("window", image_gray);
  cv::waitKey(0);

  return 0;
}