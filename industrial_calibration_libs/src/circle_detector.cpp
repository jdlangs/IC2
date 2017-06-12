/*
 IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.

 By downloading, copying, installing or using the software you agree 
 to this license.
 If you do not agree to this license, do not download, install,
 copy or use the software.


                          License Agreement
               For Open Source Computer Vision Library

Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
Copyright (C) 2009, Willow Garage Inc., all rights reserved.
Copyright (C) 2014, Southwest Research Institute, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistribution's of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistribution's in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * The name of the copyright holders may not be used to endorse or 
    promote products
    derived from this software without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall the Intel Corporation or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

// Slight Modification of OpenCV function to use ellipse fitting rather than 
// center of mass of contour to provide the location of the circle.

#include <industrial_calibration_libs/circle_detector.h>

#include <algorithm>
#include <iterator>

namespace cv
{
class CV_EXPORTS_W CircleDetectorImpl : public CircleDetector
{
public:
  using CircleDetector::detect; // Note(gChiou): Fixes woverloaded-virtual warnings

  explicit CircleDetectorImpl(const CircleDetector::Params &parameters
    = CircleDetector::Params());

  virtual void read(const FileNode &fn);
  virtual void write(FileStorage &fs) const;

protected:
  struct CV_EXPORTS Center
  {
      Point2d location;
      double radius;
      double confidence;
  };

  virtual void detect(InputArray image, std::vector<KeyPoint> &keypoints, 
    InputArray mask = noArray());

  virtual void findCircles(InputArray image, InputArray binaryImage, std::vector<Center> &centers) const;

  Params params;
};

CircleDetector::Params::Params()
{
  thresholdStep = 10;
  minThreshold = 50;
  maxThreshold = 220;
  minRepeatability = 2;
  minDistBetweenCircles = 10;
  minRadiusDiff = 10;

  filterByColor = true;
  circleColor = 0;

  filterByArea = true;
  minArea = 25;
  maxArea = 5000;

  filterByCircularity = false;
  minCircularity = 0.8f;
  maxCircularity = std::numeric_limits<float>::max();

  filterByInertia = true;
  minInertiaRatio = 0.1f;
  maxInertiaRatio = std::numeric_limits<float>::max();

  filterByConvexity = true;
  minConvexity = 0.95f;
  maxConvexity = std::numeric_limits<float>::max();
}

// Note(gChiou): Keeping this in for now in case I broke something.
// void CircleDetector::Params::read(const cv::FileNode &fn)
// {
//   thresholdStep = fn["thresholdStep"];
//   minThreshold = fn["minThreshold"];
//   maxThreshold = fn["maxThreshold"];

//   minRepeatability = (size_t)(int)fn["minRepeatability"];
//   minDistBetweenCircles = fn["minDistBetweenCircles"];

//   filterByColor = (int)fn["filterByColor"] != 0 ? true : false;
//   circleColor = (uchar)(int)fn["circleColor"];

//   filterByArea = (int)fn["filterByArea"] != 0 ? true : false;
//   minArea = fn["minArea"];
//   maxArea = fn["maxArea"];

//   filterByCircularity = (int)fn["filterByCircularity"] != 0 ? true : false;
//   minCircularity = fn["minCircularity"];
//   maxCircularity = fn["maxCircularity"];

//   filterByInertia = (int)fn["filterByInertia"] != 0 ? true : false;
//   minInertiaRatio = fn["minInertiaRatio"];
//   maxInertiaRatio = fn["maxInertiaRatio"];

//   filterByConvexity = (int)fn["filterByConvexity"] != 0 ? true : false;
//   minConvexity = fn["minConvexity"];
//   maxConvexity = fn["maxConvexity"];
// }

void CircleDetector::Params::read(const cv::FileNode &fn)
{
  thresholdStep = fn["thresholdStep"];
  minThreshold = fn["minThreshold"];
  maxThreshold = fn["maxThreshold"];

  minRepeatability = static_cast<size_t>(static_cast<int>(fn["minRepeatability"]));
  minDistBetweenCircles = fn["minDistBetweenCircles"];

  filterByColor = static_cast<int>(fn["filterByColor"]) != 0 ? true : false;
  circleColor = static_cast<uchar>(static_cast<int>(fn["circleColor"]));

  filterByArea = static_cast<int>(fn["filterByArea"]) != 0 ? true : false;
  minArea = fn["minArea"];
  maxArea = fn["maxArea"];

  filterByCircularity = static_cast<int>(fn["filterByCircularity"]) != 0 ? true : false;
  minCircularity = fn["minCircularity"];
  maxCircularity = fn["maxCircularity"];

  filterByInertia = static_cast<int>(fn["filterByInertia"]) != 0 ? true : false;
  minInertiaRatio = fn["minInertiaRatio"];
  maxInertiaRatio = fn["maxInertiaRatio"];

  filterByConvexity = static_cast<int>(fn["filterByConvexity"]) != 0 ? true : false;
  minConvexity = fn["minConvexity"];
  maxConvexity = fn["maxConvexity"];
}

// Note(gChiou): Keeping this in for now in case I broke something.
// void CircleDetector::Params::write(cv::FileStorage &fs) const
// {
//   fs << "thresholdStep" << thresholdStep;
//   fs << "minThreshold" << minThreshold;
//   fs << "maxThreshold" << maxThreshold;

//   fs << "minRepeatability" << (int)minRepeatability;
//   fs << "minDistBetweenCircles" << minDistBetweenCircles;

//   fs << "filterByColor" << (int)filterByColor;
//   fs << "circleColor" << (int)circleColor;

//   fs << "filterByArea" << (int)filterByArea;
//   fs << "minArea" << minArea;
//   fs << "maxArea" << maxArea;

//   fs << "filterByCircularity" << (int)filterByCircularity;
//   fs << "minCircularity" << minCircularity;
//   fs << "maxCircularity" << maxCircularity;

//   fs << "filterByInertia" << (int)filterByInertia;
//   fs << "minInertiaRatio" << minInertiaRatio;
//   fs << "maxInertiaRatio" << maxInertiaRatio;

//   fs << "filterByConvexity" << (int)filterByConvexity;
//   fs << "minConvexity" << minConvexity;
//   fs << "maxConvexity" << maxConvexity;
// }

void CircleDetector::Params::write(cv::FileStorage &fs) const
{
  fs << "thresholdStep" << thresholdStep;
  fs << "minThreshold" << minThreshold;
  fs << "maxThreshold" << maxThreshold;

  fs << "minRepeatability" << static_cast<int>(minRepeatability);
  fs << "minDistBetweenCircles" << minDistBetweenCircles;

  fs << "filterByColor" << static_cast<int>(filterByColor);
  fs << "circleColor" << static_cast<int>(circleColor);

  fs << "filterByArea" << static_cast<int>(filterByArea);
  fs << "minArea" << minArea;
  fs << "maxArea" << maxArea;

  fs << "filterByCircularity" << static_cast<int>(filterByCircularity);
  fs << "minCircularity" << minCircularity;
  fs << "maxCircularity" << maxCircularity;

  fs << "filterByInertia" << static_cast<int>(filterByInertia);
  fs << "minInertiaRatio" << minInertiaRatio;
  fs << "maxInertiaRatio" << maxInertiaRatio;

  fs << "filterByConvexity" << static_cast<int>(filterByConvexity);
  fs << "minConvexity" << minConvexity;
  fs << "maxConvexity" << maxConvexity;
}

CircleDetectorImpl::CircleDetectorImpl(const CircleDetector::Params &parameters) :
  params(parameters) { }

void CircleDetectorImpl::read(const cv::FileNode &fn)
{
  params.read(fn);
}

void CircleDetectorImpl::write(cv::FileStorage &fs) const
{
  writeFormat(fs);
  params.write(fs);
}

void CircleDetectorImpl::findCircles(InputArray _image, InputArray _binaryImage, 
  std::vector<Center> &centers) const
{
  Mat image       = _image.getMat(); // Oh so much  cleaner this way :(
  Mat binaryImage = _binaryImage.getMat();
  
  (void)image;
  centers.clear();

  std::vector<std::vector<Point>> contours;
  Mat tmpBinaryImage = binaryImage.clone();
  findContours(tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  
  // Loop on all contours
  for (std::size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
  {
    // Each if statement may eliminate a contour through the continue function
    // Some if statements may also set the confidence whose default is 1.0
    Center center;
    center.confidence = 1;
    Moments moms = moments(Mat(contours[contourIdx]));
    if (params.filterByArea)
    {
      double area = moms.m00;
      if (area < params.minArea || area >= params.maxArea)
        continue;
    }
      
    if (params.filterByCircularity)
    {
      double area = moms.m00;
      double perimeter = arcLength(Mat(contours[contourIdx]), true);
      double ratio = 4 * CV_PI * area / (perimeter * perimeter);
      if (ratio < params.minCircularity || ratio >= params.maxCircularity)
        continue;
    }
      
    if (params.filterByInertia)
    {
      double denominator = sqrt(pow(2 * moms.mu11, 2) + pow(moms.mu20 - moms.mu02, 2));
      const double eps = 1e-2;
      double ratio;
      if (denominator > eps)
      {
        double cosmin = (moms.mu20 - moms.mu02) / denominator;
        double sinmin = 2 * moms.mu11 / denominator;
        double cosmax = -cosmin;
        double sinmax = -sinmin;
        
        double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
        double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
        ratio = imin / imax;
      }
      else
      {
        ratio = 1;
      }
      
      if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio)
        continue;
      
      center.confidence = ratio * ratio;
    }
      
    if (params.filterByConvexity)
    {
      std::vector<Point> hull;
      convexHull(Mat(contours[contourIdx]), hull);
      double area = contourArea(Mat(contours[contourIdx]));
      double hullArea = contourArea(Mat(hull));
      double ratio = area / hullArea;
      if (ratio < params.minConvexity || ratio >= params.maxConvexity)
        continue;
    }
    Mat pointsf;
    Mat(contours[contourIdx]).convertTo(pointsf, CV_32F);
    if(pointsf.rows<5) continue;
    RotatedRect box = fitEllipse(pointsf);
    
    // Find center
    center.location = box.center;
   
    // One more filter by color of central pixel
    if (params.filterByColor)
    {
      if (binaryImage.at<uchar>(cvRound(center.location.y), cvRound(center.location.x)) != params.circleColor)
        continue;
    }
      
    center.radius = (box.size.height+box.size.width) / 4.0;
    centers.push_back(center);
  }
}

void CircleDetectorImpl::detect(InputArray _image, std::vector<KeyPoint> &keypoints, 
  InputArray mask)
{
  Mat image = _image.getMat();
  keypoints.clear();
  Mat grayscaleImage;

  if (image.channels() == 3)
    cvtColor(image, grayscaleImage, CV_BGR2GRAY);
  else
    grayscaleImage = image;

  std::vector<std::vector<Center>> centers;
  for (double thresh = params.minThreshold; thresh < params.maxThreshold; 
    thresh += params.thresholdStep)
  { 
    Mat binarizedImage;
    threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);
    std::vector<Center> curCenters;
    findCircles(grayscaleImage, binarizedImage, curCenters);
    std::vector<std::vector<Center>> newCenters;
    for (std::size_t i = 0; i < curCenters.size(); i++)
    {
      bool isNew = true;
      for (std::size_t j = 0; j < centers.size(); j++)
      {
        double dist = norm(centers[j][ centers[j].size() / 2 ].location 
          - curCenters[i].location);
        double rad_diff = fabs(centers[j][ centers[j].size() / 2 ].radius 
          - curCenters[i].radius);

        isNew = dist >= params.minDistBetweenCircles || rad_diff >= params.minRadiusDiff;

        if (!isNew)
        {
          centers[j].push_back(curCenters[i]);

          std::size_t k = centers[j].size() - 1;
          while( k > 0 && centers[j][k].radius < centers[j][k-1].radius )
          {
            centers[j][k] = centers[j][k-1];
            k--;
          }
          centers[j][k] = curCenters[i];
          break;
        }
      }

      if (isNew)
      {
        newCenters.push_back(std::vector<Center> (1, curCenters[i]));
      }
    }
  std::copy(newCenters.begin(), newCenters.end(), std::back_inserter(centers));
  }

  for (std::size_t i = 0; i < centers.size(); i++)
  {
    if (centers[i].size() < params.minRepeatability)
      continue;
    Point2d sumPoint(0, 0);
    double normalizer = 0;
    for (std::size_t j = 0; j < centers[i].size(); j++)
    {
      sumPoint += centers[i][j].confidence * centers[i][j].location;
      normalizer += centers[i][j].confidence;
    }
    sumPoint *= (1. / normalizer);
    KeyPoint kpt(sumPoint, static_cast<float>(centers[i][centers[i].size() / 2].radius*2.0));
    keypoints.push_back(kpt);
  }
}

Ptr<CircleDetector> CircleDetector::create(const CircleDetector::Params &params)
{
  return makePtr<CircleDetectorImpl>(params);
}

} // namespace cv