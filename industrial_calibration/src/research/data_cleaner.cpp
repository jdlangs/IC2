/*
  This program will run through the data and ask the user if a certain
  data point should be deleted.

  Then it will rename all the data.
*/

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <industrial_calibration_libs/industrial_calibration_libs.h>
#include <industrial_calibration/helper_functions.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

#define ICL industrial_calibration_libs

typedef std::vector<cv::Mat> CalibrationImages;

bool isPNG(const std::string &file_name);

int getTotalImages(const std::string &path);

int getKey(void);

bool deleteImage(const cv::Mat &image);

int checkCalibrationImages(const std::string &path, 
  const ICL::Target &target);

void renameAllImages(const std::string &data_path);

bool isPNG(const std::string &file_name)
{
  std::size_t dot_location = file_name.find('.');
  std::string extension = file_name.substr(dot_location + 1);
  if (extension.compare("png") == 0) {return true;}
  return false;
}

int getTotalImages(const std::string &path)
{
  int count = 0;

  boost::filesystem::path image_dir(path);
  boost::filesystem::directory_iterator end_iter;

  if (boost::filesystem::exists(image_dir) &&
    boost::filesystem::is_directory(image_dir))
  {
    for (boost::filesystem::directory_iterator dir_iter(image_dir);
      dir_iter != end_iter; ++dir_iter)
    {
      if (boost::filesystem::is_regular_file(dir_iter->status()) &&
        isPNG(dir_iter->path().filename().string()))
      {
        count += 1;
      }
    }
  }
  return count;
}

int getKey(void)
{
  int key = cv::waitKey(0) % 256;

  // ASCII 'backspace', 'esc', 'space', 'delete'
  if (key == 8 || key == 27 || key == 32 || key == 255)
    return key;
  else
    return getKey();
}

// If true, delete, if false, keep.
bool deleteImage(const cv::Mat &image)
{
  cv::imshow("Image", image);
  int key = getKey();

  if (key == 32) // ASCII 'space'
  {
    return false;
  }
  if (key == 8 || key == 255) // ASCII 'backspace' or 'delete'
  {
    return true;
  }
  if (key == 27) // ASCII 'Esc'
  {
    exit(0);
  }
  return false;  
}

int checkCalibrationImages(const std::string &path, const ICL::Target &target)
{
  // Keep track of number of images deleted.
  int number_deleted = 0;

  // Create a cv window
  cv::namedWindow("Image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO);

  // Create an observation extractor to extract observations
  ICL::ObservationExtractor observation_extractor(target);

  boost::filesystem::path image_dir(path);
  boost::filesystem::directory_iterator end_iter;

  // Instead of deleting the images, move it to a directory called "bad".
  boost::filesystem::path bad_image_dir(path + "bad/");
  if (!boost::filesystem::exists(bad_image_dir))
  {
    boost::filesystem::create_directory(bad_image_dir);
  }

  if (boost::filesystem::exists(image_dir) &&
    boost::filesystem::is_directory(image_dir))
  {
    for (boost::filesystem::directory_iterator dir_iter(image_dir);
      dir_iter != end_iter; ++dir_iter)
    {
      if (boost::filesystem::is_regular_file(dir_iter->status()) &&
        isPNG(dir_iter->path().filename().string()))
      {
        std::string image_path = path + dir_iter->path().filename().string();
        cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

        // Extract observations
        cv::Mat grid_image;
        observation_extractor.extractObservation(image, grid_image);

        if (deleteImage(grid_image))
        {
          boost::filesystem::rename(image_dir.string() + dir_iter->path().filename().string(),
            bad_image_dir.string() + dir_iter->path().filename().string());
          number_deleted += 1;
        }
      }
    }
  }
  return number_deleted;
}

// I could probably do this up in checkCalibrationImages
// but since speed is not an issue, i'm keeping it in its own function.
void renameAllImages(const std::string &data_path)
{
  std::size_t count = 0;

  boost::filesystem::path image_dir(data_path);
  boost::filesystem::directory_iterator end_iter;

  if (boost::filesystem::exists(image_dir) &&
    boost::filesystem::is_directory(image_dir))
  {
    for (boost::filesystem::directory_iterator dir_iter(image_dir);
      dir_iter != end_iter; ++dir_iter)
    {
      if (boost::filesystem::is_regular_file(dir_iter->status()) &&
        isPNG(dir_iter->path().filename().string()))
      {
        boost::filesystem::rename(image_dir.string() + dir_iter->path().filename().string(),
          image_dir.string() + std::to_string(count) + ".png");
        count += 1;
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "intrinsic_opencv");
  ros::NodeHandle pnh("~");

  std::string data_path;
  pnh.getParam("data_path", data_path);

  // Load Target Data
  ICL::Target target(data_path + "mcircles_9x12.yaml");

  // Delete bad calibration images.
  std::cout << "Total Images Before: " << getTotalImages(data_path) << '\n';
  std::cout << "Number Deleted: " << checkCalibrationImages(data_path, target) << '\n';
  std::cout << "Total Images After: " << getTotalImages(data_path) << '\n';

  // Rename and sort...
  renameAllImages(data_path);
  std::cout << "Total Images After Rename: " << getTotalImages(data_path) << '\n';

  return 0;
}