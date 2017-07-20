#include <industrial_calibration/cal_data_collector.h>

CalDataCollector::CalDataCollector(ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh), pnh_(pnh), image_transport(pnh_), exit_(false)
{
  this->initDisplayWindow("Camera View");

  image_subscriber = image_transport.subscribe("image_topic", 1,
    boost::bind(&CalDataCollector::imageCallback, this, _1));

  pnh_.getParam("pattern_cols", pattern_cols_);
  pnh_.getParam("pattern_rows", pattern_rows_);
  pnh_.getParam("save_path", save_path_);
}

void CalDataCollector::collectData(void)
{
  if (!this->checkSettings() || this->exit_) {return;}
  ros::spin();
}

bool CalDataCollector::drawGrid(const cv::Mat &input_image, cv::Mat &output_image)
{
  std::size_t cols = static_cast<std::size_t>(pattern_cols_);
  std::size_t rows = static_cast<std::size_t>(pattern_rows_);

  std::vector<cv::Point2f> centers;
  cv::Size pattern_size(cols, rows);
  cv::SimpleBlobDetector::Params params;
  params.maxArea = input_image.cols * input_image.rows;
  const cv::Ptr<cv::FeatureDetector> &blob_detector = cv::SimpleBlobDetector::create(params);

  for (double alpha = 1.0; alpha < 3.0; alpha += 0.1)
  {
    for (int beta = 0; beta < 100; beta++)
    {
      cv::Mat altered_image;
      input_image.convertTo(altered_image, -1, alpha, beta);
      bool pattern_found = cv::findCirclesGrid(altered_image, pattern_size, centers,
        cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blob_detector);
      if (pattern_found && centers.size() == (cols*rows))
      {
        cv::Mat center_image = cv::Mat(centers);
        cv::Mat center_converted;
        center_image.convertTo(center_converted, CV_32F);
        cv::drawChessboardCorners(output_image, pattern_size, cv::Mat(centers), 
          pattern_found);
        return true;
      }
    }
  }
  return false;
}

void CalDataCollector::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  boost::mutex::scoped_lock lock(MUTEX);

  cv_bridge::CvImageConstPtr msg_ptr;

  try {msg_ptr = cv_bridge::toCvCopy(msg);}

  catch (cv_bridge::Exception &ex) 
  {
    ROS_ERROR_STREAM("Could not load image from message");
  }

  raw_image_ = msg_ptr->image;
  grid_image_ = msg_ptr->image;

  // Find circlegrid and draw grid
  if (!raw_image_.empty() && !grid_image_.empty())
  {
    if (this->drawGrid(raw_image_, grid_image_))
    {
      ROS_INFO_STREAM("Modified Circle Grid Found!");
      cv::imshow(cv_window_name_, grid_image_);
      if ((cv::waitKey(30) % 256) == 27) // ESC
      {
        cv::destroyWindow(cv_window_name_);
        exit_ = true;
        return;
      } 
    }
  }
}

void CalDataCollector::mouseCallbackInternal(int event, int x, int y, int flags)
{
  if (event != cv::EVENT_LBUTTONDOWN) {return;}
  boost::mutex::scoped_lock lock(MUTEX, boost::try_to_lock);
  
  if (!lock.owns_lock()) {return;}
  else
  {
    // Save Image
    ROS_INFO_STREAM("Saving Image");
  }
}

void CalDataCollector::mouseCallback(int event, int x, int y, int flags, void* param)
{
  CalDataCollector *cal_data_collector = static_cast<CalDataCollector*>(param);
  cal_data_collector->mouseCallbackInternal(event, x, y, flags);
}

void CalDataCollector::initDisplayWindow(const std::string &window_name)
{
  cv_window_name_ = window_name;
  cv::namedWindow(cv_window_name_, CV_WINDOW_NORMAL);
  cv::setMouseCallback(cv_window_name_, &mouseCallback);
}

bool CalDataCollector::checkSettings(void)
{
  if (pattern_cols_ < 1 || pattern_rows_ < 1) 
  {
    ROS_ERROR_STREAM("The columns and rows for the pattern must be set!");
    return false;
  }

  if (save_path_.empty())
  {
    ROS_ERROR_STREAM("No save path specified!");
    return false;
  }

  return true;  
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cal_data_collector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  CalDataCollector cal_data_collector(nh, pnh);
  cal_data_collector.collectData();

  return 0;
}