#include <industrial_calibration/cal_data_collector.h>

CalDataCollector::CalDataCollector(ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh), pnh_(pnh), image_transport_(pnh_), exit_(false)
{
  this->initDisplayWindow("Camera View");

  joint_state_subscriber_ = pnh_.subscribe("/joint_states", 1,
    &CalDataCollector::jointStateCallback, this);

  image_subscriber_ = image_transport_.subscribe("/calibration_image", 1,
    boost::bind(&CalDataCollector::imageCallback, this, _1));

  // image_subscriber_ = image_transport_.subscribe("/usb_cam/image_raw", 1,

  pnh_.getParam("pattern_cols", pattern_cols_);
  pnh_.getParam("pattern_rows", pattern_rows_);
  pnh_.getParam("save_path", save_path_);
}

void CalDataCollector::collectData(void)
{
  if (!this->checkSettings() || this->exit_) {return;}
  ros::spin();
}

void CalDataCollector::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  boost::mutex::scoped_lock lock(MUTEX);

  ROS_INFO_STREAM("Joint State MSG Received: " << msg->name.size());

  std::size_t size = msg->name.size();
  joint_names_.clear();
  joint_names_.resize(size);
  joint_state_.clear();
  joint_state_.resize(size);

  for (std::size_t i = 0; i < size; i++)
  {
    joint_names_[i] = msg->name[i];
    joint_state_[i] = msg->position[i];
  }
}

bool CalDataCollector::drawGrid(const cv::Mat &input_image, cv::Mat &output_image)
{
  boost::mutex::scoped_lock lock(MUTEX);

  std::size_t cols = static_cast<std::size_t>(pattern_cols_);
  std::size_t rows = static_cast<std::size_t>(pattern_rows_);

  std::vector<cv::Point2f> centers;
  cv::Size pattern_size(cols, rows);
  cv::SimpleBlobDetector::Params params;
  params.maxArea = input_image.cols * input_image.rows;
  const cv::Ptr<cv::FeatureDetector> &blob_detector = cv::SimpleBlobDetector::create(params);

  for (double alpha = 1.0; alpha < 3.0; alpha += 0.5)
  {
    for (int beta = 0; beta < 100; beta += 50)
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
  cv_bridge::CvImageConstPtr msg_ptr;

  try {msg_ptr = cv_bridge::toCvCopy(msg);}

  catch (cv_bridge::Exception &ex) 
  {
    ROS_ERROR_STREAM("Could not load image from message");
  }

  raw_image_ = msg_ptr->image;
  grid_image_ = msg_ptr->image;

  // Find circlegrid and draw grid
  cv::Mat display_image;
  if (!raw_image_.empty() && !grid_image_.empty())
  {
    if (this->drawGrid(raw_image_, grid_image_))
    {
      ROS_INFO_STREAM("CIRCLE GRID FOUND!!!");
      display_image = grid_image_;
    }
    else
    {
      ROS_INFO_STREAM("NO CIRCLE GRID FOUND!!!");
      display_image = raw_image_;    
    }

    cv::imshow(cv_window_name_, display_image);
    if ((cv::waitKey(30) % 256) == 27) // ESC
    {
      cv::destroyWindow(cv_window_name_);
      exit_ = true;
      return;
    } 
  }
  else
  {
    ROS_ERROR_STREAM("INPUT IMAGE IS EMPTY!");
  }
}

void CalDataCollector::mouseCallbackInternal(int event, int x, int y, int flags)
{
  if (event != cv::EVENT_LBUTTONDOWN) {return;}

  boost::mutex::scoped_lock lock(MUTEX, boost::try_to_lock);
  if (!lock.owns_lock()) 
  {
    ROS_INFO_STREAM("I DONT OWN THIS LOCK");
    return;
  }
  else
  {
    // Save Image
    ROS_INFO_STREAM("Saving Image: " << joint_names_.size() << " " << joint_state_.size());
    for (std::size_t i = 0; i < joint_names_.size(); i++)
    {
      ROS_INFO_STREAM("Name: " << joint_names_[i] << " Position: " << joint_state_[i]);
    }
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
  cv::startWindowThread();
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