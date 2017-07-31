#include <industrial_calibration/cal_data_collector.h>

CalDataCollector::CalDataCollector(ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh), pnh_(pnh), image_transport_(pnh_)
{
  this->initDisplayWindow("Camera View");

#if 0
  joint_state_subscriber_ = pnh_.subscribe("/joint_states", 1,
    &CalDataCollector::jointStateCallback, this);

  image_subscriber_ = image_transport_.subscribe("/calibration_image", 1,
    boost::bind(&CalDataCollector::imageCallback, this, _1));
#endif

  pnh_.getParam("pattern_cols", pattern_cols_);
  pnh_.getParam("pattern_rows", pattern_rows_);
  pnh_.getParam("save_path", save_path_);

  // TEMPORARY, REMOVE THIS LATER
  i_ = 0;
}

void CalDataCollector::collectData(void)
{
  if (!this->checkSettings()) {return;}

  message_filters::Subscriber<sensor_msgs::Image> image_sub(pnh_, 
    "/calibration_image", 1);
  message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub(pnh_, 
    "/joint_states", 1);
  message_filters::Synchronizer<SyncPolicy> synchronizer(SyncPolicy(10), image_sub, 
    joint_state_sub);
  synchronizer.registerCallback(boost::bind(&CalDataCollector::synchronizedMessageCallback, 
    this, _1, _2));
  
  ros::spin();
}

void CalDataCollector::synchronizedMessageCallback(const sensor_msgs::ImageConstPtr &image_msg,
  const sensor_msgs::JointStateConstPtr &joint_state_msg)
{
  ROS_INFO_STREAM("Joint State MSG Received: " << joint_state_msg->name.size());

  std::size_t size = joint_state_msg->name.size();
  joint_names_.clear();
  joint_names_.resize(size);
  joint_state_.clear();
  joint_state_.resize(size);

  for (std::size_t i = 0; i < size; i++)
  {
    joint_names_[i] = joint_state_msg->name[i];
    joint_state_[i] = joint_state_msg->position[i];
  }

  ROS_INFO_STREAM("Image MSG Received");

  cv_bridge::CvImageConstPtr msg_ptr;

  try {msg_ptr = cv_bridge::toCvCopy(image_msg);}

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

    int key = cv::waitKey(30);

    if ((key % 256) == 83 || (key % 256) == 115) // S/s
    {
      saveData(raw_image_);
    }

    if ((key % 256) == 27) // ESC
    {
      cv::destroyWindow(cv_window_name_);
      ros::shutdown();
      return;
    } 
  }
  else
  {
    ROS_ERROR_STREAM("INPUT IMAGE IS EMPTY!");
  }  
}

void CalDataCollector::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  // boost::mutex::scoped_lock lock(MUTEX);

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
  // boost::mutex::scoped_lock lock(MUTEX);
  ROS_INFO_STREAM("Image MSG Received");

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

    int key = cv::waitKey(30);

    if ((key % 256) == 83 || (key % 256) == 115) // S/s
    {
      saveData(raw_image_);
    }

    if ((key % 256) == 27) // ESC
    {
      cv::destroyWindow(cv_window_name_);
      ros::shutdown();
      return;
    } 
  }
  else
  {
    ROS_ERROR_STREAM("INPUT IMAGE IS EMPTY!");
  }
}

inline void CalDataCollector::saveData(const cv::Mat &image)
{
  // TF Transforms
  std::string base_link = "base_link";
  std::string link_6 = "link_6";
  std::string tool0 = "tool0";
  tf_.waitForTransform(base_link, tool0, ros::Time(), ros::Duration(1.0));

  try
  {
    tf::StampedTransform transform;
    tf_.lookupTransform(base_link, tool0, ros::Time(), transform);

    tf::Vector3 origin = transform.getOrigin();
    tf::Quaternion quaternion = transform.getRotation();
#if 1
    ROS_INFO_STREAM("Translation: [" << origin.getX() << ", " << origin.getY() <<
      ", " << origin.getZ() << "]");
    ROS_INFO_STREAM("Quaternion: [" << quaternion.getX() << ", " << quaternion.getY() <<
      ", " << quaternion.getZ() << ", " << quaternion.getW() << "]");    
#endif
  #include <iomanip>
  #include <iostream>
  std::cout.precision(3);
  std::cout.setf(std::ios::fixed,std::ios::floatfield);
  std::cout << "- Translation: [" << origin.getX() << ", " << origin.getY() 
    << ", " << origin.getZ() << "]" <<   std::endl;
  std::cout << "- Rotation: in Quaternion [" << quaternion.getX() << ", " 
    << quaternion.getY() << ", " << quaternion.getZ() << ", " 
    << quaternion.getW() << "]" << std::endl;

  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR_STREAM("TF Exception: " << ex.what());
  }


  ROS_INFO_STREAM("Saving Joints: " << joint_names_.size() << " " << joint_state_.size());
  for (std::size_t i = 0; i < joint_names_.size(); i++)
  {
    ROS_INFO_STREAM("Name: " << joint_names_[i] << " Position: " << joint_state_[i]);
  }  

  ROS_INFO_STREAM("Saving Image");
  try
  {
    // No compression for png
    std::vector<int> png_params;
    png_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    png_params.push_back(0);

    std::string image_file_name = save_path_ + std::to_string(i_) + ".png";
    ROS_INFO_STREAM(save_path_);
    if (cv::imwrite(image_file_name, raw_image_, png_params))
    {
      ROS_INFO_STREAM("Saving Image: " << image_file_name);
      i_++;
    }
    else
    {
      ROS_ERROR_STREAM("Fail");
    }
  }
  catch (std::exception &ex)
  {
    ROS_ERROR_STREAM("More Fail");
  }  
}

void CalDataCollector::mouseCallbackInternal(int event, int x, int y, int flags)
{
  #if 0
  if (event != cv::EVENT_LBUTTONDOWN) {return;}

  boost::mutex::scoped_lock lock(MUTEX, boost::try_to_lock);
  if (!lock.owns_lock()) 
  {
    ROS_INFO_STREAM("I DONT OWN THIS LOCK");
    return;
  }

  else
  {
    try
    {
      // No compression for png
      std::vector<int> png_params;
      png_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
      png_params.push_back(0);

      if (cv::imwrite(save_path_ + std::to_string(i_), raw_image_, png_params))
      {
        ROS_INFO_STREAM("Saving Image: " << i_ << ".png");
        i_++;
      }
      else
      {
        ROS_ERROR_STREAM("Fail");
      }
    }
    catch (std::exception &ex)
    {
      ROS_ERROR_STREAM("More Fail");
    }
    
  }

  // else
  // {
    // Save Image
    // ROS_INFO_STREAM("Saving Images");
    // ROS_INFO_STREAM("Saving Image: " << joint_names_.size() << " " << joint_state_.size());
    // for (std::size_t i = 0; i < joint_names_.size(); i++)
    // {
      // ROS_INFO_STREAM("Name: " << joint_names_[i] << " Position: " << joint_state_[i]);
    // }
  // }
  #endif
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
  // cv::setMouseCallback(cv_window_name_, &mouseCallback);
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