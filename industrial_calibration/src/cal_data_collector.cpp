#include <industrial_calibration/cal_data_collector.h>

CalDataCollector::CalDataCollector(ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh), pnh_(pnh), i_(0) // RENAME i_ TO SOMETHING ELSE
{
  this->initDisplayWindow("Camera View");

  pnh_.getParam("pattern_cols", pattern_cols_);
  pnh_.getParam("pattern_rows", pattern_rows_);
  pnh_.getParam("from_link", from_link_);
  pnh_.getParam("to_link", to_link_);
  pnh_.getParam("save_path", save_path_);

  camera_info_sub_ = pnh_.subscribe("camera_info", 1, 
    &CalDataCollector::cameraInfoCallback, this);
}

void CalDataCollector::collectData(void)
{
  if (!this->checkSettings()) {return;}

  message_filters::Subscriber<sensor_msgs::Image> image_sub(pnh_, 
    "image", 1);
  message_filters::Subscriber<sensor_msgs::JointState> joint_state_sub(pnh_, 
    "joint_states", 1);
  message_filters::Synchronizer<SyncPolicy> synchronizer(SyncPolicy(10), image_sub, 
    joint_state_sub);
  synchronizer.registerCallback(boost::bind(&CalDataCollector::synchronizedMessageCallback, 
    this, _1, _2));

  ros::spin();

  ros::Rate rate(60);
  rate.sleep();
}

void CalDataCollector::synchronizedMessageCallback(const sensor_msgs::ImageConstPtr &image_msg,
  const sensor_msgs::JointStateConstPtr &joint_state_msg)
{
  ROS_INFO_STREAM("Joint State MSG Received: " << joint_state_msg->name.size());

  std::size_t size = joint_state_msg->name.size();
  
  std::vector<std::string> joint_names;
  std::vector<float> joint_state;

  joint_names.clear();
  joint_names.resize(size);
  joint_state.clear();
  joint_state.resize(size);

  for (std::size_t i = 0; i < size; i++)
  {
    joint_names[i] = joint_state_msg->name[i];
    joint_state[i] = joint_state_msg->position[i];
  }

  ROS_INFO_STREAM("Image MSG Received");

  cv_bridge::CvImageConstPtr msg_ptr;

  try {msg_ptr = cv_bridge::toCvCopy(image_msg);}
  catch (cv_bridge::Exception &ex) 
  {
    ROS_ERROR_STREAM("Could not load image from message");
  }

  cv::Mat raw_image;
  msg_ptr->image.copyTo(raw_image);

  cv::Mat grid_image;
  msg_ptr->image.copyTo(grid_image);

  // Find circlegrid and draw grid
  cv::Mat display_image;
  if (!raw_image.empty() && !grid_image.empty())
  {
    if (this->drawGrid(grid_image))
    {
      ROS_INFO_STREAM("CIRCLE GRID FOUND!!!");
      display_image = grid_image;
    }
    else
    {
      ROS_INFO_STREAM("NO CIRCLE GRID FOUND!!!");
      display_image = raw_image;    
    }

    cv::imshow(cv_window_name_, display_image);

    int key = cv::waitKey(30);

    if ((key % 256) == 83 || (key % 256) == 115) // ASCII S / s
    {
      this->saveCalibrationData(raw_image, joint_names, joint_state);
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

void CalDataCollector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  ROS_INFO_STREAM("CameraInfo MSG Received!");
  intrinsic_matrix_.resize(9);
  for (std::size_t i = 0; i < 9; i++)
  {
    intrinsic_matrix_[i] = msg->K[i];
  }

  // Only want to subscribe once since this shouldn't change.
  camera_info_sub_.shutdown();
}

inline bool CalDataCollector::drawGrid(cv::Mat &image)
{
  std::size_t cols = static_cast<std::size_t>(pattern_cols_);
  std::size_t rows = static_cast<std::size_t>(pattern_rows_);

  std::vector<cv::Point2f> centers;
  cv::Size pattern_size(cols, rows);
  cv::SimpleBlobDetector::Params params;
  params.maxArea = image.cols * image.rows;
  const cv::Ptr<cv::FeatureDetector> &blob_detector = cv::SimpleBlobDetector::create(params);

  bool pattern_found = cv::findCirclesGrid(image, pattern_size, centers,
    cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, blob_detector);
  if (pattern_found && centers.size() == (cols*rows))
  {
    cv::Mat center_image = cv::Mat(centers);
    cv::Mat center_converted;
    center_image.convertTo(center_converted, CV_32F);
    cv::drawChessboardCorners(image, pattern_size, cv::Mat(centers), 
      pattern_found);
    return true;
  }

  return false;
}

inline void CalDataCollector::printTransform(const tf::StampedTransform &transform)
{
  tf::Vector3 origin = transform.getOrigin();
  tf::Quaternion quaternion = transform.getRotation();

  ROS_INFO_STREAM("Translation: [" << origin.getX() << ", " << origin.getY() <<
    ", " << origin.getZ() << "]");
  ROS_INFO_STREAM("Quaternion: [" << quaternion.getX() << ", " << quaternion.getY() <<
    ", " << quaternion.getZ() << ", " << quaternion.getW() << "]");
}

inline void CalDataCollector::writeTransformToYAML(YAML::Emitter &out, 
  const std::string &from_link, const std::string &to_link, 
  const tf::StampedTransform &transform)
{
  std::vector<double> translation; translation.resize(3);
  std::vector<double> quaternion; quaternion.resize(4);

  translation[0] = transform.getOrigin().getX();
  translation[1] = transform.getOrigin().getY();
  translation[2] = transform.getOrigin().getZ();

  quaternion[0] = transform.getRotation().getX();
  quaternion[1] = transform.getRotation().getY();
  quaternion[2] = transform.getRotation().getZ();
  quaternion[3] = transform.getRotation().getW();

  std::string name = from_link + "_to_" + to_link;

  out << YAML::Key << name;
  out << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "From";
      out << YAML::Value << from_link;
    out << YAML::Key << "To";
      out << YAML::Value << to_link;
    out << YAML::Key << "Translation";
      out << YAML::Value << translation;
    out << YAML::Key << "Quaternion";
      out << YAML::Value << quaternion;
  out << YAML::EndMap;
}

inline void CalDataCollector::writeJointStateToYAML(YAML::Emitter &out,
  const std::vector<std::string> &joint_names, const std::vector<float> &joint_state)
{
  out << YAML::Key << "Joint Names";
  out << YAML::Value << joint_names;
  out << YAML::Key << "Joint State";
  out << YAML::Value << joint_state;
}

inline void CalDataCollector::writeIntrinsicMatrixToYAML(YAML::Emitter &out,
  const std::vector<double> &intrinsic_matrix)
{
  out << YAML::Key << "Intrinsic Matrix";
  out << YAML::Value << intrinsic_matrix;
}

inline void CalDataCollector::saveCalibrationData(const cv::Mat &image,
  const std::vector<std::string> &joint_names, const std::vector<float> &joint_state)
{
  // TF Transforms
  tf_.waitForTransform(from_link_, to_link_, ros::Time(), ros::Duration(0.5));
  tf::StampedTransform transform;
  
  try
  {
    tf_.lookupTransform(from_link_, to_link_, ros::Time(), transform);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR_STREAM("TF Exception: " << ex.what());
  }

  YAML::Emitter out;
  out << YAML::BeginMap;
    this->writeTransformToYAML(out, from_link_, to_link_, transform);
    this->writeJointStateToYAML(out, joint_names, joint_state);
    this->writeIntrinsicMatrixToYAML(out, intrinsic_matrix_);
  out << YAML::EndMap;

  if (out.good())
  {
    std::string yaml_file_name = save_path_ + std::to_string(i_) + ".yaml";
    ROS_INFO_STREAM("Saving YAML to: " << yaml_file_name);
    std::ofstream yaml_file(yaml_file_name);
    yaml_file << out.c_str();
  }

  try
  {
    // No compression for png
    std::vector<int> png_params;
    png_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    png_params.push_back(0);

    std::string image_file_name = save_path_ + std::to_string(i_) + ".png";
    if (cv::imwrite(image_file_name, image, png_params))
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

void CalDataCollector::initDisplayWindow(const std::string &window_name)
{
  cv_window_name_ = window_name;
  cv::namedWindow(cv_window_name_, CV_WINDOW_NORMAL);
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