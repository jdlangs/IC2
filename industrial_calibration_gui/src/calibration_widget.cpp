#include <industrial_calibration_gui/calibration_widget.h>

namespace industrial_calibration_gui
{
CalibrationWidget::CalibrationWidget(QWidget* parent) : QWidget(parent), pnh_("~"),
  it_(pnh_)
{
  // UI setup
  ui_ = new Ui::CalibrationWidget;
  ui_->setupUi(this);

  this->instructions_checkbox_state_ = false;
  this->target_set_from_file_ = false;
  this->collecting_data_ = true;
  this->updateCalibrationTypeText(ui_->calibration_type_combo_box->currentIndex());
  this->updateTopicList();
  this->updateFrameList();

  // Start page
  connect(ui_->instructions_checkbox, SIGNAL(stateChanged(int)),
    this, SLOT(instructionsCheckbox()));
  connect(ui_->start_data_collection_button, SIGNAL(clicked()), 
    this, SLOT(startDataCollectionButton()));
  connect(ui_->calibration_type_combo_box, SIGNAL(currentIndexChanged(int)), 
    this, SLOT(selectCalibrationTypeComboBox()));

  // Data collection page
  connect(ui_->output_location_line, SIGNAL(editingFinished()),
    this, SLOT(outputLocationLine()));
  connect(ui_->output_location_button, SIGNAL(clicked()),
    this, SLOT(outputLocationButton()));
  connect(ui_->load_target_line, SIGNAL(editingFinished()),
    this, SLOT(loadTargetLine()));
  connect(ui_->load_target_button, SIGNAL(clicked()),
    this, SLOT(loadTargetButton()));
  connect(ui_->refresh_button, SIGNAL(clicked()),
    this, SLOT(refreshComboBoxes()));
  connect(ui_->set_inputs_button, SIGNAL(clicked()),
    this, SLOT(setInputsButton()));
  connect(ui_->save_image_button, SIGNAL(clicked()),
    this, SLOT(saveImageButton()));
  connect(ui_->start_calibration_button, SIGNAL(clicked()),
    this, SLOT(startCalibrationButton()));
}

CalibrationWidget::~CalibrationWidget() { }

// Start Page
void CalibrationWidget::instructionsCheckbox(void)
{
  if (ui_->instructions_checkbox->isChecked())
  {
    this->instructions_checkbox_state_ = true;
  }
  else
  {
    this->instructions_checkbox_state_ = false;
  }
}

void CalibrationWidget::startDataCollectionButton(void)
{
  if (this->instructions_checkbox_state_)
  {
    if (ui_->calibration_type_combo_box->currentIndex() != 0)
    {
      ui_->stackedWidget->setCurrentIndex(1);
      this->refreshComboBoxes();
    }
  }
}

void CalibrationWidget::selectCalibrationTypeComboBox(void)
{
  int current_index = ui_->calibration_type_combo_box->currentIndex();
  ROS_INFO_STREAM("Combo Box State Changed " << current_index);
  this->updateCalibrationTypeText(current_index);
}

void CalibrationWidget::updateCalibrationTypeText(int current_index)
{
  switch (current_index)
  {
    case welcome_screen:
      ui_->calibration_type_text_browser->setText("Welcome to the industrial_calibration_gui.");
      break;

    case camera_on_wrist_extrinsic:
      ui_->calibration_type_text_browser->setText("Static Target Moving Camera on Wrist (Extrinsic)");
      break;

    case camera_on_wrist_intrinsic:
      ui_->calibration_type_text_browser->setText("Static Target Moving Camera on Wrist (Extrinsic + Intrinsic) [EXPERIMENTAL]");
      break;

    case camera_in_world_extrinsic:
      ui_->calibration_type_text_browser->setText("Static Camera Moving Target on Wrist (Extrinsic)");

    case camera_in_world_intrinsic:
      ui_->calibration_type_text_browser->setText("Static Camera Moving Target on Wrist (Extrinsic + Intrinsic) [EXPERIMENTAL]");

    default:
      ui_->calibration_type_text_browser->setText("Welcome to the industrial_calibration_gui.");
      break;
  }
  calibration_type_ = static_cast<CalibrationType>(current_index);
}

// Manual data collection page
void CalibrationWidget::consoleLogInfo(const std::string &message)
{
  QString qmessage = QString::fromStdString("[INFO]: " + message);

  ui_->console_logger->setTextColor(QColor("green"));
  ui_->console_logger->append(qmessage);
}

void CalibrationWidget::consoleLogError(const std::string &message)
{
  QString qmessage = QString::fromStdString("[ERROR]: " + message);
  ui_->console_logger->setTextColor(QColor("red"));
  ui_->console_logger->append(qmessage);
}

void CalibrationWidget::outputLocationButton(void)
{
  save_data_directory_ = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
    "/home", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

  if (!save_data_directory_.endsWith("/"))
  {
    save_data_directory_.append("/");
  }

  ui_->output_location_line->setText(save_data_directory_); 
  CONSOLE_LOG_INFO("Data will be saved to: " << save_data_directory_.toStdString());
}

void CalibrationWidget::outputLocationLine(void)
{
  save_data_directory_ = ui_->output_location_line->text();

  if (!save_data_directory_.endsWith("/"))
  {
    save_data_directory_.append("/");
  }

  CONSOLE_LOG_INFO("Data will be saved to: " << save_data_directory_.toStdString());  
}

void CalibrationWidget::loadTargetLine(void)
{
  QString target_file = ui_->load_target_line->text();
  CONSOLE_LOG_INFO("Loading target from: " << target_file.toStdString());
  try
  {
    if (target_.loadTargetFromYAML(target_file.toStdString()))
    {
      CONSOLE_LOG_INFO("Target successfully loaded from: " 
        << target_file.toStdString());
      this->setTargetLines(target_);
      this->target_set_from_file_ = true;
    }
    else
    {
      CONSOLE_LOG_ERROR("Unable to load target from: " 
        << target_file.toStdString());
    }
  }
  catch (std::exception &ex)
  {
      CONSOLE_LOG_ERROR("Unable to load target from: " 
        << target_file.toStdString());
  }
}

void CalibrationWidget::loadTargetButton(void)
{
  QString target_file = QFileDialog::getOpenFileName(this, tr("Open File"),
    "/home", tr("YAML Files (*.yaml *.yml)"));
  ui_->load_target_line->setText(target_file);
  CONSOLE_LOG_INFO("Loading target from: " << target_file.toStdString());
  try
  {
    if (target_.loadTargetFromYAML(target_file.toStdString()))
    {
      CONSOLE_LOG_INFO("Target successfully loaded from: " << target_file.toStdString());
      this->setTargetLines(target_);
      this->target_set_from_file_ = true;
    }
    else
    {
      CONSOLE_LOG_ERROR("Unable to load target from: " 
        << target_file.toStdString());
    }  
  }
  catch (std::exception &ex)
  {
      CONSOLE_LOG_ERROR("Unable to load target from: " 
        << target_file.toStdString());
  }
}

void CalibrationWidget::setTargetLines(const industrial_calibration_libs::Target &target)
{
  // Round numbers with decimals
  std::stringstream target_circle_diameter_stream;
  target_circle_diameter_stream << std::fixed << std::setprecision(5) 
    << target.getDefinition().circle_diameter;
  std::stringstream target_point_spacing_stream;
  target_point_spacing_stream << std::fixed << std::setprecision(5)
    << target.getDefinition().spacing;
  
  // Setting line outputs
  QString target_name = QString::fromStdString(target.getDefinition().target_name);
  QString target_rows = QString::fromStdString(std::to_string(target.getDefinition().target_rows));
  QString target_cols = QString::fromStdString(std::to_string(target.getDefinition().target_cols));
  QString target_points = QString::fromStdString(std::to_string(
    target.getDefinition().target_rows*target.getDefinition().target_cols));
  QString target_circle_diameter = 
    QString::fromStdString(target_circle_diameter_stream.str());
  QString target_point_spacing = 
    QString::fromStdString(target_point_spacing_stream.str());

  ui_->target_name_line->setText(target_name);
  ui_->target_name_line->setReadOnly(true);
  ui_->target_type_combo_box->setCurrentIndex(target.getDefinition().target_type);
  ui_->target_type_combo_box->setDisabled(true);
  ui_->target_rows_line->setText(target_rows);
  ui_->target_rows_line->setReadOnly(true);
  ui_->target_cols_line->setText(target_cols);
  ui_->target_cols_line->setReadOnly(true);  
  ui_->target_points_line->setText(target_points);
  ui_->target_points_line->setReadOnly(true);
  ui_->target_circle_diameter_line->setText(target_circle_diameter);
  ui_->target_circle_diameter_line->setReadOnly(true);
  ui_->target_point_spacing_line->setText(target_point_spacing);
  ui_->target_point_spacing_line->setReadOnly(true);
}

void CalibrationWidget::updateTopicList(void)
{
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);

  std::vector<std::string> image_topic_list;
  std::vector<std::string> camera_info_topic_list;
  
  for (ros::master::V_TopicInfo::iterator it = topics.begin(); it != topics.end(); it++)
  {
    const ros::master::TopicInfo &info = *it;
    if (info.datatype == "sensor_msgs/Image")
    {
      image_topic_list.push_back(info.name);
    }
    else if (info.datatype == "sensor_msgs/CameraInfo")
    {
      camera_info_topic_list.push_back(info.name);
    }
  }
  if (image_topic_list.size() == 0)
  {
    CONSOLE_LOG_ERROR("There are no sensor_msg/Image topics being published");
  }
  if (camera_info_topic_list.size() == 0)
  {
    CONSOLE_LOG_ERROR("There are no sensor_msg/CameraInfo topics being published");
  }

  {
    std::lock_guard<std::mutex> lock(this->topic_list_mutex_);
    this->image_topic_list_.clear(); // Do these need to be cleared?
    this->camera_info_topic_list_.clear(); // Do these need to be cleared? 
    this->image_topic_list_ = image_topic_list;
    this->camera_info_topic_list_ = camera_info_topic_list;
  }
}

void CalibrationWidget::updateFrameList(void)
{
  std::vector<std::string> frame_list;
  tf_.getFrameStrings(frame_list);

  std::lock_guard<std::mutex> lock(this->frame_list_mutex_);
  this->frame_list_.clear();
  this->frame_list_ = frame_list;
}

void CalibrationWidget::setInputsButton(void)
{
  // Checking if any of the input fields are empty.
  if (this->checkEmptyLines()) {return;}

  // Generate a target definition if target isn't set from file.
  if (!target_set_from_file_)
  {
    industrial_calibration_libs::TargetDefinition target_definition;

    target_definition.target_name = ui_->target_name_line->text().toStdString();
    target_definition.target_type 
      = static_cast<std::size_t>(ui_->target_type_combo_box->currentIndex());
    target_definition.target_rows 
      = std::strtoul(ui_->target_rows_line->text().toStdString().c_str(), NULL, 0);
    target_definition.target_cols
      = std::strtoul(ui_->target_cols_line->text().toStdString().c_str(), NULL, 0);
    target_definition.circle_diameter
      = std::atof(ui_->target_circle_diameter_line->text().toStdString().c_str());
    target_definition.spacing
      = std::atof(ui_->target_point_spacing_line->text().toStdString().c_str());

    // Load target definition into target
    if (this->checkTarget(target_definition))
    {
      if (target_.loadTargetFromDefinition(target_definition))
      {
        CONSOLE_LOG_INFO("Successfully loaded target definition of name = " <<
          target_definition.target_name << ", of type = " 
          << target_definition.target_type <<
          ", with rows = " << target_definition.target_rows << ", and cols = " <<
          target_definition.target_cols << ", and total points = " << 
          target_definition.target_cols*target_definition.target_rows << 
          ", with circle diameter = " << target_definition.circle_diameter <<
          " (m), and point spacing = " << target_definition.spacing << " (m).");
      }
    }
    else {return;}
  }

  // Getting data from top and link name inputs
  std::string base_link = ui_->base_link_combo_box->currentText().toStdString();
  std::string tip_link = ui_->tip_link_combo_box->currentText().toStdString();
  std::string camera_frame = ui_->camera_calibration_frame_combo_box->currentText().toStdString();
  std::string image_topic = ui_->image_topic_combo_box->currentText().toStdString();
  std::string camera_info_topic = ui_->camera_info_topic_combo_box->currentText().toStdString();

  CONSOLE_LOG_INFO("Setting parameters | base_link = " << base_link <<
    ", tip_link = " << tip_link << ", camera_frame = " << camera_frame << 
    ", image_topic = " << image_topic << ", camera_info_topic = " << 
    camera_info_topic << ".");

  this->collecting_data_ = true;
  CONSOLE_LOG_INFO("Subscribing to image topic: " << image_topic << 
    " and publishing images to ~grid_image");
  this->collectData(base_link, tip_link, camera_frame, image_topic, camera_info_topic);
}

bool CalibrationWidget::checkEmptyLines(void)
{
  if (!target_set_from_file_)
  {
    if (ui_->target_name_line->text().isEmpty() ||
      ui_->target_rows_line->text().isEmpty() ||
      ui_->target_cols_line->text().isEmpty() ||
      ui_->target_points_line->text().isEmpty() ||
      ui_->target_circle_diameter_line->text().isEmpty() ||
      ui_->target_point_spacing_line->text().isEmpty())
    {
      CONSOLE_LOG_ERROR("Please set all target input fields or load a target from a YAML file!");
      return true;
    }
    if (ui_->target_type_combo_box->currentIndex() != 2)
    {
      CONSOLE_LOG_ERROR("The modified circle grid target type is the only " <<
        "target supported at this time");
      return true;
    }
  }
  if (ui_->base_link_combo_box->currentText().isEmpty() ||
    ui_->tip_link_combo_box->currentText().isEmpty() ||
    ui_->camera_calibration_frame_combo_box->currentText().isEmpty() ||
    ui_->image_topic_combo_box->currentText().isEmpty() ||
    ui_->camera_info_topic_combo_box->currentText().isEmpty())
  {
    CONSOLE_LOG_ERROR("Please set all link names and topics!");
    return true;
  }
  return false;
}

bool CalibrationWidget::checkTarget(const 
  industrial_calibration_libs::TargetDefinition &target_definition)
{
  if (target_definition.target_name.empty()) 
  {
    CONSOLE_LOG_ERROR("The 'Target Name' field is empty!");
    return false;
  }
  if (target_definition.target_type != 2) 
  {
    CONSOLE_LOG_ERROR("Target type must be set to Modified Circle Grid!");
    return false;
  }
  if (target_definition.target_rows == 0 || target_definition.target_cols == 0 ||
    target_definition.circle_diameter == 0 || target_definition.spacing == 0) 
  {
    CONSOLE_LOG_ERROR("One of the input fields has a non-numeric value or is set to zero!");
    return false;
  }
  return true;
}

void CalibrationWidget::collectData(const std::string &base_link, 
  const std::string &tip_link, const std::string &camera_frame, 
  const std::string &image_topic, const std::string &camera_info_topic)
{
  // Subscribe to the camera image
  try
  {
    camera_image_subscriber_ = it_.subscribe(image_topic, 1, 
      boost::bind(&CalibrationWidget::imageCallback, this, _1));  
  }
  catch (std::exception &ex)
  {
    CONSOLE_LOG_ERROR("Unable to subscribe to image topic: " << image_topic);
  }

  // Advertise output image from observation finder
  try
  {
    grid_image_publisher_ = it_.advertise("grid_image", 1);
  }
  catch (std::exception &ex)
  {
    CONSOLE_LOG_ERROR("Unable to advertise grid_image topic!");
  }

  // Subscribe to camera info message
  try
  {
    camera_info_subscriber_ = pnh_.subscribe(camera_info_topic, 1, 
      &CalibrationWidget::cameraInfoCallback, this);
  }
  catch (std::exception &ex)
  {
    CONSOLE_LOG_ERROR("Unable to subscribe to camera_info topic: " << camera_info_topic);
  }

  // Set link names
  {
    std::lock_guard<std::mutex> lock(this->camera_image_mutex_);
    this->base_link_ = base_link;
    this->tip_link_ = tip_link;
    this->camera_frame_ = camera_frame;
  }
}

void CalibrationWidget::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImageConstPtr msg_ptr;
  try
  {
    msg_ptr = cv_bridge::toCvCopy(msg);
  }
  catch (cv_bridge::Exception &ex)
  {
    CONSOLE_LOG_ERROR("Could not load image from message!");
  }
  cv::Mat raw_image;
  cv::cvtColor(msg_ptr->image, raw_image, CV_RGB2BGR);

  cv::Mat grid_image;
  cv::cvtColor(msg_ptr->image, grid_image, CV_RGB2BGR);

  {
    std::lock_guard<std::mutex> lock(this->camera_image_mutex_);
    raw_image.copyTo(this->camera_image_);
  }

  // Find circlegrid and draw grid
  cv::Mat display_image;
  if (!raw_image.empty() && !grid_image.empty())
  {
    if (this->drawGrid(grid_image))
    {
      display_image = grid_image;
    }
    else
    {
      display_image = raw_image;
    }

    // Publish the 'display_image'.
    cv_bridge::CvImage out_msg;
    out_msg.header = msg_ptr->header;
    out_msg.encoding = "bgr8"; // This should probably be set differently.
    out_msg.image = display_image;
    grid_image_publisher_.publish(out_msg.toImageMsg());
  }
  else
  {
    CONSOLE_LOG_ERROR("Input Image is Empty!");
  }  
}

bool CalibrationWidget::drawGrid(cv::Mat &image)
{
  industrial_calibration_libs::ObservationExtractor observation_extractor(target_);
  if (observation_extractor.extractObservation(image, image)) {return true;}
  else {return false;}
}

void CalibrationWidget::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
  CONSOLE_LOG_INFO("camera_info MSG Received");

  camera_info_ = *msg;

  // Shut down subscriber once we get a single message.
  camera_info_subscriber_.shutdown();
}

void CalibrationWidget::saveImageButton(void)
{
  // Get image
  cv::Mat image;

  {
    std::lock_guard<std::mutex> lock(this->camera_image_mutex_);
    camera_image_.copyTo(image); 
  }

  // Get tf transforms
  tf_.waitForTransform(base_link_, tip_link_, ros::Time(), ros::Duration(0.5));
  tf_.waitForTransform(tip_link_, camera_frame_, ros::Time(), ros::Duration(0.5));
  
  tf::StampedTransform base_link_to_tip_link;
  tf::StampedTransform tip_link_to_camera_frame;

  try
  {
    tf_.lookupTransform(base_link_, tip_link_, ros::Time(), base_link_to_tip_link);
    tf_.lookupTransform(tip_link_, camera_frame_, ros::Time(), tip_link_to_camera_frame);
  }
  catch (tf::TransformException &ex)
  {
    CONSOLE_LOG_ERROR("tf Exception: " << ex.what());
    CONSOLE_LOG_ERROR("Unable to capture tf data, data not saved");
    return;
  }

  observation_images_.push_back(image);
  base_to_tool_transforms_.push_back(base_link_to_tip_link);
  tool_to_camera_transforms_.push_back(tip_link_to_camera_frame);

  std::size_t index = observation_images_.size();
  CONSOLE_LOG_INFO("Observation #" << index - 1 << " Saved!");
}

void CalibrationWidget::startCalibrationButton(void)
{
  // Shutdown subscribers and publishers
  camera_image_subscriber_.shutdown();
  grid_image_publisher_.shutdown();
  bool save_data = true;

  std::string save_data_directory = this->save_data_directory_.toStdString();
  if (save_data_directory.empty()) 
  {
    // May want to save to a default directory in the future...
    // Maybe ~/.ros/industrial_calibration
    CONSOLE_LOG_ERROR("No save data directory specified, data will not" <<
      " be saved!");
    save_data = false;
  }

  if (save_data) {this->saveData(save_data_directory);}

  // Start Calibration (increase stackedWidget index)
  CONSOLE_LOG_INFO("Starting Calibration");
  if (tool_to_camera_transforms_match_)
  {
    switch (calibration_type_)
    {
      case welcome_screen:
        CONSOLE_LOG_ERROR("Calibration type not selected, I have no idea how"
          << " you made it this far...");
        break;

      // Only one supported right now...
      case camera_on_wrist_extrinsic:
        this->runCameraOnWristExtrinsic();
        break;

      case camera_on_wrist_intrinsic:
        this->runCameraOnWristIntrinsic();
        break;

      case camera_in_world_extrinsic:
        this->runCameraInWorldExtrinsic();
        break;

      case camera_in_world_intrinsic:
        this->runCameraInWorldIntrinsic();
        break;

      default:
        CONSOLE_LOG_ERROR("Please select a calibration type");
        break;
    }
  }
  else
  {
    CONSOLE_LOG_ERROR("Transforms do not match, restart the program");
    return;
  }
}

void CalibrationWidget::runCameraOnWristExtrinsic(void)
{
  CONSOLE_LOG_INFO("Running static target moving camera on wrist!");

  // Initial Camera Intrinsics from camera_info.
  double intrinsics[4];
  intrinsics[0] = camera_info_.K[0]; // fx
  intrinsics[1] = camera_info_.K[4]; // fy
  intrinsics[2] = camera_info_.K[2]; // cx
  intrinsics[3] = camera_info_.K[5]; // cy

  // Setting seed parameters for camera extrinsics from tool to camera_frame.
  industrial_calibration_libs::Pose6D tool_to_camera;
  tool_to_camera = this->tfToPose6D(tool_to_camera_transforms_[0]);

  // Setting a seed guess for target pose.
  // TODO(gChiou): Add marker for user to drag around.
  industrial_calibration_libs::Pose6D target_pose;
  target_pose.setOrigin(0.0, 0.0, 0.0);
  target_pose.setEulerZYX(0.0, 0.0, 0.0);

  // Convert base_to_tool stamped transforms to Pose6D.
  std::vector<industrial_calibration_libs::Pose6D> link_poses;
  link_poses.reserve(base_to_tool_transforms_.size());
  for (std::size_t i = 0; i < base_to_tool_transforms_.size(); i++)
  {
    link_poses.push_back(this->tfToPose6D(base_to_tool_transforms_[i]));
  }

  // Convert images to observations
  industrial_calibration_libs::ObservationData observation_data;
  observation_data = this->cvMatToObservations(observation_images_);

  // Set Params
  industrial_calibration_libs::CameraOnWristExtrinsicParams params;
  params.intrinsics = industrial_calibration_libs::IntrinsicsPartial(
    intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]);
  params.tool_to_camera = industrial_calibration_libs::Extrinsics(tool_to_camera.getInverse());
  params.target_to_base = industrial_calibration_libs::Extrinsics(target_pose);
  params.base_to_tool = link_poses;

  // Running calibration and printing results to console
  industrial_calibration_libs::CameraOnWristExtrinsic calibration(observation_data,
  target_, params);

  calibration.runCalibration();
  calibration.displayCovariance();

  industrial_calibration_libs::CameraOnWristExtrinsic::Result results = calibration.getResults();

  // Remove this later...
  ROS_INFO_STREAM("Extrinsic Parameters");
  ROS_INFO_STREAM("Translation x: " << results.extrinsics[3]);
  ROS_INFO_STREAM("Translation y: " << results.extrinsics[4]);
  ROS_INFO_STREAM("Translation z: " << results.extrinsics[5]);
  ROS_INFO_STREAM("Rotation x: " << results.extrinsics[0]);
  ROS_INFO_STREAM("Rotation y: " << results.extrinsics[1]);
  ROS_INFO_STREAM("Rotation z: " << results.extrinsics[2]);

  ROS_INFO_STREAM("Target to Base");
  ROS_INFO_STREAM("Translation x: " << results.target_to_base[3]);
  ROS_INFO_STREAM("Translation y: " << results.target_to_base[4]);
  ROS_INFO_STREAM("Translation z: " << results.target_to_base[5]);
  ROS_INFO_STREAM("Rotation x: " << results.target_to_base[0]);
  ROS_INFO_STREAM("Rotation y: " << results.target_to_base[1]);
  ROS_INFO_STREAM("Rotation z: " << results.target_to_base[2]);

  ROS_INFO_STREAM("Initial Cost: " << calibration.getInitialCost());
  ROS_INFO_STREAM("Final Cost: " << calibration.getFinalCost());  
}

void CalibrationWidget::runCameraOnWristIntrinsic(void)
{
  CONSOLE_LOG_ERROR("Not Supported Yet!");
}

void CalibrationWidget::runCameraInWorldExtrinsic(void)
{
  CONSOLE_LOG_ERROR("Not Supported Yet!");
}

void CalibrationWidget::runCameraInWorldIntrinsic(void)
{
  CONSOLE_LOG_ERROR("Not Supported Yet!");
}

industrial_calibration_libs::Pose6D 
CalibrationWidget::tfToPose6D(const tf::StampedTransform &t)
{
  double tx = t.getOrigin().getX();
  double ty = t.getOrigin().getY();
  double tz = t.getOrigin().getZ();

  double qx = t.getRotation().getX();
  double qy = t.getRotation().getY();
  double qz = t.getRotation().getZ();
  double qw = t.getRotation().getW();

  industrial_calibration_libs::Pose6D pose;
  pose.setOrigin(tx, ty, tz);
  pose.setQuaternion(qx, qy, qz, qw);

  return pose;
}

industrial_calibration_libs::ObservationData 
CalibrationWidget::cvMatToObservations(const std::vector<cv::Mat> &images)
{
  industrial_calibration_libs::ObservationExtractor observation_extractor(target_);
  for (std::size_t i = 0; i < images.size(); i++)
  {
    cv::Mat output_image;
    observation_extractor.extractObservation(images[i]);
  }

  return observation_extractor.getObservationData();
}

void CalibrationWidget::saveData(const std::string &directory)
{
  // Check if number of observations matches number of transforms
  if (observation_images_.size() != base_to_tool_transforms_.size()
    && observation_images_.size() != tool_to_camera_transforms_.size())
  {
    CONSOLE_LOG_ERROR("The number of images is not equal to " <<
      "the number of transforms");
    return;
  }

  // Check if all tool0 to camera_frame transfroms match
  tool_to_camera_transforms_match_ = true;
  for (std::size_t i = 0; i < tool_to_camera_transforms_.size(); i++)
  {
    if (tool_to_camera_transforms_[0] != tool_to_camera_transforms_[i])
    {
      tool_to_camera_transforms_match_ = false;
      break;
    }
  }
  if (!tool_to_camera_transforms_match_)
  {
    CONSOLE_LOG_ERROR("The transforms between link: " << tip_link_ 
      << " and link: " << camera_frame_ << " don't match!"
      << " These transforms should remain constant");
    return;
  }

  std::string date_time = this->getDateTimeString();

  std::string data_directory_string = directory + date_time;
  std::string image_directory_string = data_directory_string + "/images";
  std::string tf_directory_string = data_directory_string + "/tf";

  // Use boost filesystem to create these directories.
  boost::filesystem::path data_directory(data_directory_string);
  boost::filesystem::path image_directory(image_directory_string);
  boost::filesystem::path tf_directory(tf_directory_string);

  try
  {
    boost::filesystem::create_directory(data_directory);
    boost::filesystem::create_directory(image_directory);
    boost::filesystem::create_directory(tf_directory);
  }
  catch (std::exception &ex)
  {
    CONSOLE_LOG_ERROR("Unable to create directory to save data," <<
      " please specify a directory that exists"); 
  }

  CONSOLE_LOG_INFO("Saving calibration data to: " << data_directory_string);

  // Compression params for png (set to none)
  std::vector<int> png_params;
  png_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  png_params.push_back(0);

  // Saving images
  for (std::size_t i = 0; i < observation_images_.size(); i++)
  {
    try
    {
      std::string file_name = image_directory_string + "/" 
        + std::to_string(i) + ".png";
      cv::imwrite(file_name, observation_images_[i], png_params);
      CONSOLE_LOG_INFO(file_name << " has been saved.");
    }
    catch (std::exception &ex)
    {
      CONSOLE_LOG_ERROR("Failed to save image: " << std::to_string(i) 
        << ".png to " << image_directory_string);
    }
  }

  // Saving transforms from base to tool0
  for (std::size_t i = 0; i < base_to_tool_transforms_.size(); i++)
  {
    try
    {
      std::string file_name = tf_directory_string + "/" + std::to_string(i) 
        + ".yaml";
      this->tfToYAML(file_name, base_to_tool_transforms_[i], base_link_,
        tip_link_);
    }
    catch (std::exception &ex)
    {
      CONSOLE_LOG_ERROR("Failed to save transform between " << base_link_ 
        << " and " << tip_link_ << " to: " << std::to_string(i)
        << ".yaml in " << tf_directory_string);
    }
  }

  // Saving single transform between tool0 and camera_frame
  try
  {
    std::string file_name = data_directory_string + "/" + tip_link_ + "_to_"
      + camera_frame_ + ".yaml";
    this->tfToYAML(file_name, tool_to_camera_transforms_[0], tip_link_,
      camera_frame_);
  }
  catch (std::exception &ex)
  {
      CONSOLE_LOG_ERROR("Failed to save transform between " << tip_link_ 
        << " and " << camera_frame_ << " to: " << data_directory_string 
        << "/" + tip_link_ << "_to_" << camera_frame_ + ".yaml");    
  }
}

std::string CalibrationWidget::getDateTimeString(void)
{
  // Get the date-time to use as a directory name
  boost::posix_time::ptime boost_posix_time = ros::Time::now().toBoost();
  std::string date_time = boost::posix_time::to_simple_string(boost_posix_time);
  std::replace(date_time.begin(), date_time.end(), ':', '-');
  std::replace(date_time.begin(), date_time.end(), ' ', '-');
  date_time = date_time.substr(0, date_time.find("."));

  return date_time;
}

void CalibrationWidget::tfToYAML(const std::string &filename, 
  const tf::StampedTransform &transform, const std::string &from_link, 
  const std::string &to_link)
{
  std::string name = from_link + "_to_" + to_link;

  std::vector<double> translation; translation.resize(3);
  std::vector<double> quaternion; quaternion.resize(4);

  translation[0] = transform.getOrigin().getX();
  translation[1] = transform.getOrigin().getY();
  translation[2] = transform.getOrigin().getZ();

  quaternion[0] = transform.getRotation().getX();
  quaternion[1] = transform.getRotation().getY();
  quaternion[2] = transform.getRotation().getZ();
  quaternion[3] = transform.getRotation().getW();

  YAML::Emitter out;
  out << YAML::BeginMap;
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
  out << YAML::EndMap;

  if (out.good())
  {
    std::ofstream yaml_file(filename);
    yaml_file << out.c_str();
    CONSOLE_LOG_INFO(filename << " has been saved.");
  }
  else
  {
    CONSOLE_LOG_ERROR("YAML file is bad!");
  }
}

void CalibrationWidget::refreshComboBoxes(void)
{
  this->updateTopicList();
  this->updateFrameList();

  // Set frame combo box data
  ui_->base_link_combo_box->clear();
  ui_->tip_link_combo_box->clear();
  ui_->camera_calibration_frame_combo_box->clear();
  if (this->frame_list_.size() > 0)
  {
    QStringList frame_list;
    for (std::size_t i = 0; i < this->frame_list_.size(); i++)
    {
      frame_list.append(QString::fromStdString(this->frame_list_[i]));
    }
    ui_->base_link_combo_box->addItems(frame_list);
    ui_->tip_link_combo_box->addItems(frame_list);
    ui_->camera_calibration_frame_combo_box->addItems(frame_list);
  }

  // Set image_topic combo box data
  ui_->image_topic_combo_box->clear();
  if (this->image_topic_list_.size() > 0)
  {
    QStringList image_topic_list;
    for (std::size_t i = 0; i < this->image_topic_list_.size(); i++)
    {
      image_topic_list.append(QString::fromStdString(this->image_topic_list_[i]));
    }
    ui_->image_topic_combo_box->addItems(image_topic_list);
  }

  // Set camera_info_topic combo box data
  ui_->camera_info_topic_combo_box->clear();
  if (this->camera_info_topic_list_.size() > 0)
  {
    QStringList camera_info_topic_list;
    for (std::size_t i = 0; i < this->camera_info_topic_list_.size(); i++)
    {
      camera_info_topic_list.append(QString::fromStdString(this->camera_info_topic_list_[i]));
    }
    ui_->camera_info_topic_combo_box->addItems(camera_info_topic_list);
  }
}

bool operator!=(const tf::StampedTransform &t1, const tf::StampedTransform &t2)
{
  if (t1.getOrigin().getX() == t2.getOrigin().getX() 
    || t1.getOrigin().getY() == t2.getOrigin().getY()
    || t1.getOrigin().getZ() == t2.getOrigin().getZ()
    || t1.getRotation().getX() == t2.getRotation().getX()
    || t1.getRotation().getY() == t2.getRotation().getY()
    || t1.getRotation().getZ() == t2.getRotation().getZ()
    || t1.getRotation().getW() == t2.getRotation().getW())
  {
    return false;
  }
  else {return true;}
}
} // namespace industrial_calibration_gui
