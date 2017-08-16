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

  // Start page
  connect(ui_->instructions_checkbox, SIGNAL(stateChanged(int)),
    this, SLOT(instructionsCheckbox()));
  connect(ui_->start_calibration_button, SIGNAL(clicked()), 
    this, SLOT(startCalibrationButton()));
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
  connect(ui_->set_inputs_button, SIGNAL(clicked()),
    this, SLOT(setInputsButton()));
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

void CalibrationWidget::startCalibrationButton(void)
{
  if (this->instructions_checkbox_state_)
  {
    ui_->stackedWidget->setCurrentIndex(1);
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
    case 0:
      ui_->calibration_type_text_browser->setText("Welcome to the industrial_calibration_gui.");
      break;

    case 1:
      ui_->calibration_type_text_browser->setText("Static Target Moving Camera on Wrist (Extrinsic)");
      break;

    case 2:
      ui_->calibration_type_text_browser->setText("Static Target Moving Camera on Wrist (Extrinsic + Intrinsic) [EXPERIMENTAL]");
      break;

    case 3:
      ui_->calibration_type_text_browser->setText("Static Camera Moving Target on Wrist (Extrinsic)");

    case 4:
      ui_->calibration_type_text_browser->setText("Static Camera Moving Target on Wrist (Extrinsic + Intrinsic) [EXPERIMENTAL]");

    default:
      ui_->calibration_type_text_browser->setText("Welcome to the industrial_calibration_gui.");
      break;
  }
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
  ui_->output_location_line->setText(save_data_directory_); 
  CONSOLE_LOG_INFO("Data will be saved to: " << save_data_directory_.toStdString());
}

void CalibrationWidget::outputLocationLine(void)
{
  save_data_directory_ = ui_->output_location_line->text();
  CONSOLE_LOG_INFO("Data will be saved to: " << save_data_directory_.toStdString());  
}

void CalibrationWidget::loadTargetLine(void)
{
  QString target_file = ui_->load_target_line->text();
  CONSOLE_LOG_INFO("Loading target from: " << target_file.toStdString());
  if (target_.loadTargetFromYAML(target_file.toStdString()))
  {
    CONSOLE_LOG_INFO("Target successfully loaded from: " << target_file.toStdString());
    this->setTargetLines(target_);
    this->target_set_from_file_ = true;
  }
  else
  {
    CONSOLE_LOG_ERROR("Unable to load target from: " << target_file.toStdString());
  }

}

void CalibrationWidget::loadTargetButton(void)
{
  QString target_file = QFileDialog::getOpenFileName(this, tr("Open File"),
    "/home", tr("YAML Files (*.yaml *.yml)"));
  ui_->load_target_line->setText(target_file);
  CONSOLE_LOG_INFO("Loading target from: " << target_file.toStdString());
  if (target_.loadTargetFromYAML(target_file.toStdString()))
  {
    CONSOLE_LOG_INFO("Target successfully loaded from: " << target_file.toStdString());
    this->setTargetLines(target_);
    this->target_set_from_file_ = true;
  }
  else
  {
    CONSOLE_LOG_ERROR("Unable to load target from: " << target_file.toStdString());
  }  
}

void CalibrationWidget::setTargetLines(const industrial_calibration_libs::Target &target)
{
  // Round numbers with decimals
  std::stringstream target_circle_diameter_stream;
  target_circle_diameter_stream << std::fixed << std::setprecision(5) 
    << target.getData().circle_diameter;
  std::stringstream target_point_spacing_stream;
  target_point_spacing_stream << std::fixed << std::setprecision(5)
    << target.getData().spacing;
  
  // Setting line outputs
  QString target_name = QString::fromStdString(target.getData().target_name);
  QString target_rows = QString::fromStdString(std::to_string(target.getData().target_rows));
  QString target_cols = QString::fromStdString(std::to_string(target.getData().target_cols));
  QString target_points = 
    QString::fromStdString(std::to_string(target.getData().target_points));
  QString target_circle_diameter = 
    QString::fromStdString(target_circle_diameter_stream.str());
  QString target_point_spacing = 
    QString::fromStdString(target_point_spacing_stream.str());

  ui_->target_name_line->setText(target_name);
  ui_->target_name_line->setReadOnly(true);
  ui_->target_type_combo_box->setCurrentIndex(target.getData().target_type);
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
    target_definition.target_points
      = std::strtoul(ui_->target_points_line->text().toStdString().c_str(), NULL, 0);
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
          target_definition.target_points << ", with circle diameter = " <<
          target_definition.circle_diameter << " (m), and point spacing = " <<
          target_definition.spacing << " (m).");
      }
    }
    else {return;}
  }

  // Getting data from top and link name inputs
  std::string base_link = ui_->base_link_line->text().toStdString();
  std::string tip_link = ui_->tip_link_line->text().toStdString();
  std::string camera_frame = ui_->camera_calibration_frame_line->text().toStdString();
  std::string image_topic = ui_->image_topic_line->text().toStdString();
  std::string camera_info_topic = ui_->camera_info_topic_line->text().toStdString();

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
  if (ui_->base_link_line->text().isEmpty() ||
    ui_->tip_link_line->text().isEmpty() ||
    ui_->camera_calibration_frame_line->text().isEmpty() ||
    ui_->image_topic_line->text().isEmpty() ||
    ui_->camera_info_topic_line->text().isEmpty())
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
    target_definition.target_points == 0 || target_definition.circle_diameter == 0 ||
    target_definition.spacing == 0) 
  {
    CONSOLE_LOG_ERROR("One of the input fields has a non-numeric value or is set to zero!");
    return false;
  }
  if (target_definition.target_rows*target_definition.target_cols != 
    target_definition.target_points) 
  {
    CONSOLE_LOG_ERROR("Target rows * cols do not match total number of points!");
    return false;
  }
  return true;
}

void CalibrationWidget::collectData(const std::string &base_link, 
  const std::string &tip_link, const std::string &camera_frame, 
  const std::string &image_topic, const std::string &camera_info_topic)
{
  camera_image_subscriber_ = it_.subscribe(image_topic, 1, 
    boost::bind(&CalibrationWidget::imageCallback, this, _1));

  grid_image_publisher_ = it_.advertise("grid_image", 1);
}

void CalibrationWidget::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  ROS_INFO_STREAM("Image MSG Received!");

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

  // Find circlegrid and draw grid
  cv::Mat display_image;
  if (!raw_image.empty() && !grid_image.empty())
  {
    if (this->drawGrid(grid_image))
    {
      ROS_INFO_STREAM("Circle Grid Found!");
      display_image = grid_image;
    }
    else
    {
      ROS_INFO_STREAM("No Circle Grid FOund!");
      display_image = raw_image;
    }
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
} // namespace industrial_calibration_gui

