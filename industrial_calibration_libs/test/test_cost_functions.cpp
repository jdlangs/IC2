#include <test_utils.h>

typedef std::vector<double> JointStates;
typedef std::vector<double> Translation;
typedef std::vector<double> Quaternion;
typedef std::vector<double> RotationRad;
typedef std::vector<double> RotationDeg;

struct LinkData
{
  JointStates joint_states;
  Translation translation;
  Quaternion rotation_quat;
  RotationRad rotation_rad;
  RotationDeg rotation_deg;
};


bool parseYAML(const YAML::Node &node, const std::string &var_name, 
  std::vector<double> &var_value)
{
  var_value.clear();
  if (node[var_name])
  {
    const YAML::Node n = node[var_name];
    var_value.reserve(n.size());
    for (std::size_t i = 0; i < n.size(); i++)
    {
      double value = n[i].as<double>();
      var_value.push_back(value);
    }
    if (var_value.size() == n.size()) {return true;}
  }
  return false;
}

bool loadLinkData(const std::size_t &index, const std::string &path,
  LinkData *link_data)
{
  bool success = true;
  std::string file_path = path + std::to_string(index+1) + ".yaml";

  YAML::Node data_yaml;
  try
  {
    data_yaml = YAML::LoadFile(file_path);
    if (!data_yaml["joints"]) {return false;}
  }
  catch (YAML::BadFile &bf) {return false;}\

  success &= parseYAML(data_yaml, "joints", link_data->joint_states);
  success &= parseYAML(data_yaml, "translation", link_data->translation);
  success &= parseYAML(data_yaml, "rotation_quat", link_data->rotation_quat);
  success &= parseYAML(data_yaml, "rotation_rad", link_data->rotation_rad);
  success &= parseYAML(data_yaml, "rotation_deg", link_data->rotation_deg);

  return success;
}

void printVector(const std::vector<double> &vec)
{
  for (std::size_t i = 0; i < vec.size(); i++)
  {
    CONSOLE_OUTPUT(vec[i]);
  }
}

bool convertToPose6D(const std::vector<LinkData> &link_data, 
  std::vector<industrial_calibration_libs::Pose6D> *link_poses)
{
  link_poses->reserve(link_data.size());

  for (std::size_t i = 0; i < link_data.size(); i++)
  {
    industrial_calibration_libs::Pose6D link_pose;
    double tx = link_data[i].translation[0];
    double ty = link_data[i].translation[1];
    double tz = link_data[i].translation[2];
    double qx = link_data[i].rotation_quat[0];
    double qy = link_data[i].rotation_quat[1];
    double qz = link_data[i].rotation_quat[2];
    double qw = link_data[i].rotation_quat[3];
    link_pose.setOrigin(tx, ty, tz);
    link_pose.setQuaternion(qz, qy, qz, qw);

    link_poses->push_back(link_pose);
  }

  if (link_poses->size() == link_data.size()) {return true;}
  else {return false;}
}

TEST(CostFunctions, intrinsic_calibration)
{
  // Load Target Data
  industrial_calibration_libs::Target my_target;
  EXPECT_TRUE(my_target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5.yaml"));

  // Load Calibration Images
  const std::size_t num_images = 15;
  std::vector<cv::Mat> cal_images;
  cal_images.reserve(num_images);
  std::string cal_image_path = "mcircles_7x5/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i+1) + ".jpg";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    ASSERT_TRUE(!image.empty());
    ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    cal_images.push_back(image);    
  }

  // Extract Observations
  industrial_calibration_libs::ObservationExtractor observation_extractor(cal_images, my_target);
  ASSERT_TRUE(observation_extractor.extractObservations());

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData();  

  EXPECT_EQ(observation_data.size(), num_images);

  // Load Link Data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp_link_data;
    EXPECT_TRUE(loadLinkData(i, "mcircles_7x5/tf/", &temp_link_data));
    link_data.push_back(temp_link_data);
  }

  EXPECT_TRUE(link_data.size() == num_images);
  for (std::size_t i = 0; i < link_data.size(); i++)
  {
    EXPECT_TRUE(link_data[i].joint_states.size() == 6);
    EXPECT_TRUE(link_data[i].translation.size() == 3);
    EXPECT_TRUE(link_data[i].rotation_quat.size() == 4);
    EXPECT_TRUE(link_data[i].rotation_rad.size() == 3);
    EXPECT_TRUE(link_data[i].rotation_deg.size() == 3);
  }

  // Note(gChiou): Prints out all recorded poses, leaving commented 
  // for debugging purposes.
#if 0
  for (std::size_t i = 0; i < num_images; i++)
  {
    CONSOLE_OUTPUT("Link Data for Image: " << i+1);
    CONSOLE_OUTPUT("Joint States");
    printVector(link_data[i].joint_states);
    CONSOLE_OUTPUT("Translation");
    printVector(link_data[i].translation);
    CONSOLE_OUTPUT("Quaternion");
    printVector(link_data[i].rotation_quat);
    CONSOLE_OUTPUT("Rotation Rad");
    printVector(link_data[i].rotation_rad);
    CONSOLE_OUTPUT("Rotation Deg");
    printVector(link_data[i].rotation_deg);
  }
#endif

  // Convert Link Data to a vector of Pose6D poses
  std::vector<industrial_calibration_libs::Pose6D> link_poses;
  EXPECT_TRUE(convertToPose6D(link_data, &link_poses));

  industrial_calibration_libs::IntrinsicCalibration 
    intrinsic_cal(observation_data, my_target, link_poses);

  EXPECT_TRUE(intrinsic_cal.calibrate());

  industrial_calibration_libs::IntrinsicResults results = intrinsic_cal.getResults();

  // Note(gChiou): Prints out calibration results
#if 1
  CONSOLE_OUTPUT("Extrinsic Parameters");
  for (std::size_t i = 0; i < 6; i++)
  {
    CONSOLE_OUTPUT(results.extrinsics[i]);
  }
  CONSOLE_OUTPUT("Intrinsic Parameters");
  for (std::size_t i = 0; i < 9; i++)
  {
    CONSOLE_OUTPUT(results.intrinsics[i]);
  }
  CONSOLE_OUTPUT("Target to World");
  for (std::size_t i = 0; i < 6; i++)
  {
    CONSOLE_OUTPUT(results.target_to_world[i]);
  }
#endif
}

#if 1
TEST(CostFunctions, extrinsic_calibration)
{
  // Load Target Data
  industrial_calibration_libs::Target my_target;
  EXPECT_TRUE(my_target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5.yaml"));

  // Load Calibration Images
  const std::size_t num_images = 15;
  std::vector<cv::Mat> cal_images;
  cal_images.reserve(num_images);
  std::string cal_image_path = "mcircles_7x5/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i+1) + ".jpg";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    ASSERT_TRUE(!image.empty());
    ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    cal_images.push_back(image);    
  }

  // Extract Observations
  industrial_calibration_libs::ObservationExtractor observation_extractor(cal_images, my_target);
  ASSERT_TRUE(observation_extractor.extractObservations());

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData();  

  EXPECT_EQ(observation_data.size(), num_images);

  // Load Link Data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp_link_data;
    EXPECT_TRUE(loadLinkData(i, "mcircles_7x5/tf/", &temp_link_data));
    link_data.push_back(temp_link_data);
  }

  EXPECT_TRUE(link_data.size() == num_images);
  for (std::size_t i = 0; i < link_data.size(); i++)
  {
    EXPECT_TRUE(link_data[i].joint_states.size() == 6);
    EXPECT_TRUE(link_data[i].translation.size() == 3);
    EXPECT_TRUE(link_data[i].rotation_quat.size() == 4);
    EXPECT_TRUE(link_data[i].rotation_rad.size() == 3);
    EXPECT_TRUE(link_data[i].rotation_deg.size() == 3);
  }

  // Convert Link Data to a vector of Pose6D poses
  double intrinsics[4];
  for (std::size_t i = 0; i < 4; i++) {intrinsics[i] = 150;}

  std::vector<industrial_calibration_libs::Pose6D> link_poses;
  EXPECT_TRUE(convertToPose6D(link_data, &link_poses));

  industrial_calibration_libs::ExtrinsicCalibration 
    extrinsic_cal(observation_data, my_target, link_poses, intrinsics);

  EXPECT_TRUE(extrinsic_cal.calibrate());

  industrial_calibration_libs::ExtrinsicResults results = extrinsic_cal.getResults();

  // Note(gChiou): Prints out calibration results
#if 1
  CONSOLE_OUTPUT("Extrinsic Parameters");
  for (std::size_t i = 0; i < 6; i++)
  {
    CONSOLE_OUTPUT(results.extrinsics[i]);
  }
  CONSOLE_OUTPUT("Target to World");
  for (std::size_t i = 0; i < 6; i++)
  {
    CONSOLE_OUTPUT(results.target_to_world[i]);
  }
#endif    
}
#endif
