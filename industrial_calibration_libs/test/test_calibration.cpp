#include <test_utils.h>

#define TEST_1
#define TEST_2
#define TEST_3
#define TEST_4

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
  catch (YAML::BadFile &bf) {return false;}

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
    link_pose.setQuaternion(qx, qy, qz, qw);

    link_poses->push_back(link_pose);
  }

  if (link_poses->size() == link_data.size()) {return true;}
  else {return false;}
}

#ifdef TEST_1
TEST(Calibration, MovingCameraOnWristStaticTargetExtrinsic_dataset_1)
{
  // Load Target Data
  industrial_calibration_libs::Target target;
  EXPECT_TRUE(target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5.yaml"));

  // Load Calibration Images
  const std::size_t num_images = 15;
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = "mcircles_7x5/dataset_1/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i+1) + ".jpg";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    ASSERT_TRUE(!image.empty());
    ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    calibration_images.push_back(image);    
  }

  // Extract Observations
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    cv::Mat output_image;
    ASSERT_TRUE(observation_extractor.extractObservation(calibration_images[i],
      output_image));
  }

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData(); 

  EXPECT_EQ(observation_data.size(), num_images);

  // Load Link Data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp_link_data;
    EXPECT_TRUE(loadLinkData(i, "mcircles_7x5/dataset_1/tf/", &temp_link_data));
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
  intrinsics[0] = 944.72;
  intrinsics[1] = 940.92;
  intrinsics[2] = 925.56;
  intrinsics[3] = 518.89;

  // Set camera extrinsics seed
  industrial_calibration_libs::Pose6D link_6_to_camera;
  link_6_to_camera.setOrigin(0.125, 0.000, 0.091);
  link_6_to_camera.setQuaternion(0.500, 0.500, 0.500, 0.500);

  double extrinsics[6];
  extrinsics[0] = link_6_to_camera.ax;
  extrinsics[1] = link_6_to_camera.ay;
  extrinsics[2] = link_6_to_camera.az;
  extrinsics[3] = link_6_to_camera.x;
  extrinsics[4] = link_6_to_camera.y;
  extrinsics[5] = link_6_to_camera.z;

  double target_to_base[6] = {0};

  target_to_base[3] = 0.75;
  target_to_base[4] = 0.1;
  target_to_base[5] = 0.1;  

  std::vector<industrial_calibration_libs::Pose6D> link_poses;
  EXPECT_TRUE(convertToPose6D(link_data, &link_poses));

  industrial_calibration_libs::MovingCameraOnWristStaticTargetExtrinsic calibration(observation_data, target);

  calibration.initKnownValues(link_poses, intrinsics);
  calibration.initSeedValues(extrinsics, target_to_base);

  EXPECT_TRUE(calibration.runCalibration());

  calibration.displayCovariance();

  industrial_calibration_libs::MovingCameraOnWristStaticTargetExtrinsic::Result results = calibration.getResults();

  // Note(gChiou): Prints out calibration results
#if 1
  CONSOLE_OUTPUT("Extrinsic Parameters");
  CONSOLE_OUTPUT("Translation x: " << results.extrinsics[3]);
  CONSOLE_OUTPUT("Translation y: " << results.extrinsics[4]);
  CONSOLE_OUTPUT("Translation z: " << results.extrinsics[5]);
  CONSOLE_OUTPUT("Rotation x: " << results.extrinsics[0]);
  CONSOLE_OUTPUT("Rotation y: " << results.extrinsics[1]);
  CONSOLE_OUTPUT("Rotation z: " << results.extrinsics[2]);

  CONSOLE_OUTPUT("Target to Base");
  CONSOLE_OUTPUT("Translation x: " << results.target_to_base[3]);
  CONSOLE_OUTPUT("Translation y: " << results.target_to_base[4]);
  CONSOLE_OUTPUT("Translation z: " << results.target_to_base[5]);
  CONSOLE_OUTPUT("Rotation x: " << results.target_to_base[0]);
  CONSOLE_OUTPUT("Rotation y: " << results.target_to_base[1]);
  CONSOLE_OUTPUT("Rotation z: " << results.target_to_base[2]);
#endif    
}
#endif

#ifdef TEST_2
TEST(Calibration, MovingCameraOnWristStaticTargetExtrinsic_dataset_2)
{
  // Load Target Data
  industrial_calibration_libs::Target target;
  EXPECT_TRUE(target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5.yaml"));

  // Load Calibration Images
  const std::size_t num_images = 24;
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = "mcircles_7x5/dataset_2/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i+1) + ".jpg";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    ASSERT_TRUE(!image.empty());
    ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    calibration_images.push_back(image);    
  }

  // Extract Observations
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    cv::Mat output_image;
    ASSERT_TRUE(observation_extractor.extractObservation(calibration_images[i],
      output_image));
  }

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData(); 

  EXPECT_EQ(observation_data.size(), num_images);

  // Load Link Data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp_link_data;
    EXPECT_TRUE(loadLinkData(i, "mcircles_7x5/dataset_2/tf/", &temp_link_data));
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
  intrinsics[0] = 944.72;
  intrinsics[1] = 940.92;
  intrinsics[2] = 925.56;
  intrinsics[3] = 518.89;

  // Set camera extrinsics seed
  industrial_calibration_libs::Pose6D link_6_to_camera;
  link_6_to_camera.setOrigin(0.125, 0.000, 0.091);
  link_6_to_camera.setQuaternion(0.500, 0.500, 0.500, 0.500);

  double extrinsics[6];
  extrinsics[0] = link_6_to_camera.ax;
  extrinsics[1] = link_6_to_camera.ay;
  extrinsics[2] = link_6_to_camera.az;
  extrinsics[3] = link_6_to_camera.x;
  extrinsics[4] = link_6_to_camera.y;
  extrinsics[5] = link_6_to_camera.z;

  double target_to_base[6] = {0};

  target_to_base[3] = 0.75;
  target_to_base[4] = 0.1;
  target_to_base[5] = 0.1;

  std::vector<industrial_calibration_libs::Pose6D> link_poses;
  EXPECT_TRUE(convertToPose6D(link_data, &link_poses));

  industrial_calibration_libs::MovingCameraOnWristStaticTargetExtrinsic calibration(observation_data, target);

  calibration.initKnownValues(link_poses, intrinsics);
  calibration.initSeedValues(extrinsics, target_to_base);

  EXPECT_TRUE(calibration.runCalibration());

  calibration.displayCovariance();

  industrial_calibration_libs::MovingCameraOnWristStaticTargetExtrinsic::Result results = calibration.getResults();

  // Note(gChiou): Prints out calibration results
#if 1
  CONSOLE_OUTPUT("Extrinsic Parameters");
  CONSOLE_OUTPUT("Translation x: " << results.extrinsics[3]);
  CONSOLE_OUTPUT("Translation y: " << results.extrinsics[4]);
  CONSOLE_OUTPUT("Translation z: " << results.extrinsics[5]);
  CONSOLE_OUTPUT("Rotation x: " << results.extrinsics[0]);
  CONSOLE_OUTPUT("Rotation y: " << results.extrinsics[1]);
  CONSOLE_OUTPUT("Rotation z: " << results.extrinsics[2]);

  CONSOLE_OUTPUT("Target to Base");
  CONSOLE_OUTPUT("Translation x: " << results.target_to_base[3]);
  CONSOLE_OUTPUT("Translation y: " << results.target_to_base[4]);
  CONSOLE_OUTPUT("Translation z: " << results.target_to_base[5]);
  CONSOLE_OUTPUT("Rotation x: " << results.target_to_base[0]);
  CONSOLE_OUTPUT("Rotation y: " << results.target_to_base[1]);
  CONSOLE_OUTPUT("Rotation z: " << results.target_to_base[2]);
#endif    
}
#endif

#ifdef TEST_3
TEST(Calibration, MovingCameraOnWristStaticTargetIntrinsic_dataset_1)
{
  // Load Target Data
  industrial_calibration_libs::Target target;
  EXPECT_TRUE(target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5.yaml"));

  // Load Calibration Images
  const std::size_t num_images = 15;
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = "mcircles_7x5/dataset_1/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i+1) + ".jpg";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    ASSERT_TRUE(!image.empty());
    ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    calibration_images.push_back(image);    
  }

  // Extract Observations
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    cv::Mat output_image;
    ASSERT_TRUE(observation_extractor.extractObservation(calibration_images[i],
      output_image));
  }

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData(); 

  EXPECT_EQ(observation_data.size(), num_images);

  // Load Link Data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp_link_data;
    EXPECT_TRUE(loadLinkData(i, "mcircles_7x5/dataset_1/tf/", &temp_link_data));
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
  double intrinsics[9] = {0};
  intrinsics[0] = 944.72;
  intrinsics[1] = 940.92;
  intrinsics[2] = 925.56;
  intrinsics[3] = 518.89;

  // Set camera extrinsics seed
  industrial_calibration_libs::Pose6D link_6_to_camera;
  link_6_to_camera.setOrigin(0.125, 0.000, 0.091);
  link_6_to_camera.setQuaternion(0.500, 0.500, 0.500, 0.500);

  double extrinsics[6];
  extrinsics[0] = link_6_to_camera.ax;
  extrinsics[1] = link_6_to_camera.ay;
  extrinsics[2] = link_6_to_camera.az;
  extrinsics[3] = link_6_to_camera.x;
  extrinsics[4] = link_6_to_camera.y;
  extrinsics[5] = link_6_to_camera.z;

  double target_to_base[6] = {0};

  target_to_base[3] = 0.75;
  target_to_base[4] = 0.1;
  target_to_base[5] = 0.1;

  std::vector<industrial_calibration_libs::Pose6D> link_poses;
  EXPECT_TRUE(convertToPose6D(link_data, &link_poses));

  industrial_calibration_libs::MovingCameraOnWristStaticTargetIntrinsic calibration(observation_data, target);

  calibration.initKnownValues(link_poses);
  calibration.initSeedValues(extrinsics, target_to_base, intrinsics);

  EXPECT_TRUE(calibration.runCalibration());

  // calibration.displayCovariance();

  industrial_calibration_libs::MovingCameraOnWristStaticTargetIntrinsic::Result results = calibration.getResults();

  // Note(gChiou): Prints out calibration results
#if 1
  CONSOLE_OUTPUT("Extrinsic Parameters");
  CONSOLE_OUTPUT("Translation x: " << results.extrinsics[3]);
  CONSOLE_OUTPUT("Translation y: " << results.extrinsics[4]);
  CONSOLE_OUTPUT("Translation z: " << results.extrinsics[5]);
  CONSOLE_OUTPUT("Rotation x: " << results.extrinsics[0]);
  CONSOLE_OUTPUT("Rotation y: " << results.extrinsics[1]);
  CONSOLE_OUTPUT("Rotation z: " << results.extrinsics[2]);

  CONSOLE_OUTPUT("Target to Base");
  CONSOLE_OUTPUT("Translation x: " << results.target_to_base[3]);
  CONSOLE_OUTPUT("Translation y: " << results.target_to_base[4]);
  CONSOLE_OUTPUT("Translation z: " << results.target_to_base[5]);
  CONSOLE_OUTPUT("Rotation x: " << results.target_to_base[0]);
  CONSOLE_OUTPUT("Rotation y: " << results.target_to_base[1]);
  CONSOLE_OUTPUT("Rotation z: " << results.target_to_base[2]);

  CONSOLE_OUTPUT("Intrinsic Parameters");
  CONSOLE_OUTPUT("Focal Length x: " << results.intrinsics[0]);
  CONSOLE_OUTPUT("Focal Length y: " << results.intrinsics[1]);
  CONSOLE_OUTPUT("Optical Center x: " << results.intrinsics[2]);
  CONSOLE_OUTPUT("Optical Center y: " << results.intrinsics[3]);
  CONSOLE_OUTPUT("Distortion k1: " << results.intrinsics[4]);
  CONSOLE_OUTPUT("Distortion k2: " << results.intrinsics[5]);
  CONSOLE_OUTPUT("Distortion k3: " << results.intrinsics[6]);
  CONSOLE_OUTPUT("Distortion p1: " << results.intrinsics[7]);
  CONSOLE_OUTPUT("Distortion p2: " << results.intrinsics[8]);
#endif  
}
#endif

#ifdef TEST_4
TEST(Calibration, MovingCameraOnWristStaticTargetIntrinsic_dataset_2)
{
  // Load Target Data
  industrial_calibration_libs::Target target;
  EXPECT_TRUE(target.loadTargetFromYAML("mcircles_7x5/mcircles_7x5.yaml"));

  // Load Calibration Images
  const std::size_t num_images = 24;
  std::vector<cv::Mat> calibration_images;
  calibration_images.reserve(num_images);
  std::string cal_image_path = "mcircles_7x5/dataset_2/images/";

  for (std::size_t i = 0; i < num_images; i++)
  {
    std::string image_path = cal_image_path + std::to_string(i+1) + ".jpg";
    cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);

    ASSERT_TRUE(!image.empty());
    ASSERT_TRUE(image.size().width > 0 && image.size().height > 0);

    calibration_images.push_back(image);    
  }

  // Extract Observations
  industrial_calibration_libs::ObservationExtractor observation_extractor(target);
  for (std::size_t i = 0; i < calibration_images.size(); i++)
  {
    cv::Mat output_image;
    ASSERT_TRUE(observation_extractor.extractObservation(calibration_images[i],
      output_image));
  }

  industrial_calibration_libs::ObservationData observation_data = observation_extractor.getObservationData(); 

  EXPECT_EQ(observation_data.size(), num_images);

  // Load Link Data
  std::vector<LinkData> link_data;
  link_data.reserve(num_images);
  for (std::size_t i = 0; i < num_images; i++)
  {
    LinkData temp_link_data;
    EXPECT_TRUE(loadLinkData(i, "mcircles_7x5/dataset_2/tf/", &temp_link_data));
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
  double intrinsics[9] = {0};
  intrinsics[0] = 944.72;
  intrinsics[1] = 940.92;
  intrinsics[2] = 925.56;
  intrinsics[3] = 518.89;

  // Set camera extrinsics seed
  industrial_calibration_libs::Pose6D link_6_to_camera;
  link_6_to_camera.setOrigin(0.125, 0.000, 0.091);
  link_6_to_camera.setQuaternion(0.500, 0.500, 0.500, 0.500);

  double extrinsics[6];
  extrinsics[0] = link_6_to_camera.ax;
  extrinsics[1] = link_6_to_camera.ay;
  extrinsics[2] = link_6_to_camera.az;
  extrinsics[3] = link_6_to_camera.x;
  extrinsics[4] = link_6_to_camera.y;
  extrinsics[5] = link_6_to_camera.z;

  double target_to_base[6] = {0};

  target_to_base[3] = 0.75;
  target_to_base[4] = 0.1;
  target_to_base[5] = 0.1;

  std::vector<industrial_calibration_libs::Pose6D> link_poses;
  EXPECT_TRUE(convertToPose6D(link_data, &link_poses));

  industrial_calibration_libs::MovingCameraOnWristStaticTargetIntrinsic calibration(observation_data, target);

  calibration.initKnownValues(link_poses);
  calibration.initSeedValues(extrinsics, target_to_base, intrinsics);

  EXPECT_TRUE(calibration.runCalibration());

  // calibration.displayCovariance();

  industrial_calibration_libs::MovingCameraOnWristStaticTargetIntrinsic::Result results = calibration.getResults();

  // Note(gChiou): Prints out calibration results
#if 1
  CONSOLE_OUTPUT("Extrinsic Parameters");
  CONSOLE_OUTPUT("Translation x: " << results.extrinsics[3]);
  CONSOLE_OUTPUT("Translation y: " << results.extrinsics[4]);
  CONSOLE_OUTPUT("Translation z: " << results.extrinsics[5]);
  CONSOLE_OUTPUT("Rotation x: " << results.extrinsics[0]);
  CONSOLE_OUTPUT("Rotation y: " << results.extrinsics[1]);
  CONSOLE_OUTPUT("Rotation z: " << results.extrinsics[2]);

  CONSOLE_OUTPUT("Target to Base");
  CONSOLE_OUTPUT("Translation x: " << results.target_to_base[3]);
  CONSOLE_OUTPUT("Translation y: " << results.target_to_base[4]);
  CONSOLE_OUTPUT("Translation z: " << results.target_to_base[5]);
  CONSOLE_OUTPUT("Rotation x: " << results.target_to_base[0]);
  CONSOLE_OUTPUT("Rotation y: " << results.target_to_base[1]);
  CONSOLE_OUTPUT("Rotation z: " << results.target_to_base[2]);

  CONSOLE_OUTPUT("Intrinsic Parameters");
  CONSOLE_OUTPUT("Focal Length x: " << results.intrinsics[0]);
  CONSOLE_OUTPUT("Focal Length y: " << results.intrinsics[1]);
  CONSOLE_OUTPUT("Optical Center x: " << results.intrinsics[2]);
  CONSOLE_OUTPUT("Optical Center y: " << results.intrinsics[3]);
  CONSOLE_OUTPUT("Distortion k1: " << results.intrinsics[4]);
  CONSOLE_OUTPUT("Distortion k2: " << results.intrinsics[5]);
  CONSOLE_OUTPUT("Distortion k3: " << results.intrinsics[6]);
  CONSOLE_OUTPUT("Distortion p1: " << results.intrinsics[7]);
  CONSOLE_OUTPUT("Distortion p2: " << results.intrinsics[8]);
#endif  
}
#endif

