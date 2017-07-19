#include <industrial_calibration/cal_data_collector.h>

CalDataCollector::CalDataCollector(ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh), pnh_(pnh)
{

}

void CalDataCollector::collectData(void)
{
  ROS_INFO_STREAM("Hello World!");
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