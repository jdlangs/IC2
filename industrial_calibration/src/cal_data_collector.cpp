#include <industrial_calibration/cal_data_collector.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cal_data_collector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ROS_INFO_STREAM("Hello World!");
  return 0;
}