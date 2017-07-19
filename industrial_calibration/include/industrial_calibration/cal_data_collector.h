#ifndef CAL_DATA_COLLECTOR_H
#define CAL_DATA_COLLECTOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

class CalDataCollector
{
public:
  CalDataCollector(ros::NodeHandle nh, ros::NodeHandle pnh);

  ~CalDataCollector(void) { }

  void collectData(void);

// Private Methods
private:

// Private Variables
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};
#endif // CAL_DATA_COLLECTOR_H