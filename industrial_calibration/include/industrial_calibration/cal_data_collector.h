#ifndef CAL_DATA_COLLECTOR_H
#define CAL_DATA_COLLECTOR_H

#include <ros/ros.h>

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
};

#endif // CAL_DATA_COLLECTOR_H