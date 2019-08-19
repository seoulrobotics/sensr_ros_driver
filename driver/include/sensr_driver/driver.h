#ifndef _SENSR_DRIVER_H_
#define _SENSR_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include "sensr.h"

namespace sensr_driver
{
class SensrDriver
{
public:
  SensrDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
                 
  ~SensrDriver() {}

  bool Poll(void);
private:
  ros::Publisher output_;
  sensr::Client* client_;
};

} // namespace sensr_driver

#endif // _SENSR_DRIVER_H_
