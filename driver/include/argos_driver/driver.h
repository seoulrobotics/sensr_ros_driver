#ifndef _ARGOS_DRIVER_H_
#define _ARGOS_DRIVER_H_ 1

#include <string>
#include <ros/ros.h>
#include "argos.h"

namespace argos_driver
{
class ArgosDriver
{
public:
  ArgosDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
                 
  ~ArgosDriver() {}

  bool Poll(void);
private:
  ros::Publisher output_;
  argos::Client client_;
};

} // namespace argos_driver

#endif // _ARGOS_DRIVER_H_
