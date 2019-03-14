#include <string>
#include <cmath>

#include <ros/ros.h>

namespace argos_driver
{

ArgosDriver::ArgosDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{

}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool ArgosDriver::poll(void)
{
  return true;
}

void ArgosDriver::diagTimerCallback(const ros::TimerEvent &event)
{
}

} // namespace argos_driver
