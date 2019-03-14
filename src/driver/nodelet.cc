#include <string>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "argos_driver/driver.h"

namespace argos_driver
{

class DriverNodelet: public nodelet::Nodelet
{
public:

  DriverNodelet():
    running_(false)
  {}

  ~DriverNodelet()
  {
    if (running_)
      {
        NODELET_INFO("shutting down driver thread");
        running_ = false;
        deviceThread_->join();
        NODELET_INFO("driver thread stopped");
      }
  }

private:
  virtual void onInit(void);
  virtual void devicePoll(void);
};

void DriverNodelet::onInit()
{
}

/** @brief Device poll thread main loop. */
void DriverNodelet::devicePoll()
{
  while(ros::ok())
  {
    // poll device until end of file
    if (!running_)
      ROS_ERROR_THROTTLE(1.0, "DriverNodelet::devicePoll - Failed to poll device.");
  }
}

} // namespace argos_driver

// Register this plugin with pluginlib.  Names must match nodelet_argos.xml.
//
// parameters are: class type, base class type
PLUGINLIB_EXPORT_CLASS(argos_driver::DriverNodelet, nodelet::Nodelet)
