#include <string>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include "sensr_driver/driver.h"

namespace sensr_driver
{

class DriverNodelet : public nodelet::Nodelet
{
public:
  DriverNodelet() 
    : running_(false) {}

  ~DriverNodelet()
  {
    if(running_)
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

  volatile bool running_;
  boost::shared_ptr<boost::thread> deviceThread_;
  boost::shared_ptr<SensrDriver> driver_; ///< driver implementation class
};

void DriverNodelet::onInit()
{
  driver_.reset(new SensrDriver(getNodeHandle(), getPrivateNodeHandle()));
  // spawn device poll thread
  running_ = true;
  deviceThread_ = boost::shared_ptr< boost::thread >
    (new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));

}

/** @brief Device poll thread main loop. */
void DriverNodelet::devicePoll()
{
  while (ros::ok())
  {
    running_ = driver_->Poll();
    if(!running_)
    {
      ROS_ERROR_THROTTLE(1.0, "DriverNodelet::devicePoll - Failed to poll device.");
    }
  }
  running_ = false;
}

} // namespace sensr_driver

// Register this plugin with pluginlib.  Names must match nodelet_sensr.xml.
//
// parameters are: class type, base class type
PLUGINLIB_EXPORT_CLASS(sensr_driver::DriverNodelet, nodelet::Nodelet)
