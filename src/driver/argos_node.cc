#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "argos_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // start the driver

  // loop until shut down or end of file
  while(ros::ok() /* && dvr.poll() */)
  {
    ros::spinOnce();
  }

  return 0;
}
