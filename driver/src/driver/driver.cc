#include <string>
#include <cmath>

#include <ros/ros.h>
#include "sensr_driver/driver.h"
#include "sensr_msgs/sensr_message.h"

namespace sensr_driver
{

SensrDriver::SensrDriver(ros::NodeHandle node, ros::NodeHandle private_nh) 
  : client_(nullptr)
{
  output_ = node.advertise<sensr_msgs::sensr_message>("sensr_packets", 10);
  // Get param for ip.
  std::string ip_address;
  private_nh.param("sensr_ip", ip_address, std::string("localhost"));
  client_ = new sensr::Client(ip_address.c_str());
  ROS_INFO("Sensr driver succesfully started - listening on ip %s", ip_address.c_str());
}

void CopyBoundingBox(const bounding_box& source, sensr_msgs::bounding_box& target)
{
  float* position_ptr = (float*)source.position().data();
  float* size_ptr = (float*)source.size().data();
  target.position.x = position_ptr[0];
  target.position.y = position_ptr[1];
  target.position.z = position_ptr[2];
  target.size.x = size_ptr[0];
  target.size.y = size_ptr[1];
  target.size.z = size_ptr[2];
  target.yaw = source.yaw();
}

void CopyPoints(const point_list& source, std::vector<geometry_msgs::Vector3>& target)
{
  target.resize(source.size());
  const float* ptr = (const float*)source.points().data();
  for(int i = 0; i < source.size(); ++i)
  {
    const float* point = ptr + i * 3;
    target[i].x = point[0];
    target[i].y = point[1];
    target[i].z = point[2];
  }
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool SensrDriver::Poll(void)
{
  output_message message;
  if (client_->ReceiveMessageAsync(message))
  {
    sensr_msgs::sensr_messagePtr packet(new sensr_msgs::sensr_message);
    packet->header.stamp.sec = message.time_stamp().seconds();
    packet->header.stamp.nsec = message.time_stamp().nano_seconds();

    auto& ground_points = message.point_cloud().ground_points();
    int ground_points_count = ground_points.size();
    packet->ground_points.resize(ground_points_count);
    for(int i = 0; i < ground_points_count; ++i)
    {
      float* ptr = ((float*)ground_points.points().data()) + i * 3;
      packet->ground_points[i].x = ptr[0];
      packet->ground_points[i].y = ptr[1];
      packet->ground_points[i].z = ptr[2];
    }

    auto& object_points = message.point_cloud().object_points();
    int object_points_count = object_points.size();
    packet->object_points.resize(object_points_count);
    for(int i = 0; i < object_points_count; ++i)
    {
      float* ptr = ((float*)object_points.points().data()) + i * 3;
      packet->object_points[i].x = ptr[0];
      packet->object_points[i].y = ptr[1];
      packet->object_points[i].z = ptr[2];
    }

    auto& invalid_points = message.point_cloud().invalid_points();
    int invalid_points_count = invalid_points.size();
    packet->invalid_points.resize(invalid_points_count);
    for(int i = 0; i < invalid_points_count; ++i)
    {
      float* ptr = ((float*)invalid_points.points().data()) + i * 3;
      packet->invalid_points[i].x = ptr[0];
      packet->invalid_points[i].y = ptr[1];
      packet->invalid_points[i].z = ptr[2];
    }

    int non_tracked_objects_count = message.non_tracked_objects_size();
    packet->non_tracked_objects.resize(non_tracked_objects_count);
    for(int i = 0; i < non_tracked_objects_count; ++i)
    {
      auto& source_object = message.non_tracked_objects(i);
      auto& object = packet->non_tracked_objects[i];
      object.id = source_object.id();
      CopyBoundingBox(source_object.bbox(), object.bounding_box);
    }

    int tracked_objects_count = message.tracked_objects_size();
    packet->tracked_objects.resize(tracked_objects_count);
    for(int i = 0; i < tracked_objects_count; ++i)
    {
      auto& source_object = message.tracked_objects(i);
      auto& object = packet->tracked_objects[i];
      object.id = source_object.id();
      object.label = source_object.label();
      object.probability = source_object.probability();
      object.tracking_reliable = source_object.tracking_reliable();

      float* ptr = (float*)source_object.velocity().data();
      object.velocity.x = ptr[0];
      object.velocity.y = ptr[1];
      object.velocity.z = ptr[2];

      CopyBoundingBox(source_object.bbox(), object.bounding_box);
      CopyPoints(source_object.history(), object.history);
      CopyPoints(source_object.prediction(), object.prediction);
    }

    output_.publish(packet);
  }
  return true;
}

} // namespace sensr_driver
