cmake_minimum_required(VERSION 2.8.3)
project(sensr_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  sensr_message.msg
  bounding_box.msg
  non_tracked_object.msg
  tracked_object.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
