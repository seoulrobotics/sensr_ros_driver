# Seoul Robotics perception packet.
std_msgs/Header header # Contains timestamp
geometry_msgs/Vector3[] ground_points
geometry_msgs/Vector3[] invalid_points
argos_msgs/tracked_object[] tracked_objects
argos_msgs/non_tracked_object[] non_tracked_objects