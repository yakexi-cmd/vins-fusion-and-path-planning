#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

extern ros::Publisher grid_map_pub,grid_occupied_map_pub,Astar_path_pub,nav_path;
void define_publisher(ros::NodeHandle &nh);

#endif  // MY_PUBLISHERS_H