#include "publisher/publisher.h"


// ros::Publisher _grid_map_pub,_grid_occupied_map_pub,nav_path;
// ros::Publisher _Astar_path_pub;

// void define_publisher(ros::NodeHandle &nh)
// {
//     //二维栅格地图的发布器
//     _grid_map_pub=nh.advertise<visualization_msgs::Marker>("/grid_map_vis",10);
//     _grid_occupied_map_pub=nh.advertise<visualization_msgs::Marker>("/grid_occupied_map_vis",10);
//     nav_path=nh.advertise<nav_msgs::Path>("/nav_path",10);
//     _Astar_path_pub=nh.advertise<visualization_msgs::Marker>("/Astar_path_pub",10);
// }