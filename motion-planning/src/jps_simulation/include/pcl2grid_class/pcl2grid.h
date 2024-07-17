#include <ros/ros.h>
#include <nav_msgs/GetMap.h>//用于请求地图数据
#include <nav_msgs/OccupancyGrid.h>//导入了nav_msgs中的occupancyGrid消息类型，表示占据栅格地图
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
// 点云数据库包括点云滤波，配准，特征提取，表面重建等方面
#include <pcl/filters/conditional_removal.h>//条件滤波器头文件
#include <pcl/filters/passthrough.h> //直通滤波器头文件
#include <pcl/filters/radius_outlier_removal.h> //半径滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器头文件
#include <pcl/filters/voxel_grid.h> //体素滤波器头文件
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Node_class/Node.h>
#include "tic_toc.h"

using namespace std;

class Pcl2Grid
{
public:
    Pcl2Grid();
    void setMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,  const double &z_pos);
    void visualizeGridMap(const ros::Publisher* pub_grid_map);
     string map_topic_name;
    string pcd_file;
    double xmin,xmax,ymin,ymax,zmin,zmax;
    int grid_col_size;
    int grid_row_size;
    pcl::PointCloud<pcl::PointXYZ> grid_map_pc;
    sensor_msgs::PointCloud2 pcd2grid_map;//又转换为了ros消息类型  
    vector<NodePtr> my_grid_map;//使用自己定义的类，存储栅格占据情况
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud;
    double map_resolution;
    ros::Publisher map_topic_pub;

    typedef std::shared_ptr<Pcl2Grid> pcl2gridPtr;
};




