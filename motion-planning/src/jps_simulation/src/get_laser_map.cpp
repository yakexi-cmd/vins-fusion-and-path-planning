#include <iostream>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

// pcl库用于处理点云数据
// 用于定义点云数据结构
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
// 用于定义点的类型
// pcl::PointXYZ 三维点，包括xyz信息
// pcl:PointXYZRGB 带有RGB颜色信息的三维点
// pcl::PointNormal 包含法线信息的三维点
// pcl::PointXYZI 带有强度信息的三维点
//pcl::PointXYZL 带有标签信息的三维点
#include <pcl/point_types.h>
// 用于引入体素网格滤波器，通过对体素网格单元中的点进行平均来对点云进行降采样
#include <pcl/filters/voxel_grid.h>
// 使用flann库实现KSTree，用于在点云中进行高效的近邻搜索
#include <pcl/kdtree/kdtree_flann.h>

//用于处理坐标变换的数据类型，包括tf::Quaternion四元数和vector3d三维向量
#include <tf/transform_datatypes.h>
// 提供了广播坐标变换，将坐标变换发送到ros系统中，便于其他节点获取和调用
#include <tf/transform_broadcaster.h>
#include "Jps_class/Jps.h"
#include "publisher/publisher.h"
#include <mutex>
Pcl2Grid pcd_map;

// 用于判断是否进入回调函数，检测3个回调函数:start,end,laser点云
bool flag=false,is_odom=false,is_lasercloud=false,is_endPts=false;
Eigen::Vector3d start_vec3d(0, 0, 0);
NodePtr start_position,end_position;
//此处定义发布话题所需要的发布器和接收器，为全局变量
ros::Publisher grid_map_pub, grid_occupied_map_pub, nav_path, Astar_path_pub;
ros::Subscriber sub_lasermap,sub_startPts,sub_endPts;
//接收完整的点云地图
pcl::PointCloud<pcl::PointXYZ>::Ptr total_pcl_map(new pcl::PointCloud<pcl::PointXYZ>);;

// 成功订阅点云地图后执行的回调函数
// input: 订阅的点云信息
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    if (laserCloudMsg->data.size() == 0) {
        ROS_ERROR("Received empty message from /laser_map topic.");
        return; // Skip processing empty message
    }
    TicToc tt;
    // cout<<"============================================laserCloudHandler================================================="<<endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudPclMsgPtr(new pcl::PointCloud<pcl::PointXYZ>());
    //1.把ros话题消息类型转换为c++中通用的pcl库laserCloudIn,将sensor_msgs传感器的数据变换为pcl库
    pcl::fromROSMsg(*laserCloudMsg,*laserCloudPclMsgPtr);
    ROS_INFO("[laserCloudHandler] : laserCloudPclMsgPtr->points.size : %d",laserCloudPclMsgPtr->points.size());
    //-------------此处通过各帧点云获得累计地图----------------------//
    // *total_pcl_map+=*laserCloudPclMsgPtr;
    // ROS_INFO("[laserCloudHandler] start SetMapTopicMsg : start_vec3d(2) : %f", start_vec3d(2));
    
    //2.将转换得到的pcl点云地图转换为pcd2grid_map二维栅格地图
    pcd_map.setMapTopicMsg(laserCloudPclMsgPtr, start_vec3d(2));//通过该函数，二维栅格地图被存储在pcd_map.my_grid_map中
    // ROS_INFO("[laserCloudHandler] end SetMapTopicMsg : start_vec3d(2) : %f", start_vec3d(2));
    //--------------------------------------------- -------------//
    //3.发布得到的栅格地图【将地图信息全部存入jps类中】
    double time = tt.toc();
    is_lasercloud=true;
    // 分别发布栅格地图(sensor_msgs)和路径信息(Path)
    pcd_map.visualizeGridMap(&grid_map_pub);
    
    std::cout << "[laserCloudHandler] update time : " << time << " s.\n";  
}
void startPts_callback(const nav_msgs::Odometry::ConstPtr& startPts)
{
    TicToc t_start;
    // cout<<"============================================startPts_callback================================================="<<endl;
    start_vec3d << startPts->pose.pose.position.x, startPts->pose.pose.position.y, startPts->pose.pose.position.z;
    double x=startPts->pose.pose.position.x;
    double y=startPts->pose.pose.position.y;   
    // cout<<"(m 为单位)start position is :("<<x<<","<<y<<","<<pcd_map.grid_col_size<<endl;
    int map_x=int((x-pcd_map.xmin)/pcd_map.map_resolution);
    int map_y=int((y-pcd_map.ymin)/pcd_map.map_resolution);
    // cout<<"start position is :("<<map_x<<","<<map_y<<endl;
    start_position=pcd_map.my_grid_map[map_x*pcd_map.grid_col_size+map_y];
    is_odom=true;
    double time=t_start.toc();
    cout<<" [startPts_callback] update time :" << time << "s.\n"<<endl;

}
//使用2d nav goal获得目标点在地图中的二维坐标信息
void endPts_callback(const geometry_msgs::PoseStamped::ConstPtr& endPts)
{
    TicToc t_end;
    // cout<<"============================================endPts_callback================================================="<<endl;
    double x=endPts->pose.position.x;
    double y=endPts->pose.position.y;

    int map_x = int ((x-pcd_map.xmin)/pcd_map.map_resolution);
    int map_y = int ((y-pcd_map.ymin)/pcd_map.map_resolution);
    // cout<<"end position is :("<<map_x<<","<<map_y<<endl;
    end_position = pcd_map.my_grid_map[map_x*pcd_map.grid_col_size+map_y];
    is_endPts=true;
    double time=t_end.toc();
    cout<<" [endPts_callback] update time :" << time << "s.\n"<<endl;

}

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"laser_map");
    ros::NodeHandle nh;//表示正在使用私有命名空间，某些节点定义了特定的参数，而不是全局参数，避免冲突

    grid_map_pub=nh.advertise<sensor_msgs::PointCloud2>("/grid_map_vis",10);
    grid_occupied_map_pub=nh.advertise<visualization_msgs::Marker>("/grid_occupied_map_vis",10);
    nav_path=nh.advertise<nav_msgs::Path>("/nav_path",10);
    Astar_path_pub=nh.advertise<visualization_msgs::Marker>("/Astar_path_pub",10);
    sub_lasermap=nh.subscribe<sensor_msgs::PointCloud2>("/surfel_fusion/pointcloud",1, laserCloudHandler);
    sub_startPts=nh.subscribe<nav_msgs::Odometry>("/odometry_rect",10,startPts_callback);
    sub_endPts=nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal",10,endPts_callback);

    ros::Rate loop_rate(10);
    // 订阅当前位置和终点信息
    // 订阅点云地图并处理
    Jps jps_(pcd_map);    
    while(ros::ok())
    {
        ros::spinOnce();
        ros::Time start_time, end_time;
        ROS_INFO(" start entrance \n");
        if(is_odom && is_endPts && is_lasercloud)
        {
            start_time=ros::Time::now();
            jps_.updateMap(pcd_map);//地图初始化，获得vector<Node> grid_map;之后通过对一维数组进行i*row_size+j方式确定node所在位置
            jps_.startPtr=start_position;//将rviz中读取到的坐标值放入我的jps类中
            jps_.endPtr=end_position;
            if(jps_.startPtr != nullptr && jps_.endPtr != nullptr && jps_.startPtr!=jps_.endPtr)
            {
                //1.进行规划前，首先进行初始化,清空openlist和closelist,防止前面得到的数据对后续规划产生影响
                jps_.path.clear();
                while(!jps_.openlist.empty())
                {
                    jps_.openlist.pop();
                }
                jps_.closelist.clear();

                jps_.findPath_JPS(); 
                flag_start=0;
                // // 重置条件
                is_odom = false;
                is_lasercloud = false;
                // is_endPts=false;
                // 清空指针
                start_position=nullptr;
                // end_position=nullptr; 
            }  
            ROS_INFO("[main] Path size: %d ",jps_.path.size());
            jps_.visualizePath(jps_.path,&nav_path);
            // jps_.nav_showPath(jps_.path,pcd_map);
            end_time=ros::Time::now();
        }
        // cout<<"~~~~~~~~~~~~~~~~~~~nav_showPath:~~~~~~~~~~~~~~~~"<<jps_.path.size()<<endl;
        ros::Duration duration=end_time-start_time;
        double duration_sec=duration.toSec();
        cout << "Code execution time:  " <<  duration_sec << " seconds" << endl;

        loop_rate.sleep(); 
    }
    // pcl::io::savePCDFile("/home/parallels/开源slam/路径规划/jps_ws/src/jps_simulation/pcd/total_map.pcd",*total_pcl_map);
    return 0;
}