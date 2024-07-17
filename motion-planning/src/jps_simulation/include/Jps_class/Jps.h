#pragma once
#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <memory>
#include <limits>
#include <queue>
#include <ros/ros.h>
//该部分使用nav实现可视化效果
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "pcl2grid_class/pcl2grid.h"
#include "Node_class/Node.h"
using namespace std;
extern int flag_start;//表示起始节点的方向已经全部遍历过
 
//此处定义枚举类型表示方向

struct Position
{
    int x,y;//定义x,y坐标
    Position(int x_init,int y_init):x(x_init),y(y_init){}
};
/*
    存放并处理二维栅格地图
*/
class Jps
{
public:
    //  ------jps中地图相关部分 ----------//
    int jps_row,jps_col;//定义jps地图中的宽和高
    //定义jps地图中的 : 起始节点和终止节点
    NodePtr startPtr,endPtr;
    vector<NodePtr> jps_grid_map;//定义jps地图
    pcl::PointCloud<pcl::PointXYZ> jps_occupied_grid_map;//只定义被占据部分的栅格
    double xmin,xmax,ymin,ymax,zmin,zmax;//定义栅格地图的边界范围
    double jps_resolution;

    // ---------------关于jps寻路算法的变量----------------//
    priority_queue<NodePtr,vector<NodePtr>,compute_Priority> openlist;//优先队列
    vector<NodePtr> closelist;
    vector<NodePtr> jump_points;
    queue<Direction> dirList;
    vector<NodePtr> path;
    // --------------------------------------------------//

    Jps(Pcl2Grid &pcd_class)//Jps构造函数初始化
    {
        jps_row=pcd_class.grid_row_size;
        jps_col=pcd_class.grid_col_size;
        jps_grid_map=pcd_class.my_grid_map;
        jps_resolution=pcd_class.map_resolution;
        xmin=pcd_class.xmin;
        xmax=pcd_class.xmax;
        ymin=pcd_class.ymin;
        ymax=pcd_class.ymax;
    }
    // 在rviz中可视化路径
    void visualizePath(vector<NodePtr>path,const ros::Publisher* pub_path);
    void updateOccupiedMap(Pcl2Grid &pcd_class);//初始化pcl地图
    void updateMap(Pcl2Grid &pcd_class);//初始化pcl地图
    void drawBresenhamCircle(int x,int y,int r);
    void set_occupied(int x,int y);//将jps栅格地图的部分节点设置为占据状态
    // ---------------关于jps寻路算法的函数----------------//
    //计算h(n)使用曼哈顿距离或者欧式距离
    double calc_hScore(const NodePtr curPtr ,const NodePtr targetPtr);
    bool is_valid(NodePtr node);
    bool In_openlist(NodePtr node);
    bool In_closelist(NodePtr node);
    bool map_is_valid(NodePtr node);
    bool is_inborder(NodePtr node);//判断是否在边界范围内
    void findPath_JPS();//采用jps方式的路径搜索方法
    void add2openlist(NodePtr curPtr);
    void find_JumpPoint(NodePtr curPtr);//用于查找跳点
    bool find_forceNebor(NodePtr curPtr,Direction dir);//output :true/false,用于判断是/否存在强迫邻居
    void get_direction(NodePtr fatherPtr,NodePtr childPtr);//获得当前节点需要遍历的方向
    void returnPath(NodePtr curPtr,vector<NodePtr> &buf);//返回最终查询路径
    inline int toAddress(int x,int y,int col_size)
    {
        return x * col_size + y;
    }
    // --------------------------------------------------//
};