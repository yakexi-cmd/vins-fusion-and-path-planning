// /* 
//     JPS寻路方法
//         1.openlist中遍历节点
//         2.寻找跳点jump point
//             3.寻找强迫邻居--只有周围被占据，只能从跳点得到的点为强迫节点

//---------------简易版JPS----------------//
/* 
    A_star  inclue  Node.h
        private: 
            map  <Node>
            path 
            start_pt
            end_pt 
*/

// 1. 初始化A_star map 大小
// 2. while()
        // 2.1 接受起点终点, 传给A_star类
        // 2.2 执行planning
        // 2.3 visulization

#include "my_Astar/Jps.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
using namespace std;

ros::Publisher _grid_path_pub,_grid_occupied_path_pub,_jps_path_pub;
pcd2pgm pcd_class;
NodePtr start_position,end_position;
bool flag=false;

//-----------------接收rviz中起始和终止部分的节点-------------//
//-----start和end分别表示2d pose estimate 和2d nav goal得到的坐标值--------//
void startPts_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& goal)
{
    double x=goal->pose.pose.position.x;
    double y=goal->pose.pose.position.y;
    int map_x = int ((x - pcd_class.xmin)/pcd_class.map_resolution);
    int map_y = int ((y - pcd_class.ymin)/pcd_class.map_resolution);
    start_position = pcd_class.my_grid_map[map_x*pcd_class.grid_col_size+map_y];
    cout<<"Start position is :("<<start_position->x<<","<<start_position->y<<endl;
}
void endPts_callback(const geometry_msgs::PoseStamped::ConstPtr& end_goal)
{
    double x=end_goal->pose.position.x;
    double y=end_goal->pose.position.y;
    int map_x = int ((x - pcd_class.xmin)/pcd_class.map_resolution);
    int map_y = int ((y - pcd_class.ymin)/pcd_class.map_resolution);
    end_position = pcd_class.my_grid_map[map_x*pcd_class.grid_col_size+map_y];
    cout<<"end position is :("<<end_position->x<<","<<end_position->y<<endl;
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"Astar_init");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(1.0);
    pcd_class.map_topic_pub=nh.advertise<sensor_msgs::PointCloud2>(pcd_class.map_topic_name,1);
    cout<<"xmin:"<<pcd_class.xmin<<"-xmax:"<<pcd_class.xmax<<"--ymin:"<<pcd_class.ymin<<"--ymax:"<<pcd_class.ymax<<endl;

    //----------------------此处定义一些地图相关参数--------------------------//
    //--------------------------关于地图处理部分----------------------------//
    //----------把Astar定义成一个类，想像成一个好用的职员，让她处理相关工作--------//
    //1.地图初始化部分
    Jps jps_(nh);
    // start_position = pcd_class.my_grid_map[167*pcd_class.grid_col_size+125];
    // end_position = pcd_class.my_grid_map[104*pcd_class.grid_col_size+157];
    // jps_.initMap(pcd_class);//地图初始化，获得vector<Node> grid_map;之后通过对一维数组进行i*row_size+j方式确定node所在位置
    
    // start_position = jps_.jps_grid_map[jps_.toAddress(159,126,jps_.jps_col)];//pcd_class.my_grid_map[150*pcd_class.grid_col_size+125];
    // end_position = jps_.jps_grid_map[jps_.toAddress(103,202,jps_.jps_col)];//pcd_class.my_grid_map[134*pcd_class.grid_col_size+150];
    // jps_.startPtr=start_position;//将rviz中读取到的坐标值放入我的jps类中
    // jps_.endPtr=end_position;
    // jps_.showMap();
    // jps_.findPath_JPS(); 
    
    // jps_.gridPath_pub(jps_.path,pcd_class);
    // jps_.nav_showPath(jps_.path,pcd_class);
    jps_.initMap(pcd_class);//地图初始化，获得vector<Node> grid_map;之后通过对一维数组进行i*row_size+j方式确定node所在位置
    //-----------------------读取起点和终点-----------------------------//
    ros::Subscriber startPts_sub=nh.subscribe("/initialpose",1000,startPts_callback);
    ros::Subscriber endPts_sub  =nh.subscribe("/move_base_simple/goal",1000,endPts_callback);
    
    //3.jps算法进行处理
    while(ros::ok())
    {
        pcd_class.map_topic_pub.publish(pcd_class.pcd2grid_map);//这个是发送二维栅格地图
        jps_.startPtr=start_position;//将rviz中读取到的坐标值放入我的jps类中
        jps_.endPtr=end_position;
        if(jps_.startPtr != nullptr && jps_.endPtr != nullptr && jps_.startPtr!=jps_.endPtr)
        {
            cout<<"start_position is :("<<start_position->x<<endl;
            cout<<"end_position is :("<<end_position->x<<endl;
            jps_.findPath_JPS(); 
            flag=true;
            start_position=nullptr;
            end_position=nullptr;
            flag_start=0;
            
            // while(!jps_.openlist.empty())
            //     jps_.openlist.pop();
            // jps_.closelist.clear();
            
        }
        jps_.gridPath_pub(jps_.path,pcd_class);
        jps_.nav_showPath(jps_.path,pcd_class);
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
    
}