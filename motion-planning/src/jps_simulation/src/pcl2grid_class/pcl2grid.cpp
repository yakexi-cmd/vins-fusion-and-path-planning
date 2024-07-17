// 伪代码框架
/*
    main()
    {
        0.初始化--包括pcl_path,pcl_cloud(Ptr智能指针类型),包括    msg.header.seq/stamp/frame_id ,  msg.info.map_load_time/resolution
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud
        1.加载pcd文件----pcl::loadPCDFile(file_path,*pcl_cloud)//file_path中存放pcd文件路径，将它存放到*pcl_cloud的指针中
        2.pcd转换为2d栅格地图SetMapTopicMsg(pcl_cloud,&msg) //这里msg使用的是引用，因此调用的时候是msg.,如果是指针*msg，则调用的时候是msg->
        {
            2.1设置高度和灰度的映射范围 --(z_max,thre_gray_max),(z_min,thre_gray_min)
            2.2得到两者的k,b线性对应关系k=(z_max-z_min)/(thre_gray_max-thre_gray_min)
            2.3 for循环遍历点云地图中的3d点--- for(int i=0;i<pcl_cloud->points.size();i++)
            {
                2.3.1 找到实际点云地图中的 xmax,xmin,ymax,ymin
            }
            2.4初始化二维栅格地图msg
                msg.info.width/height;
                msg.origin.position.x=xmin/.y=ymin/.z=0   ,   msg.origin.orientation.x=0.0/.y=0.0/.z=0.0/.w=1.0
            2.5for循环重新遍历点云地图中的3d点----for(int i=0;i<pcl_cloud->points.size();i++)
            {
                2.5.1 找到各个点在2d栅格平面的位置--msg_x=(pcl_cloud->points[i].x-xmin)/map_resolution
                                                msg_y=(pcl_cloud->points[i].y-ymin)/map_resolutin
                2.5.2判断2.5.1得到的(msg_x,msg_y)是否在栅格地图范围内 if(msg_x>0 && msg_x <msg.info.width && msg_y>0 && msg_y<msg.info.height)
                {
                    2.5.2.1 如果满足条件，将该节点对应的位置进行标记msg[msg_x * msg.info.width + msg_y]=100;
                }
            }
        }
    }

*/
/* 
代码框架：以直通滤波为例
    1.通过定义launch文件，设置参数
        将滤波的高度限制
        计算实际长度/分辨率----得到地图的像素点
    2.读取点云文件
    3.定义一个passthrough的对象，通过其中的参数处理点云数据
        passthrough.setInputCloud(pcd_cloud)//填入点云数据
        passthrough.setFilterFileName("z") //填入处理的坐标轴，此处是对z轴进行处理
        passthrough.setFilterLimits(thres_low,thres_high) //设置滤波的范围
        passthrough.filter(*cloud)//执行滤波，cloud中保存滤波后的点云文件
        pcl::io::savePCDFile<pcl::PointXYZ>(file_directory+"map_filter.pcd",*cloud)
        //保存滤波后的文件
    4.把pcl的点云数据类型转换为nav_msgs::OccupancyGrid的二维栅格类型的数据
    5.发布二维数据的话题消息
*/
// voxelGrid resolution TODO:

/*
    主要应用于将点云数据转换为栅格地图的形式
*/
#include <pcl2grid_class/pcl2grid.h>
#include <cmath>


void Pcl2Grid::setMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, const double &z_pos)
{   
    // double zmin=zmax=0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    //检查是否存在点云数据
    if(pcl_cloud->points.empty())
    {
      ROS_WARN("pcd is empty!\n");
      return;
    }

    ROS_INFO("[SetMapTopicMsg] : pcl_cloud->points.size() : %d", pcl_cloud->points.size());
    for(int i=0;i<pcl_cloud->points.size();i++)
    {
        //把以m为单位的尺度转换成像素为单位
        int msg_x=static_cast<int>(ceil((pcl_cloud->points[i].x - xmin)/map_resolution));
        int msg_y=static_cast<int>(ceil((pcl_cloud->points[i].y - ymin)/map_resolution));
        // int msg_z=int ((filter_cloud->points[i].z - zmin))/map_resolution;
        if(msg_x<0 || msg_x>=grid_row_size -1)
            continue;
        if(msg_y<0 || msg_y >= grid_col_size - 1)
            continue;
        if (pcl_cloud->points[i].z - z_pos > -0.5)//点云数据和雷达坐标差值>0.5时，需要设置为占据状态
        {
            // double radius=0.05;//表示小车对角线/2的长度，是膨胀栅格地图的半径
            // int grid_radius = static_cast<int>(radius/map_resolution) ;//表示需要膨胀的像素数量
            
            auto map_node = my_grid_map[msg_x * grid_col_size + msg_y];
            map_node->is_occupied=true;

            // drawBresenhamCircle(center_x,center_y,grid_radius);
            // for(int x= msg_x - grid_radius ; x <= msg_x + grid_radius ; x++)
            // {
            //     for(int y=msg_y - grid_radius; y<= msg_y +grid_radius; y++)
            //     {
            //         if(pow(x-msg_x,2)+pow(y-msg_y,2) <= pow(grid_radius,2))
            //             my_grid_map[ x * grid_col_size +y]->is_occupied=true;
            //     }
                    
            // }
        }
    }
} 
// 发布栅格地图
void Pcl2Grid::visualizeGridMap(const ros::Publisher* pub_grid_map)
{
    for(auto map_ptr : my_grid_map)
    {
        if(!map_ptr->is_occupied)
            continue;
        pcl::PointXYZ pt;
        pt.x = (map_ptr->x - 30 / 0.05) * 0.05;
        pt.y = (map_ptr->y - 30 / 0.05) * 0.05;;
        pt.z = 0.0;
        grid_map_pc.push_back(pt);
    }
    sensor_msgs::PointCloud2 pc_msg;
    pcl::toROSMsg(grid_map_pc, pc_msg);
    pc_msg.header.stamp = ros::Time::now();
    pc_msg.header.frame_id = "camera_init";
    pub_grid_map->publish(pc_msg);
}


Pcl2Grid::Pcl2Grid()
{
    map_resolution=0.05;//设置点云地图的分辨率

    // 设置初始的row和col，单位是m
    xmin=-30;xmax=30;ymin=-30;ymax=30;
    //初始化栅格地图大小,将之前pcd三维点云地图(以m为单位)转换为以像素为单位
    grid_col_size=static_cast<int>(ceil((xmax-xmin)/map_resolution));//对应有多少列
    grid_row_size=static_cast<int>(ceil((ymax-ymin)/map_resolution));//对应有多少行
    my_grid_map.clear();
    my_grid_map.resize(grid_col_size*grid_row_size);
    // cout<<"grid_col_size:"<<grid_col_size<<"grid_row_size"<<grid_row_size<<endl;
    TicToc tictoc;
    for(int i=0;i<grid_row_size;i++)
    {
        for(int j=0;j<grid_col_size;j++)
        {
            NodePtr map_node=make_shared<Node>(i, j, false);
            my_grid_map[i*grid_col_size+j]=map_node;
        }
    }
    double time = tictoc.toc();
    std::cout << "[Pcl2Grid] : init map consumption time : " << time << " s \n";
    // cout<<"地图初始化完成!!"<<endl;
}