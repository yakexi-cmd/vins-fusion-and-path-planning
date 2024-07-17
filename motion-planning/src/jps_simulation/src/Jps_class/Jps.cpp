#include "Jps_class/Jps.h"
#include "publisher/publisher.h"
using namespace std;

/*
1.找到强迫邻居force_neighbor
2.把得到的跳点加入到openlist中，进行遍历

*/
int flag_start=0;//表示起始节点的方向已经全部遍历过
// 定义打印dir的运算符<<，能够将Direction打印出来
ostream& operator<<(ostream &oc,const Direction &dir)
{
    switch(dir)
    {
        case Direction::left:
            oc<<"left";
            break;
        case Direction::right:
            oc<<"right";
            break;
        case Direction::up:
            oc<<"up";
            break;
        case Direction::down:
            oc<<"down";
            break;
        case Direction::up_left:
            oc<<"up_left";
            break;
        case Direction::up_right:
            oc<<"up_right";
            break;
        case Direction::down_left:
            oc<<"down_left";
            break;
        case Direction::down_right:
            oc<<"down_right";
            break;
        default:
            break;
    }
    return oc;
}
// 更新二维栅格地图
void Jps::updateOccupiedMap(Pcl2Grid &pcd_class)
{
    pcl::PointXYZ pt;
    for(auto map_ptr : pcd_class.my_grid_map)
    {
        if(!map_ptr->is_occupied)
            continue;
        pt.x=map_ptr->x;
        pt.y=map_ptr->y;
        pt.z=0;
        jps_occupied_grid_map.push_back(pt);
    }
    
}
// void Jps::updateMap(Pcl2Grid &pcd_class)
// {
//     for(auto map_ptr:pcd_class.grid_map_pc)
//     {
//         int x=static_cast<int>(ceil(map_ptr.x-xmin)/jps_resolution);
//         int y=static_cast<int>(ceil(map_ptr.y-ymin)/jps_resolution);
//         jps_grid_map[x*jps_col+y]->is_occupied=true;
//     }
// }

void Jps::set_occupied(int x,int y)
{
    if(x >= 0 && x <jps_row && y >=0 && y < jps_col)
        jps_grid_map[ x * jps_col + y]->is_occupied=true;
}
// input param: xi,yi,r(当前位置的像素坐标和半径)
void Jps::drawBresenhamCircle(int x,int y,int r)
{
    double distanceB,distanceC;
    int x_new=0,y_new=r;
    // distanceB=(x+1)^2+y^2-r^2;//计算像素到真实圆的边缘距离的相对度量，反映误差值
    // distanceC=(x+1)^2+(y-1)^2-r*r;
    // ROS_INFO("[updateMap by drawing BresenhamCircle \n] distanceB=%f,distanceC=%f",distanceB,distanceC);
    double di=2*(x+1)^2+y^2+(y-1)^2-2*r^2;//di=distanceB+distanceC
    
    while(x_new<=y_new) //当点处于第一段圆弧的时候进行操作，并在其中扩展到八个象限
    {
        // 从第一象限扩展到整个圆
        set_occupied(x+x_new,y+y_new);//将第一象限最初的圆弧设置为占据状态
        set_occupied(x-x_new,y+y_new);//将第二象限最初的圆弧设置为占据状态
        set_occupied(x-x_new,y-y_new);//将第三象限最初的圆弧设置为占据状态
        set_occupied(x+x_new,y-y_new);//将第四象限最初的圆弧设置为占据状态
        
        set_occupied(x+y_new,y+x_new);//将第一象限对称区域的圆弧设置为占据状态
        set_occupied(x-y_new,y+x_new);//将第四象限对称区域的圆弧设置为占据状态
        set_occupied(x-y_new,y-x_new);//将第四象限对称区域的圆弧设置为占据状态
        set_occupied(x+y_new,y-x_new);//将第四象限对称区域的圆弧设置为占据状态
        x_new++;
        if(di<0)
        {
            di+=4*x+6;
        }
        if(di>=0)
        {
            y_new--;
            di+=4*(x-y)+10;
        }
    }
    
    
    
}
// 此处使用膨胀栅格地图
void Jps::updateMap(Pcl2Grid &pcd_class)
{
    
    for(auto map_ptr:pcd_class.grid_map_pc)
    {
        int center_x=static_cast<int>(ceil(map_ptr.x-xmin)/jps_resolution);
        int center_y=static_cast<int>(ceil(map_ptr.y-ymin)/jps_resolution);
        jps_grid_map[center_x * jps_col + center_y]->is_occupied=true;
        // ROS_INFO("[up dateMap get x and y \n] x=%f,y=%f",center_x,center_y);
    
        
    }
}
// 可视化:发布路径信息
void Jps::visualizePath(vector<NodePtr>path,const ros::Publisher* pub_path)
{
    nav_msgs::Path jps_path;
    jps_path.header.stamp=ros::Time::now();
    jps_path.header.frame_id="camera_init";
    geometry_msgs::PoseStamped path_buf;
    for(int i=0;i<path.size();i++)
    {
        path_buf.pose.position.x=path[i]->x*jps_resolution+xmin;
        path_buf.pose.position.y=path[i]->y*jps_resolution+ymin;
        path_buf.pose.position.z=0;
     
        jps_path.poses.push_back(path_buf);
    }
    pub_path->publish(jps_path);
}

// --------------------------------------关于jps寻路算法的函数-------------------------------//
//计算h(n)使用曼哈顿距离或者欧式距离
double Jps::calc_hScore(const NodePtr curPtr ,const NodePtr targetPtr)
{
    // if(targetPtr == nullptr)
    //     return;
    // 如果使用的是曼哈顿距离，需要计算|x|+|y|
    // double hscore=abs(curPtr->x - targetPtr->x) + abs(curPtr->y - targetPtr->y);
    // 如果使用欧式距离
    double hscore=sqrt(
        (curPtr->x - targetPtr->x)*(curPtr->x - targetPtr->x) + (curPtr->y - targetPtr->y)*(curPtr->y - targetPtr->y)
    );
    return hscore;
}

//构造函数,用于判断地图中某节点是否被占据
bool Jps::map_is_valid(NodePtr node)
{
    NodePtr bufPtr=jps_grid_map[toAddress(node->x,node->y,jps_col)];
    if (bufPtr->is_occupied)
        return false;
    else
        return true;
}
bool Jps::is_inborder(NodePtr node)
{
    
    if(node->x >= 0 && node->x <jps_row && node->y >=0 && node->y < jps_col)
        return true;
    else
        return false;     
}

void Jps::get_direction(NodePtr fatherPtr,NodePtr childPtr)
{
    int dx,dy;
    Direction dir;//定义一个Direction类型，用于判定从父节点到子节点的移动方向是枚举类型的哪一种
    dx=childPtr->x-fatherPtr->x;
    dy=childPtr->y-fatherPtr->y;


    if(dx <= -1 && dy == 0)//up{0,1}
        dir=Direction::up;
    if(dx >= 1 && dy == 0)//down{0,-1}
        dir=Direction::down;
    if(dx == 0 && dy <= -1)//left{-1,0}
        dir=Direction::left;
    if(dx == 0 && dy >= 1)//right{1,0}
        dir=Direction::right;
    if(dx <= -1 && dy <= -1)//up_left{-1,1}
        dir=Direction::up_left;
    if(dx <= -1 && dy >= 1)//up_right{1,1}
        dir=Direction::up_right;
    if(dx >= 1&& dy <= -1)//down_left{-1,-1}
        dir=Direction::down_left;
    if(dx >= 1&& dy >= 1)//down_right{1,-1}
        dir=Direction::down_right;
    if(dx == 0 && dy ==0 )
        dir=Direction::none_;   
    /*
    cout<<"经过get_direction后得到的方向是:"<< dir <<endl;
    */   
    
    switch(dir)
    {
        case (Direction::up):
            childPtr->dirList.push(Direction::up);
            childPtr->dirList.push(Direction::up_left);
            childPtr->dirList.push(Direction::up_right);
            break;
        case (Direction::down):
            childPtr->dirList.push(Direction::down);
            childPtr->dirList.push(Direction::down_left);
            childPtr->dirList.push(Direction::down_right);
            break;
        case(Direction::left):
            childPtr->dirList.push(Direction::left);
            childPtr->dirList.push(Direction::up_left);
            childPtr->dirList.push(Direction::down_left);
            break;
        case (Direction::right):
            childPtr->dirList.push(Direction::right);
            childPtr->dirList.push(Direction::up_right);
            childPtr->dirList.push(Direction::down_right);
            break;
        case (Direction::up_right):
            childPtr->dirList.push(Direction::right);
            childPtr->dirList.push(Direction::up);
            childPtr->dirList.push(Direction::up_right);
            break;
        case (Direction::up_left):
            childPtr->dirList.push(Direction::left);
            childPtr->dirList.push(Direction::up);
            childPtr->dirList.push(Direction::up_left);
            break;
        case (Direction::down_right):
            childPtr->dirList.push(Direction::right);
            childPtr->dirList.push(Direction::down);
            childPtr->dirList.push(Direction::down_right);
            break;
        case (Direction::down_left):
            childPtr->dirList.push(Direction::left);
            childPtr->dirList.push(Direction::down);
            childPtr->dirList.push(Direction::down_left);
            break;
        case (Direction::none_):
            // cout<<"与当前点重合"<<endl;
            
            break;
        default:
            break;
    }
    /*
    cout<<"经过get_direction后得到的size:"<< childPtr->dirList.size() <<endl;
    */
    
}
/*功能：
    1.判断节点是否在openlist中，若不在，加入openlist
    2.计算f(n),h(h),g(n)
*/
bool Jps::In_openlist(NodePtr node)
{
    bool in_opnelist=false;
    priority_queue<NodePtr,vector<NodePtr>,compute_Priority> openlist_buf=openlist;
    while(!openlist_buf.empty())
    {
        // cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        // cout<<"openlist中的元素有"<<openlist_buf.size()<<","<<openlist_buf.top()->x<<","<<openlist_buf.top()->y<<"*******node中的元素有"<<node->x<<","<<node->y<<endl;
        // cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        
       if(node->x == openlist_buf.top()->x && node->y == openlist_buf.top()->y)
        {
            // cout<<"openlist中的元素有"<<openlist_buf.size()<<","<<openlist_buf.top()->x<<","<<openlist_buf.top()->y<<"*******node中的元素有"<<node->x<<","<<node->y<<endl;
            in_opnelist = true;
            return true;
        }
            
        openlist_buf.pop();
    }
    return false;
}
bool Jps::In_closelist(NodePtr node)
{
    for(int i=0;i<closelist.size();i++)
    {
        if(node->x==closelist[i]->x && node->y == closelist[i]->y)
        {
            return true;
        }
    }
    return false;
}
void Jps::add2openlist(NodePtr curPtr)
{
    if(!is_inborder(curPtr) || !map_is_valid(curPtr))
        return;
    NodePtr fatherPtr=curPtr->camefrom;
    NodePtr jump_point=curPtr->jumpPoint;
    // cout<<"------当前父节点-fatherPtr:"<<fatherPtr->x<<","<<fatherPtr->y<<","<<fatherPtr->idx<<endl;
    //只有被遍历过的[即加入到closelist中的]才能作为父节点，中间过程中经过的点不可以作为父节点
    // -----------------------------------------------------------------------//
    while(fatherPtr->idx!=-1)
    {
        fatherPtr=fatherPtr->camefrom;
    }
    priority_queue<NodePtr,vector<NodePtr>,compute_Priority> openlist_buf=openlist;
    if(In_openlist(curPtr))
    {
        if(curPtr->gScore > fatherPtr->gScore+calc_hScore(curPtr,fatherPtr))
        {
            curPtr->gScore=fatherPtr->gScore+calc_hScore(curPtr,fatherPtr);
            curPtr->fScore=curPtr->gScore+calc_hScore(curPtr,endPtr);
            curPtr->camefrom=fatherPtr;
        }
        return;
    }
    if(In_closelist(curPtr)|| fatherPtr->idx!=-1)
        return;
    // 2.当前节点不在openlist中，将其加入openlist
    curPtr->idx=1;//将该节点加入openlist中
    curPtr->gScore=fatherPtr->gScore+calc_hScore(curPtr,fatherPtr);
    curPtr->fScore=curPtr->gScore+calc_hScore(curPtr,endPtr);
    curPtr->camefrom=fatherPtr;
    openlist.push(curPtr);//如果节点没有被访问，加入openlist中
    // cout<<"==============新加入openlist中的节点有("<<curPtr->x<<","<<curPtr->y<<","<<curPtr->idx<<")对应的f是"<<curPtr->fScore<<",对应的g是"<<curPtr->gScore<<",中间距离h:"<<calc_hScore(curPtr,fatherPtr)<<"========对应的父节点是:("<<fatherPtr->x<<","<<fatherPtr->y<<","<<fatherPtr->idx<<")对应的g是"<<fatherPtr->gScore<<endl;
}
/*   Jps查找强迫邻居
得到跳点后，查找跳点是否存在强迫邻居:

input:当前跳点curPtr，移动方向dir
output:返回值类型：bool [true or false]

存在强迫邻居 -- return true
不存在强迫邻居 -- return false
*/
// bool Jps::find_forceNebor(NodePtr curPtr,Direction dir)
// {   
//     Position  up(-1,0);//up{0,1}
//     Position down(1,0);
//     Position left(0,-1);
//     Position right(0,1);
//     Position up_left(-1,-1);
//     Position up_right(-1,1);
//     Position down_left(1,-1);
//     Position down_right(1,1);
//     //各方向跳点初始化
//     NodePtr bufPtr_up=make_shared<Node>(curPtr->x,curPtr->y);
//     NodePtr bufPtr_down=make_shared<Node>(curPtr->x,curPtr->y);
//     NodePtr bufPtr_left=make_shared<Node>(curPtr->x,curPtr->y);
//     NodePtr bufPtr_right=make_shared<Node>(curPtr->x,curPtr->y);

//     bool is_forcenebor=false;//初始化为正常节点

//     bool c1,c2;//用于判断当前节点的邻居节点的占用情况
//     int x = curPtr->x;
//     int y = curPtr->y;
//     // cout << "==================查找方向================"<<endl;
//     switch(dir)
//     {
//         // 首先从直线方向进行查找
//         case Direction::up :{
//             while(1)
//             {
//                 if(!is_inborder(bufPtr_up)||!map_is_valid(bufPtr_up) )
//                 {
//                     return is_forcenebor;//如果超过边界范围或遇到障碍物，退出
//                 }    
//                 cout<<"bufPtr_up"<<bufPtr_up->x<<","<<bufPtr_up->y<<endl;
//                 // cout<<"jumpPtr="<<curPtr->jumpPoint->x<<","<<curPtr->jumpPoint->y<<","<<curPtr->jumpPoint->idx<<endl;
                
//                 if(bufPtr_up->x == endPtr->x && bufPtr_up->y==endPtr->y ){//终点也属于跳点
//                     cout<<"跳点查询到终点";
//                     curPtr->jumpPoint=endPtr; 
//                     endPtr->camefrom=curPtr; 
//                     openlist.push(endPtr); 
//                     return true;
//                 } 

                
//                 if(is_inborder(make_shared<Node>(bufPtr_up->x+left.x,bufPtr_up->y+left.y)) )
//                     c1 = !map_is_valid(make_shared<Node>(bufPtr_up->x+left.x,bufPtr_up->y+left.y)) && map_is_valid(make_shared<Node>(bufPtr_up->x+up.x,bufPtr_up->y+up.y)) && map_is_valid(make_shared<Node>(bufPtr_up->x+up_left.x,bufPtr_up->y+up_left.y));
//                 if(is_inborder(make_shared<Node>(bufPtr_up->x+right.x,bufPtr_up->y+right.y)))
//                     c2 = !map_is_valid(make_shared<Node>(bufPtr_up->x+right.x,bufPtr_up->y+right.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_up->x+up.x,bufPtr_up->y+up.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_up->x+up_right.x,bufPtr_up->y+up_right.y,jps_col));

//                 if(c1 && c2) //表示两侧都有障碍物
//                 {
//                     cout<<"c1&&c2"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_up->x ,bufPtr_up->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_up->x + up.x,bufPtr_up->y + up.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         cout<<"jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
//                         jumpPts->force_nebor.push_back(force_nei);

//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr; 
//                         // add2openlist(bufPtr_up);
//                         is_forcenebor=true;
//                         // break;
//                     }
//                 }
//                 if (c1)
//                 {
//                     cout<<"c1"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
                    
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_up->x ,bufPtr_up->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_up->x + up_left.x,bufPtr_up->y + up_left.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         cout<<"jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
//                         jumpPts->force_nebor.push_back(force_nei);

//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr;
//                         // add2openlist(bufPtr_up);
//                         is_forcenebor=true;
//                         // break;
//                     }
//                 }
                    
//                 if (c2)
//                 {
//                     cout<<"c2"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
                    
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_up->x ,bufPtr_up->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_up->x + up_right.x,bufPtr_up->y + up_right.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         cout<<"jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
//                         jumpPts->force_nebor.push_back(force_nei);
                        
//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr;
//                         // add2openlist(bufPtr_up);
//                         is_forcenebor=true;
//                         // break;
//                     }
//                 }
//                 if(is_forcenebor==true)
//                     return true;
//                 bufPtr_up->x=bufPtr_up->x-1;//如果没有查找到强迫邻居，继续向上移动
//             }
//             break;
//         }
//         case Direction::down:{
//             while(1)
//             {
//                 if(!is_inborder(bufPtr_down) || !map_is_valid(bufPtr_down))
//                 {
//                     cout<<"超过边界范围或遇到障碍物，退出"<<endl;
//                     return is_forcenebor;//如果超过边界范围或遇到障碍物，退出
//                 }    
//                 cout<<"bufPtr_down"<<bufPtr_down->x<<","<<bufPtr_down->y<<endl;
//                 // cout<<"jumpPtr="<<curPtr->jumpPoint->x<<","<<curPtr->jumpPoint->y<<","<<curPtr->jumpPoint<<endl;
                
//                 if(bufPtr_down->x==endPtr->x && bufPtr_down->y==endPtr->y){
//                     cout<<"跳点查询到终点";
//                     curPtr->jumpPoint=endPtr; 
//                     endPtr->camefrom=curPtr; 
//                     openlist.push(endPtr); 
//                     return true;
//                 }
//                 if(is_inborder(make_shared<Node>(bufPtr_down->x+left.x,bufPtr_down->y+left.y)))
//                     c1 = !map_is_valid(make_shared<Node>(bufPtr_down->x+left.x,bufPtr_down->y+left.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_down->x+down.x,bufPtr_down->y+down.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_down->x+down_left.x,bufPtr_down->y+down_left.y,jps_col));
//                 if(is_inborder(make_shared<Node>(bufPtr_down->x+right.x,bufPtr_down->y+right.y)))
//                     c2 = !map_is_valid(make_shared<Node>(bufPtr_down->x+right.x,bufPtr_down->y+right.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_down->x+down.x,bufPtr_down->y+down.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_down->x+down_right.x,bufPtr_down->y+down_right.y,jps_col));

//                 if(c1 && c2) //表示两侧都有障碍物
//                 {
//                     cout<<"c1&&c2"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
                    
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_down->x ,bufPtr_down->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_down->x + down.x,bufPtr_down->y + down.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         cout<<"jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
//                         jumpPts->force_nebor.push_back(force_nei);

//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr;
//                         // add2openlist(bufPtr_down);
//                         is_forcenebor=true;
//                         // break;
//                     }
//                 } 
//                 if (c1)
//                 {
//                     cout<<"c1"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_down->x ,bufPtr_down->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_down->x + down_left.x,bufPtr_down->y + down_left.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         cout<<"jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
//                         jumpPts->force_nebor.push_back(force_nei);
                        
//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr;
//                         // add2openlist(bufPtr_down);
//                         is_forcenebor=true;
//                         // break;
//                     }
//                 }
//                 if (c2)
//                 {
//                     cout<<"c2"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_down->x ,bufPtr_down->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_down->x + down_right.x,bufPtr_down->y + down_right.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         cout<<"jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
//                         jumpPts->force_nebor.push_back(force_nei);
                        
//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr;
//                         // add2openlist(bufPtr_down);
//                         is_forcenebor=true;
//                         // break;
//                     }
//                 } 
//                 if(is_forcenebor==true)
//                     return true;
//                 bufPtr_down->x=bufPtr_down->x+1;//如果没有查找到强迫邻居，继续向下移动
//             }
//             break;
//         }
//         case Direction::left:{
            
//             while(1)
//             {
//                 if(!is_inborder(bufPtr_left)|| !map_is_valid(bufPtr_left) )
//                     return is_forcenebor;//如果超过边界范围或遇到障碍物，退出  
//                 cout<<"bufPtr_left"<<bufPtr_left->x<<","<<bufPtr_left->y<<endl;
//                 // cout<<"jumpPtr="<<curPtr->jumpPoint->x<<","<<curPtr->jumpPoint->y<<","<<curPtr->jumpPoint->idx<<endl;
//                 if(bufPtr_left->x==endPtr->x && bufPtr_left->y==endPtr->y){
//                     cout<<"跳点查询到终点";
//                     curPtr->jumpPoint=endPtr; 
//                     endPtr->camefrom=curPtr; 
//                     openlist.push(endPtr); 
//                     return true;
//                 }

                
//                 if(is_inborder(make_shared<Node>(bufPtr_left->x+up.x,bufPtr_left->y+up.y)) )
//                     c1 = !map_is_valid(make_shared<Node>(bufPtr_left->x+up.x,bufPtr_left->y+up.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_left->x+left.x,bufPtr_left->y+left.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_left->x+up_left.x,bufPtr_left->y+up_left.y,jps_col));
//                 if(is_inborder(make_shared<Node>(bufPtr_left->x+down.x,bufPtr_left->y+down.y)))
//                     c2 = !map_is_valid(make_shared<Node>(bufPtr_left->x+down.x,bufPtr_left->y+down.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_left->x+left.x,bufPtr_left->y+left.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_left->x+down_left.x,bufPtr_left->y+down_left.y,jps_col));
                
//                 if(c1 && c2) //表示两侧都有障碍物
//                 {
//                     cout<<"c1&&c2"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_left->x ,bufPtr_left->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_left->x + left.x,bufPtr_left->y + left.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         cout<<"jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
//                         jumpPts->force_nebor.push_back(force_nei);

//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr;
//                         // add2openlist(bufPtr_left);
//                         is_forcenebor=true;   
//                         // break;              
//                     }   
//                 }
//                 if (c1)
//                 {
//                     cout<<"c1"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_left->x ,bufPtr_left->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_left->x + up_left.x,bufPtr_left->y + up_left.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         cout<<"jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
//                         jumpPts->force_nebor.push_back(force_nei);
                        
//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr;
//                         // add2openlist(bufPtr_left);
//                         is_forcenebor=true;
//                         // break;
//                     }
//                 }
//                 if (c2)
//                 {
//                     cout<<"c2"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_left->x ,bufPtr_left->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_left->x + down_left.x,bufPtr_left->y + down_left.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         cout<<"jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
//                         jumpPts->force_nebor.push_back(force_nei);
                        
//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr;
//                         // add2openlist(bufPtr_left);
//                         is_forcenebor=true;
//                         // break;
//                     }
//                 }
//                 if(is_forcenebor==true)
//                     return true;
//                 bufPtr_left->y=bufPtr_left->y-1;//如果没有查找到强迫邻居，继续向左移动
//                 }
//             break;
//         }
//         case Direction::right:{
//             while(1)
//             {
//                 if(!is_inborder(bufPtr_right) || !map_is_valid(bufPtr_right))
//                     return is_forcenebor;//如果超过边界范围或遇到障碍物，退出
//                 cout<<"bufPtr_right"<<bufPtr_right->x<<","<<bufPtr_right->y<<endl;;
//                 // cout<<"jumpPtr="<<curPtr->jumpPoint->x<<","<<curPtr->jumpPoint->y<<","<<curPtr->jumpPoint->idx<<endl;
                
//                 if(bufPtr_right->x==endPtr->x && bufPtr_right->y==endPtr->y){
//                     curPtr->jumpPoint=endPtr; 
//                     endPtr->camefrom=curPtr; 
//                     openlist.push(endPtr); 
//                     return true;
//                 }
//                 if(is_inborder(make_shared<Node>(bufPtr_right->x+up.x,bufPtr_right->y+up.y)))
//                     c1 = !map_is_valid(make_shared<Node>(bufPtr_right->x+up.x,bufPtr_right->y+up.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_right->x+right.x,bufPtr_right->y+right.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_right->x+up_right.x,bufPtr_right->y+up_right.y,jps_col));
//                 if(is_inborder(make_shared<Node>(bufPtr_right->x+down.x,bufPtr_right->y+down.y)))
//                     c2 = !map_is_valid(make_shared<Node>(bufPtr_right->x+down.x,bufPtr_right->y+down.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_right->x+right.x,bufPtr_right->y+right.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_right->x+down_right.x,bufPtr_right->y+down_right.y,jps_col));
//                 if(c1 && c2) //表示两侧都有障碍物
//                 {
//                     cout<<"c1&&c2"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_right->x ,bufPtr_right->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_right->x + right.x,bufPtr_right->y + right.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         cout<<"jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
//                         jumpPts->force_nebor.push_back(force_nei);
                        
//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr; 
//                         // add2openlist(bufPtr_right);
//                         is_forcenebor=true;
//                         // break;
//                     }
//                 }
//                 if (c1)
//                 {
//                     cout<<"c1"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_right->x ,bufPtr_right->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_right->x + up_right.x,bufPtr_right->y + up_right.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         jumpPts->force_nebor.push_back(force_nei);

//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr;
//                         // add2openlist(bufPtr_right);
//                         is_forcenebor=true;
//                         // break;
//                     }
//                 }
//                 if (c2)
//                 {
//                     cout<<"c2"<<endl;
//                     // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
//                     NodePtr jumpPts = make_shared<Node>(bufPtr_right->x ,bufPtr_right->y);
//                     NodePtr force_nei = make_shared<Node>(bufPtr_right->x + down_right.x,bufPtr_right->y + down_right.y);
//                     force_nei->camefrom=jumpPts;
//                     if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
//                     {
//                         jumpPts->force_nebor.push_back(force_nei);
                        
//                         curPtr->jumpPoint=jumpPts;
//                         jumpPts->camefrom=curPtr;
//                         // add2openlist(bufPtr_right);
//                         is_forcenebor=true;
//                         // break;
//                     }
//                 }
//                 if(is_forcenebor==true)
//                     return true;
//                 bufPtr_right->y=bufPtr_right->y+1;//如果没有查找到强迫邻居，继续向右移动
//             }
//             break;
//         }
//         default:
//             cout<<"没有该方向"<<endl;
//             break;    
//     }
//     return is_forcenebor;
// }
bool Jps::find_forceNebor(NodePtr curPtr,Direction dir)
{   
    Position  up(-1,0);//up{0,1}
    Position down(1,0);
    Position left(0,-1);
    Position right(0,1);
    Position up_left(-1,-1);
    Position up_right(-1,1);
    Position down_left(1,-1);
    Position down_right(1,1);
    int x = curPtr->x;
    int y = curPtr->y;
    // cout<<"[find_forceNebor]: curPrt:"<<curPtr->x<<","<<curPtr->y<<endl;
    //各方向跳点初始化
    NodePtr bufPtr_up=make_shared<Node>(x,y);
    NodePtr bufPtr_down=make_shared<Node>(x,y);
    NodePtr bufPtr_left=make_shared<Node>(x,y);
    NodePtr bufPtr_right=make_shared<Node>(x,y);

    bool is_forcenebor=false;//初始化为正常节点

    bool c1,c2;//用于判断当前节点的邻居节点的占用情况
    
    // cout << "==================查找方向================"<<endl;
    switch(dir)
    {
        // 首先从直线方向进行查找
        case Direction::up :{
            while(1)
            {
                if(!is_inborder(bufPtr_up)||!map_is_valid(bufPtr_up) )
                {
                    return is_forcenebor;//如果超过边界范围或遇到障碍物，退出
                }    
                // cout<<"bufPtr_up"<<bufPtr_up->x<<","<<bufPtr_up->y<<endl;
                // cout<<"jumpPtr="<<curPtr->jumpPoint->x<<","<<curPtr->jumpPoint->y<<","<<curPtr->jumpPoint->idx<<endl;
                
                if(bufPtr_up->x == endPtr->x && bufPtr_up->y==endPtr->y ){//终点也属于跳点
                    // cout<<"跳点查询到终点";
                    curPtr->jumpPoint=endPtr; 
                    endPtr->camefrom=curPtr; 
                    openlist.push(endPtr); 
                    return true;
                } 

                NodePtr jumpPts = make_shared<Node>(bufPtr_up->x ,bufPtr_up->y);
                    
                if(is_inborder(make_shared<Node>(bufPtr_up->x+left.x,bufPtr_up->y+left.y)) )
                    c1 = !map_is_valid(make_shared<Node>(bufPtr_up->x+left.x,bufPtr_up->y+left.y)) && map_is_valid(make_shared<Node>(bufPtr_up->x+up.x,bufPtr_up->y+up.y)) && map_is_valid(make_shared<Node>(bufPtr_up->x+up_left.x,bufPtr_up->y+up_left.y));
                if(is_inborder(make_shared<Node>(bufPtr_up->x+right.x,bufPtr_up->y+right.y)))
                    c2 = !map_is_valid(make_shared<Node>(bufPtr_up->x+right.x,bufPtr_up->y+right.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_up->x+up.x,bufPtr_up->y+up.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_up->x+up_right.x,bufPtr_up->y+up_right.y,jps_col));

                if (c1)
                {
                    // cout<<"c1"<<endl;
                    // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
                    
                    NodePtr force_nei = make_shared<Node>(bufPtr_up->x + up_left.x,bufPtr_up->y + up_left.y);
                    force_nei->camefrom=jumpPts;
                    if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
                    {
                        // cout<<"curPtr"<<curPtr->x<<","<<curPtr->y<<",jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
                        jumpPts->force_nebor.push_back(force_nei);

                        curPtr->jumpPoint=jumpPts;
                        jumpPts->camefrom=curPtr;
                        // add2openlist(bufPtr_up);
                        is_forcenebor=true;
                        // break;
                    }
                }
                    
                if (c2)
                {
                    // cout<<"c2"<<endl;
                    // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
                    
                    NodePtr force_nei = make_shared<Node>(bufPtr_up->x + up_right.x,bufPtr_up->y + up_right.y);
                    force_nei->camefrom=jumpPts;
                    if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
                    {
                        // cout<<"curPtr"<<curPtr->x<<","<<curPtr->y<<",jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
                        jumpPts->force_nebor.push_back(force_nei);
                        
                        curPtr->jumpPoint=jumpPts;
                        jumpPts->camefrom=curPtr;
                        // add2openlist(bufPtr_up);
                        is_forcenebor=true;
                        // break;
                    }
                }
                bool is_curptr=false;
                if(jumpPts->x==curPtr->x && jumpPts->y == curPtr->y && In_closelist(jumpPts))
                {
                    is_curptr=true;
                    is_forcenebor=false;
                    if(jumpPts->force_nebor.size()!=0)//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
                    {
                        for(int i=0;i<jumpPts->force_nebor.size();i++)//跳点可能包含不止一个邻居节点，都要加入openlist中
                        {
                            NodePtr force_= jumpPts->force_nebor[i]; //首先选取force_nebors中的一个force_nebor
                            // cout<<"---------当前存在强迫邻居为-----"<<jumpPts->force_nebor.size()<<","<<force_->x<<","<<force_->y<<"来自于:"<<force_->camefrom->x<<","<<force_->camefrom->y<<endl;  
                            if(!In_openlist(force_) && !In_closelist(force_))
                                add2openlist(force_);
                        }
                    }
                }

                if(is_forcenebor==true && !is_curptr)
                    return true;
                bufPtr_up->x=bufPtr_up->x-1;//如果没有查找到强迫邻居，继续向上移动
            }
            break;
        }
        case Direction::down:{
            while(1)
            {
                if(!is_inborder(bufPtr_down) || !map_is_valid(bufPtr_down))
                {
                    // cout<<"超过边界范围或遇到障碍物，退出"<<endl;
                    return is_forcenebor;//如果超过边界范围或遇到障碍物，退出
                }    
                // cout<<"bufPtr_down"<<bufPtr_down->x<<","<<bufPtr_down->y<<endl;
                // cout<<"jumpPts="<<curPtr->jumpPoint->x<<","<<curPtr->jumpPoint->y<<","<<curPtr->jumpPoint<<endl;
                
                if(bufPtr_down->x==endPtr->x && bufPtr_down->y==endPtr->y){
                    // cout<<"跳点查询到终点";
                    curPtr->jumpPoint=endPtr; 
                    endPtr->camefrom=curPtr; 
                    openlist.push(endPtr); 
                    return true;
                }
                NodePtr jumpPts = make_shared<Node>(bufPtr_down->x ,bufPtr_down->y);
                    
                if(is_inborder(make_shared<Node>(bufPtr_down->x+left.x,bufPtr_down->y+left.y)))
                    c1 = !map_is_valid(make_shared<Node>(bufPtr_down->x+left.x,bufPtr_down->y+left.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_down->x+down.x,bufPtr_down->y+down.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_down->x+down_left.x,bufPtr_down->y+down_left.y,jps_col));
                if(is_inborder(make_shared<Node>(bufPtr_down->x+right.x,bufPtr_down->y+right.y)))
                    c2 = !map_is_valid(make_shared<Node>(bufPtr_down->x+right.x,bufPtr_down->y+right.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_down->x+down.x,bufPtr_down->y+down.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_down->x+down_right.x,bufPtr_down->y+down_right.y,jps_col));

                if (c1)
                {
                    // cout<<"c1"<<endl;
                    // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
                    NodePtr force_nei = make_shared<Node>(bufPtr_down->x + down_left.x,bufPtr_down->y + down_left.y);
                    force_nei->camefrom=jumpPts;
                    if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
                    {
                        // cout<<"curPtr"<<curPtr->x<<","<<curPtr->y<<",jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
                        jumpPts->force_nebor.push_back(force_nei);
                        
                        curPtr->jumpPoint=jumpPts;
                        jumpPts->camefrom=curPtr;
                        // add2openlist(bufPtr_down);
                        is_forcenebor=true;
                        // break;
                    }
                }
                if (c2)
                {
                    // cout<<"c2"<<endl;
                    // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
                    NodePtr force_nei = make_shared<Node>(bufPtr_down->x + down_right.x,bufPtr_down->y + down_right.y);
                    force_nei->camefrom=jumpPts;
                    if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
                    {
                        // cout<<"curPtr"<<curPtr->x<<","<<curPtr->y<<",jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
                        jumpPts->force_nebor.push_back(force_nei);
                        
                        curPtr->jumpPoint=jumpPts;
                        jumpPts->camefrom=curPtr;
                        // add2openlist(bufPtr_down);
                        is_forcenebor=true;
                        // break;
                    }
                } 
                bool is_curptr=false;
                if(jumpPts->x==curPtr->x && jumpPts->y == curPtr->y && In_closelist(jumpPts))
                {
                    is_curptr=true;
                    is_forcenebor=false;
                    if(jumpPts->force_nebor.size()!=0)//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
                    {
                        for(int i=0;i<jumpPts->force_nebor.size();i++)//跳点可能包含不止一个邻居节点，都要加入openlist中
                        {
                            NodePtr force_= jumpPts->force_nebor[i]; //首先选取force_nebors中的一个force_nebor
                            // cout<<"---------当前存在强迫邻居为-----"<<jumpPts->force_nebor.size()<<","<<force_->x<<","<<force_->y<<"来自于:"<<force_->camefrom->x<<","<<force_->camefrom->y<<endl;  
                            if(!In_openlist(force_) && !In_closelist(force_))
                                add2openlist(force_);
                        }
                    }
                }

                if(is_forcenebor==true && !is_curptr)
                    return true;
                bufPtr_down->x=bufPtr_down->x+1;//如果没有查找到强迫邻居，继续向下移动
            }
            break;
        }
        case Direction::left:{
            
            while(1)
            {
                if(!is_inborder(bufPtr_left)|| !map_is_valid(bufPtr_left) )
                    return is_forcenebor;//如果超过边界范围或遇到障碍物，退出  
                // cout<<"bufPtr_left"<<bufPtr_left->x<<","<<bufPtr_left->y<<endl;
                // cout<<"jumpPts="<<curPtr->jumpPoint->x<<","<<curPtr->jumpPoint->y<<","<<curPtr->jumpPoint->idx<<endl;
                if(bufPtr_left->x==endPtr->x && bufPtr_left->y==endPtr->y){
                    // cout<<"跳点查询到终点";
                    curPtr->jumpPoint=endPtr; 
                    endPtr->camefrom=curPtr; 
                    openlist.push(endPtr); 
                    return true;
                }

                NodePtr jumpPts = make_shared<Node>(bufPtr_left->x ,bufPtr_left->y);
                    
                if(is_inborder(make_shared<Node>(bufPtr_left->x+up.x,bufPtr_left->y+up.y)) )
                    c1 = !map_is_valid(make_shared<Node>(bufPtr_left->x+up.x,bufPtr_left->y+up.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_left->x+left.x,bufPtr_left->y+left.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_left->x+up_left.x,bufPtr_left->y+up_left.y,jps_col));
                if(is_inborder(make_shared<Node>(bufPtr_left->x+down.x,bufPtr_left->y+down.y)))
                    c2 = !map_is_valid(make_shared<Node>(bufPtr_left->x+down.x,bufPtr_left->y+down.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_left->x+left.x,bufPtr_left->y+left.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_left->x+down_left.x,bufPtr_left->y+down_left.y,jps_col));
                
                if (c1)
                {
                    // cout<<"c1"<<endl;
                    // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
                    NodePtr force_nei = make_shared<Node>(bufPtr_left->x + up_left.x,bufPtr_left->y + up_left.y);
                    force_nei->camefrom=jumpPts;
                    if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
                    {
                        // cout<<"curPtr"<<curPtr->x<<","<<curPtr->y<<",jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
                        jumpPts->force_nebor.push_back(force_nei);
                        
                        curPtr->jumpPoint=jumpPts;
                        jumpPts->camefrom=curPtr;
                        // add2openlist(bufPtr_left);
                        is_forcenebor=true;
                        // break;
                    }
                }
                if (c2)
                {
                    // cout<<"c2"<<endl;
                    // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
                    NodePtr force_nei = make_shared<Node>(bufPtr_left->x + down_left.x,bufPtr_left->y + down_left.y);
                    force_nei->camefrom=jumpPts;
                    if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
                    {
                        // cout<<"curPtr"<<curPtr->x<<","<<curPtr->y<<",jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
                        jumpPts->force_nebor.push_back(force_nei);
                        
                        curPtr->jumpPoint=jumpPts;
                        jumpPts->camefrom=curPtr;
                        // add2openlist(bufPtr_left);
                        is_forcenebor=true;
                        // break;
                    }
                }
                bool is_curptr=false;
                if(jumpPts->x==curPtr->x && jumpPts->y == curPtr->y && In_closelist(jumpPts))
                {
                    is_curptr=true;
                    is_forcenebor=false;
                    if(jumpPts->force_nebor.size()!=0)//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
                    {
                        for(int i=0;i<jumpPts->force_nebor.size();i++)//跳点可能包含不止一个邻居节点，都要加入openlist中
                        {
                            NodePtr force_= jumpPts->force_nebor[i]; //首先选取force_nebors中的一个force_nebor
                            // cout<<"---------当前存在强迫邻居为-----"<<jumpPts->force_nebor.size()<<","<<force_->x<<","<<force_->y<<"来自于:"<<force_->camefrom->x<<","<<force_->camefrom->y<<endl;  
                            if(!In_openlist(force_) && !In_closelist(force_))
                                add2openlist(force_);
                        }
                    }
                }

                if(is_forcenebor==true && !is_curptr)
                    return true;
                bufPtr_left->y=bufPtr_left->y-1;//如果没有查找到强迫邻居，继续向左移动
                }
            break;
        }
        case Direction::right:{
            while(1)
            {
                if(!is_inborder(bufPtr_right) || !map_is_valid(bufPtr_right))
                    return is_forcenebor;//如果超过边界范围或遇到障碍物，退出
                // cout<<"bufPtr_right"<<bufPtr_right->x<<","<<bufPtr_right->y<<endl;;
                // cout<<"jumpPts="<<curPtr->jumpPoint->x<<","<<curPtr->jumpPoint->y<<","<<curPtr->jumpPoint->idx<<endl;
                
                if(bufPtr_right->x==endPtr->x && bufPtr_right->y==endPtr->y){
                    curPtr->jumpPoint=endPtr; 
                    endPtr->camefrom=curPtr; 
                    openlist.push(endPtr); 
                    return true;
                }
                NodePtr jumpPts = make_shared<Node>(bufPtr_right->x ,bufPtr_right->y);
                if(is_inborder(make_shared<Node>(bufPtr_right->x+up.x,bufPtr_right->y+up.y)))
                    c1 = !map_is_valid(make_shared<Node>(bufPtr_right->x+up.x,bufPtr_right->y+up.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_right->x+right.x,bufPtr_right->y+right.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_right->x+up_right.x,bufPtr_right->y+up_right.y,jps_col));
                if(is_inborder(make_shared<Node>(bufPtr_right->x+down.x,bufPtr_right->y+down.y)))
                    c2 = !map_is_valid(make_shared<Node>(bufPtr_right->x+down.x,bufPtr_right->y+down.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_right->x+right.x,bufPtr_right->y+right.y,jps_col)) && map_is_valid(make_shared<Node>(bufPtr_right->x+down_right.x,bufPtr_right->y+down_right.y,jps_col));
                if (c1)
                {
                    // cout<<"c1"<<endl;
                    // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
                    NodePtr force_nei = make_shared<Node>(bufPtr_right->x + up_right.x,bufPtr_right->y + up_right.y);
                    force_nei->camefrom=jumpPts;
                    if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
                    {
                        // cout<<"curPtr"<<curPtr->x<<","<<curPtr->y<<",jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
                        
                        jumpPts->force_nebor.push_back(force_nei);

                        curPtr->jumpPoint=jumpPts;
                        jumpPts->camefrom=curPtr;
                        // add2openlist(bufPtr_right);
                        is_forcenebor=true;
                        // break;
                    }
                }
                if (c2)
                {
                    // cout<<"c2"<<endl;
                    // 此处应该单独设定指针jumpPts，初始化为bufPtr_up，而不能让force_nei->camefrom，curPtr->jumpPoint这类直接指向bufPtr_up，不然在if条件外，若bufPtr_up发生改变，则相应指向相同内存的指针数值也会发生改变!!
                    NodePtr force_nei = make_shared<Node>(bufPtr_right->x + down_right.x,bufPtr_right->y + down_right.y);
                    force_nei->camefrom=jumpPts;
                    if(map_is_valid(force_nei) && is_inborder(force_nei))//如果节点没有被占据，将该节点加入到邻居节点中
                    {
                        // cout<<"curPtr"<<curPtr->x<<","<<curPtr->y<<",jumpPts"<<jumpPts->x<<"x"<<jumpPts->y<<",force_nei:"<<force_nei->x<<","<<force_nei->y<<endl;
                        
                        jumpPts->force_nebor.push_back(force_nei);
                        
                        curPtr->jumpPoint=jumpPts;
                        jumpPts->camefrom=curPtr;
                        // add2openlist(bufPtr_right);
                        is_forcenebor=true;
                        // break;
                    }
                }
                bool is_curptr=false;
                if(jumpPts->x==curPtr->x && jumpPts->y == curPtr->y && In_closelist(jumpPts))
                {
                    is_curptr=true;
                    is_forcenebor=false;
                    if(jumpPts->force_nebor.size()!=0)//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
                    {
                        for(int i=0;i<jumpPts->force_nebor.size();i++)//跳点可能包含不止一个邻居节点，都要加入openlist中
                        {
                            NodePtr force_= jumpPts->force_nebor[i]; //首先选取force_nebors中的一个force_nebor
                            // cout<<"---------当前存在强迫邻居为-----"<<jumpPts->force_nebor.size()<<","<<force_->x<<","<<force_->y<<"来自于:"<<force_->camefrom->x<<","<<force_->camefrom->y<<endl;  
                            if(!In_openlist(force_) && !In_closelist(force_))
                                add2openlist(force_);
                        }
                    }
                }
                if(is_forcenebor==true && !is_curptr)
                    return true;
                bufPtr_right->y=bufPtr_right->y+1;//如果没有查找到强迫邻居，继续向右移动
            }
            break;
        }
        default:
            cout<<"没有该方向"<<endl;
            break;    
    }
    return is_forcenebor;
}


/*
    input param: Direction ,NodePtr
    output : 将jump_point加入到openlist中
递归寻找跳点
    if 当前点是终点
        return 终点;
    if 方向是斜向：
        if 传入点存在强迫邻居find_forceNebor：
            return 跳点
        if(递归寻找跳点(x+1,y，朝向横向))不为空
            返回此传入点
        if(递归寻找跳点(x,y+1，朝向纵向))不为空
            返回此传入点
    elseif 方向为水平:
        if 传入点存在强迫邻居find_forceNebor:
            return 跳点
    elseif 方向是纵向
        if 传入点存在强迫邻居find_forceNebor:
            return 跳点
    return （递归寻找跳点(x+1,y+1,朝向对角)）
*/
// void Jps::find_JumpPoint(NodePtr curPtr)
// {
//     //1.如果没有父节点，第一次需要把所有方向都遍历一遍
//     //2.如果已经有了curPtr和dir，确定下一次dir的时候最多有三个方向--通过find_forceNebor进行判断
//     // 3.如果找到跳点，加入列表中
//     Position up_left(-1,-1);
//     Position up_right(-1,1);
//     Position down_left(1,-1);
//     Position down_right(1,1);
//     if(!is_inborder(curPtr)||!map_is_valid(curPtr))
//     {
//         // cout<<"超出范围"<<endl;
//         return ;
//     } 
//     if(curPtr == startPtr)
//     {   //第一个节点要遍历周围8个方向，先直线，再斜向
//         flag_start++;
//         // cout<<"!!!!!!!!!!!当前遍历的是初始节点!!!!!!!!!!!!!!!!"<<endl;
//         if(flag_start ==1 )
//         {
//             curPtr->dirList.push(Direction::left);
//             curPtr->dirList.push(Direction::right);
//             curPtr->dirList.push(Direction::up);
//             curPtr->dirList.push(Direction::down);
//             curPtr->dirList.push(Direction::up_left);
//             curPtr->dirList.push(Direction::up_right);
//             curPtr->dirList.push(Direction::down_left);
//             curPtr->dirList.push(Direction::down_right);
//         }
//     }   
//     else if(curPtr->force_nebor.size()!=0 && In_closelist(curPtr))
//     {
//         // cout<<"-------------------force_nebor_size:"<<curPtr->force_nebor.size();
//         for(int i=0;i<curPtr->force_nebor.size();i++)
//         {
//             NodePtr force_= curPtr->force_nebor[i]; //首先选取force_nebors中的一个force_nebor
//             // cout<<"---------当前存在强迫邻居为-----"<<curPtr->force_nebor.size()<<","<<force_->x<<","<<force_->y<<"来自于:"<<force_->camefrom->x<<","<<force_->camefrom->y<<endl;  
//             if(!In_openlist(force_) && !In_closelist(force_))
//                 add2openlist(force_);

//             int dx,dy;
//             Direction dir;//定义一个Direction类型，用于判定从父节点到子节点的移动方向是枚举类型的哪一种
    
//             dx=force_->x-curPtr->x;
//             dy=force_->y-curPtr->y;
//             if(dx <= -1 && dy == 0)//up{0,1}
//                 dir=Direction::up;
//             if(dx >= 1 && dy == 0)//down{0,-1}
//                 dir=Direction::down;
//             if(dx == 0 && dy <= -1)//left{-1,0}
//                 dir=Direction::left;
//             if(dx == 0 && dy >= 1)//right{1,0}
//                 dir=Direction::right;
//             if(dx <= -1 && dy <= -1)//up_left{-1,1}
//                 dir=Direction::up_left;
//             if(dx <= -1 && dy >= 1)//up_right{1,1}
//                 dir=Direction::up_right;
//             if(dx >= 1&& dy <= -1)//down_left{-1,-1}
//                 dir=Direction::down_left;
//             if(dx >= 1&& dy >= 1)//down_right{1,-1}
//                 dir=Direction::down_right;
//             if(dx == 0 && dy ==0 )
//                 dir=Direction::none_; 
//             curPtr->dirList.push(dir);
//         }
//     }
//     else
//     {
//         // cout<<"find_JumpPoint-当前节点存在父节点"<<endl;
//         while(!curPtr->dirList.empty())
//             curPtr->dirList.pop();
//         get_direction(curPtr->camefrom,curPtr);
//     }
   
//     while(!curPtr->dirList.empty())//如果当前节点存在遍历方向，继续向下执行
//     {
//         // 此部分用于显示当前节点curPtr存在的方向
//         // -----------------------------------------------------//
//         NodePtr buf_=make_shared<Node>();
//         buf_->dirList=curPtr->dirList;
//         while(!buf_->dirList.empty())
//         {
//             // cout<<"当前遍历的跳点("<<curPtr->x<<","<<curPtr->y<<")对应的方向有:"<<buf_->dirList.front()<<endl;
//             buf_->dirList.pop();
//         }
//         // cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
//         // -----------------------------------------------------//
//         //获得遍历的第一个方向【首先水平,然后斜向】
//         Direction dir=curPtr->dirList.front();
//         curPtr->dirList.pop();
//         if(dir == Direction::down_left || dir ==Direction::down_right || dir ==Direction::up_left || dir ==Direction::up_right)//表示如果判定得到强迫节点在斜线方向上
//         {
            
//             if(dir==Direction::down_left )
//             {
//                 NodePtr newPtr_ve=make_shared<Node>(); 
//                 newPtr_ve->x=curPtr->x+down_left.x;
//                 newPtr_ve->y=curPtr->y+down_left.y;
//                 newPtr_ve->camefrom=curPtr;
//                 newPtr_ve->is_diagonal=true;
//                 find_JumpPoint(newPtr_ve);
//             }
//             if(dir==Direction::down_right )
//             {
//                 NodePtr newPtr_ve=make_shared<Node>();  
//                 newPtr_ve->x=curPtr->x+down_right.x;
//                 newPtr_ve->y=curPtr->y+down_right.y;
//                 newPtr_ve->camefrom=curPtr;
//                 newPtr_ve->is_diagonal=true;
//                 find_JumpPoint(newPtr_ve);
//             }
//             if(dir==Direction::up_left )
//             {
//                 NodePtr newPtr_ve=make_shared<Node>(); 
//                 newPtr_ve->x = curPtr->x+up_left.x;
//                 newPtr_ve->y = curPtr->y+up_left.y;
//                 newPtr_ve->camefrom=curPtr;
//                 newPtr_ve->is_diagonal=true;
//                 find_JumpPoint(newPtr_ve);
//             }            
//             if(dir==Direction::up_right )
//             {
                
//                 NodePtr newPtr_ve=make_shared<Node>();
//                 newPtr_ve->x=curPtr->x+up_right.x;
//                 newPtr_ve->y=curPtr->y+up_right.y;
//                 newPtr_ve->camefrom=curPtr;
//                 newPtr_ve->is_diagonal=true;
//                 find_JumpPoint(newPtr_ve);
//             }         
//          }
//         if(dir == Direction::left || dir ==Direction::right)
//         {
//             if(find_forceNebor(curPtr,dir))
//             {
//                 // cout<<"水平遍历的过程中已经找到了强迫邻居和对应跳点"<<curPtr->x<<","<<curPtr->y<<","<<curPtr->idx<<"当前节点是否在openlist中（0-不在，1-在)"<<In_openlist(curPtr)<<endl;
//                 // cout<<"jumpPtr="<<curPtr->jumpPoint->x<<","<<curPtr->jumpPoint->y<<","<<curPtr->jumpPoint->idx<<endl;
//                 if(!In_openlist(curPtr) && !In_closelist(curPtr))
//                     add2openlist(curPtr);

//                 NodePtr jumpPtr=curPtr->jumpPoint; 
//                 if(In_closelist(curPtr)&& jumpPtr!=nullptr )//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
//                 {
//                     if(!In_openlist(jumpPtr) && !In_closelist(jumpPtr))
//                         add2openlist(jumpPtr);
//                     if(jumpPtr->force_nebor.size()!=0 && In_closelist(jumpPtr))//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
//                     {
//                         for(int i=0;i<jumpPtr->force_nebor.size();i++)//跳点可能包含不止一个邻居节点，都要加入openlist中
//                         {
//                             NodePtr force_= jumpPtr->force_nebor[i]; //首先选取force_nebors中的一个force_nebor
//                             // cout<<"---------当前存在强迫邻居为-----"<<jumpPtr->force_nebor.size()<<","<<force_->x<<","<<force_->y<<"来自于:"<<force_->camefrom->x<<","<<force_->camefrom->y<<endl;  
//                             if(!In_openlist(force_) && !In_closelist(force_))
//                                 add2openlist(force_);
//                         }
//                     }
//                 }
//                 if(curPtr->is_diagonal==true)
//                 {
//                     // cout<<"!!!!!!!is_diagonal--true!!!!!!!!!"<<endl;
//                     break;
//                 }  
//                 else
//                 {
//                     // cout<<"!!!!!!!is_diagonal--false!!!!!!!!!"<<endl;
//                     continue;
//                 }
//             }                   
//         }
//         if(dir == Direction::up || dir ==Direction::down) //对于方向是纵向的情况
//         {
//             if(find_forceNebor(curPtr,dir))
//             {
//                 // cout<<"竖直遍历的过程中已经找到了强迫邻居和对应跳点"<<curPtr->x<<","<<curPtr->y<<","<<map_is_valid(curPtr)<<endl;
//                 if(!In_openlist(curPtr) && !In_closelist(curPtr))
//                     add2openlist(curPtr);
//                 NodePtr jumpPtr=curPtr->jumpPoint; 
//                 if(In_closelist(curPtr)&& jumpPtr!=nullptr )//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
//                 {

//                     if(!In_openlist(jumpPtr) && !In_closelist(jumpPtr))
//                         add2openlist(jumpPtr);
//                     if(jumpPtr->force_nebor.size()!=0 && In_closelist(jumpPtr))//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
//                     {
//                         for(int i=0;i<jumpPtr->force_nebor.size();i++)//跳点可能包含不止一个邻居节点，都要加入openlist中
//                         {
//                             NodePtr force_= jumpPtr->force_nebor[i]; //首先选取force_nebors中的一个force_nebor
//                             // cout<<"---------当前存在强迫邻居为-----"<<jumpPtr->force_nebor.size()<<","<<force_->x<<","<<force_->y<<"来自于:"<<force_->camefrom->x<<","<<force_->camefrom->y<<endl;  
//                             if(!In_openlist(force_) && !In_closelist(force_))
//                                 add2openlist(force_);
//                         }
//                     }
//                 }
//                 if(curPtr->is_diagonal==true)
//                 {
//                     // cout<<"!!!!!!!is_diagonal--true!!!!!!!!!"<<endl;
//                     break;
//                 }  
//                 else
//                 {
//                     // cout<<"!!!!!!!is_diagonal--false!!!!!!!!!"<<endl;
//                     continue;
//                 }
//             }
                
//         }
//     }                        
// }
void Jps::find_JumpPoint(NodePtr curPtr)
{
    //1.如果没有父节点，第一次需要把所有方向都遍历一遍
    //2.如果已经有了curPtr和dir，确定下一次dir的时候最多有三个方向--通过find_forceNebor进行判断
    // 3.如果找到跳点，加入列表中
    Position up_left(-1,-1);
    Position up_right(-1,1);
    Position down_left(1,-1);
    Position down_right(1,1);
    if(!is_inborder(curPtr)||!map_is_valid(curPtr))
    {
        // cout<<"超出范围"<<endl;
        return ;
    } 
    // cout<<"-------------------force_nebor_size:"<<curPtr->force_nebor.size()<<endl<<"In_closelist(curPtr):"<<In_closelist(curPtr)<<endl;
    
    if(curPtr == startPtr)
    {   //第一个节点要遍历周围8个方向，先直线，再斜向
        flag_start++;
        // cout<<"!!!!!!!!!!!当前遍历的是初始节点!!!!!!!!!!!!!!!!"<<endl;
        if(flag_start ==1 )
        {
            curPtr->dirList.push(Direction::left);
            curPtr->dirList.push(Direction::right);
            curPtr->dirList.push(Direction::up);
            curPtr->dirList.push(Direction::down);
            curPtr->dirList.push(Direction::up_left);
            curPtr->dirList.push(Direction::up_right);
            curPtr->dirList.push(Direction::down_left);
            curPtr->dirList.push(Direction::down_right);
        }
    }   
        
    else if(curPtr->force_nebor.size()!=0 && In_closelist(curPtr))
    {
        for(int i=0;i<curPtr->force_nebor.size();i++)
        {
            NodePtr force_= curPtr->force_nebor[i]; //首先选取force_nebors中的一个force_nebor
            // cout<<"---------当前存在强迫邻居为-----"<<curPtr->force_nebor.size()<<","<<force_->x<<","<<force_->y<<"来自于:"<<force_->camefrom->x<<","<<force_->camefrom->y<<endl;  
            if(!In_openlist(force_) && !In_closelist(force_))
                add2openlist(force_);

            int dx,dy;
            Direction dir;//定义一个Direction类型，用于判定从父节点到子节点的移动方向是枚举类型的哪一种
    
            dx=force_->x-curPtr->x;
            dy=force_->y-curPtr->y;
            if(dx <= -1 && dy == 0)//up{0,1}
                dir=Direction::up;
            if(dx >= 1 && dy == 0)//down{0,-1}
                dir=Direction::down;
            if(dx == 0 && dy <= -1)//left{-1,0}
                dir=Direction::left;
            if(dx == 0 && dy >= 1)//right{1,0}
                dir=Direction::right;
            if(dx <= -1 && dy <= -1)//up_left{-1,1}
            {
                curPtr->dirList.push(Direction::up_left);
                curPtr->dirList.push(Direction::up);
                curPtr->dirList.push(Direction::left);
            }
            if(dx <= -1 && dy >= 1)//up_right{1,1}
            {
                curPtr->dirList.push(Direction::up_right);
                curPtr->dirList.push(Direction::up);
                curPtr->dirList.push(Direction::right);
            }
            if(dx >= 1&& dy <= -1)//down_left{-1,-1}
            {
                curPtr->dirList.push(Direction::down_left);
                curPtr->dirList.push(Direction::down);
                curPtr->dirList.push(Direction::left);
            }
            if(dx >= 1&& dy >= 1)//down_right{1,-1}
            {
                curPtr->dirList.push(Direction::down_right);
                curPtr->dirList.push(Direction::down);
                curPtr->dirList.push(Direction::right);
            }
                
            if(dx == 0 && dy ==0 )
                curPtr->dirList.push(Direction::none_);
        }
    }
    else
    {
        // cout<<"find_JumpPoint-当前节点存在父节点"<<endl;
        while(!curPtr->dirList.empty())
            curPtr->dirList.pop();
        get_direction(curPtr->camefrom,curPtr);
    }
   
    while(!curPtr->dirList.empty())//如果当前节点存在遍历方向，继续向下执行
    {
        // 此部分用于显示当前节点curPtr存在的方向
        // -----------------------------------------------------//
        NodePtr buf_=make_shared<Node>();
        buf_->dirList=curPtr->dirList;
        while(!buf_->dirList.empty())
        {
            // cout<<"当前遍历的跳点("<<curPtr->x<<","<<curPtr->y<<")对应的方向有:"<<buf_->dirList.front()<<endl;
            buf_->dirList.pop();
        }
        // cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
        // -----------------------------------------------------//
        //获得遍历的第一个方向【首先水平,然后斜向】
        Direction dir=curPtr->dirList.front();
        curPtr->dirList.pop();
        Position  up(-1,0);//up{0,1}
        Position down(1,0);
        Position left(0,-1);
        Position right(0,1);
        Position up_left(-1,-1);
        Position up_right(-1,1);
        Position down_left(1,-1);
        Position down_right(1,1);
        if(dir == Direction::down_left || dir ==Direction::down_right || dir ==Direction::up_left || dir ==Direction::up_right)//表示如果判定得到强迫节点在斜线方向上
        {
            
            if(dir==Direction::down_left )
            {
                NodePtr newPtr_ve=make_shared<Node>(); 
                NodePtr leftPtr=make_shared<Node>(curPtr->x+left.x,curPtr->y+left.y);
                NodePtr downPtr=make_shared<Node>(curPtr->x+down.x,curPtr->y+down.y);
                // cout<<"leftPtr"<<leftPtr->x<<","<<leftPtr->y<<",downPtr"<<downPtr->x<<","<<downPtr->y;
                if(!map_is_valid(leftPtr) && !map_is_valid(downPtr))
                {
                    // cout<<"当前左侧和下侧都有障碍"<<endl;
                }
                else{
                    newPtr_ve->x=curPtr->x+down_left.x;
                    newPtr_ve->y=curPtr->y+down_left.y;
                    newPtr_ve->camefrom=curPtr;
                    newPtr_ve->is_diagonal=true;
                    find_JumpPoint(newPtr_ve);
                }
                
            }
            if(dir==Direction::down_right )
            {
                NodePtr newPtr_ve=make_shared<Node>();  
                NodePtr rightPtr=make_shared<Node>(curPtr->x+right.x,curPtr->y+right.y);
                NodePtr downPtr=make_shared<Node>(curPtr->x+down.x,curPtr->y+down.y);
                // cout<<"rightPtr"<<rightPtr->x<<","<<rightPtr->y<<",downPtr"<<downPtr->x<<","<<downPtr->y;
                
                if(!map_is_valid(rightPtr) && !map_is_valid(downPtr))
                {
                    // cout<<"当前右侧和下侧都有障碍"<<endl;
                }
                else{
                    newPtr_ve->x=curPtr->x+down_right.x;
                    newPtr_ve->y=curPtr->y+down_right.y;
                    newPtr_ve->camefrom=curPtr;
                    newPtr_ve->is_diagonal=true;
                    find_JumpPoint(newPtr_ve);
                }
            }
            if(dir==Direction::up_left )
            {
                NodePtr newPtr_ve=make_shared<Node>(); 
                NodePtr leftPtr=make_shared<Node>(curPtr->x+left.x,curPtr->y+left.y);
                NodePtr upPtr=make_shared<Node>(curPtr->x+up.x,curPtr->y+up.y);
                if(!map_is_valid(leftPtr) && !map_is_valid(upPtr))
                {
                    // cout<<"当前左侧和上侧都有障碍"<<endl;
                }
                else{
                    newPtr_ve->x = curPtr->x+up_left.x;
                    newPtr_ve->y = curPtr->y+up_left.y;
                    newPtr_ve->camefrom=curPtr;
                    newPtr_ve->is_diagonal=true;
                    find_JumpPoint(newPtr_ve);
                }
            }            
            if(dir==Direction::up_right )
            {
                
                NodePtr newPtr_ve=make_shared<Node>();
                NodePtr rightPtr=make_shared<Node>(curPtr->x+right.x,curPtr->y+right.y);
                NodePtr upPtr=make_shared<Node>(curPtr->x+up.x,curPtr->y+up.y);
                if(!map_is_valid(rightPtr) && !map_is_valid(upPtr))
                {
                    // cout<<"当前右侧和上侧都有障碍"<<endl;
                }
                else{
                    newPtr_ve->x=curPtr->x+up_right.x;
                    newPtr_ve->y=curPtr->y+up_right.y;
                    newPtr_ve->camefrom=curPtr;
                    newPtr_ve->is_diagonal=true;
                    find_JumpPoint(newPtr_ve);
                }
            }         
         }
        if(dir == Direction::left || dir ==Direction::right)
        {
            if(find_forceNebor(curPtr,dir))
            {
                // cout<<"水平遍历的过程中已经找到了强迫邻居和对应跳点"<<curPtr->x<<","<<curPtr->y<<","<<curPtr->idx<<"当前节点是否在openlist中（0-不在，1-在)"<<In_openlist(curPtr)<<endl;
                // cout<<"jumpPtr="<<curPtr->jumpPoint->x<<","<<curPtr->jumpPoint->y<<","<<curPtr->jumpPoint->idx<<endl;
                if(!In_openlist(curPtr) && !In_closelist(curPtr))
                    add2openlist(curPtr);

                NodePtr jumpPtr=curPtr->jumpPoint; 
                // cout<<"In_closelist:"<<In_closelist(curPtr)<<",jumpPtr:"<<jumpPtr->x<<","<<jumpPtr->y<<endl;
                if(In_closelist(curPtr)&& jumpPtr!=nullptr )//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
                {
                    if(!In_openlist(jumpPtr) && !In_closelist(jumpPtr))
                        add2openlist(jumpPtr);
                    if(jumpPtr->force_nebor.size()!=0 && In_closelist(jumpPtr))//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
                    {
                        for(int i=0;i<jumpPtr->force_nebor.size();i++)//跳点可能包含不止一个邻居节点，都要加入openlist中
                        {
                            NodePtr force_= jumpPtr->force_nebor[i]; //首先选取force_nebors中的一个force_nebor
                            // cout<<"---------当前存在强迫邻居为-----"<<jumpPtr->force_nebor.size()<<","<<force_->x<<","<<force_->y<<"来自于:"<<force_->camefrom->x<<","<<force_->camefrom->y<<endl;  
                            if(!In_openlist(force_) && !In_closelist(force_))
                                add2openlist(force_);
                        }
                    }
                }
                if(curPtr->is_diagonal==true)
                {
                    // cout<<"!!!!!!!is_diagonal--true!!!!!!!!!"<<endl;
                    break;
                }  
                else
                {
                    // cout<<"!!!!!!!is_diagonal--false!!!!!!!!!"<<endl;
                    continue;
                }
            }                   
        }
        if(dir == Direction::up || dir ==Direction::down) //对于方向是纵向的情况
        {
            if(find_forceNebor(curPtr,dir))
            {
                // cout<<"竖直遍历的过程中已经找到了强迫邻居和对应跳点"<<curPtr->x<<","<<curPtr->y<<","<<map_is_valid(curPtr)<<endl;
                if(!In_openlist(curPtr) && !In_closelist(curPtr))
                    add2openlist(curPtr);
                NodePtr jumpPtr=curPtr->jumpPoint; 
                if(In_closelist(curPtr)&& jumpPtr!=nullptr )//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
                {

                    if(!In_openlist(jumpPtr) && !In_closelist(jumpPtr))
                        add2openlist(jumpPtr);
                    if(jumpPtr->force_nebor.size()!=0 && In_closelist(jumpPtr))//邻居节点加入的条件：当前跳点已经被遍历，并且当前跳点存在邻居节点
                    {
                        for(int i=0;i<jumpPtr->force_nebor.size();i++)//跳点可能包含不止一个邻居节点，都要加入openlist中
                        {
                            NodePtr force_= jumpPtr->force_nebor[i]; //首先选取force_nebors中的一个force_nebor
                            // cout<<"---------当前存在强迫邻居为-----"<<jumpPtr->force_nebor.size()<<","<<force_->x<<","<<force_->y<<"来自于:"<<force_->camefrom->x<<","<<force_->camefrom->y<<endl;  
                            if(!In_openlist(force_) && !In_closelist(force_))
                                add2openlist(force_);
                        }
                    }
                }
                if(curPtr->is_diagonal==true)
                {
                    // cout<<"!!!!!!!is_diagonal--true!!!!!!!!!"<<endl;
                    break;
                }  
                else
                {
                    // cout<<"!!!!!!!is_diagonal--false!!!!!!!!!"<<endl;
                    continue;
                }
            }
            
                
        }
    }                        
}
//【代码核心】jps寻路算法
bool get_endPts=false;
void Jps::findPath_JPS()
{
    path.clear();
    while(!openlist.empty())
    {
        openlist.pop();
    }
    closelist.clear();
    // cout<<"开始查找路径"<<endl;
    //startPtr和endPtr在initMap中初始化，因此此处startPtr和endPtr均为已知
    startPtr->gScore=0;
    startPtr->fScore=startPtr->gScore+calc_hScore(startPtr,endPtr);
    startPtr->camefrom=startPtr;
    openlist.push(startPtr);//将第一个节点加入到openlist中

    //遍历openlist中存储的跳点，从而对跳点进行探索获得完整的jps路径
    while(!openlist.empty())
    {
    //第一步，从f(n)中选取代价最小的节点
        NodePtr curPtr=openlist.top();//优先队列使用.top()选取当前节点
        openlist.pop();//将选取过的当前指针提取出来 
        // cout<<"当前遍历的节点是："<<curPtr->x<<","<<curPtr->y<<","<<curPtr->idx<<","<<curPtr->fScore<<endl<<"openlist的大小为:"<<openlist.size()<<endl;
        if(!map_is_valid(curPtr) || !is_inborder(curPtr))
        {
            // cout<<"!!!!!!!当前节点不在可行范围内!!!该节点不能够执行后续操作!!!!!!!!!"<<endl;
            continue;
        }
        curPtr->idx=-1;//将已经遍历过的点的idx设置为-1，即加入closelist中【表示已经查询过】
        curPtr->is_diagonal=false;
        closelist.push_back(curPtr);
        // cout<<"当前遍历的curPtr->idx节点是："<<curPtr->x<<","<<curPtr->y<<","<<curPtr->idx<<endl;
        //到达终点时的判定条件:如果找到目标节点，回溯路径，否则进行下一步
        //【情况一：到达终点】
        if(curPtr->x== endPtr->x && curPtr->y == endPtr->y)
        {
            path.clear();
            // cout<<"查询到终点!!!"<<endl;
            // ROS_INFO("find the path!");
            returnPath(endPtr,path);
            // cout<<"path_size:"<<path.size()<<endl;
            for(int i=0;i<path.size();i++)
            {
                cout<<path[i]->x<<","<<path[i]->y<<endl;
            }
            // ROS_INFO("return path over!");
            while(!openlist.empty())
            {
                openlist.pop();
            }
                
            closelist.clear();
            // cout<<"~~~~~openlist-size~~~~~"<<openlist.size()<<endl;
            // cout<<"~~~~~closelist-size~~~~~"<<closelist.size()<<endl;
            return ;
        }
        //第三步，寻找跳点find_JumpPoint,并加入openlist中
        find_JumpPoint(curPtr);         
    }
}

// 如果查询到结果，将路径进行存储
void Jps::returnPath(NodePtr curPtr,vector<NodePtr> &buf)
{
    // buf.clear();
    while(curPtr->x != startPtr->x || curPtr->y != startPtr->y)
    {
        buf.push_back(curPtr);
        // cout<<"起始节点（"<<startPtr->x<<","<<startPtr->y<<startPtr->idx<<endl;//")来自("<<curPtr->camefrom->x<<","<<curPtr->camefrom->y<<endl;
        
        // cout<<"当前节点（"<<curPtr->x<<","<<curPtr->y<<curPtr->idx<<endl;//")来自("<<curPtr->camefrom->x<<","<<curPtr->camefrom->y<<endl;
        curPtr=curPtr->camefrom;
    }
    buf.push_back(startPtr);
    reverse(buf.begin(),buf.end());
}


// --------------------------------------关于jps寻路算法的函数-------------------------------//

