#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <vector>
#include <limits>
#include <queue>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using namespace std;
enum class Direction{
    left,
    right,
    up,
    down,
    up_left,
    up_right,
    down_left,
    down_right,
    none_
};

class Node;
// typedef Node* NodePtr;
typedef std::shared_ptr<Node> NodePtr;
extern vector<vector<Node>> map_;

class Node
{
public:
    int x,y;//对应2d坐标
    double fScore = INFINITY;
    double gScore = INFINITY;
    double hScore = 0;
    bool is_occupied = false;//判断当前节点是否被占据
    bool jump_visited;//判断当前点是不是跳点
    int idx = 0;
   
    bool has_force,horizontal_traversal,vertical_traversal;//如果has_force=true表示当前节点存在跳点
    NodePtr camefrom = nullptr;//定义邻居节点的父节点
    NodePtr jumpPoint;//定义节点的子节点
    
    vector<NodePtr> neighbors;//表示存储指向相邻节点的指针容器
    vector<NodePtr> force_nebor;
    queue<Direction> dirList;
    bool is_diagonal;
    //numeric_limits返回double类型的正无穷大值。eg：起始节点gScore被初始化为0，而其他节点gScore被初始化为正无穷大
    
    Node(double x_,double y_,bool is_occupied_):x(x_),y(y_),is_occupied(is_occupied_),fScore(0),gScore(numeric_limits<double>::infinity()),hScore(0),camefrom(nullptr){}
    Node();
    Node(const double& cur_x,const double& cur_y);
    Node(const double& cur_x,const double& cur_y,NodePtr parent);
    // double calc_hScore(const NodePtr curPtr ,const NodePtr targetPtr);
};

struct compute_Priority //进行运算符的重载，重载优先队列的比较运算符，使之按照fScore的数值进行排序
{
    bool operator()(const NodePtr node1,const NodePtr node2) const
    {
        return node1->fScore > node2->fScore;//TODO：一会打印出来看一下，最小堆，按照fScore从小到大排序
    }
};

#endif