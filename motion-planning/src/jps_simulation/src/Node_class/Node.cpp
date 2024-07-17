#include "Node_class/Node.h"

using namespace std;



Node::Node()
{
    
}

Node::Node(const double& cur_x,const double& cur_y)
{
    x=cur_x;
    y=cur_y;
    is_occupied=false;
    fScore=0;
    gScore=numeric_limits<double>::infinity();
    hScore=0;
    has_force=false;
    vertical_traversal=false;
    horizontal_traversal=false;
    idx=0;
    is_diagonal=false;
    camefrom=nullptr;
}

Node::Node(const double& cur_x,const double& cur_y,NodePtr parent)
{
    x=cur_x;
    y=cur_y;
    is_occupied=false;
    fScore=0;
    gScore=numeric_limits<double>::infinity();
    hScore=0;
    has_force=false;
    vertical_traversal=false;
    horizontal_traversal=false;
    idx=0;
    is_diagonal=false;
    camefrom=parent;
}

