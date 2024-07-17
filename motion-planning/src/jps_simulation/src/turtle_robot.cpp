#include "ros/ros.h"
#include "turtlesim/Pose.h"

//引入tf坐标变换
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
// 引入geometry库
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub_msgs;
using namespace std;
void sub_callback(const turtlesim::Pose::ConstPtr &turtle_msg)
{
    cout<<"111111111111111"<<endl;
    // geometry_msgs::TransformStamped transform_stamped;
    // transform_stamped.header.stamp=ros::Time::now();
    // transform_stamped.header.frame_id="map";
    // transform_stamped.child_frame_id="odom";

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(turtle_msg->x,turtle_msg->y,0));

    tf::Quaternion qtn;
    qtn.setRPY(0,0,turtle_msg->theta);
    transform.setRotation(qtn);

    tf::StampedTransform transform_stamped(transform,ros::Time::now(),"map","odom");

    tf::TransformBroadcaster tf_broadcaster;
    tf_broadcaster.sendTransform(transform_stamped);
    // pub_msgs.publish(transform_stamped);
}
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"turtle_robot_sub");
    ros::NodeHandle nh("~");
    ros::Subscriber sub_msgs=nh.subscribe<turtlesim::Pose>("/turtle1/pose",10,sub_callback);
    // pub_msgs=nh.advertise<geometry_msgs::Vector3>("/geometry_pub",10);
    ros::spin();
    return 0;
}