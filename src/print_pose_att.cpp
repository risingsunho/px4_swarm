#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"

#define degreeToradian 0.01745329251994329576923690768489

geometry_msgs::Vector3 pos;
geometry_msgs::Vector3 att;
void ReceivePose(geometry_msgs::PoseStamped quat)
{

    tf::Quaternion q(quat.pose.orientation.x, quat.pose.orientation.y, quat.pose.orientation.z, quat.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pos.x=quat.pose.position.x;
    pos.y=quat.pose.position.y;
    pos.z=quat.pose.position.z;
    att.x=roll/degreeToradian;
    att.y=pitch/degreeToradian;
    att.z=yaw/degreeToradian;
    att.z=att.z+90;
    ROS_INFO("x : %f",pos.x);
    ROS_INFO("y : %f",pos.y);
    ROS_INFO("z : %f",pos.z);
    ROS_INFO("roll : %f",att.x);
    ROS_INFO("pitch : %f",att.y);
    ROS_INFO("yaw : %f",att.z);

}
int main(int argc, char **argv)
{
   ros::init(argc, argv, "print_pose_att");
   ros::NodeHandle n;

   ros::Subscriber manual_sub=n.subscribe("/mavros/local_position/pose", 100, ReceivePose);

   ros::Rate loop_rate(100);
   ros::spinOnce();


   while(ros::ok()){

       ros::spinOnce();
       loop_rate.sleep();
   }
   
      
   return 0;
}

   
