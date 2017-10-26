#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/ManualControl.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"


#define degreeToradian 0.01745329251994329576923690768489

using namespace std;

geometry_msgs::TwistStamped msg;
//geometry_msgs::PoseStamped msg;

geometry_msgs::Vector3 pos;
geometry_msgs::Vector3 att;
geometry_msgs::Vector3 rpy;
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
    if(att.z>180)
    {
        att.z=-90-(90-(att.z-180));
    }
    rpy.x=att.x;
    rpy.y=att.y;
    rpy.z=att.z;
}
void ReceiveManual(mavros_msgs::ManualControl vel)
{
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = 1;
    geometry_msgs::Vector3 fdirection;
    geometry_msgs::Vector3 rdirection;
    double x,y;
    x=-sin(att.z*degreeToradian);
    y=cos(att.z*degreeToradian);

    fdirection.x=x/sqrt(x*x+y*y);
    fdirection.y=y/sqrt(x*x+y*y);

    rdirection.x=cos(-90*degreeToradian)*fdirection.x-sin(-90*degreeToradian)*fdirection.y;
    rdirection.y=sin(-90*degreeToradian)*fdirection.x+cos(-90*degreeToradian)*fdirection.y;

    rdirection.x=rdirection.x/sqrt(rdirection.x*rdirection.x+rdirection.y*rdirection.y);
    rdirection.y=rdirection.y/sqrt(rdirection.x*rdirection.x+rdirection.y*rdirection.y);


    msg.twist.linear.x = fdirection.x*vel.x+rdirection.x*vel.y;
    msg.twist.linear.y = fdirection.y*vel.x+rdirection.y*vel.y;
    msg.twist.linear.z = (vel.z-0.5)*2;
    msg.twist.angular.z=-vel.r;


    /*msg.twist.linear.x=msg.twist.linear.x*3;
    msg.twist.linear.y=msg.twist.linear.y*3;
    msg.twist.linear.z=msg.twist.linear.z*3;
    msg.twist.angular.z=msg.twist.angular.z*-2.5;*/


}
int main(int argc, char **argv)
{
   ros::init(argc, argv, "pub_setpoints");
   ros::NodeHandle n;
   ros::Publisher pos_pub = n.advertise<geometry_msgs::Vector3>("/pos",100);
   ros::Publisher rpy_pub = n.advertise<geometry_msgs::Vector3>("/RollPitchYaw",100);
   ros::Publisher chatter_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",100);
   ros::Subscriber manual_sub=n.subscribe("/mavros1/manual_control/control", 100, ReceiveManual);
   ros::Subscriber pose_sub=n.subscribe("/mavros/local_position/pose", 10, ReceivePose);

   ros::Rate loop_rate(30);
   ros::spinOnce();


   while(ros::ok()){
      // pub_att.publish(cmd_att);
       chatter_pub.publish(msg);
       rpy_pub.publish(rpy);
       pos_pub.publish(pos);
       ros::spinOnce();
       loop_rate.sleep();
   }
   
      
   return 0;
}
