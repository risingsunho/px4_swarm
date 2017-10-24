#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>

geometry_msgs::Vector3 key;
geometry_msgs::PoseStamped msg;
mavros_msgs::State current_state;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void ReceiveKey(geometry_msgs::Vector3 vel)
{
    key=vel;        
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = 1;
    msg.pose.position.x += key.x;//0.001*some_object.position_x;
    msg.pose.position.y += key.y;//0.001*some_object.position_y;
    msg.pose.position.z += key.z;//0.001*some_object.position_z;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;

}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "setpoint_flight");
   ros::NodeHandle n;
   ros::Subscriber manual_sub=n.subscribe("/keyInput", 100, ReceiveKey);
   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
   ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
   ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
   ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

   //the setpoint publishing rate MUST be faster than 2Hz
   ros::Rate rate(20.0);

   // wait for FCU connection
   while(ros::ok() && current_state.connected){
       ros::spinOnce();
       rate.sleep();
   }


   ros::Rate loop_rate(20);

 /*  for(int i = 100; ros::ok() && i > 0; --i){
       chatter_pub.publish(msg);
       ros::spinOnce();
       rate.sleep();
   }*/

   msg.pose.position.x = 0;
   msg.pose.position.y = 0;
   msg.pose.position.z = 1;


   mavros_msgs::SetMode offb_set_mode;
   offb_set_mode.request.custom_mode = "OFFBOARD";

   mavros_msgs::CommandBool arm_cmd;
   arm_cmd.request.value = true;
   ros::Time last_request = ros::Time::now();

   while(ros::ok()){
       if( current_state.mode != "OFFBOARD" &&
           (ros::Time::now() - last_request > ros::Duration(5.0))){
           if( set_mode_client.call(offb_set_mode) &&
               offb_set_mode.response.success){
               ROS_INFO("Offboard enabled");
           }
           last_request = ros::Time::now();
       } else {
           /*if( !current_state.armed &&
               (ros::Time::now() - last_request > ros::Duration(5.0))){
               if( arming_client.call(arm_cmd) &&
                   arm_cmd.response.success){
                   ROS_INFO("Vehicle armed");
               }
               last_request = ros::Time::now();
           }*/
       }

       chatter_pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();
   }
   
      
   return 0;
}
