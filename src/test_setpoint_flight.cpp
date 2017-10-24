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
mavros_msgs::State current_state;

geometry_msgs::PoseStamped msg;
geometry_msgs::PoseStamped msg1;
geometry_msgs::PoseStamped msg2;


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

    msg1.header.stamp = ros::Time::now();
    msg1.header.frame_id = 1;
    msg1.pose.position.x += key.x;//0.001*some_object.position_x;
    msg1.pose.position.y += key.y;//0.001*some_object.position_y;
    msg1.pose.position.z += key.z;//0.001*some_object.position_z;
    msg1.pose.orientation.x = 0;
    msg1.pose.orientation.y = 0;
    msg1.pose.orientation.z = 0;
    msg1.pose.orientation.w = 1;

    msg2.header.stamp = ros::Time::now();
    msg2.header.frame_id = 1;
    msg2.pose.position.x += key.x;//0.001*some_object.position_x;
    msg2.pose.position.y += key.y;//0.001*some_object.position_y;
    msg2.pose.position.z += key.z;//0.001*some_object.position_z;
    msg2.pose.orientation.x = 0;
    msg2.pose.orientation.y = 0;
    msg2.pose.orientation.z = 0;
    msg2.pose.orientation.w = 1;

}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "test_setpoint_flight");
   ros::NodeHandle n;
   ros::Subscriber manual_sub=n.subscribe("/keyInput", 100, ReceiveKey);

   //UAV1
   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros1/setpoint_position/local",100);
   ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("/mavros1/state", 10, state_cb);
   ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros1/cmd/arming");
   ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros1/set_mode");
   //UAV2
   ros::Publisher chatter_pub1 = n.advertise<geometry_msgs::PoseStamped>("/mavros2/setpoint_position/local",100);
   ros::Subscriber state_sub1 = n.subscribe<mavros_msgs::State>("/mavros2/state", 10, state_cb);
   ros::ServiceClient arming_client1 = n.serviceClient<mavros_msgs::CommandBool>("/mavros2/cmd/arming");
   ros::ServiceClient set_mode_client1 = n.serviceClient<mavros_msgs::SetMode>("/mavros2/set_mode");


   //UAV3
   ros::Publisher chatter_pub2 = n.advertise<geometry_msgs::PoseStamped>("/mavros3/setpoint_position/local",100);
   ros::Subscriber state_sub2 = n.subscribe<mavros_msgs::State>("/mavros3/state", 10, state_cb);
   ros::ServiceClient arming_client2 = n.serviceClient<mavros_msgs::CommandBool>("/mavros3/cmd/arming");
   ros::ServiceClient set_mode_client2 = n.serviceClient<mavros_msgs::SetMode>("/mavros3/set_mode");

   //the setpoint publishing rate MUST be faster than 2Hz
   ros::Rate rate(20.0);

   // wait for FCU connection
   while(ros::ok() && current_state.connected){
       ros::spinOnce();
       rate.sleep();
   }


   ros::Rate loop_rate(20);



   msg.pose.position.x = 0;
   msg.pose.position.y = 0;
   msg.pose.position.z = 1;

   msg1.pose.position.x = 1;
   msg1.pose.position.y = 1;
   msg1.pose.position.z = 1;

   msg2.pose.position.x = 2;
   msg2.pose.position.y = 2;
   msg2.pose.position.z = 1;


   mavros_msgs::SetMode offb_set_mode;
   mavros_msgs::SetMode offb_set_mode1;
   mavros_msgs::SetMode offb_set_mode2;


   offb_set_mode.request.custom_mode = "OFFBOARD";
   offb_set_mode1.request.custom_mode = "OFFBOARD";
   offb_set_mode2.request.custom_mode = "OFFBOARD";

   mavros_msgs::CommandBool arm_cmd;
   mavros_msgs::CommandBool arm_cmd1;
   mavros_msgs::CommandBool arm_cmd2;

   arm_cmd.request.value = true;
   arm_cmd1.request.value = true;
   arm_cmd2.request.value = true;

   ros::Time last_request = ros::Time::now();
   ros::Time last_request1 = ros::Time::now();
   ros::Time last_request2 = ros::Time::now();

   while(ros::ok()){
       if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
       {
           if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
           {
               ROS_INFO("Offboard enabled");
           }
           last_request = ros::Time::now();
       }
       else
       {
           if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
           {
               if( arming_client.call(arm_cmd) && arm_cmd.response.success)
               {
                   ROS_INFO("Vehicle armed");
               }
               last_request = ros::Time::now();
           }
       }
       if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request1 > ros::Duration(5.0)))
       {
           if( set_mode_client1.call(offb_set_mode1) && offb_set_mode1.response.success)
           {
               ROS_INFO("Offboard1 enabled");
           }
           last_request1 = ros::Time::now();
       }
       else
       {
           if( !current_state.armed && (ros::Time::now() - last_request1 > ros::Duration(5.0)))
           {
               if( arming_client1.call(arm_cmd) && arm_cmd1.response.success)
               {
                   ROS_INFO("Vehicle1 armed");
               }
               last_request1 = ros::Time::now();
           }
       }
       if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request2 > ros::Duration(5.0)))
       {
           if( set_mode_client2.call(offb_set_mode2) && offb_set_mode2.response.success)
           {
               ROS_INFO("Offboard2 enabled");
           }
           last_request2 = ros::Time::now();
       }
       else
       {
           if( !current_state.armed && (ros::Time::now() - last_request2 > ros::Duration(5.0)))
           {
               if( arming_client2.call(arm_cmd) && arm_cmd2.response.success)
               {
                   ROS_INFO("Vehicle2 armed");
               }
               last_request2 = ros::Time::now();
           }
       }

       chatter_pub.publish(msg);
       chatter_pub1.publish(msg1);
       chatter_pub2.publish(msg2);

       ros::spinOnce();
       loop_rate.sleep();
   }
   
      
   return 0;
}

