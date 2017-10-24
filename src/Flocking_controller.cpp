#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include "sensor_msgs/NavSatFix.h"

#define longtolatscale 0.8004163362410785091197462331483
#define lattolongscale 1.2493498129938325118272112550467
#define ZeropointX 126.865995
#define ZeropointY 37.6005335


geometry_msgs::Vector3 key;
mavros_msgs::State current_state;

geometry_msgs::PoseStamped msg;
geometry_msgs::PoseStamped msg1;
geometry_msgs::PoseStamped msg2;

geometry_msgs::Vector3 U1pose;
geometry_msgs::Vector3 U2pose;
geometry_msgs::Vector3 U3pose;
ros::Publisher chatter_pub;
ros::Publisher chatter_pub1;
ros::Publisher chatter_pub2;
float height1=0;
float height=0;
float speed=1;

geometry_msgs::Vector3 GPStoWorldCoordinate(sensor_msgs::NavSatFix gps)
{
    geometry_msgs::Vector3 tmp;
    tmp.x=(gps.longitude-ZeropointX)*longtolatscale;
    tmp.y=gps.latitude-ZeropointY;
    tmp.x=tmp.x*8000;
    tmp.y=tmp.y*8000;
    return tmp;
}
geometry_msgs::Vector3 CalDirection(geometry_msgs::Vector3 origin,geometry_msgs::Vector3 desired)
{
    geometry_msgs::Vector3 translated;

    //float Gx=(desired.x+origin.x)/2;
    //float Gy=(desired.y+origin.y)/2;
    float Gx=desired.x;
    float Gy=desired.y;   //목표의 월드 좌표
    float Cx=origin.x;
    float Cy=origin.y;   //드론의 월드 좌표



    translated.x = (Gx - Cx)*speed;
    translated.y = (Gy - Cy)*speed;

    return translated;
}
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void ReceiveGPS1(sensor_msgs::NavSatFix vel)
{
    U1pose=GPStoWorldCoordinate(vel);
}
void ReceiveGPS2(sensor_msgs::NavSatFix vel)
{
    U2pose=GPStoWorldCoordinate(vel);
    geometry_msgs::Vector3 U2desire;
    geometry_msgs::Vector3 tmpdir;

    U2desire.x=U1pose.x-0.1;
    U2desire.y=U1pose.y-0.1;
    tmpdir=CalDirection(U2pose,U2desire);
    //ROS_INFO("UAV2 :(%f,%f)",tmpdir.x,tmpdir.y);
    if(height>4.5)
    {
        msg1.pose.position.x+=(tmpdir.x)/10;
        msg1.pose.position.y+=(tmpdir.y)/10;
        msg1.pose.position.z = 5;
        chatter_pub1.publish(msg1);
    }
}

void ReceiveGPS3(sensor_msgs::NavSatFix vel)
{
    U3pose=GPStoWorldCoordinate(vel);
    geometry_msgs::Vector3 U3desire;
    geometry_msgs::Vector3 tmpdir;

    U3desire.x=U1pose.x-0.2;
    U3desire.y=U1pose.y-0.2;
    tmpdir=CalDirection(U3pose,U3desire);
    //ROS_INFO("UAV3 :(%f,%f)",tmpdir.x,tmpdir.y);
    if(height1>4.5)
    {
        msg2.pose.position.x+=tmpdir.x/10;
        msg2.pose.position.y+=tmpdir.y/10;
        msg2.pose.position.z = 5;
        chatter_pub2.publish(msg2);
    }
}
void receivePose(geometry_msgs::PoseStamped vel){
    height = vel.pose.position.z;
}
void receivePose1(geometry_msgs::PoseStamped vel){
    height1 = vel.pose.position.z;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "Flocking_controller");
   ros::NodeHandle n;

   //UAV1
   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros1/mavros/setpoint_position/local",100);
   //UAV2
   ros::Publisher chatter_pub1 = n.advertise<geometry_msgs::PoseStamped>("/mavros2/mavros/setpoint_position/local",100);
   //UAV3
   ros::Publisher chatter_pub2 = n.advertise<geometry_msgs::PoseStamped>("/mavros3/mavros/setpoint_position/local",100);


   ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("/mavros1/mavros/state", 10, state_cb);
   ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros1/mavros/cmd/arming");
   ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros1/mavros/set_mode");
   ros::Subscriber state_sub1 = n.subscribe<mavros_msgs::State>("/mavros2/mavros/state", 10, state_cb);
   ros::ServiceClient arming_client1 = n.serviceClient<mavros_msgs::CommandBool>("/mavros2/mavros/cmd/arming");
   ros::ServiceClient set_mode_client1 = n.serviceClient<mavros_msgs::SetMode>("/mavros2/mavros/set_mode");


   ros::Subscriber state_sub2 = n.subscribe<mavros_msgs::State>("/mavros3/mavros/state", 10, state_cb);
   ros::ServiceClient arming_client2 = n.serviceClient<mavros_msgs::CommandBool>("/mavros3/mavros/cmd/arming");
   ros::ServiceClient set_mode_client2 = n.serviceClient<mavros_msgs::SetMode>("/mavros3/mavros/set_mode");

   ros::Subscriber gps_sub1=n.subscribe("/mavros1/mavros/global_position/global", 1, ReceiveGPS1);
   ros::Subscriber gps_sub2=n.subscribe("/mavros2/mavros/global_position/global", 1, ReceiveGPS2);
   ros::Subscriber gps_sub3=n.subscribe("/mavros3/mavros/global_position/global", 1, ReceiveGPS3);

   ros::Subscriber takeoff_client = n.subscribe<geometry_msgs::PoseStamped>("/mavros2/mavros/local_position/pose",1,receivePose);
   ros::Subscriber takeoff_client1 = n.subscribe<geometry_msgs::PoseStamped>("/mavros3/mavros/local_position/pose",1,receivePose1);


   //the setpoint publishing rate MUST be faster than 2Hz
   ros::Rate rate(20.0);

   // wait for FCU connection
   while(ros::ok() && current_state.connected){
       ros::spinOnce();
       rate.sleep();
   }


   ros::Rate loop_rate(20);


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

       ros::spinOnce();
       loop_rate.sleep();
   }


   return 0;
}

