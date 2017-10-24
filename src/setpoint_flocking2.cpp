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
#include "math.h"
#define longtolatscale 0.8004163362410785091197462331483
#define lattolongscale 1.2493498129938325118272112550467
#define ZeropointX 126.865995
#define ZeropointY 37.6005335


mavros_msgs::State current_state;
geometry_msgs::PoseStamped msg;
geometry_msgs::Vector3 U1pose;
geometry_msgs::Vector3 U2pose;
geometry_msgs::Vector3 U3pose;



geometry_msgs::Vector3 Alignment;
geometry_msgs::Vector3 Coh;
geometry_msgs::Vector3 Sep;

geometry_msgs::Vector3 direction;

ros::Publisher chatter_pub;
float speed=1;
float height=0;

geometry_msgs::Vector3 GPStoWorldCoordinate(sensor_msgs::NavSatFix gps)
{
    geometry_msgs::Vector3 tmp;
    tmp.x=(gps.longitude-ZeropointX)*longtolatscale;
    tmp.y=gps.latitude-ZeropointY;

    tmp.x=tmp.x*8000;
    tmp.y=tmp.y*8000;
    return tmp;
}
void receivePose(geometry_msgs::PoseStamped vel){
    height = vel.pose.position.z;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 vec)
{
    geometry_msgs::Vector3 tmp;
    float size=sqrt(vec.x*vec.x+vec.y*vec.y);
    tmp.x=vec.x/size;
    tmp.y=vec.y/size;

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
geometry_msgs::Vector3 VecPlus2(geometry_msgs::Vector3 A,geometry_msgs::Vector3 B)
{
    geometry_msgs::Vector3 sum;
    sum.x=A.x+B.x;
    sum.y=A.y+B.y;
    return sum;
}
geometry_msgs::Vector3 VecPlus3(geometry_msgs::Vector3 A,geometry_msgs::Vector3 B,geometry_msgs::Vector3 C)
{
    geometry_msgs::Vector3 sum;
    sum.x=A.x+B.x+C.x;
    sum.y=A.y+B.y+C.y;
    return sum;
}

void ReceiveGPS1(sensor_msgs::NavSatFix vel)
{
    U1pose=GPStoWorldCoordinate(vel);
}
void ReceiveGPS2(sensor_msgs::NavSatFix vel)
{
    U2pose=GPStoWorldCoordinate(vel);
}
void ReceiveGPS3(sensor_msgs::NavSatFix vel)
{
    U3pose=GPStoWorldCoordinate(vel);
    geometry_msgs::Vector3 U3desire;
    geometry_msgs::Vector3 tmpdir;


    Alignment=normalize(Alignment);

    Coh.x=VecPlus3(U1pose,U2pose,U3pose).x/3;
    Coh.y=VecPlus3(U1pose,U2pose,U3pose).y/3;
    Coh=CalDirection(U3pose,Coh);
    Coh=normalize(Coh);
    //Coh.x=(U1pose.x+U2pose.x+U3pose.x)/3;
    //Coh.y=(U1pose.y+U2pose.y+U3pose.y)/3;
    Sep=VecPlus2(CalDirection(U2pose,U3pose),CalDirection(U1pose,U3pose));
    Sep=normalize(Sep);
    //Sep.x=CalDirection(U2pose,U3pose).x+CalDirection(U1pose,U3pose).x;
    //Sep.y=CalDirection(U2pose,U3pose).y+CalDirection(U1pose,U3pose).y;




    tmpdir=VecPlus3(Alignment,Coh,Sep);

    msg.pose.position.x+=tmpdir.x/10;
    msg.pose.position.y+=tmpdir.y/10;
    msg.pose.position.z = 5;
    chatter_pub.publish(msg);

}


void ReceiveDirection(geometry_msgs::Vector3 vel)
{
    Alignment=vel;
}



int main(int argc, char **argv)
{
   ros::init(argc, argv, "setpoint_flocking2");
   ros::NodeHandle n;

   //UAV1
   ros::Subscriber manual_sub=n.subscribe("/direction", 1, ReceiveDirection);

   chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros3/setpoint_position/local",100);
   ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("/mavros3/state", 10, state_cb);


   ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros3/cmd/arming");
   ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros3/set_mode");


   ros::Subscriber gps_sub1=n.subscribe("/mavros1/global_position/global", 1, ReceiveGPS1);
   ros::Subscriber gps_sub2=n.subscribe("/mavros2/global_position/global", 1, ReceiveGPS2);
   ros::Subscriber gps_sub3=n.subscribe("/mavros3/global_position/global", 1, ReceiveGPS3);

   ros::Subscriber takeoff_client = n.subscribe<geometry_msgs::PoseStamped>("/mavros3/local_position/pose",1,receivePose);


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
   msg.pose.position.z = 5;



   mavros_msgs::SetMode offb_set_mode;
   offb_set_mode.request.custom_mode = "OFFBOARD";
   mavros_msgs::CommandBool arm_cmd;
   arm_cmd.request.value = true;
   ros::Time last_request = ros::Time::now();


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
       ros::spinOnce();
       loop_rate.sleep();
   }
   
      
   return 0;
}


