#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <math.h>
#include <std_msgs/Bool.h>



geometry_msgs::Vector3 key;
geometry_msgs::PoseStamped msg;

float speed=0.02;

float initX=0;
float initY=0;
float initZ=0;
float takeoffHeight=3.5;

mavros_msgs::State current_state;
bool allready=false;
bool U1ready=false;

void receiveStart(std_msgs::Bool vel)
{
    allready=vel.data;
}
geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 vec) //정규화
{
    geometry_msgs::Vector3 tmp;
    float size=sqrt(vec.x*vec.x+vec.y*vec.y);
    tmp.x=vec.x/size;
    tmp.y=vec.y/size;
    tmp.z=vec.z;
    return tmp;
}
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void receivePose(geometry_msgs::PoseStamped vel)
{
    initX = vel.pose.position.x;
    initY = vel.pose.position.y;
    initZ = vel.pose.position.z;
    if(!U1ready)
    {
	ROS_INFO("U1 local_pose ready!");
    }
    U1ready=true;

}
void ReceiveDirection(geometry_msgs::Vector3 vel)
{
    key=vel;
    float size=sqrt(key.x*key.x+key.y*key.y);
    if(size>=0.1)
    {
        key=normalize(key);
    }

    if(allready && U1ready)
    {
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = 1;
        msg.pose.position.x += key.x*speed;
        msg.pose.position.y += key.y*speed;
        msg.pose.position.z = key.z;
        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = 0;
        msg.pose.orientation.w = 1;        
    }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "gps_setpoint_flight1");
   ros::NodeHandle n;
   ros::Subscriber manual_sub=n.subscribe("/direction1", 1, ReceiveDirection);
   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros1/setpoint_position/local",1);
   ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("/mavros1/state", 10, state_cb);
   ros::Subscriber takeoff_client = n.subscribe<geometry_msgs::PoseStamped>("/mavros1/local_position/pose",1,receivePose);
   ros::Subscriber start_client1 = n.subscribe<std_msgs::Bool>("/start",1,receiveStart);
   ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros1/cmd/arming");
   ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros1/set_mode");

   //the setpoint publishing rate MUST be faster than 2Hz
   ros::Rate rate(20.0);

   // wait for FCU connection
   while(ros::ok() && current_state.connected){
       ros::spinOnce();
       rate.sleep();
   }


   ros::Rate loop_rate(20);

   msg.pose.position.x = initX;
   msg.pose.position.y = initY;
   msg.pose.position.z = initZ+takeoffHeight;


  mavros_msgs::SetMode offb_set_mode;
   offb_set_mode.request.custom_mode = "OFFBOARD";

   mavros_msgs::CommandBool arm_cmd;
   arm_cmd.request.value = true;
   ros::Time last_request = ros::Time::now();

   while(ros::ok())
   {
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
       if(!current_state.armed && current_state.mode == "OFFBOARD" && allready)
       {
           ROS_INFO("exit");
            break;
       }
       if(U1ready)
       {
          chatter_pub.publish(msg);
       }
       ros::spinOnce();
       loop_rate.sleep();
   }


   return 0;
}
