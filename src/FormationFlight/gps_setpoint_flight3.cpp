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
#include <std_msgs/Int32.h>

geometry_msgs::Vector3 key;
geometry_msgs::PoseStamped msg;   

float speed=0.02;

float initX=0;
float initY=0;
float initZ=0;
float takeoffheight=3.5;
mavros_msgs::State current_state;

std_msgs::Int32 state;

bool allready=false;
bool U3ready=false;

void ReceiveState(std_msgs::Int32 vel)
{
    state=vel;
}

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
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void receivePose(geometry_msgs::PoseStamped vel){
    initX=vel.pose.position.x;
    initY=vel.pose.position.y;
    initZ=vel.pose.position.z;
    if(!U3ready)
    {
	ROS_INFO("U3 local_pose ready!");
    }
    U3ready=true;
}

void ReceiveDirection(geometry_msgs::Vector3 vel)
{
    key=vel;    
    if(state.data==2)
    {
        key=normalize(key);
        speed=0.03;
    }
    else if(state.data==0 || state.data==1)
    {
        key=normalize(key);
        speed=0.02;
    }
    if(allready&&U3ready)
    {
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = 1;
        msg.pose.position.x += key.x*speed;//0.001*some_object.position_x;
        msg.pose.position.y += key.y*speed;//0.001*some_object.position_y;
        msg.pose.position.z = key.z;//0.001*some_object.position_z;
        msg.pose.orientation.x = 0;
        msg.pose.orientation.y = 0;
        msg.pose.orientation.z = 0;
        msg.pose.orientation.w = 1;
    }
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "gps_setpoint_flight3");
   ros::NodeHandle n;

   ros::Subscriber flock_state_sub=n.subscribe("/state3", 1, ReceiveState);

   ros::Subscriber manual_sub=n.subscribe("/direction3", 1, ReceiveDirection);
   ros::Publisher chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros3/setpoint_position/local",1);
   ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>("/mavros3/state", 10, state_cb);
   ros::Subscriber takeoff_client = n.subscribe<geometry_msgs::PoseStamped>("/mavros3/local_position/pose",1,receivePose);

   ros::Subscriber start_client3 = n.subscribe<std_msgs::Bool>("/start",1,receiveStart);

   ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>("/mavros3/cmd/arming");
   ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>("/mavros3/set_mode");

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
   msg.pose.position.z = initZ+takeoffheight;


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
           if( !current_state.armed &&
               (ros::Time::now() - last_request > ros::Duration(5.0))){
               if( arming_client.call(arm_cmd) &&
                   arm_cmd.response.success){
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

       if(U3ready)
       {
           chatter_pub.publish(msg);
       }
       ros::spinOnce();
       loop_rate.sleep();
   }


   return 0;
}
