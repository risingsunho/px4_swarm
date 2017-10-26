#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <math.h>
#include <tf/transform_datatypes.h>
#include "sensor_msgs/NavSatFix.h"
#include "SVector3.h"


#define longtolatscale 0.8004163362410785091197462331483
#define lattolongscale 1.2493498129938325118272112550467
#define ZeropointX 126.865995
#define ZeropointY 37.6005335
#define degreeToradian 0.01745329251994329576923690768489

using namespace std;

Vector3 U1dir;
Vector3 U2dir;
Vector3 U3dir;

sensor_msgs::NavSatFix U1GPS;
sensor_msgs::NavSatFix U2GPS;
sensor_msgs::NavSatFix U3GPS;


sensor_msgs::NavSatFix msg;
sensor_msgs::NavSatFix msg1;

ros::Publisher mission_pub;
ros::Publisher mission_pub1;

Vector3 U1pose;
Vector3 U2pose;
Vector3 U3pose;



Vector3 GPStoWorldCoordinate(sensor_msgs::NavSatFix gps) //gps -> 좌표
{
    double x,y,z;
    x=(gps.longitude-ZeropointX)*longtolatscale;
    y=gps.latitude-ZeropointY;
    x=x*10000;
    y=y*10000;
    Vector3 tmp(x,y,z);
    return tmp;
}
sensor_msgs::NavSatFix WorldCoordinatetoGPS(Vector3 coor) //좌표계 -> gps
{
    sensor_msgs::NavSatFix gps;
    coor.x=coor.x*0.0001;
    coor.y=coor.y*0.0001;
    gps.longitude=(coor.x*lattolongscale)+ZeropointX;
    gps.latitude=coor.y+ZeropointY;

    return gps;
}

void ReceiveGPS1(sensor_msgs::NavSatFix vel)
{
    U1GPS=vel;
    U1pose=GPStoWorldCoordinate(U1GPS);
}
void ReceiveGPS2(sensor_msgs::NavSatFix vel)
{
    U2GPS=vel;
    U2pose=GPStoWorldCoordinate(U2GPS);
}
void ReceiveGPS3(sensor_msgs::NavSatFix vel)
{
    U3GPS=vel;
    U3pose=GPStoWorldCoordinate(U3GPS);
}
void ReceiveDirection(geometry_msgs::Vector3 vel)
{
    U1dir.x=vel.x;
    U1dir.y=vel.y;
    U1dir.z=vel.z;
}

void Formation_Flight()
{
    Vector3 fdirection;
    Vector3 bdirection;
    Vector3 rdirection;
    Vector3 ldirection;

    float scale=0.1;

    fdirection=U1dir.normalize();

    rdirection.x=cos(-90*degreeToradian)*fdirection.x-sin(-90*degreeToradian)*fdirection.y;
    rdirection.y=sin(-90*degreeToradian)*fdirection.x+cos(-90*degreeToradian)*fdirection.y;


    rdirection=rdirection.normalize();


    bdirection=fdirection*-1;


    ldirection=rdirection*-1;

    //리더의 앞과 오른쪽 벡터 계산


    fdirection=fdirection*scale;

    bdirection=bdirection*scale*1.5;

    rdirection=rdirection*scale*2;

    ldirection=ldirection*scale*2;



    U2dir=U1pose+bdirection+ldirection;
    U3dir=U1pose+bdirection+rdirection;



   // ROS_INFO("u1position.x : %f",U1pose.x);
   // ROS_INFO("u1position.y : %f",U1pose.y);
   // ROS_INFO("uav2position.x : %f",uav2position.x);
   // ROS_INFO("uav2position.y : %f",uav2position.y);


    msg=WorldCoordinatetoGPS(U2dir);
    msg1=WorldCoordinatetoGPS(U3dir);

    msg.altitude=U1GPS.altitude;
    msg1.altitude=U1GPS.altitude;

    mission_pub.publish(msg);
    mission_pub1.publish(msg1);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "formation_flight");
   ros::NodeHandle n;
   ros::Subscriber gps_sub1=n.subscribe("/mavros1/global_position/global", 1, ReceiveGPS1);
   ros::Subscriber gps_sub2=n.subscribe("/mavros2/global_position/global", 1, ReceiveGPS2);
   ros::Subscriber gps_sub3=n.subscribe("/mavros3/global_position/global", 1, ReceiveGPS3);


   mission_pub = n.advertise<sensor_msgs::NavSatFix>("/target2",10);
   mission_pub1 = n.advertise<sensor_msgs::NavSatFix>("/target3",10);

   ros::Subscriber manual_sub=n.subscribe("/direction1", 1, ReceiveDirection);

   ros::Rate loop_rate(30);
   ros::spinOnce();


   while(ros::ok()){
       Formation_Flight();
       ros::spinOnce();
       loop_rate.sleep();
   }


   return 0;
}
