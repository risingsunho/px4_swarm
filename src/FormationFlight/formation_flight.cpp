#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <math.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>

#include "sensor_msgs/NavSatFix.h"

#define longtolatscale 0.8004163362410785091197462331483
#define lattolongscale 1.2493498129938325118272112550467
#define ZeropointX 126.865995
#define ZeropointY 37.6005335
#define degreeToradian 0.01745329251994329576923690768489

using namespace std;

geometry_msgs::Vector3 U1dir;

sensor_msgs::NavSatFix U1GPS;
sensor_msgs::NavSatFix msg;
sensor_msgs::NavSatFix msg1;

geometry_msgs::Vector3 U1pose;
geometry_msgs::Vector3 U2pose;
geometry_msgs::Vector3 U3pose;

ros::Publisher mission_pub;
ros::Publisher  mission_pub1;

geometry_msgs::Vector3 VecPlus2(geometry_msgs::Vector3 A,geometry_msgs::Vector3 B)
{
    geometry_msgs::Vector3 sum;
    sum.x=A.x+B.x;
    sum.y=A.y+B.y;
    return sum;
}

geometry_msgs::Vector3 GPStoWorldCoordinate(sensor_msgs::NavSatFix gps) //gps -> 좌표
{
    geometry_msgs::Vector3 tmp;
    tmp.x=(gps.longitude-ZeropointX)*longtolatscale;
    tmp.y=gps.latitude-ZeropointY;
    tmp.x=tmp.x*10000;
    tmp.y=tmp.y*10000;
    return tmp;
}
sensor_msgs::NavSatFix WorldCoordinatetoGPS(geometry_msgs::Vector3 coor) //좌표계 -> gps
{
    sensor_msgs::NavSatFix gps;
    coor.x=coor.x*0.0001;
    coor.y=coor.y*0.0001;
    gps.longitude=(coor.x*lattolongscale)+ZeropointX;
    gps.latitude=coor.y+ZeropointY;

    return gps;
}


geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 vec) //정규화
{
    geometry_msgs::Vector3 tmp;
    float size=sqrt(vec.x*vec.x+vec.y*vec.y);
    tmp.x=vec.x/size;
    tmp.y=vec.y/size;
    return tmp;
}
void ReceiveDirection(geometry_msgs::Vector3 vel)
{
    U1dir=vel;
}
void ReceiveGPS1(sensor_msgs::NavSatFix vel)
{
    U1GPS=vel;
    U1pose=GPStoWorldCoordinate(U1GPS);
}


void ForwardDirection()
{
    geometry_msgs::Vector3 fdirection;
    geometry_msgs::Vector3 bdirection;
    geometry_msgs::Vector3 rdirection;
    geometry_msgs::Vector3 ldirection;

    float scale=0.1;

    if(U1dir.x==0 && U1dir.y==0)
    {
        U1dir.x=-0.7;
        U1dir.y=1;
    }

    fdirection=normalize(U1dir);

    rdirection.x=cos(-90*degreeToradian)*fdirection.x-sin(-90*degreeToradian)*fdirection.y;
    rdirection.y=sin(-90*degreeToradian)*fdirection.x+cos(-90*degreeToradian)*fdirection.y;

    rdirection=normalize(rdirection);


    bdirection.x=-fdirection.x;
    bdirection.y=-fdirection.y;

    ldirection.x=-rdirection.x;
    ldirection.y=-rdirection.y;
    //리더의 앞과 오른쪽 벡터 계산


    fdirection.x*=scale;
    fdirection.y*=scale;

    bdirection.x*=scale*1.5;
    bdirection.y*=scale*1.5;

    rdirection.x*=scale*2;
    rdirection.y*=scale*2;

    ldirection.x*=scale*2;
    ldirection.y*=scale*2;






    geometry_msgs::Vector3 uav2position;
    geometry_msgs::Vector3 uav3position;

    uav2position=VecPlus2(U1pose,bdirection);
    uav2position=VecPlus2(uav2position,ldirection);

   // ROS_INFO("u1position.x : %f",U1pose.x);
   // ROS_INFO("u1position.y : %f",U1pose.y);
   // ROS_INFO("uav2position.x : %f",uav2position.x);
   // ROS_INFO("uav2position.y : %f",uav2position.y);

    uav3position=VecPlus2(U1pose,bdirection);
    uav3position=VecPlus2(uav3position,rdirection);




    msg=WorldCoordinatetoGPS(uav2position);
    msg1=WorldCoordinatetoGPS(uav3position);
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


   mission_pub = n.advertise<sensor_msgs::NavSatFix>("/target2",10);
   mission_pub1 = n.advertise<sensor_msgs::NavSatFix>("/target3",10);

   ros::Subscriber manual_sub=n.subscribe("/direction1", 1, ReceiveDirection);

   ros::Rate loop_rate(30);
   ros::spinOnce();


   while(ros::ok()){
       ForwardDirection();
       ros::spinOnce();
       loop_rate.sleep();
   }


   return 0;
}
