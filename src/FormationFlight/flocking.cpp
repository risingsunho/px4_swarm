#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#include "sensor_msgs/NavSatFix.h"
#include "SVector3.h"
#define longtolatscale 0.8004163362410785091197462331483
#define lattolongscale 1.2493498129938325118272112550467
#define ZeropointX 126.865995
#define ZeropointY 37.6005335
#define degreeToradian 0.01745329251994329576923690768489

using namespace std;


geometry_msgs::Vector3 U1dir;
Vector3 U2dir;
Vector3 U3dir;

sensor_msgs::NavSatFix U1GPS;
sensor_msgs::NavSatFix U2GPS;
sensor_msgs::NavSatFix U3GPS;

sensor_msgs::NavSatFix msg;
sensor_msgs::NavSatFix msg1;

Vector3 U1pose;
Vector3 U2pose;
Vector3 U3pose;


ros::Publisher mission_pub;
ros::Publisher mission_pub1;



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
    U1dir=vel;
}

Vector3 Alignment()
{
    Vector3 tmp;
    tmp.x=U1dir.x;
    tmp.y=U1dir.y;
    return tmp;
}
Vector3 Seperate(Vector3 me, Vector3 u1, Vector3 u2)
{
    Vector3 v1,v2;
    double k1,k2;
    k1=(me-u1).size()/((me-u1).size()+(me-u2).size());
    k2=(me-u2).size()/((me-u1).size()+(me-u2).size());
    v1=me-u1;
    v2=me-u2;
    v1=v1*k2;
    v2=v2*k1;
    return v1+v2;
}
Vector3 Cohension(Vector3 pos, Vector3 center)
{
    return center-pos;
}

void ForwardDirection()
{
    Vector3 flockcenter;
    Vector3 U2Ali;
    Vector3 U3Ali;
    Vector3 U2Sep;
    Vector3 U3Sep;
    Vector3 U2Coh;
    Vector3 U3Coh;

    double Ka=1;
    double Ks=1;
    double Kc=1;

    double dist2_1;
    double dist2_3;
    double dist3_1;

    flockcenter=(U1pose+U2pose+U3pose)/3;

    dist2_1=abs((U1pose-U2pose).size());
    dist2_3=abs((U2pose-U3pose).size());
    dist3_1=abs((U1pose-U3pose).size());

    U2Ali=Alignment();

    U2Sep=Seperate(U2pose,U1pose,U3pose);

    U2Coh=Cohension(U2pose,flockcenter);

    U3Ali=Alignment();

    U3Sep=Seperate(U3pose,U1pose,U2pose);

    U3Coh=Cohension(U3pose,flockcenter);

    U2Ali=U2Ali.normalize();
    U2Sep=U2Sep.normalize();
    U2Coh=U2Coh.normalize();
    U3Ali=U3Ali.normalize();
    U3Sep=U3Sep.normalize();
    U3Coh=U3Coh.normalize();


    if(dist2_3 <0.25 || dist2_1<0.25)
    {
        Ka=1;
        Ks=1;
        Kc=0;
        U2Ali=U2Ali*Ka;
        U2Sep=U2Sep*Ks;
        U2Coh=U2Coh*Kc;
        U2dir=(U2Ali+U2Sep+U2Coh);
        ROS_INFO("U2close");

    }
    else if((dist2_1>=0.25 && dist2_1 <0.6) && (dist2_3>=0.25 && dist2_3 <0.6))
    {
        Ka=1;
        Ks=1;
        Kc=1;
        U2Ali=U2Ali*Ka;
        U2Sep=U2Sep*Ks;
        U2Coh=U2Coh*Kc;
        U2dir=(U2Ali+U2Sep+U2Coh);
        ROS_INFO("U2normal");
    }
    else
    {
        Ka=1;
        Ks=0;
        Kc=1;
        U2Ali=U2Ali*Ka;
        U2Sep=U2Sep*Ks;
        U2Coh=U2Coh*Kc;
        U2dir=(U2Ali+U2Sep+U2Coh);
        ROS_INFO("U2far");
    }


    if(dist2_3<0.25 || dist3_1<0.25)
    {
        //U3dir=(U2pose-U3pose);
        //U3dir.normalize()*100;
        Ka=1;
        Ks=1;
        Kc=0;
        U3Ali=U3Ali*Ka;
        U3Sep=U3Sep*Ks;
        U3Coh=U3Coh*Kc;
        U3dir=(U3Ali+U3Sep+U3Coh);
        ROS_INFO("U3close");
    }
    else if((dist2_3>=0.25 && dist2_3 <0.6) && (dist3_1>=0.25 && dist3_1 <0.6))
    {

        Ka=1;
        Ks=1;
        Kc=1;
        U3Ali=U3Ali*Ka;
        U3Sep=U3Sep*Ks;
        U3Coh=U3Coh*Kc;
        U3dir=(U3Ali+U3Sep+U3Coh);
        ROS_INFO("U3normal");
    }
    else
    {
        Ka=1;
        Ks=0;
        Kc=1;
        U3Ali=U3Ali*Ka;
        U3Sep=U3Sep*Ks;
        U3Coh=U3Coh*Kc;
        U3dir=(U3Ali+U3Sep+U3Coh);
        ROS_INFO("U3far");
    }


    U2dir=U2dir.normalize();
    U3dir=U3dir.normalize();

   // ROS_INFO("dist2_1 : %f",dist2_1);
  //  ROS_INFO("dist2_3 : %f",dist2_3);
   // ROS_INFO("dist3_1 : %f",dist3_1);


   /* ROS_INFO("U2Ali.x : %f, U2Ali.y : %f",U2Ali.x,U2Ali.y);
    ROS_INFO("U2Sep.x : %f, U2Sep.y : %f",U2Sep.x,U2Sep.y);
    ROS_INFO("U2Coh.x : %f, U2Coh.y : %f",U2Coh.x,U2Coh.y);
    ROS_INFO("U2dir.x : %f, U2dir.y : %f",U2dir.x,U2dir.y);


    ROS_INFO("U3Ali.x : %f, U3Ali.y : %f",U3Ali.x,U3Ali.y);
    ROS_INFO("U3Sep.x : %f, U3Sep.y : %f",U3Sep.x,U3Sep.y);
    ROS_INFO("U3Coh.x : %f, U3Coh.y : %f",U3Coh.x,U3Coh.y);
    ROS_INFO("U3dir.x : %f, U3dir.y : %f",U3dir.x,U3dir.y);*/


    U2dir=U2pose+U2dir;
    U3dir=U3pose+U3dir;
    msg=WorldCoordinatetoGPS(U2dir);
    msg1=WorldCoordinatetoGPS(U3dir);
    msg.altitude=U1GPS.altitude;
    msg1.altitude=U1GPS.altitude;

    mission_pub.publish(msg);
    mission_pub1.publish(msg1);
}






int main(int argc, char **argv)
{
   ros::init(argc, argv, "flocking");
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
       ForwardDirection();
       ros::spinOnce();
       loop_rate.sleep();
   }


   return 0;
}
