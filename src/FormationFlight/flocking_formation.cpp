#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>
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

Vector3 U1pose;
Vector3 U2pose;
Vector3 U3pose;

std_msgs::Int32 U2state;
std_msgs::Int32 U3state;


ros::Publisher mission_pub;
ros::Publisher mission_pub1;

ros::Publisher state_pub;
ros::Publisher state_pub1;



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

}
void Flocking()
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
        U2state.data=0;
        Ka=1;
        Ks=1;
        Kc=0;
        U2Ali=U2Ali*Ka;
        U2Sep=U2Sep*Ks;
        U2Coh=U2Coh*Kc;
        U2dir=(U2Ali+U2Sep+U2Coh);
        U2dir=U2dir.normalize();
        U2dir=U2pose+U2dir;
        //ROS_INFO("U2close");

    }
    else if((dist2_1>=0.25 && dist2_1 <0.5) && (dist2_3>=0.25 && dist2_3 <0.5))
    {
        U2state.data=1;
        Formation_Flight();
        /*Ka=1;
        Ks=1;
        Kc=1;
        U2Ali=U2Ali*Ka;
        U2Sep=U2Sep*Ks;
        U2Coh=U2Coh*Kc;
        U2dir=(U2Ali+U2Sep+U2Coh);*/
        //ROS_INFO("U2normal");
    }
    else
    {
        Formation_Flight();
        U2state.data=2;
       /* Ka=1;
        Ks=0;
        Kc=1;
        U2Ali=U2Ali*Ka;
        U2Sep=U2Sep*Ks;
        U2Coh=U2Coh*Kc;
        U2dir=(U2Ali+U2Sep+U2Coh);
        //ROS_INFO("U2far");*/
    }


    if(dist2_3<0.25 || dist3_1<0.25)
    {
        U3state.data=0;
        Ka=1;
        Ks=1;
        Kc=0;
        U3Ali=U3Ali*Ka;
        U3Sep=U3Sep*Ks;
        U3Coh=U3Coh*Kc;
        U3dir=(U3Ali+U3Sep+U3Coh);
        U3dir=U3dir.normalize();
        U3dir=U3pose+U3dir;
        //ROS_INFO("U3close");
    }
    else if((dist2_3>=0.25 && dist2_3 <0.5) && (dist3_1>=0.25 && dist3_1 <0.5))
    {
        U3state.data=1;
        Formation_Flight();
        /*Ka=1;
        Ks=1;
        Kc=1;
        U3Ali=U3Ali*Ka;
        U3Sep=U3Sep*Ks;
        U3Coh=U3Coh*Kc;
        U3dir=(U3Ali+U3Sep+U3Coh);
        //ROS_INFO("U3normal");*/
    }
    else
    {
        U3state.data=2;
        Formation_Flight();
        /*Ka=1;
        Ks=0;
        Kc=1;
        U3Ali=U3Ali*Ka;
        U3Sep=U3Sep*Ks;
        U3Coh=U3Coh*Kc;
        U3dir=(U3Ali+U3Sep+U3Coh);
        //ROS_INFO("U3far");*/
    }


    msg=WorldCoordinatetoGPS(U2dir);
    msg1=WorldCoordinatetoGPS(U3dir);

    msg.altitude=U1GPS.altitude;
    msg1.altitude=U1GPS.altitude;

    mission_pub.publish(msg);
    mission_pub1.publish(msg1);

    state_pub.publish(U2state);
    state_pub1.publish(U3state);
}






int main(int argc, char **argv)
{
   ros::init(argc, argv, "flocking_formation");
   ros::NodeHandle n;
   ros::Subscriber gps_sub1=n.subscribe("/mavros1/global_position/global", 1, ReceiveGPS1);
   ros::Subscriber gps_sub2=n.subscribe("/mavros2/global_position/global", 1, ReceiveGPS2);
   ros::Subscriber gps_sub3=n.subscribe("/mavros3/global_position/global", 1, ReceiveGPS3);


   mission_pub = n.advertise<sensor_msgs::NavSatFix>("/target2",10);
   mission_pub1 = n.advertise<sensor_msgs::NavSatFix>("/target3",10);

   state_pub = n.advertise<std_msgs::Int32>("/state2",10);
   state_pub1 = n.advertise<std_msgs::Int32>("/state3",10);

   ros::Subscriber manual_sub=n.subscribe("/direction1", 1, ReceiveDirection);

   ros::Rate loop_rate(30);
   ros::spinOnce();


   while(ros::ok()){
       Flocking();
       ros::spinOnce();
       loop_rate.sleep();
   }


   return 0;
}
