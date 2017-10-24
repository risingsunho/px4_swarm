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

geometry_msgs::Vector3 att;
geometry_msgs::Vector3 rpy;

geometry_msgs::PoseStamped msg;
geometry_msgs::PoseStamped msg1;

geometry_msgs::Vector3 U1pose;
geometry_msgs::Vector3 U2pose;
geometry_msgs::Vector3 U3pose;

ros::Publisher chatter_pub;
ros::Publisher chatter_pub1;

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
    coor.x=coor.x/10000;
    coor.y=coor.y/10000;
    gps.latitude=(coor.x+ZeropointX)*lattolongscale;
    gps.longitude=coor.y+ZeropointY;

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

void ReceivePose(geometry_msgs::PoseStamped quat) //yaw 값 구하기
{
    tf::Quaternion q(quat.pose.orientation.x, quat.pose.orientation.y, quat.pose.orientation.z, quat.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    att.x=roll/degreeToradian;
    att.y=pitch/degreeToradian;
    att.z=yaw/degreeToradian;
    att.z=att.z+90;
    if(att.z>180)
    {
        att.z=-90-(90-(att.z-180));
    }
    rpy.x=att.x;
    rpy.y=att.y;
    rpy.z=att.z;
}

void ForwardDirection()
{
    geometry_msgs::Vector3 fdirection;
    geometry_msgs::Vector3 bdirection;
    geometry_msgs::Vector3 rdirection;
    geometry_msgs::Vector3 ldirection;

    double x,y;

    x=-sin(att.z*degreeToradian);
    y=cos(att.z*degreeToradian);

    fdirection.x=x/sqrt(x*x+y*y);
    fdirection.y=y/sqrt(x*x+y*y);

    rdirection.x=cos(-90*degreeToradian)*fdirection.x-sin(-90*degreeToradian)*fdirection.y;
    rdirection.y=sin(-90*degreeToradian)*fdirection.x+cos(-90*degreeToradian)*fdirection.y;

    rdirection=normalize(rdirection);


    bdirection.x=-fdirection.x;
    bdirection.y=-fdirection.y;

    ldirection.x=-rdirection.x;
    ldirection.y=-rdirection.y;
    //리더의 앞과 오른쪽 벡터 계산

    geometry_msgs::Vector3 uav2position;
    geometry_msgs::Vector3 uav3position;

    uav2position=VecPlus2(U1pose,bdirection);
    uav2position=VecPlus2(uav2position,ldirection);

    uav3position=VecPlus2(U1pose,bdirection);
    uav3position=VecPlus2(uav3position,rdirection);


    msg.pose.position.x=uav2position.x;
    msg.pose.position.y=uav2position.y;
    msg.pose.position.z=5;

    msg1.pose.position.x=uav3position.x;
    msg1.pose.position.y=uav3position.y;
    msg1.pose.position.y=5;

    chatter_pub.publish(msg);
    chatter_pub.publish(msg1);
}


void ReceiveGPS1(sensor_msgs::NavSatFix vel)
{
    U1pose=GPStoWorldCoordinate(vel);
}



int main(int argc, char **argv)
{
   ros::init(argc, argv, "formation_flight");
   ros::NodeHandle n;
   ros::Subscriber pose_sub=n.subscribe("/mavros1/local_position/pose", 10, ReceivePose);
   ros::Subscriber gps_sub1=n.subscribe("/mavros1/global_position/global", 1, ReceiveGPS1);


   //mission_pub = n.advertise<sensor_msgs::NavSatFix>("/target2",10);
   //mission_pub2 = n.advertise<sensor_msgs::NavSatFix>("/target3",10);

   chatter_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros2/setpoint_position/local",100);
   chatter_pub1 = n.advertise<geometry_msgs::PoseStamped>("/mavros3/setpoint_position/local",100);

   ros::Rate loop_rate(30);
   ros::spinOnce();


   while(ros::ok()){
       ForwardDirection();
       ros::spinOnce();
       loop_rate.sleep();
   }


   return 0;
}
