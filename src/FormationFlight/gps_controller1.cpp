
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <math.h>
#define longtolatscale 0.8004163362410785091197462331483
#define lattolongscale 1.2493498129938325118272112550467
#define ZeropointX 126.865995
#define ZeropointY 37.6005335

ros::Publisher arr_pub;
sensor_msgs::NavSatFix CurrentGPS;
sensor_msgs::NavSatFix DesiredGPS;
geometry_msgs::Vector3 vec;
float c=10000;
std_msgs::Bool next;

geometry_msgs::Vector3 GPStoWorldCoordinate(sensor_msgs::NavSatFix gps)
{
    geometry_msgs::Vector3 tmp;
    tmp.x = (gps.longitude-ZeropointX)*longtolatscale;
    tmp.y = gps.latitude-ZeropointY;
    tmp.x=tmp.x*c;
    tmp.y=tmp.y*c;    
    return tmp;
}

geometry_msgs::Vector3 CalDirection()
{
    geometry_msgs::Vector3 translated;
    geometry_msgs::Vector3 currentPlot = GPStoWorldCoordinate(CurrentGPS);
    geometry_msgs::Vector3 desiredPlot = GPStoWorldCoordinate(DesiredGPS);

    float Gx=desiredPlot.x;
    float Gy=desiredPlot.y;   //목표의 월드 좌표
    float Cx=currentPlot.x;
    float Cy=currentPlot.y;   //드론의 월드 좌표

    translated.x = (Gx - Cx);
    translated.y = (Gy - Cy);

    return translated;
}

void ReceiveGPS(sensor_msgs::NavSatFix vel)
{

    CurrentGPS=vel;
    vec=CalDirection();
    vec.z=DesiredGPS.altitude;
    if(DesiredGPS.latitude==37.6005007 && DesiredGPS.longitude==126.8666028)
    {
        if(vec.x < 0.05 && vec.y < 0.05 && vec.x > -0.05 && vec.y > -0.05)
        {
            //vec.x=0;
            //vec.y=0;
            vec.z=DesiredGPS.altitude;
            next.data = true;
            arr_pub.publish(next);
        }
        else
        {
            next.data = false;
            arr_pub.publish(next);
        }
    }
    else
    {
        if(vec.x < 0.05 && vec.y < 0.05 && vec.x > -0.05 && vec.y > -0.05)
        {
            next.data = true;
            arr_pub.publish(next);
        }
        else
        {
            next.data = false;
            arr_pub.publish(next);
        }
    }
}

void ReceiveMission(sensor_msgs::NavSatFix vel)
{
    DesiredGPS = vel;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_controller1");
  ros::NodeHandle n;
  DesiredGPS.latitude=37.600341;
  DesiredGPS.longitude=126.865951;
  DesiredGPS.altitude=5;
  ros::Subscriber gps_sub=n.subscribe("/mavros1/global_position/global", 1, ReceiveGPS);
  ros::Subscriber mission_sub=n.subscribe("/target1", 1, ReceiveMission);
  ros::Publisher dir_pub = n.advertise<geometry_msgs::Vector3>("/direction1",10);
  arr_pub = n.advertise<std_msgs::Bool>("/arrived1",10);
  next.data=false;
  ros::Rate loop_rate(20);
  while(ros::ok()){
      dir_pub.publish(vec);
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
