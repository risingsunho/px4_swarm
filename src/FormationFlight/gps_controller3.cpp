
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
sensor_msgs::NavSatFix U2GPS;
sensor_msgs::NavSatFix DesiredGPS;
geometry_msgs::Vector3 vec;
geometry_msgs::Vector3 distance;
bool collision=false;
float c=10000;
int cnt=0;

geometry_msgs::Vector3 GPStoWorldCoordinate(sensor_msgs::NavSatFix gps)
{
    geometry_msgs::Vector3 tmp;
    tmp.x = (gps.longitude-ZeropointX)*longtolatscale;
    tmp.y = gps.latitude-ZeropointY;
    tmp.x=tmp.x*c;
    tmp.y=tmp.y*c;
    return tmp;
}

geometry_msgs::Vector3 CalDirection(sensor_msgs::NavSatFix cur,sensor_msgs::NavSatFix des)
{
    geometry_msgs::Vector3 translated;
    geometry_msgs::Vector3 currentPlot = GPStoWorldCoordinate(cur);
    geometry_msgs::Vector3 desiredPlot = GPStoWorldCoordinate(des);

    float Gx=desiredPlot.x;
    float Gy=desiredPlot.y;   //목표의 월드 좌표
    float Cx=currentPlot.x;
    float Cy=currentPlot.y;   //드론의 월드 좌표

    translated.x = (Gx - Cx);
    translated.y = (Gy - Cy);

    return translated;
}

void ReceiveGPS1(sensor_msgs::NavSatFix vel)
{
    CurrentGPS=vel;
    vec=CalDirection(CurrentGPS,DesiredGPS);
   // vec.z=DesiredGPS.altitude;
	if(collision)
    {
        vec=CalDirection(U2GPS,CurrentGPS);
        vec.z=DesiredGPS.altitude;
        vec.x=5*vec.x;
        vec.y=5*vec.y;
	}
}
void ReceiveGPS(sensor_msgs::NavSatFix vel)
{
	U2GPS=vel;
	distance=CalDirection(CurrentGPS,U2GPS);
	if(distance.x < 0.1 && distance.y < 0.1 && distance.x > -0.1 && distance.y > -0.1)
    {
		collision=true;
    }
	else
	{
		collision=false;
	}
	
}
void ReceiveMission(sensor_msgs::NavSatFix vel)
{
    DesiredGPS = vel;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_controller3");
  ros::NodeHandle n;
  DesiredGPS.latitude=37.600341;
  DesiredGPS.longitude=126.865951;
  DesiredGPS.altitude=5;
  ros::Subscriber gps_sub=n.subscribe("/mavros2/global_position/global", 1, ReceiveGPS);
  ros::Subscriber gps_sub1=n.subscribe("/mavros3/global_position/global", 1, ReceiveGPS1);
  ros::Subscriber mission_sub=n.subscribe("/target3", 1, ReceiveMission);
  ros::Publisher dir_pub = n.advertise<geometry_msgs::Vector3>("/direction3",10);

  ros::Rate loop_rate(20);
  while(ros::ok()){
      dir_pub.publish(vec);
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
