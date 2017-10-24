#include <ros/ros.h>
#include <math.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"

#define PI 3.14159265358979323846264338
#define degreeToradian 0.01745329251994329576923690768489

#define ZeropointX 126.865995
#define ZeropointY 37.6005335


#define longtolatscale 0.8004163362410785091197462331483
#define lattolongscale 1.2493498129938325118272112550467

using namespace std;

ros::Publisher move_pub;

geometry_msgs::Quaternion quat;

sensor_msgs::NavSatFix CurrentGPS;
sensor_msgs::NavSatFix DesiredGPS;

geometry_msgs::Vector3 CurrentPlot;
geometry_msgs::Vector3 DesiredPlot;
geometry_msgs::Vector3 TranslatedPlot;
geometry_msgs::TwistStamped comm;

geometry_msgs::Vector3 att;

geometry_msgs::Vector3 normalize(geometry_msgs::Vector3 v)
{
	geometry_msgs::Vector3 tp;
	tp.x=(v.x*v.x)/sqrt(v.x*v.x+v.y*v.y);
	tp.y=(v.y*v.y)/sqrt(v.x*v.x+v.y*v.y);
	
	return tp;
}

geometry_msgs::Vector3 ToEulerianAngle(geometry_msgs::Quaternion quat)
{
	tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
	tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    geometry_msgs::Vector3 ans;
    ans.x=roll;
    ans.y=pitch;
    ans.z=yaw;   

    return ans;
}


geometry_msgs::Vector3 GPStoWorldCoordinate(sensor_msgs::NavSatFix gps)
{
	geometry_msgs::Vector3 tmp;
	tmp.x=(gps.longitude-ZeropointX)*longtolatscale;
	tmp.y=gps.latitude-ZeropointY;
	return tmp;
}

float bearingP1toP2(float P1_latitude, float P1_longitude, float P2_latitude, float P2_longitude)
{
	// 현재 위치 : 위도나 경도는 지구 중심을 기반으로 하는 각도이기 때문에 라디안 각도로 변환한다.
	float Cur_Lat_radian = P1_latitude * (3.141592 / 180);
	float Cur_Lon_radian = P1_longitude * (3.141592 / 180);


	// 목표 위치 : 위도나 경도는 지구 중심을 기반으로 하는 각도이기 때문에 라디안 각도로 변환한다.
	float Dest_Lat_radian = P2_latitude * (3.141592 / 180);
	float Dest_Lon_radian = P2_longitude * (3.141592 / 180);

	// radian distance
	float radian_distance = 0;
	radian_distance = acos(sin(Cur_Lat_radian) * sin(Dest_Lat_radian)
		+ cos(Cur_Lat_radian) * cos(Dest_Lat_radian) * cos(Cur_Lon_radian - Dest_Lon_radian));

	// 목적지 이동 방향을 구한다.(현재 좌표에서 다음 좌표로 이동하기 위해서는 방향을 설정해야 한다. 라디안값이다.
	float radian_bearing = acos((sin(Dest_Lat_radian) - sin(Cur_Lat_radian)
		* cos(radian_distance)) / (cos(Cur_Lat_radian) * sin(radian_distance)));

	// acos의 인수로 주어지는 x는 360분법의 각도가 아닌 radian(호도)값이다.		

	float true_bearing = 0;
	if (sin(Dest_Lon_radian - Cur_Lon_radian) < 0)
	{
		true_bearing = radian_bearing * (180 / 3.141592);
		true_bearing = 360 - true_bearing;
	}
	else
	{
		true_bearing = radian_bearing * (180 / 3.141592);
	}

	return (float)true_bearing;
}

void CommandBebop()
{
	float angle=0;	
	angle=bearingP1toP2(CurrentGPS.latitude,CurrentGPS.longitude,DesiredGPS.latitude,DesiredGPS.longitude);
	//현재 위치와 목표 위치 사이의 각도	
	
    comm.twist.linear.x=(DesiredPlot.x-CurrentPlot.x)*10000;
    comm.twist.linear.y=(DesiredPlot.y-CurrentPlot.y)*10000;

	
	
	if(angle-90<180)//반시계
	{
		comm.twist.angular.z=-0.2;	
	}
	else if(angle-90>=180 || angle<90)//시계
	{
		comm.twist.angular.z=0.2;	
	}	
	move_pub.publish(comm);	
}

void ReceiveOdom(geometry_msgs::PoseStamped quat)
{
    double theta;

    tf::Quaternion q(quat.pose.orientation.x, quat.pose.orientation.y, quat.pose.orientation.z, quat.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    att.z=yaw/degreeToradian;
    att.z=att.z+90;
    if(att.z>180)
    {
        att.z=-90-(90-(att.z-180));
    }
    CommandBebop();
}
void ReceiveGPS(sensor_msgs::NavSatFix vel)
{
    CurrentGPS=vel;
    CurrentPlot=GPStoWorldCoordinate(CurrentGPS); //현재위치 월드좌표
}
void ReceiveDesiredGPS(sensor_msgs::NavSatFix vel)
{
    DesiredGPS=vel;
    DesiredPlot=GPStoWorldCoordinate(DesiredGPS); //목표위치 월드좌표
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "BebopAutoPilot");    
    ros::NodeHandle nh_;
    
    ros::Subscriber sub_odom = nh_.subscribe("/mavros1/local_position/pose", 1, ReceiveOdom);
	ros::Subscriber sub_gps = nh_.subscribe("/mavros1/global_position/global", 1, ReceiveGPS);
	ros::Subscriber sub_desiredgps = nh_.subscribe("/target1", 1, ReceiveDesiredGPS);
	
    move_pub = nh_.advertise<geometry_msgs::TwistStamped>("/mavros1/setpoint_velocity/cmd_vel",100);
	//move_pub=nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);	
    
       
    ros::spin();
	
	return 0;	
}





