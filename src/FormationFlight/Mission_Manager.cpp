#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <std_msgs/Bool.h>
#define MissionSize 6



geometry_msgs::Vector3 dir;
sensor_msgs::NavSatFix Missions[MissionSize];
sensor_msgs::NavSatFix CurrentMission;
bool arrived=false;
int MissionNum=0;
ros::Publisher mission_pub;
double MstartTime;
double MTime;


void ReceiveMissionReceived(std_msgs::Bool vel)
{
    arrived=vel.data;

    if(arrived)
    {        
        ROS_INFO("Current Mission : %d",MissionNum);
        if(MissionNum <= MissionSize)
        {
            MissionNum++;            
            ROS_INFO("arrived!! Goto Next WP");
        }        
        else if (MissionNum>MissionSize)
        {
            ROS_INFO("finished");
        }
    }    
}
void InputMission(sensor_msgs::NavSatFix M,int order)
{
    Missions[order]=M;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Mission_Manager");
  ros::NodeHandle n;
  sensor_msgs::NavSatFix tmp;


  // 2017-09-07

  /*tmp.latitude = 37.6008854;
  tmp.longitude = 126.866451;
  tmp.altitude = 5;
  InputMission(tmp, 0);

  tmp.latitude = 37.600807;
  tmp.longitude = 126.8667029;
  tmp.altitude = 5;
  InputMission(tmp, 1);

  tmp.latitude = 37.6005007;
  tmp.longitude = 126.8666028;
  tmp.altitude = 5;
  InputMission(tmp, 2);

  tmp.latitude = 37.6005007;
  tmp.longitude = 126.8666028;
  tmp.altitude = 3;
  InputMission(tmp, 3);

  tmp.latitude = 37.6005435;
  tmp.longitude = 126.8664685;
  tmp.altitude = 5;
  InputMission(tmp, 4);

  tmp.latitude = 37.6006283;
  tmp.longitude = 126.8664249;
  tmp.altitude = 7;
  InputMission(tmp, 5);

  tmp.latitude = 37.6007109;
  tmp.longitude = 126.8663684;
  tmp.altitude = 5;
  InputMission(tmp, 6);

  tmp.latitude = 37.6006794;
  tmp.longitude = 126.8662114;
  tmp.altitude = 5;
  InputMission(tmp, 7);

  tmp.latitude = 37.6007491;
  tmp.longitude = 126.8662097;
  tmp.altitude = 5;
  InputMission(tmp, 8);

  tmp.latitude = 37.6007818;
  tmp.longitude = 126.8662665;
  tmp.altitude = 5;
  InputMission(tmp, 9);

  tmp.latitude = 37.6009635;
  tmp.longitude = 126.8662035;
  tmp.altitude = 3;
  InputMission(tmp, 10);*/


////////////////////  본관////////////////
  tmp.latitude=37.5995457;
  tmp.longitude=126.863386;
  tmp.altitude=3.5;
  InputMission(tmp,0);





  tmp.latitude=37.5996187;
  tmp.longitude=126.8632602;
  tmp.altitude=3.5;
  InputMission(tmp,1);




  tmp.latitude=37.599537;
  tmp.longitude=126.8631766;
  tmp.altitude=3.5;
  InputMission(tmp,2);




  tmp.latitude=37.5994378;
  tmp.longitude=126.8633271;
  tmp.altitude=3.5;
  InputMission(tmp,3);


  tmp.latitude=37.5993906;
  tmp.longitude=126.8635055;
  tmp.altitude=3.5;
  InputMission(tmp,4);


  tmp.latitude=37.5995247;
  tmp.longitude=126.8634441;
  tmp.altitude=3.5;
  InputMission(tmp,5);


  ros::Subscriber arr_sub=n.subscribe("/arrived1", 10, ReceiveMissionReceived);
//  mission_pub = n.advertise<sensor_msgs::NavSatFix>("/manual_Mission",10);
  mission_pub = n.advertise<sensor_msgs::NavSatFix>("/target1",10);

  ros::Rate loop_rate(20);
  while(ros::ok()){
      CurrentMission=Missions[MissionNum];
      mission_pub.publish(CurrentMission);
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
