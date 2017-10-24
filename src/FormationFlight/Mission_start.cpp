#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>



std_msgs::Bool start;
ros::Publisher mission_pub;

void PublishStart()
{
    start.data=false;
    char a;
    std::cin>>a;
    if(a=='s')
    {
        start.data=true;
        mission_pub.publish(start);
    }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Mission_start");
  ros::NodeHandle n;
  mission_pub = n.advertise<std_msgs::Bool>("/start",10);

  ros::Rate loop_rate(20);
  while(ros::ok()){
      PublishStart();
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
