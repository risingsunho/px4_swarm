#include <ros/ros.h>
#include <stdio.h>
#include "TCPClient.h"
#include "geometry_msgs/Vector3.h"
#include <sstream>

using namespace std;

TCPClient client;
string msg;
geometry_msgs::Vector3 pos;
geometry_msgs::Vector3 att;
bool attOK=false;
bool posOK=false;

std::string Convert(float number)
{
    std::ostringstream buff;
    buff<<number;
    return buff.str();   
}
void ReceiveATT(geometry_msgs::Vector3 vel)
{
    att.x=vel.x;
    att.y=vel.y;
    att.z=vel.z;
    attOK=true;
}
void ReceivePOS(geometry_msgs::Vector3 vel)
{
    pos.x=vel.x;
    pos.y=vel.y;
    pos.z=vel.z;
    posOK=true;
}
int main(int argc, char **argv)
{
   ros::init(argc, argv, "Client_test");
   ros::NodeHandle n;
   ros::Rate loop_rate(30);

   ros::Subscriber rpy_sub = n.subscribe("/RollPitchYaw", 100, ReceiveATT);
   ros::Subscriber pos_sub = n.subscribe("/pos", 100, ReceivePOS);


   client.setup("192.168.88.246",1234);


   while(ros::ok()){
       if(attOK && posOK)
       {
           msg = "\n"+Convert(pos.x)+"\t"+Convert(pos.y)+"\t"+Convert(pos.z)+"\t"+Convert(att.x)+"\t"+Convert(att.y)+"\t"+Convert(att.z)+"\n";
           client.Send(msg);
           cout<<msg<<"\n";
           attOK=false;
           posOK=false;
       }
       ros::spinOnce();
       loop_rate.sleep();
   }


   return 0;
}
