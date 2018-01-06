#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grSim_Packet.pb.h"
#include "grSim_Replacement.pb.h"
#include "grSim_Commands.pb.h"
#include <sslDebug_Data.pb.h>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <QtGui/QMainWindow>
#include <QtGui/QDialog>
#include <QtNetwork>
#include <std_msgs/Int64MultiArray.h>
#include <stdlib.h>
#include "../../ssl_common/include/ssl_common/geometry.hpp"




using namespace std;

QUdpSocket udpsocket;
QHostAddress _addr = QHostAddress("127.0.0.1");
QHostAddress _addrGC = QHostAddress("224.5.23.2");
quint16 _port = 20011;
quint16 _portGC = 10010;

void Callback(const std_msgs::Int64MultiArray::ConstPtr& msg)
{
  grSim_Packet packet;
  static fstream f ("/home/kgpkubs/Desktop/debug.csv",ios::out|ios::app);
  static bool col = 0;
  static long long int prevX = 0, prevY = 0;
  static vector <pair<Vector2D<int>,Vector2D<int> > > l ;
  packet.mutable_debuginfo()->set_id(ros::this_node::getName());
//  Debug_Circle *circle = packet.mutable_debuginfo()->add_circle();
  vector<Debug_Line> *line_full;
  Debug_Line *line= (packet.mutable_debuginfo()->add_line());
  
  line->set_x1(msg->data[0]);
  line->set_y1(msg->data[1]);
  line->set_x2(prevX);
  line->set_y2(prevY);
  line->set_color(col);
  
  line_full->push_back(*line);

  if((msg->data[0]-prevX)*(msg->data[0]-prevX)+(msg->data[1]-prevY)*(msg->data[1]-prevY)>100){
    l.push_back(make_pair(Vector2D<int>(msg->data[0],msg->data[1]),Vector2D<int>(prevX,prevY)));
//    ROS_INFO("New line added: x: %lld y: %lld xp: %lld yp: %lld",msg->data[0],msg->data[1],prevX,prevY);
   ROS_INFO("%lld, %lld, %lld, %lld",msg->data[0],msg->data[1],prevX,prevY);
    f << msg->data[0] << "," << msg->data[1] << "," << prevX << "," << prevY << "\n";
    f.close();
    prevX = msg->data[0];
    prevY = msg->data[1];
  }
  for(int i=0;i<l.size();i++){
    Debug_Line *line = packet.mutable_debuginfo()->add_line();
    line->set_x1(l[i].first.x);
    line->set_y1(l[i].first.y);
    line->set_x2(l[i].second.x);
    line->set_y2(l[i].second.y);
    line->set_color(col);
    line_full->push_back(*line);
  }

 /* circle->set_x(100);
  circle->set_y(0);
  circle->set_radius(100.);
  circle->set_color(0);*/

  
  // col = rand()*100;

  QByteArray dgram;
  dgram.resize(packet.ByteSize());
  packet.SerializeToArray(dgram.data(), dgram.size());
  udpsocket.writeDatagram(dgram, _addr, _port);
}

void CallbackGC(const std_msgs::Int64MultiArray::ConstPtr& msg){
    sslDebug_Data packet;
    static fstream f ("/home/kgpkubs/Desktop/debug.csv",ios::out|ios::app);
    static bool col = 0;
    static long long int prevX = 0, prevY = 0;
    static vector <pair<Vector2D<int>,Vector2D<int> > > l ;

    packet.set_id(ros::this_node::getName());

    if((msg->data[0]-prevX)*(msg->data[0]-prevX)+(msg->data[1]-prevY)*(msg->data[1]-prevY)>100){
    l.push_back(make_pair(Vector2D<int>(msg->data[0],msg->data[1]),Vector2D<int>(prevX,prevY)));
    ROS_INFO("%lld, %lld, %lld, %lld",msg->data[0],msg->data[1],prevX,prevY);
    f << msg->data[0] << "," << msg->data[1] << "," << prevX << "," << prevY << "\n";
    f.close();
    prevX = msg->data[0];
    prevY = msg->data[1];
  }
  for(int i=0;i<l.size();i++){
    Debug_Line *line = packet.add_line();
    line->set_x1(l[i].first.x);
    line->set_y1(l[i].first.y);
    line->set_x2(l[i].second.x);
    line->set_y2(l[i].second.y);
    line->set_color(col);
    }

    QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    udpsocket.writeDatagram(dgram, _addrGC, _portGC);
  }


int main(int argc, char **argv)
{

  GOOGLE_PROTOBUF_VERIFY_VERSION;
  ros::init(argc, argv, "line_debugger");
  ros::NodeHandle n;


   ros::Subscriber sub = n.subscribe("/debugger", 1000, CallbackGC);
  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}
