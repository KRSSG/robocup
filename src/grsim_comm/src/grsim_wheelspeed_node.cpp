#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grSim_Packet.pb.h"
#include "grSim_Replacement.pb.h"
#include "grSim_Commands.pb.h"
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <QtGui/QMainWindow>
#include <QtGui/QDialog>
#include <QtNetwork>
#include <bits/stdc++.h>
#include "krssg_ssl_msgs/gr_BallReplacement.h"
#include "krssg_ssl_msgs/gr_Robot_Command.h"
#include "krssg_ssl_msgs/gr_RobotReplacement.h"
#include "krssg_ssl_msgs/gr_Packet.h"
#include "krssg_ssl_msgs/gr_Commands.h"
#include "krssg_ssl_msgs/gr_Replacement.h"


using namespace std;

QUdpSocket udpsocket;
QHostAddress _addr = QHostAddress("127.0.0.1");
quint16 _port = 20011;

void Callback(const krssg_ssl_msgs::gr_Commands::ConstPtr& msg)
{
    grSim_Packet packet;
    packet.mutable_commands()->set_isteamyellow(msg->isteamyellow);
    packet.mutable_commands()->set_timestamp(msg->timestamp);
    grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
    command->set_id(msg->robot_commands.id);
    command->set_kickspeedx(msg->robot_commands.kickspeedx);
    command->set_kickspeedz(msg->robot_commands.kickspeedz);
    command->set_veltangent(0);
    command->set_velnormal(0);
    command->set_velangular(0);
    command->set_spinner(msg->robot_commands.spinner);
    command->set_wheelsspeed(1);

    double arr[]={M_PI/3,3*M_PI/4,5*M_PI/4,5*M_PI/3};
    double a[4];
    double vx,vy,vw;
 
    vx=msg->robot_commands.veltangent;
    vy=msg->robot_commands.velnormal;
    vw = msg->robot_commands.velangular;
    const double BOT_RAD = 0.09;
    const double fac = 1/0.025; // 1/wheel_radius
    for (int i = 0; i < 4; ++i)
    { 
      a[i]=-sin(arr[i])*vx+cos(arr[i])*vy+vw*BOT_RAD;
      a[i] *= fac;
    }
 
    command->set_wheel1(a[0]);
    command->set_wheel2(a[1]);
    command->set_wheel3(a[2]);
    command->set_wheel4(a[3]);
    QByteArray dgram;
    dgram.resize(packet.ByteSize());
    packet.SerializeToArray(dgram.data(), dgram.size());
    udpsocket.writeDatagram(dgram, _addr, _port);
}

int main(int argc, char **argv)
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  ros::init(argc, argv, "grsim_node");
  ros::NodeHandle n,n1;
  ros::Subscriber sub = n1.subscribe("/grsim_data", 1000, Callback);
  ros::spin();
  return 0;
}
