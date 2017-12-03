#include "ros/ros.h"
#include "std_msgs/String.h"
#include "grSim_Packet.pb.h"
#include "grSim_Replacement.pb.h"
#include "grSim_Commands.pb.h"
#include <sslDebug_Data.pb.h>

#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <QtGui/QMainWindow>
#include <QtGui/QDialog>
#include <QtNetwork>

#include "krssg_ssl_msgs/gr_BallReplacement.h"
#include "krssg_ssl_msgs/gr_Robot_Command.h"
#include "krssg_ssl_msgs/gr_RobotReplacement.h"
#include "krssg_ssl_msgs/gr_Packet.h"
#include "krssg_ssl_msgs/gr_Commands.h"
#include "krssg_ssl_msgs/gr_Replacement.h"
#include <krssg_ssl_msgs/sslDebug_Data.h>
// listens to 2 topics:
// grsim_data and grsim_debug_data
using namespace std;

QUdpSocket udpsocket;
QHostAddress _addr = QHostAddress("127.0.0.1");
quint16 _port = 20011;

void CallbackCmd(const krssg_ssl_msgs::gr_Commands::ConstPtr& msg)
{
	grSim_Packet packet;
	packet.mutable_commands()->set_isteamyellow(msg->isteamyellow);
	packet.mutable_commands()->set_timestamp(msg->timestamp);
	grSim_Robot_Command* command = packet.mutable_commands()->add_robot_commands();
	command->set_id(msg->robot_commands.id);
	command->set_kickspeedx(msg->robot_commands.kickspeedx);
	command->set_kickspeedz(msg->robot_commands.kickspeedz);
	command->set_veltangent(msg->robot_commands.veltangent);
	command->set_velnormal(msg->robot_commands.velnormal);
	command->set_velangular(msg->robot_commands.velangular);
	command->set_spinner(msg->robot_commands.spinner);
	command->set_wheelsspeed(0);
	QByteArray dgram;
	dgram.resize(packet.ByteSize());
	packet.SerializeToArray(dgram.data(), dgram.size());
	udpsocket.writeDatagram(dgram, _addr, _port);
}
void CallbackDbg(const krssg_ssl_msgs::sslDebug_Data::ConstPtr& msg)
{	
	grSim_Packet packet;
	packet.mutable_debuginfo()->set_id(msg->id);
	for (int i = 0; i < msg->circle.size(); ++i)
	{
		Debug_Circle *c = packet.mutable_debuginfo()->add_circle();
		c->set_x(msg->circle[i].x);
		c->set_y(msg->circle[i].y);
		c->set_radius(msg->circle[i].radius);
		c->set_color(msg->circle[i].color);
	}
	for (int i = 0; i < msg->line.size(); ++i)
	{
		Debug_Line *l = packet.mutable_debuginfo()->add_line();
		l->set_x1(msg->line[i].x1);
		l->set_x2(msg->line[i].x2);
		l->set_y1(msg->line[i].y1);
		l->set_y2(msg->line[i].y2);
		l->set_color(msg->line[i].color);
	}
	
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
	ros::Subscriber subCmd = n1.subscribe("/grsim_data", 1000, CallbackCmd);
	ros::Subscriber subDbg = n1.subscribe("/grsim_debug_data", 1000, CallbackDbg);
	ros::spin();
	return 0;
}
