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

#include "krssg_ssl_msgs/gr_BallReplacement.h"
#include "krssg_ssl_msgs/gr_Robot_Command.h"
#include "krssg_ssl_msgs/gr_RobotReplacement.h"
#include "krssg_ssl_msgs/gr_Packet.h"
#include "krssg_ssl_msgs/gr_Commands.h"
#include "krssg_ssl_msgs/gr_Replacement.h"
#include <krssg_ssl_msgs/SSL_DetectionFrame.h>

using namespace std;

float botx, boty, bottheta, x, y, t = 0., dt = 0.3;
ros::Subscriber sub;
ros::Publisher test_pub;
//ros::Rate loop_rate(100);

float xx(float t) {
	return 0.5 * cos(3.14 * t*0.02);
}
float yy(float t) {
	return 0.5 * sin(3.14 * t*0.02);
}
float getV_tangent(float x1, float y1, float x2, float y2, float dt, float angle) {
	double vx = (x2-x1)/dt;
	double vy = (y2-y1)/dt;
	return vx*cos(angle)+vy*sin(angle);
}
float getV_normal(float x1, float y1, float x2, float y2, float dt, float angle) {
	double vx = (x2-x1)/dt;
	double vy = (y2-y1)/dt;
	return -vx*sin(angle)+vy*cos(angle);
}

void Callback(const krssg_ssl_msgs::SSL_DetectionFrame::ConstPtr& vmsg)
{
	botx = vmsg->robots_blue[0].x;
	boty = vmsg->robots_blue[0].y;
	bottheta = vmsg->robots_blue[0].orientation;
	x = xx(t + dt);
	y = yy(t + dt);
	botx /= 1000.;
	boty /= 1000.;
	
	krssg_ssl_msgs::gr_Robot_Command msg;
	msg.id = 0;
	msg.kickspeedx = 0;
	msg.kickspeedz = 0;
	msg.veltangent = getV_tangent(botx, boty, x, y, dt, bottheta);
	msg.velnormal  = getV_normal(botx, boty, x, y, dt, bottheta);
	msg.velangular = 0;
	msg.spinner = 0;
	msg.wheelsspeed = 0;
	
	krssg_ssl_msgs::gr_Commands command;
	command.robot_commands = msg;
	command.isteamyellow = false;
	command.timestamp = 0.0;

	cout << "Bot position: " << botx << ", " << boty << ", " << bottheta << endl;
	cout << "Velocities: " << getV_tangent(botx, boty, x, y, dt, bottheta) << ", " << getV_normal(botx, boty, x, y, dt, bottheta) << endl;

	test_pub.publish(command);
	usleep(16000);
	t += 0.16;
	//loop_rate.sleep();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "traj_node");
	ros::NodeHandle nh, ns;
	test_pub = nh.advertise<krssg_ssl_msgs::gr_Commands>("/grsim_data", 1000);
	sub = ns.subscribe("vision", 1000, Callback);
	ros::spin();
    return 0;
}