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

#include <sstream>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_ssl");
  ros::NodeHandle nh;
  ros::Publisher test_pub = nh.advertise<krssg_ssl_msgs::gr_Commands>("/grsim_data", 1000);
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
		krssg_ssl_msgs::gr_Robot_Command msg;
		msg.id = 1;
		msg.kickspeedx = 1;
		msg.kickspeedz = 2;
		msg.veltangent = 0;
		msg.velnormal  = 0;
		msg.velangular = 3;
		msg.spinner = 0;
		msg.wheelsspeed = 0;
		
		krssg_ssl_msgs::gr_Commands command;
		command.robot_commands = msg;
		command.isteamyellow = true;
		command.timestamp = 0.0;
		
		test_pub.publish(command);
		ros::spinOnce();
		loop_rate.sleep();
    }


  return 0;
}

