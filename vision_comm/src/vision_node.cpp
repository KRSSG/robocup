#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Pose2D.h"
#include "class_node.hpp"

#include "robocup_ssl_client.h"
#include <sstream>
#include <krssg_ssl_msgs/SSL_DetectionFrame.h>
#include <krssg_ssl_msgs/BeliefState.h>

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

#include "timer.h"
#include <ctime>
#include <climits>

using namespace std;
using namespace geometry_msgs;

bool isteamyellow = false;
bool first_packet = true;

int main(int argc, char **argv)
{
	int vision = 1;
	vision = atof(argv[1]);
	isteamyellow = atof(argv[2]);
	isteamyellow = !isteamyellow;
	ros::init(argc, argv, "vision_node");
	ros::NodeHandle n;
	BeliefState bf(isteamyellow);
	
	ros::Publisher pulisher = n.advertise<krssg_ssl_msgs::BeliefState>("vision", 10000);

	GOOGLE_PROTOBUF_VERIFY_VERSION;
	int port = vision == 1 ? 10020: 10006;
	RoboCupSSLClient client(port);	
	client.open(true);
	ROS_INFO("Connected to %s.\n", vision == 1? "grSim vision" : "ssl-vision");
	SSL_WrapperPacket packet;
	while(ros::ok()) {
		//see if the packet contains a robot detection frame:
		if (client.receive(packet)) {
			krssg_ssl_msgs::BeliefState final_msg;
			krssg_ssl_msgs::SSL_DetectionFrame msg;
			if (packet.has_detection()) {
				SSL_DetectionFrame detection = packet.detection();
				int balls_n = detection.balls_size();
				int robots_blue_n =  detection.robots_blue_size();
				int robots_yellow_n =  detection.robots_yellow_size();

				msg.frame_number = detection.frame_number();
				msg.t_capture = detection.t_capture();
				msg.t_sent = detection.t_sent();
				msg.camera_id = detection.camera_id();

				//Ball info:
				for (int i = 0; i < balls_n; i++) {
					SSL_DetectionBall ball = detection.balls(i);
					krssg_ssl_msgs::SSL_DetectionBall ballmsg;
					ballmsg.confidence = ball.confidence();
					ballmsg.area = ball.area();
					ballmsg.x = ball.x();
					ballmsg.y = ball.y();
					ballmsg.z = ball.z();
					ballmsg.pixel_x = ball.pixel_x();
					ballmsg.pixel_y = ball.pixel_y();
					msg.balls.push_back(ballmsg);
				}

				//Blue robot info:
				for (int i = 0; i < robots_blue_n; i++) {
					SSL_DetectionRobot robot = detection.robots_blue(i);
					krssg_ssl_msgs::SSL_DetectionRobot botmsg;
					botmsg.confidence = robot.confidence();
					botmsg.robot_id = robot.robot_id();
					botmsg.x = robot.x();
					botmsg.y = robot.y();
					botmsg.orientation = robot.orientation();
					botmsg.pixel_x = robot.pixel_x();
					botmsg.pixel_y = robot.pixel_y();
					botmsg.height = robot.height();
					msg.robots_blue.push_back(botmsg);
				}

				//Yellow robot info:
				for (int i = 0; i < robots_yellow_n; i++) {
					SSL_DetectionRobot robot = detection.robots_yellow(i);
					krssg_ssl_msgs::SSL_DetectionRobot botmsg;
					botmsg.confidence = robot.confidence();
					botmsg.robot_id = robot.robot_id();
					botmsg.x = robot.x();
					botmsg.y = robot.y();
					botmsg.orientation = robot.orientation();
					botmsg.pixel_x = robot.pixel_x();
					botmsg.pixel_y = robot.pixel_y();
					botmsg.height = robot.height();
					msg.robots_yellow.push_back(botmsg);
				}
			}

			bf.update_frame(&msg);
			bf.print(vision == 1? "grSim vision" : "ssl-vision");
			final_msg = bf.get_beliefstate_msg();
			pulisher.publish(final_msg);
		}
		ros::spinOnce();
	}
	return 0;
}
