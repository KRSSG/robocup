#include "ros/ros.h"
#include "std_msgs/String.h"

#include "robocup_ssl_client.h"
#include <sstream>
#include <krssg_ssl_msgs/SSL_DetectionFrame.h>

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include <ctime>
#include "timer.h"


using namespace std;


int main(int argc, char **argv)
{
	// by default, connects to SSLVision port
	// if a non-zero number is passed as argument, connects to grsim vision port.
	int use_grsim_vision = 0;
	if (argc > 1) {
		use_grsim_vision = atof(argv[1]);
	}

	ros::init(argc, argv, "vision_node");
	ros::NodeHandle n;
	
	ros::Publisher chatter_pub = n.advertise<krssg_ssl_msgs::SSL_DetectionFrame>("vision", 10000);
	// ros::Rate loop_rate(10);

	
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	int port = use_grsim_vision? 10020: 10006;
	RoboCupSSLClient client(port);	
	client.open(true);
	printf("Connected to %s.\n", use_grsim_vision? "grSim vision" : "ssl-vision");
	SSL_WrapperPacket packet;
	while(ros::ok()) {
		if (client.receive(packet)) {
			krssg_ssl_msgs::SSL_DetectionFrame msg;
			//see if the packet contains a robot detection frame:
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

			//see if packet contains geometry data:
			// right now, doing nothing for geometry data.
			if (packet.has_geometry()) {
				const SSL_GeometryData & geom = packet.geometry();
				const SSL_GeometryFieldSize & field = geom.field();
				int calib_n = geom.calib_size();
				for (int i=0; i< calib_n; i++) {
					const SSL_GeometryCameraCalibration & calib = geom.calib(i);
				}
			}
			printf("sending packet...\n");
			chatter_pub.publish(msg);
		}
		ros::spinOnce();
		// loop_rate.sleep();
	}

	return 0;
}
