#include "ros/ros.h"
#include "std_msgs/String.h"
#include <bits/stdc++.h>
#include "robocup_ssl_client.h"
#include <sstream>
#include <krssg_ssl_msgs/SSL_DetectionFrame.h>

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include <ctime>
#include "timer.h"
#include <unistd.h>

#define num_cam 4

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

	float max_ball_confidence = 0;
	float max_blue_bot_confidence[6] = {0};
	float max_yellow_bot_confidence[6] = {0};
	int camera_bool[num_cam] = {0};
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	int port = use_grsim_vision? 10020: 10006;
	string net_address = "224.5.23.2";
	RoboCupSSLClient client(port, net_address);	
	client.open();
	printf("Connected to %s.\n", use_grsim_vision? "grSim vision" : "ssl-vision");
	SSL_WrapperPacket packet;
	krssg_ssl_msgs::SSL_DetectionFrame msg;
	int is_blue_bot_detected[6] = {0};
	int is_yellow_bot_detected[6] = {0};
	map< int, krssg_ssl_msgs::SSL_DetectionRobot > blue_bots;
	map< int, krssg_ssl_msgs::SSL_DetectionRobot > yellow_bots;
	krssg_ssl_msgs::SSL_DetectionBall ball;
	while(ros::ok()) {
		//printf("........abcd\n");
		if (client.receive(packet)) {
			cout<<"In IF************\n";
			
			//see if the packet contains a robot detection frame:
			if (packet.has_detection()) {
				SSL_DetectionFrame detection = packet.detection();
				int balls_n = detection.balls_size();
				int robots_blue_n =  detection.robots_blue_size();
				int robots_yellow_n =  detection.robots_yellow_size();

				msg.frame_number = detection.frame_number();
				msg.t_capture = detection.t_capture();
				
				cout<<"Frame number: "<<msg.frame_number<<endl;
				msg.t_sent = detection.t_sent();
				msg.camera_id = detection.camera_id();
				//Ball info:
				cout<<"Camera num: "<<detection.camera_id()<<endl;
				if (camera_bool[detection.camera_id()] == 1)
				{
					max_ball_confidence = 0;
					msg.balls.push_back(ball);	
					// cout << "ball.x"				
				}
				for (int i = 0; i < balls_n; i++) {
					SSL_DetectionBall ball_new = detection.balls(i);
					
					//krssg_ssl_msgs::SSL_DetectionBall ballmsg;
					if (ball_new.confidence() > max_ball_confidence)
					{
						max_ball_confidence = ball_new.confidence();
						ball.confidence = ball_new.confidence();
						ball.area = ball_new.area();
						ball.x = ball_new.x();
						ball.y = ball_new.y();
						ball.z = ball_new.z();
					}
					
					// Not using 
					// ballmsg.pixel_x = ball.pixel_x();
					// ballmsg.pixel_y = ball.pixel_y();
					//msg.balls.push_back(ballmsg);
					//cout << "Ball pos: " << ballmsg.x<<"," << ballmsg.y<<endl;
				}

				//Blue robot info:
				//cout<<"********* "<<robots_blue_n<<endl;
				if (camera_bool[detection.camera_id()] == 1)
				{
					//cout<<"Updating Blue bots"<<endl;
					for(int i = 0; i < 6; i++)
					{
						//if(is_blue_bot_detected[i] == 1)
						msg.robots_blue.push_back(blue_bots[i]);
						max_blue_bot_confidence[i] = 0;
					}
					for(int i = 0; i < 6; i++)
						is_blue_bot_detected[i] = 0;
				}
				for (int i = 0; i < robots_blue_n; i++) {
					SSL_DetectionRobot robot = detection.robots_blue(i);
					// krssg_ssl_msgs::SSL_DetectionRobot botmsg;
					blue_bots[robot.robot_id()].robot_id = robot.robot_id();
					if (robot.confidence() > max_blue_bot_confidence[blue_bots[robot.robot_id()].robot_id])
					{
						max_blue_bot_confidence[blue_bots[robot.robot_id()].robot_id] = robot.confidence();
						blue_bots[robot.robot_id()].confidence = robot.confidence();
						blue_bots[robot.robot_id()].x = robot.x();
						blue_bots[robot.robot_id()].y = robot.y();
						blue_bots[robot.robot_id()].orientation = robot.orientation();
					}
					// Not using
					// botmsg.pixel_x = robot.pixel_x();
					// botmsg.pixel_y = robot.pixel_y();
					// botmsg.height = robot.height();
					//msg.robots_blue.push_back(botmsg);
					//cout << "Blue bot detected, bot_id =  " << robot.robot_id() << " pos: " << robot.x() << " " << robot.y() << endl;
				}
				if (camera_bool[detection.camera_id()] == 1)
				{
					for(int i = 0; i < 6; i++)
						max_yellow_bot_confidence[i] = 0,is_yellow_bot_detected[i] = 0;
					//msg.robots_yellow = yellow_bots;
				}
				//Yellow robot info:
				for (int i = 0; i < robots_yellow_n; i++) {
					SSL_DetectionRobot robot = detection.robots_yellow(i);
					
					//krssg_ssl_msgs::SSL_DetectionRobot botmsg;
					yellow_bots[robot.robot_id()].robot_id = robot.robot_id();
					if (robot.confidence() > max_yellow_bot_confidence[yellow_bots[robot.robot_id()].robot_id])
					{
						max_yellow_bot_confidence[yellow_bots[robot.robot_id()].robot_id] = robot.confidence();
						yellow_bots[robot.robot_id()].confidence = robot.confidence();
						yellow_bots[robot.robot_id()].x = robot.x();
						yellow_bots[robot.robot_id()].y = robot.y();
						yellow_bots[robot.robot_id()].orientation = robot.orientation();
					}
					
					// Not using
					// botmsg.pixel_x = robot.pixel_x();
					// botmsg.pixel_y = robot.pixel_y();
					// botmsg.height = robot.height();
					//msg.robots_yellow.push_back(botmsg);

				}
				if(camera_bool[detection.camera_id()] == 1)
				{	
					for(int i = 0; i < 6; i++)
					{
						//if(is_yellow_bot_detected[i] == 1)
						msg.robots_yellow.push_back(yellow_bots[i]);
						
					}
					
					cout << "ball size       "<<msg.balls.size() << endl;
					chatter_pub.publish(msg);
					for(int i = 0; i < num_cam; i++)
					{
						camera_bool[i] = 0;
					}
					msg.robots_blue.clear();
					msg.robots_yellow.clear();
					msg.balls.clear();

				}
				camera_bool[detection.camera_id()] = 1;

			}

			//see if packet contains geometry data:
			// right now, doing nothing for geometry data.
			// if (packet.has_geometry()) {
			// 	const SSL_GeometryData & geom = packet.geometry();
			// 	const SSL_GeometryFieldSize & field = geom.field();
			// 	int calib_n = geom.calib_size();
			// 	for (int i=0; i< calib_n; i++) {
			// 		const SSL_GeometryCameraCalibration & calib = geom.calib(i);
			// 	}
			// }
			//cout<<"sending packet...\n";
			
			//chatter_pub.publish(msg);
		}
		ros::spinOnce();
		// loop_rate.sleep();
	}

	return 0;
}
