#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Pose2D.h"

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

// Currently not adding geometry data

bool isteamyellow = false;

void update_bot_pos(std::vector<krssg_ssl_msgs::SSL_DetectionRobot> robot,
	std::vector<geometry_msgs::Pose2D> pos) {

	for (int i = 0; i < robot.size(); ++i) {
		int bot_id = robot[i].robot_id;
		pos[bot_id].x = robot[bot_id].x;
		pos[bot_id].y = robot[bot_id].y;
	}
}


void update_belief_data(
	krssg_ssl_msgs::SSL_DetectionFrame *frame,
	krssg_ssl_msgs::BeliefState *bf) {

	bf->stamp = ros::Time::now();
	bf->isteamyellow = isteamyellow;
	bf->frame_number = frame->frame_number;
	bf->t_capture = frame->t_capture;
	bf->t_sent = frame->t_sent;

	// Ball
	Pose2D ballPos;
	float max_confidence = INT_MIN;
	int ball_idx;
	for (int i = 0; i < frame->balls.size(); ++i) {
		float confidence = frame->balls[ball_idx].confidence;
		if (max_confidence < confidence) {
			max_confidence = confidence;
			ball_idx = i;
		}
	}
	ballPos.x = frame->balls[ball_idx].x;
	ballPos.y = frame->balls[ball_idx].y;
	bf->ballPos = ballPos;

	// Bot Positions
	vector<Pose2D> home_bots;
	vector<Pose2D> away_bots;
	if (isteamyellow) {
		update_bot_pos(frame->robots_yellow, home_bots);
		update_bot_pos(frame->robots_blue, away_bots);
	} else {
		update_bot_pos(frame->robots_blue, home_bots);
		update_bot_pos(frame->robots_yellow, away_bots);
	}
	bf->homePos = home_bots;
	bf->awayPos = away_bots;
}


int main(int argc, char **argv)
{

	int vision = 0;
	if (argc < 2) {
		std::cerr << "usage:\n ./vision <vision_port> <team_id>\n"
					 << "<vision_port> : Use 0 for grSim and 1 for ssl-vision\n"
					 << "<team_id> : Use 0 for our team as blue team\n";
		return -1;
	} else {
		vision = atof(argv[1]);
		isteamyellow = atof(argv[2]);
	}

	ros::init(argc, argv, "vision_node");
	ros::NodeHandle n;
	
	ros::Publisher pulisher = n.advertise<krssg_ssl_msgs::BeliefState>("vision", 10000);
	
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	int port = vision? 10020: 10006;
	RoboCupSSLClient client(port);	
	client.open(true);
	printf("Connected to %s.\n", vision? "grSim vision" : "ssl-vision");
	SSL_WrapperPacket packet;
	while(ros::ok()) {
		//see if the packet contains a robot detection frame:
		if (client.receive(packet)) {
			krssg_ssl_msgs::SSL_DetectionFrame msg;
			krssg_ssl_msgs::BeliefState bf;
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

			printf("sending packet...\n");

			update_belief_data(&msg, &bf);
			pulisher.publish(bf);
		}
		ros::spinOnce();
	}

	return 0;
}
