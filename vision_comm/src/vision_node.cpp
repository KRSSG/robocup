#include "ros/ros.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Pose2D.h"
#include "class_node.hpp"

#include "robocup_ssl_client.h"
#include "krssg_ssl_msgs/Vector2f.h"
#include "krssg_ssl_msgs/BeliefState.h"
#include "krssg_ssl_msgs/SSL_DetectionFrame.h"
#include "krssg_ssl_msgs/SSL_DetectionBall.h"
#include "krssg_ssl_msgs/SSL_DetectionRobot.h"
#include "krssg_ssl_msgs/SSL_GeometryData.h"
#include "krssg_ssl_msgs/SSL_GeometryCameraCalibration.h"
#include "krssg_ssl_msgs/SSL_GeometryFieldSize.h"
#include "krssg_ssl_msgs/SSL_WrapperPacket.h"
#include "krssg_ssl_msgs/SSL_FieldLineSegment.h"
#include "krssg_ssl_msgs/SSL_FieldCircularArc.h"

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

#include "timer.h"
#include <sstream>
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

	ros::init(argc, argv, "vision_node");
	ros::NodeHandle n;
	BeliefState bf(isteamyellow);
	
	ros::Publisher pulisher = n.advertise<krssg_ssl_msgs::BeliefState>("vision", 10000);

	GOOGLE_PROTOBUF_VERIFY_VERSION;
	int port = vision == 1 ? 10020: 10006;
	RoboCupSSLClient client(port);	
	client.open(true);
	printf("Connected to %s.\n", vision? "grSim vision" : "ssl-vision");

	SSL_WrapperPacket packet;
   krssg_ssl_msgs::SSL_WrapperPacket ssl_packet;
	while(ros::ok()) {
		//see if the packet contains a robot detection frame:
		if (client.receive(packet)) {
			krssg_ssl_msgs::BeliefState final_msg;

         // Detection Data
			krssg_ssl_msgs::SSL_DetectionFrame ssl_frame;
			if (packet.has_detection()) {
				SSL_DetectionFrame detection = packet.detection();
				int balls_n = detection.balls_size();
				int robots_blue_n =  detection.robots_blue_size();
				int robots_yellow_n =  detection.robots_yellow_size();

				ssl_frame.frame_number = detection.frame_number();
				ssl_frame.t_capture = detection.t_capture();
				ssl_frame.t_sent = detection.t_sent();
				ssl_frame.camera_id = detection.camera_id();

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
					ssl_frame.balls.push_back(ballmsg);
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
					ssl_frame.robots_blue.push_back(botmsg);
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
					ssl_frame.robots_yellow.push_back(botmsg);
				}
			}
         ssl_packet.detection = ssl_frame;

         // Geometry related data
         krssg_ssl_msgs::SSL_GeometryData geom_msg;
         if (packet.has_geometry()) {

            // Geometry Field Related data
            const SSL_GeometryData & geom = packet.geometry();
            krssg_ssl_msgs::SSL_GeometryFieldSize field_size_msg;
            const SSL_GeometryFieldSize & field = geom.field();
            field_size_msg.field_length = field.field_length();
            field_size_msg.field_width = field.field_width();
            field_size_msg.boundary_width = field.boundary_width();
				field_size_msg.goal_width = field.goal_width();
				field_size_msg.goal_depth = field.goal_depth();

            // Field Lines info
   			for(int i =0; i<field.field_lines_size(); i++) {
   			  const SSL_FieldLineSegment &field_lines = field.field_lines(i);
              krssg_ssl_msgs::SSL_FieldLineSegment field_lines_msg;
              field_lines_msg.name = field_lines.name();
              field_lines_msg.p1.x = field_lines.p1().x();
              field_lines_msg.p1.y = field_lines.p1().y();
              field_lines_msg.p2.x = field_lines.p2().x();
              field_lines_msg.p2.y = field_lines.p2().y();
              field_lines_msg.thickness = field_lines.thickness();
              field_size_msg.field_lines.push_back(field_lines_msg);
   			}

            // Field Arcs info
   			for(int i =0; i<field.field_arcs_size(); i++) {
   				const SSL_FieldCircularArc &field_arcs = field.field_arcs(i);
               krssg_ssl_msgs::SSL_FieldCircularArc field_arcs_msg;
               field_arcs_msg.name = field_arcs.name();
               field_arcs_msg.center.x = field_arcs.center().x();
               field_arcs_msg.center.y = field_arcs.center().y();
               field_arcs_msg.radius = field_arcs.radius();
               field_arcs_msg.a1 = field_arcs.a1();
               field_arcs_msg.a2 = field_arcs.a2();
               field_arcs_msg.thickness = field_arcs.thickness();
               field_size_msg.field_arcs.push_back(field_arcs_msg);
   			}
            geom_msg.field = field_size_msg;

            // Camera specific data
            int calib_n = geom.calib_size();
            for(int i=0; i< calib_n; i++) {
               const SSL_GeometryCameraCalibration & calib = geom.calib(i);
               krssg_ssl_msgs::SSL_GeometryCameraCalibration calib_msg;
					calib_msg.camera_id =  calib.camera_id();
					calib_msg.focal_length = calib.focal_length();
					calib_msg.principal_point_x = calib.principal_point_x();
					calib_msg.principal_point_y = calib.principal_point_y();
					calib_msg.distortion = calib.distortion();
					calib_msg.q0 = calib.q0();
					calib_msg.q1 = calib.q1();
					calib_msg.q2 = calib.q2();
					calib_msg.q3 = calib.q3();
					calib_msg.tx = calib.tx();
					calib_msg.ty = calib.ty();
					calib_msg.tz = calib.tz();

               if (calib.has_derived_camera_world_tx() &&
                   calib.has_derived_camera_world_ty() &&
                   calib.has_derived_camera_world_tz()) {
                     calib_msg.derived_camera_world_tx = calib.derived_camera_world_tx();
                     calib_msg.derived_camera_world_ty = calib.derived_camera_world_ty();
                     calib_msg.derived_camera_world_tz = calib.derived_camera_world_tz();
               }

               geom_msg.calib.push_back(calib_msg);
            }
         }
         ssl_packet.geometry = geom_msg;

			bf.update_frame(&ssl_packet);
			printf("sending packet...\n");
			bf.print(vision == 1? "grSim vision" : "ssl-vision");
			final_msg = bf.get_beliefstate_msg();
			pulisher.publish(final_msg);
		}
		ros::spinOnce();
	}
	return 0;
}
