#include "ros/ros.h"
#include "std_msgs/String.h"

#include "robocup_ssl_client.h"
#include <sstream>
#include <krssg_ssl_msgs/Ref.h>
#include <krssg_ssl_msgs/SSL_Refbox.h>
#include <krssg_ssl_msgs/team_info.h>

#include "ssl_referee.pb.h"
#include <ctime>
#include "timer.h"


using namespace std;


int main(int argc, char **argv)
{

	cout<<"Intialised Node\n";
	ros::init(argc, argv, "ref_data");
	ros::NodeHandle n;	
	ros::Publisher chatter_pub = n.advertise<krssg_ssl_msgs::Ref>("/ref_data", 10000);
	// ros::Rate loop_rate(10);

	GOOGLE_PROTOBUF_VERIFY_VERSION;
	int port = 10003;
	RoboCupSSLClient client(port);	
	client.open(true);
	printf("Connected to referee-box.\n");
	
	Referee packet;
	while(ros::ok()) {
		if (client.receive(packet)) {

			krssg_ssl_msgs::Ref msg;
			krssg_ssl_msgs::team_info yellow, blue;
			krssg_ssl_msgs::point_2d designated_position;
			krssg_ssl_msgs::game_event gameEvent;

			msg.packet_timestamp = packet.packet_timestamp();
			msg.stage            = packet.stage();
			msg.stage_time_left  = packet.stage_time_left();
			msg.command          = packet.command();
			msg.command_counter  = packet.command_counter();
			msg.command_timestamp= packet.stage();

			yellow.name          = packet.yellow().name();
			yellow.score         = packet.yellow().score();
			yellow.red_cards     = packet.yellow().red_cards();
			
			int yellow_card_times_size = packet.yellow().yellow_card_times_size();
			for (int i = 0; i < yellow_card_times_size; ++i)
			{
				yellow.yellow_card_times.push_back(packet.yellow().yellow_card_times(i));
			}

			yellow.yellow_cards   = packet.yellow().yellow_cards();
			yellow.timeouts       = packet.yellow().timeouts();
			yellow.timeout_time   = packet.yellow().timeout_time();
			yellow.goalie         = packet.yellow().goalkeeper();
			
			blue.name             = packet.blue().name();
			blue.score            = packet.blue().score();
			blue.red_cards        = packet.blue().red_cards();
			yellow_card_times_size= packet.blue().yellow_card_times_size();
			
			for (int i = 0; i < yellow_card_times_size; ++i)
			{
				blue.yellow_card_times.push_back(packet.blue().yellow_card_times(i));
			}

			blue.yellow_cards= packet.blue().yellow_cards();
			blue.timeouts    = packet.blue().timeouts();
			blue.timeout_time= packet.blue().timeout_time();
			blue.goalie      = packet.blue().goalkeeper();
			
			msg.yellow       = yellow;
			msg.blue         = blue;

			// if (packet.has_designated_position())
			// {	
			// 	designated_position.x  = packet.designated_position().x();
			// 	designated_position.y  = packet.designated_position().y();
			// 	msg.designated_position= designated_position;
			// }

			// msg.blueTeamOnPositiveHalf = packet.blueTeamOnPositiveHalf();
			// if (packet.has_blueTeamOnPositiveHalf())
			// {
			// }
			// if (packet.has_gameEvent())
			// {
			// 	gameEvent.gameEventType = packet.gameEvent().gameEventType();
			// 	krssg_ssl_msgs::Originator originator;
			// 	if (packet.gameEvent().has_originator())
			// 	{
			// 		originator.team = packet.gameEvent().originator().team();
			// 		if (packet.gameEvent().originator().has_botId())
			// 		{
			// 			originator.botId = packet.gameEvent().originator().botId();
			// 		}
			// 		gameEvent.originator = originator;
			// 	}
			// 	if (packet.gameEvent().has_message())
			// 	{
			// 		gameEvent.message = packet.gameEvent().message();
			// 	}
			// 	msg.gameEvent = gameEvent;
			// }

			printf("sending packet...\n");
			chatter_pub.publish(msg);
		}
		ros::spinOnce();
		// loop_rate.sleep();
	}

	return 0;
}
