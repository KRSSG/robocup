#include <iostream>
#include <cstring>
#include <ctime>
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic_factory.h"
#include <stdio.h>

#include "ros/ros.h"
#include "robot.h"

int main(int argc, char *argv[])
{
  using namespace Strategy;
  ros::init(argc, argv, "robot"); // node name should always be remapped (eg bot0, bot1 etc.);
  ros::NodeHandle n("~");
  int botID;
  if (!n.getParam("botID", botID)) {
    printf("ERROR: need to pass botID as private parameter to start node. Exiting.\n");
    ros::shutdown();
    return 0;
  }
  // create robot object
  Robot robot(botID, n);
  ros::Subscriber tp_sub = n.subscribe("/tactic_topic", 1000, &Robot::tacticPacketCallback, &robot); // this topic needs to be remapped
  ros::Subscriber bs_sub = n.subscribe("/belief_state", 1000, &Robot::beliefStateCallback, &robot);
  ros::spin();

  return 0;
}