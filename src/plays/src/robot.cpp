#include <iostream>
#include <cstring>
#include <ctime>
#include "krssg_ssl_msgs/BeliefState.h"
#include "tactics/tactic_factory.h"
#include <stdio.h>

#include "ros/ros.h"
#include "robot.h"
using namespace std;


namespace Strategy {

  Robot::Robot(int botID, ros::NodeHandle &n): botID(botID), 
                                              gotTacticPacket(false), 
                                              n(n),
                                              commandPub(n.advertise<krssg_ssl_msgs::gr_Commands>("/grsim_data", 1000))
  {
    // lets make the default tactic = TPosition, pos = (0,0)
    curTactic = TacticFactory::instance()->Create("TPosition", botID);
    // tParam should already be 0,0,0 no need to set again.
  }

  Robot::~Robot() {}

  void Robot::beliefStateCallback(const krssg_ssl_msgs::BeliefState::ConstPtr &bs) {
    using namespace krssg_ssl_msgs;
    // printf("got beliefState\n");

    // printf("bot %d: (%f, %f)\n", botID, bs->homePos[botID].x, bs->homePos[botID].y);
    if (gotTacticPacket) {
      gotTacticPacket = false;
      curTactic = TacticFactory::instance()->Create(tID, botID);
      curParam = curTactic->paramFromJSON(tParamJSON);
    }
    gr_Robot_Command robot_command = curTactic->execute(*bs, curParam);
    gr_Commands command;
    command.robot_commands = robot_command;
    command.timestamp = ros::Time::now().toSec();
    command.isteamyellow = bs->isteamyellow;
    commandPub.publish(command);
  }
  void Robot::tacticPacketCallback(const krssg_ssl_msgs::TacticPacket::ConstPtr& tp) {
    printf("got tactic packet for bot (%d), tactic = (%s)\n", botID, tp->tID.c_str());
    tParamJSON = tp->tParamJSON;
    tID = tp->tID;
    gotTacticPacket =  true;
  }

}

