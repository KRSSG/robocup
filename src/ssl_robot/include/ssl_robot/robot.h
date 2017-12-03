#ifndef ROBOT_H
#define ROBOT_H
#include "ros/ros.h"
#include "tactics/tactic.h"
#include <tactics/tactic_factory.h>
#include <krssg_ssl_msgs/BeliefState.h>
#include <krssg_ssl_msgs/gr_Commands.h>
#include <krssg_ssl_msgs/TacticPacket.h>
#include <string>


namespace Strategy
{
  class Robot 
  {

  public:

    void beliefStateCallback(const krssg_ssl_msgs::BeliefState::ConstPtr& bs);
    void tacticPacketCallback(const krssg_ssl_msgs::TacticPacket::ConstPtr& tp);
  
    int            botID;
    // Belief State object 
    krssg_ssl_msgs::BeliefState   bs;
    // Stores the current tactic and is controlled by the Play Layer
    std::string    tID, tParamJSON;

    // is set to true whenever a packet is received from the play executer.
    bool gotTacticPacket;

    // node handle
    ros::NodeHandle &n;

    // bot command publisher
    ros::Publisher commandPub;

    // pointer to current tactic
    std::auto_ptr<Tactic> curTactic;

    // Stores the current tactic's parameters and is controlled by the Play Layer
    Tactic::Param  curParam;
  public:
    Robot(int botID, ros::NodeHandle &n);

    ~Robot();
  }; // class Robot
} // namespace Strategy

#endif // ROBOT_H
