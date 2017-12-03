#pragma once
#include "ros/ros.h"
#include "tactics/tactic.h"
#include <tactics/tactic_factory.h>
#include <krssg_ssl_msgs/BeliefState.h>
#include <krssg_ssl_msgs/gr_Commands.h>
#include <krssg_ssl_msgs/TacticPacket.h>
#include <krssg_ssl_msgs/sslDebug_Data.h>
#include <string>


#include <robojackets/planning/RRTPlanner.hpp>
#include <robojackets/planning/MotionInstant.hpp>
#include <robojackets/planning/MotionCommand.hpp>
#include <robojackets/planning/MotionConstraints.hpp>
#include <robojackets/Configuration.hpp>
#include <robojackets/planning/Path.hpp>
#include <robojackets/Geometry2d/ShapeSet.hpp>
#include <iostream>
#include "robojackets/MotionControl.hpp"
#include <sys/time.h>


namespace Strategy
{
  class RJRobot 
  {

  public:

    void beliefStateCallback(const krssg_ssl_msgs::BeliefState::ConstPtr& bs);
    void tacticPacketCallback(const krssg_ssl_msgs::TacticPacket::ConstPtr& tp);
  private:
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

    // debug data publishers
    ros::Publisher debugPub;

    // pointer to current tactic
    std::auto_ptr<Tactic> curTactic;

    // Stores the current tactic's parameters and is controlled by the Play Layer
    Tactic::Param  curParam;

    // controller
    std::unique_ptr<MotionControl> controller;

    // planner
    std::unique_ptr<Planning::SingleRobotPathPlanner> planner;

    // path
    std::unique_ptr<Planning::Path> path;

  public:
    RJRobot(int          botID, ros::NodeHandle &n);

    ~RJRobot();
  }; // class RJRobot
} // namespace Strategy

