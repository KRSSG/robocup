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
    curTactic = TacticFactory::instance()->Create("TStop", botID);
    // tParam should already be 0,0,0 no need to set again.
  }

  Robot::~Robot() {}

  void PRINT(const krssg_ssl_msgs::BeliefState::ConstPtr &bs)
  {
    cout<<"isteamyellow: "<<bs->isteamyellow<<" frame_number: "<<bs->frame_number<<" t_capture: "<<bs->t_capture<<" t_sent: "<<bs->t_sent<<endl;
    cout<<"ballPos.x: "<<bs->ballPos.x<<" bs->ballPos.y "<<bs->ballPos.y<<" bs->ballVel.x: "<<bs->ballVel.x<<" bs->ballVel.y: "<<bs->ballVel.y<<endl;
    cout<<"ballDetected: "<<bs->ballDetected<<endl;
    for(int i=0;i<6;i++)
    {
      cout<<"awayPos "<<i<<" : "<<bs->awayPos[i].x<<" "<<bs->awayPos[i].y<<endl;
    }
    for(int i=0;i<6;i++)
    {
      cout<<"homePos "<<i<<" : "<<bs->homePos[i].x<<" "<<bs->homePos[i].y<<endl;
    }
    for(int i=0;i<6;i++)
    {   
      cout<<"homeDetected "<<i<<" : "<<bs->homeDetected[i];
    }
    for(int i=0;i<6;i++)
    {
      cout<<"awayDetected "<<i<<" : "<<bs->awayDetected[i];
    }
    cout<<"our_bot_closest_to_ball: "<<bs->our_bot_closest_to_ball<<" opp_bot_closest_to_ball: "<<bs->opp_bot_closest_to_ball<<" our_goalie: "<<bs->our_goalie<<endl;
    cout<<"opp_goalie: "<<bs->opp_goalie<<" opp_bot_marking_our_attacker: "<<bs->opp_bot_marking_our_attacker<<" ball_at_corners: "<<bs->ball_at_corners<<endl;
    cout<<"ball_in_our_half: "<<bs->ball_in_our_half<<" ball_in_our_possession: "<<bs->ball_in_our_possession<<endl;
  }

  void Robot::beliefStateCallback(const krssg_ssl_msgs::BeliefState::ConstPtr &bs) {
    using namespace krssg_ssl_msgs;
    // printf("got beliefState\n");
    cout<<botID<<"In BS"<<endl;

    // printf("bot %d: (%f, %f)\n", botID, bs->homePos[botID].x, bs->homePos[botID].y);
    if (gotTacticPacket) {
      // cout<<botID<<"In IF 1"<<endl;
      gotTacticPacket = false;
      curTactic = TacticFactory::instance()->Create(tID, botID);
      // cout<<botID<<"In IF 2"<<endl;
      curParam = curTactic->paramFromJSON(tParamJSON);
    }
    // if(bs==NULL)
    //   cout<<"Is NULL "<<endl;
    if(botID == 1)
    {
      // PRINT(bs);
    }
    // cout<<botID<<"TParamJSON "<<tParamJSON<<endl;
    gr_Robot_Command robot_command = curTactic->execute(*bs, curParam);

    
    // cout<<botID<<"Executed"<<endl;
    gr_Commands command;
    command.robot_commands = robot_command;
    command.timestamp = ros::Time::now().toSec();
    command.isteamyellow = bs->isteamyellow;
    commandPub.publish(command);

    cout<<botID<<"Published"<<endl;
  }
  void Robot::tacticPacketCallback(const krssg_ssl_msgs::TacticPacket::ConstPtr& tp) {
  //  printf("got tactic packet for bot (%d), tactic = (%s)\n", botID, tp->tID.c_str());
    cout<<botID<<"In TPC"<<endl;
    tParamJSON = tp->tParamJSON;
    tID = tp->tID;
    gotTacticPacket =  true;
  }

}

