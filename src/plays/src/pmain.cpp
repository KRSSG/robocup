#include <iostream>
#include <cstring>
#include <ctime>
#include "krssg_ssl_msgs/BeliefState.h"
#include "pExec.h"
#include <stdio.h>

#include "ros/ros.h"
#include <krssg_ssl_msgs/TacticPacket.h>
#include "tactics/tactic.h"
#include <tactics/tactic_factory.h>
#include <string>
#include <utility>
#include <fstream>

using namespace krssg_ssl_msgs;
using namespace Strategy;

ros::Subscriber state_sub;
krssg_ssl_msgs::BeliefState state;
std::vector<std::pair<std::string, Strategy::Tactic::Param> > roleList;

void publishing();

void Callback(const krssg_ssl_msgs::BeliefState::ConstPtr& msg)
{
  fstream f;
  f.open("/home/kgpkubs/ssl/src/plays/positon.txt",fstream::app);
 // ROS_INFO("in Callback %f ",msg->ballPos.x);
  state.isteamyellow=msg->isteamyellow;
  state.frame_number=msg->frame_number ;
  state.t_capture=msg->t_capture  ;   
  state.t_sent=msg->t_sent   ;
  state.ballPos=msg->ballPos  ;     
  state.ballVel=msg->ballVel  ;
  state.awayPos=msg->awayPos ;
  state.homePos=msg->homePos;
  state.ballDetected=msg->ballDetected;
  state.homeDetected=msg->homeDetected;
  state.awayDetected=msg->awayDetected;
  state.our_bot_closest_to_ball=msg->our_bot_closest_to_ball;
  state.opp_bot_closest_to_ball=msg->opp_bot_closest_to_ball;
  state.our_goalie=msg->our_goalie;      
  state.opp_goalie=msg->opp_goalie;   
  state.opp_bot_marking_our_attacker=msg->opp_bot_marking_our_attacker;
  state.ball_at_corners=msg->ball_at_corners;
  state.ball_in_our_half=msg->ball_in_our_half;
  state.ball_in_our_possession=msg->ball_in_our_possession;
  
  // f<<"x: "<<msg->homePos[4].x<<" y: "<<msg->homePos[4].y<<"\n";
  // f<<"ball -- x: "<<msg->ballPos.x<<" y: "<<msg->ballPos.y<<"\n";
  cout<<"ball -- x: "<<msg->ballPos.x<<" y: "<<msg->ballPos.y<<"\n";
  cout<<"x: "<<msg->homePos[1].x<<" y: "<<msg->homePos[1].y<<endl;
  cout<<"awayDetected: "<<msg->awayDetected.size()<<" homeDetected: "<<msg->homeDetected.size()<<endl;
  // f.close();
  publishing();
  return;
}

PExec *pExec = NULL;

void publishing()
{
  ros::NodeHandle n;
  ros::Publisher tp0_pub = n.advertise<krssg_ssl_msgs::TacticPacket>("tactic_0", 1000);
  ros::Publisher tp1_pub = n.advertise<krssg_ssl_msgs::TacticPacket>("tactic_1", 1000);
  ros::Publisher tp2_pub = n.advertise<krssg_ssl_msgs::TacticPacket>("tactic_2", 1000);
  ros::Publisher tp3_pub = n.advertise<krssg_ssl_msgs::TacticPacket>("tactic_3", 1000);
  ros::Publisher tp4_pub = n.advertise<krssg_ssl_msgs::TacticPacket>("tactic_4", 1000);
  ros::Publisher tp5_pub = n.advertise<krssg_ssl_msgs::TacticPacket>("tactic_5", 1000);

  //****************************************************************
  krssg_ssl_msgs::TacticPacket tp0, tp1,tp2,tp3,tp4,tp5;
  Robot** robot;
  //****************************************************************
  if(pExec==NULL)
    pExec = new PExec(&state,n);
  
  fstream f;
  f.open("/home/gunjan/catkin_ws/src/play/playRunning.txt",fstream::out | fstream::app);
//  f<<"play terminated: "<<pExec->playTerminated(state)<<endl;
  f.close();

 if(pExec->playTerminated(state))
  {
    pExec->evaluatePlay();
    delete pExec;
    pExec = new PExec(&state,n);
    robot=pExec->selectPlay(state);
  }
  robot=pExec->executePlay(state);
  
  
  tp0.tID = std::string(robot[0]->tID);
  tp0.tParamJSON =robot[0]->tParamJSON;
  printf("Bot 0 %s %s \n",(tp0.tID).c_str(),(tp0.tParamJSON.c_str()));

  tp1.tID = std::string(robot[1]->tID);
  tp1.tParamJSON =robot[1]->tParamJSON; 
  printf("Bot 1 %s %s \n",tp1.tID.c_str(),(tp1.tParamJSON.c_str()));
  
  tp2.tID = std::string(robot[2]->tID);
  tp2.tParamJSON =robot[2]->tParamJSON;
  printf("Bot 2 %s %s\n",tp2.tID.c_str(),(tp2.tParamJSON.c_str()));
  
  tp3.tID = std::string(robot[3]->tID);
  tp3.tParamJSON =robot[3]->tParamJSON;
  printf("Bot 3 %s %s\n",tp3.tID.c_str(),(tp3.tParamJSON.c_str()));
  
  tp4.tID = std::string(robot[4]->tID);
  tp4.tParamJSON =robot[4]->tParamJSON;
  printf("Bot 4 %s %s\n",tp4.tID.c_str(),(tp4.tParamJSON.c_str()));
  
  tp5.tID = std::string(robot[5]->tID);
  tp5.tParamJSON =robot[5]->tParamJSON;
//  printf("Bot 5 %s %s\n",tp5.tID.c_str(),(tp5.tParamJSON.c_str()));

  tp0_pub.publish(tp0);
  tp1_pub.publish(tp1);
  tp2_pub.publish(tp2);
  tp3_pub.publish(tp3);
  tp4_pub.publish(tp4);
  tp5_pub.publish(tp5);  
}

int main(int argc, char  *argv[])
{
  // send a dummy TacticPacket
  ros::init(argc, argv, "play_node");
//  ros::init(argc, argv, "debug_node");

  ros::NodeHandle n;
  state_sub = n.subscribe("/belief_state", 1000, Callback);
  ros::spin();
  return 0;
}
