#include <iostream>
#include <cstring>
#include <ctime>
#include "krssg_ssl_msgs/BeliefState.h"
#include <stdio.h>
#include <ssl_common/config.h>
#include <cmath>
#include "ros/ros.h"
#include <krssg_ssl_msgs/TacticPacket.h>
#include <tactics/tactic_factory.h>
#include <string>
#include <ssl_common/geometry.hpp>

using namespace Strategy;

int flag0=-1,flag1=-1;
krssg_ssl_msgs::TacticPacket tp0,tp1,tp2,tp3,tp4;
ros::Publisher tp1_pub, tp2_pub, tp0_pub,tp3_pub,tp4_pub,tp5_pub;
Tactic::Param tParam;

void Callback(const krssg_ssl_msgs::BeliefState::ConstPtr& msg){
  Vector2D<int> ballPos(msg->ballPos.x, msg->ballPos.y);
  Vector2D<int> botPos_1(msg->homePos[1].x, msg->homePos[1].y);
  Vector2D<int> botPos_2(msg->homePos[2].x, msg->homePos[2].y);
  Vector2D<int> botPos_3(msg->homePos[3].x, msg->homePos[3].y);
  
  float bot1_ball = Vector2D<int>::dist(ballPos, botPos_1);
  float bot2_ball = Vector2D<int>::dist(ballPos, botPos_2);
  float bot3_ball = Vector2D<int>::dist(ballPos, botPos_3);
  
  float threshold = 20 * BOT_BALL_THRESH;
  float delta = 3 * BOT_BALL_THRESH; 

  Vector2D<int> PassPos;
tp3.tID = std::string("TPosition");
tParam.PositionP.x= msg->ballPos.x;
tParam.PositionP.y= msg->ballPos.y;
    tp3.tParamJSON = TacticFactory::instance()->Create("TPosition", 3)->paramToJSON(tParam);    
tp1.tID = std::string("TPosition");
    tp1.tParamJSON = TacticFactory::instance()->Create("TPosition", 1)->paramToJSON(tParam);     
tp2.tID = std::string("TPosition");
    tp2.tParamJSON = TacticFactory::instance()->Create("TPosition", 2)->paramToJSON(tParam);    

    // cout<<"[1] PassToPointP.x : "<<tParam.PassToPointP.x<<"PassToPointP.y : "<<tParam.PassToPointP.y<<endl;
  /*if (ballPos.x > (2/3)*HALF_FIELD_MAXX){
    PassPos.x = HALF_FIELD_MAXX/ 2;
    PassPos.y = 0;
  }
  else {
    PassPos.x = (2/3) * HALF_FIELD_MAXX;
    PassPos.y = - HALF_FIELD_MAXY / 2;
  }

// always goalie
std::cout<<"Bot 2 : Goalie"<<std::endl;
tp2.tID=std::string("TGoalie");
tp2.tParamJSON = TacticFactory::instance()->Create("TGoalie", 2)->paramToJSON(tParam);

// kick to goal
// cout<<"ballpos.x : "<<ballPos.x <<"\thalf field ======== "<<float(2.0/3.0)*HALF_FIELD_MAXX<<endl;
// cout<< " 1 distance with ball :: " << bot1_ball << endl;
// cout << " 3 distance with ball :: " << bot3_ball << endl ;
if(ballPos.x >= float(2.0/3.0)*HALF_FIELD_MAXX){
  if(bot1_ball < bot3_ball){
    std::cout<<"Bot 1 : kick to goal"<<std::endl;
    tp1.tID = std::string("TKickToGoal");
    tp1.tParamJSON = TacticFactory::instance()->Create("TKickToGoal", 3)->paramToJSON(tParam);    

    std::cout<<"Bot 3 : stop"<<std::endl; 
    tParam.PositionP.x = float(2.0/3.0)*HALF_FIELD_MAXX;
    tParam.PositionP.y = -botPos_1.y;
    tp3.tID=std::string("TPosition");
    tp3.tParamJSON = TacticFactory::instance()->Create("TPosition", 1)->paramToJSON(tParam);
  }
  else {
    std::cout<<"Bot 1 : stop"<<std::endl;
    tParam.PositionP.x = float(2.0/3.0)*HALF_FIELD_MAXX;
    tParam.PositionP.y = -botPos_3.y;
    tp1.tID=std::string("TPosition");
    tp1.tParamJSON = TacticFactory::instance()->Create("TPosition", 3)->paramToJSON(tParam);         

    std::cout<<"Bot 3 : kick to goal"<<std::endl;
    tp3.tID = std::string("TKickToGoal");
    tp3.tParamJSON = TacticFactory::instance()->Create("TKickToGoal", 1)->paramToJSON(tParam);    
  }
}
// pass to point
else{
  if(bot1_ball < bot3_ball){
    std::cout<<"Bot 1 : pass to point"<<std::endl;
    tParam.PassToPointP.x=botPos_3.x;
    // tParam.PassToPointP.x=float(2.0/3.0)*HALF_FIELD_MAXX+delta;
    tParam.PassToPointP.y= botPos_3.y;
    // tParam.PassToPointP.y= HALF_FIELD_MAXY / 2.0;
    tp1.tID = std::string("TPassToPoint");
    tp1.tParamJSON = TacticFactory::instance()->Create("TPassToPoint", 3)->paramToJSON(tParam);    
    cout<<"[1] PassToPointP.x : "<<tParam.PassToPointP.x<<"PassToPointP.y : "<<tParam.PassToPointP.y<<endl;

    // std::cout<<"Bot 3 : Receive"<<std::endl;
    // // tParam.ReceiveP.x = botPos_3.x;
    //  tParam.ReceiveP.x = float(2.0/3.0)*HALF_FIELD_MAXX+delta;
    // //tParam.ReceiveP.y = botPos_3.y;
    //  tParam.ReceiveP.y = HALF_FIELD_MAXY / 2.0;
    // tp3.tID=std::string("TReceive");
    // tp3.tParamJSON = TacticFactory::instance()->Create("TReceive", 1)->paramToJSON(tParam);
    // cout<<"[3] ReceiveP.x : "<<;

  }
  else{
    std::cout<<"Bot 3 : Pass To Point"<<std::endl;
    // tParam.PassToPointP.x=float(2.0/3.0)*HALF_FIELD_MAXX+delta;
    tParam.PassToPointP.x=botPos_1.x;
    // tParam.PassToPointP.y= HALF_FIELD_MAXY / 2.0;
    tParam.PassToPointP.y= botPos_1.y;
    tp3.tID = std::string("TPassToPoint");
    tp3.tParamJSON = TacticFactory::instance()->Create("TPassToPoint", 1)->paramToJSON(tParam);      
    cout<<"[3] PassToPointP.x : "<<tParam.PassToPointP.x<<"PassToPointP.y : "<<tParam.PassToPointP.y<<endl;



   //  std::cout<<"Bot 1 : Receive"<<std::endl;
   //  tParam.ReceiveP.x = float(2.0/3.0)*HALF_FIELD_MAXX+delta;
   // // tParam.ReceiveP.x = botPos_1.x;
   //   tParam.ReceiveP.y = HALF_FIELD_MAXY / 2.0;
   //  // tParam.ReceiveP.y = botPos_1.y;
   //  tp1.tID=std::string("TReceive");
   //  tp1.tParamJSON = TacticFactory::instance()->Create("TReceive", 3)->paramToJSON(tParam);    
  }*/
// }
  
  //   tp1.tID = std::string("TGoalie");
  //   tp1.tParamJSON = TacticFactory::instance()->Create("TGoalie", 1)->paramToJSON(tParam);   

  //   tp2.tID = std::string("TKickToGoal");
  //   tp2.tParamJSON = TacticFactory::instance()->Create("TKickToGoal", 2)->paramToJSON(tParam);    

  //   tp3.tID = std::string("TKickToGoal");
  //   tp3.tParamJSON = TacticFactory::instance()->Create("TKickToGoal", 3)->paramToJSON(tParam);    

  // std::cout<<"\n\n\n"<<std::endl;
  tp2_pub.publish(tp2);
  tp1_pub.publish(tp1);
  tp3_pub.publish(tp3);
}


int main(int argc, char  *argv[])
{
  // send a dummy TacticPacket
  ros::init(argc, argv, "test_play_node");
  ros::NodeHandle n;
  
  tp0_pub = n.advertise<krssg_ssl_msgs::TacticPacket>("tactic_0", 1000);
  tp1_pub = n.advertise<krssg_ssl_msgs::TacticPacket>("tactic_1", 1000);
  tp2_pub = n.advertise<krssg_ssl_msgs::TacticPacket>("tactic_2", 1000);
  tp3_pub = n.advertise<krssg_ssl_msgs::TacticPacket>("tactic_3", 1000);
  tp4_pub = n.advertise<krssg_ssl_msgs::TacticPacket>("tactic_4", 1000);
  ros::Subscriber bs=n.subscribe("/belief_state",1000,Callback);
  
    // tp1.tID = std::string("TGoalie");
    // tp1.tParamJSON = TacticFactory::instance()->Create("TGoalie", 1)->paramToJSON(tParam);   

    // tp2.tID = std::string("TKickToGoal");
    // tp2.tParamJSON = TacticFactory::instance()->Create("TKickToGoal", 2)->paramToJSON(tParam);    

    // tp3.tID = std::string("TKickToGoal");
    // tp3.tParamJSON = TacticFactory::instance()->Create("TKickToGoal", 3)->paramToJSON(tParam);    

  ros::spin();
  return 0;
}