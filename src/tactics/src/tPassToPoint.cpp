#include <list>
#include "tPassToPoint.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>

#include <fstream>
#include <stdio.h>
#include <ssl_common/geometry.hpp>
#include <skills/skillSet.h>
#include "ros/ros.h"


#define THRES (0.8f)

namespace Strategy
{
  TPassToPoint::TPassToPoint(int botID) : Tactic( botID) { 
   iState=GOTOBALL;
  }

  TPassToPoint::~TPassToPoint() { } 

  bool TPassToPoint::isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const {
    if(iState == FINISHED) 
    {
      fstream f;
      f.open("/home/gunjan/catkin_ws/src/play/iState.txt",fstream::out| fstream::app);
      f.close();
      return true;
    }
    else 
      {
       fstream f;
        f.open("/home/gunjan/catkin_ws/src/play/iState.txt",fstream::out| fstream::app);
        f.close();
        return false;
      }
  }
  
  inline bool TPassToPoint::isActiveTactic(void) const {
    return true;
  }

  int TPassToPoint::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const {
  
    int minv = *(freeBots.begin());
    float mindis = -1;
    for (std::list<int>::const_iterator it = freeBots.begin(); it != freeBots.end(); ++it)
    {
      Vector2D<int> homePos(state.homePos[*it].x, state.homePos[*it].y);
      Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);

      float dis_from_ball = (homePos - ballPos).absSq();
      if (mindis < 0) {
        mindis = dis_from_ball;
        minv = *it; 
      }
      else if(dis_from_ball < mindis) {
        mindis = dis_from_ball;
        minv = *it;
      }
    }
    assert(mindis >= 0.0f);
    return minv;
  }

  gr_Robot_Command TPassToPoint::execute(const BeliefState &state, const Tactic::Param& tParam) {
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    Vector2D<int> oppPos(state.homePos[1].x, state.homePos[1].y);
    float dist = Vector2D<int>::dist(ballPos, botPos);
    Strategy::SkillSet::SkillID sID;
    SkillSet::SParam sParam;

    float angle = normalizeAngle(Vector2D<int>::angle(oppPos, botPos) - (state.homePos[botID].theta));

    if(ballPos.x>HALF_FIELD_MAXX/2){
      cout<<"passed - repositioning"<<endl;
      
      Strategy::SkillSet::SkillID sID = SkillSet::GoToPoint;
      SkillSet::SParam sParam;
      sParam.GoToPointP.x             = HALF_FIELD_MAXX/3 ;
      sParam.GoToPointP.y             = -HALF_FIELD_MAXX/4 ;
      sParam.GoToPointP.finalslope    = Vector2D<int>::angle(oppPos, botPos) ;
      sParam.GoToPointP.finalVelocity = 0;
      Strategy::SkillSet *ptr = SkillSet::instance();
      return ptr->executeSkill(sID, sParam, state, botID);
    }

    cout<< "dist: " << dist<< "angle: "<< angle<< endl;
    if(dist < 5*BOT_RADIUS){
      cout<<"1"<<endl;
        if(abs(angle)>0.5){
          cout<<"3"<<endl;
          sID = SkillSet::TurnToPoint;
          sParam.TurnToPointP.x = state.homePos[1].x;
          sParam.TurnToPointP.y = state.homePos[1].y;
          sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA;
          return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        }
        else{
          cout<<"5"<<endl;
          Strategy::SkillSet::SkillID sID = SkillSet::Kick;
          sParam.KickP.power = 7.0f;
          iState = FINISHED;
          return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        }

      
    }
    else{
      cout<<"6"<<endl;
      sID = SkillSet::Stop;
      return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
    }



//     Vector2D<int> reveiveP(tParam.PassToPointP.x, tParam.PassToPointP.y);
//     Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
//     Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);

//     float Pr_angle = normalizeAngle(Vector2D<int>::angle(reveiveP, botPos) - state.homePos[botID].theta);
//     float dist = Vector2D<int>::dist(ballPos, botPos);
  
//     cout << "Distance from ball: " << dist << " DRIBBLER_BALL_THRESH : "<< 0.2*DRIBBLER_BALL_THRESH<< endl;
//     float angleThresh = 0.05;
//     if (dist > 1*DRIBBLER_BALL_THRESH && 0) {
//       iState=GOTOBALL;
//       cout<<"GOTOBALL\n";
//     }
//     else if(abs(Pr_angle) >= angleThresh) {
//       iState = TURNING;
//       cout<<"TURNING\n";
//     }    
//     else {       
//           iState=KICKING;
//           cout<<"KICKING\n";
//     }  

//     Strategy::SkillSet::SkillID sID;
//     SkillSet::SParam sParam;
    

//     cout<<"anlge : "<<abs(Pr_angle)<<"\n\n\n\t------------Thresh : "<<angleThresh<<"---------\n\n\n\n";
    
//     switch(iState)
//     {
//       case GOTOBALL:
//       {
//         float dis_from_point = sqrt((botPos - ballPos).absSq());
//         float angle = normalizeAngle(Vector2D<int>::angle(ballPos, botPos) - (state.homePos[botID].theta));

//         // cout<<"Distance from ball: "<< dis_from_point <<" Angle left: "<< angle <<"\n";
        
//         if(dist < 4*DRIBBLER_BALL_THRESH && abs(angle)>1)
//         {
//           // cout << "TurnToPoint\n";
//           Strategy::SkillSet::SkillID sID = SkillSet::TurnToPoint;
//           Strategy::SkillSet *ptr = SkillSet::instance();
//           sParam.TurnToPointP.x = state.ballPos.x ;
//           sParam.TurnToPointP.y = state.ballPos.y ;
//           sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA;
//           return ptr->executeSkill(sID, sParam, state, botID);
//         }
//         else 
//         {
//           // cout << "GoToPoint\n";
//           Strategy::SkillSet::SkillID sID = SkillSet::GoToPoint;
//           SkillSet::SParam sParam;
//           sParam.GoToPointP.x             = state.ballPos.x ;
//           sParam.GoToPointP.y             = state.ballPos.y ;
//           sParam.GoToPointP.finalslope    = Vector2D<int>::angle(ballPos, botPos) ;
//           sParam.GoToPointP.finalVelocity = 0;

//           Strategy::SkillSet *ptr = SkillSet::instance();
//           return ptr->executeSkill(sID, sParam, state, botID);
//         }
//       }
//       case TURNING:
//       {

//         sID = SkillSet::TurnToPoint;
//         sParam.TurnToPointP.x = reveiveP.x;
//         sParam.TurnToPointP.y = reveiveP.y;
//         sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA;
//         return SkillSet::instance()->executeSkill(sID, sParam, state, botID);

// /*        sID = SkillSet::DribbleTurn;
//         sParam.DribbleTurnP.x = reveiveP.x;
//         sParam.DribbleTurnP.y = reveiveP.y;
//         sParam.DribbleTurnP.max_omega = MAX_BOT_OMEGA/3.0;
//         sParam.DribbleTurnP.turn_radius = 12.0*BOT_RADIUS;
//         return SkillSet::instance()->executeSkill(sID, sParam, state, botID);*/
//         break;
//       }  
//       case KICKING:
//       {
//         // cout << "------------KICKING---------------";
//         sID = SkillSet::Kick;
//         sParam.KickP.power = 7.0f;
//         iState = FINISHED;
//         return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
//         break;
//       }        

//     } 


  }
    
  

  Tactic::Param TPassToPoint::paramFromJSON(string json) {
    using namespace rapidjson;
      Tactic::Param tParam;
      Document d;
      d.Parse(json.c_str());
      tParam.PassToPointP.x = d["x"].GetDouble();
      tParam.PassToPointP.y = d["y"].GetDouble();

      return tParam;
 }

  string TPassToPoint::paramToJSON(Tactic::Param tParam) {
    using namespace rapidjson;
      StringBuffer buffer;
      Writer<StringBuffer> w(buffer);
      w.StartObject();
      w.String("x");
      w.Double(tParam.PassToPointP.x);
      w.String("y");
      w.Double(tParam.PassToPointP.y);
      w.EndObject();
      return buffer.GetString();
  }    
}

