#include <list>
#include "tReceive.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <ssl_common/geometry.hpp>
#include <skills/skillSet.h>
#include "ros/ros.h"

#define THRES (0.8f)

namespace Strategy
{
  TReceive::TReceive(int botID) : Tactic( botID) { 
  } 

  
  TReceive::~TReceive() { } 

  bool TReceive::isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const {
    Vector2D<int> botPos(bs.homePos[botID].x, bs.homePos[botID].y);
    Vector2D<int> ballPos(bs.ballPos.x, bs.ballPos.y);
    float dist = Vector2D<int>::dist(botPos, receivePoint);
    float ballDist = Vector2D<int>::dist(botPos, ballPos);

    if(ballDist<1.6*DRIBBLER_BALL_THRESH && dist<2*BALL_RADIUS) 
      return true;
    else
      return false;
  }
  
  inline bool TReceive::isActiveTactic(void) const {
    return true;
  }

  int TReceive::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const {
    //HAS TO BE WRITTEN.
    int minv = *(freeBots.begin());
    float mindis = -1;
    for (std::list<int>::const_iterator it = freeBots.begin(); it != freeBots.end(); ++it)
    {
      Vector2D<int> pointPos(tParam.ReceiveP.x,tParam.ReceiveP.y);
      Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);

      float dis_from_ball = (pointPos - ballPos).absSq();
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
    
  gr_Robot_Command TReceive::execute(const BeliefState &state, const Tactic::Param& tParam) {
    
    Vector2D<int> receivePoint(tParam.ReceiveP.x, tParam.ReceiveP.y);
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
    float dist = Vector2D<int>::dist(botPos, receivePoint);
    float ballDist = Vector2D<int>::dist(botPos, ballPos);
    if(ballDist >= 4 * BOT_RADIUS) {
      if(dist > 3*BOT_RADIUS) {
        iState = GOTOPOINT;
      }
      else {
        if(dist <= 3*BOT_RADIUS) {
          iState = GOTOBALL;
        }
        else {
          iState = GOTOPOINT;
        }
      }
    }
    else if(ballDist >= 2 * BOT_RADIUS && abs(atan2((state.ballPos.y - state.homePos[botID].y) , (state.ballPos.x - state.homePos[botID].x) ))>0.3 ){
      iState = GOTOBALL;
    }
    else {
      iState = RECEIVEBALL;
    }

    Strategy::SkillSet::SkillID sID;
    SkillSet::SParam sParam;
    switch(iState) {
      case GOTOPOINT: 
      {
        sID = SkillSet::GoToPoint;
        sParam.GoToPointP.x = receivePoint.x;
        sParam.GoToPointP.y = receivePoint.y;
        sParam.GoToPointP.finalVelocity  = 0;
        sParam.GoToPointP.finalslope = Vector2D<int>::angle(ballPos, botPos);
        sParam.GoToPointP.align = false;
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        break;
      }
      case GOTOBALL: 
      {

        sID = SkillSet::GoToPoint;
        sParam.GoToPointP.x = state.ballPos.x;
        sParam.GoToPointP.y = state.ballPos.y;
        sParam.GoToPointP.finalVelocity  = 0;
        sParam.GoToPointP.finalslope = Vector2D<int>::angle(ballPos, botPos);
        sParam.GoToPointP.align = false;
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        break;
      }
      case RECEIVEBALL:
      {
        iState = FINISHED;
      }
    }
  }  

  Tactic::Param TReceive::paramFromJSON(string json) {
    using namespace rapidjson;
      Tactic::Param tParam;
      Document d;
      d.Parse(json.c_str());
      tParam.PassToPointP.x = d["x"].GetDouble();
      tParam.PassToPointP.y = d["y"].GetDouble();
    return tParam;
  }

  string TReceive::paramToJSON(Tactic::Param tParam) {
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
