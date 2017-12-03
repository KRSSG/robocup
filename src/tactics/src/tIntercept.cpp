#include <list>
#include "tIntercept.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include <stdio.h>
#include <ssl_common/geometry.hpp>
#include <skills/skillSet.h>
#include <vector>
#include <fstream>

#define TEAMSIZE (5)

namespace Strategy {

  TIntercept::TIntercept(int botID) : Tactic(botID) {
  }

  TIntercept::~TIntercept() { } 

  bool TIntercept::isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const {
    return false;
  }
 
  bool TIntercept::isActiveTactic(void) const {
    return true;
  }

  int TIntercept::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const
  {
      int markBotID = tParam.InterceptP.awayBotID;
      int minv   = *(freeBots.begin());
      int mindis = -1.0f;
      Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
      for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {
        for(int bID = 0; bID < TEAMSIZE; bID++) {
          Vector2D<int> oppBot(state.awayPos[bID].x, state.awayPos[bID].y);
          float opp_dis_from_ball = Vector2D<int>::dist(oppBot, ballPos);
          if(mindis < 0.0 || mindis > opp_dis_from_ball) {
            mindis = opp_dis_from_ball;
            minv = bID;
          }
        }
        int passerBotID = minv;
        Vector2D<int> passerBot(state.awayPos[passerBotID].x, state.awayPos[passerBotID].y);
        Vector2D<int> receiverBot(state.awayPos[markBotID].x, state.awayPos[markBotID].y);
        Vector2D<int> homePos(state.homePos[*it].x, state.homePos[*it].y);
        
        /*float dis_from_point = (homePos - ballPos).absSq();
        if(*it == prevID)
          dis_from_point -= HYSTERESIS;
        if(dis_from_point < mindis)
        {
          mindis = dis_from_point;
          minv = *it;
        }
      }
      printf("%d assigned Position\n", minv);
      return minv;*/
    } // chooseBestBot
  }

    gr_Robot_Command TIntercept::execute(const BeliefState &state, const Param& tParam) {
      
      int markBotID = tParam.InterceptP.awayBotID;
      Vector2D<int> markBot(state.homePos[markBotID].x, state.homePos[markBotID].y);
      Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
      Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
      int passerBotID=-1;int mindis = -1.0f;
        for(int i = 0; i < TEAMSIZE; i++) {
          if(i != botID && i != markBotID) {
            Vector2D<int> bot(state.homePos[i].x, state.homePos[i].y);
            float dis = Vector2D<int>::dist(bot, ballPos);
            if(0.0 > mindis) {
              mindis = dis; passerBotID = i;
            }
            else if(dis < mindis) {
              mindis = dis; passerBotID = i;
            }
          }
        }
      Vector2D<int> passerBot(state.homePos[passerBotID].x, state.homePos[passerBotID].y);
      //goal_point is the point which will be blocked by the intercepter
      Vector2D<int> goal_point(OUR_GOAL_X, 0.0f);
      Vector2D<int> destination; float destinationSlope;
      float ballFromBot = Vector2D<int>::dist(ballPos, botPos);
      float ballFromPasser = Vector2D<int>::dist(ballPos, passerBot);
      float ballFromMark = Vector2D<int>::dist(ballPos, markBot);
      float ballFromOpp = ballFromPasser > ballFromMark ? ballFromMark/1.0f : ballFromPasser /1.0f;

      Strategy::SkillSet::SkillID sID;
      SkillSet::SParam sParam;
      if(ballFromOpp < ballFromBot) {

        //where == 0 corresponds to intercepting the pass
        if(tParam.InterceptP.where == 0){
          destination = 0.7 * markBot + 0.3 * passerBot;
        }
        else{
          //intercept the goal shot
          destination = 0.7 * markBot + 0.3 * goal_point;
        }
        destinationSlope = Vector2D<int>::angle(passerBot,botPos);
        
        float pointFromBot = Vector2D<int>::dist(destination, botPos);
        if(pointFromBot > BOT_BALL_THRESH) {
          iState = GOTOPOSITION;
        }
        else {
          if(pointFromBot <= DRIBBLER_BALL_THRESH) {
            iState = GOTOBALL;
          }
          else {
            iState = GOTOPOSITION;
          }
        }
      } 
      else {
        if(ballFromBot > DRIBBLER_BALL_THRESH) {
          iState = GOTOBALL;
        }
        else {
          //iState = CLEAR;
        }
      }
      switch(iState) {
        case GOTOPOSITION:
        {
          sID = SkillSet::GoToPoint;
          sParam.GoToPointP.x = destination.x;
          sParam.GoToPointP.y = destination.y;
          sParam.GoToPointP.finalVelocity = MAX_BOT_SPEED;
          sParam.GoToPointP.finalslope = destinationSlope;
          sParam.GoToPointP.align = false;
          return SkillSet::instance()->executeSkill(sID, sParam, state, botID);    
        }
        case GOTOBALL:
        {
          sID = SkillSet::GoToBall;
          return SkillSet::instance()->executeSkill(sID, sParam, state, botID);     
        }
        case CLEAR:
        {
          sID = SkillSet::Kick;
          sParam.KickP.power = 10.0f;
          return SkillSet::instance()->executeSkill(sID, sParam, state, botID); 
        }
      }
      
    }
    
    Tactic::Param TIntercept::paramFromJSON(string json) {
      using namespace rapidjson;
      Tactic::Param tParam;
      Document d;
      d.Parse(json.c_str());
      tParam.InterceptP.awayBotID = d["awayBotID"].GetDouble();
      tParam.InterceptP.where = d["where"].GetDouble();
      return tParam;

    }

    string TIntercept::paramToJSON(Tactic::Param tParam) {
      using namespace rapidjson;
      StringBuffer buffer;
      Writer<StringBuffer> w(buffer);
      w.StartObject();
      w.String("awayBotID");
      w.Double(tParam.InterceptP.awayBotID);
      w.String("where");
      w.Double(tParam.InterceptP.where);
      w.EndObject();
      return buffer.GetString();
    }
  
  }