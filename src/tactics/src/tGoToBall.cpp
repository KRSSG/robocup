#include <list>
#include "tGoToBall.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ssl_common/geometry.hpp>
#include <skills/skillSet.h>

namespace Strategy
{
  	TGoToBall::TGoToBall(int botID) :
      Tactic(botID)
    { 
      
    } 
    TGoToBall::~TGoToBall()
    { }

    bool TGoToBall::isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const 
    {
      Vector2D<int> botpos(bs.homePos[botID].x,bs.homePos[botID].y);
      Vector2D<int> ballPos(bs.ballPos.x, bs.ballPos.y);
      float dis_from_point = sqrt((botpos - ballPos).absSq());
      float angle = normalizeAngle(Vector2D<int>::angle(ballPos, botpos) - (bs.homePos[botID].theta));
      if(dis_from_point<3*BOT_POINT_THRESH && abs(angle) < 0.2)
        return true;
      return false;
    }
    inline bool TGoToBall::isActiveTactic(void) const
    {
      return true;
    }

    int TGoToBall::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const
    {
        return 3;

    	  int minv = *(freeBots.begin());
      	int mindis = 10000;
      	for (std::list<int>::const_iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      	{
        // TODO make the bot choosing process more sophisticated, the logic below returns the 1st available bot
        	Vector2D<int> homePos(state.homePos[*it].x, state.homePos[*it].y);
        	Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);

        	float dis_from_ball = (homePos - ballPos).absSq();
        	if(dis_from_ball < mindis)
        	{
          		dis_from_ball = mindis;
          		minv = *it;
        	}
      	}
        return 1;
      	return minv;
    }

    gr_Robot_Command TGoToBall::execute(const BeliefState &state, const Tactic::Param& tParam)
    {
        int dist = 0;
        Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
        Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
        dist = Vector2D<int>::dist(ballPos, botPos);
        SkillSet::SParam sParam;

        float dis_from_point = sqrt((botPos - ballPos).absSq());
        float angle = normalizeAngle(Vector2D<int>::angle(ballPos, botPos) - (state.homePos[botID].theta));

        // std::fstream f;
        // f.open("/home/kgpkubs/Desktop/angle_debug.txt",std::ios::out|std::ios::app);
        // cout<<"Distance from ball: "<< dis_from_point <<" Angle left: "<< angle <<"\n";
        
        if(dist < 4*DRIBBLER_BALL_THRESH && abs(angle)>0.35)
        {
          cout << "TurnToPoint\n";
          //Strategy::SkillSet::SkillID sID = SkillSet::Dribble;
          Strategy::SkillSet::SkillID sID = SkillSet::TurnToPoint;
          Strategy::SkillSet *ptr = SkillSet::instance();
          sParam.TurnToPointP.x = state.ballPos.x ;
          sParam.TurnToPointP.y = state.ballPos.y ;
          sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA;
          //Strategy::SkillSet::SkillID sID = SkillSet::Kick;
          //sParam.KickP.power = 7.0f;
          //iState = FINISHED;
          //return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
          return ptr->executeSkill(sID, sParam, state, botID);
        }
    	  else 
        {
          cout << "GoToPoint\n";
          Strategy::SkillSet::SkillID sID = SkillSet::GoToPoint;
	        SkillSet::SParam sParam;
          sParam.GoToPointP.x             = state.ballPos.x ;
          sParam.GoToPointP.y             = state.ballPos.y ;
          // sParam.GoToPointP.align         = tParam.PositionP.align;
          sParam.GoToPointP.finalslope    = Vector2D<int>::angle(ballPos, botPos) ;
          sParam.GoToPointP.finalVelocity = 0;

          // sParam.GoToBallP.intercept = tParam.GoToBallP.intercept ;
      	  // Execute the selected skill
      	  Strategy::SkillSet *ptr = SkillSet::instance();
      	  return ptr->executeSkill(sID, sParam, state, botID);
        }
        
    }

    Tactic::Param TGoToBall::paramFromJSON(string json) {
      using namespace rapidjson;
      Tactic::Param tParam;      
      return tParam;
    }

    string TGoToBall::paramToJSON(Tactic::Param tParam) {
      return string("");
    }
} //namespace Strategy