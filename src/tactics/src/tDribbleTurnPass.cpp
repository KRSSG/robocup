#include <list>
#include "tDribbleTurnPass.h"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <fstream>
#include <stdio.h>
#include <ssl_common/geometry.hpp>
#include <skills/skillSet.h>

#define THRES (0.8f)

namespace Strategy
{
  TDribbleTurnPass::TDribbleTurnPass(int botID) : Tactic( botID) { 
   
    //point = Vector2D<int>(HALF_FIELD_MAXX / 2.0f , HALF_FIELD_MAXt / 2.0f);
  }

  TDribbleTurnPass::~TDribbleTurnPass() { } 

  bool TDribbleTurnPass::isCompleted(const BeliefState &bs,const Tactic::Param& tParam) const {
 /*   fstream file;
    file.open("/home/animesh/Documents/isCompleted.txt",fstream::out|fstream::app);
    file<<"isCompleted: "<<(iState==FINISHED)<<"\n";
    file.close();

  */
    return false;
    return iState == FINISHED;
  }
  
  inline bool TDribbleTurnPass::isActiveTactic(void) const {
 /*   fstream file;
    file.open("/home/animesh/Documents/isActiveTactic.txt",fstream::out|fstream::app);
    file<<"isActiveTactic: "<<(iState!=FINISHED)<<"\n";
    file.close();

   */
    return true;
    return iState != FINISHED;
  }

  int TDribbleTurnPass::chooseBestBot(const BeliefState &state, std::list<int>& freeBots, const Param& tParam, int prevID) const {
 /*   fstream file;
    file.open("/home/animesh/Documents/isActiveTactic.txt",fstream::out|fstream::app);
    file<<"isActiveTactic: "<<(iState!=FINISHED)<<"\n";
    file.close();
  */ /* int minv = *(freeBots.begin());
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
    return minv;*/
    return 3;
  }

  gr_Robot_Command TDribbleTurnPass::execute(const BeliefState &state, const Tactic::Param& tParam) {
    
   // botID=1;
    Vector2D<int> point(tParam.DribbleTurnPassP.x, tParam.DribbleTurnPassP.y);
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    
    float dist = Vector2D<int>::dist(ballPos, botPos);
 //   float finalSlope = Vector2D<int>::angle(point, ballPos);
    float angleToTurn = normalizeAngle(state.homePos[botID].theta - Vector2D<int>::angle(point, ballPos));

    float pointDis = Vector2D<int>::dist(botPos, point);
    float goalBotAngle = Vector2D<int>::angle(point, botPos);
    float ballBotAngle = Vector2D<int>::angle(ballPos, botPos);
    float angle = Vector2D<int>::angle(point, ballPos);
    float delta;
    if(pointDis)
      delta = 0.085;/*3*asin(BOT_RADIUS / (10.0 * pointDis))*/
    else delta = 3.0 / 4.0;
    float angleUp = angle + delta;
    float angleDown = angle - delta;

    Strategy::SkillSet::SkillID sID;
    SkillSet::SParam sParam;

    if (dist >= DRIBBLER_BALL_THRESH) {
      iState=GOTOBALL;
    }
    
    else if (abs(angle - ballBotAngle) > delta ){
    // else if (abs(angle - ballBotAngle) > delta || (point.x - ballPos.x)*(ballPos.x - botPos.x) < 0
      iState=DRIBBLETURN;
    }
    else {
      iState=PASSING;
    }  
    
    
    switch(iState)
    {
      case GOTOBALL:
      {
        sID = SkillSet::GoToPoint;
        sParam.GoToPointP.x             = state.ballPos.x ;
        sParam.GoToPointP.y             = state.ballPos.y ;
          // sParam.GoToPointP.align         = tParam.PositionP.align;
        sParam.GoToPointP.finalslope    = Vector2D<int>::angle(ballPos, botPos) ;
        sParam.GoToPointP.finalVelocity = 0;
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        break;
      }
      case DRIBBLETURN:
      {
        sID = SkillSet::DribbleTurn;
        sParam.DribbleTurnP.x = HALF_FIELD_MAXX;
        sParam.DribbleTurnP.y = 0;
        sParam.DribbleTurnP.max_omega = MAX_BOT_OMEGA/3.0;
        sParam.DribbleTurnP.turn_radius = 12.0*BOT_RADIUS;
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        break;
      }  
      case PASSING:
      {
        float power = pointDis / (HALF_FIELD_MAXX / 8.0);
        power = power > 6.0 ? 6.0 : power;
        power = power < 3.0 ? 3.0 : power;
        sID = SkillSet::Kick;
        sParam.KickP.power = power; 
        iState = FINISHED;
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);
        break;
      }   /*     
        case GOTOBALL: case TURNING: case PASSING:
          sID=SkillSet::TurnToPoint;
          sParam.TurnToPointP.x=point.x;
          sParam.TurnToPointP.y=point.y;
          sParam.TurnToPointP.max_omega = MAX_BOT_OMEGA;
          return SkillSet::instance()->executeSkill(sID,sParam,state,botID);
          */
    }
  }  

  Tactic::Param TDribbleTurnPass::paramFromJSON(string json) {
    using namespace rapidjson;
      Tactic::Param tParam;
      Document d;
      d.Parse(json.c_str());
      tParam.DribbleTurnPassP.x = d["x"].GetDouble();
      tParam.DribbleTurnPassP.y = d["y"].GetDouble();
      return tParam;
  }

  string TDribbleTurnPass::paramToJSON(Tactic::Param tParam) {
    using namespace rapidjson;
      StringBuffer buffer;
      Writer<StringBuffer> w(buffer);
      w.StartObject();
      w.String("x");
      w.Double(tParam.DribbleTurnPassP.x);
      w.String("y");
      w.Double(tParam.DribbleTurnPassP.y);
      w.EndObject();
      return buffer.GetString();
  }
    
} 
