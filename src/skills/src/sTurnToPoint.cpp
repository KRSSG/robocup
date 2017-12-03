#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>
#include <cstdio>
#include <vector>
#include <iostream>

#include <ssl_common/geometry.hpp>
namespace Strategy
{
  gr_Robot_Command SkillSet::turnToPoint(const SParam &param, const BeliefState &state, int botID)
  {
    Vector2D<int> point(param.TurnToPointP.x, param.TurnToPointP.y);
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);

    float finalSlope = Vector2D<int>::angle(point, botPos);
    float turnAngleLeft = normalizeAngle(finalSlope - state.homePos[botID].theta); // Angle left to turn
    float omega = 1.5*turnAngleLeft * param.TurnToPointP.max_omega / (2 * PI); // Speedup turn
    if(omega < MIN_BOT_OMEGA && omega > -MIN_BOT_OMEGA)
    {
      if(omega < 0) omega = -MIN_BOT_OMEGA;
      else omega = MIN_BOT_OMEGA;
    }
    float v_x = omega*BOT_BALL_THRESH*1.5;
    // comm.addCircle(state->homePos[botID].x,  state->homePos[botID].y, 50);
    float dist = Vector2D<int>::dist(ballPos, botPos);
    if(dist < DRIBBLER_BALL_THRESH*4)
    {
      std::cout<<"In dribbler on for ttp"<<std::endl;
      return getRobotCommandMessage(botID, v_x, 0, omega, 0, true);
    }
    else
    {
      std::cout<<"In dribbler off for ttp"<<std::endl;
      return getRobotCommandMessage(botID, 0, 0, omega, 0, true);
    }
     // return getRobotCommandMessage(botID,0,MAX_BOT_SPEED/2,0,0,true);
  }
}
