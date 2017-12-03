#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include <ssl_common/geometry.hpp>

namespace Strategy
{
  gr_Robot_Command SkillSet::turnToAngle(const SParam &param, const BeliefState &state, int botID)
  {
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);

    float finalSlope = param.TurnToAngleP.finalslope;
    float turnAngleLeft = normalizeAngle(finalSlope - state.homePos[botID].theta); // Angle left to turn
    float omega = turnAngleLeft * MAX_BOT_OMEGA/(2*PI); // Speedup turn 
    if(omega < MIN_BOT_OMEGA/2 && omega > -MIN_BOT_OMEGA/2) { // This is a rare used skill so believe in Accuracy more than speed. Hence reducing minimum Omega
      if(omega < 0) omega = -MIN_BOT_OMEGA/2;
      else omega = MIN_BOT_OMEGA/2;
    }
    float dist = Vector2D<int>::dist(ballPos, botPos);
    if(dist < DRIBBLER_BALL_THRESH){
      return getRobotCommandMessage(botID, 0, 0, omega, 0, true);
    }
    else{
      return getRobotCommandMessage(botID, 0, 0, omega, 0, false);
    }
  }
}
