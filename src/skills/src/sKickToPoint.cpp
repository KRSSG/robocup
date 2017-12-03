#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/geometry.hpp>
#include <ssl_common/config.h>

namespace Strategy
{
  gr_Robot_Command SkillSet::kickToPoint(const SParam &param, const BeliefState &state, int botID)
  {
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
    Vector2D<int> destPoint(param.KickToPointP.x, param.KickToPointP.y);
    float finalSlope = Vector2D<int>::angle(destPoint, botPos);
    float turnAngleLeft = normalizeAngle(finalSlope - state.homePos[botID].theta); // Angle left to turn
    float dist = Vector2D<int>::dist(ballPos, botPos);
    if(dist > BOT_BALL_THRESH)
    {
      //printf("going to ball %d, %f\n", BOT_BALL_THRESH, dist);
      return goToBall(param, state, botID);
    }
    if(fabs(turnAngleLeft) > SATISFIABLE_THETA/2)
    {
      SkillSet::SParam paramt;
      paramt.TurnToPointP.x = destPoint.x;
      paramt.TurnToPointP.y = destPoint.y;
      paramt.TurnToPointP.max_omega = MAX_BOT_OMEGA*3;
      return turnToPoint(paramt, state, botID);
    }
    //printf("should kick now %f\n", turnAngleLeft);
    return getRobotCommandMessage(botID, 0, 0, 0, param.KickToPointP.power, false);
  }
}
