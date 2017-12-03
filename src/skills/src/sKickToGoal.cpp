#include "skillSet.h"
#include "beliefState.h"
#include "fieldConfig.h"

namespace Strategy
{
  void SkillSet::kickToGoal(const SParam &param)
  {
    Vector2D<int> goal(HALF_FIELD_MAXX, 0);	
    float finalSlope = Vector2D<int>::angle(goal, state->homePos[botID]);
    float turnAngleLeft = normalizeAngle(finalSlope - state->homeAngle[botID]); // Angle left to turn
    float dist = Vector2D<int>::dist(state->ballPos, state->homePos[botID]);
    if(dist > BOT_BALL_THRESH)
    {
      //printf("going to ball %d, %f\n", BOT_BALL_THRESH, dist);
      goToBall(param);
      return;
    }
    if(fabs(turnAngleLeft) > 2 * SATISFIABLE_THETA)
    {
      SkillSet::SParam paramt;
      paramt.TurnToPointP.x = goal.x;
      paramt.TurnToPointP.y = goal.y;
      turnToPoint(paramt);
      return;
    }
    //printf("should kick now %f\n", turnAngleLeft);
    comm.sendCommand(botID, 0, 0, 0, 7, false);
  }
}
