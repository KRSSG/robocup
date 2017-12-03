#include "skillSet.h"
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>
#include <ssl_common/config.h>
#include <ssl_common/grSimComm.h>

using namespace std;

namespace Strategy
{
  gr_Robot_Command SkillSet::goalKeeping(const SParam& param, const BeliefState &state, int botID)
  {
    SParam pCopy = param;
    static int framecount = 1;
    static Vector2D<int> ballInitialpos;
    gr_Robot_Command comm;
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
    Vector2D<int> ballFinalpos, botDestination, point, nextWP, nextNWP;;
//    if(framecount == 1)
//    {
//      ballInitialpos = ballPos;
//      framecount++;
//      return;
//    }
//    else if((framecount % 5) == 0)
//    {
//      framecount = 1;
//      ballFinalpos = ballPos;
//    }
//    else
//    {
//      framecount++;
//      return;
//    }

    //if bot moves parallel to x axis (y is constant)
    botDestination.y = state.homePos[botID].y;
    botDestination.x = ((botDestination.y - ballFinalpos.y) * (ballFinalpos.x - ballInitialpos.x) / (ballFinalpos.y - ballInitialpos.y)) + ballFinalpos.x;

    //if bot moves parallel to y axis (x is constant)
    /*botDestination.x = state.homePos[botID].x;
    botDestination.y = (ballFinalpos.y - ballInitialpos.y)/(ballFinalpos.x - ballInitialpos.x)*(botDestination.x - ballFinalpos.x) + ballFinalpos.y;*/

    pCopy.GoalKeepingP.x = botDestination.x;
    pCopy.GoalKeepingP.y = botDestination.y;
    pCopy.GoalKeepingP.finalslope = 0;
//    goToPointFast(pCopy);

    float v_x, v_y, v_t;

    point.x = pCopy.GoalKeepingP.x;
    point.y = pCopy.GoalKeepingP.y;

    using Navigation::obstacle;
    vector<obstacle> obs;
    obstacle o;
    for (int i = 0; i < state.homeDetected.size(); ++i)
    {
      o.x = state.homePos[i].x;
      o.y = state.homePos[i].y;
      o.radius = 2 * BOT_RADIUS;
      obs.push_back(o);
    }

    for (int i = state.homeDetected.size(); i < state.homeDetected.size() + state.awayDetected.size(); ++i)
    {
      o.x = state.awayPos[i - state.homeDetected.size()].x;
      o.y = state.awayPos[i - state.homeDetected.size()].y;
      o.radius = 2 * BOT_RADIUS;
      obs.push_back(o);
    }

    // use the mergescurver planner
    Navigation::MergeSCurve pathPlanner;
    pathPlanner.plan(botPos,
                      point,
                      &nextWP,
                      &nextNWP,
                      obs,
                      obs.size(),
                      botID,
                      true);

    float angle = Vector2D<int>::angle(nextWP, botPos);
    float dist = Vector2D<int>::dist(point, botPos);
    float theta = (state.homePos[botID].theta - angle);
    float rot_theta = (state.homePos[botID].theta - pCopy.GoalKeepingP.finalslope) * (180 / PI);
    if(rot_theta > 0)
    {
      if(rot_theta < DRIBBLER_BALL_ANGLE_RANGE)
      {
        v_t = 0;
      }
      if(rot_theta < 45)
      {
        v_t = -rot_theta / 10;
      }
      else
      {
        v_t = -4.5f;
      }
    }
    float profileFactor = MAX_BOT_SPEED;
    if(profileFactor < MIN_BOT_SPEED)
      profileFactor = MIN_BOT_SPEED;
    v_x = profileFactor * sin(theta);
    v_y = profileFactor * cos(theta);

    if(dist < BOT_BALL_THRESH)
    {
      comm = getRobotCommandMessage(botID, 0, 0, 0, 0, false);
      return comm;
    }
    else
    {
      comm = getRobotCommandMessage(botID, v_x, v_y, v_t, 0, false);
      return comm;
    }
  }
}
