#include "skillSet.h"
// #include "logger.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>

// using namespace Util;
//addCircle

using namespace std;

namespace Strategy
{
  gr_Robot_Command SkillSet::dribbleToPoint(const SParam &param, const BeliefState &state, int botID)
  {
    float v_x, v_y;
    Vector2D<int> point(param.DribbleToPointP.x, param.DribbleToPointP.y);
    Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);
    float ballSlope = Vector2D<int>::angle(ballPos, botPos);
    float ballDist  = Vector2D<int>::dist(ballPos, botPos);
    float ballTheta = fabs(normalizeAngle(state.homePos[botID].theta - ballSlope));
//    if(ballDist > BOT_BALL_THRESH || ballTheta > DRIBBLER_BALL_ANGLE_RANGE)
//    {
//      return goToBall(param, state, botID);
//    }
    using Navigation::obstacle;
      vector<obstacle> obs;
      obstacle o;
      for (int i = 0; i < state.homeDetected.size(); ++i)
      {
        o.x = state.homePos[i].x;
        o.y = state.homePos[i].y;
        o.radius = 3 * BOT_RADIUS;
        obs.push_back(o);
      }

    for (int i = state.homeDetected.size(); i < state.homeDetected.size() + state.awayDetected.size(); ++i)
      {
        o.x = state.awayPos[i - state.homeDetected.size()].x;
        o.y = state.awayPos[i - state.homeDetected.size()].y;
        o.radius = 3 * BOT_RADIUS;
        obs.push_back(o);
      }
    Vector2D<int> nextWP, nextNWP;

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

    if (nextWP.valid())
    {
      // comm.addCircle(nextWP.x, nextWP.y, 50);
      // comm.addLine(state.homePos[botID].x, state.homePos[botID].y, nextWP.x, nextWP.y);
    }
    if (nextNWP.valid())
    {
      // comm.addCircle(nextNWP.x, nextNWP.y, 50);
      // comm.addLine(nextWP.x, nextWP.y, nextNWP.x, nextNWP.y);
    }
  
//    Logger::toStdOut("Dribbling\n");
    float dist = Vector2D<int>::dist(nextWP, botPos);
//    float angle = Vector2D<int>::angle(nextWP, botPos);
    float angle = param.DribbleToPointP.finalslope;
    float theta = normalizeAngle(state.homePos[botID].theta - angle);
    float profileFactor = 2 * dist * MAX_BOT_SPEED / MAX_FIELD_DIST;
    if(fabs(profileFactor) < MIN_BOT_SPEED )
    {
      if(profileFactor < 0 )
        profileFactor = - MIN_BOT_SPEED;
      else
        profileFactor = MIN_BOT_SPEED;
    }
    v_y = profileFactor * cos(theta);
    v_x = profileFactor * sin(theta);
    float romega = theta / ( 2 * PI ) * MAX_BOT_OMEGA;
    if(v_y < - MAX_BACK_DRIBBLE_V_Y)
    {
      v_x /= (v_y / -MAX_BACK_DRIBBLE_V_Y);
      v_y = -MAX_BACK_DRIBBLE_V_Y;
    }
  else if(v_y > MAX_FRONT_DRIBBLE_V_Y)
  {
    v_x /= (v_y / MAX_FRONT_DRIBBLE_V_Y);
    v_y = MAX_FRONT_DRIBBLE_V_Y;
  }
  
    if(v_x > MAX_DRIBBLE_V_X)
    {
      // Clamp right velocity
      v_y /= (v_x / MAX_DRIBBLE_V_X);
      v_x = MAX_DRIBBLE_V_X;
    }
  else if(v_x < -MAX_DRIBBLE_V_X)
  {
    v_y /= (v_x / -MAX_DRIBBLE_V_X);
    v_x = -MAX_DRIBBLE_V_X;
  }
  
  if(fabs(theta)> SATISFIABLE_THETA)
  { 
    if(fabs(romega < MIN_BOT_OMEGA))
    {
      if(romega > 0)
        romega = MIN_BOT_OMEGA;
      else
        romega = -MIN_BOT_OMEGA;
    }
    else if(fabs(romega) > MAX_DRIBBLE_R)
    {
      if(romega > 0)
        romega = MAX_DRIBBLE_R;
      else
        romega = -MAX_DRIBBLE_R;
    }
  }else
  {
    romega = 0;
  }
    if(dist < BOT_BALL_THRESH )
    {
        return getRobotCommandMessage(botID, 0, 0, 0, 0, true);
    }
    else
    {
        return getRobotCommandMessage(botID, v_x, v_y, -romega, 0, true);
    }
  } // dribbleToPoint
}
