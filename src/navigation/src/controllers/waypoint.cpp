// way point controller
#include "controllers/waypoint.h"
#include <ssl_common/config.h>
#include <ssl_common/grSimComm.h>
namespace Navigation {

  using Strategy::getRobotCommandMessage;
  gr_Robot_Command waypointCommand(int botID, const BeliefState &state, Vector2D<int> nextWP, Vector2D<int> nextNWP, float finalSlope, bool align) {

    Vector2D<int> pos(state.homePos[botID].x, state.homePos[botID].y);
    float homeAngle = state.homePos[botID].theta;
    float motionAngle = Vector2D<int>::angle(nextWP, pos);

    if(nextNWP.valid())
      finalSlope = Vector2D<int>::angle(nextNWP, nextWP);

    float turnAngleLeft = normalizeAngle(finalSlope - homeAngle); // Angle left to turn
    //printf("Target x: %d, y:%d, a:%f || Left x:%d, y:%d, a:%f\n", nextWP.x, nextWP.y, finalSlope, 
           // nextWP.x - pos.x, nextWP.y - pos.y, turnAngleLeft);
    
    float omega = turnAngleLeft * MAX_BOT_OMEGA / (2 * PI); // Speedup turn
    if(omega < MIN_BOT_OMEGA && omega > -MIN_BOT_OMEGA)
    {
      if(omega < 0) omega = -MIN_BOT_OMEGA;
      else omega = MIN_BOT_OMEGA;
    }

    float dist = Vector2D<int>::dist(nextWP, pos);  // Distance of next waypoint from the bot
    float theta =  motionAngle - homeAngle;               // Angle of dest with respect to bot's frame of reference

    float profileFactor = (dist * 2 / MAX_FIELD_DIST) * MAX_BOT_SPEED;
    if(profileFactor < MIN_BOT_SPEED)
      profileFactor = MIN_BOT_SPEED;
    else if(profileFactor > MAX_BOT_SPEED)
      profileFactor = MAX_BOT_SPEED;
    profileFactor = MAX_BOT_SPEED;

    if(dist < BOT_POINT_THRESH)
    {
      if(align == true)
      {
        if((turnAngleLeft) > -DRIBBLER_BALL_ANGLE_RANGE  && (turnAngleLeft) < DRIBBLER_BALL_ANGLE_RANGE)
        {
          return getRobotCommandMessage(botID, 0, 0, 0, 0, true);
          // comm.sendCommand(botID, 0, 0, 0, 0, true);
        }
        else
          return getRobotCommandMessage(botID, 0, 0, omega, 0, true);
          // comm.sendCommand(botID, 0, 0, omega, 0, true);
      }
      else
        return getRobotCommandMessage(botID, 0, 0, 0, 0, true);
        // comm.sendCommand(botID, 0, 0, 0, 0, true);
    }
    else
    {
      return getRobotCommandMessage(botID, profileFactor * sin(-theta), profileFactor * cos(-theta), omega, 0, false);
      // comm.sendCommand(botID, profileFactor * sin(-theta), profileFactor * cos(-theta), omega, 0, false);
    }

  }

}