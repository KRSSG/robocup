#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>
#include <cstdio>
#include <vector>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>

#include <ssl_common/config.h>
#include <ssl_common/grSimComm.h>

#define POINTPREDICTIONFACTOR 2

using namespace std;

namespace Strategy
{
  gr_Robot_Command SkillSet::goToBall(const SParam &param, const BeliefState &state, int botID)
  {
    using Navigation::obstacle;
    #if 1
      vector<obstacle> obs;
      for(int i = 0; i < state.homeDetected.size(); ++i) {
        if (state.homeDetected[i]) {
          obstacle o;
          o.x = state.homePos[i].x;
          o.y = state.homePos[i].y;
          o.radius = 2 * BOT_RADIUS;
          obs.push_back(o);
        }
      }

      for(int i = 0; i < state.awayDetected.size(); ++i) {
        if (state.awayDetected[i]) {
          obstacle o;
          o.x = state.awayPos[i].x;
          o.y = state.awayPos[i].y;
          o.radius = 2 * BOT_RADIUS;
          obs.push_back(o);
        }
      }
      
      Vector2D<int> ballfinalpos;
      ballfinalpos.x = state.ballPos.x + (state.ballVel.x / POINTPREDICTIONFACTOR);
      ballfinalpos.y = state.ballPos.y + (state.ballVel.y / POINTPREDICTIONFACTOR);
      Vector2D<int> point, nextWP, nextNWP;
      Navigation::MergeSCurve pathPlanner;
      Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);

      pathPlanner.plan(botPos,
                        ballfinalpos,
                        &nextWP,
                        &nextNWP,
                        obs,
                        obs.size(),
                        botID,
                        true);
    #else
    
    #endif

    #if 1
    
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y);    
    float dist = Vector2D<int>::dist(botPos, ballPos);
    float maxDisToTurn = dist - 3.5f * BOT_BALL_THRESH;
    float angleToTurn = normalizeAngle(Vector2D<int>::angle(ballPos, botPos) - (state.homePos[botID].theta)); 
    cout<<"Angle left to turn ============= "<<Vector2D<int>::angle(ballPos, botPos) - (state.homePos[botID].theta)<<","<<angleToTurn<<endl;

    float minReachTime = maxDisToTurn / MAX_BOT_SPEED;
    float maxReachTime = maxDisToTurn / MIN_BOT_SPEED;
    
    float minTurnTime = angleToTurn / MAX_BOT_OMEGA;
    float maxTurnTime = angleToTurn / MIN_BOT_OMEGA;
    
    float speed = 0.0f;
    float omega = angleToTurn * MAX_BOT_OMEGA / (2 * PI);

    if (omega < MIN_BOT_OMEGA && omega > -MIN_BOT_OMEGA) {
      if (omega < 0) omega = -MIN_BOT_OMEGA;
      else omega = MIN_BOT_OMEGA;
    }

    if(maxDisToTurn > 0) {
      if(minTurnTime > maxReachTime) {
        speed = MIN_BOT_SPEED;
      }
      else if (minReachTime > maxTurnTime) {
        speed = MAX_BOT_SPEED;
      }
      else if(minReachTime < minTurnTime) {
        speed =  maxDisToTurn / minTurnTime;
      }
      else if (minTurnTime < minReachTime) {
        speed = MAX_BOT_SPEED;
      }
    }
    else {
      speed = dist / MAX_FIELD_DIST * MAX_BOT_SPEED;
    }

    float motionAngle = Vector2D<int>::angle(nextWP, botPos);
    float theta =  motionAngle - state.homePos[botID].theta;
    
    cout<<"dist =========== "<<dist<<"  -------- "<<2*DRIBBLER_BALL_THRESH<<" ------------  "<<2*BOT_BALL_THRESH<<endl;
    
    bool dribbler = Vector2D<int>::dist(botPos,ballPos)<5*BOT_RADIUS && angleToTurn< 0.5 ? true:false;

    if(param.GoToBallP.intercept == false) {
      if (dist < 2*DRIBBLER_BALL_THRESH) {
        cout<<"11111111111111111111"<<endl;
        if(dist < 1*BOT_BALL_THRESH) {          
        cout<<"2222222222222222222222"<<endl;

          return getRobotCommandMessage(botID, 0, 0, 0, 0, dribbler);
        }
        else {
        cout<<"3333333333333333333333"<<endl;
          return getRobotCommandMessage(botID, speed * sin(-theta), speed * cos(-theta), omega, 0, dribbler);
        }
      }
      else {
        cout<<"44444444444444444444"<<endl;
        return getRobotCommandMessage(botID, speed * sin(-theta), speed * cos(-theta), omega, 0, dribbler);
      }
    }
    else {
        cout<<"5555555555555555555555"<<endl;
      if (dist > 0.7*BOT_BALL_THRESH) {
        return getRobotCommandMessage(botID, speed * sin(-theta), speed * cos(-theta), 0, 0, dribbler);
      }
      else {
        cout<<"66666666666666666666666"<<endl;
        return getRobotCommandMessage(botID, 0, 0, 0, 0, dribbler);
      }

    }
    
       
  #else
  
  #endif
  }
}

