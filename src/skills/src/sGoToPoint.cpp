
#include <fstream>
#include "../../skills/include/skills/skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include "../../navigation/include/navigation/planners.h"
#include "../../navigation/include/navigation/controllers/waypoint.h"
#include <cstdio>
#include <vector>
#include "../../navigation/include/navigation/controllers/waypoint.h"
#include <ssl_common/config.h>
#include <ssl_common/grSimComm.h>
#include "ros/ros.h"
#include <std_msgs/Int64MultiArray.h>
#include <fstream>

#define POINTPREDICTIONFACTOR 2

using namespace std;

namespace Strategy
{
  gr_Robot_Command SkillSet::goToPoint(const SParam &param, const BeliefState &state, int botID)
  {
    using Navigation::obstacle;
    #if 1
      vector<obstacle> obs;
      for(int i = 0; i < state.homeDetected.size(); ++i) {
        if (state.homeDetected[i] && i != botID) {
          obstacle o;
          o.x = state.homePos[i].x;
          o.y = state.homePos[i].y;
          o.radius = 3.3 * BOT_RADIUS;
          obs.push_back(o);

          // cout<<"i: "<<i<<"\tbotID: "<<botID<<endl;
        }
      }

      for(int i = 0; i < state.awayDetected.size(); ++i) {
        if (state.awayDetected[i]) {
          // cout<<"i ========================= "<<i<<endl;
          obstacle o;
          o.x = state.awayPos[i].x;
          o.y = state.awayPos[i].y;
          o.radius = 3.3 * BOT_RADIUS;
          // o.radius = 3.5 * BOT_RADIUS;
         obs.push_back(o);
        }
      }
      
      // obstacle boundary;
      // boundary.x = -HALF_FIELD_MAXX;
      // boundary.y = -HALF_FIELD_MAXY;
      // obs.push_back(boundary);
      // boundary.x = HALF_FIELD_MAXX;
      // boundary.y = HALF_FIELD_MAXY;
      // obs.push_back(boundary);
      // boundary.x = HALF_FIELD_MAXX;
      // boundary.y = -HALF_FIELD_MAXY;
      // obs.push_back(boundary);
      // boundary.x = -HALF_FIELD_MAXX;
      // boundary.y = HALF_FIELD_MAXY;
      // obs.push_back(boundary);


      Vector2D<int> pointPos;
      pointPos.x = param.GoToPointP.x;
      pointPos.y = param.GoToPointP.y;
      Vector2D<int> point, nextWP, nextNWP;
      Navigation::MergeSCurve pathPlanner;
      Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);

      pathPlanner.plan(botPos,
                        pointPos,
                        &nextWP,
                        &nextNWP,
                        obs,
                        obs.size(),
                        botID,
                        true);
      

      // cout<<"frame number: "<<state.frame_number<<endl;
      // cout<<"botPos[botID].x: "<<state.homePos[botID].x<<"\tbotPos[botID].y: "<<state.homePos[botID].y<<endl;
      // cout<<"finalPos.x : "<<param.GoToPointP.x<<"\tfinalPos.y: "<<param.GoToPointP.y<<endl;
      // cout<<"pointpos.x : "<<pointPos.x<<"\tPointpos.y: "<<pointPos.y<<endl;
      // cout<<"nextWP.x : "<<nextWP.x<<"\tnextWP.y: "<<nextWP.y<<endl;



     /* fstream file;
      file.open("/home/kgpkubs/Desktop/debugger/debugger.txt", ios::app | ios::out);
      file<<"nextWP.x:\t"<<nextWP.x<<"\tnextWP.y:\t"<<nextWP.y<<"\n";
      file<<"nextNWP.x:\t"<<nextNWP.x<<"\tnextNWP.y:\t"<<nextNWP.y<<"\n";
      file.close();*/

      // rosnode to publish waypoints of trajectory
      // int num = 1;
      // char ** v ; 
      // ros::init(num, v,"debug_node");

      static ros::NodeHandle n2;
      static ros::Publisher debug_lines = n2.advertise<std_msgs::Int64MultiArray>("debugger", 1000);
      std_msgs::Int64MultiArray arr;
      arr.data.push_back(nextWP.x);
      arr.data.push_back(nextWP.y);
      arr.data.push_back(nextNWP.x);
      arr.data.push_back(nextNWP.y);
      // ROS_INFO("\n\n %d %d HERE.....\n", nextWP.x,nextWP.y);
      debug_lines.publish(arr);


    #else
    
    #endif

    #if 1
    
    float dist = Vector2D<int>::dist(botPos, pointPos);
    float maxDisToTurn = dist - 5.0f * BOT_BALL_THRESH / 4;
    float angleToTurn = normalizeAngle(param.GoToPointP.finalslope - (state.homePos[botID].theta)); 
    
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
    Vector2D<int> ballPos(state.ballPos.x, state.ballPos.y); 
      // // cout<< "dist: " << dist << " max dist to turn: " << maxDisToTurn << endl;

    // cout<<"--------------------------------------------------------------------------------- \n \n dist =========== "<<Vector2D<int>::dist(botPos,ballPos)<<"  -------- "<<20*BOT_RADIUS<<" ------------  "<<angleToTurn << "----- \n \n"<<endl;
    
    bool dribbler = Vector2D<int>::dist(botPos,ballPos)<5*BOT_RADIUS && angleToTurn< 0.6 ? true:false;
    if(param.GoToPointP.align == false) {
      // cout<<"in GoToPointP 1_1"<<endl;
      if (dist < DRIBBLER_BALL_THRESH) {
        if(dist < 2*BOT_BALL_THRESH) { 
          // cout << "\n\n  Case 1 \n\n" << dribbler << "\n\n\n\n\n";         
          return getRobotCommandMessage(botID, 0, 0, 0, 0, dribbler);
        }
        else {

          // cout<< "speed_x: " << speed * sin(-theta) << " speed_y: " << speed * cos(-theta);
          // cout << "\n\n Case 2\n\n" << dribbler << "\n\n\n\n\n";
          return getRobotCommandMessage(botID, speed * sin(-theta), speed * cos(-theta), omega, 0, dribbler);
        }
      }
      else {
      // cout<<"in GoToPointP 1_2"<<endl;
        // cout<< "speed_x: " << speed * sin(-theta) << " speed_y: " << speed * cos(-theta);
        return getRobotCommandMessage(botID, speed * sin(-theta), speed * cos(-theta), omega, 0, dribbler);
      }
      // cout<<"in GoToPointP 1_3"<<endl;
    }
    else {
      // cout<<"in GoToPointP 2_1"<<endl;
      if (dist > BOT_BALL_THRESH / 4) {
      // cout<<"in GoToPointP 2_2"<<endl;
      // cout<< "speed_x: " << speed * sin(-theta) << " speed_y: " << speed * cos(-theta);
        return getRobotCommandMessage(botID, speed * sin(-theta), speed * cos(-theta), 0, 0, dribbler);
      }
      else {
        // cout<<"in GoToPointP 2_3"<<endl;
        // cout << "\n\n\n\n" << dribbler << "\n\n\n\n\n";
        return getRobotCommandMessage(botID, 0, 0, 0, 0, dribbler);
      }

    }
    
       
  #else
  
  #endif
  }
}
