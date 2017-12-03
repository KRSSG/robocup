#include "skillSet.h"
#include <ssl_common/grSimComm.h>
#include <ssl_common/config.h>
#include <navigation/planners.h>
#include <navigation/controllers/waypoint.h>

#define DEFEND_RADIUS 50.0
using namespace std;
//addCircle..
namespace Strategy
{
	gr_Robot_Command SkillSet::defendPoint(const SParam &param, const BeliefState &state, int botID)
	{
		Vector2D<int> botPos(state.homePos[botID].x, state.homePos[botID].y);
		float point_ball_angle = atan2(state.ballPos.y-param.DefendPointP.y,state.ballPos.x-param.DefendPointP.x);
		Vector2D<int> dpoint;
		//comm.addCircle(param.DefendPointP.x,param.DefendPointP.y,50);
		dpoint.y=param.DefendPointP.y + (param.DefendPointP.radius)*sin(point_ball_angle);

		dpoint.x=param.DefendPointP.x + (param.DefendPointP.radius)*cos(point_ball_angle);


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
		Vector2D<int> point, nextWP, nextNWP;

		// use the mergescurver planner
   		Navigation::MergeSCurve pathPlanner;
		pathPlanner.plan(botPos,
		                  dpoint,
		                  &nextWP,
		                  &nextNWP,
		                  obs,
		                  state.homeDetected.size() + state.awayDetected.size(),
		                  botID,
		                  true);

		if(nextWP.valid())
		{
		// 	comm.addCircle(nextWP.x, nextWP.y, 50);
		// 	comm.addLine(state.homePos[botID].x, state.homePos[botID].y, nextWP.x, nextWP.y);
		}
		if(nextNWP.valid())
		{
		// 	comm.addCircle(nextNWP.x, nextNWP.y, 50);
		// 	comm.addLine(nextWP.x, nextWP.y, nextNWP.x, nextNWP.y);
		}

		float motionAngle = Vector2D<int>::angle(nextWP, botPos);

		float finalSlope;   // final slope the current bot motion should aim for!
		if(nextNWP.valid())
			finalSlope = Vector2D<int>::angle(nextNWP, nextWP);
		else
			finalSlope = Vector2D<int>::angle(nextWP, botPos);

		finalSlope=point_ball_angle;
		float turnAngleLeft = normalizeAngle(finalSlope - state.homePos[botID].theta); // Angle left to turn

		float omega = turnAngleLeft * MAX_BOT_OMEGA / (2 * PI); // Speedup turn
		if(omega < MIN_BOT_OMEGA && omega > -MIN_BOT_OMEGA)
		{
			if(omega < 0) omega = -MIN_BOT_OMEGA;
			else omega = MIN_BOT_OMEGA;
		}

		float dist = Vector2D<int>::dist(nextWP, botPos);  // Distance of next waypoint from the bot
		float theta =  motionAngle - state.homePos[botID].theta;               // Angle of dest with respect to bot's frame of reference

		float profileFactor = (dist * 2 / MAX_FIELD_DIST) * MAX_BOT_SPEED;
		if(profileFactor < MIN_BOT_SPEED)
			profileFactor = MIN_BOT_SPEED;
		else if(profileFactor > MAX_BOT_SPEED)
			profileFactor = MAX_BOT_SPEED;

		if(dist < BOT_POINT_THRESH)
		{
			if((turnAngleLeft) > -DRIBBLER_BALL_ANGLE_RANGE  && (turnAngleLeft) < DRIBBLER_BALL_ANGLE_RANGE)
			{
				return getRobotCommandMessage(botID, 0, 0, 0, 0, true); //kick
			}
			else{
				return getRobotCommandMessage(botID, 0, 0, omega, 0, true);
			}
		}
		else
		{
			return getRobotCommandMessage(botID, profileFactor * sin(-theta), profileFactor * cos(-theta), omega, 0, false);
		}
	}
}
