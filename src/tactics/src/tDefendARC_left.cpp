#include <list>
#include <ros/ros.h>
#include "tDefendARC_left.hpp"
#include "skills/skillSet.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <ssl_common/geometry.hpp>
#include <skills/skillSet.h>

#define sgn(x) (((x)<0)?(-1):(1))
#define TEAMSIZE (6)

		/*
		#####################
		FURTHER IMPROVEMENTS:
		#####################

		#1 in the kicking internal state we can direct the ball towards our attacker
		#2 write a better heuristic for the determination of primary defender
		#3 before kicking we can dribble the ball to a better point for kicking / passing 
		   to our attacker
		*/

namespace Strategy {

	TDefendARC_left::TDefendARC_left(int botID) : Tactic(botID) {
		iState = POSITION;
	}

	TDefendARC_left::~TDefendARC_left() {}

	bool TDefendARC_left::isCompleted(const BeliefState& state,const Tactic::Param& tParam) const {
		return false;
	}//isCompleted

	inline bool TDefendARC_left::isActiveTactic(void) const {
		return iState != KICK;
	}//isActiveTactic

	int TDefendARC_left::chooseBestBot(const BeliefState& state, std::list<int>& freebots, const Param& tParam, int prevID) const {
		Vector2D<float> p_threat;
		// Vector2D<float> upper_limit(-HALF_FIELD_MAXX, 0);
		Vector2D<float> upper_limit(-HALF_FIELD_MAXX, OUR_GOAL_MAXY *(2.0/ 3.0f));
		Vector2D<float> lower_limit(upper_limit.x, -1 * upper_limit.y);
		Vector2D<float> sol_right;
		Vector2D<float> sol_left;
        Vector2D<float> C(-HALF_FIELD_MAXX, 0.0f);
        float R = HALF_FIELD_MAXY / 2.5f;

		p_threat.x = state.ballPos.x;
		p_threat.y = state.ballPos.y;
		inter_circle_and_line(p_threat, upper_limit, C, R, sol_left);
		Vector2D<float> P1(sol_left.x, sol_left.y);
		int best_bot = -1;
		float min_dis = 999999.9f;

		//iterate over all the bots and see the one closest to the point
		for(std::list<int>::const_iterator itr = freebots.begin(); itr != freebots.end(); ++itr) {
			Vector2D<float> bot(state.homePos[*itr].x, state.homePos[*itr].y);
			if(Vector2D<float>::dist(P1, bot) < min_dis){
				best_bot = *itr;
				min_dis = Vector2D<float>::dist(P1, bot);
			}
		}

		assert(best_bot != -1);
		return best_bot;
	}//choosebestbot

	gr_Robot_Command TDefendARC_left::execute(const BeliefState& state, const Param& tParam) {
		Vector2D<float> P1;
		Vector2D<float> P2;
		Vector2D<float> C(-HALF_FIELD_MAXX, 0.0f);
		Vector2D<float> botPos(state.homePos[botID].x, state.homePos[botID].y);
		Vector2D<float> dPoint;
		float min_dis = 999999.9f;
		int sel_bID=-1;

		Vector2D<float> p_threat;
		Vector2D<float> sol_mid;
		Vector2D<float> ballPos(state.ballPos.x, state.ballPos.y);
		Vector2D<float> p_def(state.homePos[botID].x, state.homePos[botID].y);
		Vector2D<float> other_bot;
		p_threat.x = state.ballPos.x;
		p_threat.y = state.ballPos.y;
		float R = HALF_FIELD_MAXY / 2.3f;
		float delta_x = 0 * BOT_RADIUS;
		float delta_y = 2.7 * BOT_RADIUS;

		inter_circle_and_line(p_threat, C, C, R, sol_mid);

		//decide the internal state of the bot
		if(Vector2D<float>::dist(ballPos, botPos) >= DRIBBLER_BALL_THRESH * 5){
			iState = POSITION;
		}
		else if(Vector2D<float>::dist(ballPos, botPos) <= DRIBBLER_BALL_THRESH * 5
			   && (Vector2D<float>::dist(ballPos, botPos) > DRIBBLER_BALL_THRESH * 1.3)){
			/*
			  iterate over all the bots and 
			  select the primary defender which is closest to the ball 
			*/	
			for(int bID = 0; bID < TEAMSIZE; bID++){
				other_bot.x = state.homePos[bID].x; 
				other_bot.y = state.homePos[bID].y;

				float distance = Vector2D<float>::dist(other_bot, ballPos);
				if(distance < min_dis){
					min_dis = distance;
					sel_bID = bID;
				}
			}
			if(sel_bID == botID)
				iState = GO_TO_BALL;
			else
				iState = POSITION;
		}
		else {
			if (Vector2D<float>::dist(botPos, ballPos) < DRIBBLER_BALL_THRESH)
				iState = KICK;
			else
				iState = GO_TO_BALL;
		}

		//float R = HALF_FIELD_MAXY / 2.5f;
		Strategy::SkillSet::SkillID sID;
		SkillSet::SParam sParam;

		switch(iState) {
			case POSITION:
			{
				sID = SkillSet::GoToPoint;

		        sParam.GoToPointP.x = sol_mid.x - delta_x;
	            sParam.GoToPointP.y = sol_mid.y - delta_y;				
		        sParam.GoToPointP.align = true;
		        sParam.GoToPointP.finalVelocity = 0.0f;
		        sParam.GoToPointP.finalslope = atan2( (state.ballPos.y - state.homePos[botID].y) , (state.ballPos.x - state.homePos[botID].x));
		        break;
			}
			case GO_TO_BALL:
			{
				sID = SkillSet::GoToBall;
			}
			case KICK:
			{
			    sID = SkillSet::Kick;
			    sParam.KickP.power = 7.0f;	
			}
		}//switch case statement
		cout << "left: " << sParam.GoToPointP.x << "\t" <<  sParam.GoToPointP.y;
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);

	} //execute

    Tactic::Param TDefendARC_left::paramFromJSON(string json) {
      using namespace rapidjson; 
      Tactic::Param tParam;
      return tParam;
    }

    string TDefendARC_left::paramToJSON(Tactic::Param tParam) {
      return string("");
    }

	int TDefendARC_left::primary_threat(const BeliefState& state) const {
		std::vector<int> away_in_our_side;
		float thresh = 100000.0f;
		int threat = -1;

		if(sqrt(pow(state.ballVel.x, 2) + pow(state.ballVel.y, 2)) <= LOW_BALL_VELOCITY_THRES){
			return -1;
		}
		else {
			if(!ball_velocity_direction(state)) {

				//calculate the botID of the opponent bots which are on our goalie side
				for(int i = 0; i < state.awayDetected.size(); ++i) {
					if(state.awayPos[i].x < 0) {
						away_in_our_side.push_back(i);
					}
				}

				//for each of the detected bots on our goalie side apply the heuristic
				for(std::vector<int>::iterator itr = away_in_our_side.begin();itr != away_in_our_side.end(); ++itr){
					if(state.awayPos[*itr].x < thresh){
						threat = *itr;
						thresh = state.awayPos[*itr].x;
					}
				}
				return threat;
			}
			else {
				return -1;
			}
		}
	}//primary_threat

	void TDefendARC_left::inter_circle_and_line(Vector2D<float> P1, Vector2D<float> P2, Vector2D<float> C, float R, Vector2D<float>& P) const {
		//here P1 always represents the threat position
		float tx1, tx2, ty1, ty2;
		float dx, dy, dr, D;

		//shift the origin to the center of the circle
		tx1 = P1.x - C.x;
		tx2 = P2.x - C.x;
		ty1 = P1.y - C.y;
		ty2 = P2.y - C.y;
		
		/*
			calculate the intersection of the line and circle of radius R
			http://mathworld.wolfram.com/Circle-LineIntersection.html
		*/

		dx  = tx2 - tx1;
		dy  = ty2 - ty1;
		dr  = sqrt(pow(dx, 2) + pow(dy, 2));
		 D  = tx1 * ty2 - tx2 * ty1;
		P.x = float((D * dy + sgn(dy) * dx * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2)));
		P.y = float(((-D * dx) + abs(dy) * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2)));

		if(P.x < 0) {
			    P.x = float((D * dy - sgn(dy) * dx * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2)));
		}

		if(P1.y > 0) {
		    if(P.y < 0) {
			    P.y = float((-D * dx - abs(dy) * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2)));
		    }
		}
		else {
		    if(P.y > 0) {
			    P.y = float((-D * dx - abs(dy) * sqrt((pow(R, 2) * pow(dr, 2)) - pow(D,2)))/(pow(dr, 2)));
		    }
		}

		//convert the co-ordinates back to the original system
		P.x = P.x + C.x;
		P.y = P.y + C.y;
	}//inter_circle_and_line function

	bool TDefendARC_left::ball_velocity_direction(const BeliefState& state) const {
		Vector2D<float> ball_vel(state.ballVel.x, state.ballVel.y);
		Vector2D<float> ball_pos(state.ballPos.x, state.ballPos.y);
		Vector2D<float> Upper_limit(-HALF_FIELD_MAXX, OUR_GOAL_MAXY / 1.2f + BOT_BALL_THRESH);
		Vector2D<float> Lower_limit(-HALF_FIELD_MAXX, OUR_GOAL_MINY / 1.2f - BOT_BALL_THRESH);
		float alpha_1;
		float alpha_2;
		float ball_vel_angle;

		alpha_1 = atan2((Upper_limit.y - ball_pos.y), (ball_pos.x - Upper_limit.x));
		alpha_2 = atan2((Lower_limit.y - ball_pos.y), (ball_pos.x - Lower_limit.x));

		
		if(ball_vel.x < 0) {
			ball_vel_angle = atan2(state.ballVel.y, -1 * state.ballVel.x);
		}
		else {
			return false;
		}

		//deciding the direction of the ball's velocity
		if(ball_vel.y > 0){
			if(ball_vel_angle > alpha_2 && ball_vel_angle < alpha_1){
				return true;
			}
			else {
				return false;
			}
		}
		else {
			if(abs(ball_vel_angle) > abs(alpha_1) && abs(ball_vel_angle) < abs(alpha_2)) {
				return true;
			}
			else {
				return false;
			}
		}

	}//ball_velocity_direction function
/*
	void TDefendARC::secondary_threat(const BeliefState& state, std::vector<int> &vec, int pri_threat) const {

		/*
		CRITERIA FOR THE DETERMINATION OF SECONDARY THREAT

		#1 opponents closer to our goalie side 
		#2 intercepting goal > intercepting the opp
		#3 opponents having larger open angle on our goal
		#4 this doesn't include the bot which is likely to recieve the ball
		*/
/*		std::vector<int> away_in_our_side;

		for(int away_botID = 0; away_botID < state.awayDetected.size(); ++away_botID) {
			if(state.awayPos[away_botID].x < 0 && away_botID != pri_threat) {
				away_in_our_side.push_back(away_botID);
			}
		}

		//rank these opponents on how close they are to the goal
		//first element in the vector is the closest one 
		int temp;
		for(std::vector<int>::iterator i_itr = away_in_our_side.begin(); i_itr != away_in_our_side.end(); ++i_itr) {
			for(std::vector<int>::iterator j_itr = i_itr + 1; j_itr != away_in_our_side.end(); ++j_itr) {
				if(state.awayPos[*i_itr].x > state.awayPos[*j_itr].x){
					temp = *i_itr;
					*i_itr = *j_itr;
					*j_itr = temp;
				}
			}
		}

		//calculate the open angle of each of these bots to our goal
	}

*/

} //namespace strategy