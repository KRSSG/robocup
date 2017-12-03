#include <list>
#include <ros/ros.h>
#include "tDefendARC.hpp"
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

	TDefendARC::TDefendARC(int botID) : Tactic(botID) {
		iState = POSITION;
	}

	TDefendARC::~TDefendARC() {}

	bool TDefendARC::isCompleted(const BeliefState& state,const Tactic::Param& tParam) const {
		return iState == KICK;
	}//isCompleted

	inline bool TDefendARC::isActiveTactic(void) const {
		return iState != KICK;
	}//isActiveTactic

	int TDefendARC::chooseBestBot(const BeliefState& state, std::list<int>& freebots, const Param& tParam, int prevID) const {
		Vector2D<float> p_threat;
		Vector2D<float> upper_limit(-HALF_FIELD_MAXX, OUR_GOAL_MAXY / 1.30f);
		Vector2D<float> lower_limit(upper_limit.x, -1 * upper_limit.y);
		Vector2D<float> sol_right;
		Vector2D<float> sol_left;
        Vector2D<float> C(-HALF_FIELD_MAXX, 0.0f);
        float R = HALF_FIELD_MAXY / 2.5f;

		p_threat.x = state.ballPos.x;
		p_threat.y = state.ballPos.y;
		inter_circle_and_line(p_threat, upper_limit, C, R, sol_left);
        inter_circle_and_line(p_threat, lower_limit, C, R, sol_right);

		Vector2D<float> P1(sol_left.x, sol_left.y);
		Vector2D<float> P2(sol_right.x, sol_right.y);
		Vector2D<float> dPoint((P1.x + P2.x)/ 2.0f, (P1.y + P2.y) / 2.0f);
		int best_bot = -1;
		float min_dis = 999999.9f;

		//iterate over all the bots and see the one closest to the point
		for(std::list<int>::const_iterator itr = freebots.begin(); itr != freebots.end(); ++itr) {
			Vector2D<float> bot(state.homePos[*itr].x, state.homePos[*itr].y);
			if(Vector2D<float>::dist(dPoint, bot) < min_dis){
				best_bot = *itr;
				min_dis = Vector2D<float>::dist(dPoint, bot);
			}
		}

		assert(best_bot != -1);
		return best_bot;
	}//choosebestbot

	gr_Robot_Command TDefendARC::execute(const BeliefState& state, const Param& tParam) {
		Vector2D<float> P1;
		Vector2D<float> P2;
		Vector2D<float> C(-HALF_FIELD_MAXX, 0.0f);
		//Vector2D<float> homePos(state.homePos[botID].x, state.homePos[botID].y);
		//Vector2D<float> mid_arc((P1.x + P2.x)/2.0, (P1.y + P2.y)/2.0);
		//Vector2D<float> origin(0.0f, 0.0f);
		Vector2D<float> dPoint;
		Vector2D<float> tP1;
		Vector2D<float> tP2;
		//Vector2D<float> t_homePos;
		Vector2D<float> p_threat;
		Vector2D<float> sol_right;
		Vector2D<float> sol_mid;
		Vector2D<float> sol_left;
		Vector2D<float> upper_limit(-HALF_FIELD_MAXX, OUR_GOAL_MAXY / 1.30f);
		Vector2D<float> lower_limit(upper_limit.x, -1 * upper_limit.y);
		Vector2D<float> ballPos(state.ballPos.x, state.ballPos.y);
		Vector2D<float> p_def(state.homePos[botID].x, state.homePos[botID].y);
		Vector2D<float> mid_arc;
		p_threat.x = state.ballPos.x;
		p_threat.y = state.ballPos.y;
		float R = HALF_FIELD_MAXY / 2.5f;

		inter_circle_and_line(p_threat, upper_limit, C, R, sol_left);
        inter_circle_and_line(p_threat, lower_limit, C, R, sol_right);
		inter_circle_and_line(p_threat, C, C, R, sol_mid);

		//mid_arc.x = (sol_left.x + sol_right.x ) / 2.0f;
		//mid_arc.y = (sol_left.y + sol_right.y ) / 2.0f;

		//decide the internal state of the bot
		if(Vector2D<float>::dist(ballPos, mid_arc) >= DRIBBLER_BALL_THRESH * 1.3){
			iState = POSITION;
		}
		else {
			iState = KICK;
		}
		
		//float R = HALF_FIELD_MAXY / 2.5f;
		Strategy::SkillSet::SkillID sID;
		SkillSet::SParam sParam;

		switch(iState) {
			case POSITION:
			{
				sID = SkillSet::GoToPoint;
		        //transform the co-ordinates into the center's reference frame 
		        //tP1.x = P1.x - C.x;
		        //tP1.y = P1.y - C.y;
		        //tP2.x = P2.x - C.x;
		        //tP2.y = P2.y - C.y;

		    /*    //calculate the open angle from the primary  threat
		        int pt = primary_threat(state);
		        pt = -1;
		        if(pt == -1){
			       p_threat.x = state.ballPos.x;
			       p_threat.y = state.ballPos.y;
		        }
		        else {
			       p_threat.x = state.awayPos[pt].x;
		           p_threat.y = state.awayPos[pt].y;
		        }

		     */

		        //calculate the points of intersection of line and the circle
		       // inter_circle_and_line(p_threat, upper_limit, C, R, sol_left);
        		//inter_circle_and_line(p_threat, lower_limit, C, R, sol_right);
		        //inter_circle_and_line(p _threat, C, C, R, sol_mid);

		        //go to the point and anticipate for the ball
		        //just for testing purpose side = 0 //represents the left side
		        if(tParam.DefendARCP.side == 0) {
			        sParam.GoToPointP.x = (sol_left.x + sol_mid.x) / 2.0f;
		            sParam.GoToPointP.y = (sol_left.y + sol_mid.y) / 2.0f;
		        }
		        else if(tParam.DefendARCP.side == 1){
			        sParam.GoToPointP.x = (sol_right.x + sol_mid.x) / 2.0f;
		            sParam.GoToPointP.y = (sol_right.y + sol_mid.y) / 2.0f;
		        }
		        else {
		        	sParam.GoToPointP.x = sol_mid.x;
		        	sParam.GoToPointP.y = sol_mid.y;
		        }

		        //decide to whether kick or dribble or pass the ball
		        sParam.GoToPointP.finalslope = atan2((p_threat.y - state.homePos[botID].y), (p_threat.x - state.homePos[botID].x));
		        sParam.GoToPointP.align = true;
		        sParam.GoToPointP.finalVelocity = 0.0f;
		        //ROS_INFO("atan2 = %f", atan2((p_threat.y - state.homePos[botID].y), (p_threat.x - state.homePos[botID].x)));
		        break;
			}
			case GO_TO_BALL:
			{
				sID = SkillSet::GoToBall;
				break;
			}
			case KICK:
			{
				if(Vector2D<float>::dist(mid_arc, ballPos) >= DRIBBLER_BALL_THRESH )
					sID = SkillSet::GoToBall;
				else{
				    sID = SkillSet::Kick;
				    sParam.KickP.power = 7.0f;	
				}
				break;
			}
		}//switch case statement
        return SkillSet::instance()->executeSkill(sID, sParam, state, botID);

	} //execute

	string TDefendARC::paramToJSON(Tactic::Param tParam) {
		using namespace rapidjson;
		StringBuffer buffer;
		Writer<StringBuffer> w(buffer);

		w.StartObject();
		w.String("x1");
		w.Double(tParam.DefendARCP.x1);
		w.String("x2");
		w.Double(tParam.DefendARCP.x2);
		w.String("y1");
		w.Double(tParam.DefendARCP.y1);
		w.String("y2");
		w.Double(tParam.DefendARCP.y2);
		w.String("xc");
		w.Double(tParam.DefendARCP.xc);
		w.String("yc");
		w.Double(tParam.DefendARCP.yc);
		w.String("side");
		w.Double(tParam.DefendARCP.side);
		w.EndObject();

		return buffer.GetString();

	}//paramToJSON

	Tactic::Param TDefendARC::paramFromJSON(string json) {
		using namespace rapidjson;
		Tactic::Param tParam;
		Document d;

		d.Parse(json.c_str());
		tParam.DefendARCP.x1 = d["x1"].GetDouble();
		tParam.DefendARCP.x2 = d["x2"].GetDouble();
		tParam.DefendARCP.y1 = d["y1"].GetDouble();
		tParam.DefendARCP.y2 = d["y2"].GetDouble();
		tParam.DefendARCP.xc = d["xc"].GetDouble();
		tParam.DefendARCP.yc = d["yc"].GetDouble();
		tParam.DefendARCP.side = d["side"].GetDouble();

		return tParam;
	}//paramFromJSON

	int TDefendARC::primary_threat(const BeliefState& state) const {
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

	void TDefendARC::inter_circle_and_line(Vector2D<float> P1, Vector2D<float> P2, Vector2D<float> C, float R, Vector2D<float>& P) const {
		//here P1 always represents the threat position
		float tx1, tx2, ty1, ty2;
		float dx, dy, dr, D;

		//shift the origin to the center of the circle
		tx1 = P1.x - C.x;
		tx2 = P2.x - C.x;
		ty1 = P1.y - C.y;
		ty2 = P2.y - C.y;

		//calculate the intersection of the line and circle of radius R
		//http://mathworld.wolfram.com/Circle-LineIntersection.html
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

	bool TDefendARC::ball_velocity_direction(const BeliefState& state) const {
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

}//namespace strategy